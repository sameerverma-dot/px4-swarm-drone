#!/usr/bin/env python3
"""
detector_node.py - YOLO landmine/object detector on the drone's downward camera.

Pipeline:
  Gazebo camera sensor  --(ros_gz_bridge)-->  ROS 2 sensor_msgs/Image
      --> this node runs Ultralytics YOLO --> annotated image + geotagged hits

What it does:
  * subscribes to the bridged camera image (param: image_topic)
  * runs a YOLO model (Ultralytics) on each frame
  * publishes an annotated image (param: annotated_topic) for RViz / rqt
  * for each detection, estimates a ground position (nadir projection using the
    drone's local NED position + heading) and appends new hazards to a CSV in
    ~/maps, de-duplicated by distance.

Requirements (SYSTEM python that runs ROS 2 - not the ~/yolo_test venv):
  pip install --user ultralytics        # also pulls opencv-python + numpy

GEO-LOCATION IS APPROXIMATE (Phase I): assumes a level drone and a perfect
nadir camera, flat ground at home altitude. Good enough to place hazard markers;
calibrate the pixel->ground axis mapping against a known target before trusting
absolute coordinates (see the note at project_to_ground()).
"""

import csv
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition

try:
    import cv2
except ImportError as e:  # pragma: no cover
    raise SystemExit("cv2 not found - install with: pip install --user opencv-python") from e

try:
    from ultralytics import YOLO
except ImportError as e:  # pragma: no cover
    raise SystemExit("ultralytics not found - install with: pip install --user ultralytics") from e


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.declare_parameter('image_topic', '/drone/camera')
        self.declare_parameter('annotated_topic', '/detection/image_annotated')
        self.declare_parameter('weights', 'yolov8n.pt')     # or ~/runs/.../best.pt
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('hfov_rad', 1.74)            # from mono_cam SDF
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('hazard_csv', '~/maps/hazard_points.csv')
        self.declare_parameter('min_sep_m', 2.0)            # dedup radius (metres)
        self.declare_parameter('classes', '')               # '' = all; else CSV of names to keep
        self.declare_parameter('device', '')                # '' = auto (cuda if available), else 'cuda:0'/'cpu'
        self.declare_parameter('imgsz', 640)                # inference size; smaller = much faster
        self.declare_parameter('half', True)                # fp16 on GPU (ignored on CPU)

        gp = self.get_parameter
        self.image_topic = str(gp('image_topic').value)
        self.annotated_topic = str(gp('annotated_topic').value)
        self.weights = os.path.expanduser(str(gp('weights').value))
        self.conf = float(gp('conf').value)
        self.hfov = float(gp('hfov_rad').value)
        self.publish_annotated = bool(gp('publish_annotated').value)
        self.hazard_csv = os.path.expanduser(str(gp('hazard_csv').value))
        self.min_sep = float(gp('min_sep_m').value)
        self.imgsz = int(gp('imgsz').value)
        self.half_req = bool(gp('half').value)
        keep = str(gp('classes').value).strip()
        self.keep = set(s.strip() for s in keep.split(',') if s.strip()) if keep else None

        # ---- device selection (explicit, and logged) ----
        dev = str(gp('device').value).strip()
        cuda_ok = False
        try:
            import torch
            cuda_ok = torch.cuda.is_available()
            gpu_name = torch.cuda.get_device_name(0) if cuda_ok else 'n/a'
            self.get_logger().info(
                f"torch {torch.__version__} | CUDA available: {cuda_ok} | GPU: {gpu_name}")
            if not cuda_ok:
                self.get_logger().warn(
                    "CUDA NOT available -> YOLO will run on CPU (slow). Install a CUDA "
                    "build of torch:  pip install --user --force-reinstall torch "
                    "--index-url https://download.pytorch.org/whl/cu121")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"torch probe failed: {e}")
        self.device = dev if dev else ('cuda:0' if cuda_ok else 'cpu')
        self.half = bool(self.half_req and str(self.device).startswith('cuda'))

        self.get_logger().info(f"loading YOLO weights: {self.weights}")
        self.model = YOLO(self.weights)
        try:
            self.model.to(self.device)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"model.to({self.device}) failed: {e}")
        if self.half:
            try:
                self.model.model.half()   # one-time fp16 cast, no per-frame warning
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"fp16 cast skipped: {e}")
                self.half = False
        self.get_logger().info(
            f"inference device={self.device} imgsz={self.imgsz} fp16={self.half}")

        # camera / pose state
        self.pos = VehicleLocalPosition()
        self.pos_valid = False
        self.hazards = []   # list of (north, east) already recorded

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, px4_qos)

        self.pub_annot = self.create_publisher(Image, self.annotated_topic, 1)

        os.makedirs(os.path.dirname(self.hazard_csv), exist_ok=True)
        if not os.path.exists(self.hazard_csv):
            with open(self.hazard_csv, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['t_s', 'x_ned_north', 'y_ned_east', 'class', 'conf', 'alt_m'])

        self.frame_count = 0
        self.get_logger().info(
            f"detector up: image='{self.image_topic}' -> annotated='{self.annotated_topic}', "
            f"hazards -> {self.hazard_csv}")

    # ---------- callbacks ----------
    def on_pos(self, msg):
        self.pos = msg
        self.pos_valid = bool(msg.xy_valid and msg.z_valid)

    def img_to_np(self, msg):
        """sensor_msgs/Image -> HxWx3 BGR numpy (handles rgb8/bgr8)."""
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            arr = arr.reshape((msg.height, msg.width, 3))
        except ValueError:
            return None
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]  # RGB -> BGR
        return np.ascontiguousarray(arr)

    def np_to_img(self, frame, header):
        m = Image()
        m.header = header
        m.height, m.width = frame.shape[:2]
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = frame.shape[1] * 3
        m.data = frame.tobytes()
        return m

    def project_to_ground(self, u, v, img_w, img_h):
        """
        Approx nadir projection: pixel (u,v) -> (north, east) in local NED.
        Assumes level flight, camera looking straight down, flat ground.
        NOTE: the forward/right axis mapping below is an assumption about the
        image orientation - verify against a known target and flip signs if the
        recorded points are mirrored/rotated.
        """
        if not self.pos_valid:
            return None
        h = -float(self.pos.z)          # altitude above home (m), z is down
        if h < 1.0:
            return None                 # too low / on ground
        mpp = (2.0 * h * math.tan(self.hfov / 2.0)) / img_w   # metres per pixel
        du = (u - img_w / 2.0)          # +right in image
        dv = (v - img_h / 2.0)          # +down in image
        fwd = -dv * mpp                 # image up  -> forward (assumption)
        right = du * mpp                # image right-> right   (assumption)
        yaw = float(self.pos.heading)   # rad, NED
        north = self.pos.x + fwd * math.cos(yaw) - right * math.sin(yaw)
        east = self.pos.y + fwd * math.sin(yaw) + right * math.cos(yaw)
        return north, east

    def record_hazard(self, north, east, cls, conf):
        for (hn, he) in self.hazards:
            if math.hypot(north - hn, east - he) < self.min_sep:
                return False            # already have one here
        self.hazards.append((north, east))
        with open(self.hazard_csv, 'a', newline='') as f:
            csv.writer(f).writerow(
                [f"{time.time():.1f}", f"{north:.2f}", f"{east:.2f}",
                 cls, f"{conf:.2f}", f"{-self.pos.z:.1f}"])
        self.get_logger().info(
            f"HAZARD #{len(self.hazards)}: {cls} conf={conf:.2f} at "
            f"N={north:.1f} E={east:.1f} (alt {-self.pos.z:.1f}m)")
        return True

    def on_image(self, msg):
        frame = self.img_to_np(msg)
        if frame is None:
            return
        self.frame_count += 1
        if self.frame_count % 50 == 0:
            now = time.time()
            prev = getattr(self, '_t_last', None)
            self._t_last = now
            if prev:
                self.get_logger().info(
                    f"{50.0 / max(now - prev, 1e-6):.1f} FPS through detector "
                    f"(device={self.device})")
        # NOTE: 'half' is deprecated in ultralytics >=8.4 (warns once per frame and
        # floods the log). Weights are already moved to fp16 on GPU below at load
        # time, so we simply don't pass it here.
        results = self.model(frame, conf=self.conf, imgsz=self.imgsz,
                             device=self.device, verbose=False)
        r = results[0]
        img_h, img_w = frame.shape[:2]

        for box in r.boxes:
            cls_id = int(box.cls[0])
            name = self.model.names.get(cls_id, str(cls_id)) if hasattr(self.model, 'names') else str(cls_id)
            if self.keep is not None and name not in self.keep:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            if self.publish_annotated:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            g = self.project_to_ground(u, v, img_w, img_h)
            if g is not None:
                self.record_hazard(g[0], g[1], name, conf)

        if self.publish_annotated:
            self.pub_annot.publish(self.np_to_img(frame, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
