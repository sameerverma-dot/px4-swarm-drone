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

--------------------------------------------------------------------------
THREE THINGS THAT MAKE THE MAP TRUSTWORTHY (added after the first full run)
--------------------------------------------------------------------------
1. POSE TIME-MATCHING. The old code geotagged with the *latest* pose, but a
   frame is already 0.2-0.5 s old by the time YOLO finishes with it. At the
   ~9 m/s the drone was flying, that is a 2-5 m along-track error - which is
   exactly what the first flight showed (East was accurate to 0.4 m, North was
   off by +-3 m with the sign flipping run to run: the classic signature of
   latency, not of a wrong axis). We now keep a short ring buffer of poses and
   look up the pose at (frame arrival time - pose_lag_s).

2. DETECTION GATE. survey_node publishes std_msgs/Bool on /survey/detecting,
   True only while it is actually flying survey lanes. With require_gate:=true
   the detector ignores frames during climb, RTL and landing - which is where
   all those alt=29.7 m "hazards" came from. Gated-off frames skip inference
   (the expensive part) but still republish the raw frame, so the rqt video
   feed stays live instead of looking crashed.

3. CLASS FILTER AT THE MODEL. `classes` is now pushed into YOLO itself, so
   airplane/kite/bird are never drawn and never scored. COCO weights
   hallucinate confidently on featureless nadir ground; until real landmine
   weights exist, restrict to what you actually placed in the world.

GEO-LOCATION IS STILL APPROXIMATE (Phase I): assumes a level drone, a perfect
nadir camera, and flat ground at home altitude.

--------------------------------------------------------------------------
MULTI-DRONE (SWARM): ONE detector, N cameras, ONE hazard list
--------------------------------------------------------------------------
SWARM_PLAN.md's numbers say a single shared detector has 5x headroom across
3 drones, so this node was built to serve N drones rather than spawn N
detector processes: one model load, no extra GPU contention, and correct
deduplication for free (one `self.hazards` list, not one per drone racing
to append to the same CSV).

Multi-drone mode is opt-in via the PLURAL parameters (image_topics,
pose_namespaces, gate_topics, annotated_topics, home_offsets), all
semicolon-separated, one entry per drone, same order. Leaving them empty
(the default) falls back to the original singular parameters (image_topic,
gate_topic, annotated_topic) for exactly one, unnamespaced drone - so every
existing single-drone launch command is unaffected.

pose_namespaces entries are PX4 DDS namespace prefixes, e.g. '' (instance 0,
unnamespaced /fmu/...) or '/px4_1' (instance 1). See PX4's rcS: instance N
gets namespace 'px4_N', instance 0 gets none.

home_offsets entries are 'north,east' metres to ADD to that drone's
NADIR-PROJECTED position so all drones' hazards land in ONE shared frame -
the origin of drone 0's home, per NEXT_SESSION.md's stated trap: each PX4
instance's local NED frame originates at ITS OWN spawn point, so drone 1's
raw (north, east) is meaningless next to drone 0's unless shifted by
however far drone 1 was spawned from drone 0's home.
"""

import bisect
import csv
import math
import os
import time
from collections import deque
from functools import partial

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
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
        # Inference size. Camera capture is 1280x960 (mono_cam SDF) - imgsz must
        # match or ultralytics downscales the detail straight back out (measured
        # in experiments/px_sweep.py: conf 0.06 vs 0.68 at 10 m for the same
        # 1280 capture at imgsz 640 vs 1280). This is what raises the altitude
        # ceiling from 5.6 m to 11.2 m per SWARM_PLAN.md.
        self.declare_parameter('imgsz', 1280)
        self.declare_parameter('half', True)                # fp16 on GPU (ignored on CPU)
        # --- geolocation quality ---
        self.declare_parameter('pose_lag_s', 0.25)          # camera+bridge latency to compensate
        self.declare_parameter('min_alt_m', 1.0)            # below this, geometry is meaningless
        self.declare_parameter('max_alt_m', 40.0)           # above this, footprint is huge & useless
        # --- gating (see docstring point 2) ---
        self.declare_parameter('gate_topic', '/survey/detecting')
        self.declare_parameter('require_gate', False)       # True = no gate msg -> no detection
        # --- geotag axis calibration (leave all False unless a known target proves otherwise) ---
        self.declare_parameter('geo_swap_axes', False)
        self.declare_parameter('geo_flip_forward', False)
        self.declare_parameter('geo_flip_right', False)
        # --- multi-drone (swarm): semicolon-separated, one entry per drone,
        # same order across all four. Empty = single drone, using the
        # singular params above (image_topic/gate_topic/annotated_topic),
        # unnamespaced (instance 0) and no home offset. See docstring.
        self.declare_parameter('image_topics', '')
        self.declare_parameter('pose_namespaces', '')
        self.declare_parameter('gate_topics', '')
        self.declare_parameter('annotated_topics', '')
        self.declare_parameter('home_offsets', '')

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
        self.pose_lag = float(gp('pose_lag_s').value)
        self.min_alt = float(gp('min_alt_m').value)
        self.max_alt = float(gp('max_alt_m').value)
        self.require_gate = bool(gp('require_gate').value)
        self.geo_swap = bool(gp('geo_swap_axes').value)
        self.geo_flip_f = bool(gp('geo_flip_forward').value)
        self.geo_flip_r = bool(gp('geo_flip_right').value)
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

        # Resolve the class-name filter to model class IDs so the filtering happens
        # INSIDE YOLO: unwanted classes are never drawn and never scored.
        self.keep_ids = None
        names = getattr(self.model, 'names', {}) or {}
        if self.keep:
            self.keep_ids = [i for i, n in names.items() if n in self.keep]
            unknown = self.keep - {names[i] for i in self.keep_ids}
            if unknown:
                self.get_logger().warn(
                    f"classes not in this model, ignored: {sorted(unknown)}")
            if not self.keep_ids:
                # An empty list would be handed to ultralytics as `classes=[]`,
                # whose behaviour is not worth relying on. Fall back to None
                # (= keep all) and let the name filter below reject everything,
                # so the failure is loud and predictable rather than silent.
                self.keep_ids = None
                self.get_logger().error(
                    "none of the requested classes exist in the model -> "
                    "nothing will be recorded. Check the 'classes' parameter.")
            else:
                self.get_logger().info(
                    f"class filter active: {sorted(self.keep)} -> ids {self.keep_ids}")

        # ---- multi-drone drone list (see module docstring) ----
        # Each entry: image_topic, pose_ns (PX4 DDS namespace prefix, '' =
        # instance 0 / unnamespaced), gate_topic, annotated_topic,
        # (offset_north, offset_east) to fold this drone's local NED frame
        # into the shared map frame. Falls back to the singular params for
        # exactly one drone when the plural params are all unset, so a plain
        # single-drone launch is byte-for-byte the old behaviour.
        img_topics = self._split(gp('image_topics').value)
        n = len(img_topics) if img_topics else 1
        pose_ns = self._split(gp('pose_namespaces').value, expect=n, default='')
        gate_topics = self._split(gp('gate_topics').value, expect=n,
                                   default=str(gp('gate_topic').value))
        annot_topics = self._split(gp('annotated_topics').value, expect=n,
                                    default=str(gp('annotated_topic').value))
        offsets = self._split(gp('home_offsets').value, expect=n, default='0,0')
        if not img_topics:
            img_topics = [self.image_topic]

        self.drones = []
        for i in range(n):
            on, oe = (float(v) for v in offsets[i].split(','))
            self.drones.append({
                'image_topic': img_topics[i],
                'pose_ns': pose_ns[i],
                'gate_topic': gate_topics[i],
                'annotated_topic': annot_topics[i] if n == 1 else self._per_drone_topic(annot_topics[i], i, n),
                'offset_n': on, 'offset_e': oe,
                # pose ring buffer (time-matched geolocation, docstring point 1):
                # wall-clock keyed, entries (t, x, y, z, heading). ~10 s at 50 Hz.
                'pose_t': deque(maxlen=600),
                'pose_v': deque(maxlen=600),
                # detection gate (docstring point 2)
                'gate': not self.require_gate,
                'gate_seen': False,
                'pub_annot': None,
            })
        if n > 1:
            self.get_logger().info(
                f"multi-drone mode: {n} drones sharing one detector + one hazard list")
            for i, d in enumerate(self.drones):
                self.get_logger().info(
                    f"  drone {i}: image='{d['image_topic']}' pose_ns='{d['pose_ns']}' "
                    f"gate='{d['gate_topic']}' offset=({d['offset_n']:.1f},{d['offset_e']:.1f})")

        self.hazards = []   # list of (north, east) already recorded - shared across all drones

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        gate_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Image QoS: BEST_EFFORT, KEEP_LAST, depth **1**.
        #
        # qos_profile_sensor_data has depth 5. The camera publishes at 15 Hz;
        # at imgsz 1280 the detector consumes ~2.9 FPS, so the queue sits FULL
        # and every frame handed to on_image is already ~4 frames (~0.27 s)
        # old before transport is even counted. That staleness lands directly
        # in the geotag: the 8 Sep flight at 10 m needed 0.76 s of pose_lag to
        # centre a detection that 0.25 s used to cover, and put the target 2.9 m
        # out instead of 0.75 m.
        #
        # Depth 1 makes the middleware DROP the backlog and hand over the
        # newest frame, which fixes the cause instead of compensating for it —
        # and it stays correct as the drone count changes, whereas a hand-tuned
        # pose_lag would need re-measuring for every N (a shared detector at
        # N drones consumes each stream N times slower, so the backlog, and the
        # error, would grow with the swarm).
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        for i, d in enumerate(self.drones):
            self.create_subscription(
                Image, d['image_topic'], partial(self.on_image, drone_idx=i),
                img_qos)
            # PX4 publishes VehicleLocalPosition on the versioned topic
            # (.../vehicle_local_position_v1) on this build; other builds use the
            # unversioned name. Subscribe to both. Without this the pose buffer
            # stays empty forever and NO detection can ever be geotagged.
            ns = d['pose_ns']
            for t in (f'{ns}/fmu/out/vehicle_local_position', f'{ns}/fmu/out/vehicle_local_position_v1'):
                self.create_subscription(
                    VehicleLocalPosition, t, partial(self.on_pos, drone_idx=i), px4_qos)
            self.create_subscription(
                Bool, d['gate_topic'], partial(self.on_gate, drone_idx=i), gate_qos)
            if self.publish_annotated:
                d['pub_annot'] = self.create_publisher(Image, d['annotated_topic'], 1)

        os.makedirs(os.path.dirname(self.hazard_csv), exist_ok=True)
        if not os.path.exists(self.hazard_csv):
            with open(self.hazard_csv, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['t_s', 'x_ned_north', 'y_ned_east', 'class', 'conf', 'alt_m', 'drone'])

        self.frame_count = 0
        for i, d in enumerate(self.drones):
            self.get_logger().info(
                f"detector up: drone {i} image='{d['image_topic']}' -> "
                f"annotated='{d['annotated_topic']}', hazards -> {self.hazard_csv}")
        self.get_logger().info(
            f"require_gate={self.require_gate} | pose_lag={self.pose_lag}s | "
            f"alt window [{self.min_alt}, {self.max_alt}] m | imgsz={self.imgsz}")

    # ---------- helpers ----------
    def _split(self, value, expect=None, default=None):
        """';'-separated param -> list of stripped strings.

        An EMPTY SEGMENT IS MEANINGFUL DATA here (e.g. pose_namespaces' ''
        means "instance 0, unnamespaced") and must NOT be dropped - only the
        whole-string-empty case ('', no ';' at all) means "not configured,
        broadcast `default` to every drone" (the single-drone fallback path).
        Dropping empty segments would shift every entry after the first ''
        down by one and silently misassign a drone's topic to another
        drone's pose - which is exactly what an earlier version of this
        function did (caught by a launch dry-run: pose_namespaces=';/px4_1'
        collapsed to ['/px4_1'] and broadcast to BOTH drones instead of
        ['', '/px4_1']).

        With `expect` given (a drone count): a single entry broadcasts that
        one value to every drone. Anything else that doesn't match `expect`
        in length is padded/truncated with `default` and a warning is
        logged, so a swarm launch with a missing entry fails loud (wrong
        count in the log) rather than silently misassigning a drone.
        """
        value = str(value)
        items = [] if value.strip() == '' else [s.strip() for s in value.split(';')]
        if expect is None:
            return items
        if not items:
            return [default] * expect
        if len(items) == expect:
            return items
        if len(items) == 1:
            return items * expect
        self.get_logger().warn(
            f"expected {expect} ';'-separated entries, got {len(items)}: {items} "
            f"-> padding/truncating with {default!r}")
        return (items + [default] * expect)[:expect]

    @staticmethod
    def _per_drone_topic(base, i, n):
        return base if n <= 1 else f"{base}_{i}"

    # ---------- callbacks ----------
    def on_pos(self, msg, drone_idx=0):
        if not (msg.xy_valid and msg.z_valid):
            return
        d = self.drones[drone_idx]
        d['pose_t'].append(time.time())
        d['pose_v'].append((float(msg.x), float(msg.y), float(msg.z), float(msg.heading)))

    def on_gate(self, msg, drone_idx=0):
        d = self.drones[drone_idx]
        new = bool(msg.data)
        if new != d['gate'] or not d['gate_seen']:
            self.get_logger().info(f"drone {drone_idx} detection gate -> {'OPEN' if new else 'closed'}")
        d['gate'] = new
        d['gate_seen'] = True

    def pose_at(self, t, drone_idx=0):
        """Nearest buffered pose (for the given drone) to wall-clock time t.
        None if that drone's buffer is empty or the closest sample is more
        than 0.5 s away (stale/no telemetry)."""
        d = self.drones[drone_idx]
        ts = d['pose_t']
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        j = min(cand, key=lambda k: abs(ts[k] - t))
        if abs(ts[j] - t) > 0.5:
            return None
        return d['pose_v'][j]

    def img_to_np(self, msg):
        """sensor_msgs/Image -> HxWx3 BGR numpy (handles rgb8/bgr8).

        np.frombuffer() is READ-ONLY. cv2.rectangle/putText write in place and
        raise on a read-only array, so force a writable copy. (The rgb8 path used
        to get one for free via the channel flip; bgr8 did not - that was a latent
        crash in test_perception.py.)
        """
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            arr = arr.reshape((msg.height, msg.width, 3))
        except ValueError:
            return None
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]  # RGB -> BGR
        return np.ascontiguousarray(arr).copy()

    def np_to_img(self, frame, header):
        m = Image()
        m.header = header
        m.height, m.width = frame.shape[:2]
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = frame.shape[1] * 3
        m.data = frame.tobytes()
        return m

    def project_to_ground(self, u, v, img_w, img_h, pose):
        """
        Approx nadir projection: pixel (u,v) -> (north, east) in local NED, using
        the pose the frame was actually taken at (not the latest one).

        The three geo_* flags exist so a mirrored/rotated result can be corrected
        without editing code. As of the calibration flight they should all stay
        False: with a target at N=10 E=15, the recorded East was 14.5-14.7 (good)
        and the North error was symmetric about the truth - latency, not axes.
        """
        x, y, z, yaw = pose
        h = -z                          # altitude above home (m); z is down
        if h < self.min_alt or h > self.max_alt:
            return None
        mpp = (2.0 * h * math.tan(self.hfov / 2.0)) / img_w   # metres per pixel
        du = (u - img_w / 2.0)          # +right in image
        dv = (v - img_h / 2.0)          # +down in image
        fwd = -dv * mpp                 # image up   -> forward
        right = du * mpp                # image right-> right
        if self.geo_swap:
            fwd, right = right, fwd
        if self.geo_flip_f:
            fwd = -fwd
        if self.geo_flip_r:
            right = -right
        north = x + fwd * math.cos(yaw) - right * math.sin(yaw)
        east = y + fwd * math.sin(yaw) + right * math.cos(yaw)
        return north, east

    def record_hazard(self, north, east, cls, conf, alt, drone_idx=0):
        # Shared across every drone: this ONE list (and CSV) is what makes
        # dedup correct for a swarm instead of something to bolt on - N
        # detector processes each appending to the same file would never
        # dedup against each other. See module docstring.
        for (hn, he) in self.hazards:
            if math.hypot(north - hn, east - he) < self.min_sep:
                return False            # already have one here
        self.hazards.append((north, east))
        with open(self.hazard_csv, 'a', newline='') as f:
            csv.writer(f).writerow(
                [f"{time.time():.1f}", f"{north:.2f}", f"{east:.2f}",
                 cls, f"{conf:.2f}", f"{alt:.1f}", str(drone_idx)])
        self.get_logger().info(
            f"HAZARD #{len(self.hazards)}: {cls} conf={conf:.2f} at "
            f"N={north:.1f} E={east:.1f} (alt {alt:.1f}m, drone {drone_idx})")
        return True

    def on_image(self, msg, drone_idx=0):
        # Timestamp FIRST: everything after this (decode, inference) is latency we
        # must not attribute to the drone's position.
        t_rx = time.time()
        d = self.drones[drone_idx]

        frame = self.img_to_np(msg)
        if frame is None:
            return

        # Gated off (climb / RTL / landing / no survey running): skip INFERENCE,
        # which is the expensive part, but keep republishing the raw frame so
        # rqt_image_view stays live. A frozen video feed looks like a crash.
        if not d['gate']:
            if self.publish_annotated and d['pub_annot'] is not None:
                d['pub_annot'].publish(self.np_to_img(frame, msg.header))
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
        # floods the log). Weights are already moved to fp16 on GPU at load time,
        # so we simply don't pass it here.
        results = self.model(frame, conf=self.conf, imgsz=self.imgsz,
                             device=self.device, classes=self.keep_ids,
                             verbose=False)
        r = results[0]
        img_h, img_w = frame.shape[:2]

        pose = self.pose_at(t_rx - self.pose_lag, drone_idx=drone_idx)
        if pose is None and len(r.boxes) and self.frame_count % 50 == 0:
            self.get_logger().warn(
                f"drone {drone_idx}: detections but no pose within 0.5 s of the "
                f"frame - is its PX4 telemetry flowing? (geotagging skipped)")

        for box in r.boxes:
            cls_id = int(box.cls[0])
            name = self.model.names.get(cls_id, str(cls_id)) if hasattr(self.model, 'names') else str(cls_id)
            # Backstop for the model-level `classes` filter: covers the case
            # where a requested name did not resolve to an id, and any model
            # whose names dict disagrees with its output ids.
            if self.keep is not None and name not in self.keep:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            if self.publish_annotated:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            if pose is None:
                continue
            g = self.project_to_ground(u, v, img_w, img_h, pose)
            if g is not None:
                # Fold this drone's own local-NED detection into the shared
                # map frame (drone 0's home) before recording it - see the
                # home_offsets note in the module docstring.
                north = g[0] + d['offset_n']
                east = g[1] + d['offset_e']
                self.record_hazard(north, east, name, conf, -pose[2], drone_idx=drone_idx)

        if self.publish_annotated and d['pub_annot'] is not None:
            d['pub_annot'].publish(self.np_to_img(frame, msg.header))


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
