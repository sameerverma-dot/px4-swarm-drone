#!/usr/bin/env python3
"""
test_perception.py — offline end-to-end test of the detection pipeline.

Proves YOLO detection + pixel->world geolocation + hazard CSV writing WITHOUT
needing Gazebo, a flight, or a detectable model in the world.

It publishes:
  * a real test image (Ultralytics' bundled bus.jpg — contains people + a bus,
    which are COCO classes yolov8n knows) on  /test/image
  * a synthetic VehicleLocalPosition on /fmu/out/vehicle_local_position so the
    detector's geolocation has a pose to work with (20 m altitude, at origin)

RUN IT WITH THE SIM STOPPED (it fakes /fmu/out/vehicle_local_position).

Terminal 1:
    cd ~/px4_ros_ws && source install/setup.bash
    ros2 run perception detector_node --ros-args -p image_topic:=/test/image

Terminal 2:
    cd ~/px4_ros_ws && source install/setup.bash
    python3 src/perception/test_perception.py

Terminal 3 (watch):
    ros2 run rqt_image_view rqt_image_view /detection/image_annotated

EXPECT: detector logs "HAZARD #n: person ..." lines, boxes in rqt_image_view,
and rows appearing in ~/maps/hazard_points.csv.
"""

import os
import pathlib
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition

try:
    import cv2
except ImportError:
    sys.exit("cv2 missing:  pip install --user opencv-python")


def find_test_image():
    """Ultralytics ships bus.jpg/zidane.jpg — use one, else synthesise."""
    try:
        import ultralytics
        assets = pathlib.Path(ultralytics.__file__).parent / 'assets'
        for name in ('bus.jpg', 'zidane.jpg'):
            p = assets / name
            if p.exists():
                img = cv2.imread(str(p))
                if img is not None:
                    print(f"[test] using {p}")
                    return img
    except Exception as e:  # noqa: BLE001
        print(f"[test] ultralytics assets unavailable: {e}")
    print("[test] WARNING: falling back to a synthetic image — YOLO will likely "
          "find nothing in it. That tests plumbing only, not detection.")
    img = np.full((480, 640, 3), 120, np.uint8)
    cv2.rectangle(img, (250, 180), (390, 400), (60, 60, 60), -1)
    return img


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_perception_publisher')
        self.frame = find_test_image()
        h, w = self.frame.shape[:2]
        self.get_logger().info(f"publishing {w}x{h} test image at 5 Hz on /test/image")

        self.pub_img = self.create_publisher(Image, '/test/image', qos_profile_sensor_data)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_pos = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', px4_qos)

        self.create_timer(0.2, self.tick)   # 5 Hz

    def tick(self):
        now = self.get_clock().now().to_msg()
        us = int(self.get_clock().now().nanoseconds / 1000)

        # --- synthetic pose: hovering at 20 m over home, facing north ---
        p = VehicleLocalPosition()
        p.timestamp = us
        p.xy_valid = True
        p.z_valid = True
        p.x, p.y, p.z = 0.0, 0.0, -20.0     # NED: 20 m up
        p.heading = 0.0
        self.pub_pos.publish(p)

        # --- image ---
        m = Image()
        m.header.stamp = now
        m.header.frame_id = 'test_cam'
        m.height, m.width = self.frame.shape[:2]
        m.encoding = 'bgr8'
        m.is_bigendian = 0
        m.step = self.frame.shape[1] * 3
        m.data = self.frame.tobytes()
        self.pub_img.publish(m)


def main():
    rclpy.init()
    n = TestPublisher()
    print("[test] publishing... Ctrl-C to stop. Watch the detector terminal for HAZARD lines.")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
        csv = os.path.expanduser('~/maps/hazard_points.csv')
        if os.path.exists(csv):
            print(f"\n[test] hazard CSV now contains "
                  f"{sum(1 for _ in open(csv)) - 1} row(s): {csv}")


if __name__ == '__main__':
    main()
