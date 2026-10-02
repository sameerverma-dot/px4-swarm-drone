#!/usr/bin/env python3
"""
grab_frames.py - save EVERY annotated detector frame to disk during a flight.

    ros2 run ... is not needed; just run it with python3 while the mission flies:

        python3 tools/grab_frames.py                                  # drone 0
        python3 tools/grab_frames.py /detection/image_annotated_1     # drone 1
        python3 tools/grab_frames.py /detection/image_annotated_0 ~/maps/frames

WHY THIS EXISTS
Catching the YOLO box live in rqt_image_view means hitting a button inside a
~5 second window while also watching Gazebo. That is a bad way to get a
screenshot you actually need. This subscribes to the same annotated stream and
writes every frame, so after the flight you scan a folder of thumbnails and
pick the best one instead of racing the aircraft.

Frames are JPEG (quality 95) named by arrival order and wall-clock time, so
they sort correctly and you can match a frame to a HAZARD log line by its
timestamp.

Stop it with Ctrl-C when the survey completes.
"""
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:
    sys.exit("cv2 not importable - source install/setup.bash first")


class Grabber(Node):
    def __init__(self, topic, outdir):
        super().__init__('grab_frames')
        self.outdir = outdir
        self.n = 0
        os.makedirs(outdir, exist_ok=True)
        # /detection/* is published BEST_EFFORT with depth 1, same as the
        # detector's own image input. A RELIABLE subscription would not match
        # and this node would sit silent - the exact trap documented in
        # docs/CHEATSHEET.md.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, topic, self.on_img, qos)
        self.get_logger().info(f"saving frames from {topic} -> {outdir}")

    def on_img(self, msg):
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            if msg.encoding in ('bgr8', 'rgb8'):
                img = buf.reshape(h, w, 3)
                if msg.encoding == 'rgb8':
                    img = img[:, :, ::-1]
            elif msg.encoding == 'mono8':
                img = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:
                self.get_logger().warn(f"unhandled encoding {msg.encoding}")
                return
        except ValueError:
            return  # partial frame
        self.n += 1
        name = f"frame_{self.n:04d}_{time.time():.1f}.jpg"
        cv2.imwrite(os.path.join(self.outdir, name),
                    np.ascontiguousarray(img),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if self.n % 10 == 0:
            self.get_logger().info(f"{self.n} frames saved")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/detection/image_annotated_0'
    outdir = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/maps/frames')
    rclpy.init()
    node = Grabber(topic, outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done - {node.n} frames in {outdir}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
