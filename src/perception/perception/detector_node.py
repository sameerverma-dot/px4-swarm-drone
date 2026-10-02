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
deduplication for free (one hazard registry, not one per drone racing
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

import array
import bisect
import csv
import math
import os
import threading
import time
from collections import deque
from functools import partial

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from swarm_msgs.msg import HazardReport

from perception.hazard_registry import Hazard, HazardRegistry

try:
    import cv2
except ImportError as e:  # pragma: no cover
    raise SystemExit("cv2 not found - install with: pip install --user opencv-python") from e

try:
    from ultralytics import YOLO
except ImportError as e:  # pragma: no cover
    raise SystemExit("ultralytics not found - install with: pip install --user ultralytics") from e



def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q = (w, x, y, z), Hamilton convention."""
    w, x, y, z = q
    vx, vy, vz = v
    # v' = v + 2 w (q_v x v) + 2 q_v x (q_v x v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def quat_euler(q):
    """(roll, pitch, yaw) in radians from (w, x, y, z), aerospace ZYX."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


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
        # Dedup radius. Two detections closer than this are treated as one
        # object. The right value is ~2x the geolocation sigma: merging inside
        # your own error bars is correct, merging beyond them loses real targets.
        #
        # At 640 px / 5 m the error was ~0.75 m and 2.0 was comfortable. At
        # 1280 px / 10 m the 8 Sep flight put three hits on ONE person at
        # 0.78 / 2.44 / 3.08 m from truth (RMS 2.31 m), spaced 2.3-3.9 m apart -
        # all just outside 2.0, so one person was logged as THREE hazards.
        # 4.5 ~= 2 x 2.31 was the fix then - and it merged real objects 2-4 m
        # apart. With attitude-aware projection, frames from turns dropped
        # (max_rate_dps) and duplicate boxes merged (box_merge_m), the 2 Oct
        # calibration measured steady-flight scatter at 0.36 m RMS, 0.85 m max,
        # and error to truth at most 1.4 m (tools/analyse_sightings.py). 2.5 m
        # covers one drone's scatter and two drones disagreeing about one object,
        # and keeps objects >= 2.5 m apart separate even when seen in separate
        # frames (in the same frame they stay separate down to box_merge_m).
        #
        # Then 2.5 merged a real pair 2 m apart: seen far ahead, YOLO boxed both
        # people as one (fix at their midpoint), and a later view of one of them
        # alone fell within 2.5 m of it. A merged mine is a live mine missing
        # from the map; a duplicate is a second flag next to a real one. So err
        # to duplicates: 1.5 m, ~2x the 0.85 m worst steady scatter, with
        # near-nadir views weighted up (see record weights in process_frame).
        self.declare_parameter('min_sep_m', 1.5)            # dedup radius (metres)
        # Two boxes in ONE frame closer than this on the ground are one object
        # boxed twice (whole + part), not two objects. See hazard_registry.py.
        self.declare_parameter('box_merge_m', 1.0)
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
        # --- diagnostics ---
        # YOLO runs at this floor so near-misses are visible in the stats line;
        # only detections >= conf are drawn or recorded.
        self.declare_parameter('diag_conf', 0.25)
        self.declare_parameter('stats_period_s', 5.0)
        # --- onboard swarm (one detector per drone) ---
        # drone_id >= 0 = this detector runs ON drone `drone_id` and labels its
        # hazards d<id>-<n>. swarm_hazard_topic set = share the hazard list with
        # peers over that topic (see hazard_registry.py). Both off by default.
        self.declare_parameter('drone_id', -1)
        self.declare_parameter('swarm_hazard_topic', '')
        # Refined positions are re-broadcast at most this often per hazard.
        self.declare_parameter('broadcast_period_s', 1.0)
        # --- geolocation model ---
        # Rotate the camera ray by the drone's full attitude (roll, pitch, yaw)
        # instead of assuming level flight. Forward pitch at survey speed tilts a
        # fixed nadir camera backward, which a level-flight projection turns into
        # a consistent along-track error. False = old level-flight projection.
        self.declare_parameter('use_attitude', True)
        # Do not record while the drone is rotating faster than this (deg/s,
        # from the attitude 0.1 s either side of the frame). Turning onto a lane
        # it banks 20+ deg and yaws ~140 deg/s; a few tens of ms of timing error
        # is then several degrees, and those frames put a person directly under
        # the lane 5 m off (2 Oct). Steady survey flight is < 10 deg/s. 0 = off.
        self.declare_parameter('max_rate_dps', 30.0)
        # Every person box in a gated frame - recorded ones, and the ones below
        # threshold or dropped while turning (outcome 'below' / 'turning') -
        # with the pose, velocity and attitude it was geotagged with. Input to
        # tools/analyse_sightings.py (swath, timing, scatter). '' = off.
        self.declare_parameter('sightings_csv', '')

        gp = self.get_parameter
        self.image_topic = str(gp('image_topic').value)
        self.annotated_topic = str(gp('annotated_topic').value)
        self.weights = os.path.expanduser(str(gp('weights').value))
        self.conf = float(gp('conf').value)
        self.hfov = float(gp('hfov_rad').value)
        self.publish_annotated = bool(gp('publish_annotated').value)
        self.hazard_csv = os.path.expanduser(str(gp('hazard_csv').value))
        self.min_sep = float(gp('min_sep_m').value)
        self.drone_id = int(gp('drone_id').value)
        self.swarm_hazard_topic = str(gp('swarm_hazard_topic').value).strip()
        self.broadcast_period = float(gp('broadcast_period_s').value)
        self.use_attitude = bool(gp('use_attitude').value)
        self.max_rate = max(0.0, float(gp('max_rate_dps').value))
        self.box_merge = max(0.0, float(gp('box_merge_m').value))
        self.sightings_csv = os.path.expanduser(str(gp('sightings_csv').value).strip())
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
        self.keep_label = '/'.join(sorted(self.keep)) if self.keep else 'detection'
        self.diag_conf = float(gp('diag_conf').value)

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

        # Warm up NOW, on the pad. The first inference pays for CUDA kernel
        # setup at this imgsz; done lazily it landed on the first gated frame,
        # and with three detectors warming up on one GPU at once the first
        # 5-20 s of every drone's first lane went uninspected ("inferred 0.0
        # fps" while frames arrived - measured 2 Oct).
        t_w = time.time()
        try:
            self.model(np.zeros((960, 1280, 3), np.uint8), imgsz=self.imgsz,
                       device=self.device, verbose=False)
            self.get_logger().info(f"model warm-up {time.time() - t_w:.1f} s")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"model warm-up failed (first frame will be slow): {e}")

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
                # wall-clock keyed, entries (x, y, z, heading, vx, vy). ~10 s at 50 Hz.
                'pose_t': deque(maxlen=600),
                'pose_v': deque(maxlen=600),
                # attitude ring buffer, same keying: quaternion (w, x, y, z) FRD->NED
                'att_t': deque(maxlen=1200),
                'att_v': deque(maxlen=1200),
                # detection gate (docstring point 2)
                'gate': not self.require_gate,
                'gate_seen': False,
                'pub_annot': None,
                'st': self._new_stats(),
            })
        if n > 1:
            self.get_logger().info(
                f"multi-drone mode: {n} drones sharing one detector + one hazard list")
            for i, d in enumerate(self.drones):
                self.get_logger().info(
                    f"  drone {i}: image='{d['image_topic']}' pose_ns='{d['pose_ns']}' "
                    f"gate='{d['gate_topic']}' offset=({d['offset_n']:.1f},{d['offset_e']:.1f})")

        self.me = self.drone_id if self.drone_id >= 0 else 0
        self.registry = HazardRegistry(self.me, self.min_sep, self.box_merge)
        self._suppressed = set()               # peer hazard ids we've reported re-seeing
        self.hazard_lock = threading.Lock()   # infer worker vs peer-report callback
        self._published = {}                  # own hazard id -> (version, wall time) last broadcast
        self._sight_f = None
        if self.sightings_csv:
            os.makedirs(os.path.dirname(self.sightings_csv), exist_ok=True)
            self._sight_f = open(self.sightings_csv, 'w', newline='')
            self._sight_w = csv.writer(self._sight_f)
            self._sight_w.writerow(
                ['t_rx', 'drone', 'u', 'v', 'img_w', 'img_h', 'conf', 'x', 'y', 'z',
                 'vx', 'vy', 'yaw', 'roll', 'pitch', 'north', 'east', 'north_level',
                 'east_level', 'hazard_id', 'outcome'])

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

        # Two callback groups on a MultiThreadedExecutor (see main()). On a
        # single-threaded executor no pose callback can run while YOLO is busy,
        # so at imgsz 1280 the "50 Hz" pose ring buffer was measured filling at
        # 1 Hz (27 Sep, 2 drones). Frames then either matched a pose up to
        # ~0.5 s off (≈1.9 m at 3.8 m/s - geotag scatter) or found none within
        # 0.5 s and were DROPPED: drone 1's conf-0.65 hit was lost exactly that
        # way. Images stay mutually exclusive (one model, one GPU); pose, gate
        # and stats run alongside them.
        self.cb_images = MutuallyExclusiveCallbackGroup()   # receive + stash only
        self.cb_infer = MutuallyExclusiveCallbackGroup()    # the one YOLO worker
        self.cb_state = MutuallyExclusiveCallbackGroup()    # pose, gate, stats
        self.pose_lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self._rr = 0
        for i, d in enumerate(self.drones):
            d['latest'] = None
            self.create_subscription(
                Image, d['image_topic'], partial(self.on_image, drone_idx=i),
                img_qos, callback_group=self.cb_images)
            # PX4 publishes VehicleLocalPosition on the versioned topic
            # (.../vehicle_local_position_v1) on this build; other builds use the
            # unversioned name. Subscribe to both. Without this the pose buffer
            # stays empty forever and NO detection can ever be geotagged.
            ns = d['pose_ns']
            for t in (f'{ns}/fmu/out/vehicle_local_position', f'{ns}/fmu/out/vehicle_local_position_v1'):
                self.create_subscription(
                    VehicleLocalPosition, t, partial(self.on_pos, drone_idx=i), px4_qos,
                    callback_group=self.cb_state)
            if self.use_attitude or self.max_rate > 0:
                for t in (f'{ns}/fmu/out/vehicle_attitude', f'{ns}/fmu/out/vehicle_attitude_v1'):
                    self.create_subscription(
                        VehicleAttitude, t, partial(self.on_att, drone_idx=i), px4_qos,
                        callback_group=self.cb_state)
            self.create_subscription(
                Bool, d['gate_topic'], partial(self.on_gate, drone_idx=i), gate_qos,
                callback_group=self.cb_state)
            if self.publish_annotated:
                d['pub_annot'] = self.create_publisher(Image, d['annotated_topic'], 1)

        self._open_csv()

        # Shared hazard list. Onboard, each drone's detector broadcasts what it
        # finds and folds in what its peers find. RELIABLE + TRANSIENT_LOCAL so a
        # late joiner (a rebooted drone, the optional ground station) receives
        # the history, not just what is broadcast after it starts.
        self.pub_hazard = None
        if self.swarm_hazard_topic:
            hz_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=200)
            self.pub_hazard = self.create_publisher(HazardReport, self.swarm_hazard_topic, hz_qos)
            self.create_subscription(HazardReport, self.swarm_hazard_topic,
                                     self.on_peer_hazard, hz_qos,
                                     callback_group=self.cb_state)
            self.get_logger().info(
                f"shared hazard list on '{self.swarm_hazard_topic}' as drone {self.me}")

        for i, d in enumerate(self.drones):
            self.get_logger().info(
                f"detector up: drone {self._label(i)} image='{d['image_topic']}' -> "
                f"annotated='{d['annotated_topic']}', hazards -> {self.hazard_csv}")
        self.get_logger().info(
            f"require_gate={self.require_gate} | pose_lag={self.pose_lag}s | "
            f"alt window [{self.min_alt}, {self.max_alt}] m | imgsz={self.imgsz}")

        # Per-drone health line while any gate is open. A shared detector can
        # starve one drone, and a target seen just under `conf` never produces a
        # HAZARD line - both looked identical ("drone 1 found nothing") until
        # this existed.
        self.stats_period = max(1.0, float(gp('stats_period_s').value))
        self._stats_t0 = time.time()
        self.create_timer(self.stats_period, self.log_stats, callback_group=self.cb_state)
        self.create_timer(0.5, self.flush_refinements, callback_group=self.cb_state)
        # Busy-ish poll: a tick with nothing waiting is a few microseconds, and a
        # tick that finds a frame runs inference back-to-back.
        self.create_timer(0.005, self.infer_tick, callback_group=self.cb_infer)

    @staticmethod
    def _new_stats():
        return {'rx': 0, 'inferred': 0, 'pose': 0, 'best': 0.0, 'best_below': 0.0,
                'hits': 0, 'no_pose': 0, 'alt_window': 0, 'unsteady': 0,
                'lag_pose': [], 'lag_att': [], 'lag_img': [],
                't_frame': 0.0, 't_yolo': 0.0, 't_preprocess': 0.0,
                't_inference': 0.0, 't_postprocess': 0.0}

    def log_stats(self):
        now = time.time()
        dt = max(now - self._stats_t0, 1e-6)
        self._stats_t0 = now
        if not any(d['gate'] and d['gate_seen'] for d in self.drones) and self.require_gate:
            for d in self.drones:
                d['st'] = self._new_stats()
            return
        for i, d in enumerate(self.drones):
            st = d['st']
            msg = (f"drone {self._label(i)} [{'OPEN' if d['gate'] else 'closed'}] "
                   f"camera {st['rx'] / dt:.1f} fps, inferred {st['inferred'] / dt:.1f} fps, "
                   f"pose {st['pose'] / dt:.0f} Hz, "
                   f"best {self.keep_label} {st['best']:.2f} "
                   f"(below-threshold best {st['best_below']:.2f}), hits {st['hits']}")
            if st['inferred']:
                n = st['inferred']
                msg += (f" | ms/frame: total {1000 * st['t_frame'] / n:.0f}, yolo "
                        f"{1000 * st['t_yolo'] / n:.0f} (pre {1000 * st['t_preprocess'] / n:.0f}"
                        f" / gpu {1000 * st['t_inference'] / n:.0f}"
                        f" / post {1000 * st['t_postprocess'] / n:.0f})")
            # Arrival delay after PX4's own timestamp (pose, att), and image
            # delay above the period's fastest frame (img, whose stamp is sim time).
            lags = [(k, sorted(st['lag_' + k])) for k in ('pose', 'att', 'img')]
            if all(v for _, v in lags):
                lags[2] = ('img+', [x - lags[2][1][0] for x in lags[2][1]])
                msg += " | delay ms median/max: " + ", ".join(
                    f"{k} {1000 * v[len(v) // 2]:.0f}/{1000 * v[-1]:.0f}" for k, v in lags)
            if st['no_pose'] or st['alt_window'] or st['unsteady']:
                msg += (f", DROPPED {st['no_pose']} no-pose / "
                        f"{st['alt_window']} outside alt window / "
                        f"{st['unsteady']} turning")
            if st['no_pose'] or (d['gate'] and st['rx'] == 0):
                self.get_logger().warn(msg + ("  <- NO CAMERA FRAMES" if st['rx'] == 0 else ""))
            else:
                self.get_logger().info(msg)
            d['st'] = self._new_stats()

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

    def _label(self, idx):
        """Drone number for log lines: the onboard drone id if this detector
        runs on a drone, else the camera's index in a shared detector."""
        return self.drone_id if self.drone_id >= 0 else idx

    @staticmethod
    def _per_drone_topic(base, i, n):
        return base if n <= 1 else f"{base}_{i}"

    # ---------- callbacks ----------
    def on_pos(self, msg, drone_idx=0):
        if not (msg.xy_valid and msg.z_valid):
            return
        d = self.drones[drone_idx]
        d['st']['pose'] += 1
        now = time.time()
        d['st']['lag_pose'].append(now - msg.timestamp * 1e-6)
        with self.pose_lock:    # pose_at() reads these from the image thread
            d['pose_t'].append(now)
            d['pose_v'].append((float(msg.x), float(msg.y), float(msg.z), float(msg.heading),
                                float(msg.vx), float(msg.vy)))

    def on_att(self, msg, drone_idx=0):
        d = self.drones[drone_idx]
        now = time.time()
        d['st']['lag_att'].append(now - msg.timestamp * 1e-6)
        with self.pose_lock:
            d['att_t'].append(now)
            d['att_v'].append(tuple(float(v) for v in msg.q))

    def on_gate(self, msg, drone_idx=0):
        d = self.drones[drone_idx]
        new = bool(msg.data)
        if new != d['gate'] or not d['gate_seen']:
            self.get_logger().info(f"drone {self._label(drone_idx)} detection gate -> {'OPEN' if new else 'closed'}")
        d['gate'] = new
        d['gate_seen'] = True

    def _nearest(self, ts, vs, t, max_dt):
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        j = min(cand, key=lambda k: abs(ts[k] - t))
        return vs[j] if abs(ts[j] - t) <= max_dt else None

    def pose_at(self, t, drone_idx=0):
        """Nearest buffered pose (for the given drone) to wall-clock time t.
        None if that drone's buffer is empty or the closest sample is more
        than 0.5 s away (stale/no telemetry)."""
        d = self.drones[drone_idx]
        with self.pose_lock:
            return self._nearest(d['pose_t'], d['pose_v'], t, 0.5)

    def att_at(self, t, drone_idx=0):
        """Attitude quaternion nearest t, or None if none within 0.1 s
        (attitude changes fast; a stale one is worse than assuming level)."""
        d = self.drones[drone_idx]
        with self.pose_lock:
            return self._nearest(d['att_t'], d['att_v'], t, 0.1)

    def turn_rate(self, t, drone_idx=0):
        """Rotation rate (deg/s) around time t from the attitude 0.1 s either
        side, or None if the buffer does not cover it."""
        d = self.drones[drone_idx]
        with self.pose_lock:
            q0 = self._nearest(d['att_t'], d['att_v'], t - 0.1, 0.05)
            q1 = self._nearest(d['att_t'], d['att_v'], t + 0.1, 0.05)
        if q0 is None or q1 is None:
            return None
        dot = min(1.0, abs(sum(a * b for a, b in zip(q0, q1))))
        return math.degrees(2.0 * math.acos(dot)) / 0.2

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
        # array('B'), NOT bytes: rclpy's generated setter stores an array('B')
        # as-is, but validates any other sequence element by element in Python
        # - 296 ms for one 1280x960 frame vs 5 ms (measured 27 Sep). That was
        # ~85% of every frame's cost and the real cause of the old "~2 Hz, not
        # YOLO" pipeline ceiling (PROGRESS.md 5.1).
        m.data = array.array('B', frame.tobytes())
        return m

    def project_to_ground(self, u, v, img_w, img_h, pose, q=None):
        """
        Pixel (u,v) -> (north, east) in local NED: the camera ray, rotated into
        NED, intersected with flat ground at home altitude - using the pose the
        frame was actually taken at (not the latest one).

        Camera mounting (x500_mono_cam_down: mono_cam pitched 90 deg about body
        Y): optical axis = body down, image up = body forward, image right = body
        right. In body FRD a pixel's ray is (fwd, right, 1) with
        fwd = -(v - cy)/f, right = (u - cx)/f, f = (img_w/2)/tan(hfov/2).

        q = attitude quaternion (w, x, y, z), FRD body -> NED. With q the ray is
        rotated by the full attitude, so forward pitch at survey speed (which
        tilts the "nadir" camera backward) no longer lands as a consistent
        along-track error. q None = level flight at heading `yaw` - the original
        model, and exactly what this reduces to when roll = pitch = 0.

        The three geo_* flags exist so a mirrored/rotated result can be corrected
        without editing code. They should all stay False: with a target at N=10
        E=15 the recorded East was 14.5-14.7 and the North error was symmetric
        about the truth - latency, not axes.
        """
        x, y, z, yaw = pose[:4]
        h = -z                          # altitude above home (m); z is down
        if h < self.min_alt or h > self.max_alt:
            return None
        f = (img_w / 2.0) / math.tan(self.hfov / 2.0)
        fwd = -(v - img_h / 2.0) / f    # image up    -> forward
        right = (u - img_w / 2.0) / f   # image right -> right
        if self.geo_swap:
            fwd, right = right, fwd
        if self.geo_flip_f:
            fwd = -fwd
        if self.geo_flip_r:
            right = -right
        if q is None:
            dn = fwd * math.cos(yaw) - right * math.sin(yaw)
            de = fwd * math.sin(yaw) + right * math.cos(yaw)
            return x + h * dn, y + h * de
        dn, de, dd = quat_rotate(q, (fwd, right, 1.0))
        if dd < 0.2:                    # ray near-horizontal: no sane ground hit
            return None
        t = h / dd
        return x + t * dn, y + t * de

    CSV_HEADER = ['t_s', 'x_ned_north', 'y_ned_east', 'class', 'conf', 'alt_m',
                  'drone', 'hazard_id', 'status', 'logged_by',
                  'version', 'n_sightings', 'spread_m']

    def _open_csv(self):
        """Append-only log. A file left by an older version (different columns)
        is set aside rather than appended to with a different column count."""
        os.makedirs(os.path.dirname(self.hazard_csv), exist_ok=True)
        if os.path.exists(self.hazard_csv):
            with open(self.hazard_csv, newline='') as f:
                header = next(csv.reader(f), [])
            if header == self.CSV_HEADER:
                return
            old = f"{self.hazard_csv[:-4]}_oldformat_{int(time.time())}.csv"
            os.rename(self.hazard_csv, old)
            self.get_logger().warn(f"older-format hazard CSV moved aside -> {old}")
        with open(self.hazard_csv, 'w', newline='') as f:
            csv.writer(f).writerow(self.CSV_HEADER)

    def _log_row(self, h, status):
        """status: own (first sighting), update (own refinement), peer,
        peer_update, superseded. A hazard's CURRENT position is its last row."""
        with open(self.hazard_csv, 'a', newline='') as f:
            csv.writer(f).writerow(
                [f"{h.stamp:.1f}", f"{h.north:.2f}", f"{h.east:.2f}", h.cls,
                 f"{h.conf:.2f}", f"{h.alt:.1f}", str(h.origin), h.hazard_id,
                 status, str(self.me), str(h.version), str(h.n), f"{h.spread:.2f}"])

    def record_frame(self, sightings, drone_idx=0):
        """All of one frame's geotagged detections at once, so the registry can
        associate them one-to-one: two objects in the same frame never merge
        (mines 1-3 m apart). See hazard_registry.py.
        sightings: list of (north, east, alt, cls, conf, weight). -> list of
        (outcome, hazard) per sighting."""
        origin = self.me if self.drone_id >= 0 else drone_idx
        new, notes = [], []
        with self.hazard_lock:
            out = self.registry.add_frame(sightings, time.time(), origin=origin)
            for (outcome, h) in out:
                if outcome == 'new':
                    self._log_row(h, 'own')
                    new.append(h)
                elif outcome == 'seen' and h.hazard_id not in self._suppressed:
                    # A PEER logged it - the shared list doing its job.
                    self._suppressed.add(h.hazard_id)
                    notes.append(h)
            n_known = len(self.registry.items)
        for h in new:
            self._publish(h)
            self.get_logger().info(
                f"HAZARD {h.hazard_id}: {h.cls} conf={h.conf:.2f} at "
                f"N={h.north:.1f} E={h.east:.1f} (alt {h.alt:.1f}m, drone {origin}) "
                f"- swarm list now {n_known}")
        for h in notes:
            self.get_logger().info(
                f"saw {h.cls} at N={h.north:.1f} E={h.east:.1f} - already logged by "
                f"drone {h.origin} as {h.hazard_id}, not logged again")
        return out

    def _publish(self, h):
        self._published[h.hazard_id] = (h.version, time.time())
        if self.pub_hazard is not None:
            self.pub_hazard.publish(self._to_report(h))

    def flush_refinements(self, force=False):
        """Re-broadcast own hazards whose position improved, at most once per
        broadcast_period_s each, so peers converge on the refined position."""
        now = time.time()
        todo = []
        with self.hazard_lock:
            for hid, (ver, t) in list(self._published.items()):
                h = self.registry.items.get(hid)
                if h is None or h.version <= ver:
                    continue
                if force or now - t >= self.broadcast_period:
                    self._log_row(h, 'update')
                    todo.append(h)
        for h in todo:
            self._publish(h)

    def _to_report(self, h):
        m = HazardReport()
        m.hazard_id, m.origin_drone, m.seq = h.hazard_id, h.origin, h.seq
        m.version, m.n_sightings, m.spread_m = h.version, h.n, float(h.spread)
        m.stamp.sec = int(h.stamp)
        m.stamp.nanosec = int((h.stamp - int(h.stamp)) * 1e9)
        m.north, m.east, m.alt = float(h.north), float(h.east), float(h.alt)
        m.cls, m.conf = h.cls, float(h.conf)
        return m

    def on_peer_hazard(self, m):
        if m.origin_drone == self.me:
            return                      # our own broadcast coming back
        h = Hazard(m.hazard_id, m.origin_drone, m.seq,
                   m.stamp.sec + m.stamp.nanosec * 1e-9,
                   m.north, m.east, m.alt, m.cls, m.conf,
                   version=m.version, n=m.n_sightings, spread=m.spread_m)
        with self.hazard_lock:
            outcome, superseded = self.registry.add_peer(h)
            if outcome == 'new':
                self._log_row(h, 'peer')
            elif outcome == 'update':
                self._log_row(self.registry.items[h.hazard_id], 'peer_update')
            elif outcome == 'replaces':
                self._log_row(superseded, 'superseded')
                self._log_row(h, 'peer')
        if outcome in ('repeat', 'update'):
            return
        msg = {'new': 'added to swarm list',
               'dup': 'duplicate of an earlier hazard, ignored',
               'replaces': f"same object as our {superseded.hazard_id if superseded else ''}"
                           f" and earlier - ours superseded"}[outcome]
        self.get_logger().info(
            f"peer hazard {h.hazard_id} at N={h.north:.1f} E={h.east:.1f}: {msg}")

    def on_image(self, msg, drone_idx=0):
        """Stash only. Timestamped on ARRIVAL: everything after this (queueing
        behind another drone's inference, decode, inference) is latency we must
        not attribute to the drone's position. Newer frames overwrite older
        ones, so the worker always gets the freshest frame per drone."""
        t_rx = time.time()
        d = self.drones[drone_idx]
        d['st']['rx'] += 1
        # In sim the stamp is SIM time, so only the spread of this is meaningful.
        d['st']['lag_img'].append(t_rx - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        with self.frame_lock:
            d['latest'] = (msg, t_rx)

    def infer_tick(self):
        """Serve ONE waiting frame, round-robin across drones.

        With image callbacks in one mutually-exclusive group, the executor
        handed every free slot to drone 0's subscription (created first, and it
        always had a fresh frame): drone 1 was measured at 0 fps. Explicit
        round-robin gives each of N drones 1/N of the detector."""
        n = len(self.drones)
        for k in range(n):
            idx = (self._rr + k) % n
            d = self.drones[idx]
            with self.frame_lock:
                item, d['latest'] = d['latest'], None
            if item is not None:
                self._rr = (idx + 1) % n
                self.process_frame(item[0], item[1], idx)
                return

    def process_frame(self, msg, t_rx, drone_idx):
        t_start = time.time()
        d = self.drones[drone_idx]
        frame = self.img_to_np(msg)
        if frame is None:
            return
        # Only build/publish the annotated image while something is watching it.
        # Three detectors each pushing 3.7 MB frames that nobody subscribed to
        # cut throughput from 8.0 to 3.3 fps per drone and pose from 50 to 22 Hz
        # (measured 2 Oct). rqt_image_view / grab_frames.py subscribing turns it on.
        annotate = (self.publish_annotated and d['pub_annot'] is not None
                    and d['pub_annot'].get_subscription_count() > 0)

        # Gated off (climb / RTL / landing / no survey running): skip INFERENCE,
        # which is the expensive part, but keep republishing the raw frame so
        # rqt_image_view stays live. A frozen video feed looks like a crash.
        if not d['gate']:
            if annotate:
                d['pub_annot'].publish(self.np_to_img(frame, msg.header))
            return
        st = d['st']
        st['inferred'] += 1
        # NOTE: 'half' is deprecated in ultralytics >=8.4 (warns once per frame and
        # floods the log). Weights are already moved to fp16 on GPU at load time,
        # so we simply don't pass it here.
        # Inference runs at the DIAGNOSTIC floor, not at `conf`, so a target seen
        # at 0.55 against a 0.65 threshold shows up in the stats line instead of
        # vanishing. Only boxes >= conf are drawn or recorded.
        t0 = time.time()
        results = self.model(frame, conf=min(self.conf, self.diag_conf), imgsz=self.imgsz,
                             device=self.device, classes=self.keep_ids,
                             verbose=False)
        r = results[0]
        st['t_yolo'] += time.time() - t0
        for k in ('preprocess', 'inference', 'postprocess'):
            st['t_' + k] += (getattr(r, 'speed', None) or {}).get(k, 0.0) / 1000.0
        img_h, img_w = frame.shape[:2]

        t_cap = t_rx - self.pose_lag      # best estimate of when the frame was taken
        pose = self.pose_at(t_cap, drone_idx=drone_idx)
        q = self.att_at(t_cap, drone_idx=drone_idx) if self.use_attitude else None
        rate = self.turn_rate(t_cap, drone_idx=drone_idx) if self.max_rate > 0 else None
        turning = rate is not None and rate > self.max_rate

        sightings, meta, below = [], [], []
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
            if conf < self.conf:
                st['best_below'] = max(st['best_below'], conf)
                # Below threshold: never recorded, but logged for calibration -
                # confidence against where in the image the object was is what
                # sets the usable detection swath (and so the lane spacing).
                if self._sight_f is not None and pose is not None:
                    g = self.project_to_ground(u, v, img_w, img_h, pose, q)
                    if g is not None:
                        below.append((u, v, conf, g, 'below'))
                continue
            st['best'] = max(st['best'], conf)

            if annotate:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            if pose is None:
                st['no_pose'] += 1
                continue
            g = self.project_to_ground(u, v, img_w, img_h, pose, q)
            if g is None:
                st['alt_window'] += 1
                continue
            if turning:
                st['unsteady'] += 1
                below.append((u, v, conf, g, 'turning'))
                continue
            # Fold this drone's own local-NED detection into the shared map
            # frame (drone 0's home) before recording it - see the
            # home_offsets note in the module docstring.
            north = g[0] + d['offset_n']
            east = g[1] + d['offset_e']
            # Weight for the hazard's position estimate: confidence x cos^2 of
            # the off-nadir angle. Oblique views carry more attitude/height
            # error and are where two close objects get boxed as one.
            h2 = pose[2] * pose[2]
            r2 = (g[0] - pose[0]) ** 2 + (g[1] - pose[1]) ** 2
            sightings.append((north, east, -pose[2], name, conf, conf * h2 / max(h2 + r2, 1e-6)))
            meta.append((u, v, conf, g))

        out = self.record_frame(sightings, drone_idx=drone_idx) if sightings else []
        st['hits'] += sum(1 for o, _ in out if o == 'new')
        if self._sight_f is not None and (meta or below):
            roll, pitch, yaw = quat_euler(q) if q is not None else (0.0, 0.0, pose[3])
            rows = [(m, h.hazard_id, o) for m, (o, h) in zip(meta, out)] + \
                   [(m[:4], '', m[4]) for m in below]
            for (u, v, conf, g), hid, outcome in rows:
                gl = self.project_to_ground(u, v, img_w, img_h, pose, None)
                self._sight_w.writerow(
                    [f"{t_rx:.3f}", self._label(drone_idx), f"{u:.1f}", f"{v:.1f}",
                     img_w, img_h, f"{conf:.3f}"]
                    + [f"{c:.3f}" for c in (pose[0], pose[1], pose[2], pose[4], pose[5],
                                            yaw, roll, pitch,
                                            g[0] + d['offset_n'], g[1] + d['offset_e'],
                                            gl[0] + d['offset_n'], gl[1] + d['offset_e'])]
                    + [hid, outcome])
            self._sight_f.flush()

        if annotate:
            d['pub_annot'].publish(self.np_to_img(frame, msg.header))
        st['t_frame'] += time.time() - t_start


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    # Multi-threaded so the pose/gate callback group keeps running while the
    # image group is inside YOLO - see the callback-group comment in __init__.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.flush_refinements(force=True)   # last refined positions into the log
            if node._sight_f is not None:
                node._sight_f.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
