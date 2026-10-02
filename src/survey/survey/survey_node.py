#!/usr/bin/env python3
"""
survey_node.py - boustrophedon ("lawnmower") area survey for a PX4 multirotor.

Flies one vehicle over a rectangular area defined in the PX4 local NED frame
(home = 0,0), climbing to a fixed altitude and sweeping back and forth along
the X (north) axis, stepping in Y (east) by lane_spacing.

Runnable two ways (identical behaviour):
  ros2 run survey survey_node --ros-args -p x_max:=40.0 -p y_max:=30.0 ...
  python3 survey_node.py --ros-args -p x_max:=40.0 ...        (standalone)

Frame: PX4 local NED. z is DOWN, so flight altitude -> z = -altitude.

Verification (verify:=true): records the flown track from
/fmu/out/vehicle_local_position, checks every target waypoint was approached
within reach_tol, checks return-to-home when RTL is enabled, writes the track
to a CSV in ~/maps, and logs a single PASS/FAIL line.

PRE-REQ set in the PX4 (pxh>) console before arming from ROS 2:
  param set NAV_DLL_ACT 0
  param set CBRK_SUPPLY_CHK 894281
(otherwise external arming fails with "No connection to GCS")

Heading: yaw_mode defaults to 'course' (face the direction of travel). Setpoints
used to go out with the default yaw=0.0, which is an ACTIVE command to face
North, so the drone crabbed sideways down every south-bound lane.
Use yaw_mode:=fixed to get the old behaviour back.

lane_spacing:=0.0 (default) derives it from what the DETECTOR can see, not from
the camera footprint: 2*altitude*tan(detect_fov_deg/2)*(1-sidelap). The 2 Oct
calibration (tools/analyse_sightings.py) found a person is reliably scored
>= 0.65 only within ~2.5 m of the track at 10 m altitude, against a 23.7 m wide
footprint: footprint-spaced lanes (16.6 m) left targets 7.5 m off-lane unrecorded.

MULTI-DRONE: namespace:='' (default) is PX4 instance 0, unnamespaced /fmu/...
For instance i>0 PX4 namespaces its DDS topics /px4_<i>/fmu/... (PX4's rcS);
pass the matching namespace:='/px4_1' etc.

--------------------------------------------------------------------------
ONBOARD SWARM MODE (drone_id >= 0)
--------------------------------------------------------------------------
This node is the flight half of what runs on each drone's onboard computer.
It needs nothing from a ground station: it computes its own band from
(drone_id, num_drones, swarm_y_min, swarm_y_max), flies it, logs, and returns.
Peers talk to each other directly over /swarm/heartbeat (swarm_msgs):

  * heartbeat     ~4 Hz: id, shared-frame position, state, band/lane, lanes
                  done, claimed band, ETA, battery.
  * takeover      when its own band is done, a drone takes over a peer's
                  unfinished lanes if the peer returned early, or went silent
                  and its projected finish time has passed (a silent drone may
                  still be flying - see swarm_logic.SwarmCoordinator). The
                  nearest available drone claims; ties -> lower id.
  * separation    if a peer comes inside sep_horizontal_m / sep_vertical_m, one
                  drone gives way: a working drone beats an idle (HOLD) one,
                  which beats one already yielding, ties to the lower id; a
                  peer under PX4 control (RTL) is always given way to. The
                  yielding drone stops, moves vertically clear (never through
                  the other drone), then carries on along its route at that
                  altitude until the peer is clear. On top of altitude
                  separation: takeover transits fly a drone-specific layer
                  above survey altitude; PX4 RTL altitudes are staggered.

All decision logic lives in swarm_logic.py and is unit-tested there. With
drone_id < 0 (default) none of this runs and the node behaves exactly as the
single-drone survey always has.

Fault-injection hooks for testing (off by default): start_delay_s,
abort_after_lanes.
"""

import csv
import math
import os
import time
from enum import Enum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from swarm_msgs.msg import DroneHeartbeat

from survey import swarm_logic as sl


class Phase(Enum):
    WAIT = -1         # swarm test hook: start_delay_s on the pad
    INIT = 0          # stream setpoints, then request offboard + arm
    ENGAGE = 1        # wait for offboard + armed confirmation
    SURVEY = 2        # fly self.wp (own band, then any takeover appended)
    RTL = 3           # command RTL, wait for it to engage
    RETURN_WAIT = 4   # wait until back near home
    DONE = 5          # verified, CSV written, shutting down
    HOLD = 6          # swarm: own band done, hovering while a peer is undecided


LANE_KINDS = ('lane_start', 'lane_end')


class SurveyNode(Node):
    def __init__(self):
        super().__init__('survey_node')

        # ---- parameters ----
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 40.0)
        self.declare_parameter('y_min', 0.0)
        self.declare_parameter('y_max', 30.0)
        self.declare_parameter('altitude', 10.0)       # metres AGL (positive)
        # lane_spacing <= 0 derives it from the camera footprint (see below),
        # which is what you actually want for a coverage survey. A positive
        # value overrides the derivation.
        self.declare_parameter('lane_spacing', 0.0)    # metres between lanes; 0 = derive
        self.declare_parameter('sidelap', 0.2)         # fraction of overlap between lanes
        self.declare_parameter('hfov_rad', 1.74)       # mono_cam SDF; must match the detector
        # Full cross-track angle within which the detector reliably scores a
        # target >= its threshold. Measured 2 Oct over three flights, 74 steady
        # passes of a COCO 'person' at 10 m (tools/analyse_sightings.py):
        #   lateral 0-1.5 m 10/11 recorded, 1.5-3 m 7/11, 3-4.5 m 3/12,
        #   beyond 4.5 m 10/40 (only lucky oblique views).
        # 28 deg = +-2.5 m at 10 m; with 20% sidelap -> 4 m lanes, so every point
        # is within 2 m of one lane and ~2 m of the next: >= ~90% per target.
        # Re-measure for any other weights - a landmine model has its own swath.
        self.declare_parameter('detect_fov_deg', 28.0)
        # Each lane starts and ends this far OUTSIDE [x_min, x_max], and the
        # detection gate is open only while the drone is over [x_min, x_max].
        # Turning onto a lane the drone banks 20+ deg and yaws ~140 deg/s, and
        # geotags stay poor for a while after: over two flights (2 Oct) the
        # error to truth was p90 1.47 m in the first 6 m of a lane against
        # <= 0.5 m from 6 m on, and ~1 m again while braking at the end. The
        # camera sees ~9 m ahead and behind, so the area's edges are still seen.
        self.declare_parameter('lead_in_m', 6.0)
        self.declare_parameter('reach_tol', 1.5)       # waypoint reach tolerance (m)
        self.declare_parameter('return_tol', 3.0)      # home-return tolerance (m)
        self.declare_parameter('rtl_on_complete', True)
        self.declare_parameter('verify', True)
        self.declare_parameter('csv_dir', '~/maps')
        # PX4 DDS namespace prefix for a multi-vehicle sim, e.g. '/px4_1' for
        # instance 1 ('' = instance 0, unnamespaced - the single-drone default).
        self.declare_parameter('namespace', '')
        # Filename prefix for the track CSV. Two drones finishing in the same
        # wall-clock second (a real risk in a swarm) would otherwise collide,
        # since write_csv() names the file by whole seconds.
        self.declare_parameter('csv_prefix', 'survey_track')
        self.declare_parameter('arm_timeout_s', 30.0)
        self.declare_parameter('return_timeout_s', 180.0)
        # Carrot-chasing lookahead. A bare position setpoint at the far end of a
        # lane makes PX4 sprint at MPC_XY_VEL_MAX - the first camera run flew the
        # lanes at ~9.3 m/s, which at ~5 FPS is ~2 m of blur+latency per frame and
        # wrecks geotag accuracy. Feeding the setpoint forward a fixed distance
        # instead caps ground speed at roughly MPC_XY_P * lookahead (~0.95 * L).
        # Set 0.0 to restore the old fly-flat-out behaviour.
        self.declare_parameter('lookahead_m', 4.0)
        # Topic the detector listens on so it only geotags during actual survey
        # lanes - not during climb, RTL or landing.
        self.declare_parameter('detect_topic', '/survey/detecting')
        # Heading control. Every TrajectorySetpoint used to go out with the
        # default yaw=0.0, which is an ACTIVE command to face North - so the
        # drone crabbed sideways down every south-bound lane.
        #   'course' - face the direction of travel (default)
        #   'fixed'  - hold fixed_yaw_deg (0 = North; the old behaviour)
        #   'hold'   - send NaN, i.e. don't command yaw at all; PX4 keeps
        #              whatever heading it has
        self.declare_parameter('yaw_mode', 'course')
        self.declare_parameter('fixed_yaw_deg', 0.0)
        # Below this distance to the waypoint the bearing is numerically
        # meaningless (atan2 of two tiny numbers), so freeze the last command
        # instead of letting the nose twitch.
        self.declare_parameter('yaw_deadzone_m', 1.0)

        # --- onboard swarm mode (see module docstring); drone_id < 0 = off ---
        self.declare_parameter('drone_id', -1)
        self.declare_parameter('num_drones', 1)
        self.declare_parameter('swarm_y_min', 0.0)     # whole area, shared frame
        self.declare_parameter('swarm_y_max', 60.0)
        self.declare_parameter('heartbeat_topic', '/swarm/heartbeat')
        self.declare_parameter('heartbeat_hz', 4.0)
        self.declare_parameter('peer_timeout_s', 3.0)
        self.declare_parameter('startup_grace_s', 45.0)
        self.declare_parameter('deadline_margin_s', 15.0)
        self.declare_parameter('claim_wait_s', 20.0)
        self.declare_parameter('takeover', True)
        self.declare_parameter('min_takeover_battery', 30.0)
        self.declare_parameter('max_hold_s', 120.0)
        self.declare_parameter('transit_alt_offset_m', 3.0)
        self.declare_parameter('min_battery_pct', 20.0)   # below -> return early; <0 off
        self.declare_parameter('separation', True)
        self.declare_parameter('sep_horizontal_m', 8.0)
        self.declare_parameter('sep_vertical_m', 5.0)
        self.declare_parameter('sep_climb_m', 5.0)
        self.declare_parameter('sep_clear_s', 2.0)
        # A yield moves away horizontally; after this long, against a peer that
        # is holding its altitude, it passes over/under instead (separation()).
        self.declare_parameter('yield_pass_s', 20.0)
        # --- fault injection, for testing the swarm behaviours ---
        self.declare_parameter('start_delay_s', 0.0)
        self.declare_parameter('abort_after_lanes', -1)

        g = self.get_parameter
        self.x_min = float(g('x_min').value)
        self.x_max = float(g('x_max').value)
        lead = max(0.0, float(g('lead_in_m').value))
        self.lane_x0, self.lane_x1 = self.x_min - lead, self.x_max + lead
        self.y_min = float(g('y_min').value)
        self.y_max = float(g('y_max').value)
        self.altitude = float(g('altitude').value)
        self.lane_spacing = float(g('lane_spacing').value)
        self.sidelap = min(max(float(g('sidelap').value), 0.0), 0.9)
        self.hfov = float(g('hfov_rad').value)
        self.reach_tol = float(g('reach_tol').value)
        self.return_tol = float(g('return_tol').value)
        self.rtl_on_complete = bool(g('rtl_on_complete').value)
        self.verify = bool(g('verify').value)
        self.csv_dir = os.path.expanduser(str(g('csv_dir').value))
        self.namespace = str(g('namespace').value).strip()
        if self.namespace and not self.namespace.startswith('/'):
            self.namespace = '/' + self.namespace
        self.csv_prefix = str(g('csv_prefix').value)
        self.arm_timeout_s = float(g('arm_timeout_s').value)
        self.return_timeout_s = float(g('return_timeout_s').value)
        self.lookahead = float(g('lookahead_m').value)
        self.detect_topic = str(g('detect_topic').value)
        self.yaw_mode = str(g('yaw_mode').value).strip().lower()
        if self.yaw_mode not in ('course', 'fixed', 'hold'):
            self.get_logger().warn(
                f"unknown yaw_mode '{self.yaw_mode}' -> falling back to 'course'")
            self.yaw_mode = 'course'
        self.fixed_yaw = math.radians(float(g('fixed_yaw_deg').value))
        self.yaw_deadzone = float(g('yaw_deadzone_m').value)

        self.me = int(g('drone_id').value)
        self.swarm = self.me >= 0
        self.num_drones = max(1, int(g('num_drones').value))
        self.sy_min = float(g('swarm_y_min').value)
        self.sy_max = float(g('swarm_y_max').value)
        self.hb_period = 1.0 / max(0.5, float(g('heartbeat_hz').value))
        self.peer_timeout = float(g('peer_timeout_s').value)
        self.takeover_on = bool(g('takeover').value)
        self.max_hold_s = float(g('max_hold_s').value)
        self.transit_off = float(g('transit_alt_offset_m').value)
        self.min_batt = float(g('min_battery_pct').value)
        self.sep_on = bool(g('separation').value)
        self.sep_h = float(g('sep_horizontal_m').value)
        self.sep_v = float(g('sep_vertical_m').value)
        self.sep_climb = float(g('sep_climb_m').value)
        self.sep_clear_s = float(g('sep_clear_s').value)
        self.yield_pass_s = float(g('yield_pass_s').value)
        self.start_delay = max(0.0, float(g('start_delay_s').value))
        self.abort_after = int(g('abort_after_lanes').value)

        self.z = -abs(self.altitude)  # NED down: negative = up

        # Derive lane spacing from what the camera can actually see, rather than
        # guessing. A nadir camera at height h sees a strip 2*h*tan(HFOV/2) wide;
        # stepping by that much times (1 - sidelap) guarantees the requested
        # overlap between adjacent lanes. Too wide leaves unphotographed gaps
        # between lanes - the survey "passes" while missing ground.
        #
        # The camera seeing the ground is not the detector finding the target:
        # far off-track the object is small and seen obliquely and confidence
        # drops below threshold. So the strip that counts is the DETECTION
        # swath (detect_fov_deg), capped at the footprint.
        self.footprint_w = 2.0 * self.altitude * math.tan(self.hfov / 2.0)
        self.swath_w = min(self.footprint_w, 2.0 * self.altitude * math.tan(
            math.radians(min(max(float(g('detect_fov_deg').value), 1.0), 170.0)) / 2.0))
        if self.lane_spacing <= 0.0:
            self.lane_spacing = max(self.swath_w * (1.0 - self.sidelap), 0.5)
            self.get_logger().info(
                f"lane_spacing derived: detection swath {self.swath_w:.2f} m "
                f"(camera footprint {self.footprint_w:.2f} m) at {self.altitude:.1f} m "
                f"altitude, {self.sidelap:.0%} sidelap -> {self.lane_spacing:.2f} m")
        elif self.lane_spacing > self.swath_w:
            self.get_logger().warn(
                f"lane_spacing {self.lane_spacing:.1f} m EXCEEDS the detection "
                f"swath {self.swath_w:.1f} m at {self.altitude:.1f} m altitude -> "
                f"targets between lanes can go unrecorded. "
                f"Use lane_spacing:=0.0 to derive it.")

        if self.swarm:
            # Band geometry first: build_waypoints() below needs it.
            n, me = self.num_drones, self.me
            self.band_h = sl.band_height(n, self.sy_min, self.sy_max)
            self.my_east0 = sl.band_origin_east(me, n, self.sy_min, self.sy_max)
            # The survey box this drone actually flies, in its own local frame.
            self.y_min, self.y_max = 0.0, self.band_h
            self.lanes_y = sl.lane_offsets(self.band_h, self.lane_spacing)

        # ---- QoS: matches px4_ros_com examples (proven with this PX4/px4_msgs) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- publishers ----
        ns = self.namespace
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos)

        # Detection gate for the perception node. TRANSIENT_LOCAL so a detector
        # that starts late still gets the current value instead of guessing.
        gate_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_detect = self.create_publisher(Bool, self.detect_topic, gate_qos)

        # ---- subscribers ----
        # PX4 message versioning: messages with MESSAGE_VERSION > 0 are published
        # on a topic with a _vN suffix (VehicleLocalPosition -> _v1,
        # VehicleStatus -> _v4). Older/other builds use the unversioned name.
        # Subscribe to BOTH so this works on either, whichever actually publishes.
        for t in (f'{ns}/fmu/out/vehicle_local_position', f'{ns}/fmu/out/vehicle_local_position_v1'):
            self.create_subscription(VehicleLocalPosition, t, self.on_pos, qos)
        for t in (f'{ns}/fmu/out/vehicle_status', f'{ns}/fmu/out/vehicle_status_v4'):
            self.create_subscription(VehicleStatus, t, self.on_status, qos)

        # ---- state ----
        self.pos = VehicleLocalPosition()
        self.pos_valid = False
        self.status = VehicleStatus()
        self.phase = Phase.INIT
        self.done = False        # set when the node may exit; main() watches it
        self.done_at = None      # swarm: keep heartbeating briefly after DONE
        self.counter = 0
        self.last_engage_t = 0.0
        self.t0 = self.now_s()
        self.phase_t0 = self.t0
        self.wp = self.build_waypoints()
        self.wp_idx = 0
        self.wp_min_dist = [float('inf')] * len(self.wp)
        self.track = []          # (t_s, x, y, z)
        self.returned = False
        self.detecting = None            # last value published on detect_topic
        self.reached_alt = False         # latched once we first reach survey altitude
        # Last yaw actually commanded in 'course' mode (rad, NED). Seeded NaN,
        # not fixed_yaw: before the first real bearing is computed there is no
        # course to face, and seeding it with fixed_yaw (default 0.0) meant
        # 'course' mode opened by commanding North - a heading nobody asked for.
        # NaN means "don't control yaw yet", so the drone keeps whatever heading
        # it has until an actual course exists. 'fixed' mode does not use this.
        self.cmd_yaw = float('nan')

        if self.swarm:
            self._init_swarm(qos)

        self.get_logger().info(
            f"Survey ns='{self.namespace or '(none)'}' x[{self.x_min},{self.x_max}] "
            f"y[{self.y_min},{self.y_max}] alt={self.altitude}m lane={self.lane_spacing}m "
            f"-> {len(self.wp)} waypoints; RTL={'on' if self.rtl_on_complete else 'off'}, "
            f"verify={self.verify}, lookahead={self.lookahead}m, yaw={self.yaw_mode}")
        self.publish_detecting(False)

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    # ---------- swarm setup ----------
    def _init_swarm(self, px4_qos):
        n, me = self.num_drones, self.me
        self.coord = sl.SwarmCoordinator(
            me, n, len(self.lanes_y),
            peer_timeout_s=self.peer_timeout,
            startup_grace_s=float(self.get_parameter('startup_grace_s').value),
            deadline_margin_s=float(self.get_parameter('deadline_margin_s').value),
            min_takeover_battery=float(self.get_parameter('min_takeover_battery').value),
            claim_wait_s=float(self.get_parameter('claim_wait_s').value))
        self.coord.start(self.now_s())

        self.own_lanes_done = 0
        self.mask = 0                    # bands this drone has fully covered
        self.claim = None                # {'band', 'from', 'lanes_done'} while taking over
        self.takeovers = []              # history for the VERIFY line
        self.hold_t0 = None
        self.hold_xy = None
        self.hold_reason = ''
        self.yielding = None             # see separation()
        self.last_yield = {}             # peer id -> when my last yield to it ended
        self.gate_alt_ok = False
        self.battery = -1.0
        self.end_reason = ''
        self.offboard_lost_t = None
        self.peer_live = {}              # for SILENT / back log lines
        self.peers_heard = set()
        self.last_hb_t = 0.0
        self.cruise = 0.95 * self.lookahead if self.lookahead > 0 else 8.0

        ns = self.namespace
        hb_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        topic = str(self.get_parameter('heartbeat_topic').value)
        self.pub_hb = self.create_publisher(DroneHeartbeat, topic, hb_qos)
        self.create_subscription(DroneHeartbeat, topic, self.on_heartbeat, hb_qos)
        for t in (f'{ns}/fmu/out/battery_status', f'{ns}/fmu/out/battery_status_v1'):
            self.create_subscription(BatteryStatus, t, self.on_battery, px4_qos)
        if self.start_delay > 0:
            self.phase = Phase.WAIT
        self.get_logger().info(
            f"SWARM drone {me}/{n}: band {me} = shared east "
            f"[{self.my_east0:.1f}, {self.my_east0 + self.band_h:.1f}], "
            f"{len(self.lanes_y)} lanes; takeover={'on' if self.takeover_on else 'off'}, "
            f"separation={'on' if self.sep_on else 'off'} ({self.sep_h:g} m / {self.sep_v:g} m)"
            + (f"; TEST start_delay {self.start_delay:g} s" if self.start_delay else "")
            + (f"; TEST abort after {self.abort_after} lane(s)" if self.abort_after >= 0 else ""))

    # ---------- time helpers ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ---------- waypoint generation ----------
    def build_waypoints(self):
        """Waypoints as dicts: x, y, z (local NED), kind, band, lane.

        Single-drone: exactly the old pattern - climb at home, then lanes over
        [y_min, y_max]. Swarm: the same pattern over this drone's own band."""
        wps = [dict(x=0.0, y=0.0, z=self.z, kind='climb', band=-1, lane=-1)]
        if self.swarm:
            for x, y, k, end in sl.lane_waypoints(self.lanes_y, self.lane_x0, self.lane_x1):
                wps.append(dict(x=x, y=y, z=self.z, kind='lane_end' if end else 'lane_start',
                                band=self.me, lane=k))
            return wps
        # Same strip-centred rule as the swarm (swarm_logic.lane_offsets): lanes
        # down the middle of equal strips, none wasted on the area's edges.
        ys = [self.y_min + o for o in sl.lane_offsets(self.y_max - self.y_min, self.lane_spacing)]
        for i, yy in enumerate(ys):
            a, b = (self.lane_x0, self.lane_x1) if i % 2 == 0 else (self.lane_x1, self.lane_x0)
            wps.append(dict(x=a, y=yy, z=self.z, kind='lane_start', band=-1, lane=i))
            wps.append(dict(x=b, y=yy, z=self.z, kind='lane_end', band=-1, lane=i))
        return wps

    def takeover_waypoints(self, band, from_lane):
        """Lanes from_lane.. of `band`, in MY local frame, entered from the
        nearer end, reached by a transit in this drone's own altitude layer."""
        dy = sl.band_origin_east(band, self.num_drones, self.sy_min, self.sy_max) - self.my_east0
        px, py = (self.pos.x, self.pos.y) if self.pos_valid else (0.0, 0.0)
        best = None
        for rev in (False, True):
            lanes = sl.lane_waypoints(self.lanes_y, self.lane_x0, self.lane_x1, from_lane, rev)
            d = math.hypot(lanes[0][0] - px, lanes[0][1] + dy - py)
            if best is None or d < best[0]:
                best = (d, lanes)
        lanes = best[1]
        z_transit = self.z - (self.transit_off + 1.0 * self.me)   # up = more negative
        x0, y0 = lanes[0][0], lanes[0][1] + dy
        wps = [dict(x=px, y=py, z=z_transit, kind='transit', band=band, lane=-1),
               dict(x=x0, y=y0, z=z_transit, kind='transit', band=band, lane=-1)]
        for x, y, k, end in lanes:
            wps.append(dict(x=x, y=y + dy, z=self.z,
                            kind='lane_end' if end else 'lane_start', band=band, lane=k))
        return wps

    # ---------- subscriptions ----------
    def on_pos(self, msg):
        self.pos = msg
        self.pos_valid = bool(msg.xy_valid and msg.z_valid)
        if self.pos_valid and self.phase not in (Phase.WAIT, Phase.INIT, Phase.DONE):
            self.track.append((self.now_s() - self.t0, msg.x, msg.y, msg.z))

    def on_status(self, msg):
        self.status = msg

    def on_battery(self, msg):
        if msg.remaining >= 0:
            self.battery = 100.0 * float(msg.remaining)

    def on_heartbeat(self, m):
        if m.drone_id == self.me:
            return
        now = self.now_s()
        if m.drone_id not in self.peers_heard:
            self.peers_heard.add(m.drone_id)
            self.get_logger().info(
                f"peer {m.drone_id} heard: {sl.STATE_NAMES[m.state]}, "
                f"lanes {m.own_lanes_done}/{m.own_lanes_total}")
        self.coord.update(sl.PeerInfo(
            m.drone_id, m.state, m.north, m.east, m.alt, m.own_lanes_done,
            m.own_lanes_total, m.claimed_band, m.claimed_from_lane,
            m.bands_done_mask, m.eta_s, m.battery_pct, now))

    @property
    def is_offboard(self):
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def is_armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    # ---------- publishers ----------
    def send_ocm(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.timestamp = self.us()
        self.pub_ocm.publish(m)

    def send_sp(self, x, y, z, yaw=float('nan')):
        # The default is NaN, NOT 0.0, and that is the whole point. PX4 reads a
        # NaN yaw setpoint as "don't control yaw"; it reads 0.0 as "face North".
        # This function used to default to 0.0, so any caller that forgot the
        # argument silently commanded North - which is exactly how the drone
        # ended up crabbing sideways down every south-bound lane while the code
        # looked correct. Defaulting to NaN makes a forgotten argument harmless
        # instead of wrong.
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.velocity = [float('nan')] * 3
        m.acceleration = [float('nan')] * 3
        m.yaw = float(yaw)
        m.timestamp = self.us()
        self.pub_sp.publish(m)

    def climb_yaw(self):
        """Yaw during the vertical climb: NaN (leave it alone) unless the user
        explicitly asked for a fixed heading."""
        return self.fixed_yaw if self.yaw_mode == 'fixed' else float('nan')

    def desired_yaw(self, tx, ty):
        """Yaw to command while heading for waypoint (tx, ty), in radians.

        PX4 yaw is measured from NORTH, positive toward EAST. Local NED has
        x=North and y=East, so the bearing from here to the target is
        atan2(delta_east, delta_north) - i.e. atan2(dy, dx), NOT the atan2(y, x)
        you would write for a normal maths plot.

        Returns NaN in 'hold' mode: PX4 treats a NaN yaw setpoint as "don't
        control yaw", which is different from commanding 0 (= face North).
        """
        if self.yaw_mode == 'hold':
            return float('nan')
        if self.yaw_mode == 'fixed':
            return self.fixed_yaw
        # 'course' - face the direction of travel
        if self.pos_valid:
            dx, dy = tx - self.pos.x, ty - self.pos.y
            if math.hypot(dx, dy) >= self.yaw_deadzone:
                self.cmd_yaw = math.atan2(dy, dx)
        # Inside the deadzone (or before we have a position) keep the last
        # command, so the nose does not spin while the drone climbs or settles
        # onto a waypoint. Until a bearing has ever been computed cmd_yaw is
        # still NaN, which PX4 reads as "leave yaw alone" - the correct answer
        # when there is no course yet, and not the same as facing North.
        return self.cmd_yaw

    def course_yaw(self, idx):
        """desired_yaw() for waypoint idx, except that heading for a lane START
        the drone already faces along that lane: it sidesteps onto the lane
        instead of swinging up to 180 deg at its start, so it is straight when
        the area - and the detection gate - begins. (2 Oct: the swing from home
        onto lane 0 was still going at the area edge, and a 0.85 detection of
        a target 4 m inside was dropped as 'turning'.)"""
        w = self.wp[idx]
        if (self.yaw_mode == 'course' and w['kind'] == 'lane_start'
                and idx + 1 < len(self.wp) and self.wp[idx + 1]['kind'] == 'lane_end'):
            nxt = self.wp[idx + 1]
            self.cmd_yaw = math.atan2(nxt['y'] - w['y'], nxt['x'] - w['x'])
            return self.cmd_yaw
        return self.desired_yaw(w['x'], w['y'])

    def send_sp_limited(self, x, y, z, yaw=float('nan')):
        """Position setpoint with a speed cap.

        Instead of handing PX4 the far end of the lane (which it flies at
        MPC_XY_VEL_MAX), place the setpoint `lookahead_m` ahead of where we
        actually are. PX4's position controller then commands roughly
        MPC_XY_P * lookahead m/s. Altitude is always the true target so the climb
        is unaffected. lookahead_m <= 0 restores the direct behaviour.
        """
        if self.lookahead > 0.0 and self.pos_valid:
            dx, dy = x - self.pos.x, y - self.pos.y
            d = math.hypot(dx, dy)
            if d > self.lookahead:
                x = self.pos.x + dx / d * self.lookahead
                y = self.pos.y + dy / d * self.lookahead
        self.send_sp(x, y, z, yaw)

    def publish_detecting(self, on):
        """Tell the detector whether frames right now are worth geotagging."""
        on = bool(on)
        if on == self.detecting:
            return
        self.detecting = on
        self.pub_detect.publish(Bool(data=on))
        self.get_logger().info(f"detection gate -> {'OPEN' if on else 'closed'}")

    def send_cmd(self, command, **p):
        m = VehicleCommand()
        m.command = command
        m.param1 = float(p.get('param1', 0.0))
        m.param2 = float(p.get('param2', 0.0))
        m.param3 = float(p.get('param3', 0.0))
        m.param4 = float(p.get('param4', 0.0))
        m.param5 = float(p.get('param5', 0.0))
        m.param6 = float(p.get('param6', 0.0))
        m.param7 = float(p.get('param7', 0.0))
        # target_system=0 means "broadcast, any system" - PX4's commander
        # (Commander::handle_command) silently DROPS a command whose
        # target_system doesn't match its own vehicle_status.system_id, with
        # no error either side. Instance N sets MAV_SYS_ID=N+1 (PX4 rcS), so a
        # hardcoded target_system=1 would silently stop working on any
        # namespaced instance - exactly this project's GZ_IP/unversioned-topic
        # class of bug. 0 works for every instance because each instance's
        # command topic is already isolated by the DDS namespace, not by id.
        m.target_system = 0
        m.target_component = 0
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.us()
        self.pub_cmd.publish(m)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard(self):
        # DO_SET_MODE: base custom (1), main mode 6 = OFFBOARD
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def engage_rtl(self):
        # DO_SET_MODE: base custom (1), main mode 4 = AUTO, sub-mode 5 = RTL
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0, param3=5.0)

    # ---------- distances ----------
    def dist_to(self, x, y, z):
        return math.sqrt((self.pos.x - x) ** 2 + (self.pos.y - y) ** 2 + (self.pos.z - z) ** 2)

    def home_dist(self):
        return math.hypot(self.pos.x, self.pos.y)

    # ---------- swarm: heartbeat ----------
    def hb_state(self):
        p = self.phase
        if p in (Phase.WAIT,):
            return sl.BOOT
        if p in (Phase.INIT, Phase.ENGAGE):
            return sl.ENGAGING
        if p in (Phase.RTL, Phase.RETURN_WAIT):
            return sl.RETURNING
        if p == Phase.DONE:
            return sl.LANDED if self.returned else sl.FAILED
        if self.yielding:
            return sl.YIELD
        if p == Phase.HOLD:
            return sl.HOLD
        return sl.TAKEOVER if self.claim else sl.SURVEY

    def eta(self):
        if self.phase not in (Phase.INIT, Phase.ENGAGE, Phase.SURVEY):
            return 0.0
        rest = [(w['x'], w['y']) for w in self.wp[self.wp_idx:]]
        start = (self.pos.x, self.pos.y) if self.pos_valid else (0.0, 0.0)
        return sl.path_eta(start, rest, self.cruise)

    def publish_heartbeat(self):
        m = DroneHeartbeat()
        m.drone_id, m.num_drones = self.me, self.num_drones
        m.stamp = self.get_clock().now().to_msg()
        m.state = self.hb_state()
        if self.pos_valid:
            m.north, m.east, m.alt = (float(self.pos.x), float(self.pos.y + self.my_east0),
                                      float(-self.pos.z))
        else:
            m.north, m.east, m.alt = 0.0, float(self.my_east0), 0.0
        w = self.wp[self.wp_idx] if self.phase == Phase.SURVEY and self.wp_idx < len(self.wp) else None
        m.band = int(w['band']) if w else -1
        m.lane = int(w['lane']) if w else -1
        m.own_lanes_done = self.own_lanes_done
        m.own_lanes_total = len(self.lanes_y)
        m.claimed_band = self.claim['band'] if self.claim else -1
        m.claimed_from_lane = self.claim['from'] if self.claim else 0
        m.bands_done_mask = self.mask
        m.eta_s = float(self.eta())
        m.battery_pct = float(self.battery)
        self.pub_hb.publish(m)

    def log_peer_changes(self, now):
        for j in range(self.num_drones):
            if j == self.me or j not in self.coord.peers:
                continue
            live = self.coord.live(j, now)
            if self.peer_live.get(j, True) != live:
                p = self.coord.peers[j]
                if live:
                    self.get_logger().info(f"peer {j} back on the air")
                elif p.state in (sl.LANDED, sl.FAILED):
                    self.get_logger().info(
                        f"peer {j} off the air after {sl.STATE_NAMES[p.state]} "
                        f"(lanes {p.own_lanes_done}/{p.own_lanes_total})")
                else:
                    self.get_logger().warn(
                        f"peer {j} SILENT (last heard {now - p.rx_time:.1f} s ago: "
                        f"{sl.STATE_NAMES[p.state]}, lanes {p.own_lanes_done}/"
                        f"{p.own_lanes_total}, eta {p.eta_s:.0f} s)")
            self.peer_live[j] = live

    # ---------- swarm: separation ----------
    def work_state(self):
        """What I am doing, ignoring any yield - my priority in a conflict."""
        if self.phase == Phase.HOLD:
            return sl.HOLD
        return sl.TAKEOVER if self.claim else sl.SURVEY

    def separation(self, now):
        """-> True if this tick's setpoint has been taken over by a yield.

        CLEAR (the default yield): keep my altitude and move straight away from
        the peer until 1.25 x sep_h apart. It never changes altitude, so it
        cannot cross a peer that is itself climbing or descending. The previous
        version yielded vertically first; on 2 Oct it went up just as the peer
        began its RTL climb, and they passed 3.3 m apart at the same height.

        HOLD -> PASS: only against a peer that is holding its altitude
        (|vz| < 0.3 m/s) and still in my way after yield_pass_s, or that I
        yielded to in the last 30 s (it is parked on my route). Go vertically
        to the yield altitude at my position, then carry on along my route over
        or under it rather than waiting for it to leave. If it starts climbing
        or descending, back to CLEAR. (A peer in HOLD never blocks a working
        drone - right of way - so this is for a stationary peer under PX4
        control.)

        The yield ends when the peer is 1.25 x sep_h away horizontally (in
        CLEAR also: sep_v + 1 m away vertically) for sep_clear_s, or is silent.
        """
        if not (self.sep_on and self.pos_valid and self.is_armed
                and self.phase in (Phase.SURVEY, Phase.HOLD)):
            self.yielding = None
            return False
        me = (self.pos.x, self.pos.y + self.my_east0, -self.pos.z)
        peers = [(j, p.state, p.north, p.east, p.alt, now - p.rx_time)
                 for j, p in self.coord.peers.items()]
        hit = sl.separation_conflict(self.me, self.work_state(), me, peers,
                                     self.sep_h, self.sep_v, max_age_s=1.5)
        y = self.yielding
        if hit and hit[3]:
            if y is None:
                p = self.coord.peers.get(hit[0])
                self.yielding = y = dict(peer=hit[0], x=self.pos.x, y=self.pos.y, z=self.pos.z,
                                         stage='clear', clear_since=None, t0=now,
                                         warned=False, no_pass=False)
                self.publish_detecting(False)
                self.get_logger().warn(
                    f"SEPARATION: peer {hit[0]} at {hit[1]:.1f} m horizontal / "
                    f"{hit[2]:.1f} m vertical -> yielding: moving away at my altitude")
                if p is not None and abs(p.vz) < 0.3 and \
                        now - self.last_yield.get(hit[0], -1e9) < 30.0:
                    self._yield_vertical(y, me, p, 'is parked on my route')
            elif hit[0] != y['peer'] and y['stage'] == 'clear':
                y['peer'] = hit[0]         # a nearer conflict: get away from that one
            y['clear_since'] = None
        if y is None:
            return False

        p = self.coord.peers.get(y['peer'])
        gone = p is None or now - p.rx_time > self.peer_timeout
        dh = math.hypot(p.north - me[0], p.east - me[1]) if not gone else float('inf')
        dv = abs(p.alt - me[2]) if not gone else float('inf')
        clear_h = 1.25 * self.sep_h

        if gone:
            pass
        elif y['stage'] == 'clear':
            if (not y['no_pass'] and abs(p.vz) < 0.3 and dh < clear_h
                    and now - y['t0'] > self.yield_pass_s):
                self._yield_vertical(y, me, p, f"still in my way after {now - y['t0']:.0f} s")
        elif abs(p.vz) >= 0.5 or (y['stage'] == 'pass' and dv < self.sep_v - 1.0):
            # it started moving vertically: never race it up or down
            y.update(stage='clear', x=self.pos.x, y=self.pos.y, z=self.pos.z, no_pass=True)
            self.get_logger().warn(
                f"SEPARATION: peer {y['peer']} {'climbing' if p.vz > 0 else 'descending'} "
                f"({p.vz:+.1f} m/s, {dv:.1f} m vertically) -> moving away at my altitude")
        elif y['stage'] == 'hold' and abs(-self.pos.z + y['z']) < 0.7 \
                and dv >= self.sep_v - 0.25:
            y['stage'] = 'pass'
            self.get_logger().info(
                f"SEPARATION: vertically clear of peer {y['peer']} ({dv:.1f} m) at "
                f"{-y['z']:.1f} m -> continuing my route at that altitude")

        if not (hit and hit[3]):
            if gone or dh >= clear_h or (y['stage'] == 'clear' and dv >= self.sep_v + 1.0):
                y['clear_since'] = y['clear_since'] or now
                if now - y['clear_since'] >= self.sep_clear_s:
                    self.get_logger().info(
                        f"SEPARATION: peer {y['peer']} clear{' (silent)' if gone else ''} "
                        f"after {now - y['t0']:.0f} s -> back to survey altitude")
                    self.last_yield[y['peer']] = now
                    self.yielding = None
                    return False
            else:
                y['clear_since'] = None
        if now - y['t0'] > 60.0 and not y['warned']:
            y['warned'] = True
            self.get_logger().warn(f"SEPARATION: still yielding to peer {y['peer']} after 60 s")

        if y['stage'] == 'clear':
            if not gone and dh < clear_h:
                side = (0.0, 1.0 if self.me > y['peer'] else -1.0)
                tn, te = sl.escape_point(me[:2], (p.north, p.east), clear_h + 0.5, side,
                                         peer_vel=(p.vn, p.ve))
                y['x'], y['y'] = self.pos.x, self.pos.y
                self.send_sp_limited(tn, te - self.my_east0, y['z'], float('nan'))
            else:
                self.send_sp(y['x'], y['y'], y['z'], float('nan'))
        elif y['stage'] == 'hold':
            self.send_sp(y['x'], y['y'], y['z'], float('nan'))
        elif self.phase == Phase.HOLD:
            self.send_sp(self.hold_xy[0], self.hold_xy[1], y['z'], float('nan'))
        else:
            w = self.wp[self.wp_idx]
            self.send_sp_limited(w['x'], w['y'], y['z'], self.course_yaw(self.wp_idx))
        return True

    def _yield_vertical(self, y, me, p, why):
        """CLEAR -> HOLD: go over or under a peer that is holding its altitude."""
        alt = sl.yield_altitude(me[2], p.alt, self.sep_v, self.sep_climb)
        if abs(alt - p.alt) < self.sep_v:
            y['no_pass'] = True
            self.get_logger().warn(
                f"SEPARATION: peer {y['peer']} {why}, but there is no room to pass "
                f"under it - keeping horizontal separation")
            return
        y.update(stage='hold', x=self.pos.x, y=self.pos.y, z=-alt)
        self.get_logger().warn(
            f"SEPARATION: peer {y['peer']} {why} and holding its altitude -> going to "
            f"{alt:.1f} m to pass {'over' if alt > p.alt else 'under'} it")

    # ---------- swarm: what next, once the current work is done ----------
    def next_work(self, now):
        if not self.takeover_on:
            return self.start_rtl('mission complete')
        if 0 <= self.battery < self.coord.min_batt:
            return self.start_rtl(f'battery {self.battery:.0f}% too low to help peers')
        for band, frm in self.coord.orphans(now, self.mask):
            if self.coord.should_claim(band, now, self.battery):
                return self.claim_band(band, frm, now)
        orphans = self.coord.orphans(now, self.mask)
        pending = self.coord.pending(now, self.mask)
        if orphans or pending:
            key = (tuple(b for b, _ in orphans), tuple(b for b, _ in pending))
            if self.hold_t0 is None:
                self.hold_t0 = now
                self.hold_xy = (self.pos.x, self.pos.y) if self.pos_valid else (0.0, 0.0)
            if key != self.hold_reason:      # log on change, not every tick
                why = ', '.join([f"band {b} unfinished (better-placed peer should take it)"
                                 for b, _ in orphans] +
                                [f"band {b}: owner silent, may still be flying it - "
                                 f"not entering for another {max((t or now) - now, 0):.0f} s"
                                 for b, t in pending])
                self.get_logger().info(f"HOLD: {why}")
                self.hold_reason = key
            if now - self.hold_t0 > self.max_hold_s:
                return self.start_rtl(f'held {self.max_hold_s:.0f} s, nothing to take over')
            self.phase = Phase.HOLD
            return
        self.start_rtl('mission complete')

    def claim_band(self, band, frm, now):
        owner = self.coord.peers.get(band)
        why = ('never heard' if owner is None else
               'returned early' if owner.state in sl.FINISHED_STATES else 'silent past its ETA')
        new = self.takeover_waypoints(band, frm)
        self.wp = self.wp[:self.wp_idx] + new
        self.wp_min_dist = self.wp_min_dist[:self.wp_idx] + [float('inf')] * len(new)
        self.claim = {'band': band, 'from': frm}
        self.takeovers.append(f"band {band} lanes {frm}-{len(self.lanes_y) - 1}")
        self.hold_t0, self.hold_reason = None, ''
        self.phase = Phase.SURVEY
        self.get_logger().warn(
            f"TAKEOVER: band {band} lanes {frm}..{len(self.lanes_y) - 1} "
            f"(drone {band} {why})")

    def start_rtl(self, reason):
        self.end_reason = reason
        self.publish_detecting(False)
        self.yielding = None
        if not self.rtl_on_complete and reason == 'mission complete':
            return self.finish(True, reason='complete (no RTL)')
        self.get_logger().info(f"RTL: {reason}")
        self.engage_rtl()
        self.last_engage_t = self.now_s()
        self.phase = Phase.RTL
        self.phase_t0 = self.now_s()

    def swarm_checks(self, now):
        """Early-return triggers while flying. -> True if one fired."""
        if self.phase not in (Phase.SURVEY, Phase.HOLD):
            return False
        # PX4 took the vehicle (battery failsafe, geofence, ...): stop steering.
        if self.is_armed and not self.is_offboard:
            self.offboard_lost_t = self.offboard_lost_t or now
            if now - self.offboard_lost_t > 2.0:
                self.end_reason = f'PX4 took control (nav_state {self.status.nav_state})'
                self.get_logger().warn(f"EARLY RETURN: {self.end_reason}")
                self.publish_detecting(False)
                self.yielding = None
                self.phase, self.phase_t0 = Phase.RETURN_WAIT, now
                return True
        else:
            self.offboard_lost_t = None
        if self.min_batt >= 0 and 0 <= self.battery < self.min_batt:
            self.get_logger().warn(f"EARLY RETURN: battery {self.battery:.0f}%")
            self.start_rtl(f'battery {self.battery:.0f}% < {self.min_batt:.0f}%')
            return True
        if self.claim is None and self.phase == Phase.SURVEY:
            # A peer decided I was gone and claimed my band - two drones must not
            # share it, and the claimer is already committed.
            for j, p in self.coord.peers.items():
                if p.claimed_band == self.me and self.coord.live(j, now):
                    self.get_logger().warn(f"EARLY RETURN: peer {j} claimed my band")
                    self.start_rtl(f'peer {j} claimed my band')
                    return True
        if self.claim and self.coord.claim_conflict(self.claim['band'], now):
            self.get_logger().warn(
                f"TAKEOVER released: a better-placed peer also claimed band "
                f"{self.claim['band']}")
            self.wp = self.wp[:self.wp_idx]
            self.wp_min_dist = self.wp_min_dist[:self.wp_idx]
            self.takeovers[-1] += ' (released)'
            self.claim = None
            self.next_work(now)
            return True
        return False

    def gate_open(self, w):
        """Swarm gate: flying ALONG a lane (heading for its lane_end), near
        survey altitude (with hysteresis), not yielding. Closed in the turn
        between lanes (banking and yawing - the worst case for geotagging) and
        in transit."""
        if not self.pos_valid or self.yielding or w['kind'] != 'lane_end' \
                or not self.over_area():
            self.gate_alt_ok = False
            return False
        err = abs(-self.pos.z - self.altitude)
        self.gate_alt_ok = err < (2.0 if self.gate_alt_ok else 1.0)
        return self.gate_alt_ok

    def over_area(self):
        """Over the survey area along-track - off the lead-in and run-out,
        where the drone is still settling from the turn or already braking."""
        return self.pos_valid and self.x_min <= self.pos.x <= self.x_max

    def on_waypoint_reached(self, w):
        if w['kind'] != 'lane_end':
            return
        if w['band'] == self.me and self.claim is None:
            self.own_lanes_done = w['lane'] + 1
            self.get_logger().info(
                f"own lane {w['lane']} done ({self.own_lanes_done}/{len(self.lanes_y)})")
        elif self.claim and w['band'] == self.claim['band']:
            self.get_logger().info(f"takeover: band {w['band']} lane {w['lane']} done")

    def on_work_complete(self, now):
        if self.claim:
            self.mask |= 1 << self.claim['band']
            self.get_logger().info(f"TAKEOVER complete: band {self.claim['band']}")
            self.claim = None
        else:
            self.mask |= 1 << self.me
            self.get_logger().info("own band complete")
        self.next_work(now)

    # ---------- main loop ----------
    def tick(self):
        now = self.now_s()
        if self.swarm:
            if now - self.last_hb_t >= self.hb_period:
                self.last_hb_t = now
                self.publish_heartbeat()
                self.log_peer_changes(now)
            if self.phase == Phase.DONE:
                if self.done_at is not None and now >= self.done_at:
                    self.done = True
                return

        if self.phase == Phase.WAIT:
            if now - self.t0 >= self.start_delay:
                self.get_logger().info(f"start delay {self.start_delay:g} s over")
                self.phase = Phase.INIT
            return

        # Stream the offboard heartbeat whenever we intend to stay in offboard.
        if self.phase in (Phase.INIT, Phase.ENGAGE, Phase.SURVEY, Phase.HOLD):
            self.send_ocm()

        if self.phase == Phase.INIT:
            # Straight up at home: no horizontal course exists, so don't
            # command a heading - NaN leaves the drone's yaw alone.
            w = self.wp[0]
            self.send_sp(w['x'], w['y'], w['z'], yaw=self.climb_yaw())
            self.counter += 1
            if self.counter >= 10:      # ~1 s of setpoints before switching
                self.engage_offboard()
                self.arm()
                self.last_engage_t = now
                self.phase = Phase.ENGAGE
                self.phase_t0 = now
            return

        if self.phase == Phase.ENGAGE:
            w = self.wp[0]
            self.send_sp(w['x'], w['y'], w['z'], yaw=self.climb_yaw())
            if self.is_offboard and self.is_armed:
                self.get_logger().info("Offboard engaged + armed -> starting survey")
                self.phase = Phase.SURVEY
                self.phase_t0 = now
            elif now - self.phase_t0 > self.arm_timeout_s:
                self.get_logger().error("offboard/arm not achieved within timeout")
                self.finish(False, reason="arm/offboard timeout")
            elif now - self.last_engage_t > 1.0:
                self.engage_offboard()
                self.arm()
                self.last_engage_t = now
            return

        if self.swarm and (self.swarm_checks(now) or self.separation(now)):
            return

        if self.phase == Phase.HOLD:
            x, y = self.hold_xy
            self.send_sp(x, y, self.z, float('nan'))
            self.publish_detecting(False)
            self.next_work(now)
            return

        if self.phase == Phase.SURVEY:
            w = self.wp[self.wp_idx]
            tx, ty, tz = w['x'], w['y'], w['z']
            # Bearing is taken to the TRUE waypoint, never to the carrot: the
            # carrot converges onto the drone near arrival, where the bearing
            # between them is numerically unstable and can flip 180 degrees.
            self.send_sp_limited(tx, ty, tz, self.course_yaw(self.wp_idx))
            if self.swarm:
                self.publish_detecting(self.gate_open(w))
            else:
                # wp[0] is the climb straight up at home - nothing under us to
                # map, and the projection is garbage while altitude is still
                # changing. reached_alt is LATCHED: without it the gate chatters
                # open/closed every time the drone dips slightly during a lane
                # turn, spamming the log and republishing on every tick.
                if self.pos_valid and -self.pos.z >= 0.8 * self.altitude:
                    self.reached_alt = True
                # Only along a lane - closed in the turn between lanes, where the
                # drone banks and yaws.
                self.publish_detecting(w['kind'] == 'lane_end' and self.reached_alt
                                       and self.over_area())
            if self.pos_valid:
                d = self.dist_to(tx, ty, tz)
                self.wp_min_dist[self.wp_idx] = min(self.wp_min_dist[self.wp_idx], d)
                if d <= self.reach_tol:
                    self.get_logger().info(
                        f"reached wp {self.wp_idx + 1}/{len(self.wp)} "
                        f"({tx:.1f},{ty:.1f},{tz:.1f}) d={d:.2f}m")
                    self.wp_idx += 1
                    if self.swarm:
                        self.on_waypoint_reached(w)
                        if (self.claim is None and 0 <= self.abort_after
                                and self.own_lanes_done == self.abort_after
                                and self.own_lanes_done < len(self.lanes_y)):
                            self.get_logger().warn(
                                f"EARLY RETURN: TEST hook abort_after_lanes={self.abort_after}")
                            return self.start_rtl(
                                f'test: abort after {self.abort_after} lane(s)')
                    if self.wp_idx >= len(self.wp):
                        self.get_logger().info("survey pattern complete")
                        # Close the gate BEFORE RTL: the climb to RTL altitude was
                        # what produced the alt-29.7 m junk in hazard_points.csv.
                        self.publish_detecting(False)
                        if self.swarm:
                            return self.on_work_complete(now)
                        if self.rtl_on_complete:
                            self.engage_rtl()
                            self.last_engage_t = now
                            self.phase = Phase.RTL
                            self.phase_t0 = now
                        else:
                            self.finish(self.check_waypoints(), reason="complete (no RTL)")
            return

        if self.phase == Phase.RTL:
            # Stop streaming offboard; PX4 owns the vehicle now. Confirm RTL engaged.
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                self.get_logger().info("RTL engaged -> waiting for return")
                self.phase = Phase.RETURN_WAIT
                self.phase_t0 = now
            elif now - self.last_engage_t > 1.0:
                self.engage_rtl()
                self.last_engage_t = now
            return

        if self.phase == Phase.RETURN_WAIT:
            self.check_landing_away_from_home()
            if self.pos_valid and self.home_dist() <= self.return_tol:
                self.returned = True
                self.get_logger().info(f"returned home (d={self.home_dist():.2f} m)")
                self.finish(self.check_waypoints() and self.returned, reason="returned")
            elif now - self.phase_t0 > self.return_timeout_s:
                self.get_logger().warn("return timed out")
                self.finish(False, reason="return timeout")
            return

    LANDING_STATES = (VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                      VehicleStatus.NAVIGATION_STATE_DESCEND,
                      VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND)

    def check_landing_away_from_home(self):
        """Over a minefield the drone must only ever land at home. PX4 can still
        decide otherwise (battery EMERGENCY, position loss) - say so loudly,
        with the spot, rather than reporting a plain 'return timeout' later."""
        if (getattr(self, '_landing_flagged', False) or not self.pos_valid
                or self.status.nav_state not in self.LANDING_STATES
                or self.home_dist() <= self.return_tol):
            return
        self._landing_flagged = True
        east0 = self.my_east0 if self.swarm else 0.0
        self.end_reason = 'PX4 LANDING AWAY FROM HOME'
        self.get_logger().error(
            f"PX4 IS LANDING AWAY FROM HOME (nav_state {self.status.nav_state}) at "
            f"N={self.pos.x:.1f} E={self.pos.y + east0:.1f} (shared frame), "
            f"{self.home_dist():.0f} m from home - if that is inside the survey "
            f"area, the aircraft is down in uncleared ground")

    # ---------- verification / shutdown ----------
    def check_waypoints(self):
        return all(d <= self.reach_tol for d in self.wp_min_dist)

    def write_csv(self):
        try:
            os.makedirs(self.csv_dir, exist_ok=True)
            path = os.path.join(self.csv_dir, f"{self.csv_prefix}_{int(time.time())}.csv")
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s', 'x_ned', 'y_ned', 'z_ned'])
                for t, x, y, z in self.track:
                    w.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])
            self.get_logger().info(f"track CSV written: {path} ({len(self.track)} samples)")
            return path
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"CSV write failed: {e}")
            return None

    def finish(self, passed, reason=""):
        if self.phase == Phase.DONE:
            return
        self.phase = Phase.DONE
        self.publish_detecting(False)
        csv_path = self.write_csv() if self.verify else None
        # Only waypoints actually attempted count: after an early return the
        # rest of the band is a peer's job, not a MISS.
        flown = self.wp_min_dist[:max(self.wp_idx, 1)] if self.swarm else self.wp_min_dist
        hit = sum(1 for d in flown if d <= self.reach_tol)
        for i, (wp, d) in enumerate(zip(self.wp, self.wp_min_dist)):
            if self.swarm and i >= self.wp_idx:
                break
            self.get_logger().info(
                f"  wp{i + 1} ({wp['x']:.1f},{wp['y']:.1f},{wp['z']:.1f}) "
                f"closest={d:.2f}m {'OK' if d <= self.reach_tol else 'MISS'}")
        if self.swarm:
            total = len(self.lanes_y)
            complete = self.mask >> self.me & 1
            verdict = ('FAIL' if not self.returned else
                       'PASS' if complete and hit == len(flown) else 'PARTIAL')
            self.get_logger().info(
                f"VERIFY {verdict} | drone {self.me} | own lanes {self.own_lanes_done}/{total} "
                f"| took over: {', '.join(self.takeovers) or 'none'} | waypoints {hit}/"
                f"{len(flown)} | returned={self.returned} | why={self.end_reason or reason} "
                f"| csv={csv_path or 'n/a'}")
            # Keep heartbeating LANDED/FAILED for a moment so peers record the
            # final state instead of just seeing the drone go quiet.
            self.done_at = self.now_s() + 2.0
            return
        wp_ok = self.check_waypoints()
        verdict = 'PASS' if passed else 'FAIL'
        self.get_logger().info(
            f"VERIFY {verdict} | waypoints {hit}/{len(self.wp)} "
            f"({'OK' if wp_ok else 'INCOMPLETE'}) | returned={self.returned} | "
            f"reason={reason} | csv={csv_path or 'n/a'}")
        try:
            self.timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        # Do NOT call rclpy.shutdown() here. finish() runs inside the tick()
        # timer callback, and in Humble rclpy.shutdown() shuts down the global
        # executor, which blocks until in-flight callbacks finish - i.e. until
        # this one returns. It deadlocked every run after VERIFY: the node never
        # exited, `ros2 launch` never ended, and SIGTERM could not stop it
        # either. main() sees this flag and shuts down outside the callback.
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = SurveyNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
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
