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

KNOWN FOLLOW-UP (left intentionally - do NOT fix unless asked):
  lane_spacing is a hardcoded default. It should be derived from the downward
  camera footprint: 2*altitude*tan(HFOV/2)*(1-sidelap) once the camera is wired.
"""

import csv
import math
import os
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class Phase(Enum):
    INIT = 0          # stream setpoints, then request offboard + arm
    ENGAGE = 1        # wait for offboard + armed confirmation
    SURVEY = 2        # fly the lawnmower waypoints
    RTL = 3           # command RTL, wait for it to engage
    RETURN_WAIT = 4   # wait until back near home
    DONE = 5          # verified, CSV written, shutting down


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
        self.declare_parameter('sidelap', 0.3)         # fraction of overlap between lanes
        self.declare_parameter('hfov_rad', 1.74)       # mono_cam SDF; must match the detector
        self.declare_parameter('reach_tol', 1.5)       # waypoint reach tolerance (m)
        self.declare_parameter('return_tol', 3.0)      # home-return tolerance (m)
        self.declare_parameter('rtl_on_complete', True)
        self.declare_parameter('verify', True)
        self.declare_parameter('csv_dir', '~/maps')
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

        g = self.get_parameter
        self.x_min = float(g('x_min').value)
        self.x_max = float(g('x_max').value)
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
        self.arm_timeout_s = float(g('arm_timeout_s').value)
        self.return_timeout_s = float(g('return_timeout_s').value)
        self.lookahead = float(g('lookahead_m').value)
        self.detect_topic = str(g('detect_topic').value)

        self.z = -abs(self.altitude)  # NED down: negative = up

        # Derive lane spacing from what the camera can actually see, rather than
        # guessing. A nadir camera at height h sees a strip 2*h*tan(HFOV/2) wide;
        # stepping by that much times (1 - sidelap) guarantees the requested
        # overlap between adjacent lanes. Too wide leaves unphotographed gaps
        # between lanes - the survey "passes" while missing ground.
        self.footprint_w = 2.0 * self.altitude * math.tan(self.hfov / 2.0)
        if self.lane_spacing <= 0.0:
            self.lane_spacing = max(self.footprint_w * (1.0 - self.sidelap), 0.5)
            self.get_logger().info(
                f"lane_spacing derived: footprint {self.footprint_w:.2f} m at "
                f"{self.altitude:.1f} m altitude, {self.sidelap:.0%} sidelap "
                f"-> {self.lane_spacing:.2f} m")
        elif self.lane_spacing > self.footprint_w:
            self.get_logger().warn(
                f"lane_spacing {self.lane_spacing:.1f} m EXCEEDS the camera "
                f"footprint {self.footprint_w:.1f} m at {self.altitude:.1f} m "
                f"altitude -> unphotographed gaps between lanes. "
                f"Use lane_spacing:=0.0 to derive it.")

        # ---- QoS: matches px4_ros_com examples (proven with this PX4/px4_msgs) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- publishers ----
        self.pub_ocm = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

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
        for t in ('/fmu/out/vehicle_local_position', '/fmu/out/vehicle_local_position_v1'):
            self.create_subscription(VehicleLocalPosition, t, self.on_pos, qos)
        for t in ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v4'):
            self.create_subscription(VehicleStatus, t, self.on_status, qos)

        # ---- state ----
        self.pos = VehicleLocalPosition()
        self.pos_valid = False
        self.status = VehicleStatus()
        self.phase = Phase.INIT
        self.counter = 0
        self.last_engage_t = 0.0
        self.wp = self.build_waypoints()
        self.wp_idx = 0
        self.wp_min_dist = [float('inf')] * len(self.wp)
        self.track = []          # (t_s, x, y, z)
        self.t0 = self.now_s()
        self.phase_t0 = self.t0
        self.returned = False
        self.detecting = None            # last value published on detect_topic
        self.reached_alt = False         # latched once we first reach survey altitude

        self.get_logger().info(
            f"Survey x[{self.x_min},{self.x_max}] y[{self.y_min},{self.y_max}] "
            f"alt={self.altitude}m lane={self.lane_spacing}m -> {len(self.wp)} waypoints; "
            f"RTL={'on' if self.rtl_on_complete else 'off'}, verify={self.verify}, "
            f"lookahead={self.lookahead}m")
        self.publish_detecting(False)

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    # ---------- time helpers ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ---------- waypoint generation ----------
    def build_waypoints(self):
        step = abs(self.lane_spacing) if self.lane_spacing else (self.y_max - self.y_min)
        step = step if step > 1e-6 else 1.0
        ys = []
        y = self.y_min
        while y < self.y_max - 1e-6:
            ys.append(y)
            y += step
        ys.append(self.y_max)  # always cover the far edge

        wps = [(0.0, 0.0, self.z)]  # climb straight up at home first
        for i, yy in enumerate(ys):
            if i % 2 == 0:
                wps.append((self.x_min, yy, self.z))
                wps.append((self.x_max, yy, self.z))
            else:
                wps.append((self.x_max, yy, self.z))
                wps.append((self.x_min, yy, self.z))
        return wps

    # ---------- subscriptions ----------
    def on_pos(self, msg):
        self.pos = msg
        self.pos_valid = bool(msg.xy_valid and msg.z_valid)
        if self.pos_valid and self.phase not in (Phase.INIT, Phase.DONE):
            self.track.append((self.now_s() - self.t0, msg.x, msg.y, msg.z))

    def on_status(self, msg):
        self.status = msg

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

    def send_sp(self, x, y, z, yaw=0.0):
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.velocity = [float('nan')] * 3
        m.acceleration = [float('nan')] * 3
        m.yaw = float(yaw)
        m.timestamp = self.us()
        self.pub_sp.publish(m)

    def send_sp_limited(self, x, y, z, yaw=0.0):
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
        m.target_system = 1
        m.target_component = 1
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

    # ---------- main loop ----------
    def tick(self):
        # Stream the offboard heartbeat whenever we intend to stay in offboard.
        if self.phase in (Phase.INIT, Phase.ENGAGE, Phase.SURVEY):
            self.send_ocm()

        if self.phase == Phase.INIT:
            self.send_sp(*self.wp[0])
            self.counter += 1
            if self.counter >= 10:      # ~1 s of setpoints before switching
                self.engage_offboard()
                self.arm()
                self.last_engage_t = self.now_s()
                self.phase = Phase.ENGAGE
                self.phase_t0 = self.now_s()
            return

        if self.phase == Phase.ENGAGE:
            self.send_sp(*self.wp[0])
            if self.is_offboard and self.is_armed:
                self.get_logger().info("Offboard engaged + armed -> starting survey")
                self.phase = Phase.SURVEY
                self.phase_t0 = self.now_s()
            elif self.now_s() - self.phase_t0 > self.arm_timeout_s:
                self.get_logger().error("offboard/arm not achieved within timeout")
                self.finish(False, reason="arm/offboard timeout")
            elif self.now_s() - self.last_engage_t > 1.0:
                self.engage_offboard()
                self.arm()
                self.last_engage_t = self.now_s()
            return

        if self.phase == Phase.SURVEY:
            tx, ty, tz = self.wp[self.wp_idx]
            self.send_sp_limited(tx, ty, tz)
            # wp[0] is the climb straight up at home - nothing under us to map,
            # and the projection is garbage while altitude is still changing.
            # reached_alt is LATCHED: without it the gate chatters open/closed
            # every time the drone dips slightly during a lane turn, spamming
            # the log and republishing on every tick.
            if self.pos_valid and -self.pos.z >= 0.8 * self.altitude:
                self.reached_alt = True
            self.publish_detecting(self.wp_idx >= 1 and self.reached_alt)
            if self.pos_valid:
                d = self.dist_to(tx, ty, tz)
                self.wp_min_dist[self.wp_idx] = min(self.wp_min_dist[self.wp_idx], d)
                if d <= self.reach_tol:
                    self.get_logger().info(
                        f"reached wp {self.wp_idx + 1}/{len(self.wp)} "
                        f"({tx:.1f},{ty:.1f},{tz:.1f}) d={d:.2f}m")
                    self.wp_idx += 1
                    if self.wp_idx >= len(self.wp):
                        self.get_logger().info("survey pattern complete")
                        # Close the gate BEFORE RTL: the climb to RTL altitude was
                        # what produced the alt-29.7 m junk in hazard_points.csv.
                        self.publish_detecting(False)
                        if self.rtl_on_complete:
                            self.engage_rtl()
                            self.last_engage_t = self.now_s()
                            self.phase = Phase.RTL
                            self.phase_t0 = self.now_s()
                        else:
                            self.finish(self.check_waypoints(), reason="complete (no RTL)")
            return

        if self.phase == Phase.RTL:
            # Stop streaming offboard; PX4 owns the vehicle now. Confirm RTL engaged.
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                self.get_logger().info("RTL engaged -> waiting for return")
                self.phase = Phase.RETURN_WAIT
                self.phase_t0 = self.now_s()
            elif self.now_s() - self.last_engage_t > 1.0:
                self.engage_rtl()
                self.last_engage_t = self.now_s()
            return

        if self.phase == Phase.RETURN_WAIT:
            if self.pos_valid and self.home_dist() <= self.return_tol:
                self.returned = True
                self.get_logger().info(f"returned home (d={self.home_dist():.2f} m)")
                self.finish(self.check_waypoints() and self.returned, reason="returned")
            elif self.now_s() - self.phase_t0 > self.return_timeout_s:
                self.get_logger().warn("return timed out")
                self.finish(False, reason="return timeout")
            return

    # ---------- verification / shutdown ----------
    def check_waypoints(self):
        return all(d <= self.reach_tol for d in self.wp_min_dist)

    def write_csv(self):
        try:
            os.makedirs(self.csv_dir, exist_ok=True)
            path = os.path.join(self.csv_dir, f"survey_track_{int(time.time())}.csv")
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
        hit = sum(1 for d in self.wp_min_dist if d <= self.reach_tol)
        for i, (wp, d) in enumerate(zip(self.wp, self.wp_min_dist)):
            self.get_logger().info(
                f"  wp{i + 1} ({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f}) "
                f"closest={d:.2f}m {'OK' if d <= self.reach_tol else 'MISS'}")
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
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SurveyNode()
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
