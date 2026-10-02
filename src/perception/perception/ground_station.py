#!/usr/bin/env python3
"""
ground_station.py - OPTIONAL passive monitor for the onboard swarm.

Listens to the same peer topics the drones use (/swarm/heartbeat,
/swarm/hazards) and prints a status table plus every state change, and keeps
a merged hazard log in the run directory. It publishes NOTHING: no drone
subscribes to anything it could send, so killing it mid-flight changes
nothing for the mission - which is the point of onboard autonomy, and how the
swarm test proves it.

    ros2 run perception ground_station --ros-args -p run_dir:=~/maps/swarm_<stamp>
"""

import csv
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from swarm_msgs.msg import DroneHeartbeat, HazardReport

from perception.hazard_registry import Hazard, HazardRegistry

STATES = ['BOOT', 'ENGAGING', 'SURVEY', 'TAKEOVER', 'HOLD', 'YIELD',
          'RETURNING', 'LANDED', 'FAILED']


class GroundStation(Node):
    def __init__(self):
        super().__init__('ground_station')
        self.declare_parameter('run_dir', '~/maps')
        self.declare_parameter('heartbeat_topic', '/swarm/heartbeat')
        self.declare_parameter('hazard_topic', '/swarm/hazards')
        self.declare_parameter('min_sep_m', 1.5)
        self.declare_parameter('table_period_s', 5.0)
        self.declare_parameter('silent_after_s', 3.0)
        g = self.get_parameter
        self.run_dir = os.path.expanduser(str(g('run_dir').value))
        self.silent_after = float(g('silent_after_s').value)
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_path = os.path.join(self.run_dir, 'ground_hazards.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(['t_s', 'x_ned_north', 'y_ned_east', 'class', 'conf',
                                    'alt_m', 'drone', 'hazard_id', 'status', 'logged_by',
                                    'version', 'n_sightings', 'spread_m'])

        self.registry = HazardRegistry(me=255, min_sep_m=float(g('min_sep_m').value))
        self.hb = {}         # id -> (msg, rx_wall)
        self.prev = {}       # id -> (state, claimed_band, silent)

        hb_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        hz_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(DroneHeartbeat, str(g('heartbeat_topic').value),
                                 self.on_hb, hb_qos)
        self.create_subscription(HazardReport, str(g('hazard_topic').value),
                                 self.on_hazard, hz_qos)
        self.create_timer(1.0, self.watch)
        self.create_timer(max(1.0, float(g('table_period_s').value)), self.table)
        self.get_logger().info(f"ground station up (passive) -> {self.csv_path}")

    def on_hb(self, m):
        self.hb[m.drone_id] = (m, time.time())
        st = (m.state, m.claimed_band, False)
        old = self.prev.get(m.drone_id)
        if old is None or old[:2] != st[:2] or old[2]:
            what = STATES[m.state]
            if m.state == 6 and m.own_lanes_done < m.own_lanes_total:
                what += f" EARLY ({m.own_lanes_done}/{m.own_lanes_total} lanes)"
            if m.claimed_band >= 0:
                what += f", taking over band {m.claimed_band} from lane {m.claimed_from_lane}"
            self.get_logger().info(f"drone {m.drone_id}: {what}"
                                   + (" (back on the air)" if old and old[2] else ""))
        self.prev[m.drone_id] = st

    def watch(self):
        now = time.time()
        for i, (m, rx) in self.hb.items():
            silent = now - rx > self.silent_after
            if silent and not self.prev[i][2] and m.state not in (7, 8):
                self.get_logger().warn(f"drone {i}: SILENT (last {STATES[m.state]}, "
                                       f"lanes {m.own_lanes_done}/{m.own_lanes_total})")
            self.prev[i] = self.prev[i][:2] + (silent,)

    def on_hazard(self, m):
        h = Hazard(m.hazard_id, m.origin_drone, m.seq, m.stamp.sec + m.stamp.nanosec * 1e-9,
                   m.north, m.east, m.alt, m.cls, m.conf,
                   version=m.version, n=m.n_sightings, spread=m.spread_m)
        outcome, sup = self.registry.add_peer(h)
        rows = []
        if outcome == 'new':
            rows = [(h, 'peer')]
        elif outcome == 'update':
            rows = [(self.registry.items[h.hazard_id], 'peer_update')]
        elif outcome == 'replaces':
            rows = [(sup, 'superseded'), (h, 'peer')]
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            for x, status in rows:
                w.writerow([f"{x.stamp:.1f}", f"{x.north:.2f}", f"{x.east:.2f}", x.cls,
                            f"{x.conf:.2f}", f"{x.alt:.1f}", str(x.origin), x.hazard_id,
                            status, 'ground', str(x.version), str(x.n), f"{x.spread:.2f}"])
        if outcome in ('new', 'replaces', 'dup'):
            self.get_logger().info(
                f"hazard {h.hazard_id} from drone {h.origin}: {h.cls} {h.conf:.2f} "
                f"at N={h.north:.1f} E={h.east:.1f} [{outcome}] - map has "
                f"{len(self.registry.items)}")

    def table(self):
        if not self.hb:
            return
        now = time.time()
        lines = ["drone state      band/lane lanes  N      E      alt   batt  age"]
        for i in sorted(self.hb):
            m, rx = self.hb[i]
            lines.append(
                f"  {i}   {STATES[m.state]:<10} {m.band:>3}/{m.lane:<4} "
                f"{m.own_lanes_done}/{m.own_lanes_total}   {m.north:5.1f}  {m.east:5.1f}  "
                f"{m.alt:4.1f}  {m.battery_pct:4.0f}% {now - rx:4.1f}s")
        self.get_logger().info("swarm status\n" + "\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = GroundStation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
