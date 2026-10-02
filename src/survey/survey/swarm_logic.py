"""swarm_logic.py - the decision-making half of the swarm, with no ROS in it.

Everything here is a pure function or a plain class driven by an explicit
`now`, so it is unit-tested (test/test_swarm_logic.py) without a simulator.
survey_node.py owns the ROS side: it feeds heartbeats in and turns the
decisions into setpoints.

Frames. Band b belongs to drone b. Drone i's local NED origin is its spawn
point, which is the corner of its own band: shared-frame east = local east +
band_origin_east(i). North is the same in every frame.
"""

import math
from dataclasses import dataclass

# Heartbeat states - must match swarm_msgs/DroneHeartbeat.msg
BOOT, ENGAGING, SURVEY, TAKEOVER, HOLD, YIELD, RETURNING, LANDED, FAILED = range(9)
STATE_NAMES = ['BOOT', 'ENGAGING', 'SURVEY', 'TAKEOVER', 'HOLD', 'YIELD',
               'RETURNING', 'LANDED', 'FAILED']
# Airborne and steered by its own onboard node (so it can be asked to yield).
OFFBOARD_STATES = {ENGAGING, SURVEY, TAKEOVER, HOLD, YIELD}
# Will not fly any more of its band.
FINISHED_STATES = {RETURNING, LANDED, FAILED}


# --------------------------------------------------------------- geometry
def band_height(num_drones, y_min, y_max):
    return (y_max - y_min) / num_drones


def band_origin_east(band, num_drones, y_min, y_max):
    """Shared-frame east of band `band`'s near edge = its owner's spawn point."""
    return y_min + band * band_height(num_drones, y_min, y_max)


def lane_offsets(width, spacing):
    """East offsets of the lanes inside one band, same rule survey_node has
    always used: step by `spacing`, and always fly the far edge."""
    step = spacing if spacing > 1e-6 else width
    step = step if step > 1e-6 else 1.0
    ys, y = [], 0.0
    while y < width - 1e-6:
        ys.append(y)
        y += step
    ys.append(width)
    return ys


def lane_waypoints(lanes_y, x_min, x_max, first=0, reverse_first=False):
    """Boustrophedon over lanes[first:], in the band's local frame.

    Returns [(north, east, lane_index, is_lane_end)]. In the base pattern even
    lanes run x_min -> x_max; reverse_first flips the direction of the first
    lane flown (a takeover enters from whichever end is nearer)."""
    wps = []
    forward = (first % 2 == 0) != bool(reverse_first)
    for k in range(first, len(lanes_y)):
        a, b = (x_min, x_max) if forward else (x_max, x_min)
        wps.append((a, lanes_y[k], k, False))
        wps.append((b, lanes_y[k], k, True))
        forward = not forward
    return wps


def path_eta(start, points, speed, turn_penalty_s=2.3):
    """Seconds to fly start -> points[0] -> ... at `speed`, plus a fixed cost
    per waypoint (the 2.3 s fitted in experiments/coverage_model.py)."""
    if speed <= 0:
        return -1.0
    d, (px, py) = 0.0, start
    for (x, y) in points:
        d += math.hypot(x - px, y - py)
        px, py = x, y
    return d / speed + turn_penalty_s * len(points)


# ------------------------------------------------------------- separation
def separation_conflict(me_id, me, peers, sep_h, sep_v, max_age_s):
    """Closest peer inside the separation cylinder, and whether *I* give way.

    me    : (north, east, alt) in the shared frame
    peers : iterable of (id, state, north, east, alt, age_s)
    Returns (peer_id, dh, dv, i_yield, peer_alt) or None.

    Right of way: among drones steered by their own onboard node, the LOWER
    id keeps going and the higher id yields - deterministic, so two drones
    never both stop and wait for each other. A peer under PX4 control (RTL,
    land) cannot be asked to move, so whoever is still in offboard yields.
    """
    best = None
    for pid, pstate, pn, pe, palt, age in peers:
        if pid == me_id or age > max_age_s:
            continue
        dh = math.hypot(pn - me[0], pe - me[1])
        dv = abs(palt - me[2])
        if dh >= sep_h or dv >= sep_v:
            continue
        i_yield = pstate not in OFFBOARD_STATES or pid < me_id
        if pstate == YIELD and pid > me_id:
            i_yield = False          # it is already giving way to me
        if best is None or dh < best[1]:
            best = (pid, dh, dv, i_yield, palt)
    return best


def yield_altitude(my_alt, peer_alt, sep_v, climb_m, min_alt=3.0):
    """Where the yielding drone goes vertically while it holds position.

    Never climb THROUGH the other drone: if I am clearly below it, stay below
    (descending if needed to open sep_v); otherwise go above it."""
    if my_alt < peer_alt - 1.0:
        return max(min(my_alt, peer_alt - sep_v), min_alt)
    return max(my_alt, peer_alt) + climb_m


# ---------------------------------------------------------------- takeover
@dataclass
class PeerInfo:
    drone_id: int
    state: int
    north: float
    east: float
    alt: float
    own_lanes_done: int
    own_lanes_total: int
    claimed_band: int
    claimed_from_lane: int
    bands_done_mask: int
    eta_s: float
    battery_pct: float
    rx_time: float


class SwarmCoordinator:
    """Decides which unfinished bands exist and whether this drone should take
    one over. Bands are owned by the drone with the same index.

    A band is ORPHANED when its owner
      * announced it is leaving (RETURNING/LANDED/FAILED) with lanes left, or
      * went silent and its projected finish time has passed, or
      * was never heard from at all within startup_grace_s.
    A silent drone may still be flying (its radio died, not its Pi - onboard
    autonomy means it carries on), so a neighbour must not fly into its band
    before it would have finished anyway: deadline = last heartbeat + its own
    ETA + deadline_margin_s.
    """

    def __init__(self, me, num_drones, lanes_per_band, peer_timeout_s=3.0,
                 startup_grace_s=45.0, deadline_margin_s=15.0,
                 min_takeover_battery=30.0, claim_wait_s=20.0):
        self.me = me
        self.n = num_drones
        self.lanes_per_band = lanes_per_band
        self.peer_timeout_s = peer_timeout_s
        self.startup_grace_s = startup_grace_s
        self.margin = deadline_margin_s
        self.min_batt = min_takeover_battery
        self.claim_wait_s = claim_wait_s
        self.peers = {}
        self.t_start = None

    # ---- input
    def update(self, p: PeerInfo):
        if p.drone_id != self.me and 0 <= p.drone_id < self.n:
            self.peers[p.drone_id] = p

    def start(self, now):
        if self.t_start is None:
            self.t_start = now

    # ---- queries
    def live(self, j, now):
        p = self.peers.get(j)
        return p is not None and now - p.rx_time <= self.peer_timeout_s

    def _deadline(self, p):
        return p.rx_time + max(p.eta_s, 0.0) + self.margin

    def _batt_ok(self, pct):
        return pct < 0 or pct >= self.min_batt

    def covered_mask(self, my_mask):
        m = my_mask
        for p in self.peers.values():
            m |= p.bands_done_mask
        return m

    def band_status(self, band, now, my_mask):
        """-> (status, from_lane, available_at). status is one of
        'mine', 'covered', 'flying', 'claimed', 'pending', 'orphan'."""
        if band == self.me:
            return ('mine', 0, None)
        if self.covered_mask(my_mask) >> band & 1:
            return ('covered', 0, None)
        # someone already taking it over?
        for p in self.peers.values():
            if p.claimed_band == band:
                if now - p.rx_time <= self.peer_timeout_s:
                    return ('claimed', p.claimed_from_lane, None)
                if now < self._deadline(p):
                    return ('pending', p.claimed_from_lane, self._deadline(p))
        owner = self.peers.get(band)
        if owner is None:
            if self.t_start is None:
                return ('pending', 0, None)
            t = self.t_start + self.startup_grace_s
            return ('orphan', 0, None) if now >= t else ('pending', 0, t)
        done = owner.own_lanes_done
        if done >= owner.own_lanes_total:
            return ('covered', 0, None)
        if owner.state in FINISHED_STATES:
            return ('orphan', done, None)          # said it is leaving
        if now - owner.rx_time <= self.peer_timeout_s:
            return ('flying', done, None)          # alive and working on it
        t = self._deadline(owner)                  # silent: maybe still flying
        return ('orphan', done, None) if now >= t else ('pending', done, t)

    def orphans(self, now, my_mask):
        out = []
        for b in range(self.n):
            st, frm, _ = self.band_status(b, now, my_mask)
            if st == 'orphan':
                out.append((b, frm))
        out.sort(key=lambda bf: (abs(bf[0] - self.me), bf[0]))
        return out

    def pending(self, now, my_mask):
        return [(b, t) for b in range(self.n)
                for st, _, t in [self.band_status(b, now, my_mask)] if st == 'pending']

    def _key(self, drone, band):
        return (abs(drone - band), drone)

    def should_claim(self, band, now, my_battery):
        """True if I am the best-placed drone to take `band` now.

        Candidates are live peers that are idle (HOLD) or about to be (still
        surveying but ETA <= claim_wait_s) with battery to spare. Waiting for an
        almost-finished better neighbour is what keeps the takeover with the
        NEIGHBOUR instead of whichever drone happened to finish first."""
        if not self._batt_ok(my_battery):
            return False
        mine = self._key(self.me, band)
        for j, p in self.peers.items():
            if not self.live(j, now) or not self._batt_ok(p.battery_pct):
                continue
            idle = p.state == HOLD and p.claimed_band < 0
            soon = (p.state == SURVEY and 0 <= p.eta_s <= self.claim_wait_s
                    and p.claimed_band < 0)
            if (idle or soon) and self._key(j, band) < mine:
                return False
        return True

    def claim_conflict(self, band, now):
        """A better-placed live peer claimed the same band -> I release mine."""
        mine = self._key(self.me, band)
        return any(p.claimed_band == band and self.live(j, now)
                   and self._key(j, band) < mine
                   for j, p in self.peers.items())
