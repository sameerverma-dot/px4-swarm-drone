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
    """East offsets of the lanes inside one band [0, width]: STRIP-CENTRED.

    The band is cut into n = ceil(width / spacing) equal strips and a lane runs
    down the middle of each. Each strip is <= spacing = footprint*(1-sidelap)
    wide, so the requested sidelap holds between lanes AND across a band
    boundary (the neighbour's band follows the same rule). The old rule put a
    lane ON each band edge - the shared boundary was flown twice, by drone i's
    last lane and drone i+1's first, and half of every edge lane's footprint
    landed outside the band. For a 30 m band at 10 m altitude this is 2 lanes
    (7.5, 22.5) instead of 3 (0, 16.6, 30)."""
    if width <= 1e-6:
        return [0.0]
    n = max(1, math.ceil(width / spacing - 1e-9)) if spacing > 1e-6 else 1
    w = width / n
    return [(k + 0.5) * w for k in range(n)]


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
WORK_STATES = {ENGAGING, SURVEY, TAKEOVER}


def _right_of_way(state, drone_id):
    """Lower tuple keeps going. A drone doing its job beats an idle one
    hovering in HOLD (cheap for it to move), which beats one already giving
    way; ties go to the lower id. Deterministic, so two drones never both stop
    and wait for each other - and an idle drone parked in someone's path moves
    instead of making the working drone wait out its whole hold."""
    rank = 0 if state in WORK_STATES else 1 if state == HOLD else 2
    return (rank, drone_id)


def separation_conflict(me_id, me_state, me, peers, sep_h, sep_v, max_age_s):
    """Closest peer inside the separation cylinder, and whether *I* give way.

    me_state : my UNDERLYING state (what I am doing, not YIELD), so a working
               drone that is momentarily yielding keeps its priority
    me       : (north, east, alt) in the shared frame
    peers    : iterable of (id, state, north, east, alt, age_s)
    Returns (peer_id, dh, dv, i_yield, peer_alt) or None.

    A peer not steered by its own onboard node (PX4 RTL/land, or on the ground)
    cannot be asked to move, so I give way to it. Otherwise _right_of_way().
    """
    best = None
    for pid, pstate, pn, pe, palt, age in peers:
        if pid == me_id or age > max_age_s:
            continue
        dh = math.hypot(pn - me[0], pe - me[1])
        dv = abs(palt - me[2])
        if dh >= sep_h or dv >= sep_v:
            continue
        if pstate not in OFFBOARD_STATES:
            i_yield = True
        else:
            i_yield = _right_of_way(pstate, pid) < _right_of_way(me_state, me_id)
        if best is None or dh < best[1]:
            best = (pid, dh, dv, i_yield, palt)
    return best


def escape_point(me, peer, dist, fallback=(0.0, 1.0), peer_vel=(0.0, 0.0)):
    """(north, east) where a yielding drone goes to open horizontal separation
    without changing altitude, on the side it is already on.

    Peer hovering: `dist` straight away from it. Peer moving (> 0.5 m/s): `dist`
    off its LINE of travel, keeping my position along that line - backing away
    from a drone flying straight at me only keeps me in its path.
    `fallback` (a unit vector) breaks the tie when I am exactly on it/its line."""
    dn, de = me[0] - peer[0], me[1] - peer[1]
    sp = math.hypot(*peer_vel)
    if sp > 0.5:
        tn, te = peer_vel[0] / sp, peer_vel[1] / sp
        along = dn * tn + de * te
        pn, pe = dn - along * tn, de - along * te          # my offset off its line
        d = math.hypot(pn, pe)
        if d > 0.3:
            un, ue = pn / d, pe / d
        else:                                              # dead ahead: step aside
            un, ue = (-te, tn) if (-te * fallback[0] + tn * fallback[1]) >= 0 else (te, -tn)
        return peer[0] + along * tn + un * dist, peer[1] + along * te + ue * dist
    d = math.hypot(dn, de)
    un, ue = (dn / d, de / d) if d > 0.3 else fallback
    return peer[0] + un * dist, peer[1] + ue * dist


def yield_altitude(my_alt, peer_alt, sep_v, climb_m, min_alt=3.0, margin=1.0):
    """Where a yielding drone goes vertically to pass over or under a peer that
    is holding its altitude (see survey_node.separation).

    Never climb THROUGH the other drone: if I am clearly below it, stay below
    (descending if needed to open sep_v); otherwise go above it. The target
    clears sep_v by `margin`: parked at exactly sep_v, the peer's own altitude
    wobble kept dropping the yield back from pass to hold (2 Oct, five times in
    0.5 s). Below can be capped by min_alt short of sep_v; the drone then stays
    in hold until the peer moves away."""
    if my_alt < peer_alt - 1.0:
        return max(min(my_alt, peer_alt - sep_v - margin), min_alt)
    return max(my_alt, peer_alt) + max(climb_m, sep_v + margin)


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
    vz: float = 0.0        # climb rate (m/s, up +), from successive heartbeats
    vn: float = 0.0        # ground velocity (m/s), same source
    ve: float = 0.0


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
            prev = self.peers.get(p.drone_id)
            if prev is not None:
                dt = p.rx_time - prev.rx_time
                for k, pos in (('vz', 'alt'), ('vn', 'north'), ('ve', 'east')):
                    v = getattr(prev, k)
                    if dt > 0.05:     # smoothed: heartbeats are 4 Hz with jitter
                        v = 0.5 * v + 0.5 * (getattr(p, pos) - getattr(prev, pos)) / dt
                    setattr(p, k, v)
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
