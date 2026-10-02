"""hazard_registry.py - one drone's view of the swarm's hazard list.

Each onboard detector keeps one of these. It holds its own detections AND the
ones peers broadcast on /swarm/hazards, so a drone never logs an object a peer
already found (the band-boundary case) and no ground station is needed to
de-duplicate.

Canonical rule, applied identically on every drone so all lists converge:
two reports inside min_sep_m are the same object, and the one that stands is
the EARLIER stamp, ties going to the lower origin drone. If a peer's report
beats one we already hold (both drones saw it within the radio latency), ours
is marked superseded rather than deleted - the CSV is append-only.

No ROS imports: unit-tested in test/test_hazard_registry.py.
"""

import math
from dataclasses import dataclass


@dataclass
class Hazard:
    hazard_id: str
    origin: int
    seq: int
    stamp: float
    north: float
    east: float
    alt: float
    cls: str
    conf: float

    @property
    def key(self):
        return (self.stamp, self.origin, self.seq)


class HazardRegistry:
    def __init__(self, me, min_sep_m):
        self.me = me
        self.min_sep = min_sep_m
        self.items = {}       # hazard_id -> Hazard, current canonical set
        self.seen_ids = set()  # every id ever handled, incl. superseded/rejected
        self.seq = {}         # per-origin counter

    def nearest(self, north, east):
        best = None
        for h in self.items.values():
            d = math.hypot(h.north - north, h.east - east)
            if d < self.min_sep and (best is None or d < best[1]):
                best = (h, d)
        return best

    def add_own(self, north, east, alt, cls, conf, stamp, origin=None):
        """-> the new Hazard, or None if it duplicates a known one.
        `origin` defaults to this drone; a ground-side detector serving several
        cameras passes the camera's drone index instead."""
        if self.nearest(north, east):
            return None
        origin = self.me if origin is None else origin
        seq = self.seq[origin] = self.seq.get(origin, 0) + 1
        h = Hazard(f"d{origin}-{seq}", origin, seq, stamp,
                   north, east, alt, cls, conf)
        self.items[h.hazard_id] = h
        self.seen_ids.add(h.hazard_id)
        return h

    def add_peer(self, h):
        """-> (outcome, superseded) where outcome is
        'repeat'  already handled this id (re-broadcast / history replay)
        'new'     a new object, added
        'dup'     same object as one we hold, ours is canonical: ignored
        'replaces' same object, the peer's is canonical: ours superseded
        """
        if h.hazard_id in self.seen_ids:
            return ('repeat', None)
        self.seen_ids.add(h.hazard_id)
        near = self.nearest(h.north, h.east)
        if near is None:
            self.items[h.hazard_id] = h
            return ('new', None)
        held = near[0]
        if held.key <= h.key:
            return ('dup', None)
        del self.items[held.hazard_id]
        self.items[h.hazard_id] = h
        return ('replaces', held)
