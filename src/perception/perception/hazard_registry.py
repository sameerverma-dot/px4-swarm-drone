"""hazard_registry.py - one drone's view of the swarm's hazard list.

Each onboard detector keeps one of these. It holds its own detections AND the
ones peers broadcast on /swarm/hazards, so a drone never logs an object a peer
already found (the band-boundary case) and no ground station is needed to
de-duplicate.

Association, per frame. Every detection in a frame is matched to the NEAREST
known hazard within min_sep_m, one-to-one: a hazard can absorb at most one
detection per frame. Two objects seen in the same frame are therefore not
merged (mines can sit 1-3 m apart); a detection with no free hazard in range
becomes a new one.

The one exception is box_merge_m. YOLO often draws two boxes on ONE object
(whole body and part of it), which land 0.2-0.7 m apart on the ground (2 Oct
calibration, tools/analyse_sightings.py) - without this they became two
hazards. Within a frame, a detection closer than box_merge_m to a more
confident one is that object's duplicate box ('box_dup'). Objects closer
together than box_merge_m (default 1.0 m) are therefore one hazard.

Refinement. A merged detection refines the hazard only if this drone owns it:
the position is the weighted mean of the owner's sightings, so an early
edge-of-image fix is improved by the better views that follow. The weight is
the sighting's own (the detector passes confidence x cos^2 of the off-nadir
angle: oblique views are less accurate and can box two objects as one), else
its confidence. The id
and first-sighting stamp never change. Peers adopt the owner's highest
`version`, so every drone converges on the same position, not just the same
list.

Canonical rule for two DIFFERENT ids that turn out to be one object (two
drones logged it before hearing each other): the earlier first-sighting stamp
wins, ties to the lower origin drone. Same rule on every drone, so lists
converge. The loser is marked superseded - the CSV is append-only.

No ROS imports: unit-tested in test/test_hazard_registry.py.
"""

import math
from dataclasses import dataclass, field


@dataclass
class Hazard:
    hazard_id: str
    origin: int
    seq: int
    stamp: float           # first sighting
    north: float
    east: float
    alt: float
    cls: str
    conf: float            # best confidence seen
    version: int = 0
    n: int = 1             # sightings merged into the estimate
    spread: float = 0.0    # RMS distance of sightings from the estimate (m)
    obs: list = field(default_factory=list, repr=False)   # owner only: (n, e, w)

    @property
    def key(self):
        return (self.stamp, self.origin, self.seq)

    def refine(self, north, east, conf, weight=None):
        self.obs.append((north, east, max(conf if weight is None else weight, 1e-3)))
        w = sum(o[2] for o in self.obs)
        self.north = sum(o[0] * o[2] for o in self.obs) / w
        self.east = sum(o[1] * o[2] for o in self.obs) / w
        self.n = len(self.obs)
        self.spread = math.sqrt(sum((o[0] - self.north) ** 2 + (o[1] - self.east) ** 2
                                    for o in self.obs) / self.n)
        self.conf = max(self.conf, conf)
        self.version += 1


class HazardRegistry:
    def __init__(self, me, min_sep_m, box_merge_m=1.0):
        self.me = me
        self.min_sep = min_sep_m
        self.box_merge = box_merge_m
        self.items = {}        # hazard_id -> Hazard, current canonical set
        self.seen_ids = set()  # every id ever handled, incl. superseded
        self.seq = {}          # per-origin counter

    def nearest(self, north, east, skip_origin=None):
        best = None
        for h in self.items.values():
            if h.origin == skip_origin:
                continue
            d = math.hypot(h.north - north, h.east - east)
            if d < self.min_sep and (best is None or d < best[1]):
                best = (h, d)
        return best

    def add_frame(self, sightings, stamp, origin=None):
        """One frame's detections: list of (north, east, alt, cls, conf[, weight]).

        -> list, per sighting, of (outcome, hazard):
           'new'      created a hazard
           'refined'  merged into a hazard this drone owns, which it improved
           'seen'     merged into a PEER's hazard (not ours to refine)
           'box_dup'  a second box on an object already in this frame: the
                      hazard is that object's; nothing was changed
        `origin` defaults to this drone; a ground-side detector serving several
        cameras passes the camera's drone index instead."""
        origin = self.me if origin is None else origin
        # Duplicate boxes first: most confident wins, the rest point at it.
        dup_of, kept = {}, []
        for i in sorted(range(len(sightings)), key=lambda k: -sightings[k][4]):
            n, e = sightings[i][0], sightings[i][1]
            parent = next((k for k in kept if math.hypot(
                sightings[k][0] - n, sightings[k][1] - e) < self.box_merge), None)
            if parent is None:
                kept.append(i)
            else:
                dup_of[i] = parent
        pairs = sorted(
            (math.hypot(h.north - sightings[i][0], h.east - sightings[i][1]), i, hid)
            for i in kept
            for hid, h in self.items.items())
        out, used_h = {}, set()
        for d, i, hid in pairs:
            if d >= self.min_sep:
                break
            if i in out or hid in used_h:
                continue
            used_h.add(hid)
            h = self.items[hid]
            if h.origin == origin:
                sg = sightings[i]
                h.refine(sg[0], sg[1], sg[4], sg[5] if len(sg) > 5 else None)
                out[i] = ('refined', h)
            else:
                out[i] = ('seen', h)
        for i, sg in enumerate(sightings):
            if i in out or i in dup_of:
                continue
            n, e, alt, cls, conf = sg[:5]
            seq = self.seq[origin] = self.seq.get(origin, 0) + 1
            h = Hazard(f"d{origin}-{seq}", origin, seq, stamp, n, e, alt, cls, conf)
            h.obs.append((n, e, max(sg[5] if len(sg) > 5 else conf, 1e-3)))
            self.items[h.hazard_id] = h
            self.seen_ids.add(h.hazard_id)
            out[i] = ('new', h)
        for i, parent in dup_of.items():
            out[i] = ('box_dup', out[parent][1])
        return [out[i] for i in range(len(sightings))]

    def add_own(self, north, east, alt, cls, conf, stamp, origin=None):
        """Single detection. -> the new Hazard, or None if it merged."""
        outcome, h = self.add_frame([(north, east, alt, cls, conf)], stamp, origin)[0]
        return h if outcome == 'new' else None

    def add_peer(self, h):
        """-> (outcome, superseded) where outcome is
        'repeat'   nothing new (old version, re-broadcast, history replay)
        'update'   newer version of a hazard we hold: position adopted
        'new'      a new object, added
        'dup'      same object as one we hold, ours is canonical: ignored
        'replaces' same object, the peer's is canonical: ours superseded
        """
        held = self.items.get(h.hazard_id)
        if held is not None:
            if h.version <= held.version:
                return ('repeat', None)
            for k in ('north', 'east', 'conf', 'version', 'n', 'spread'):
                setattr(held, k, getattr(h, k))
            return ('update', None)
        if h.hazard_id in self.seen_ids:
            return ('repeat', None)
        self.seen_ids.add(h.hazard_id)
        # Only reports from a DIFFERENT drone can be the same object logged
        # twice: two ids from one origin were kept apart on purpose by that
        # drone's per-frame association (a close pair seen together).
        near = self.nearest(h.north, h.east, skip_origin=h.origin)
        if near is None:
            self.items[h.hazard_id] = h
            return ('new', None)
        mine = near[0]
        if mine.key <= h.key:
            return ('dup', None)
        del self.items[mine.hazard_id]
        self.items[h.hazard_id] = h
        return ('replaces', mine)
