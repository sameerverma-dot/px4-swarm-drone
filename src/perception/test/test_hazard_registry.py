"""Unit tests for perception/hazard_registry.py - python3 -m pytest src/perception/test -q"""

from perception.hazard_registry import Hazard, HazardRegistry


def rep(origin, seq, stamp, n, e):
    return Hazard(f"d{origin}-{seq}", origin, seq, stamp, n, e, 10.0, 'person', 0.8)


def test_own_detection_suppressed_by_peer_find_at_band_boundary():
    r0 = HazardRegistry(me=0, min_sep_m=4.5)
    assert r0.add_peer(rep(1, 1, 10.0, 15.0, 30.2))[0] == 'new'   # drone 1 found it first
    assert r0.add_own(15.4, 29.6, 10, 'person', 0.8, stamp=40.0) is None


def test_repeat_ids_are_ignored():
    r = HazardRegistry(me=0, min_sep_m=4.5)
    r.add_peer(rep(1, 1, 10.0, 15, 30))
    assert r.add_peer(rep(1, 1, 10.0, 15, 30))[0] == 'repeat'


def test_simultaneous_finds_converge_to_the_same_canonical_hazard():
    # Both drones log the object before hearing each other.
    r0, r1 = HazardRegistry(0, 4.5), HazardRegistry(1, 4.5)
    a = r0.add_own(15.0, 30.0, 10, 'person', 0.8, stamp=20.00)
    b = r1.add_own(15.3, 30.1, 10, 'person', 0.8, stamp=20.05)
    assert r0.add_peer(b) == ('dup', None)            # mine (a) is earlier
    outcome, superseded = r1.add_peer(a)
    assert outcome == 'replaces' and superseded.hazard_id == b.hazard_id
    assert set(r0.items) == set(r1.items) == {a.hazard_id}


def test_stamp_tie_goes_to_lower_origin():
    r2 = HazardRegistry(2, 4.5)
    mine = r2.add_own(1, 1, 10, 'person', 0.8, stamp=5.0)
    outcome, sup = r2.add_peer(rep(1, 7, 5.0, 1.2, 1.0))
    assert outcome == 'replaces' and sup is mine


def test_far_apart_reports_are_separate_objects():
    r = HazardRegistry(0, 4.5)
    r.add_own(15, 15, 10, 'person', 0.8, stamp=1)
    assert r.add_peer(rep(1, 1, 2, 15, 45))[0] == 'new'
    assert len(r.items) == 2
