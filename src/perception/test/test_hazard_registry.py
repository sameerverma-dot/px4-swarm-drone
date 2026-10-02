"""Unit tests for perception/hazard_registry.py - python3 -m pytest src/perception/test -q"""

import copy

import pytest

from perception.hazard_registry import Hazard, HazardRegistry


def rep(origin, seq, stamp, n, e, version=0):
    return Hazard(f"d{origin}-{seq}", origin, seq, stamp, n, e, 10.0, 'person', 0.8,
                  version=version)


def S(n, e, conf=0.8):
    return (n, e, 10.0, 'person', conf)


# ------------------------------------------------------------- same drone
def test_close_pair_in_one_frame_stays_two_hazards():
    r = HazardRegistry(me=0, min_sep_m=3.0)
    out = r.add_frame([S(10, 10), S(11.5, 10)], stamp=1)      # 1.5 m apart, same frame
    assert [o for o, _ in out] == ['new', 'new'] and len(r.items) == 2


def test_two_boxes_on_one_object_are_one_hazard():
    # 2 Oct: YOLO boxed one person twice, 0.26 m apart -> d2-1 AND d2-2.
    r = HazardRegistry(me=2, min_sep_m=2.5)
    out = r.add_frame([S(5.26, 73.91, 0.75), S(5.19, 74.16, 0.79)], stamp=1)
    assert [o for o, _ in out] == ['box_dup', 'new'] and len(r.items) == 1
    assert out[0][1] is out[1][1] and out[1][1].conf == 0.79   # the confident box wins


def test_box_dup_does_not_refine_and_pair_at_2m_survives():
    r = HazardRegistry(me=1, min_sep_m=2.5)
    a, b = [h for _, h in r.add_frame([S(15.1, 36.7), S(14.9, 38.9)], stamp=1)]  # pairA/B
    out = r.add_frame([S(15.0, 36.6, 0.9), S(15.3, 36.8, 0.3), S(15.0, 38.8, 0.85)], stamp=2)
    assert [o for o, _ in out] == ['refined', 'box_dup', 'refined']
    assert out[0][1] is a and out[1][1] is a and out[2][1] is b
    assert a.n == 2 and b.n == 2 and len(r.items) == 2


def test_close_pair_keeps_its_identity_in_later_frames():
    r = HazardRegistry(me=0, min_sep_m=3.0)
    a, b = [h for _, h in r.add_frame([S(10, 10), S(11.5, 10)], stamp=1)]
    out = r.add_frame([S(11.4, 10.1), S(10.1, 9.9)], stamp=2)  # order shuffled
    assert out[0][1] is b and out[1][1] is a
    assert all(o == 'refined' for o, _ in out) and len(r.items) == 2


def test_refinement_is_confidence_weighted_and_keeps_id_and_stamp():
    r = HazardRegistry(me=0, min_sep_m=3.0)
    h = r.add_own(10.0, 10.0, 10, 'person', 0.5, stamp=1)     # edge-of-image fix
    r.add_frame([S(12.0, 10.0, conf=0.9)], stamp=2)
    assert h.hazard_id == 'd0-1' and h.stamp == 1
    assert h.north == pytest.approx((10 * 0.5 + 12 * 0.9) / 1.4)
    assert h.n == 2 and h.version == 1 and h.conf == 0.9 and h.spread > 0


def test_explicit_weight_overrides_confidence():
    # A confident but oblique first fix must not dominate a near-nadir view.
    r = HazardRegistry(me=1, min_sep_m=1.5)
    h = r.add_frame([(15.0, 37.0, 10.0, 'person', 0.84, 0.84 * 0.3)], stamp=1)[0][1]
    r.add_frame([(15.0, 36.5, 10.0, 'person', 0.80, 0.80 * 1.0)], stamp=2)
    assert h.east == pytest.approx((37.0 * 0.252 + 36.5 * 0.8) / 1.052)
    assert h.conf == 0.84


# ------------------------------------------------------------ across drones
def test_own_detection_suppressed_by_peer_find_at_band_boundary():
    r0 = HazardRegistry(me=0, min_sep_m=3.0)
    assert r0.add_peer(rep(1, 1, 10.0, 15.0, 30.2))[0] == 'new'
    out = r0.add_frame([S(15.4, 29.6)], stamp=40.0)
    assert out[0][0] == 'seen' and len(r0.items) == 1
    assert r0.items['d1-1'].north == 15.0                     # not ours to refine


def test_owner_refinements_propagate_by_version():
    r0, r1 = HazardRegistry(0, 3.0), HazardRegistry(1, 3.0)
    h = r1.add_own(15, 30, 10, 'person', 0.6, stamp=1)
    assert r0.add_peer(copy.copy(h))[0] == 'new'
    r1.add_frame([S(16, 30, 0.9)], stamp=2)
    assert r0.add_peer(copy.copy(h)) == ('update', None)
    assert r0.items['d1-1'].north == pytest.approx(h.north)
    assert r0.add_peer(copy.copy(h))[0] == 'repeat'           # same version again


def test_peers_close_pair_is_not_merged_by_the_receiver():
    r0 = HazardRegistry(0, 3.0)
    assert r0.add_peer(rep(1, 1, 5.0, 10.0, 40.0))[0] == 'new'
    assert r0.add_peer(rep(1, 2, 5.0, 11.5, 40.0))[0] == 'new'   # drone 1 kept them apart
    assert len(r0.items) == 2


def test_simultaneous_finds_converge_to_the_same_canonical_hazard():
    r0, r1 = HazardRegistry(0, 3.0), HazardRegistry(1, 3.0)
    a = r0.add_own(15.0, 30.0, 10, 'person', 0.8, stamp=20.00)
    b = r1.add_own(15.3, 30.1, 10, 'person', 0.8, stamp=20.05)
    assert r0.add_peer(copy.copy(b)) == ('dup', None)          # mine (a) is earlier
    outcome, superseded = r1.add_peer(copy.copy(a))
    assert outcome == 'replaces' and superseded.hazard_id == b.hazard_id
    assert set(r0.items) == set(r1.items) == {a.hazard_id}


def test_stamp_tie_goes_to_lower_origin():
    r2 = HazardRegistry(2, 3.0)
    mine = r2.add_own(1, 1, 10, 'person', 0.8, stamp=5.0)
    outcome, sup = r2.add_peer(rep(1, 7, 5.0, 1.2, 1.0))
    assert outcome == 'replaces' and sup is mine


def test_far_apart_reports_are_separate_objects():
    r = HazardRegistry(0, 3.0)
    r.add_own(15, 15, 10, 'person', 0.8, stamp=1)
    assert r.add_peer(rep(1, 1, 2, 15, 45))[0] == 'new'
    assert len(r.items) == 2


def test_superseded_or_replayed_ids_stay_out():
    r = HazardRegistry(0, 3.0)
    r.add_peer(rep(1, 1, 10.0, 15, 30))
    assert r.add_peer(rep(1, 1, 10.0, 15, 30))[0] == 'repeat'
