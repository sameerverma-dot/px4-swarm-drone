"""Unit tests for survey/swarm_logic.py - run: python3 -m pytest src/survey/test -q"""

import math

import pytest

from survey import swarm_logic as sl


def peer(i, state=sl.SURVEY, done=0, total=3, claimed=-1, frm=0, mask=0,
         eta=30.0, batt=80.0, rx=100.0, n=0.0, e=0.0, alt=10.0):
    return sl.PeerInfo(i, state, n, e, alt, done, total, claimed, frm, mask, eta, batt, rx)


# ---------------------------------------------------------------- geometry
def test_lanes_are_strip_centred_and_off_the_boundary():
    assert sl.lane_offsets(30.0, 16.59) == pytest.approx([7.5, 22.5])


@pytest.mark.parametrize('width,spacing', [(30, 16.59), (20, 16.59), (90, 16.59), (7, 16.59),
                                           (45, 10.0), (33.2, 16.6)])
def test_lanes_keep_the_sidelap_everywhere_including_across_bands(width, spacing):
    ys = sl.lane_offsets(width, spacing)
    gaps = [b - a for a, b in zip(ys, ys[1:])]
    assert all(g <= spacing + 1e-9 for g in gaps)
    # half a strip to each band edge, so lane-to-lane across a boundary is also <= spacing
    assert ys[0] <= spacing / 2 + 1e-9 and width - ys[-1] <= spacing / 2 + 1e-9
    assert all(0 < y < width for y in ys)              # never ON a band boundary


def test_band_origins():
    assert [sl.band_origin_east(b, 3, 0, 90) for b in range(3)] == [0, 30, 60]


def test_boustrophedon_base_pattern():
    w = sl.lane_waypoints([0, 16.59, 30], 0, 30)
    assert [(x, y) for x, y, _, _ in w] == [(0, 0), (30, 0), (30, 16.59), (0, 16.59),
                                            (0, 30), (30, 30)]
    assert [end for *_, end in w] == [False, True] * 3


def test_takeover_can_enter_from_either_end():
    w = sl.lane_waypoints([0, 16.59, 30], 0, 30, first=1, reverse_first=True)
    assert w[0][:2] == (0, 16.59) and w[1][:2] == (30, 16.59)   # lane 1 normally runs 30->0


def test_eta_counts_distance_and_turns():
    assert sl.path_eta((0, 0), [(30, 0)], speed=3.0, turn_penalty_s=2.0) == pytest.approx(12.0)


# -------------------------------------------------------------- separation
def test_lower_id_keeps_right_of_way_between_working_drones():
    hit = sl.separation_conflict(1, sl.SURVEY, (0, 30, 10), [(0, sl.SURVEY, 2, 30, 10, 0.2)], 8, 5, 1.5)
    assert hit[0] == 0 and hit[3] is True          # drone 1 yields to drone 0
    hit = sl.separation_conflict(0, sl.SURVEY, (2, 30, 10), [(1, sl.SURVEY, 0, 30, 10, 0.2)], 8, 5, 1.5)
    assert hit[3] is False                          # drone 0 carries on


def test_idle_drone_in_hold_gives_way_to_a_working_one_regardless_of_id():
    # drone 0 parked in HOLD where drone 1's lane passes: drone 0 moves
    assert sl.separation_conflict(0, sl.HOLD, (0, 30, 10),
                                  [(1, sl.SURVEY, 1, 30, 10, 0.2)], 8, 5, 1.5)[3] is True
    assert sl.separation_conflict(1, sl.SURVEY, (1, 30, 10),
                                  [(0, sl.HOLD, 0, 30, 10, 0.2)], 8, 5, 1.5)[3] is False


def test_a_working_drone_that_is_yielding_keeps_its_priority_over_idle_peers():
    # me: working (underlying SURVEY) even while I broadcast YIELD; peer idle in HOLD
    assert sl.separation_conflict(2, sl.SURVEY, (0, 0, 10),
                                  [(0, sl.HOLD, 3, 0, 12, 0.2)], 8, 5, 1.5)[3] is False


def test_peer_under_px4_control_cannot_be_asked_to_move():
    hit = sl.separation_conflict(0, sl.SURVEY, (0, 0, 10), [(2, sl.RETURNING, 3, 0, 11, 0.2)], 8, 5, 1.5)
    assert hit[3] is True


def test_peer_already_yielding_to_me():
    hit = sl.separation_conflict(0, sl.SURVEY, (0, 0, 10), [(1, sl.YIELD, 3, 0, 12, 0.2)], 8, 5, 1.5)
    assert hit[3] is False


def test_outside_cylinder_or_stale_is_ignored():
    sc = sl.separation_conflict
    assert sc(0, sl.SURVEY, (0, 0, 10), [(1, sl.SURVEY, 9, 0, 10, 0.2)], 8, 5, 1.5) is None
    assert sc(0, sl.SURVEY, (0, 0, 10), [(1, sl.SURVEY, 1, 0, 16, 0.2)], 8, 5, 1.5) is None
    assert sc(0, sl.SURVEY, (0, 0, 10), [(1, sl.SURVEY, 1, 0, 10, 9.0)], 8, 5, 1.5) is None


def test_yield_never_climbs_through_the_other_drone():
    assert sl.yield_altitude(3.5, 10.0, 5, 5) == 3.5      # below it with room: stay
    assert sl.yield_altitude(5.0, 10.0, 5, 5) == 4.0      # open the gap + margin
    assert sl.yield_altitude(8.0, 10.0, 5, 5) == 4.0
    assert sl.yield_altitude(10.0, 10.0, 5, 5) == 16.0    # level with it: go over
    assert sl.yield_altitude(10.0, 10.0, 5, 8) == 18.0    # a bigger climb is kept
    assert sl.yield_altitude(4.0, 6.0, 5, 5, min_alt=3) == 3.0   # capped: gap < sep_v


def test_escape_point_moves_away_on_my_side():
    n, e = sl.escape_point((0.0, 3.0), (0.0, 0.0), 10.0)
    assert (n, e) == pytest.approx((0.0, 10.0))
    n, e = sl.escape_point((-4.6, 31.3), (-5.0, 28.1), 10.0)   # 2 Oct: 3.3 m apart
    assert math.hypot(n + 5.0, e - 28.1) == pytest.approx(10.0) and e > 31.3
    assert sl.escape_point((1.0, 1.0), (1.0, 1.1), 10.0, fallback=(1.0, 0.0)) == \
        pytest.approx((11.0, 1.1))


def test_escape_from_a_moving_peer_steps_off_its_line():
    # peer flying north at 3.7 m/s straight at me, 5 m ahead of it
    n, e = sl.escape_point((5.0, 0.0), (0.0, 0.0), 10.0, fallback=(0.0, 1.0),
                           peer_vel=(3.7, 0.0))
    assert (n, e) == pytest.approx((5.0, 10.0))          # sideways, not backwards
    n, e = sl.escape_point((5.0, -2.0), (0.0, 0.0), 10.0, peer_vel=(3.7, 0.0))
    assert (n, e) == pytest.approx((5.0, -10.0))         # already left of it: go left


def test_peer_climb_rate_from_heartbeats():
    c = sl.SwarmCoordinator(1, 3, 3, peer_timeout_s=3, startup_grace_s=45,
                            deadline_margin_s=15, min_takeover_battery=30, claim_wait_s=20)
    for k in range(8):                              # RTL climb at 3 m/s, 4 Hz
        c.update(peer(0, sl.RETURNING, alt=10 + 0.75 * k, rx=100 + 0.25 * k))
    assert c.peers[0].vz == pytest.approx(3.0, abs=0.1)
    assert c.peers[0].vn == pytest.approx(0.0) and c.peers[0].ve == pytest.approx(0.0)


# ---------------------------------------------------------------- takeover
def coord(me=1, now_start=0.0, **kw):
    c = sl.SwarmCoordinator(me, 3, 3, peer_timeout_s=3, startup_grace_s=45,
                            deadline_margin_s=15, min_takeover_battery=30, claim_wait_s=20, **kw)
    c.start(now_start)
    return c


def test_early_return_is_orphaned_immediately():
    c = coord(me=1)
    c.update(peer(2, state=sl.RETURNING, done=1, rx=100))
    assert c.band_status(2, 100.5, 0) == ('orphan', 1, None)


def test_silent_drone_is_given_until_its_own_eta():
    c = coord(me=1)
    c.update(peer(2, state=sl.SURVEY, done=1, eta=40, rx=100))
    assert c.band_status(2, 102, 0)[0] == 'flying'        # still talking
    st, frm, t = c.band_status(2, 110, 0)
    assert (st, frm, t) == ('pending', 1, 155)            # 100 + 40 + 15
    assert c.band_status(2, 156, 0) == ('orphan', 1, None)


def test_silent_but_finished_is_not_an_orphan():
    c = coord(me=1)
    c.update(peer(2, state=sl.SURVEY, done=3, rx=100))
    assert c.band_status(2, 500, 0)[0] == 'covered'


def test_never_heard_owner_waits_out_the_startup_grace():
    c = coord(me=0, now_start=10)
    assert c.band_status(2, 30, 0) == ('pending', 0, 55)
    assert c.band_status(2, 60, 0) == ('orphan', 0, None)


def test_covered_by_anyone_mask():
    c = coord(me=0)
    c.update(peer(2, state=sl.RETURNING, done=1, rx=100))
    c.update(peer(1, state=sl.SURVEY, mask=0b110, rx=100))
    assert c.band_status(2, 100, 0)[0] == 'covered'


def test_claimed_by_live_peer_is_not_offered_again():
    c = coord(me=0)
    c.update(peer(2, state=sl.RETURNING, done=1, rx=100))
    c.update(peer(1, state=sl.TAKEOVER, claimed=2, frm=1, rx=100))
    assert c.band_status(2, 101, 0)[0] == 'claimed'
    assert c.orphans(101, 0) == []


def test_neighbour_beats_whoever_finished_first():
    # drone 0 is idle first; band 2's real neighbour, drone 1, is 10 s from done
    c = coord(me=0)
    c.update(peer(2, state=sl.RETURNING, done=1, rx=100))
    c.update(peer(1, state=sl.SURVEY, eta=10, rx=100))
    assert not c.should_claim(2, 100, my_battery=80)
    c.update(peer(1, state=sl.SURVEY, eta=200, rx=100))   # far from done: don't wait
    assert c.should_claim(2, 100, my_battery=80)


def test_idle_better_neighbour_wins_and_low_battery_never_claims():
    c = coord(me=0)
    c.update(peer(1, state=sl.HOLD, rx=100))
    assert not c.should_claim(2, 100, my_battery=80)
    c.update(peer(1, state=sl.HOLD, batt=10, rx=100))     # it can't go
    assert c.should_claim(2, 100, my_battery=80)
    assert not c.should_claim(2, 100, my_battery=20)      # neither can I


def test_simultaneous_claims_resolve_to_the_better_key():
    c = coord(me=0)
    c.update(peer(1, state=sl.TAKEOVER, claimed=2, rx=100))
    assert c.claim_conflict(2, 100)                       # (1,1) beats (2,0)
    c1 = coord(me=1)
    c1.update(peer(0, state=sl.TAKEOVER, claimed=2, rx=100))
    assert not c1.claim_conflict(2, 100)


def test_orphans_sorted_nearest_band_first():
    c = coord(me=0, now_start=0)
    c.update(peer(1, state=sl.RETURNING, done=0, rx=100))
    c.update(peer(2, state=sl.RETURNING, done=2, rx=100))
    assert c.orphans(100, 0) == [(1, 0), (2, 2)]
