#!/usr/bin/env python3
"""
analyse_sightings.py - calibrate the detector from one swarm run's sightings.

The detector writes sightings_d<i>.csv in the run dir: one row per person box
in every gated frame, INCLUDING boxes below the recording threshold (outcome
'below'), each with the drone's pose and the box projected to the ground two
ways (attitude-aware, and level-drone). Given where the targets really are,
this answers three tuning questions from data instead of guesses:

  1. SWATH     confidence against lateral offset from the lane. The lane
               spacing must come from the width over which a target is
               reliably scored >= conf - NOT the camera footprint, which is
               much wider (2 Oct: targets 7.5 m off-lane scored 0.41-0.64
               against 0.65 and were never recorded).
  2. LAG       along-track error split by travel direction. A timing error
               shows up as an error along the direction of travel (behind the
               drone if pose_lag_s is too big, ahead if too small); a fixed
               offset does not flip with direction.
  3. SPREAD    scatter of the recorded sightings of each target - min_sep_m
               should be ~2x it, so one target is one hazard but two targets a
               couple of metres apart are not merged.

    python3 tools/analyse_sightings.py                       # newest run, targets from run dir
    python3 tools/analyse_sightings.py --run ~/maps/swarm_X --truth 15,15 --truth 15,45

Targets come from --truth N,E (repeatable) or, if none are given, from
targets.csv in the run dir (columns: name,north,east).
"""

import argparse
import csv
import glob
import json
import math
import os
import statistics as stats
from collections import defaultdict


def newest_run(maps='~/maps'):
    runs = [d for d in glob.glob(os.path.join(os.path.expanduser(maps), 'swarm_*'))
            if os.path.exists(os.path.join(d, 'run.json'))]
    if not runs:
        raise SystemExit('no swarm_* run dir with run.json in ~/maps')
    return max(runs, key=os.path.getmtime)


def load_truth(run, truth_args):
    if truth_args:
        out = []
        for k, t in enumerate(truth_args):
            n, e = (float(v) for v in t.split(','))
            out.append((f't{k}', n, e))
        return out
    p = os.path.join(run, 'targets.csv')
    if not os.path.exists(p):
        raise SystemExit(f'no --truth given and no {p}')
    with open(p) as f:
        return [(r['name'], float(r['north']), float(r['east'])) for r in csv.DictReader(f)]


def load_sightings(run, band_h):
    rows = []
    for p in sorted(glob.glob(os.path.join(run, 'sightings_d*.csv'))):
        with open(p) as f:
            for r in csv.DictReader(f):
                d = int(r['drone'].lstrip('d') or 0)
                r = {k: (v if k in ('drone', 'hazard_id', 'outcome') else float(v))
                     for k, v in r.items()}
                r['d'] = d
                # Drone position in the shared frame (drone 0's home): drone i
                # spawns band_h*i east of drone 0 (start_px4_swarm.sh).
                r['gn'], r['ge'] = r['x'], r['y'] + d * band_h
                rows.append(r)
    return rows


def nearest(truth, n, e, max_d):
    best = None
    for name, tn, te in truth:
        dist = math.hypot(n - tn, e - te)
        if dist <= max_d and (best is None or dist < best[1]):
            best = (name, dist, tn, te)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--run', default='latest')
    ap.add_argument('--truth', action='append', default=[], metavar='N,E')
    ap.add_argument('--assoc-m', type=float, default=4.0,
                    help='sighting counts for a target if projected within this (default 4)')
    ap.add_argument('--max-tilt', type=float, default=8.0,
                    help='leave out frames with roll or pitch beyond this (deg) from 2. and 3. '
                         '- the turn onto a lane banks the drone 20+ deg (default 8)')
    a = ap.parse_args()

    run = newest_run() if a.run == 'latest' else os.path.expanduser(a.run)
    meta = json.load(open(os.path.join(run, 'run.json')))
    band_h = (meta['y_max'] - meta['y_min']) / meta['num_drones']
    truth = load_truth(run, a.truth)
    rows = load_sightings(run, band_h)
    print(f'run {run}: {len(rows)} sightings, {len(truth)} targets, band {band_h:.1f} m')

    # Associate each sighting with a target (attitude projection, else level).
    for r in rows:
        m = nearest(truth, r['north'], r['east'], a.assoc_m) or \
            nearest(truth, r['north_level'], r['east_level'], a.assoc_m)
        r['target'] = m
    hit = [r for r in rows if r['target']]
    print(f'  {len(hit)} associated with a target, {len(rows) - len(hit)} not '
          f'(false positives or > {a.assoc_m} m off)')
    tilt = math.radians(a.max_tilt)
    steady = [r for r in hit if abs(r['roll']) <= tilt and abs(r['pitch']) <= tilt]
    print(f'  {len(steady)} of those in steady flight (roll and pitch within '
          f'{a.max_tilt:g} deg); 2. and 3. use only these')

    # ---- 1. SWATH: per target per lane pass ---------------------------------
    # A pass = one drone flying one lane in one direction. Lanes run north, so
    # the lateral offset is the east distance from the drone's track (median
    # over the pass's frames) to the target.
    passes = defaultdict(list)
    for r in hit:
        direction = 'N' if r['vx'] >= 0 else 'S'
        passes[(r['target'][0], r['d'], direction, round(r['ge'] / 4.0))].append(r)
    print('\n1. SWATH - best confidence per target per lane pass')
    print(f'   {"target":<12}{"drone":>6}{"dir":>4}{"lateral m":>11}{"best":>7}'
          f'{"frames":>8}{"recorded":>10}{"turning":>9}')
    table = []
    for (name, d, direction, _), rs in sorted(passes.items(),
                                              key=lambda kv: (kv[0][0], kv[0][1])):
        te = rs[0]['target'][3]
        lat = abs(te - stats.median(r['ge'] for r in rs))
        best = max(r['conf'] for r in rs)
        # 'below' = under threshold, 'turning' = dropped mid-turn; anything
        # else went to the registry.
        above = sum(1 for r in rs if r['outcome'] not in ('below', 'turning'))
        turning = sum(1 for r in rs if r['outcome'] == 'turning')
        table.append((lat, best, above))
        print(f'   {name:<12}{d:>6}{direction:>4}{lat:>11.1f}{best:>7.2f}{len(rs):>8}'
              f'{above:>10}{turning:>9}')
    print('\n   by lateral offset (passes, recorded = at least one frame went to the registry):')
    for lo in range(0, 14, 2):
        b = [t for t in table if lo <= t[0] < lo + 2]
        if b:
            rec = sum(1 for t in b if t[2] > 0)
            print(f'   {lo:>2}-{lo + 2:<2} m  passes {len(b):>2}  recorded {rec:>2}  '
                  f'best conf {min(t[1] for t in b):.2f}-{max(t[1] for t in b):.2f}')

    # ---- 2. LAG: along-track error by travel direction ----------------------
    print('\n2. ALONG-TRACK ERROR by direction (projected - truth, + = ahead of the drone)')
    for proj, (kn, ke) in (('attitude', ('north', 'east')),
                           ('level', ('north_level', 'east_level'))):
        for direction in ('N', 'S'):
            rs = [r for r in steady if (r['vx'] >= 0) == (direction == 'N')]
            if len(rs) < 3:
                continue
            sgn = 1 if direction == 'N' else -1
            along = [sgn * (r[kn] - r['target'][2]) for r in rs]
            cross = [r[ke] - r['target'][3] for r in rs]
            speed = stats.mean(abs(r['vx']) for r in rs)
            print(f'   {proj:<9}{direction}bound  n={len(rs):<4} along {stats.mean(along):+.2f} '
                  f'(sd {stats.pstdev(along):.2f})  east {stats.mean(cross):+.2f} '
                  f'(sd {stats.pstdev(cross):.2f})  speed {speed:.1f} m/s')
    rs = steady
    if rs:
        along = [(1 if r['vx'] >= 0 else -1) * (r['north'] - r['target'][2]) for r in rs]
        speed = stats.mean(math.hypot(r['vx'], r['vy']) for r in rs)
        dlag = stats.mean(along) / speed if speed > 0.5 else 0.0
        print(f'   attitude, both directions: along {stats.mean(along):+.2f} m at '
              f'{speed:.1f} m/s -> change pose_lag_s by {dlag:+.3f} s')

    # ---- 3. SPREAD: recorded sightings per target ---------------------------
    print('\n3. SPREAD of recorded sightings per target (attitude projection)')
    by_t = defaultdict(list)
    for r in steady:
        if r['outcome'] not in ('below', 'turning'):
            by_t[r['target'][0]].append(r)
    radii = []
    for name, tn, te in truth:
        rs = by_t.get(name, [])
        if not rs:
            print(f'   {name:<12} not recorded')
            continue
        mn = stats.mean(r['north'] for r in rs)
        me = stats.mean(r['east'] for r in rs)
        rad = [math.hypot(r['north'] - mn, r['east'] - me) for r in rs]
        err = [math.hypot(r['north'] - tn, r['east'] - te) for r in rs]
        rms = math.sqrt(stats.mean(x * x for x in rad))
        radii += rad
        print(f'   {name:<12} n={len(rs):<3} spread rms {rms:.2f} max {max(rad):.2f}  '
              f'error to truth mean {stats.mean(err):.2f} max {max(err):.2f}  '
              f'mean pos error {math.hypot(mn - tn, me - te):.2f}')
    if radii:
        rms = math.sqrt(stats.mean(x * x for x in radii))
        print(f'   all: spread rms {rms:.2f} m, max {max(radii):.2f} m '
              f'-> min_sep_m ~ {max(2 * rms, max(radii)):.1f}')


if __name__ == '__main__':
    main()
