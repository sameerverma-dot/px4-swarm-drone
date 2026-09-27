#!/usr/bin/env python3
"""
coverage_model.py — how much ground can this system actually clear?

Answers the questions that decide the swarm design, using parameters measured
from real flights rather than guessed:

  * What is the MAXIMUM altitude? (set by detection, not by the airframe)
  * What lane spacing does that force?
  * How long does an area take, with N drones?
  * Does sharing one detector across N drones starve any of them?
  * Is raising the camera resolution worth it?

Measured inputs (all from this project's own flights and experiments):
  HFOV 1.74 rad, 640x480 capture      mono_cam SDF
  detection floor ~24 px on target    experiments/px_sweep.py
  target ~0.5 m across from above     a standing person, nadir view
  survey speed 3.8 m/s                lookahead_m 4.0, measured median 3.05-3.8
  detector ~11 FPS at imgsz 640       measured on the RTX 4060
  imgsz 1280 costs 3.8x imgsz 640     measured
"""
import math

HFOV      = 1.74      # rad
IMG_W     = 640       # capture width, px
ASPECT    = 480 / 640
TARGET_M  = 0.5       # person, seen from directly above
MIN_PX    = 24        # detection floor, measured
SIDELAP   = 0.30
SPEED     = 3.8       # m/s, the geotag-accuracy speed cap
FPS_640   = 11.0      # measured, single drone
IMGSZ_COST = {640: 1.0, 960: 1.96, 1280: 3.82}   # measured ratios


def footprint(alt, cap_w=IMG_W):
    """(width, height) of the ground patch in view, metres."""
    w = 2 * alt * math.tan(HFOV / 2)
    return w, w * ASPECT


def max_altitude(cap_w=IMG_W):
    """Highest altitude at which the target still spans MIN_PX pixels."""
    mpp_max = TARGET_M / MIN_PX          # metres per pixel we can tolerate
    w_max = mpp_max * cap_w              # widest footprint that still resolves
    return w_max / (2 * math.tan(HFOV / 2))


def survey_time(x, y, alt, n_drones=1, speed=SPEED, cap_w=IMG_W,
                turn_penalty_s=2.3):
    """Seconds for n_drones to cover an x-by-y rectangle, splitting along y.

    turn_penalty_s is calibrated against the real 5 Sep flight: the model gave
    38 s of pure path time for a run that actually took 45 s over 3 turns.
    """
    w, _ = footprint(alt, cap_w)
    lane = w * (1 - SIDELAP)
    y_each = y / n_drones
    lanes = max(2, math.ceil(y_each / lane) + 1)
    path = lanes * x + (lanes - 1) * lane
    return path / speed + (lanes - 1) * turn_penalty_s, lanes, lane


def per_drone_fps(n_drones, imgsz=640):
    """One shared detector, round-robin across N camera streams."""
    return FPS_640 / IMGSZ_COST[imgsz] / n_drones


def frame_gap(n_drones, alt, imgsz=640, cap_w=IMG_W, speed=SPEED):
    """Along-track distance between consecutive frames of the SAME drone,
    against the footprint depth. Ratio < 1 means the ground is covered."""
    _, fh = footprint(alt, cap_w)
    gap = speed / per_drone_fps(n_drones, imgsz)
    return gap, fh, gap / fh


def hectares_per_sortie(alt, cap_w=IMG_W, speed=SPEED, endurance_s=900):
    w, _ = footprint(alt, cap_w)
    lane = w * (1 - SIDELAP)
    return speed * endurance_s * lane / 10_000


print(__doc__)
print("=" * 74)
print("\n1. THE CEILING — detection sets the maximum altitude\n")
for cap in (640, 1280):
    h = max_altitude(cap)
    w, fh = footprint(h, cap)
    print(f"   {cap}px capture -> max altitude {h:5.2f} m"
          f" | footprint {w:5.1f} x {fh:4.1f} m | lane spacing {w*(1-SIDELAP):5.2f} m")
print("""
   The airframe could fly higher. The MODEL cannot see that far. This is the
   binding constraint on the whole system, and it is a data problem.""")

print("\n2. CALIBRATION — not yet a validation\n")
t, lanes, lane = survey_time(30, 20, 5.0)
print(f"   30 x 20 m at 5 m, 1 drone: model {t:.0f} s over {lanes} lanes"
      f" (spacing {lane:.2f} m)")
print(f"   actual flight (wp1 -> wp9): 45 s over 4 lanes (spacing 8.30 m)")
print("""
   These agree exactly, and that proves nothing: turn_penalty_s was FITTED to
   this one flight, so the match is a tautology, not a prediction. The lane
   geometry and path length are independent of that fit and are correct; the
   turn cost is a single fitted constant.

   To actually validate: fly a DIFFERENT area (say 40 x 30) and compare. Until
   then treat times below as +-20 %.""")

print("\n3. DOES A SHARED DETECTOR STARVE THE DRONES?\n")
print(f"   {'drones':>6} {'imgsz':>6} {'FPS each':>9} {'frame gap':>10} {'footprint':>10} {'ratio':>7}")
for n in (1, 2, 3):
    for imgsz in (640, 1280):
        alt = max_altitude(640 if imgsz == 640 else 1280)
        gap, fh, ratio = frame_gap(n, alt, imgsz, 640 if imgsz == 640 else 1280)
        flag = "OK" if ratio < 0.5 else "TIGHT" if ratio < 1 else "GAPS"
        print(f"   {n:>6} {imgsz:>6} {per_drone_fps(n,imgsz):>8.1f}  "
              f"{gap:>9.2f}m {fh:>9.1f}m {ratio:>6.2f}  {flag}")
print("""
   Every configuration has 5x or more redundancy along-track. Sharing one
   detector across three drones does NOT starve them — an earlier worry that
   the numbers do not support. Inference throughput is not the constraint.""")

print("\n4. WHAT THE SWARM BUYS — time to clear an area\n")
for x, y, label in [(30, 20, "the test arena"), (100, 100, "1 hectare"),
                    (200, 200, "4 hectares")]:
    print(f"   {label} ({x}x{y} m):")
    for cap, imgsz in ((640, 640), (1280, 1280)):
        alt = max_altitude(cap)
        row = []
        for n in (1, 2, 3):
            t, _, _ = survey_time(x, y, alt, n, cap_w=cap)
            row.append(f"{n}x {t/60:5.1f} min")
        print(f"      {cap:>4}px @ {alt:4.1f} m:  " + "   ".join(row))
    print()

print("""   NOTE the arena row: at 1280/11.2 m the 30x20 area needs only 2 lanes,
   so 2 and 3 drones finish at the same time. Your test arena is too small to
   DEMONSTRATE the swarm's value at high altitude — the parallelism has nothing
   to divide. For the demo, either fly 640/5.6 m (where 3 drones do show a
   speedup) or use a larger area.
""")

print("5. AREA PER SORTIE (15 min usable endurance)\n")
for cap in (640, 1280):
    alt = max_altitude(cap)
    ha = hectares_per_sortie(alt, cap)
    print(f"   {cap}px @ {alt:4.1f} m: {ha:5.2f} ha per drone  |  "
          f"3 drones: {ha*3:5.2f} ha")
print(f"""
   Raising capture to 1280 (WITH imgsz raised to match) doubles the usable
   altitude, which doubles lane spacing, which doubles area per sortie —
   {hectares_per_sortie(max_altitude(1280),1280)/hectares_per_sortie(max_altitude(640),640):.2f}x. It costs 3.8x the inference time, but section 3 shows
   inference has ~5-15x headroom, so that cost does not bind.

   ONE 1280-capture drone clears more ground than TWO at 640.""")

print("\n6. THE SPEED TRADE\n")
print(f"   {'speed':>7} {'geotag err':>11} {'30x20 time':>11} {'ha/sortie':>10}")
for v, err in ((3.8, 0.75), (6.0, 1.6), (9.2, 3.98)):
    t, _, _ = survey_time(30, 20, max_altitude(640), 1, speed=v)
    ha = hectares_per_sortie(max_altitude(640), speed=v)
    print(f"   {v:>5.1f} m/s {err:>10.2f} m {t:>10.0f} s {ha:>9.2f}")
print("""
   Geotag error scales with speed (it is latency x speed). Coverage scales with
   speed too. 3.8 m/s was chosen for accuracy; if a demo needs to be shorter,
   this is the knob, and the cost is stated rather than hidden.""")
