# Hand-off — 2 Oct 2026 · and the plan for next time

*Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)*
*Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali*

Companion docs: `SYSTEM_GUIDE.md` (how it works) · `STACK_README.md` and
`CHEATSHEET.md` (how to run) · `PROGRESS.md` (status log; §11 the swarm, §12
this round) · `PHASE1_ROADMAP.md` (deliverables & grading) · `SWARM_PLAN.md`
(coverage arithmetic).

The previous version of this file (the 1 Sep session log and its plan) is in
git history; everything in its plan is done.

---

## 0. Where things stand

**The software deliverable works in simulation.** Three drones, each running
its own survey + detector + logs, coordinating only with each other:

```
VERIFY PASS | drone 0 | own lanes 8/8 | took over: none | waypoints 17/17 | returned=True
...
11 targets -> 11 hazards, 0 duplicates, error 0.09-0.29 m
saw person at N=22.1 E=29.8 - already logged by drone 1 as d1-1, not logged again
```

| Capability | Verified by |
|---|---|
| Onboard autonomy (no ground link needed) | ground station killed mid-flight (§11.3 B) |
| Band takeover: early return, dead onboard computer | flights B, C (§11.3) |
| Shared hazard list, boundary target logged once | every flight since 2 Oct |
| Separation: move away at own altitude | forced conflict, closest 5.75 m (§12.3 sep3) |
| Geotagging, calibrated from data | 11/11 targets, 0.09–0.29 m (§12.3 final) |
| Close targets (2 m apart) kept as two | 5 flights of 5 |
| Single-drone path still works | `mission.launch.py`, 2/2 targets |

Unit tests: `python3 -m pytest src/survey/test src/perception/test/test_hazard_registry.py -q` (44).

---

## 1. What changed this round (review fixes, 2 Oct)

Detail in `PROGRESS.md` §12. In short:

- **Hazard list**: matched one-to-one per frame; duplicate boxes merged
  (< 1.0 m); merge radius `min_sep_m` 4.5 → **1.5 m**; each hazard's position
  refined by its owner (conf × cos² off-nadir weights) and rebroadcast with a
  version.
- **Lane spacing from the detector's measured swath** (`detect_fov_deg` 28°),
  not the camera footprint: 4 m lanes, 8 per 30 m band (was 2 per band, and
  only 2 of 6 targets were found).
- **Detection only in steady flight over the area**: 6 m lead-in/run-out,
  gate open only over `[x_min, x_max]`, frames above 30°/s rotation dropped,
  the drone faces along a lane before reaching its start.
- **Attitude-aware projection** (level-flight put everything 0.5–0.8 m ahead);
  `pose_lag_s` 0.25 re-confirmed from data.
- **Separation**: right of way working > waiting > lower id; the yielding drone
  moves away horizontally at its own altitude (a vertical yield crossed a
  peer's RTL climb at 3.31 m); passes over/under only a peer holding its
  altitude.
- **Failsafes return home**: `COM_LOW_BAT_ACT 3` (default was warn only) next
  to `COM_OBL_RC_ACT 3`; an error is logged if PX4 lands away from home.
- **Calibration tooling**: `sightings_d<i>.csv` in every run (every box, with
  pose and attitude), `tools/analyse_sightings.py` to read it.

---

## 2. Honest state

**Weak / placeholder:**

- **The model.** COCO `person` from straight above is a stand-in. It is reliable
  only within ~2.5 m of the track at 10 m, which forces 4 m lanes and doubles
  flight time. Landmine weights will have a different swath - and therefore
  different lane spacing and flight time.
- **Flat ground.** The projection intersects the ray with flat ground at home
  altitude. Any slope is error proportional to height difference × off-nadir
  angle.
- **Onboard compute.** Each "Pi" is a process on an RTX 4060 (15 ms/frame at
  imgsz 1280). A Raspberry Pi CPU would not keep up; this needs an accelerator
  or a smaller model/`imgsz` - decide before buying hardware.
- Low-battery return is configured and alerted but not flown.
- A horizontal yield does not look at other peers or the area boundary.
- Hazard stamps are wall clock (real Pis need NTP/GPS time).

**Not started:** landmine dataset/weights · terrain following · hardware.

---

## 3. Next session — ordered plan

### Step 1 · Landmine weights (the software critical path)

Collecting and labelling nadir imagery is the bottleneck, not training. Even a
few hundred frames from the sim camera over landmine-like models beat COCO.
When weights exist:

1. Fly the calibration layout (`targets.csv` in the run dir - a lateral-offset
   sweep, a close pair, a boundary target; see `PROGRESS.md` §12.3).
2. `python3 tools/analyse_sightings.py` → set `detect_fov_deg` from where
   confidence stops being reliable, check `pose_lag_s`, check scatter against
   `min_sep_m`.
3. Re-check false positives on empty ground before touching `conf` (0.65 now;
   COCO's worst false positive was 0.61).

### Step 2 · Terrain following (software, independent of Step 1)

1. A sloped or bumpy Gazebo world, and a downward rangefinder on the model.
2. Fly a constant height above ground (PX4 terrain estimate, or offboard
   altitude from the rangefinder).
3. Project with the measured height under the camera instead of home altitude.
4. Re-run the calibration on the slope.

### Step 3 · Hardware (Track B) — 50 % of the grade, still at zero

CAD, fabricated parts, DFM + FEA/CFD report. Runs in parallel with everything
above and is the largest risk to the grade.

---

## 4. Pre-flight checklist

1. Run the launcher from a **normal terminal**, never inside tmux.
2. `bash ~/px4_ros_ws/tools/check_system.sh` - want 0 FAIL. By hand:
   `ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once`
   (`/fmu/out/*` is BEST_EFFORT; a default RELIABLE subscriber reports silence
   even when data is flowing).
3. Spawn targets after **every** sim start (`tools/add_target.sh`,
   `tools/add_swarm_targets.sh`) - GUI-inserted models die with the sim.
4. Any process touching a gz topic needs `GZ_IP=127.0.0.1`.
5. **One mission launch at a time.** A launch's detectors keep running after
   `VERIFY`; start a second launch and both share `/swarm/hazards` and write
   the same files. Stop the previous one first.
6. Stop everything with `bash ~/px4_ros_ws/tools/stop_sim.sh`
   (`tmux kill-server` alone leaves PX4 and Gazebo running).

---

## 5. Method notes worth keeping

- **A listed topic is not a published topic.** `ros2 topic list` counts
  subscribers. Use `ros2 topic hz`.
- **A loud warning is not automatically the cause.** The `dri2`/EGL errors cost
  hours; the real bug (`GZ_IP`) was silent.
- **Timing error flips with travel direction; a fixed offset does not.** Split
  along-track error by north- and south-bound lanes before touching `pose_lag_s`.
- **The camera seeing a target is not the detector finding it.** Measure the
  swath (confidence against distance from the track) before spacing lanes.
- **Log what you reject.** Below-threshold and dropped detections, with their
  pose, are what made every calibration in §12 possible.
- **Check a hypothesis before building on it.** "Attitude arrives late" looked
  like a perfect fit; measuring the arrival delay (≤ 18 ms) killed it in one
  flight and pointed at the real cause (turn transients).
