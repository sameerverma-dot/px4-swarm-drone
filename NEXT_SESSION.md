# Session log — 1 Sep 2026 · and the plan for next time

*Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)*
*Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali*

Companion docs: `SYSTEM_GUIDE.md` (how it works) · `STACK_README.md` (how to run) ·
`PHASE1_ROADMAP.md` (deliverables & grading) · `PROGRESS.md` (status log) ·
`CAMERA_DIAGNOSTIC.md` (the `GZ_IP` bug, as a case study).

---

## 0. Read this first

**The code has NOT been flown since it changed.** Everything in section 2 is
reasoned from flight data, replayed against that data, and unit-tested - but not
flight-verified. Section 4 step 1 is non-negotiable: re-fly before building on top.

Fastest way to see where you stand:

```bash
bash ~/px4_ros_ws/check_system.sh
```

It probes six layers (environment, processes, telemetry, camera, inference,
outputs) and prints PASS/FAIL per layer, so a failure names *which* layer broke.

---

## 1. What actually happened this session

### 1.1 The mission closed the loop

One command flew the entire Phase I software loop:

```
VERIFY PASS | waypoints 11/11 (OK) | returned=True | reason=returned
track CSV written: ~/maps/survey_track_1788287098.csv (2585 samples)
```

Exact command that produced it — kept here as history. `lane_spacing` was still
a manual guess then; it now derives itself, so the equivalent run today is the
same line without that argument (and gives 3 lanes, not 5):

```bash
ros2 launch survey mission.launch.py \
    x_max:=30.0 y_max:=20.0 altitude:=5.0 lane_spacing:=5.0
```

Survey → detect → geotag → RTL → verify, unattended. That is the milestone.

### 1.2 The root-cause bug that unblocked it

PX4 publishes `VehicleLocalPosition` on **`/fmu/out/vehicle_local_position_v1`**
and `VehicleStatus` on **`..._v4`**. Both nodes were subscribed to the
*unversioned* names, which on this build carry nothing. That single mismatch
caused every previous `arm/offboard timeout`, every `0 samples` track CSV, and
silently dead geotagging.

Two things made it hard to find, and both are worth remembering:

- `dds_topics.yaml` and the official `px4_ros_com` examples both use the
  unversioned name, so the code looked correct against the reference.
- `ros2 topic list` **showed** the unversioned name — but a *subscription* alone
  creates that entry. **A listed topic is not evidence of a publisher.**
  `ros2 topic hz <name>` is the test that actually distinguishes them.

Both nodes now subscribe to versioned *and* unversioned names, so they work
either way.

### 1.3 Geotag calibration — the axes were right all along

A `person` model at Gazebo `x=15, y=10` → PX4 `North=10, East=15`. Cross-referencing
`hazard_points.csv` against the 2585-sample track, for the two high-confidence hits:

| | recorded | truth | error |
|---|---|---|---|
| East (cross-track) | 14.53, 14.70 | 15.0 | **−0.47, −0.30 m** |
| North (along-track) | 4.53, 8.65 | 10.0 | −5.5, −1.4 m |

Cross-track accurate to under half a metre ⇒ **the pixel→ground axis mapping is
correct.** No `geo_swap_axes` / `geo_flip_*` flag is needed; they exist as
parameters but must stay `False`.

The along-track error is **latency, not geometry**. The proof: those two hits are
0.2 s apart yet place the same stationary person on *opposite* sides of the image
centre (`fwd` = −3.29 m then +2.94 m). That is impossible unless the two frames
were stale by different amounts. Measured ground speed on that lane was
**9.22 m/s**, so ~0.3 s of staleness = ~2.8 m of smear. Implied per-frame lag
worked out at 0.16 s and 0.52 s — i.e. real jitter, not a constant offset.

**Method worth reusing:** *consistent* error on one axis plus *sign-flipping*
error on the other is the signature of timing, not of a frame convention.

### 1.4 The detections were mostly junk

39 rows in `hazard_points.csv`; only **2** were the real target.

- 34 rows of `airplane` (17) / `kite` (16) / `bird` (1) at up to **0.85
  confidence**, on featureless grey ground. COCO `yolov8n` has never seen nadir
  imagery and hallucinates confidently.
- Confidence thresholding alone would not have saved it — the junk spanned
  0.26–0.85, overlapping the real hits (0.77, 0.89).
- **11 rows** were logged above 6 m during the RTL climb and descent, at up to
  **29.7 m** — far outside the survey area entirely.

### 1.5 The "drone went too high and circled" — not a bug

That was PX4's Return-to-Launch. QGC → Safety has *RTL Climb To 30.0 m*, so after
waypoint 11/11 the drone climbed 5 m → 30 m, flew home, and descended. Expected
behaviour. Lower it in QGC if it looks bad in the demo video.

---

## 2. What changed in the code (untested in flight)

| # | Change | Where | Why |
|---|---|---|---|
| 1 | **Pose time-matching.** Ring buffer of poses; geotag uses the pose at *frame arrival* − `pose_lag_s` (0.25), not the newest pose. | `detector_node.py` | Kills the latency error measured in §1.3 |
| 2 | **Speed cap.** `lookahead_m` (4.0) places the setpoint a fixed distance ahead instead of at the far end of the lane → ~3.8 m/s instead of 9.2. | `survey_node.py` | Less smear per frame; better coverage |
| 3 | **Detection gate.** `survey_node` publishes `Bool` on `/survey/detecting`, true only on real lanes. Gated-off frames skip inference entirely. | both | Kills the alt-29.7 m rows; frees GPU |
| 4 | **Class filter pushed into YOLO.** `classes:='person' conf:=0.40` are the new `mission.launch.py` defaults. | `perception.launch.py` | Kills the airplane/kite/bird flood |
| 5 | **Read-only buffer fix.** `img_to_np` returned a non-writable array on `bgr8`; `cv2.rectangle` would have thrown. | `detector_node.py` | Latent crash in `test_perception.py` |

Also: `std_msgs` added to both `package.xml` files; `PROGRESS.md` and
`SYSTEM_GUIDE.md` corrected (the versioned-topic entry previously stated the
*opposite* of the truth).

### 2.1 Replaying the real flight through the new code

Rather than guess at the improvement, the recorded detections were re-projected
using the pose at `(t − pose_lag)` instead of the newest pose. Recovered
per-frame lag was **0.59 s and 0.15 s** - genuine jitter, so no single constant
corrects both.

| config | RMS along-track error |
|---|---|
| as flown: 9.22 m/s, newest pose | **3.98 m** |
| lag compensation only (`pose_lag` 0.25) | 2.34 m |
| speed cap only (3.8 m/s, newest pose) | 1.64 m |
| **both — the new defaults** | **0.96 m** |

**The speed cap is the bigger lever**, which was not the original expectation:
compensation can only correct the *mean* lag, whereas flying slower shrinks the
error from whatever jitter remains. `pose_lag_s` default raised 0.15 → 0.25;
the RMS optimum is 0.37 but n=2 is far too thin to fit that tightly.

### 2.2 Second review pass — four more bugs

Found by re-reading the committed files rather than by flying:

- **Gating off inference also stopped republishing the annotated image**, so
  `/detection/image_annotated` would freeze during climb and RTL and look
  crashed. Now republishes the raw frame and skips only the inference.
- **Gate had no hysteresis** - a small altitude dip mid-lane flipped it
  closed/open, spamming the log. `reached_alt` is now latched. (Unit-tested:
  the unlatched version chatters where the latched one does not.)
- **`classes` with an unknown name** handed ultralytics an empty list, whose
  behaviour is not worth relying on. Falls back to `None` + a name filter.
- **No backstop** if a model's `names` dict disagrees with its output ids.

### 2.3 New: `lane_spacing` derives itself

`lane_spacing:=0.0` (now the default) computes `2·h·tan(HFOV/2)·(1−sidelap)`.
At 5 m that is **8.3 m**, not the 5.0 that was flown. It also *warns* when a
manually-passed spacing exceeds the footprint - that silently leaves
unphotographed gaps between lanes while still reporting `VERIFY PASS`.

### 2.4 New: the hazard map renderer

`ros2 run perception hazard_map` turns the two CSVs into `<name>.png` +
`<name>.geojson` (WGS84, opens in QGIS / Google Earth). Post-hoc `--classes`,
`--min-conf` and `--max-alt` filters let an OLD run be re-read under the new
rules without re-flying.

Rendering run 1788287098 made something visible that the CSV never did: **every
false positive sits exactly on a flight lane** - the signature of YOLO firing on
empty ground directly beneath the drone. Applying the new rules to that same
data takes 39 detections down to 4.

Also worth knowing: the nearest detection to the known target is an **airplane at
1.1 m** - closer than either real `person` hit. And even at `classes:=person
conf:=0.40`, two of the four survivors are still false positives. The class
filter is a demo crutch, not a fix.

---

## 3. Honest state of the system

**Proven by evidence:**
launcher · PX4↔ROS 2 DDS link · QGC · Gazebo camera → ROS 2 · YOLO on GPU ·
offboard boustrophedon survey with PASS/FAIL verification · RTL · pixel→ground
axis mapping · full mission in one command.

**Believed but not yet verified:**
all five changes in section 2.

**Known-weak / placeholder:**

- **The model.** COCO weights are a stand-in. This is a *dataset* problem.
- **~4.5 FPS during flight** vs ~11 FPS idle. Never diagnosed. Most likely
  Gazebo's renderer and YOLO contending for the same RTX 4060, *not* detector
  code — but that is a hypothesis, not a finding.
- **Geolocation assumes** level flight, perfect nadir, flat ground at home
  altitude. Fine for the arena; state the assumption in the report.

**Not started:**
multi-drone swarm · trained weights · the entire hardware track.

---

## 4. Next session — ordered plan

### Step 1 · Verify this session's fixes  *(~30 min — blocks everything)*

```bash
mv ~/maps/hazard_points.csv ~/maps/hazard_points_1788287.csv   # keep the old run
cd ~/px4_ros_ws
colcon build --packages-select survey perception && source install/setup.bash
bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down       # separate terminal
bash ~/px4_ros_ws/check_system.sh                              # all green before flying
ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=5.0
ros2 run perception hazard_map --area 0,30,0,20 --truth 10,15  # after it lands
```

Note there is no `lane_spacing` argument any more - it derives itself. Expect
**3 lanes instead of 5**, and a visibly slower flight (~3.8 m/s, not 9.2).

Check, in order:

1. `detection gate -> OPEN` appears **after** the climb, `closed` **before** RTL.
2. `HAZARD` lines are **only** `person`, and cluster near **N=10, E=15**.
3. No rows above ~6 m altitude in `hazard_points.csv`.
4. Lane ground speed ≈ 3.8 m/s — the flight will visibly take longer.
5. `VERIFY PASS` still appears (the speed cap must not break waypoint reaching).
6. `lane_spacing derived: footprint 11.85 m ... -> 8.30 m` in the log.
7. The rendered map puts the `person` cluster inside the known-target ring.

**If step 2 shows North still off by >2 m**, raise `pose_lag_s` toward 0.35 and
re-fly. Only if the *cross-track* (East) error goes bad should you touch the
`geo_*` flags — and it was accurate to 0.4 m, so that is unlikely.

### ~~Step 2 · Render the hazard map~~ — **DONE** (see §2.4)

### ~~Step 3 · Fix `lane_spacing`~~ — **DONE** (see §2.3)

### Step 2 · Swarm — 2 drones  *(the big one)*

This is the headline Phase I requirement and it is untouched. Approach:

1. Launch a second PX4 SITL instance with `-i 1` and a different
   `PX4_GZ_MODEL_POSE`. Instance index > 0 is expected to namespace the DDS
   topics as `/px4_1/fmu/...`.
2. **Verify that namespace with `ros2 topic hz`, do not assume it.** This session's
   whole lesson was that the documented topic name and the live one can differ.
3. Parameterise `survey_node` with a `namespace` prefix (currently hardcoded
   `/fmu/...`).
4. Split the area: simplest correct approach is to divide the rectangle along Y
   and give each drone a contiguous block of lanes — no inter-drone coordination
   needed, and it degrades gracefully to 1 drone.
5. Both drones write to the **same** `hazard_points.csv`. Check that the dedup
   list is per-node — two nodes appending concurrently will interleave rows and
   will *not* dedup against each other. Decide: one aggregator node, or a
   per-drone CSV merged afterwards. **Prefer the aggregator.**

Do 2 drones and make them solid before going to 5. The jump from 1→2 contains
every hard problem; 2→5 is mostly a loop.

### Step 3 · Start these in parallel, from day one

These two have **long lead times and low daily effort**, which is exactly why they
must start now rather than "next":

- **Hardware / Track B — 50 % of the grade, zero progress.** CAD, Make-fabricated
  parts, DFM + FEA/CFD report. No amount of software excellence compensates for
  half the rubric. This is the single largest risk to the grade in the whole
  project.
- **The dataset for landmine weights.** Collecting/labelling nadir aerial imagery
  is the bottleneck, not the training run. Even a few hundred labelled frames
  captured from your own sim camera would beat COCO. Starting this late means it
  will not be ready for the demo.

### Step 4 (optional) · Chase the FPS

Only if the demo looks bad. First measure whether Gazebo's real-time factor drops
when the detector runs — that would confirm GPU contention and point at capping
the camera `update_rate`, rather than at detector code.

---

## 5. Pre-flight checklist (saves 20 minutes every time)

1. Run the launcher from a **normal terminal**, never inside tmux.
2. `bash ~/px4_ros_ws/check_system.sh` — or, by hand, prove telemetry is alive:
   `ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once`
   (that form is unambiguous: `/fmu/out/*` is BEST_EFFORT, and a default RELIABLE
   subscriber matches nothing and reports silence even when data is flowing).
3. `commander` commands go in the **pxh>** pane, not a bash pane.
4. Any process touching a gz topic needs `GZ_IP=127.0.0.1`.
5. Move `hazard_points.csv` aside so runs stay comparable.
6. Stop everything with `tmux kill-server`.

---

## 6. Method notes worth keeping

- **A listed topic is not a published topic.** `ros2 topic list` counts
  subscribers. Use `ros2 topic hz`.
- **A loud warning is not automatically the cause.** The `dri2`/EGL errors cost
  hours; the real bug (`GZ_IP`) was silent. Twice now the noisy thing was innocent.
- **Consistent error on one axis + sign-flipping error on the other = timing,
  not frames.** That is what turned a guessed axis flip into a measured latency fix.
- **Reference examples can be wrong for your build.** `px4_ros_com` used the
  unversioned topic names; this PX4 does not publish them.
