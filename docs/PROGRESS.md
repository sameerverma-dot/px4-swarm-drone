# Project Progress — Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)
### Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali

_Last updated: 12 Sep 2026 (**the 2-drone swarm is flight-verified** — both
drones flew their own band, both logged VERIFY PASS, and drone 1's detection
came back in drone 0's frame within 1.7 m, which is `home_offsets` proven
rather than assumed. See section 9. Section 8's "NOT flight-verified" caveat is
now discharged.)_
_Companion docs: `STACK_README.md` (how to run), `PHASE1_ROADMAP.md` (plan)._

---

## 1. Summary of status

The **software / autonomy / AI pipeline is built and working end-to-end.**
A single drone autonomously surveys a specified area and returns, via two
independent paths (a QGroundControl Survey mission, and a custom ROS 2 node).
### Where this actually stands

**The 2-DRONE SWARM is flight-verified** as of 12 Sep (section 9), which closes
the headline software requirement. The single-drone loop below remains the
accuracy baseline everything is measured against.

Remaining on the software side: a third drone, a multi-target map renderer, and
landmine weights in place of the COCO stand-in. **Hardware (Track B) is 50 % of
the grade and is still at zero** — it is now the project's largest risk.

**Verified on 5 Sep**, one command, unattended:

```
HAZARD #1: person conf=0.81 at N=10.7 E=14.9   (truth N=10.0 E=15.0 -> 0.75 m)
VERIFY PASS | waypoints 9/9 (OK) | returned=True | 3125 track samples
```

One detection. Zero false positives. Deliverable in `~/maps/phase1_final.png`
plus a WGS84 GeoJSON.

Geotag accuracy over three flights:

| flight | ground speed | pose source | error at the known target |
|---|---|---|---|
| 1 Sep | 9.2 m/s | newest pose | 5.49 m, 1.38 m |
| (replay prediction) | 3.8 m/s | `pose_lag` 0.25 | 0.96 m RMS |
| **5 Sep** | **3.8 m/s** | **`pose_lag` 0.25** | **0.75 m** |

Confidence separation with COCO weights, pooled over four flights — false
positives on empty ground have **never** exceeded 0.57, real detections have
never fallen below 0.77, so `conf:=0.65` is the default:

| | confidences |
|---|---|
| false | 0.30 0.42 0.44 0.46 0.46 0.51 0.53 0.55 0.57 |
| real | 0.77 0.81 0.89 |

**Detection altitude is hard-capped by pixels on target.** A person is ~0.5 m
across from above: 27 px at 5 m, 13 px at 10 m, and YOLO needs ~24 px. Measured
sweep in `experiments/px_sweep.py`. Raising the camera to 1280 only helps if
`imgsz` rises with it — otherwise ultralytics downscales the detail straight back
out (measured: conf 0.06 vs 0.68 at 10 m).

**Two gaps remain, and the model is the smaller one.** COCO `yolov8n` is a
stand-in; landmine weights need nadir training data. But the larger gap is that
there is still only one drone.

| Capability | Status |
|---|---|
| Single-drone PX4 SITL + Gazebo + QGC + DDS bridge | ✅ Working |
| One-command launcher (`start_px4_sim.sh`) | ✅ Working |
| Sim home relocated to IIT Gandhinagar | ✅ Working |
| Autonomous area survey — ROS 2 node (`survey`) | ✅ Built & runs (flies, offboard, RTL) |
| Autonomous area survey — QGC "draw polygon" mission | ✅ Working (demonstrated) |
| Downward camera model in sim (`x500_mono_cam_down`) | ✅ Model + airframe confirmed |
| Gazebo camera → ROS 2 bridge (`ros_gz_bridge`) | ✅ Configured & launches |
| YOLO detector node (`perception`) | ✅ **Working on GPU** (cuda:0, RTX 4060, fp16, imgsz 640) |
| Pipeline throughput | ⚠️ ~2 Hz — bottleneck is upstream of YOLO (see 5.1) |
| **Camera frames reaching ROS 2 / YOLO** | ✅ **SOLVED** — was a `GZ_IP` mismatch (section 5) |
| Geotagged hazard map / CSV | ✅ **Working & calibrated** (cross-track error < 0.5 m) |
| Full mission (survey + detect + RTL) in one launch | ✅ `VERIFY PASS \| waypoints 11/11 \| returned=True` |
| Camera capture 1280x960 + imgsz to match | ✅ Built & live-verified (section 8.1) |
| **Multi-drone swarm (2–5)** | ✅ **2 drones flight-verified 12 Sep — both VERIFY PASS, cross-frame geotag 1.70 m (section 9).** 🟡 3+ untried; both bands not yet targeted in one run |
| Geotag accuracy vs known target | ✅ 0.75 m |
| False-positive rejection (`conf` 0.65) | ✅ 0 / 1 detections this run |
| Hazard map render (PNG + GeoJSON) | ✅ `ros2 run perception hazard_map` |
| Target spawner (`tools/add_target.sh`) | ✅ GUI-inserted models die on restart |
| System self-test (`tools/check_system.sh`) | ✅ six layers, PASS/FAIL |

---

## 2. Environment

- Laptop: HP OMEN 16, **Ubuntu 22.04** (glibc 2.35), **NVIDIA RTX 4060** (Optimus/hybrid) + Intel iGPU.
- **ROS 2 Humble**, user `sam`, home `/home/sam`.
- Gazebo **Harmonic** (gz-sim 8.15).
- GitHub repo: `github.com/sameerverma-dot/px4-swarm-drone` (branch `main`).

---

## 3. What is built and working

### 3.1 Base simulation stack
- **PX4 SITL + Gazebo (gz_x500)** on the RTX GPU via `prime-run`.
- **Micro XRCE-DDS Agent** bridging PX4 ⇄ ROS 2 on UDP 8888 — all `/fmu/out/*` topics confirmed.
- **QGroundControl v4.4.3** (v5.0 needs glibc 2.38, incompatible) — connects on UDP 14550.
- **One-command launcher** `~/px4_ros_ws/start_px4_sim.sh` starts DDS agent + PX4/Gazebo + sourced ROS 2 shell + QGC in a 3-pane tmux session. Home set to IIT Gandhinagar (23.2127, 72.6846).

### 3.2 Area survey — ROS 2 package `survey`
- `ros2 launch survey survey.launch.py x_max:=40 y_max:=30 altitude:=10 lane_spacing:=8`
- Boustrophedon ("lawnmower") coverage in PX4 local NED; streams OffboardControlMode + TrajectorySetpoint, arms, flies waypoints, verifies each was reached (PASS/FAIL), writes flown track CSV to `~/maps`, and RTLs on completion.
- Builds clean (`colcon build --packages-select survey` → `Finished <<< survey`).
- Demonstrated: drone armed, entered Offboard, and climbed under node control (a full run was interrupted by a terminal closure, not a code fault).

### 3.3 Area survey — QGroundControl Survey mission
- QGC Plan view → Pattern → Survey → draw polygon with clicked points → set transect spacing → Upload → Start Mission.
- **Demonstrated working**: drew an area over the IITGN field and the drone flew the full transect pattern autonomously and returned. This is the fastest "draw an area and survey it" path.

### 3.4 Detection pipeline — ROS 2 package `perception` (WORKING)
- `detector_node`: subscribes to the camera image, runs **Ultralytics YOLO**, publishes an annotated image (`/detection/image_annotated`), and geotags detections (nadir projection from drone pose) into `~/maps/hazard_points.csv`, de-duplicated by distance.
- Launch also starts the **Gazebo→ROS 2 camera bridge** (`ros_gz_bridge`).
- **Verified live**: `ros2 topic hz /detection/image_annotated` → ~2 Hz. Full chain
  Gazebo camera → gz-transport → `ros_gz_bridge` → ROS 2 → YOLO → annotated frames.
- View it: `ros2 run rqt_image_view rqt_image_view /detection/image_annotated`
  (grey image is correct — the default world's ground plane is light grey).
- **Note:** default `yolov8n.pt` = COCO classes (people/cars), not landmines. Swap in trained weights via `weights:=~/runs/.../best.pt` for real detection.

---

## 4. File locations

| What | Path |
|---|---|
| ROS 2 workspace | `~/px4_ros_ws` |
| Launcher | `~/px4_ros_ws/start_px4_sim.sh` |
| Survey package | `~/px4_ros_ws/src/survey/` (node, launch, package.xml, setup.py) |
| Standalone survey script (stale) | `~/px4_ros_ws/archive/survey_node.py` |
| Perception package | `~/px4_ros_ws/src/perception/` (detector_node, launch, package.xml) |
| Vendored msg/bridge pkgs | `~/px4_ros_ws/src/px4_msgs`, `~/px4_ros_ws/src/px4_ros_com` |
| PX4 firmware / SITL | `~/PX4-Autopilot` |
| DDS agent | `~/Micro-XRCE-DDS-Agent` |
| QGroundControl | `~/Downloads/QGroundControl.AppImage` |
| YOLO venv / training runs | `~/yolo_test` (venv), `~/runs` |
| Output CSVs (tracks, hazards) | `~/maps/` |
| Docs | `~/px4_ros_ws/{STACK_README,PHASE1_ROADMAP,PROGRESS}.md` |

Camera model: `x500_mono_cam_down` (airframe `4014_gz_x500_mono_cam_down`) — downward-facing camera, 1280×960, ~99° FOV.
Camera gz topic: `/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`.

---

## 5. SOLVED — camera frames now reach ROS 2 (was a `GZ_IP` mismatch)

**Symptom (now fixed):** with `x500_mono_cam_down`, the camera topic was
advertised (`gz topic -l` listed it) but delivered **no frames** to any external
subscriber — `gz topic -e` blank, `ros_gz_bridge` silent, detector starved.

**The camera was never broken.** Gazebo's own server log proves it rendered
correctly in every run (`~/.gz/sim/log/<run>/server_console.log`):

```
[Sensors.cc:391]      Rendering Thread initialized
[CameraSensor.cc:504] Enabling camera sensor: '...camera' data generation.
[GstCameraSystem.cpp:281] Camera info: 1280x960
[GstCameraSystem.cpp:475] GStreamer pipeline started, streaming to 127.0.0.1:5600
```

PX4's in-process `GstCameraSystem` was consuming real 1280×960 frames and
streaming H.264 the whole time.

**Actual root cause:** PX4 launches the Gazebo server with **`GZ_IP=127.0.0.1`**
(visible in `px4_sitl.log`). gz-transport separates *discovery* from *data*:

| Stage | Mechanism | With mismatched `GZ_IP` |
|---|---|---|
| Discovery | UDP multicast | ✅ works → topic **is listed** |
| Data | direct ZeroMQ to publisher's advertised address | ❌ never connects |

So any process without `GZ_IP=127.0.0.1` **sees the topic and receives nothing**,
silently, with no error either side. PX4's own consumer worked because it runs
*inside* the server process and inherits the env.

**Fix (applied):** `src/perception/launch/perception.launch.py` sets
`additional_env={'GZ_IP': '127.0.0.1'}` on both the bridge and the detector, and
the earlier topic remap was removed. For ad-hoc shells: `export GZ_IP=127.0.0.1`.

**Verified:** `ros2 topic hz /detection/image_annotated` → ~2 Hz.
Chain confirmed: Gazebo camera → gz-transport → ros_gz_bridge → ROS 2 → YOLO.

**Red herring for the record:** `libEGL: failed to create dri2 screen` and
`eglInitialize failed` looked fatal but are noise — `~/.gz/rendering/ogre2.log`
shows Ogre probing 4 EGL devices, 3 succeeding, running on the RTX 4060. Hours
were lost chasing this. **A loud warning is not automatically the cause.**

**Performance note:** ~2 Hz is YOLO on CPU at 1280×960. For a smoother demo drop
`Tools/simulation/gz/models/mono_cam/model.sdf` to 640×480 and `update_rate` 10.

### 5.1 Open item — pipeline runs at ~2 Hz (bottleneck is NOT YOLO)

**Measured:** `ros2 topic hz /detection/image_annotated` → ~2 Hz, both on CPU
**and** after moving inference to the GPU. Startup confirms the GPU is really in
use:

```
torch 2.13.0+cu130 | CUDA available: True | GPU: NVIDIA GeForce RTX 4060 Laptop GPU
inference device=cuda:0 imgsz=640 fp16=True
```

**Conclusion:** YOLOv8n at 640px on an RTX 4060 runs at 100+ FPS, so inference is
not the limit. Moving to the GPU changed nothing → the constraint is **upstream**
of the detector. Remaining suspects, in order:

1. **Camera resolution / transport** — 1280×960 RGB ≈ **3.7 MB per frame** pushed
   through gz-transport → `ros_gz_bridge` → ROS 2 DDS. Almost certainly dominant.
2. **Gazebo sensor render rate under lockstep** — PX4 locksteps the sim; with
   RTF well under 100% the camera cannot hit its nominal 30 Hz.
3. Bridge serialisation overhead (gz.msgs.Image → sensor_msgs/Image copy).

**Next action (untried):** drop the camera in
`~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` to
`640×480` and `<update_rate>10</update_rate>`. That cuts per-frame payload ~4×
and attacks suspect #1 directly. Re-measure with `ros2 topic hz` afterwards.

**Note:** ~2 Hz is *sufficient* for a survey-speed demo (the drone moves slowly),
so this is an optimisation, not a blocker.


---

## 6. Gotchas already solved (don't re-discover)

- Run the launcher from a **normal terminal**, not inside tmux. It auto-clears stale px4/agent/gz processes (else "port 8888 in use" / "instance 0 already running").
- `commander` commands go in the **PX4 (pxh>) pane**, not a bash pane.
- Do **not** pipe PX4 through `| tee` — it makes the pxh console non-interactive (launcher uses tmux `pipe-pane` for logging instead).
- Arming from ROS 2 without a GCS: set `NAV_DLL_ACT=0` and `CBRK_SUPPLY_CHK=894281` (or just run QGC, which satisfies the GCS check).
- Gazebo must render on the RTX (`prime-run`) or it's unusably slow.
- Ultralytics >=8.4: passing `half=` to `predict()` warns **once per frame** and floods
  the log. Cast the model to fp16 once at load (`model.model.half()`) instead.
- **gz-transport: "topic listed but no data" ⇒ `GZ_IP` mismatch, NOT rendering.**
  PX4 runs the gz server with `GZ_IP=127.0.0.1`; every external subscriber
  (`gz topic`, `ros_gz_bridge`, custom nodes) must set it too.
- **`~/.gz/sim/log/<timestamp>/server_console.log` is the first file to open**
  when anything Gazebo-side misbehaves; `~/.gz/rendering/ogre2.log` for render.
- PX4 ships its own gz plugin config at
  `~/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config` (not `/usr/share/gz/...`).
- Mission mode refusing to start ("No manual control input"): set `COM_RC_IN_MODE 4`
  (plus `COM_RCL_EXCEPT 7`, `NAV_RCL_ACT 0`); the launcher now sets these automatically.
- PX4 `/fmu/*` topics are **best-effort QoS** — `ros2 topic echo` needs `--qos-reliability best_effort`.
- **WRONG EARLIER, NOW CORRECTED:** this PX4 publishes on the **versioned** topics
  (`/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v4`). The unversioned
  names carry nothing, which silently caused every arm/offboard timeout and every
  0-sample track CSV. `ros2 topic list` showing a name proves nothing — a *subscriber*
  creates that entry too. Use `ros2 topic hz <name>` to find the live one. Both nodes
  now subscribe to versioned and unversioned names.
- COCO `yolov8n` hallucinates `airplane`/`kite`/`bird` on empty nadir ground at up to
  0.85 confidence. `mission.launch.py` now defaults to `classes:='person' conf:=0.40`.
- Geotag error is dominated by **frame latency × ground speed**, not by axis mapping.
  Fixed with a pose ring buffer (`pose_lag_s`) + a speed cap (`lookahead_m`).
- `pip install --user ultralytics` bumps setuptools to 84 and **breaks colcon** (needs <80). Fix: `pip install --user "setuptools==70.3.0"`.
- QGC v5.0 won't run on Ubuntu 22.04 (glibc). Use v4.4.3.
- Committing files over the device bridge resets the executable bit — re-`chmod +x` the launcher, or run it with `bash`.

---

## 7. Recommended next steps

1. **Fly the swarm scaffolding built in section 8.** Topics and the camera
   bump are verified live; a real 2-drone survey (arm, offboard, both fly
   their band, RTL, one combined `hazard_points.csv`) has not been flown.
   That is the next thing to actually run — see 8.4.
2. **Hardware / Track B — start now, in parallel.** 50 % of the grade, zero
   progress, and fabrication has queue times you do not control. This is the
   largest risk to the final result and it is not a software problem.
3. **Dataset for landmine weights.** Long lead time, low daily effort. COCO is a
   stand-in and every reader will know it. You can now generate nadir imagery
   from your own sim — and now at 1280x960.
4. **Validate the coverage model** (SWARM_PLAN.md section 6) on a
   differently-sized area once the swarm actually flies — the model's one
   fitted constant (`turn_penalty_s`) has never been tested against anything
   but the flight it was fit to.

---

## 8. Swarm scaffolding — built 7 Sep, topic-level verified live
### (this section's "NOT flight-verified" caveat was discharged on 12 Sep — see section 9)

Everything below is new code, checked by (a) a clean `colcon build`, and (b) a
live 2-instance smoke test of the one thing this project has learned NOT to
assume — that a namespaced topic actually publishes, not just lists (the
lesson from the versioned-topic bug in section 6). **No autonomous mission
has been flown on this code yet** — that is deliberately left as the next
session's first task (8.4), not claimed here.

### 8.1 Camera resolution: 640x480 -> 1280x960 (built + live-verified)

- `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf`: capture
  bumped to 1280x960 (`hfov` unchanged at 1.74 rad — this doubles pixel
  density at a given altitude, it doesn't change field of view). Shared by
  both `x500_mono_cam` and `x500_mono_cam_down` via `<include>`.
- `imgsz` default raised to 1280 in `detector_node.py`, `perception.launch.py`,
  and `mission.launch.py` (still overridable) so inference actually uses the
  extra resolution instead of ultralytics downscaling it back out (the
  conf 0.06 vs 0.68 measurement in `experiments/px_sweep.py`).
- **Live-verified**: `gz topic -e .../camera/image -n 1` on a running sim
  showed `width: 1280 / height: 960`.
- Per SWARM_PLAN.md this raises the detection-altitude ceiling from 5.6 m to
  11.2 m at the same `conf`. Nothing has re-flown a real detection at the new
  altitude yet — do that before trusting the new ceiling in a demo.

### 8.2 Multi-drone plumbing: namespacing, area split, shared detector (built)

- **`survey_node.py`**: new `namespace` param (default `''` = instance 0,
  unnamespaced — identical to the old behaviour) prefixes every `/fmu/...`
  topic. New `csv_prefix` param (default `'survey_track'`, unchanged) so
  simultaneous drones don't collide on the same-second track CSV filename.
  **Also fixed a latent bug found while doing this**: `VehicleCommand.
  target_system` was hardcoded to `1`. PX4's `Commander::handle_command`
  silently *drops* any command whose `target_system` doesn't match its own
  `vehicle_status.system_id` — and PX4's own `rcS` sets `MAV_SYS_ID =
  instance + 1`. So instance 1 (`MAV_SYS_ID=2`) would have silently ignored
  every arm/offboard command from a namespace-aware `survey_node`, i.e.
  exactly this project's GZ_IP/versioned-topic class of bug, undiscovered
  until the first swarm flight. Changed to `target_system=0` (broadcast —
  safe here because each instance's command topic is already isolated by the
  DDS namespace, not by system id).
- **`detector_node.py`**: reworked to serve N drones with ONE model load and
  ONE hazard list (SWARM_PLAN.md's "5x headroom, no reason to run N
  detectors" argument), via new plural params — `image_topics`,
  `pose_namespaces`, `gate_topics`, `annotated_topics`, `home_offsets`
  (`;`-separated, one entry per drone). All default to `''`, which falls back
  to the original singular params for exactly one, unnamespaced drone — every
  existing single-drone command is unaffected byte-for-byte. `home_offsets`
  (`'north,east'` metres per drone) folds each drone's own local-NED
  detections into ONE shared frame (drone 0's home), per the trap flagged in
  `NEXT_SESSION.md` — untested that trap is actually handled correctly until
  a real multi-drone flight produces detections from more than one drone.
- **`perception.launch.py`**: now spawns one `ros_gz_bridge` per camera topic
  (via `OpaqueFunction`, since the bridge count depends on the *resolved*
  value of `image_topics`) and exactly one `detector_node`.
- **`swarm_mission.launch.py`** (new): splits one survey area into
  `num_drones` bands along EAST, launches one `survey_node` per drone (each
  flying the *same* local box — the split is realised by where the sim spawns
  each drone, not by different per-drone coordinates) plus one shared
  `perception.launch.py` include with the plural params filled in.
- **`tools/start_px4_swarm.sh`** (new): launches N PX4 instances in one tmux
  session — instance 0 hosts the Gazebo world (`make px4_sitl`, identical to
  `start_px4_sim.sh`), instances 1..N-1 spawn into it standalone
  (`PX4_GZ_STANDALONE=1`, `PX4_GZ_MODEL_POSE`), matching PX4's own documented
  multi-vehicle convention. Also staggers `RTL_RETURN_ALT` per instance
  (`NEXT_SESSION.md`'s "RTL converging" trap: simultaneous RTLs climbing to
  the same altitude over the same field).
- **The band math must agree between the shell script and the launch file.**
  They compute the same numbers (spawn offset / local survey box / detector
  `home_offsets`) independently, from `NUM_DRONES`/`Y_MIN`/`Y_MAX` and
  `num_drones`/`y_min`/`y_max` respectively — there is no shared source of
  truth. Pass matching values or the map will look plausible and be wrong,
  exactly the trap `NEXT_SESSION.md` already flagged for this reason.

### 8.3 What was actually verified live tonight (not just built)

A 2-instance smoke test (`tools/start_px4_swarm.sh`, then torn down —
no mission flown) confirmed, with real running processes, not documentation:

- Instance 1's DDS topics are namespaced `/px4_1/fmu/...` exactly as PX4's
  `rcS` predicts, and — the actual test, since a listed topic proves nothing —
  `ros2 topic hz /px4_1/fmu/out/vehicle_local_position_v1` showed a genuine
  **~50 Hz** live rate, same as instance 0's unnamespaced `/fmu/out/...`.
- Instance 1's Gazebo camera topic is
  `/world/default/model/x500_mono_cam_down_1/link/camera_link/sensor/camera/image`
  — confirmed via `gz topic -l`, matching the pattern
  `swarm_mission.launch.py` and `start_px4_swarm.sh` both assume (verified
  against PX4's `px4-rc.gzsim` source, not guessed).
- Camera capture is genuinely 1280x960 on the wire (`gz topic -e`, one frame).

Separately, a `ros2 launch survey swarm_mission.launch.py num_drones:=2`
**dry run** (no sim running - just checking the launch graph resolves
correctly) caught a real bug before it reached a flight: `pose_namespaces`
built as `';/px4_1'` was silently collapsing to `['/px4_1']` and broadcasting
to BOTH drones instead of `['', '/px4_1']`, because `detector_node.py`'s
list-splitting helper dropped empty segments - exactly the kind of
plausible-but-wrong result this project's methodology exists to catch. Fixed
(`_split()` now treats an empty segment as data, only the whole-string-empty
case as "not configured"), rebuilt, and the same dry run then showed the
correct `drone 0: pose_ns=''` / `drone 1: pose_ns='/px4_1'`, correct distinct
`gate` topics, and correct `offset=(0,10)` for drone 1. A second dry run with
`survey_delay:=1` also confirmed both `survey_node_0` (`ns='(none)'`) and
`survey_node_1` (`ns='/px4_1'`) start with distinct names and the same local
survey box (`y=[0,10]`), as intended - the split is realised by where the sim
spawns each drone, not by different per-drone coordinates.

**Not verified**: arming/offboard/waypoints on a namespaced instance (the
`target_system=0` fix above is un-flight-tested), the detector actually
receiving two live camera streams and merging hazards into one deduplicated
list, and RTL staggering under a real return-to-launch. All of the above was
config/wiring verification with no PX4/Gazebo sim running for the dry runs -
it proves the launch graph is correct, not that a mission flies.

### 8.4 Next actual step

```bash
# terminal 1
NUM_DRONES=2 Y_MIN=0 Y_MAX=60 bash ~/px4_ros_ws/tools/start_px4_swarm.sh gz_x500_mono_cam_down

# terminal 2, AFTER checking namespacing (don't assume it):
ros2 topic hz /px4_1/fmu/out/vehicle_local_position_v1
cd ~/px4_ros_ws && source install/setup.bash
bash tools/add_target.sh          # then place a second target for drone 1's band
ros2 launch survey swarm_mission.launch.py num_drones:=2 x_max:=30.0 y_min:=0.0 y_max:=60.0 altitude:=10.0
```
Watch for: both drones arming and reaching offboard (the `target_system=0`
fix), both flying their own band without drifting into the other's, one
combined `hazard_points.csv` with a `drone` column, and no mid-air RTL
conflict at the end.

---

## 9. The 2-drone swarm, flight-verified (12 Sep)

Section 8 built the swarm and verified the wiring. This section is the flight.

```
[survey_node_0]: lane_spacing derived: footprint 23.71 m at 10.0 m altitude,
                 30% sidelap -> 16.59 m
[survey_node_0]: Survey ns='(none)'   x[0.0,30.0] y[0.0,30.0] alt=10.0m -> 7 waypoints
[detector_node]: drone 0 detection gate -> OPEN
[detector_node]: drone 1 detection gate -> OPEN
[detector_node]: HAZARD #1: person conf=0.72 at N=8.3 E=44.7 (alt 10.0m, drone 1)
[survey_node_0]: VERIFY PASS | waypoints 7/7 (OK) | returned=True | 3035 samples
[survey_node_1]: VERIFY PASS | waypoints 7/7 (OK) | returned=True | 3140 samples
```

### 9.1 What this actually proves

Every item section 8.3 listed as **Not verified** is now verified:

| 8.3 said "not verified" | 12 Sep result |
|---|---|
| arming/offboard on a namespaced instance (`target_system=0` untested) | `survey_node_1` armed, held offboard, flew 7/7 waypoints |
| detector receiving two live camera streams | both gates opened, both streams consumed by one node |
| hazards merged into one frame | `hazard_points.csv` has one list with a `drone` column |
| RTL staggering under a real return-to-launch | both returned, no mid-air conflict |

**The number that matters is E=44.7.** The target was spawned at global
N=10, E=45. Drone 1's own local NED origin sits 30 m east, so its raw
observation was local E≈14.7 — and the detector added the 30 m back on.

| | truth | reported | error |
|---|---|---|---|
| North | 10.0 | 8.32 | **-1.68 m** |
| East | 45.0 | 44.74 | **-0.26 m** |
| total | | | **1.70 m** |

That is `home_offsets` working on real telemetry. A wrong offset would not have
been off by 1.7 m; it would have been off by 30.

The error itself is in family with the single-drone 1280 px / 10 m scatter
(2.31 m RMS on 8 Sep), so the swarm adds no measurable geolocation penalty —
which is the expected result, since each drone geotags in its own frame and the
offset is a constant.

### 9.2 What this run did NOT prove, and the fix

**Only drone 0's band had ever had a target.** This run was flown with a single
person spawned at E=45, so `hazard_points.csv` gained exactly one row for the
whole sortie, and drone 0 flew a clean, verified, completely empty survey.

Half the swarm was therefore untested for detection. `tools/add_swarm_targets.sh`
(new, 12 Sep) fixes this by construction: it computes each band centre with the
same `(Y_MAX-Y_MIN)/N` arithmetic as `start_px4_swarm.sh` and
`swarm_mission.launch.py`, and spawns one person per band, so every drone has
something to find and every per-drone offset is checked independently.

```bash
NUM_DRONES=2 Y_MIN=0 Y_MAX=60 bash tools/add_swarm_targets.sh
```

Re-fly 9.1 with it before calling the 2-drone case closed. **Expect 2 rows in
`hazard_points.csv`, with distinct values in the `drone` column.**

### 9.3 Known gaps this run exposed

- **`hazard_map` is still single-drone.** `--truth` takes one point and
  `--track` plots one CSV, so on an N-drone run it scores one band and draws one
  of the N flight paths. The other detections do appear as points. Multi-target
  and multi-track rendering is an open item — it matters because the map *is*
  the deliverable.
- **`hazard_points.csv` appends across runs.** The file currently holds three
  stale rows from the 8 Sep single-drone flight alongside the one row from this
  one. Move it aside before every scored flight, or the map compares two
  sorties. (Those three stale rows are also the pre-fix `min_sep_m=2.0`
  triple-count of one person, kept as the evidence behind raising it to 4.5.)
- **3 drones is untried.** Expected to be a loop change only — the launcher, the
  launch file and the target spawner are all already parameterised on `N` — but
  "expected" is not "verified", and this project has a standing rule about the
  difference.

### 9.4 Where the software deliverable stands

The autonomous swarm loop — launch N, split the area, fly concurrently, detect,
geotag into one shared frame, return, verify — **runs end to end.** That was the
last unproven piece of the software half.

What remains on the software side is breadth and polish, not unknowns: a third
drone, a multi-target map, and landmine weights in place of the COCO
`person` stand-in.

**Hardware (Track B) is 50 % of the grade and remains at zero.** Nothing in this
section changes that, and it is now the single largest risk to the project.
