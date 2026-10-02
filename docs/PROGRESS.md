# Project Progress — Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)
### Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali

_Last updated: 2 Oct 2026 (**decentralised 3-drone swarm, flight-verified**: each
drone flies, detects and logs on its own; peers exchange heartbeats and hazard
reports directly; an early-returning or silent drone's lanes are taken over by
its neighbour; band-boundary detections are de-duplicated drone-to-drone; peers
keep separation (closest approach 5.19 m in a forced conflict). Section 11.)_
_Companion docs: `STACK_README.md` (how to run), `PHASE1_ROADMAP.md` (plan)._

---

## 1. Summary of status

The **software / autonomy / AI pipeline is built and working end-to-end.**
A single drone autonomously surveys a specified area and returns, via two
independent paths (a QGroundControl Survey mission, and a custom ROS 2 node).
### Where this actually stands

**A DECENTRALISED 3-DRONE SWARM is flight-verified** as of 2 Oct (section 11):
each drone flies, detects and logs on its own; drones exchange heartbeats and
hazards peer to peer; early-returning or silent drones have their lanes taken
over by a neighbour; peers keep separation. That closes the headline software
requirement. The single-drone loop below remains the accuracy baseline
everything is measured against.

Remaining on the software side: landmine weights in place of the COCO stand-in
(it is now the weakest link, ~88 % per pass), and a compute budget for real
onboard detection (section 11.5). **Hardware (Track B) is 50 % of
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
| Pipeline throughput | ✅ every frame inferred, ~8 fps **per drone with 3 onboard detectors**, pose 50 Hz (11.4) |
| **Camera frames reaching ROS 2 / YOLO** | ✅ **SOLVED** — was a `GZ_IP` mismatch (section 5) |
| Geotagged hazard map / CSV | ✅ **Working & calibrated** (cross-track error < 0.5 m) |
| Full mission (survey + detect + RTL) in one launch | ✅ `VERIFY PASS \| waypoints 11/11 \| returned=True` |
| Camera capture 1280x960 + imgsz to match | ✅ Built & live-verified (section 8.1) |
| **Multi-drone swarm (2–5)** | ✅ **3 drones, decentralised, 2 Oct — onboard autonomy, band takeover (early return + silent drone), shared hazard list, peer separation, all flight-verified (section 11)** |
| Geotag accuracy vs known target | ✅ 0.75 m |
| False-positive rejection (`conf` 0.65) | ✅ 0 / 1 detections this run |
| Hazard map render (PNG + GeoJSON) | ✅ `ros2 run perception hazard_map` |
| Target spawner (`tools/add_target.sh`) | ✅ GUI-inserted models die on restart |
| System self-test (`tools/check_system.sh`) | ✅ six layers, PASS/FAIL — auto-detects the PX4 instances and probes **every** drone (10.5) |
| Teardown (`tools/stop_sim.sh`) | ✅ kills by process and verifies; `tmux kill-server` alone does not (10.5) |
| Per-band target spawner (`tools/add_swarm_targets.sh`) | ✅ one target per drone band |

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

### 5.1 ~~Open item~~ SOLVED 27 Sep — pipeline runs at ~2 Hz (bottleneck is NOT YOLO)

> **Resolved in section 10.3.** The right half of this section's conclusion
> (not YOLO) was correct; the suspects below were not. It was never transport:
> `np_to_img()` assigned `bytes` to `Image.data`, which rclpy validates element
> by element in Python - 296 ms per 1280x960 frame against 5 ms for an
> `array('B')`. The history is kept below as a record of the reasoning.

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

## 9. The 2-drone swarm — flight-verified 16 Sep, completed 21 Sep

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

**(CLOSED 21 Sep - see 9.5.)** Only drone 0's band had ever had a target. This run was flown with a single
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


### 9.5 CLOSED — both bands detected, 21 Sep

Section 9.2 said the 16 Sep flight had only ever put a target in one band, so
half the swarm was untested. That is now closed. With `tools/add_swarm_targets.sh`
placing one person per band:

```
HAZARD #1: person conf=0.82 at N=14.1 E=15.6 (alt 10.0m, drone 0)
HAZARD #2: person conf=0.67 at N=14.5 E=44.9 (alt 10.0m, drone 1)
survey_node_0: VERIFY PASS | waypoints 7/7 (OK) | returned=True | 3030 samples
survey_node_1: VERIFY PASS | waypoints 7/7 (OK) | returned=True | 3105 samples
```

| drone | truth (N,E) | reported | error |
|---|---|---|---|
| 0 | 15.0, 15.0 | 14.1, 15.6 | **1.08 m** |
| 1 | 15.0, 45.0 | 14.5, 44.9 | **0.51 m** |

Both drones flew, both detected in their OWN band, and both hits landed in drone
0's frame. Drone 1's raw observation was local E about 14.9; `home_offsets` added
the 30 m. Accuracy is at or better than the single-drone 1280 px baseline, so the
swarm costs nothing in geolocation - expected, since each drone geotags in its
own frame and the offset is a constant.

### 9.6 The bug that cost 21 Sep: GZ_IP, again

Three consecutive runs had drone 1 boot blind - `No valid data from Accel 0`,
`barometer 0 missing`, `Found 0 compass`, then `Arming denied: Resolve system
health failures first` - while `/px4_1/fmu/out/vehicle_local_position_v1` was
LISTED (the DDS writer existed) and silent (EKF2 never had inputs).

Gazebo was innocent. `gz topic -e` on
`/world/default/model/x500_mono_cam_down_1/link/base_link/sensor/imu_sensor/imu`
returned real data at full rate. The sensor published; PX4 did not receive it.

Root cause: `tools/start_px4_swarm.sh` launched the standalone instances without
`GZ_IP=127.0.0.1`. On this machine gz-transport discovery works without it and
data delivery does not - exactly the distinction `check_system.sh` has always
measured in two separate probes ("camera advertised" vs "camera DELIVERING
frames with GZ_IP=127.0.0.1"), and exactly the bug documented in
`CAMERA_DIAGNOSTIC.md`. Instance 0 was unaffected because it starts the gz
server itself. Measured: `gz topic -l` returns 36 topics bare, 51 with GZ_IP.

Fixed by adding `GZ_IP=127.0.0.1` to every standalone instance and to the
launcher's ROS pane.

**Two process lessons, both expensive:**

1. Four hypotheses were tried before the right one - stale processes, a startup
   race, a Gazebo spawn defect, transport. Each was consistent with the evidence
   available at the time and the first three were wrong. What settled it was one
   measurement (`gz topic -e` on the suspect sensor) that should have been taken
   first. **Measure the link that is actually suspect before theorising about it.**
2. `check_system.sh` reported "21 passed, 0 failed - Everything probed is
   working" while drone 1 was dead. Its telemetry layer only probes `/fmu/out/*`,
   i.e. instance 0. A self-test that cannot see the swarm cannot protect the
   swarm. ~~OPEN: make check_system.sh take NUM_DRONES and probe /px4_i/ for
   each instance.~~ **Done 27 Sep (section 10.5).**

### 9.7 Where the software deliverable stands

The autonomous swarm loop — launch N, split the area, fly concurrently, detect,
geotag into one shared frame, return, verify — **runs end to end.** That was the
last unproven piece of the software half.

What remains on the software side is breadth and polish, not unknowns: a third
drone, a multi-target map, and landmine weights in place of the COCO
`person` stand-in.

**Hardware (Track B) is 50 % of the grade and remains at zero.** Nothing in this
section changes that, and it is now the single largest risk to the project.

---

## 10. The 2-drone swarm made reliable — 27 Sep

Section 9.5 closed the swarm on one flight where drone 1 scored 0.67 against a
0.65 threshold. The four swarm runs of 26 Sep then logged **zero** drone 1
detections, while drone 0 hit 3 of 4 - with identical geometry (both drones pass
1.6 m from their target). This section is the diagnosis and the fixes. Every
claim below was measured on the running stack, not inferred.

### 10.1 Evidence first: a per-drone health line

The detector could not answer "why did drone 1 find nothing": it logged only
hits above threshold, and its no-pose warning fired only when a detection
coincided with every 50th frame - effectively never. It now logs, every 5 s while
a gate is open:

```
drone 1 [OPEN] camera 9.6 fps, inferred 9.2 fps, pose 50 Hz, best person 0.78
        (below-threshold best 0.61), hits 1 | ms/frame: total 27, yolo 14 (...)
```

YOLO runs at a 0.25 diagnostic floor so near-misses are visible; only boxes
>= `conf` are drawn or recorded. `DROPPED n no-pose` and `NO CAMERA FRAMES`
are logged as warnings naming the drone.

### 10.2 Bug: drone 1's hits were silently dropped (pose sampled at 1 Hz)

The first instrumented flight showed drone 1 detect its target at **0.65 - and
the detection DROPPED, no pose**. Measured: the "50 Hz" pose ring buffer was
filling at **1 Hz** for both drones. On a single-threaded executor no pose
callback can run while YOLO is busy, so a frame either matched a pose up to
~0.5 s off (≈1.9 m at 3.8 m/s - this, not bridge jitter, is the likeliest source
of the 8 Sep 2-3 m scatter) or found none within 0.5 s and was discarded.

Fix, in two steps because the first one broke something measurable:

1. `MultiThreadedExecutor` with pose/gate/stats in their own callback group and
   a lock on the pose buffer -> pose 44 Hz. But with both image subscriptions in
   one mutually-exclusive group, the executor handed every free slot to drone
   0's subscription (created first, always has a fresh frame): **drone 1 went to
   0 fps.** The new health line caught it immediately.
2. Image callbacks now only stash the newest frame per drone, timestamped on
   arrival; one worker serves drones **round-robin**. Each of N drones gets 1/N
   of the detector by construction.

### 10.3 Bug: 85 % of every frame was one rclpy assignment (the old "~2 Hz")

With pose fixed, each drone was still inferring only ~1 fps, so each got ~5 looks
at its target per pass. Timing per frame: **total 350-450 ms, YOLO 11-15 ms**
(GPU 6-10 ms). The rest was `np_to_img()` assigning `frame.tobytes()` to
`Image.data`: rclpy's generated setter stores an `array('B')` as-is but
validates any other sequence element by element in Python.

| `Image.data =` | 1280x960 frame |
|---|---|
| `frame.tobytes()` | **296 ms** |
| `array('B', frame.tobytes())` | 5 ms |

After the one-line fix: **~27 ms per frame, every arriving frame inferred,
~8-10 fps per drone with two drones** (was ~1). This is also the answer to
section 5.1's open item - it was never transport. `test_perception.py` had the
same line and is fixed too.

### 10.4 Bug: every mission deadlocked after VERIFY

`survey_node.finish()` called `rclpy.shutdown()` from inside the `tick` timer
callback. In Humble that shuts down the global executor, which waits for
in-flight callbacks to finish - including the one making the call (confirmed in
`rclpy/__init__.py:111` and `Executor.shutdown`). The nodes never exited: the 26
Sep launch logs show them alive ~7 min after VERIFY until Ctrl-C, and after a
killed launch two orphaned `survey_node`s survived even SIGTERM. Now `finish()`
sets a flag and `main()` shuts down outside the callback; both nodes print
`process has finished cleanly` within a second of VERIFY.

### 10.5 Tooling bugs that made a 2-drone run hard to run or read

- **`hazard_map` drew the swarm wrong.** It picked the "newest" track by file
  *name*, so after any swarm run it chose `survey_track_d1_*` - drone 1's track
  in drone 1's own local frame, drawn on top of drone 0's band - and kept
  choosing that stale file after later single-drone flights. Now: newest by
  mtime; a swarm run switches on swarm mode automatically (every drone's track,
  shifted by its band offset, a home marker per drone, band boundaries);
  `--truth "15,15;15,45"` scores every band; a truth with nothing within
  `--miss-radius` (5 m) is reported **MISSED** instead of as a 30 m geotag error.
- **`check_system.sh` now sees the swarm.** It counts the running PX4
  instances and probes telemetry and camera for each (closes the 9.6 OPEN item).
- **`start_px4_swarm.sh --num-drones 2`** - the form `swarm_mission.launch.py`'s
  docstring tells you to type - used to become the Gazebo MODEL and kill the
  PX4 build. Flags now work (env vars too); unknown flags and `Y_MAX <= Y_MIN`
  are rejected; an instance that never boots is no longer reported "sensors OK";
  the blind-instance retry kills that PX4 explicitly.
- **`tools/stop_sim.sh`** (new). `tmux kill-server` leaves `make px4_sitl`, PX4
  and both `gz sim` processes running (seen three times on 27 Sep). The script
  kills by process, escalates to SIGKILL, verifies nothing is left, and never
  kills its own ancestors. Both launchers' stale-process cleanup now delegates to
  it: their old `pkill -f px4_sitl` also killed any shell or `tail -f` whose
  command line merely contained the string. Their inside-tmux guard now runs
  before cleanup.
- `TimerAction(period=<str>)` (deprecated) in `swarm_mission.launch.py` fixed;
  the launcher's exec bit restored.

### 10.6 Verification

Same sim session, `--num-drones 2 --y-min 0 --y-max 60`, altitude 10 m, 1280 px,
`conf` 0.65, one person per band at (15,15) and (15,45).

| run | code state | drone 0 | drone 1 | both VERIFY | nodes exit |
|---|---|---|---|---|---|
| 1 | before fixes, stats added | 0.81, hit | best 0.48 | PASS | no (hang) |
| 2 | before fixes | 0.82, hit | **0.65, DROPPED no-pose** | PASS | no (hang) |
| 3 | MT executor, pre round-robin/bytes fix | best 0.46 | best 0.51 | PASS | **yes** |
| **4** | **all fixes** | **0.82, 0.74 m** | **0.78, 0.82 m** | **PASS** | **yes** |
| **5** | **all fixes** | **0.78, 0.93 m** | **0.79, 1.21 m** | **PASS** | **yes** |

Runs 4 and 5: one row per target, no duplicates, no false positives, no drops,
pose 50 Hz and ~8-10 inferred fps per drone throughout. Single-drone regression
(`mission.launch.py`, target at 10,15): VERIFY PASS, clean exit, **one** row at
0.92 m - the 8 Sep run on the old code logged three rows for the same target
(0.78 / 2.44 / 3.08 m). Map: `~/maps/hazard_map_swarm_1790529684.png`.

### 10.7 Still open

- **A consistent along-track bias.** All five post-fix errors are negative North
  (-0.74, -0.82, -0.92, -1.18, -0.92 m; cross-track <= 0.27 m). All come from the
  southbound lane, so a timing term (`pose_lag_s` 0.25 too short by ~0.2 s now
  that frames are stamped on arrival) and a geometric one (forward pitch at
  3.8 m/s tilting the "nadir" camera backward, ~0.5 m at 10 m) cannot yet be
  told apart. Put a target where it is seen on a northbound lane too, then
  calibrate. Don't retune from one direction.
- **Dedup keeps the first hit, not the best.** Frames at the image edge come
  first; the single-drone run recorded 0.69 while 0.85 was seen seconds later.
- **The band boundary is flown twice.** Drone 0's last lane and drone 1's first
  lane are both at global E=30, same altitude, at different times. Lockstep keeps
  them 30 m apart; a start desync of >~15 s would put two drones on one line.
- Three drones still untested. Hardware (Track B) still at zero.

---

## 11. Decentralised 3-drone swarm — 2 Oct

Section 10 had ONE detector watching every camera - in effect a ground station.
The swarm is now decentralised: each drone runs its own stack, the way each
onboard computer (Pi) would, and drones coordinate only with each other.

### 11.1 Architecture

```
drone i (onboard.launch.py)                       peers                ground (optional)
  camera_bridge_i  -> detector_node_i  --- /swarm/hazards   <->  drones j, k  ->  ground_station
                       (own camera, own log, shared list)                           (passive: publishes
  survey_node_i  ----------------------- /swarm/heartbeat <->  drones j, k  ->       nothing)
                       (own band, takeover, separation)
```

* `swarm_msgs` (new package): `DroneHeartbeat` (id, state, shared-frame position,
  band/lane, lanes done, claimed band, ETA, battery) at 4 Hz; `HazardReport`
  (id `d<drone>-<n>`, stamp, position, class, conf), RELIABLE + TRANSIENT_LOCAL
  so a late joiner gets the history.
* `survey/swarm_logic.py`: every swarm decision as plain Python - band geometry,
  orphan detection, who claims, separation right of way. **20 unit tests.**
* `perception/hazard_registry.py`: the shared-list dedup rule. **5 unit tests.**
* `survey_node.py`: unchanged behaviour with `drone_id` unset (single-drone
  regression flown, PASS); swarm mode adds heartbeat, takeover, separation and
  early-return triggers (battery, PX4 failsafe, a peer claiming its band).
* `onboard.launch.py` (one drone) and `swarm_mission.launch.py` (N x onboard +
  passive `ground_station`). Each run gets `~/maps/swarm_<stamp>/` with a
  `run.json`, so `hazard_map` renders a run with no arguments.
* Launcher: 3 drones by default; every PX4 instance gets `COM_OBL_RC_ACT=3`.
  The default (0, Position mode) with no RC stick means a drone whose Pi dies
  **hovers in its band forever** - exactly where a neighbour will fly to take it
  over. Return mode clears the airspace. Verified in flight C.

### 11.2 The rules

**Takeover.** A band is orphaned when its owner (a) announced RETURNING/LANDED
with lanes left - immediately; (b) went silent - but only after its *own
projected finish* (last heartbeat + its ETA + 15 s): a silent drone may just
have lost its radio and still be flying its band, which onboard autonomy says it
should; or (c) was never heard within 45 s of start. The claimer is the drone
nearest the band (ties -> lower id) among those idle or within 20 s of done, so
the *neighbour* gets it rather than whoever finished first. Claims are in the
heartbeat; two simultaneous claims resolve to the better-placed drone; a drone
whose band is claimed by a live peer goes home.

**Separation.** Inside 8 m horizontal / 5 m vertical, the higher-id drone holds
position and moves vertically away - never through the other drone (if below,
it stays below). If the other drone is under PX4 control (RTL), whoever is still
in offboard yields. Resume after the peer is 10 m away for 2 s. On top of
altitude separation: takeover transits fly a per-drone layer (survey + 3 + id m)
and PX4 RTL altitudes are staggered 30/35/40 m.

**Shared hazards.** Two reports within 4.5 m are one object; the earlier stamp
wins, ties to the lower drone. Every drone applies the same rule, so all lists
converge - every drone lands with the swarm's whole map.

### 11.3 Flight results

One sim session, 3 drones, `y=[0,90]`, 10 m, 1280 px, conf 0.65; a person at
each band centre (15,15), (15,45), (15,75), plus one ON the band 0/1 boundary
at (22,30).

| flight | scenario | result |
|---|---|---|
| A | nominal | all 3 `VERIFY PASS`; 4/4 targets, 0.67-1.89 m; the boundary target logged once (drone 1) and every drone's list converged to the same 4 |
| B | drone 2 `abort_after_lanes=1`; ground station **killed at T+40 s** | drone 2 home after lane 0 (`PARTIAL`); drone 0 finished first but **held for the neighbour**; drone 1 took over band 2 lanes 1-2 and **found band 2's target there** (1.16 m); mission unaffected by the dead ground station |
| C | drone 2's onboard computer **killed** (survey, detector, camera) after lane 0 | PX4 2: `Failsafe activated -> RTL -> landed`. Peers: `SILENT ... eta 30 s`, held `not entering for another 22 s`, drone 1 claimed **at the deadline**, took over lanes 1-2, found band 2's target (1.12 m) |
| D | drone 1 `start_delay=26` - still climbing on its pad when drone 0's last lane arrives there | drone 1 `SEPARATION ... yielding: hold, go to 5.0 m`, resumed 6.7 s later; **closest approach 5.19 m** (0.74 m horizontal, 5.1 m vertical, from the tracks); all `PASS` |
| single | `mission.launch.py`, 1 drone (regression) | `VERIFY PASS 7/7`, 1 hit 0.95 m, clean exit |

Detection: 15 of 17 band-target passes logged a hit. Both misses were seen at
0.52-0.59 against the 0.65 threshold - the COCO `person` stand-in at 10 m, not
the swarm. Boundary target: 4 of 4, always exactly once.

### 11.4 Bugs found by flying it

- **Unwatched annotated images throttled all three detectors.** Each detector
  re-published every frame as a 3.7 MB annotated image whether or not anything
  subscribed. With three drones: 3.3 fps inferred, pose 22 Hz, YOLO 86 ms. Now
  only built while something subscribes: **8.2 fps, 50 Hz, 15 ms** - the
  two-drone figures, with three. This, not the GPU, was the 3-drone ceiling.
- **First lane uninspected.** YOLO's first inference does its CUDA warm-up; it
  landed on the first gated frame, and three detectors warming up at once left
  5-20 s of every drone's first lane at "inferred 0.0 fps". Warm-up now runs at
  startup (2.5 s, on the pad).
- `ground_station` crashed on the first hazard (`origin_drone` vs `origin`) -
  which incidentally showed the drones don't need it.
- Swarm-mode waypoints were built before the band geometry existed (caught by a
  launch dry run before any flight).
- `stop_sim.sh`'s unanchored `pkill -f` patterns killed any process whose command
  line merely *mentioned* `ros2 launch survey` or `gz sim` - including, via
  `start_px4_sim.sh`'s cleanup, an unrelated shell. Patterns now match the
  executables only (`^make px4_sitl`, `^gz sim`, ...).

### 11.5 Still open

- **Detection is the weak link, and it is the model.** ~88 % per pass with COCO
  `person` at 10 m; misses sit at 0.5-0.6. Trained nadir weights are the fix;
  lowering `conf` re-admits false positives (seen at 0.71 on 8 Sep).
- **Onboard compute is assumed, not budgeted.** In sim each "Pi" is a process on
  an RTX 4060 (15 ms/frame at imgsz 1280). A Raspberry Pi's CPU would manage a
  small fraction of 1 fps at that size - too few looks per pass. Real onboard
  detection needs an accelerator (Pi 5 + Hailo/Coral, or a Jetson) or a smaller
  `imgsz` with a lower altitude. Decide before buying hardware.
- **A silent drone that is still flying is invisible to separation.** The
  deadline rule keeps takeovers out of its band until it would have finished,
  and the offboard-loss RTL clears a dead Pi's drone; a drone whose radio dies
  but keeps flying *past* its ETA is not covered.
- Takeover of a takeover is not handled (a drone dying mid-takeover leaves those
  lanes unflown - logged, not re-assigned).
- Hazard stamps use wall clock; on real Pis they need NTP/GPS time for "earlier
  wins" to mean earlier (lists still converge without it).
- The band boundary is still flown twice (drone i's last lane, drone i+1's first).
- Hardware (Track B) still at zero.
