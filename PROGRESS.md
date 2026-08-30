# Project Progress — Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)
### Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali

_Last updated: 31 Aug 2026. Companion docs: `STACK_README.md` (how to run), `PHASE1_ROADMAP.md` (plan)._

---

## 1. Summary of status

The **software / autonomy / AI pipeline is built and working end-to-end.**
A single drone autonomously surveys a specified area and returns, via two
independent paths (a QGroundControl Survey mission, and a custom ROS 2 node).
The **AI detection pipeline is live**: the downward camera streams into ROS 2,
YOLO runs on the feed, and annotated frames publish at ~2 Hz. What remains for
the detection deliverable is a *detectable target* — the plumbing is done.

| Capability | Status |
|---|---|
| Single-drone PX4 SITL + Gazebo + QGC + DDS bridge | ✅ Working |
| One-command launcher (`start_px4_sim.sh`) | ✅ Working |
| Sim home relocated to IIT Gandhinagar | ✅ Working |
| Autonomous area survey — ROS 2 node (`survey`) | ✅ Built & runs (flies, offboard, RTL) |
| Autonomous area survey — QGC "draw polygon" mission | ✅ Working (demonstrated) |
| Downward camera model in sim (`x500_mono_cam_down`) | ✅ Model + airframe confirmed |
| Gazebo camera → ROS 2 bridge (`ros_gz_bridge`) | ✅ Configured & launches |
| YOLO detector node (`perception`) | ✅ **Working** — ~2 Hz on `/detection/image_annotated` |
| **Camera frames reaching ROS 2 / YOLO** | ✅ **SOLVED** — was a `GZ_IP` mismatch (section 5) |
| Geotagged hazard map / CSV | ⚙️ Running; needs a detectable target in the world |
| Multi-drone swarm (2–5) | ⬜ Not started |

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
| Standalone survey script | `~/px4_ros_ws/survey_node.py` |
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

---

## 6. Gotchas already solved (don't re-discover)

- Run the launcher from a **normal terminal**, not inside tmux. It auto-clears stale px4/agent/gz processes (else "port 8888 in use" / "instance 0 already running").
- `commander` commands go in the **PX4 (pxh>) pane**, not a bash pane.
- Do **not** pipe PX4 through `| tee` — it makes the pxh console non-interactive (launcher uses tmux `pipe-pane` for logging instead).
- Arming from ROS 2 without a GCS: set `NAV_DLL_ACT=0` and `CBRK_SUPPLY_CHK=894281` (or just run QGC, which satisfies the GCS check).
- Gazebo must render on the RTX (`prime-run`) or it's unusably slow.
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
- Correct ROS 2 topic names for this PX4 are **unversioned** (`/fmu/out/vehicle_local_position`, etc.).
- `pip install --user ultralytics` bumps setuptools to 84 and **breaks colcon** (needs <80). Fix: `pip install --user "setuptools==70.3.0"`.
- QGC v5.0 won't run on Ubuntu 22.04 (glibc). Use v4.4.3.
- Committing files over the device bridge resets the executable bit — re-`chmod +x` the launcher, or run it with `bash`.

---

## 7. Recommended next steps

1. **Get a first real detection** — insert a COCO-class model (person/vehicle) into the Gazebo world under the survey area, fly a survey, and confirm a box in `rqt_image_view`, a `HAZARD #n` log line, and a row in `~/maps/hazard_points.csv`. That closes the M3/M5 loop.
2. **Speed up** — camera to 640×480 @ 10 Hz for a smoother demo.
3. **Train landmine weights** — build a small dataset, train YOLO, drop `best.pt` into the detector.
4. **Scale to swarm (2–5)** — extend the launcher to spawn namespaced PX4 instances (`/px4_1`, `/px4_2`, …) and split a drawn area across drones.
5. **Hardware track (Track B)** — 50% of the grade is self-designed/Make parts + a DFM + FEA/CFD design report; run this in parallel (see `PHASE1_ROADMAP.md`).
