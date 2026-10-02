# System Guide — understand what you've built

*A study document for the Autonomous Swarm Drone System (Phase I).*
*Read this top-to-bottom once; after that use it as a reference.*

Companion docs — this one explains **how it works**, the others cover:
`STACK_README.md` (how to run) · `PHASE1_ROADMAP.md` (the plan & grading) ·
`PROGRESS.md` (status log) · `CAMERA_DIAGNOSTIC.md` (a solved bug, in detail).

---

## 1. The big picture

You have **five separate programs** that cooperate. Nothing here is one big app —
understanding the boundaries is most of understanding the system.

```
┌───────────────────────────────────────────────────────────────────┐
│ GAZEBO  (the physical world)                                      │
│   • simulates gravity, motors, the ground, the drone body         │
│   • renders the downward CAMERA                                   │
│   • speaks "gz-transport"                                         │
└────────────┬──────────────────────────────┬───────────────────────┘
             │ sensors (IMU/GPS/baro)       │ camera images
             ▼                              ▼
┌────────────────────────┐        ┌──────────────────────┐
│ PX4  (the flight brain)│        │ ros_gz_bridge        │
│  • estimates position  │        │  gz image ──► ROS 2  │
│  • runs the motors     │        └──────────┬───────────┘
│  • enforces safety     │                   │
│  • speaks "uORB"       │                   ▼
└──────┬─────────┬───────┘        ┌──────────────────────┐
       │         │                │ detector_node        │
       │         │ MAVLink        │  YOLO → boxes        │
       │         ▼                │  pixel → world       │
       │  ┌─────────────┐         │  → hazard CSV        │
       │  │ QGroundCtrl │         └──────────────────────┘
       │  └─────────────┘                   ▲
       │ uORB                               │
       ▼                                    │
┌────────────────────────┐                  │
│ Micro XRCE-DDS Agent   │                  │
│  uORB ◄──► ROS 2 DDS   │                  │
└──────┬─────────────────┘                  │
       │ /fmu/out/... , /fmu/in/...         │
       ▼                                    │
┌────────────────────────────────────────────────────────┐
│ YOUR ROS 2 NODES                                       │
│   survey_node  — decides where to fly, commands PX4    │
│   detector_node — decides what it sees ────────────────┘
└────────────────────────────────────────────────────────┘
```

**Two independent data highways.** This trips everyone up:

| Path | Carries | Protocol | Breaks when… |
|---|---|---|---|
| PX4 ⇄ ROS 2 | telemetry & commands (`/fmu/...`) | **DDS**, via the XRCE agent | agent not running / port 8888 taken |
| Gazebo ⇄ ROS 2 | camera images | **gz-transport**, via `ros_gz_bridge` | `GZ_IP` mismatch |

They fail independently. When something breaks, **first ask which highway**.

---

## 2. Every file, and what it does

### Your code — `~/px4_ros_ws/src/`

| File | What it does |
|---|---|
| `survey/survey/survey_node.py` | **The flight brain you wrote.** Generates a boustrophedon ("lawnmower") path over a rectangle, streams offboard setpoints to PX4, arms, flies waypoint-by-waypoint, verifies each was reached, writes the flown track to CSV, then triggers RTL. |
| `survey/launch/survey.launch.py` | Runs `survey_node` with area parameters exposed as launch args. |
| `survey/launch/mission.launch.py` | **The integration.** Starts perception first, waits 8 s, then starts the survey — so detection is live before the drone moves. |
| `perception/perception/detector_node.py` | **The eyes.** Subscribes to camera images, runs YOLO on the GPU, draws boxes, projects each detection from pixel → ground coordinates, de-duplicates, appends to the hazard CSV. |
| `perception/launch/perception.launch.py` | Starts `ros_gz_bridge` (camera into ROS 2) + `detector_node`, both with `GZ_IP=127.0.0.1`. |
| `perception/test_perception.py` | **Offline test.** Feeds the detector a known image + fake drone pose, so you can verify YOLO + geotagging without flying. |
| `survey/survey/swarm_logic.py` | Swarm rules with no ROS in them (unit-tested): band split, lane layout, ETA, separation and right of way, takeover decisions. |
| `survey/launch/onboard.launch.py` | Everything ONE drone's onboard computer runs: camera bridge, detector, survey node. |
| `survey/launch/swarm_mission.launch.py` | N × `onboard.launch.py`, one run directory (`~/maps/swarm_<stamp>/`), optional ground station. |
| `perception/perception/hazard_registry.py` | One drone's copy of the shared hazard list: per-frame association, refinement, peer merge rules (unit-tested). |
| `perception/perception/ground_station.py` | Passive monitor: status table and the merged hazard list. Kill it and nothing changes. |
| `perception/perception/hazard_map.py` | Renders a run (tracks, hazards, truth) to PNG + GeoJSON and scores it against known targets. |
| `swarm_msgs/msg/*.msg` | `DroneHeartbeat` (4 Hz peer state) and `HazardReport` (shared hazards). |
| `survey_node.py` (workspace root) | Standalone copy of the survey node, runnable without building. |

Supporting files in each package — `package.xml` (dependencies), `setup.py`
(how it installs, and the `ros2 run` entry point), `setup.cfg`,
`resource/<name>` (a marker file ROS 2 uses to find the package).

### Scripts & docs — `~/px4_ros_ws/`

```
px4_ros_ws/
├── README.md            index — start here
├── start_px4_sim.sh     THE entry point; stays at root deliberately
├── docs/                everything to read
├── tools/               everything to run that isn't a ROS node
├── experiments/         one-off measurements, kept for the report
├── src/                 the ROS 2 packages (your actual code)
└── build/ install/ log/ generated by colcon — never edit, never commit
```

| File | Purpose |
|---|---|
| `start_px4_sim.sh` | One command to boot everything: DDS agent + PX4/Gazebo + sourced ROS 2 shell + QGC, in a 3-pane tmux window. Also auto-sets the PX4 safety params. |
| `tools/check_system.sh` | Six-layer PASS/FAIL probe of the whole stack. Run before every flight. |
| `tools/add_target.sh` | Spawns the detection target into a running sim. Needed after **every** sim start. |
| `tools/start_px4_swarm.sh` | The N-drone equivalent of `start_px4_sim.sh`. |
| `tools/stop_sim.sh` | Stops everything (single or swarm); `--all` also closes QGC. |
| `tools/add_swarm_targets.sh` | One target in the middle of each drone's band. |
| `tools/analyse_sightings.py` | Calibration from a run's sightings: detection swath, timing, scatter (section 3.6). |
| `tools/diagnose_camera.sh` | Read-only fact-gatherer for camera/render problems. |
| `docs/CHEATSHEET.md` | Where things are, where each command runs, and the git flow. |
| `docs/STACK_README.md` | How to run things. |
| `docs/SYSTEM_GUIDE.md` | This file — how it all works. |
| `docs/PROGRESS.md` | What's done / blocked / next. |
| `docs/NEXT_SESSION.md` | Hand-off: current state and the ordered plan. |
| `docs/PHASE1_ROADMAP.md` | Deliverables, milestones, grading split, team roles. |
| `docs/CAMERA_DIAGNOSTIC.md` | Full write-up of the `GZ_IP` bug — worth reading as a debugging case study. |
| `experiments/px_sweep.py` | Measures YOLO confidence vs target pixel size (the altitude ceiling). |
| `experiments/px_chart.py` | Renders that measurement as a figure for the report. |

### Not your code (dependencies)

| Path | What |
|---|---|
| `~/px4_ros_ws/src/px4_msgs` | ROS 2 message definitions matching PX4's internal messages. Vendored, gitignored. |
| `~/px4_ros_ws/src/px4_ros_com` | PX4's official ROS 2 examples + frame-transform helpers. |
| `~/PX4-Autopilot` | The flight-control firmware + simulator glue. |
| `~/Micro-XRCE-DDS-Agent` | The uORB ⇄ DDS translator. |
| `~/Downloads/QGroundControl.AppImage` | Ground station GUI (v4.4.3 — v5 needs a newer Ubuntu). |
| `~/maps/` | **Your outputs**: `survey_track_*.csv` (flown path), `hazard_points.csv` (detections). |
| `~/yolo_test`, `~/runs` | YOLO virtualenv and training runs. |

`build/`, `install/`, `log/` are generated by `colcon` — never edit, never commit.

---

## 3. Concepts you need to actually understand

### 3.1 Coordinate frames — the #1 source of bugs

Three frames, and they disagree:

| Frame | X | Y | Z | Used by |
|---|---|---|---|---|
| **NED** | North | East | **Down** | PX4 (all `/fmu/` topics, your survey node) |
| **ENU** | East | North | **Up** | Gazebo world, ROS 2 convention |
| **Body FRD** | Forward | Right | Down | PX4 attitude |

Consequences you have already hit:
- Altitude of 10 m is `z = -10` in NED (negative is up).
- A model spawned in Gazebo at `x=15, y=10` sits at **North=10, East=15** in PX4.
  The axes are **swapped**.
- `px4_ros_com` ships `frame_transforms.h` for converting properly.

**Whenever a position looks mirrored or rotated, suspect a frame conversion.**

### 3.2 Offboard vs Mission — two ways to fly autonomously

| | Offboard (your `survey_node`) | Mission (QGC Survey) |
|---|---|---|
| Who decides the path | your ROS 2 code | PX4, from an uploaded waypoint list |
| How | stream `TrajectorySetpoint` at >2 Hz | upload once, PX4 executes |
| Fails if | you stop streaming for ~0.5 s → failsafe | — |
| Good for | swarm logic, reacting to detections | quick drawn-polygon surveys |

You have both. Offboard is what scales to the swarm.

### 3.3 QoS — why a topic can exist but deliver nothing

PX4 publishes `/fmu/out/*` as **BEST_EFFORT**. A subscriber demanding
RELIABLE will match *nothing* and sit silent with no error. That's why your
nodes use a best-effort QoS profile, and why `ros2 topic echo` needs
`--qos-reliability best_effort`.

### 3.4 `GZ_IP` — the bug that cost a day

PX4 launches Gazebo with `GZ_IP=127.0.0.1`. gz-transport does **discovery**
by multicast (works regardless) but **data transfer** by direct connection to
the publisher's advertised address. A subscriber without the same `GZ_IP`
therefore **sees the topic and receives nothing**, silently.

General lesson: *"listed but empty" is a connection problem, not a data problem.*

### 3.5 Versioned topics

You'll notice both `/fmu/out/vehicle_local_position` **and**
`..._v1`, and `vehicle_status` **and** `vehicle_status_v4`. PX4 is migrating to
versioned messages: any message with `MESSAGE_VERSION > 0` is published on the
`_vN` topic **only**.

**On this build, the unversioned names carry nothing.** Subscribing to them gave
zero telemetry, which silently produced every `arm/offboard timeout`, every
`0 samples` track CSV, and dead geotagging. Two things made it hard to see:

* `dds_topics.yaml` and the `px4_ros_com` examples both use the unversioned name.
* `ros2 topic list` showed the unversioned name — but **a subscription alone
  creates that entry.** Seeing a topic listed is not evidence of a publisher.

Both nodes now subscribe to the versioned *and* unversioned name, so they work
either way. To check which one is live:

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1
ros2 topic hz /fmu/out/vehicle_local_position
```

Whichever reports a rate is the real one.

### 3.6 Pixel → ground (how a detection becomes a map point)

The camera is fixed to the airframe looking straight down (`CameraJoint`,
fixed, in `x500_mono_cam_down`). Each box centre becomes a ray in the body
frame, which is rotated by the drone's **full attitude** (PX4
`vehicle_attitude`, roll + pitch + yaw) and intersected with flat ground at
home altitude:

```
f        = (image_width / 2) / tan(HFOV / 2)            # pixels
ray_body = ((v_centre − v) / f,  (u − u_centre) / f,  1)  # forward, right, down
ray_ned  = rotate(q_attitude, ray_body)
ground   = drone_position + (altitude / ray_ned.down) · (ray_ned.north, ray_ned.east)
```

A detection is meaningless without knowing where the drone was **when the
frame was taken**, so the detector keeps 2-3 s of pose and attitude and looks
both up at *frame arrival − `pose_lag_s`*.

**What the numbers rest on (2 Oct, `tools/analyse_sightings.py` over the
`sightings_d<i>.csv` the detector writes):**

| | value | evidence |
|---|---|---|
| `pose_lag_s` | **0.25 s** | along-track error +0.01 m over 298 sightings, both travel directions; a timing error would show as the same-signed error ahead/behind on north- and south-bound lanes |
| attitude vs level projection | level is **+0.5–0.8 m ahead** | the drone cruises 3° nose-down; attitude-aware removes it (use_attitude, default on) |
| steady-flight scatter | **0.25 m RMS, 0.53 m max** | per-target spread of recorded sightings |
| final error to truth | **0.09–0.29 m** | 11 targets, 3 drones |
| merge radius `min_sep_m` | **1.5 m** | ~3x the worst scatter; separates a pair 2 m apart |

**Frames that are never recorded:**

- **While turning** (`max_rate_dps`, 30°/s from the attitude 0.1 s either side
  of the frame). Turning onto a lane the drone banks 20+° and yaws ~140°/s;
  those frames put a person standing *under* the lane 5 m off.
- **Off the survey area.** The detection gate is open only along a lane *and*
  over `[x_min, x_max]`. Each lane has a 6 m lead-in/run-out outside the area
  (`lead_in_m`) so the drone is straight and level when the area starts, and
  heading for a lane start it already faces along the lane (it sidesteps
  rather than swinging up to 180°). Geotags in the first 6 m of a lane were
  p90 1.47 m against ≤ 0.5 m after.

**Turning sightings into hazards** (`perception/hazard_registry.py`):

1. Within one frame, two boxes closer than `box_merge_m` (1.0 m) on the ground
   are one object boxed twice (YOLO does this - whole body + part).
2. Each remaining detection is matched to the nearest known hazard within
   `min_sep_m`, **one-to-one per frame** - so two objects seen together are
   never merged, however close.
3. The owner refines the hazard's position as a weighted mean of its
   sightings, weight = confidence × cos²(off-nadir angle): oblique views are
   less accurate, and are where two close objects get boxed as one.

History: the 1 Sep calibration (East −0.47/−0.30 m, North −5.5/−1.4 m) is
what found the latency in the first place - a consistent error on one axis and
a sign-flipping one on the other is timing, not frames (`PROGRESS.md` §6).

**Lane spacing comes from the detector, not the camera.** The camera sees a
23.7 m wide strip at 10 m; YOLO scores a COCO `person` ≥ 0.65 reliably only
within ~2.5 m of the track (74 passes: 0–1.5 m 10/11, 1.5–3 m 7/11, 3–4.5 m
3/12). `survey_node` spaces lanes by `detect_fov_deg` (28°) × (1 − `sidelap`):
4 m. Re-measure with `tools/analyse_sightings.py` for any other weights.

---

## 4. How to run it

```bash
# 1. boot the stack (normal terminal, camera drone)
bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down

# 2. ALWAYS verify the PX4 link before flying
ros2 topic list | grep vehicle_local_position

# 3. build + fly the whole mission
cd ~/px4_ros_ws
colcon build --packages-select survey perception && source install/setup.bash
ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=10.0
#    defaults now: classes:='person' conf:=0.65 require_gate:=true lookahead_m:=4.0
#    classes:='' to see every COCO class again (expect airplane/kite/bird junk)
#    The 3-drone swarm: docs/CHEATSHEET.md section 5b.

# 4. watch
ros2 run rqt_image_view rqt_image_view /detection/image_annotated
```

Stop everything: `bash ~/px4_ros_ws/tools/stop_sim.sh` (not `tmux kill-server` — PX4 and Gazebo survive it)

### Test without flying
```bash
ros2 run perception detector_node --ros-args -p image_topic:=/test/image   # terminal 1
python3 ~/px4_ros_ws/src/perception/test_perception.py                     # terminal 2
```

---

## 5. Debugging method (what actually worked)

1. **Which highway?** DDS (`/fmu/...`) or gz-transport (camera)? They fail separately.
2. **Read the right log.** `~/.gz/sim/log/<timestamp>/server_console.log` is the
   most informative file for anything Gazebo-side. PX4's own log is in the tmux pane.
3. **A loud warning is not automatically the cause.** The `libEGL … dri2` errors
   looked fatal and were harmless noise; the real bug was silent.
4. **Prove each stage separately.** Camera producing? → bridge delivering? →
   detector receiving? → detector publishing? Bisect, don't guess.
5. **"Advertised but empty" ⇒ connection/QoS**, not the producer.

---

## 6. Where this is going

Done and flight-verified: single-drone survey; the decentralised 3-drone swarm
(onboard autonomy, heartbeats, band takeover, shared hazard list, separation);
geotagging calibrated to ~0.2 m against 11 known targets; the hazard map.

Next, in order (`NEXT_SESSION.md` has the detail):
1. **Landmine weights** trained on nadir imagery - then re-measure the
   detection swath, because lane spacing (and so flight time) follows it.
2. **Terrain following** - the projection assumes flat ground at home altitude.
3. **Hardware track** - CAD, fabricated parts, DFM + FEA/CFD report. Half the
   grade; runs in parallel with all of the above.
