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
| `survey_node.py` (workspace root) | Standalone copy of the survey node, runnable without building. |

Supporting files in each package — `package.xml` (dependencies), `setup.py`
(how it installs, and the `ros2 run` entry point), `setup.cfg`,
`resource/<name>` (a marker file ROS 2 uses to find the package).

### Scripts & docs — `~/px4_ros_ws/`

| File | Purpose |
|---|---|
| `start_px4_sim.sh` | One command to boot everything: DDS agent + PX4/Gazebo + sourced ROS 2 shell + QGC, in a 3-pane tmux window. Also auto-sets the PX4 safety params. |
| `diagnose_camera.sh` | Read-only fact-gatherer for camera/render problems. |
| `STACK_README.md` | How to run things. |
| `PHASE1_ROADMAP.md` | Deliverables, milestones, grading split, team roles. |
| `PROGRESS.md` | What's done / blocked / next. |
| `CAMERA_DIAGNOSTIC.md` | Full write-up of the `GZ_IP` bug — worth reading as a debugging case study. |
| `SYSTEM_GUIDE.md` | This file. |

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

With a nadir camera and level flight:

```
metres_per_pixel = 2 · altitude · tan(HFOV/2) / image_width
offset          = (pixel − image_centre) · metres_per_pixel
world_position  = drone_position + rotate(offset, drone_heading)
```

This is why the detector subscribes to `vehicle_local_position` as well as the
camera — a detection is meaningless without knowing where the drone was.

**Calibrated (1 Sep).** A `person` model at Gazebo `x=15, y=10` → PX4
`North=10, East=15`. The two high-confidence hits gave:

| | recorded | truth | error |
|---|---|---|---|
| East (cross-track) | 14.53, 14.70 | 15.0 | **−0.47, −0.30 m** |
| North (along-track) | 4.53, 8.65 | 10.0 | −5.5, −1.4 m |

Cross-track is accurate to under half a metre, so **the axis mapping is
correct** — no `geo_swap_axes` / `geo_flip_*` flag is needed. The along-track
error is **latency**, not geometry: the two hits are 0.2 s apart but place the
target on *opposite* sides of the image centre, which can only happen if the
frames were stale by different amounts. At the 9.2 m/s the drone was flying,
0.3 s of staleness is 2.8 m of error.

Two fixes, both now in the code:

1. `detector_node` keeps a ring buffer of poses and geotags with the pose at
   *frame arrival* minus `pose_lag_s` (default 0.15), not the newest pose.
2. `survey_node` has `lookahead_m` (default 4.0) which caps ground speed at
   ~3.8 m/s instead of letting PX4 sprint at `MPC_XY_VEL_MAX`.

Residual along-track error should now be around ±1.5 m — comparable to the
2 m dedup radius, and small against the 12 × 9 m camera footprint.

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
ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=5.0
#    defaults now: classes:='person' conf:=0.40 require_gate:=true lookahead_m:=4.0
#    classes:='' to see every COCO class again (expect airplane/kite/bird junk)

# 4. watch
ros2 run rqt_image_view rqt_image_view /detection/image_annotated
```

Stop everything: `tmux kill-server`

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

Done: single-drone autonomous area survey (two ways) · camera → ROS 2 ·
YOLO on GPU · geotagging **calibrated against a known target** · full mission
verified end-to-end (`VERIFY PASS | waypoints 11/11 | returned=True`).

Next, in order:
1. **Train landmine weights** on **aerial/nadir imagery** (COCO weights can't see
   a person from straight above — that's a data problem, not a code problem).
3. **Render the hazard map** as a real deliverable.
4. **Swarm** — multiple PX4 instances under namespaces (`/px4_1`, `/px4_2`),
   splitting one area between drones.
4. **Hardware track** — CAD, Make-fabricated parts, DFM + FEA/CFD report.
   Half the grade; runs in parallel with all of the above.
