# PX4 + ROS 2 + Gazebo + QGroundControl — how to run it

*Updated 1 Sep 2026. For what the system **is**, read `SYSTEM_GUIDE.md`.
For current status and the next-session plan, `PROGRESS.md` / `NEXT_SESSION.md`.*

---

## 1. Boot the stack

```bash
bash ~/px4_ros_ws/start_px4_sim.sh                          # plain x500, no camera
bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down    # CAMERA drone — use this
```

Run it from a **normal terminal, never from inside tmux.**

That single script brings up the whole stack in a tmux window plus the QGC
window. It clears leftover processes from a previous run, forces the RTX GPU for
Gazebo, spawns the drone at IIT Gandhinagar, and auto-sets the PX4 safety
parameters that otherwise block arming and mission start.

What starts, in order:

1. **Micro XRCE-DDS Agent** — puts PX4's internal (uORB) topics onto ROS 2 (UDP 8888).
2. **PX4 SITL + Gazebo** — the flight stack and the 3D simulator.
3. **ROS 2 shell** — a terminal with `px4_ros_ws` already sourced.
4. **QGroundControl** — the GUI ground station (auto-connects on UDP 14550).

The first three are tmux panes; QGC opens in its own window.

### ALWAYS do this before flying

```bash
ros2 topic hz /fmu/out/vehicle_local_position_v1
```

It must report a rate. If it doesn't, telemetry is dead and every mission will
silently fail with `arm/offboard timeout` and `0 samples` — this exact check
would have saved an entire session. See `SYSTEM_GUIDE.md` §3.5 on why the topic
name carries a `_v1`.

---

## 2. Fly the mission

```bash
cd ~/px4_ros_ws
colcon build --packages-select survey perception && source install/setup.bash

ros2 launch survey mission.launch.py \
    x_max:=30.0 y_max:=20.0 altitude:=5.0 lane_spacing:=5.0
```

This runs the whole Phase I loop: camera bridge + YOLO detector start first,
then after 8 s the survey node arms, flies a boustrophedon pattern, geotags what
it sees, RTLs, and prints a `VERIFY PASS/FAIL` line.

Watch it:

```bash
ros2 run rqt_image_view rqt_image_view /detection/image_annotated
```

### Survey only, no detection

```bash
ros2 launch survey survey.launch.py x_max:=40.0 y_max:=30.0 altitude:=10.0
```

### Detection only, no flight

```bash
ros2 launch perception perception.launch.py
```

### Detection without Gazebo at all (offline test)

```bash
ros2 run perception detector_node --ros-args -p image_topic:=/test/image   # terminal 1
python3 ~/px4_ros_ws/src/perception/test_perception.py                     # terminal 2
```

---

## 3. Launch arguments worth knowing

| Argument | Default | What it does |
|---|---|---|
| `x_min/x_max/y_min/y_max` | `0/40/0/30` | Survey rectangle, PX4 local NED metres, home = 0,0 |
| `altitude` | `15.0` | Metres AGL (the node converts to `z = -altitude`) |
| `lane_spacing` | `8.0` | Metres between lanes. Camera footprint at height *h* is `2·h·tan(HFOV/2)` — at 5 m that's 11.85 m, so 8.3 m gives 30 % sidelap |
| `lookahead_m` | `4.0` | **Ground-speed cap** (~0.95 × this, m/s). `0.0` = fly flat out at `MPC_XY_VEL_MAX` |
| `rtl_on_complete` | `true` | `false` leaves the drone hovering at the end |
| `weights` | `yolov8n.pt` | Point at `~/runs/detect/train/weights/best.pt` once you have trained weights |
| `classes` | `person` | Comma-separated class names to keep. `''` = keep all — expect `airplane`/`kite`/`bird` junk from COCO weights on nadir ground |
| `conf` | `0.40` | Detection confidence floor |
| `require_gate` | `true` | Only geotag while the survey node says it's flying lanes — no hazards logged during climb, RTL or landing |
| `pose_lag_s` | `0.15` | Camera+bridge latency compensated when geotagging. Raise if along-track error is consistently one-sided |
| `survey_delay` | `8.0` | Seconds to let the camera pipeline settle before the drone moves |

`geo_swap_axes` / `geo_flip_forward` / `geo_flip_right` exist on the detector but
**must stay `False`** — the axis mapping was calibrated against a known target on
1 Sep and is correct. See `NEXT_SESSION.md` §1.3.

---

## 4. Where everything lives

| What | Path |
|------|------|
| Launch script | `~/px4_ros_ws/start_px4_sim.sh` |
| ROS 2 workspace | `~/px4_ros_ws` |
| **Your** packages | `~/px4_ros_ws/src/survey`, `~/px4_ros_ws/src/perception` |
| Vendored (not yours) | `~/px4_ros_ws/src/px4_msgs`, `~/px4_ros_ws/src/px4_ros_com` |
| Launch files | `src/survey/launch/{survey,mission}.launch.py`, `src/perception/launch/perception.launch.py` |
| **Outputs** | `~/maps/survey_track_<ts>.csv` (flown path), `~/maps/hazard_points.csv` (detections) |
| PX4 firmware / SITL | `~/PX4-Autopilot` |
| DDS agent | `~/Micro-XRCE-DDS-Agent` |
| QGroundControl | `~/Downloads/QGroundControl.AppImage` (v4.4.3 — v5 needs a newer Ubuntu) |
| Per-run logs | `~/px4_ros_ws/log/sim_launch_<timestamp>/` |
| Gazebo server log | `~/.gz/sim/log/<timestamp>/server_console.log` ← open this first for any Gazebo problem |

> **Careful:** `~/px4_ros_ws/survey_node.py` (workspace root) is an old standalone
> copy kept for running without `colcon`. It is **not** the file the package uses
> and it does **not** have the current fixes. Edit
> `src/survey/survey/survey_node.py` instead.

---

## 5. tmux quick reference (the 3-pane window)

- `Ctrl-b` then `o` — cycle focus between panes (border turns green)
- `Ctrl-b` then arrow — move to the pane in that direction
- Mouse is enabled — click a pane to focus it, scroll with the wheel
  (press `q` to leave scroll mode before typing again)
- `Ctrl-b` then `d` — detach (stack keeps running); `tmux attach -t px4_sim` to return
- `tmux kill-server` — stop everything

Left pane = DDS agent · right pane = PX4 console (`pxh>`) · bottom = ROS 2 shell.

---

## 6. Common commands

In the **ROS 2 pane**. PX4 publishes best-effort, so a plain `ros2 topic echo`
shows nothing — the flag is needed. Note it exists on `echo` **only**;
`ros2 topic hz` does not accept it.

```bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
ros2 topic hz   /fmu/out/vehicle_local_position_v1
ros2 topic echo /survey/detecting                       # the detection gate
```

In the **PX4 pane** (`pxh>` prompt) — these do **not** work in a bash pane:

```
commander takeoff
commander land
listener sensor_combined
param show COM_RC_IN_MODE
```

Anything talking to a Gazebo topic from an ad-hoc shell needs:

```bash
export GZ_IP=127.0.0.1
```

Without it you will see the topic listed and receive nothing, silently.
Full story in `CAMERA_DIAGNOSTIC.md`.

---

## 7. Configuration knobs

```bash
# Different airframe/model
bash ~/px4_ros_ws/start_px4_sim.sh gz_rc_cessna

# Different home location (default is IIT Gandhinagar 23.2127, 72.6846)
PX4_HOME_LAT=23.2130 PX4_HOME_LON=72.6850 PX4_HOME_ALT=30 \
    bash ~/px4_ros_ws/start_px4_sim.sh
```

---

## 8. Notes

- **GPU:** the script forces the RTX 4060 via `prime-run` so Gazebo is smooth.
  Confirm with `watch -n1 nvidia-smi` — you should see `gz sim` using the GPU.
- **Arming without a ground station:** if you run without QGC and hit
  "Preflight Fail: No connection to the GCS", set in the PX4 pane:
  `param set NAV_DLL_ACT 0` and `param set CBRK_SUPPLY_CHK 894281`.
  The launcher already does this, plus `COM_RC_IN_MODE 4` / `COM_RCL_EXCEPT 7`
  for the "No manual control input" mission block.
- **RTL climbs to 30 m** by default (QGC → Safety → *RTL Climb To*). That is why
  the drone shoots up and circles after the survey finishes — expected, not a bug.
- **Move `~/maps/hazard_points.csv` aside between runs** or results from different
  flights will be mixed together in one file.
- **`pip install ultralytics` breaks colcon** by bumping setuptools past 80.
  Fix: `pip install --user "setuptools==70.3.0"`.
- Plug in the charger — the full sim is heavy on the battery.
