# PX4 + ROS 2 + Gazebo + QGroundControl — how to run it

## One command launches everything

```bash
~/px4_ros_ws/start_px4_sim.sh
```

That single script brings up the whole stack in a tmux window plus the QGC
window. It also clears any leftover processes from a previous run, forces the
RTX GPU for Gazebo, and spawns the drone at IIT Gandhinagar.

What starts, in order:

1. **Micro XRCE-DDS Agent** — the bridge that puts PX4's internal (uORB)
   topics onto ROS 2 (UDP port 8888).
2. **PX4 SITL + Gazebo** — the flight stack and the 3D simulator (x500 quad).
3. **ROS 2 shell** — a terminal with `px4_ros_ws` already sourced, ready for
   `ros2 ...` commands.
4. **QGroundControl** — the GUI ground station (auto-connects on UDP 14550).

The first three are tmux panes; QGC opens in its own window.

## tmux quick reference (the 3-pane window)

- `Ctrl-b` then `o` — cycle focus between panes (border turns green)
- `Ctrl-b` then arrow — move to the pane in that direction
- Mouse is enabled — click a pane to focus it, scroll with the wheel
  (press `q` to leave scroll mode before typing again)
- `Ctrl-b` then `d` — detach (stack keeps running); `tmux attach -t px4_sim` to return
- `tmux kill-server` — stop everything

Left pane = DDS agent · right pane = PX4 console (`pxh>`) · bottom = ROS 2 shell.

## Where everything lives

| What | Path |
|------|------|
| Launch script | `~/px4_ros_ws/start_px4_sim.sh` |
| ROS 2 workspace | `~/px4_ros_ws` |
| Your ROS 2 packages | `~/px4_ros_ws/src/px4_ros_com`, `~/px4_ros_ws/src/px4_msgs` |
| ROS 2 launch files | `~/px4_ros_ws/src/px4_ros_com/launch/` |
| PX4 firmware / SITL | `~/PX4-Autopilot` |
| DDS agent | `~/Micro-XRCE-DDS-Agent` |
| QGroundControl | `~/Downloads/QGroundControl.AppImage` |
| Per-run logs | `~/px4_ros_ws/log/sim_launch_<timestamp>/` |

### ROS 2 launch files (`px4_ros_com/launch/`)

- `offboard_control_launch.yaml` — runs the `offboard_control` node that arms,
  switches to offboard mode, and flies the drone from code.
- `sensor_combined_listener.launch.py` — subscribes to sensor data and prints it.

Run either from the ROS 2 pane:

```bash
ros2 launch px4_ros_com offboard_control_launch.yaml
```

## Common commands

In the **ROS 2 pane** (note the QoS flag — PX4 publishes best-effort, so a
plain `ros2 topic echo` shows nothing):

```bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort
ros2 topic hz   /fmu/out/vehicle_attitude       --qos-reliability best_effort
ros2 run px4_ros_com offboard_control            # fly from ROS 2
```

In the **PX4 pane** (`pxh>` prompt):

```
commander takeoff
commander land
listener sensor_combined
```

## Configuration knobs

Set these as environment variables in front of the launch command:

```bash
# Different airframe/model
./start_px4_sim.sh gz_rc_cessna

# Different home location (default is IIT Gandhinagar 23.2127, 72.6846)
PX4_HOME_LAT=23.2130 PX4_HOME_LON=72.6850 PX4_HOME_ALT=30 ./start_px4_sim.sh
```

## Notes

- **GPU:** the script forces the RTX 4060 via `prime-run` so Gazebo is smooth.
  Confirm with `watch -n1 nvidia-smi` — you should see `gz sim` using the GPU.
- **Arming without a ground station:** if you ever run without QGC and hit
  "Preflight Fail: No connection to the GCS", set in the PX4 pane:
  `param set NAV_DLL_ACT 0` and `param set CBRK_SUPPLY_CHK 894281`.
- Plug in the charger — the full sim is heavy on the battery.
