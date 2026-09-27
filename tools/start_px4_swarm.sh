#!/usr/bin/env bash
#
# start_px4_swarm.sh — one-shot launcher for N PX4 + Gazebo instances
#
# Same shape as start_px4_sim.sh (which stays the single-drone entry point -
# this script does NOT replace it), extended to multiple vehicles per PX4's
# own documented convention (docs/en/sim_gazebo_gz/multi_vehicle_simulation.md,
# verified against px4-rc.gzsim in this checkout, not guessed):
#
#   instance 0   make px4_sitl <model>              hosts the gz-sim world
#   instance i>0 PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="<east>,0" \
#                ./build/px4_sitl_default/bin/px4 -i <i>   spawns INTO it
#
# Instance i namespaces its DDS topics as /px4_<i>/fmu/... (instance 0 stays
# unnamespaced /fmu/...) and the gz camera topic becomes
# .../model/<model>_<i>/link/camera_link/sensor/camera/image - see
# px4-rc.gzsim's MODEL_NAME_INSTANCE. VERIFY this is actually live before
# flying anything - this project's most expensive bug was assuming a topic
# name without checking:
#
#   ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once
#
# Each drone is spawned at a pure-EAST offset of Y_MIN + i*(Y_MAX-Y_MIN)/N,
# so ITS OWN local NED frame origin equals that offset - which is exactly
# what swarm_mission.launch.py's per-drone survey box and detector
# home_offsets assume. NUM_DRONES/Y_MIN/Y_MAX here MUST match the values
# passed to swarm_mission.launch.py, because the two compute the band
# geometry independently and there is no shared source of truth between a
# shell script and a ROS launch file.
#
# Usage (flags and environment variables are equivalent; flags win):
#   bash tools/start_px4_swarm.sh                        # 2 drones, defaults
#   bash tools/start_px4_swarm.sh --num-drones 2 --y-min 0 --y-max 60 gz_x500_mono_cam_down
#   NUM_DRONES=2 Y_MIN=0 Y_MAX=60 bash tools/start_px4_swarm.sh gz_x500_mono_cam_down
#
# Stop everything: bash tools/stop_sim.sh
# (`tmux kill-server` alone is NOT enough - PX4 and Gazebo survive it.)

set -uo pipefail

PX4_DIR="$HOME/PX4-Autopilot"
AGENT_DIR="$HOME/Micro-XRCE-DDS-Agent"
ROS_WS="$HOME/px4_ros_ws"

NUM_DRONES="${NUM_DRONES:-2}"
Y_MIN="${Y_MIN:-0.0}"
Y_MAX="${Y_MAX:-60.0}"
MODEL="${PX4_MODEL:-gz_x500_mono_cam_down}"
# Parse flags. Anything starting with '-' that isn't known is an error: before
# this, `--num-drones 2` was taken as the Gazebo MODEL and PX4's build died on
# `make px4_sitl --num-drones` - with swarm_mission.launch.py's own docstring
# telling people to type exactly that.
while [ $# -gt 0 ]; do
    case "$1" in
        --num-drones) NUM_DRONES="${2:-}"; shift 2 ;;
        --y-min)      Y_MIN="${2:-}";      shift 2 ;;
        --y-max)      Y_MAX="${2:-}";      shift 2 ;;
        -h|--help)    sed -n '2,40p' "$0"; exit 0 ;;
        -*)           echo "unknown option '$1' (try --help)"; exit 1 ;;
        *)            MODEL="$1"; shift ;;
    esac
done

SESSION="px4_swarm"
# RTL_RETURN_ALT staggered per instance so simultaneous RTLs don't climb to
# the same altitude over the same field (NEXT_SESSION.md's "RTL converging" trap).
RTL_ALT_BASE="${RTL_ALT_BASE:-30}"
RTL_ALT_STEP="${RTL_ALT_STEP:-5}"

[ -d "$PX4_DIR" ] || { echo "PX4-Autopilot not found at $PX4_DIR"; exit 1; }
[ -d "$ROS_WS/install" ] || { echo "px4_ros_ws has no install/ — run 'colcon build' in $ROS_WS first"; exit 1; }
if ! [[ "$NUM_DRONES" =~ ^[0-9]+$ ]] || [ "$NUM_DRONES" -lt 1 ]; then
    echo "NUM_DRONES must be a positive integer, got '$NUM_DRONES'"; exit 1
fi
if ! awk -v a="$Y_MIN" -v b="$Y_MAX" 'BEGIN{exit !(b+0 > a+0)}'; then
    echo "Y_MAX ($Y_MAX) must be greater than Y_MIN ($Y_MIN)"; exit 1
fi

LOG_DIR="$ROS_WS/log/swarm_launch_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

AGENT_BIN="$(command -v MicroXRCEAgent || echo "$AGENT_DIR/build/MicroXRCEAgent")"
[ -x "$AGENT_BIN" ] || command -v "$AGENT_BIN" >/dev/null 2>&1 || {
    echo "MicroXRCEAgent binary not found (checked PATH and $AGENT_DIR/build/MicroXRCEAgent)"
    exit 1
}

if command -v prime-run >/dev/null 2>&1; then
    GPU_PREFIX="prime-run"
    GPU_NOTE="prime-run (RTX offload)"
else
    GPU_PREFIX="env __NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only __GLX_VENDOR_LIBRARY_NAME=nvidia"
    GPU_NOTE="NVIDIA offload env vars (prime-run not found)"
fi

PX4_HOME_LAT="${PX4_HOME_LAT:-23.2127}"
PX4_HOME_LON="${PX4_HOME_LON:-72.6846}"
PX4_HOME_ALT="${PX4_HOME_ALT:-30}"
HOME_ENV="PX4_HOME_LAT=$PX4_HOME_LAT PX4_HOME_LON=$PX4_HOME_LON PX4_HOME_ALT=$PX4_HOME_ALT"

QGC_APP="$(ls "$HOME/Downloads"/QGroundControl*.AppImage 2>/dev/null | head -n1)"
launch_qgc() {
    [ -n "$QGC_APP" ] || { echo "QGroundControl: not found in ~/Downloads (skipping)"; return; }
    if pgrep -f "QGroundControl.*AppImage" >/dev/null 2>&1; then
        echo "QGroundControl: already running"; return
    fi
    setsid "$QGC_APP" > "$LOG_DIR/qgc.log" 2>&1 < /dev/null &
    echo "QGroundControl: launched ($QGC_APP) — connects to instance 0 only"
}

# Compute each drone's EAST spawn offset now, in bash, so the log states the
# exact numbers swarm_mission.launch.py must be given too.
echo "Logs:         $LOG_DIR"
echo "Gazebo model: $MODEL"
echo "GPU:          $GPU_NOTE"
echo "Drones:       $NUM_DRONES, area y=[$Y_MIN,$Y_MAX] split along EAST"
BAND_H=$(awk -v a="$Y_MIN" -v b="$Y_MAX" -v n="$NUM_DRONES" 'BEGIN{printf "%.4f", (b-a)/n}')
for ((i=0; i<NUM_DRONES; i++)); do
    off=$(awk -v a="$Y_MIN" -v h="$BAND_H" -v i="$i" 'BEGIN{printf "%.4f", a+i*h}')
    ns="/fmu (unnamespaced)"; [ "$i" -ne 0 ] && ns="/px4_$i/fmu"
    echo "  drone $i: spawn east offset ${off} m, DDS namespace ${ns}"
done
echo "Give swarm_mission.launch.py the SAME: num_drones:=$NUM_DRONES y_min:=$Y_MIN y_max:=$Y_MAX"
echo

cleanup_stale() {
    # Delegated to tools/stop_sim.sh: it kills by process (tmux teardown leaves
    # PX4 and Gazebo running), and never kills its own ancestors. The old
    # `pkill -f px4_sitl` here also hit any shell or `tail -f` watcher whose
    # command line merely contained it.
    bash "$ROS_WS/tools/stop_sim.sh" | sed 's/^/cleanup: /'
}

# Checked BEFORE cleanup, which closes the px4_sim/px4_swarm sessions -
# possibly the very pane this is being run from.
if [ -n "${TMUX:-}" ]; then
    echo "You're inside a tmux session already."
    echo "Detach first (Ctrl-b d) or run 'bash $ROS_WS/tools/stop_sim.sh', then"
    echo "launch this script from a normal terminal."
    exit 1
fi
cleanup_stale

command -v tmux >/dev/null 2>&1 || { echo "tmux is required for the swarm launcher (sudo apt install tmux)"; exit 1; }

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n sim \
    "cd '$AGENT_DIR'; '$AGENT_BIN' udp4 -p 8888 2>&1 | tee '$LOG_DIR/agent.log'"

# ---- instance 0: hosts the gz-sim world (identical to start_px4_sim.sh) ----
px4_pane_0=$(tmux split-window -h -P -F '#{pane_id}' -t "$SESSION:sim" \
    "cd '$PX4_DIR'; $HOME_ENV $GPU_PREFIX make px4_sitl $MODEL")
tmux pipe-pane -o -t "$px4_pane_0" "cat >> '$LOG_DIR/px4_sitl_0.log'"

tmux select-layout -t "$SESSION:sim" tiled
tmux set -g mouse on
launch_qgc

# ---- wait for instance 0's build + world before spawning standalone instances ----
echo "Waiting for instance 0 to build and boot (this builds PX4 if needed - can take a while)..."
(
    for _ in $(seq 1 600); do
        grep -aq "Startup script returned successfully" "$LOG_DIR/px4_sitl_0.log" 2>/dev/null && break
        sleep 1
    done
) &
WAIT_PID=$!
wait "$WAIT_PID"
if ! grep -aq "Startup script returned successfully" "$LOG_DIR/px4_sitl_0.log" 2>/dev/null; then
    echo "WARNING: instance 0 did not report ready within 600s — spawning the"
    echo "rest anyway, but check $LOG_DIR/px4_sitl_0.log if they fail to connect."
fi
sleep 2

# ---- sensor health gate -----------------------------------------------------
# A standalone instance can boot perfectly, register every DDS writer, and put
# its model in the world while receiving NO sensor data from Gazebo. PX4 says so
# plainly in its own log ("No valid data from Accel 0", "barometer 0 missing",
# "Found 0 compass"), but nothing downstream notices: `ros2 topic list` shows
# /px4_1/fmu/out/vehicle_local_position_v1 because the WRITER exists, and
# `ros2 topic echo` then hangs forever because EKF2 never produced a solution.
#
# Observed 21 Sep 2026: identical script, identical boot sequence to the working
# 16 Sep flight - the only difference was that instance 1 attached 2.2 s later in
# sim time. A race, not a misconfiguration. So the launcher must CHECK rather
# than sleep and hope; an instance that came up blind is caught in ~20 s here
# instead of 5 minutes into a mission that cannot arm.
sensors_blind() {
    grep -aq "Preflight Fail: No valid data from Accel\|Preflight Fail: barometer 0 missing" "$1" 2>/dev/null
}

wait_for_instance() {   # $1=logfile  $2=instance index -> 0 healthy, 1 blind, 2 never booted
    local logf="$1" idx="$2"
    for _ in $(seq 1 90); do
        grep -aq "Startup script returned successfully" "$logf" 2>/dev/null && break
        sleep 1
    done
    # An instance that never finished booting used to fall through to "sensors
    # OK" here, because the blind check only looks for failure messages.
    grep -aq "Startup script returned successfully" "$logf" 2>/dev/null || return 2
    # Give the sensor pipeline a moment to either start or visibly fail.
    for _ in $(seq 1 20); do
        sensors_blind "$logf" && return 1
        sleep 1
    done
    sensors_blind "$logf" && return 1
    return 0
}

spawn_instance() {      # $1=index -> echoes the new pane id
    # GZ_IP=127.0.0.1 is NOT optional and is the whole reason drone 1 used to
    # come up blind. On this machine gz-transport DISCOVERS topics without it
    # but does not DELIVER data - the distinction tools/check_system.sh already
    # measures separately ("camera advertised" vs "camera DELIVERING frames
    # with GZ_IP=127.0.0.1"), and the same distinction that cost this project
    # days on the camera bridge (docs/CAMERA_DIAGNOSTIC.md). Measured 21 Sep:
    # `gz topic -l` returns 36 topics bare and 51 with GZ_IP=127.0.0.1.
    #
    # Instance 0 gets away without it because it STARTS the gz server itself.
    # Every standalone instance joins an already-running server as a separate
    # process, so it must advertise the same address or it subscribes to the
    # IMU, gets silence, and PX4 reports "No valid data from Accel 0" while the
    # topic sits visibly in the entity tree publishing at full rate.
    local i="$1"
    local off
    off=$(awk -v a="$Y_MIN" -v h="$BAND_H" -v i="$i" 'BEGIN{printf "%.4f", a+i*h}')
    local pane
    pane=$(tmux split-window -v -t "$SESSION:sim" -P -F '#{pane_id}' \
        "cd '$PX4_DIR'; GZ_IP=127.0.0.1 PX4_SIM_MODEL=$MODEL PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE=\"${off},0\" $HOME_ENV $GPU_PREFIX ./build/px4_sitl_default/bin/px4 -i $i")
    tmux pipe-pane -o -t "$pane" "cat >> '$LOG_DIR/px4_sitl_$i.log'"
    tmux select-layout -t "$SESSION:sim" tiled
    echo "$pane"
}

# ---- instances 1..N-1: standalone, spawned into the running world ----
declare -a px4_panes=("$px4_pane_0")
BLIND=0
for ((i=1; i<NUM_DRONES; i++)); do
    logf="$LOG_DIR/px4_sitl_$i.log"
    pane=$(spawn_instance "$i")
    echo -n "  drone $i: booting... "
    wait_for_instance "$logf" "$i"; rc=$?
    if [ "$rc" -eq 0 ]; then
        echo "sensors OK"
    else
        # One retry. Killing the tmux pane does NOT reliably kill the px4 inside
        # it (PX4 and Gazebo both survive tmux teardown on this machine - see
        # tools/stop_sim.sh), so kill this instance's process explicitly, or the
        # retry collides with "PX4 server already running for instance $i".
        # The model stays in the world; the new px4 re-attaches to it by name.
        [ "$rc" -eq 2 ] && echo "DID NOT BOOT in 90 s - retrying once" \
                        || echo "NO SENSOR DATA - retrying once"
        tmux kill-pane -t "$pane" 2>/dev/null
        pkill -f "px4_sitl_default/bin/px4 -i $i\$" 2>/dev/null
        sleep 4
        : > "$logf"
        pane=$(spawn_instance "$i")
        echo -n "  drone $i: retry...  "
        if wait_for_instance "$logf" "$i"; then
            echo "sensors OK"
        else
            echo "STILL BLIND / NOT BOOTED"
            BLIND=$((BLIND+1))
        fi
    fi
    px4_panes+=("$pane")
    sleep 1
done

if [ "$BLIND" -gt 0 ]; then
    echo
    echo "=============================================================="
    echo " $BLIND instance(s) came up with NO SENSOR DATA from Gazebo."
    echo " Do NOT fly - they cannot arm, and the mission will hang at"
    echo " 'waiting for offboard'. Their /fmu/out topics will be LISTED"
    echo " but will publish nothing, which looks like a dead topic."
    echo
    echo " Tear down completely and relaunch:"
    echo "   bash $ROS_WS/tools/stop_sim.sh"
    echo
    echo " Logs: $LOG_DIR/px4_sitl_*.log"
    echo "=============================================================="
    echo
fi

tmux split-window -v -t "$SESSION:sim" \
    "cd '$ROS_WS'; export GZ_IP=127.0.0.1; source install/setup.bash; \
     echo 'px4_ros_ws sourced — verify namespacing before flying:'; \
     echo '  ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once'; \
     exec bash"
tmux select-layout -t "$SESSION:sim" tiled

# ---- auto-set failsafe params + staggered RTL altitude on EVERY instance ----
(
    for idx in "${!px4_panes[@]}"; do
        pane="${px4_panes[$idx]}"
        rtl_alt=$((RTL_ALT_BASE + idx * RTL_ALT_STEP))
        (
            logf="$LOG_DIR/px4_sitl_${idx}.log"
            for _ in $(seq 1 90); do
                grep -aq "Startup script returned successfully" "$logf" 2>/dev/null && break
                sleep 1
            done
            sleep 2
            for p in "COM_RC_IN_MODE 4" "COM_RCL_EXCEPT 7" "NAV_RCL_ACT 0" "NAV_DLL_ACT 0" \
                     "CBRK_SUPPLY_CHK 894281" "RTL_RETURN_ALT $rtl_alt"; do
                tmux send-keys -t "$pane" "param set $p" Enter
                sleep 0.4
            done
            tmux send-keys -t "$pane" "param save" Enter
        ) &
    done
    wait
) &
echo "SITL params:  auto-setting no-RC/no-GCS + staggered RTL_RETURN_ALT on all $NUM_DRONES instances"

echo "Attaching to tmux session '$SESSION'."
echo "  Ctrl-b d   → detach (stack keeps running)"
echo "  tmux attach -t $SESSION   → reattach later"
echo "  bash $ROS_WS/tools/stop_sim.sh   → stop everything (tmux kill alone leaves PX4/Gazebo running)"
sleep 1
exec tmux attach -t "$SESSION"
