#!/usr/bin/env bash
#
# start_px4_sim.sh — one-shot launcher for the PX4 + ROS 2 + Gazebo stack
#
# Starts, in order:
#   1) the Micro XRCE-DDS Agent   (bridges PX4 uORB topics onto ROS 2 DDS)
#   2) PX4 SITL + Gazebo          (gz_x500 by default)
#   3) a shell with px4_ros_ws sourced, ready for `ros2 ...` commands
#   4) QGroundControl             (if found in ~/Downloads; auto-connects)
#
# Usage:
#   ./start_px4_sim.sh                 # default: x500 quad in Gazebo
#   ./start_px4_sim.sh gz_rc_cessna     # any other PX4 gz_* target
#   PX4_MODEL=gz_x500_depth ./start_px4_sim.sh
#
# With tmux installed you get three real panes (like three terminals) in
# one window. Without tmux it falls back to background processes with
# logs, and Ctrl-C stops everything cleanly.

set -uo pipefail

# ---- paths --------------------------------------------------------------
PX4_DIR="$HOME/PX4-Autopilot"
AGENT_DIR="$HOME/Micro-XRCE-DDS-Agent"
ROS_WS="$HOME/px4_ros_ws"
LOG_DIR="$ROS_WS/log/sim_launch_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

MODEL="${1:-${PX4_MODEL:-gz_x500}}"
SESSION="px4_sim"

# ---- sanity checks --------------------------------------------------------
[ -d "$PX4_DIR" ] || { echo "PX4-Autopilot not found at $PX4_DIR"; exit 1; }
[ -d "$ROS_WS/install" ] || { echo "px4_ros_ws has no install/ — run 'colcon build' in $ROS_WS first"; exit 1; }

AGENT_BIN="$(command -v MicroXRCEAgent || echo "$AGENT_DIR/build/MicroXRCEAgent")"
[ -x "$AGENT_BIN" ] || command -v "$AGENT_BIN" >/dev/null 2>&1 || {
    echo "MicroXRCEAgent binary not found (checked PATH and $AGENT_DIR/build/MicroXRCEAgent)"
    exit 1
}

# ---- force the dedicated GPU (NVIDIA PRIME offload) -----------------------
# On this Optimus laptop, Gazebo defaults to the Intel iGPU and runs slow
# while the RTX GPU idles. PX4 spawns the `gz sim` GUI as a child, so we
# prefix the whole PX4 launch: the offload env vars are inherited by Gazebo.
if command -v prime-run >/dev/null 2>&1; then
    GPU_PREFIX="prime-run"
    GPU_NOTE="prime-run (RTX offload)"
else
    # prime-run not installed — set the same env vars it would, directly.
    GPU_PREFIX="env __NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only __GLX_VENDOR_LIBRARY_NAME=nvidia"
    GPU_NOTE="NVIDIA offload env vars (prime-run not found)"
fi

# ---- simulation home location (default: IIT Gandhinagar, Palaj campus) ----
# PX4 SITL's built-in default home is Zurich. Override it so the drone spawns
# over IIT Gandhinagar; QGroundControl then centers the map there automatically.
# Override per-run with e.g.  PX4_HOME_LAT=.. PX4_HOME_LON=.. ./start_px4_sim.sh
PX4_HOME_LAT="${PX4_HOME_LAT:-23.2127}"
PX4_HOME_LON="${PX4_HOME_LON:-72.6846}"
PX4_HOME_ALT="${PX4_HOME_ALT:-30}"
HOME_ENV="PX4_HOME_LAT=$PX4_HOME_LAT PX4_HOME_LON=$PX4_HOME_LON PX4_HOME_ALT=$PX4_HOME_ALT"

# ---- QGroundControl (optional GUI ground station) ------------------------
# Auto-detected from ~/Downloads; launched detached (setsid) so it survives
# closing the tmux session. It auto-connects to PX4 SITL on UDP 14550.
QGC_APP="$(ls "$HOME/Downloads"/QGroundControl*.AppImage 2>/dev/null | head -n1)"
launch_qgc() {
    [ -n "$QGC_APP" ] || { echo "QGroundControl: not found in ~/Downloads (skipping)"; return; }
    if pgrep -f "QGroundControl.*AppImage" >/dev/null 2>&1; then
        echo "QGroundControl: already running"
        return
    fi
    setsid "$QGC_APP" > "$LOG_DIR/qgc.log" 2>&1 < /dev/null &
    echo "QGroundControl: launched ($QGC_APP)"
}

echo "Logs:         $LOG_DIR"
echo "Gazebo model: $MODEL"
echo "GPU:          $GPU_NOTE"
echo "Home:         IIT Gandhinagar ($PX4_HOME_LAT, $PX4_HOME_LON)"
echo "QGC:          ${QGC_APP:-<not found in ~/Downloads>}"
echo

# ---- clear leftovers from previous runs ----------------------------------
# Prevents "port 8888 already in use" (agent) and "PX4 server already
# running for instance 0" (SITL) when a prior launch didn't shut down.
cleanup_stale() {
    local found=0
    for pat in MicroXRCEAgent "bin/px4 " "px4_sitl" "gz sim" "gz-sim"; do
        if pgrep -f "$pat" >/dev/null 2>&1; then found=1; fi
    done
    if [ "$found" -eq 1 ]; then
        echo "Clearing stale sim processes from a previous run..."
        pkill -f MicroXRCEAgent 2>/dev/null
        pkill -x px4            2>/dev/null
        pkill -f "px4_sitl"     2>/dev/null
        pkill -f "gz sim"       2>/dev/null
        pkill -f "gz-sim"       2>/dev/null
        sleep 2
    fi
}
cleanup_stale

# Guard against being run *inside* an existing tmux session (nested attach
# fails). Re-run this script from a plain terminal instead.
if [ -n "${TMUX:-}" ]; then
    echo "You're inside a tmux session already."
    echo "Detach first (Ctrl-b d) or run 'tmux kill-server', then launch this"
    echo "script from a normal terminal."
    exit 1
fi

# ---- tmux path: three real panes, exactly like three terminals ------------
if command -v tmux >/dev/null 2>&1; then
    tmux kill-session -t "$SESSION" 2>/dev/null || true

    tmux new-session -d -s "$SESSION" -n sim \
        "cd '$AGENT_DIR'; '$AGENT_BIN' udp4 -p 8888 2>&1 | tee '$LOG_DIR/agent.log'"

    # PX4 pane: run WITHOUT piping through tee, otherwise the interactive
    # pxh> console won't accept keyboard input. Log via pipe-pane instead —
    # it taps the pane's output without touching its stdin.
    px4_pane=$(tmux split-window -h -P -F '#{pane_id}' -t "$SESSION:sim" \
        "cd '$PX4_DIR'; $HOME_ENV $GPU_PREFIX make px4_sitl $MODEL")
    tmux pipe-pane -o -t "$px4_pane" "cat >> '$LOG_DIR/px4_sitl.log'"

    tmux split-window -v -t "$SESSION:sim" \
        "cd '$ROS_WS'; source install/setup.bash; \
         echo 'px4_ros_ws sourced — try: ros2 topic list | grep fmu'; \
         exec bash"

    tmux select-layout -t "$SESSION:sim" tiled
    # Enable mouse so panes can be clicked to focus and scrolled with the wheel.
    tmux set -g mouse on

    # Bring up QGroundControl alongside the sim (its own window).
    launch_qgc

    # ---- auto-set SITL "no RC / no GCS" failsafe params ------------------
    # SITL has no RC transmitter and no GCS heartbeat, so PX4's failsafes
    # block arming and Mission/Offboard mode:
    #   "No manual control input"  /  "No connection to the GCS"
    # Once PX4 has booted, type these into the pxh> console and save them so
    # they persist (survives restarts until a clean rebuild):
    #   COM_RC_IN_MODE 4  -> stick input disabled: no manual control source
    #                        required (clears "No manual control input")
    #   COM_RCL_EXCEPT 7  -> except Mission+Hold+Offboard from the RC-loss
    #                        check (bit0 Mission, bit1 Hold, bit2 Offboard)
    #   NAV_RCL_ACT   0   -> disable RC-loss failsafe action
    #   NAV_DLL_ACT   0   -> disable datalink(GCS)-loss failsafe action
    #   CBRK_SUPPLY_CHK 894281 -> disable the (sim-irrelevant) power check
    (
        for _ in $(seq 1 90); do
            grep -q "Startup script returned successfully" \
                "$LOG_DIR/px4_sitl.log" 2>/dev/null && break
            sleep 1
        done
        sleep 2
        for p in "COM_RC_IN_MODE 4" "COM_RCL_EXCEPT 7" "NAV_RCL_ACT 0" "NAV_DLL_ACT 0" "CBRK_SUPPLY_CHK 894281"; do
            tmux send-keys -t "$px4_pane" "param set $p" Enter
            sleep 0.4
        done
        tmux send-keys -t "$px4_pane" "param save" Enter
    ) &
    echo "SITL params:  auto-setting no-RC/no-GCS failsafe params after boot"

    echo "Attaching to tmux session '$SESSION'."
    echo "  Ctrl-b d   → detach (stack keeps running)"
    echo "  tmux attach -t $SESSION   → reattach later"
    echo "  tmux kill-session -t $SESSION   → stop everything"
    sleep 1
    exec tmux attach -t "$SESSION"
fi

# ---- fallback: no tmux — background processes + cleanup trap --------------
echo "tmux not found — running the agent and PX4 SITL/Gazebo in the background."
echo "(for a proper 3-pane view: sudo apt install tmux)"
echo

pids=()
cleanup() {
    echo
    echo "Stopping simulation..."
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup INT TERM EXIT

( cd "$AGENT_DIR" && "$AGENT_BIN" udp4 -p 8888 ) > "$LOG_DIR/agent.log" 2>&1 &
pids+=($!)
sleep 2

( cd "$PX4_DIR" && env $HOME_ENV $GPU_PREFIX make px4_sitl "$MODEL" ) > "$LOG_DIR/px4_sitl.log" 2>&1 &
pids+=($!)

# Bring up QGroundControl alongside the sim (its own window).
launch_qgc

echo "Agent (pid ${pids[0]}) and PX4 SITL/Gazebo (pid ${pids[1]}) starting in the background."
echo "  tail -f $LOG_DIR/agent.log"
echo "  tail -f $LOG_DIR/px4_sitl.log"
echo
echo "Sourcing px4_ros_ws in this shell now — Ctrl-C stops everything below."
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
echo "px4_ros_ws sourced — try: ros2 topic list | grep fmu"
exec bash
