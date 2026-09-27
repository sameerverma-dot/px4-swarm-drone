#!/usr/bin/env bash
# stop_sim.sh - stop the whole stack (single-drone or swarm) and PROVE it stopped.
#
#   bash ~/px4_ros_ws/tools/stop_sim.sh          # sim + mission nodes, QGC left open
#   bash ~/px4_ros_ws/tools/stop_sim.sh --all     # ...and QGroundControl too
#
# Why not just `tmux kill-server`: it closes the panes, but `make px4_sitl`,
# the px4 binary and both `gz sim` processes survive the pane's SIGHUP and keep
# running - holding the GPU, the DDS agent's port 8888, and PX4 instance locks
# (observed repeatedly, Sep 2026). A mission launched in another terminal also
# keeps its detector writing to hazard_points.csv. This script kills by process,
# not by pane, then checks that nothing is left.

set -uo pipefail

ALL=0
[[ "${1:-}" == "--all" ]] && ALL=1

# pgrep/pkill -f patterns, most-dependent first
PATTERNS=(
    "ros2 launch survey"
    "ros2 launch perception"
    "survey/lib/survey/survey_node"
    "perception/lib/perception/detector_node"
    "parameter_bridge .*camera/image"
    "MicroXRCEAgent"
    "make px4_sitl"
    "px4_sitl_default"
    "gz sim"
    "gz-sim"
)
[[ $ALL -eq 1 ]] && PATTERNS+=("QGroundControl")

for s in px4_sim px4_swarm; do
    tmux kill-session -t "$s" 2>/dev/null && echo "tmux session '$s' closed"
done

# Never match ourselves or whatever launched us: `pkill -f 'gz sim'` also hits
# any parent shell whose command line merely CONTAINS 'gz sim' (e.g. a
# `bash -c "...gz sim..."` wrapper), and killed it mid-run in testing.
ANCESTORS=" "
p=$$
while [[ -n "$p" && "$p" -gt 1 ]]; do
    ANCESTORS+="$p "
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
done

matching() {   # PIDs of stack processes, excluding our own ancestry
    { for pat in "${PATTERNS[@]}"; do pgrep -f "$pat"; done; pgrep -x px4; } 2>/dev/null |
        sort -un | while read -r pid; do
            [[ "$ANCESTORS" == *" $pid "* ]] || echo "$pid"
        done
}
left() { local pids; pids=$(matching); [[ -n "$pids" ]] && ps -o pid=,args= -p $pids 2>/dev/null; }
signal_all() { local pids; pids=$(matching); [[ -n "$pids" ]] && kill "-$1" $pids 2>/dev/null; }

signal_all TERM
for _ in $(seq 1 10); do
    [[ -z "$(left)" ]] && break
    sleep 0.5
done

if [[ -n "$(left)" ]]; then
    echo "still running after SIGTERM - sending SIGKILL:"
    left | cut -c1-110 | sed 's/^/  /'
    signal_all KILL
    sleep 1
fi

REMAINING=$(left)
if [[ -n "$REMAINING" ]]; then
    echo "FAILED - these are still running:"
    echo "$REMAINING" | sed 's/^/  /'
    exit 1
fi
echo "stopped: no PX4, Gazebo, DDS agent or mission nodes running"
[[ $ALL -eq 0 ]] && pgrep -f QGroundControl >/dev/null 2>&1 && \
    echo "(QGroundControl left open - use --all to close it too)"
exit 0
