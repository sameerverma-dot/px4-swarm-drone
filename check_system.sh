#!/usr/bin/env bash
# check_system.sh - what is actually working right now?
#
# Probes the stack layer by layer and prints PASS / FAIL / SKIP for each, so a
# failure tells you WHICH layer broke instead of "the mission didn't work".
#
#   bash ~/px4_ros_ws/check_system.sh          # full check (sim should be running)
#   bash ~/px4_ros_ws/check_system.sh --quick  # skip the live-topic probes
#
# Read-only: starts nothing, kills nothing, changes nothing.
#
# Layers, in dependency order - the first FAIL is the one to fix:
#   0 environment   ROS 2 + workspace built
#   1 processes     DDS agent, PX4, Gazebo
#   2 telemetry     PX4 -> ROS 2 over DDS          (the versioned-topic trap)
#   3 camera        Gazebo -> ROS 2 over gz-transport (the GZ_IP trap)
#   4 inference     torch / CUDA / ultralytics / cv2
#   5 outputs       ~/maps and the map renderer

set -uo pipefail

QUICK=0
[[ "${1:-}" == "--quick" ]] && QUICK=1

WS="$HOME/px4_ros_ws"
CAM_GZ="/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
export GZ_IP=127.0.0.1

G=$'\033[0;32m'; R=$'\033[0;31m'; Y=$'\033[0;33m'; D=$'\033[2m'; N=$'\033[0m'
pass=0; fail=0; skip=0
FIRST_FAIL=""

ok()   { printf "  ${G}PASS${N}  %-34s %s\n" "$1" "${2:-}"; pass=$((pass+1)); }
no()   { printf "  ${R}FAIL${N}  %-34s %s\n" "$1" "${2:-}"; fail=$((fail+1));
         [[ -z "$FIRST_FAIL" ]] && FIRST_FAIL="$1|${3:-}"; return 0; }
sk()   { printf "  ${Y}SKIP${N}  %-34s %s\n" "$1" "${2:-}"; skip=$((skip+1)); }
hdr()  { printf "\n${D}%s${N}\n" "$1"; }

# ---------------------------------------------------------------- 0 environment
hdr "0 · environment"

if [[ -n "${ROS_DISTRO:-}" ]]; then
    ok "ROS 2 sourced" "$ROS_DISTRO"
else
    no "ROS 2 sourced" "ROS_DISTRO unset" "source /opt/ros/humble/setup.bash"
fi

if [[ -f "$WS/install/setup.bash" ]]; then
    ok "workspace built" "$WS/install"
else
    no "workspace built" "no install/setup.bash" \
       "cd $WS && colcon build --packages-select survey perception"
fi

# Are the built packages the CURRENT source? A stale build is the single most
# common reason a fix "didn't work".
stale=""
if [[ ! -f "$WS/install/setup.bash" ]]; then
    # Without a build there is nothing to be stale against - don't report a
    # vacuous PASS here, the missing build is already the failure above.
    sk "build up to date" "nothing built yet"
else
for pkg in survey perception; do
    # Compare the newest SOURCE .py against the newest INSTALLED .py.
    # (The previous version compared against the install/<pkg> DIRECTORY, whose
    # mtime only changes when its direct children do - so it reported "stale"
    # immediately after a successful build. False alarms train you to ignore
    # the check, which defeats the point of having it.)
    src_t=$(find "$WS/src/$pkg" -name '*.py' -printf '%T@\n' 2>/dev/null | sort -n | tail -1)
    inst_t=$(find "$WS/install/$pkg" -name '*.py' -printf '%T@\n' 2>/dev/null | sort -n | tail -1)
    if [[ -z "$inst_t" ]]; then
        stale="$stale $pkg(not-installed)"
    elif awk "BEGIN{exit !(${src_t:-0} > $inst_t + 2)}"; then
        stale="$stale $pkg"
    fi
done
if [[ -n "$stale" ]]; then
    no "build up to date" "source newer than build:$stale" \
       "cd $WS && colcon build --packages-select survey perception && source install/setup.bash"
else
    ok "build up to date" ""
fi
fi

for f in src/survey/survey/survey_node.py \
         src/perception/perception/detector_node.py \
         src/perception/perception/hazard_map.py; do
    [[ -f "$WS/$f" ]] && ok "source present" "$(basename "$f")" \
                      || no "source present" "MISSING $f"
done

# ---------------------------------------------------------------- 1 processes
hdr "1 · processes"

proc_check() {  # name pattern hint
    if pgrep -f "$2" >/dev/null 2>&1; then
        ok "$1" "pid $(pgrep -f "$2" | head -1)"
    else
        no "$1" "not running" "$3"
    fi
}
LAUNCH="bash $WS/start_px4_sim.sh gz_x500_mono_cam_down"
proc_check "Micro XRCE-DDS agent" "MicroXRCEAgent"  "$LAUNCH"
proc_check "PX4 SITL"             "bin/px4"         "$LAUNCH"
proc_check "Gazebo server"        "gz sim"          "$LAUNCH"

SIM_UP=0
pgrep -f "bin/px4" >/dev/null 2>&1 && pgrep -f "MicroXRCEAgent" >/dev/null 2>&1 && SIM_UP=1

# ---------------------------------------------------------------- 2 telemetry
hdr "2 · telemetry  (PX4 -> ROS 2, over DDS)"

# NOTE: /fmu/out/* is BEST_EFFORT. `ros2 topic echo --qos-reliability
# best_effort --once` is the probe that is unambiguously correct here - a
# default RELIABLE subscriber matches nothing and reports silence even when
# data is flowing. Also: a topic APPEARING in `ros2 topic list` proves nothing,
# because a subscriber creates that entry too.
live_topic() {  # topic -> 0 if a message arrives within N seconds
    timeout "${2:-6}" ros2 topic echo "$1" --qos-reliability best_effort --once \
        >/dev/null 2>&1
}

if [[ $QUICK -eq 1 ]]; then
    sk "telemetry probes" "--quick"
elif [[ $SIM_UP -eq 0 ]]; then
    sk "telemetry probes" "sim not running"
else
    FOUND=""
    for t in /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_local_position; do
        if live_topic "$t"; then FOUND="$t"; break; fi
    done
    if [[ -n "$FOUND" ]]; then
        ok "vehicle_local_position" "live on $FOUND"
        [[ "$FOUND" == *_v1 ]] || printf "        ${Y}note${N} unversioned topic is the live one on this build\n"
    else
        no "vehicle_local_position" "NO DATA on either name" \
           "agent running but PX4 not connected - restart the stack"
    fi

    FOUND=""
    for t in /fmu/out/vehicle_status_v4 /fmu/out/vehicle_status; do
        if live_topic "$t"; then FOUND="$t"; break; fi
    done
    [[ -n "$FOUND" ]] && ok "vehicle_status" "live on $FOUND" \
                      || no "vehicle_status" "NO DATA on either name"
fi

# ---------------------------------------------------------------- 3 camera
hdr "3 · camera  (Gazebo -> ROS 2, over gz-transport)"

if [[ $QUICK -eq 1 ]]; then
    sk "camera probes" "--quick"
elif ! pgrep -f "gz sim" >/dev/null 2>&1; then
    sk "camera probes" "Gazebo not running"
elif ! command -v gz >/dev/null 2>&1; then
    sk "camera probes" "gz CLI not on PATH"
else
    if gz topic -l 2>/dev/null | grep -q "camera/image"; then
        ok "camera advertised" "gz lists the topic"
    else
        no "camera advertised" "not in gz topic -l" \
           "wrong model? launch with gz_x500_mono_cam_down"
    fi
    # Advertised != delivering. THIS is the check that catches a GZ_IP mismatch.
    BYTES=$(timeout 6 gz topic -e -t "$CAM_GZ" 2>/dev/null | head -c 200 | wc -c)
    if [[ "${BYTES:-0}" -gt 0 ]]; then
        ok "camera DELIVERING frames" "with GZ_IP=127.0.0.1"
    else
        no "camera DELIVERING frames" "advertised but silent" \
           "GZ_IP mismatch - see CAMERA_DIAGNOSTIC.md"
    fi

    if timeout 6 ros2 topic echo "$CAM_GZ" --once >/dev/null 2>&1; then
        ok "camera bridged into ROS 2" ""
    else
        sk "camera bridged into ROS 2" "bridge not running (starts with the mission)"
    fi
fi

# ---------------------------------------------------------------- 4 inference
hdr "4 · inference"

py_has() { python3 -c "import $1" >/dev/null 2>&1; }

# "missing" and "installed but won't import" are DIFFERENT faults with different
# fixes, and calling both "missing" sends you in a circle: pip replies "already
# satisfied" while the check keeps failing. Same trap as a topic being listed
# but not published - report what is actually wrong.
py_check() {   # module  pip-name  [note]
    local err
    if err=$(python3 -c "import $1" 2>&1); then
        ok "python: $1" "${3:-}"
    elif [[ "$err" == *"No module named"* ]]; then
        no "python: $1" "not installed" "pip install --user $2"
    else
        local why
        why=$(printf '%s' "$err" | grep -v '^\s*File \|^\s*$\|^Traceback' | tail -1 | cut -c1-58)
        no "python: $1" "INSTALLED BUT BROKEN - $why" \
           "pip install --user --upgrade $2     # usually a numpy ABI mismatch"
    fi
}

py_check cv2         opencv-python
py_check numpy       numpy
py_check ultralytics ultralytics
py_check matplotlib  matplotlib  "(hazard_map)"

if py_has torch; then
    read -r TV CU GPU < <(python3 - <<'PY'
import torch
print(torch.__version__, torch.cuda.is_available(),
      (torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'n/a').replace(' ', '_'))
PY
)
    if [[ "$CU" == "True" ]]; then
        ok "torch + CUDA" "$TV on ${GPU//_/ }"
    else
        no "torch + CUDA" "torch $TV, CUDA NOT available -> YOLO on CPU" \
           "pip install --user --force-reinstall torch --index-url https://download.pytorch.org/whl/cu121"
    fi
else
    no "torch" "missing" "pip install --user torch"
fi

# setuptools >= 80 silently breaks colcon
if py_has setuptools; then
    SV=$(python3 -c "import setuptools;print(setuptools.__version__)" 2>/dev/null)
    if [[ "${SV%%.*}" -ge 80 ]] 2>/dev/null; then
        no "setuptools < 80" "$SV breaks colcon" 'pip install --user "setuptools==70.3.0"'
    else
        ok "setuptools < 80" "$SV"
    fi
fi

# ---------------------------------------------------------------- 5 outputs
hdr "5 · outputs"

if [[ -d "$HOME/maps" ]]; then
    NT=$(ls "$HOME"/maps/survey_track_*.csv 2>/dev/null | wc -l)
    ok "~/maps exists" "$NT track file(s)"
    LATEST=$(ls -t "$HOME"/maps/survey_track_*.csv 2>/dev/null | head -1)
    if [[ -n "$LATEST" ]]; then
        ROWS=$(($(wc -l < "$LATEST") - 1))
        if [[ "$ROWS" -gt 10 ]]; then
            ok "latest track has data" "$ROWS samples · $(basename "$LATEST")"
        else
            no "latest track has data" "only $ROWS samples - telemetry was dead that run"
        fi
    fi
else
    sk "~/maps exists" "no missions flown yet"
fi

# ---------------------------------------------------------------- verdict
printf "\n${D}%s${N}\n" "────────────────────────────────────────────────────────"
printf "  ${G}%d passed${N}   ${R}%d failed${N}   ${Y}%d skipped${N}\n" "$pass" "$fail" "$skip"
if [[ $fail -eq 0 ]]; then
    printf "\n  Everything probed is working. Fly it:\n"
    printf "    ${D}cd %s && source install/setup.bash${N}\n" "$WS"
    printf "    ${D}ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=5.0${N}\n"
else
    printf "\n  Fix the FIRST failure above - later layers depend on earlier ones.\n"
    if [[ -n "$FIRST_FAIL" ]]; then
        printf "    first: ${R}%s${N}\n" "${FIRST_FAIL%%|*}"
        [[ -n "${FIRST_FAIL#*|}" ]] && printf "    try:   ${D}%s${N}\n" "${FIRST_FAIL#*|}"
    fi
fi
printf "\n"
exit 0
