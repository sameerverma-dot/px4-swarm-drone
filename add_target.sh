#!/usr/bin/env bash
# add_target.sh - spawn a detectable target into the RUNNING Gazebo sim.
#
#   bash ~/px4_ros_ws/add_target.sh              # person at PX4 N=10, E=15
#   bash ~/px4_ros_ws/add_target.sh 20 5         # person at PX4 N=20, E=5
#   bash ~/px4_ros_ws/add_target.sh 10 15 mine1  # give it a name
#
# WHY THIS EXISTS
# A model inserted by hand through the Gazebo GUI lives only in that session.
# Restart the sim and it is gone - which is exactly why the 5 Sep flight logged
# VERIFY PASS, flew perfect lanes over the target position, and found nothing:
# there was nothing there. Run this after every sim start, or the survey is
# searching an empty field.
#
# COORDINATE TRAP (the one documented in SYSTEM_GUIDE.md §3.1)
#   PX4 local NED : x = North, y = East
#   Gazebo world  : x = East,  y = North     <- SWAPPED
# So PX4 (N=10, E=15) is Gazebo (x=15, y=10). This script takes NORTH EAST in
# PX4 order and does the swap for you - pass the same numbers you pass to
# hazard_map --truth.

set -uo pipefail

NORTH="${1:-10}"
EAST="${2:-15}"
NAME="${3:-target_person}"
WORLD="${GZ_WORLD:-default}"
MODEL_URI="${TARGET_MODEL:-https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person}"

# gz service talks over gz-transport, so it needs the same GZ_IP as everything
# else PX4 launched. Without it the call silently times out.
export GZ_IP=127.0.0.1

if ! command -v gz >/dev/null 2>&1; then
    echo "gz CLI not found on PATH" >&2; exit 1
fi
if ! pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "Gazebo is not running - start the sim first:" >&2
    echo "  bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down" >&2
    exit 1
fi

# NED -> ENU: Gazebo x is East, Gazebo y is North.
GZ_X="$EAST"
GZ_Y="$NORTH"

echo "spawning '$NAME' at PX4 N=$NORTH E=$EAST  (Gazebo x=$GZ_X y=$GZ_Y)"
echo "model: $MODEL_URI"

OUT=$(gz service -s "/world/$WORLD/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 20000 \
    --req "sdf_filename: \"$MODEL_URI\", name: \"$NAME\", allow_renaming: true, pose: {position: {x: $GZ_X, y: $GZ_Y, z: 0}}" 2>&1)
echo "$OUT"

if [[ "$OUT" == *"data: true"* ]]; then
    echo
    echo "OK. Verify it is really there:"
    echo "  GZ_IP=127.0.0.1 gz model --list | grep -i '$NAME'"
    echo
    echo "Then fly, and check the map against this position:"
    echo "  ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=5.0"
    echo "  ros2 run perception hazard_map --area 0,30,0,20 --truth $NORTH,$EAST"
else
    echo
    echo "Spawn did NOT report success." >&2
    echo "Most likely the Fuel model could not be downloaded (needs internet the" >&2
    echo "first time; it is cached in ~/.gz/fuel afterwards)." >&2
    echo "If Fuel is unreachable, insert a person once via the Gazebo GUI" >&2
    echo "(top-right ... menu -> Resource Spawner), which caches it, then re-run this." >&2
    exit 1
fi
