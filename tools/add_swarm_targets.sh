#!/usr/bin/env bash
#
# add_swarm_targets.sh — one target per drone band, for a SWARM run.
#
#   NUM_DRONES=2 Y_MIN=0 Y_MAX=60 bash tools/add_swarm_targets.sh
#   NUM_DRONES=3 Y_MIN=0 Y_MAX=90 X_MAX=30 bash tools/add_swarm_targets.sh
#
# WHY THIS EXISTS (and why tools/add_target.sh is not enough)
#
# add_target.sh spawns ONE person at a fixed spot — by default PX4 N=10, E=15,
# which lands inside drone 0's band and nowhere near anyone else's. The 12 Sep
# 2-drone flight was run that way and `hazard_points.csv` recorded exactly one
# detection for the whole sortie. Every other drone flew a clean, verified,
# completely empty survey. That is a test that cannot fail and therefore cannot
# pass: the thing a swarm run is supposed to prove is that drone i's detection,
# taken in drone i's OWN local NED frame, is folded back into drone 0's frame by
# the detector's `home_offsets`. With nothing under drone i, home_offsets is
# never exercised at all.
#
# This script puts a person in the middle of every band, so each drone has
# something to find and each per-drone offset gets checked independently.
#
# BAND MATH — must match tools/start_px4_swarm.sh and swarm_mission.launch.py
#
#   band_h        = (Y_MAX - Y_MIN) / NUM_DRONES
#   drone i origin= global east  Y_MIN + i*band_h          (its spawn pose)
#   drone i covers= global east [Y_MIN + i*band_h, Y_MIN + (i+1)*band_h]
#                   global north [X_MIN, X_MAX]
#   target i      = global north NORTH, east Y_MIN + (i+0.5)*band_h
#
# There is no shared source of truth between a shell script and a ROS launch
# file, so NUM_DRONES / Y_MIN / Y_MAX must be passed identically to all three
# (this script, the swarm launcher, and swarm_mission.launch.py). Get one of
# them wrong and the drones fly bands that do not contain the targets.
#
# COORDINATE TRAP (SYSTEM_GUIDE.md section 3.1)
#   PX4 local NED : x = North, y = East
#   Gazebo world  : x = East,  y = North      <- SWAPPED
# This script takes and prints everything in PX4 order (North, East) and does
# the swap itself, so the numbers here are the same numbers you give
# `hazard_map --truth`.

set -uo pipefail

NUM_DRONES="${NUM_DRONES:-2}"
Y_MIN="${Y_MIN:-0.0}"
Y_MAX="${Y_MAX:-60.0}"
X_MIN="${X_MIN:-0.0}"
X_MAX="${X_MAX:-30.0}"
# Where along the lane (north) to put every target. Default: middle of the
# surveyed strip, so it is far from the turn-arounds at either end where the
# aircraft is banking and the nadir assumption is worst.
NORTH="${NORTH:-}"
PREFIX="${PREFIX:-swarm_target}"
WORLD="${GZ_WORLD:-default}"
MODEL_URI="${TARGET_MODEL:-https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person}"

export GZ_IP=127.0.0.1

# ---- preconditions ---------------------------------------------------------

command -v gz >/dev/null 2>&1 || { echo "gz CLI not found on PATH" >&2; exit 1; }

if ! pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "Gazebo is not running — start the swarm sim first:" >&2
    echo "  NUM_DRONES=$NUM_DRONES Y_MIN=$Y_MIN Y_MAX=$Y_MAX bash tools/start_px4_swarm.sh gz_x500_mono_cam_down" >&2
    exit 1
fi

if ! [[ "$NUM_DRONES" =~ ^[0-9]+$ ]] || [ "$NUM_DRONES" -lt 1 ]; then
    echo "NUM_DRONES must be a positive integer, got '$NUM_DRONES'" >&2; exit 1
fi

BAND_H=$(awk -v a="$Y_MIN" -v b="$Y_MAX" -v n="$NUM_DRONES" 'BEGIN{printf "%.4f", (b-a)/n}')
if awk -v h="$BAND_H" 'BEGIN{exit !(h <= 0)}'; then
    echo "band height is $BAND_H m — Y_MAX ($Y_MAX) must be greater than Y_MIN ($Y_MIN)" >&2
    exit 1
fi

[ -n "$NORTH" ] || NORTH=$(awk -v a="$X_MIN" -v b="$X_MAX" 'BEGIN{printf "%.4f", a+(b-a)/2}')

if awk -v n="$NORTH" -v a="$X_MIN" -v b="$X_MAX" 'BEGIN{exit !(n < a || n > b)}'; then
    echo "WARNING: NORTH=$NORTH is outside the surveyed strip x=[$X_MIN,$X_MAX]." >&2
    echo "         The drones will never fly over these targets." >&2
fi

# A target sits at the centre of its band, so the worst case is half a band
# from the nearest lane. If the band is much wider than the camera footprint
# the survey still covers it (lanes are spaced by footprint*(1-sidelap)), but
# if the band is NARROWER than one lane spacing the drone flies a single pass
# and the centre is what it sees — worth knowing which case you are in.
echo "Swarm targets — $NUM_DRONES band(s) over east [$Y_MIN, $Y_MAX], band height ${BAND_H} m"
echo "Model: $MODEL_URI"
echo

EXISTING=$(gz model --list 2>/dev/null)
TRUTHS=()
FAILED=0
SKIPPED=0

for ((i=0; i<NUM_DRONES; i++)); do
    EAST=$(awk -v a="$Y_MIN" -v h="$BAND_H" -v i="$i" 'BEGIN{printf "%.4f", a+(i+0.5)*h}')
    NAME="${PREFIX}_${i}"
    TRUTHS+=("$NORTH,$EAST")

    # Gazebo world x is East, world y is North.
    GZ_X="$EAST"
    GZ_Y="$NORTH"

    # `gz model --list` prints "    - <name>" per model. -w keeps swarm_target_1
    # from matching swarm_target_10 (underscore and digits are word characters).
    if grep -qw -- "$NAME" <<<"$EXISTING"; then
        echo "  drone $i: '$NAME' already in the world — skipping (PX4 N=$NORTH E=$EAST)"
        SKIPPED=$((SKIPPED+1))
        continue
    fi

    printf "  drone %d: spawning '%s' at PX4 N=%s E=%s  (Gazebo x=%s y=%s) ... " \
        "$i" "$NAME" "$NORTH" "$EAST" "$GZ_X" "$GZ_Y"

    OUT=$(gz service -s "/world/$WORLD/create" \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 20000 \
        --req "sdf_filename: \"$MODEL_URI\", name: \"$NAME\", allow_renaming: false, pose: {position: {x: $GZ_X, y: $GZ_Y, z: 0}}" 2>&1)

    if [[ "$OUT" == *"data: true"* ]]; then
        echo "OK"
    else
        echo "FAILED"
        echo "      $OUT" | sed 's/^/      /'
        FAILED=$((FAILED+1))
    fi
    sleep 1
done

echo

if [ "$FAILED" -gt 0 ]; then
    echo "$FAILED spawn(s) did not report success." >&2
    echo "Most likely the Fuel model could not be downloaded (needs internet the" >&2
    echo "first time; cached in ~/.gz/fuel afterwards). If Fuel is unreachable," >&2
    echo "insert one person via the Gazebo GUI (top-right ... -> Resource Spawner)," >&2
    echo "which caches the model, then re-run this script." >&2
    exit 1
fi

echo "Verify they are really in the world:"
echo "  GZ_IP=127.0.0.1 gz model --list | grep -i '$PREFIX'"
echo
echo "Ground truth, in PX4 (North,East) — the same order hazard_map --truth wants:"
for ((i=0; i<NUM_DRONES; i++)); do
    echo "  drone $i band -> ${TRUTHS[$i]}"
done
echo
echo "hazard_points.csv APPENDS across runs. Move the old one aside first, or"
echo "you will score this flight against last flight's detections:"
echo "  mv ~/maps/hazard_points.csv ~/maps/hazard_points_\$(date +%s).csv"
echo
echo "Then fly, and render the map:"
echo "  ros2 launch survey swarm_mission.launch.py num_drones:=$NUM_DRONES \\"
echo "      x_min:=$X_MIN x_max:=$X_MAX y_min:=$Y_MIN y_max:=$Y_MAX altitude:=10.0"
echo "  ros2 run perception hazard_map --area $X_MIN,$X_MAX,$Y_MIN,$Y_MAX --truth ${TRUTHS[0]}"
echo
echo "NOTE: hazard_map --truth takes ONE point and --track plots ONE track file,"
echo "so with $NUM_DRONES drones it scores drone 0's band only and draws one"
echo "flight path. The other detections still appear as points. Making the map"
echo "multi-target and multi-track is a known open item (docs/NEXT_SESSION.md)."
