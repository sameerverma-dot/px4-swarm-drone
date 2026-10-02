#!/usr/bin/env bash
#
# make_working_doc.sh - export every file a Claude project needs to understand
# this workspace - the ones that change every session plus everything they
# reference (included launch files, linked docs, helper scripts) - into
# ~/px4_ros_ws/working_doc/ as REAL copies, ready to drag in in one go, plus a CHANGED.txt saying which ones are new since the
# last export (so you only re-upload those).
#
#   bash ~/px4_ros_ws/tools/make_working_doc.sh           # refresh now
#
# The first run also installs a git post-commit hook, so after that the folder
# refreshes itself every time you `git commit` - which is how every session
# ends. You should not normally need to run this by hand again.
#
# The originals never move: colcon builds from src/, and the docs link to each
# other by path. working_doc/ is an EXPORT, regenerated from the originals, and
# is git-ignored so the copies never get committed twice. Never edit a file in
# working_doc/ - the next export overwrites it. Edit the original.

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR="$WS/working_doc"
MANIFEST="$DIR/.manifest"
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

FILES=(
    # status, plan and how-to docs
    docs/PROGRESS.md
    docs/NEXT_SESSION.md
    docs/CHEATSHEET.md
    docs/STACK_README.md
    docs/SYSTEM_GUIDE.md
    docs/SWARM_PLAN.md
    docs/PHASE1_ROADMAP.md
    docs/CAMERA_DIAGNOSTIC.md
    README.md
    # flight + perception code
    src/survey/survey/survey_node.py
    src/survey/survey/swarm_logic.py
    src/perception/perception/detector_node.py
    src/perception/perception/hazard_map.py
    src/perception/perception/hazard_registry.py
    src/perception/perception/ground_station.py
    # swarm messages (drones talk to each other through these)
    src/swarm_msgs/msg/DroneHeartbeat.msg
    src/swarm_msgs/msg/HazardReport.msg
    # unit tests for the swarm rules
    src/survey/test/test_swarm_logic.py
    src/perception/test/test_hazard_registry.py
    src/perception/test_perception.py
    # launch files - all four: mission and swarm_mission INCLUDE survey and
    # perception, so exporting only the outer two leaves them unreadable
    src/survey/launch/swarm_mission.launch.py
    src/survey/launch/mission.launch.py
    src/survey/launch/survey.launch.py
    src/survey/launch/onboard.launch.py
    src/perception/launch/perception.launch.py
    # launchers and tools
    start_px4_sim.sh
    tools/start_px4_swarm.sh
    tools/check_system.sh
    tools/add_swarm_targets.sh
    tools/add_target.sh
    tools/diagnose_camera.sh
    tools/stop_sim.sh
)

mkdir -p "$DIR"

# checksums from the previous export, to work out what changed
declare -A OLD=()
if [ -f "$MANIFEST" ]; then
    while read -r sum name; do OLD["$name"]="$sum"; done < "$MANIFEST"
fi
FIRST=0; [ -f "$MANIFEST" ] || FIRST=1

declare -A WANT=()
changed=()
: > "$MANIFEST.new"
for f in "${FILES[@]}"; do
    src="$WS/$f"
    if [ ! -f "$src" ]; then
        [ "$QUIET" -eq 1 ] || echo "  SKIP (not found): $f"
        continue
    fi
    name="$(basename "$f")"
    WANT["$name"]=1
    # --remove-destination also replaces the symlinks an older version of this
    # script left here, instead of failing with "same file".
    cp --remove-destination "$src" "$DIR/$name"
    sum="$(sha1sum "$src" | cut -d' ' -f1)"
    echo "$sum $name" >> "$MANIFEST.new"
    if [ -z "${OLD[$name]:-}" ]; then
        changed+=("$name  (new)")
    elif [ "${OLD[$name]}" != "$sum" ]; then
        changed+=("$name")
    fi
done
mv "$MANIFEST.new" "$MANIFEST"

# drop anything in working_doc that is no longer on the list
for p in "$DIR"/*; do
    [ -e "$p" ] || [ -L "$p" ] || continue
    n="$(basename "$p")"
    [ "$n" = "CHANGED.txt" ] && continue
    [ -n "${WANT[$n]:-}" ] || rm -f "$p"
done

{
    echo "working_doc export: $(date '+%d %b %Y %H:%M')"
    if [ "$FIRST" -eq 1 ]; then
        echo "First export - upload everything in this folder."
    elif [ "${#changed[@]}" -eq 0 ]; then
        echo "Nothing changed since the last export - the project is up to date."
    else
        echo "Changed since the last export - re-upload these (delete the old copy in the project first):"
    fi
    for n in "${changed[@]}"; do echo "  $n"; done
} > "$DIR/CHANGED.txt"

# one-time: refresh automatically on every commit
HOOK="$WS/.git/hooks/post-commit"
if [ -d "$WS/.git/hooks" ] && ! grep -q make_working_doc "$HOOK" 2>/dev/null; then
    [ -f "$HOOK" ] || echo '#!/bin/sh' > "$HOOK"
    echo 'bash "$(git rev-parse --show-toplevel)/tools/make_working_doc.sh" --quiet' >> "$HOOK"
    chmod +x "$HOOK"
    [ "$QUIET" -eq 1 ] || echo "  installed git post-commit hook: working_doc now refreshes on every commit"
fi

# keep the copies out of git
GI="$WS/.gitignore"
if ! grep -qx 'working_doc/' "$GI" 2>/dev/null; then
    echo 'working_doc/' >> "$GI"
fi

if [ "$QUIET" -eq 1 ]; then
    echo "working_doc refreshed: ${#changed[@]} file(s) changed -> $DIR/CHANGED.txt"
else
    echo
    cat "$DIR/CHANGED.txt"
    echo
    echo "Folder: $DIR"
fi
