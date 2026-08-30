#!/usr/bin/env bash
# diagnose_camera.sh — one-shot diagnostic for
# "Gazebo camera topic is advertised but publishes no frames"
#
# READ-ONLY: this gathers facts and changes nothing.
#
# Best run in TWO stages:
#   1) with the sim NOT running   -> sections 1-3, 7 (static config)
#   2) with the sim RUNNING       -> sections 4-6 (live state)
# Just run it twice and paste both outputs.

echo "############################################################"
echo "# Gazebo camera diagnostic — $(date)"
echo "############################################################"

echo
echo "=== 1. Session & graphics stack ==============================="
echo "XDG_SESSION_TYPE : ${XDG_SESSION_TYPE:-<unset>}   <-- 'wayland' is a known EGL breaker"
echo "Distro           : $(lsb_release -ds 2>/dev/null || echo '?')"
echo "Kernel           : $(uname -r)"
echo "DISPLAY          : ${DISPLAY:-<unset>}   WAYLAND_DISPLAY: ${WAYLAND_DISPLAY:-<unset>}"
echo
echo "--- NVIDIA ---"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1 | sed 's/^/  /'
echo "  prime-select: $(prime-select query 2>/dev/null || echo '<not installed>')"
echo
echo "--- DRI render nodes (need card* and renderD*) ---"
ls -l /dev/dri/ 2>&1 | sed 's/^/  /'
echo
echo "--- EGL vendor ICDs ---"
ls /usr/share/glvnd/egl_vendor.d/ 2>&1 | sed 's/^/  /'

echo
echo "=== 2. Gazebo version & installed render engines =============="
gz sim --version 2>&1 | sed 's/^/  /'
echo "--- gz-rendering engine plugins present ---"
ls /usr/lib/x86_64-linux-gnu/gz-rendering-*/engine-plugins/ 2>&1 | sed 's/^/  /'
echo "  (want to see BOTH libgz-rendering-ogre.so and libgz-rendering-ogre2.so)"

echo
echo "=== 3. Server plugin config (who loads the Sensors system) ====="
echo "NOTE: PX4's world file has NO <plugin> tags, so gz-sim falls back to"
echo "      server.config. The Sensors plugin there picks the render engine."
for f in "$HOME/.gz/sim/8/server.config" /usr/share/gz/gz-sim8/server.config; do
  echo "--- $f ---"
  if [ -f "$f" ]; then
    grep -nE "sensors-system|render_engine|physics-system|imu-system|navsat|magnetometer|air-pressure|scene-broadcaster|user-commands" "$f" | sed 's/^/  /'
  else
    echo "  <missing>"
  fi
done
echo
echo "--- plugin count in PX4 world file (0 = uses server.config above) ---"
echo -n "  default.sdf <plugin> count: "
grep -c "<plugin" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf" 2>/dev/null || echo "?"

echo
echo "=== 4. Is the sim running? which process renders? ============="
echo "--- gz processes ---"
pgrep -af "gz sim" 2>/dev/null | sed 's/^/  /' || echo "  <gz sim NOT running>"
echo "--- px4 process ---"
pgrep -af "bin/px4" 2>/dev/null | sed 's/^/  /' || echo "  <px4 NOT running>"
echo
echo "--- GPU process table (is the SERVER on the GPU, or only 'gz sim -g'?) ---"
nvidia-smi 2>/dev/null | sed -n '/Processes/,$p' | sed 's/^/  /'

echo
echo "=== 5. Camera topic: advertised vs actually publishing ========"
CAM=$(gz topic -l 2>/dev/null | grep -m1 "camera/image")
echo "  topic: ${CAM:-<none found — is the sim running with a camera model?>}"
if [ -n "$CAM" ]; then
  echo "  --- publisher info ---"
  gz topic -i -t "$CAM" 2>&1 | sed 's/^/    /'
  echo "  --- sampling 5 s for actual messages ---"
  timeout 5 gz topic -e -t "$CAM" 2>&1 | head -c 400 | sed 's/^/    /'
  echo
  echo "  >>> EMPTY above = topic advertised but NO frames rendered (the bug)."
fi

echo
echo "=== 6. ROS 2 side (only meaningful if frames exist) ==========="
if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list 2>/dev/null | grep -iE "camera|image" | sed 's/^/  /' || echo "  <no image topics>"
else
  echo "  ros2 not sourced in this shell (fine — check section 5 first)"
fi

echo
echo "=== 7. ISOLATION TEST (run this manually, sim stopped) ========"
cat <<'EOF'
  This proves whether the fault is PX4-specific or system-wide.
  Stop everything first, then run Gazebo's own camera demo VERBOSE:

      gz sim -v 4 -r sensors_demo.sdf
      # if that world doesn't exist, try:  gz sim -v 4 -r camera_sensor.sdf

  In a second terminal:
      gz topic -l | grep -i image
      gz topic -e -t <that topic> | head -20

  INTERPRETATION:
    * Demo camera DOES publish  -> your graphics stack is fine; the fault is
                                   in how PX4 launches the server. Fix #2/#3.
    * Demo camera does NOT publish -> system-wide gz sensor-rendering failure,
                                   independent of PX4. Fix #1 (ogre) or #4.
  The -v 4 output prints the REAL render-engine error that PX4's launcher hides.
  Copy those lines — they name the exact failure.
EOF

echo
echo "############ end of diagnostic ############"
