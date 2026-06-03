#!/usr/bin/env bash
# Run this ON THE REAL ROBOT (Jetson) inside ~/ranger_ws.
# It only INSPECTS the workspace; it changes nothing.
set -u
echo "================ TCA-BEV environment check ================"
echo "ROS_DISTRO = ${ROS_DISTRO:-<not sourced>}"
command -v ros2 >/dev/null && echo "ros2: $(ros2 --version 2>/dev/null || echo found)" || echo "ros2: NOT FOUND (source /opt/ros/<distro>/setup.bash)"
echo
WS="${1:-$HOME/ranger_ws}"
echo "Workspace under test: $WS"
[ -d "$WS/src" ] && echo "[OK] $WS/src exists -> looks like a ROS2 workspace" || echo "[WARN] no $WS/src — is this a workspace?"
echo
echo "---- packages found under src/ ----"
if [ -d "$WS/src" ]; then
  find "$WS/src" -maxdepth 3 -name package.xml -printf '%h\n' | sed "s#$WS/src/##" | sort -u
fi
echo
echo "---- OFFICIAL packages (DO NOT MODIFY) ----"
find "$WS/src" -maxdepth 3 -name package.xml 2>/dev/null | while read -r p; do
  name=$(grep -oPm1 '(?<=<name>)[^<]+' "$p")
  case "$name" in
    ranger*|*ranger*|ugv_sdk*|*scout*|limo*|livox_ros_driver*|realsense*)
      echo "  [PROTECTED] $name  ($(dirname "$p"))" ;;
  esac
done
echo
echo "---- live topics (need drivers running) ----"
ros2 topic list 2>/dev/null | sort || echo "(no ros2 / no topics)"
echo
echo "---- checking expected topics ----"
for t in /livox/lidar /livox/imu /camera/color/image_raw \
         /camera/depth/image_rect_raw /cmd_vel /odom /motion_state \
         /actuator_state /system_state /battery_state /tf /tf_static; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then echo "  [present] $t"; else echo "  [absent ] $t"; fi
done
echo "==========================================================="
