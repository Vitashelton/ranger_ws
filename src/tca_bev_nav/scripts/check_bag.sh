#!/usr/bin/env bash
# Inspect a recorded bag and flag missing/low-rate topics.
set -euo pipefail
BAG="${1:?path to bag dir required}"
echo "==== ros2 bag info ===="
ros2 bag info "$BAG"
echo
echo "Sanity checklist (verify manually against output above):"
echo "  [ ] /livox/lidar present and ~10 Hz"
echo "  [ ] /camera/depth/image_rect_raw present"
echo "  [ ] /odom present and continuous"
echo "  [ ] /tf and /tf_static present"
echo "  [ ] /teacher/cmd_vel_raw present (if teleop run)"
echo "  [ ] /cmd_vel_safe present (safety supervisor was running)"
echo "  [ ] duration matches the intended scene length"
