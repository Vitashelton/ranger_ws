#!/usr/bin/env bash
# Record a TCA-BEV data bag. Usage:
#   ./record_rosbag.sh <scene> <run_id>
# Example:
#   ./record_rosbag.sh corridor 001   -> tcabev_corridor_001_YYYYmmdd_HHMMSS
set -euo pipefail
SCENE="${1:?scene required: corridor|doorway|pillar|pedestrian|box|mixed}"
RUN="${2:?run id required, e.g. 001}"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="tcabev_${SCENE}_${RUN}_${STAMP}"

# Topics to record. We record raw sensor + chassis feedback + tf, plus the
# safe command channel so teacher labels are captured. We DO NOT need /scan.
TOPICS=(
  /livox/lidar /livox/imu
  /camera/color/image_raw /camera/color/camera_info
  /camera/depth/image_rect_raw /camera/depth/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
  /odom /motion_state /actuator_state /system_state /battery_state
  /tf /tf_static
  /teacher/cmd_vel_raw /cmd_vel_safe /cmd_vel
  /time_align/status /calibration/status /bev/status
)

echo "Recording -> $OUT"
echo "Topics: ${TOPICS[*]}"
ros2 bag record -o "$OUT" "${TOPICS[@]}"
