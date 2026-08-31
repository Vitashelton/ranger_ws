#!/usr/bin/env bash
set -eo pipefail

workspace="/home/zbx/ranger_ws"
data_root="${RANGER_RESEARCH_DATA:-/data/ranger_nav/research/bags}"

source /opt/ros/humble/setup.bash
source /home/zbx/ros2_comm_pc.sh
source "$workspace/install/setup.bash"
set -u

required_topics=(
  /livox/lidar
  /livox/imu
  /odom
  /motion_state
  /Odometry
  /lio/base_odom
  /nav/points
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
)

record_topics=(
  /livox/lidar
  /livox/imu
  /odom
  /motion_state
  /Odometry
  /lio/base_odom
  /cloud_registered
  /cloud_registered_body
  /nav/points
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
  /cmd_vel
  /cmd_vel_nav
  /plan
  /local_plan
  /tf
  /tf_static
)

usage() {
  echo "用法："
  echo "  ./real_research.sh check"
  echo "  ./real_research.sh timing"
  echo "  ./real_research.sh record <标签>"
  echo "  ./real_research.sh info <bag目录>"
}

check_topics() {
  local missing=0
  echo "实机研究链路检查："
  for topic in "${required_topics[@]}"; do
    local info publishers
    info="$(ros2 topic info "$topic" 2>/dev/null || true)"
    publishers="$(awk '/Publisher count:/ {print $3; exit}' <<<"$info")"
    if [[ "${publishers:-0}" =~ ^[0-9]+$ ]] && (( publishers > 0 )); then
      printf '  [OK] %s (%s publisher)\n' "$topic" "$publishers"
    else
      printf '  [--] %s (no publisher)\n' "$topic"
      missing=$((missing + 1))
    fi
  done
  echo
  echo "缺少 $missing/${#required_topics[@]} 个主题。"
  echo "录包前应启动Jetson硬件和PC FAST-LIO；Nav2可以暂时不启动。"
  return 0
}

record_session() {
  local label="${1:-}"
  if [[ -z "$label" ]]; then
    echo "record需要标签，例如 baseline_corridor_outbound" >&2
    exit 2
  fi
  if [[ ! "$label" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "标签只能包含字母、数字、点、下划线和横线" >&2
    exit 2
  fi
  mkdir -p "$data_root"
  local stamp session
  stamp="$(date +%Y%m%d_%H%M%S)"
  session="$data_root/${stamp}_${label}"
  mkdir -p "$session"
  ros2 topic list -t > "$session/topics_start.txt"
  {
    printf 'schema_version=1\n'
    printf 'label=%s\n' "$label"
    printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
    printf 'host=%s\n' "$(hostname)"
    printf 'hardware=RangerMini_2.0,MID-360S,D435i,Jetson_Orin_Nano\n'
    printf 'ros_domain_id=%s\n' "${ROS_DOMAIN_ID:-}"
    printf 'git_commit=%s\n' "$(git -C "$workspace" rev-parse --verify HEAD 2>/dev/null || printf unknown)"
  } > "$session/session.env"
  echo "开始录制：$session/bag"
  echo "完成路线后按 Ctrl-C；不要用此bag直接向实车回放/cmd_vel。"
  set +e
  ros2 bag record \
    --include-unpublished-topics \
    --compression-mode file \
    --compression-format zstd \
    --max-cache-size 1073741824 \
    -o "$session/bag" \
    "${record_topics[@]}"
  local status=$?
  set -e
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >> "$session/session.env"
  if [[ -f "$session/bag/metadata.yaml" ]]; then
    ros2 bag info "$session/bag" > "$session/bag_info.txt"
  fi
  echo "数据保存在：$session"
  return "$status"
}

command="${1:-}"
case "$command" in
  check)
    check_topics
    ;;
  timing)
    exec ros2 run ranger_nav_metrics timing_monitor
    ;;
  record)
    record_session "${2:-}"
    ;;
  info)
    if [[ -z "${2:-}" ]]; then
      usage
      exit 2
    fi
    exec ros2 bag info "$2"
    ;;
  *)
    usage
    exit 2
    ;;
esac
