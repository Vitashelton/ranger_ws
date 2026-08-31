#!/usr/bin/env bash
set -eo pipefail

workspace="/home/zbx/ranger_ws"
data_root="${RANGER_DOORWAY_DATA:-/data/ranger_nav/research/doorway_bags}"

source /opt/ros/humble/setup.bash
source /home/zbx/ros2_comm_pc.sh
source "$workspace/install/setup.bash"

usage() {
  echo "用法："
  echo "  ./doorway_research.sh check"
  echo "  ./doorway_research.sh run <PASSABLE|BLOCKED|UNKNOWN> <门距离米>"
  echo "  ./doorway_research.sh label <PASSABLE|BLOCKED|UNKNOWN>"
  echo "  ./doorway_research.sh record <PASSABLE|BLOCKED|UNKNOWN> <门距离米> <重复号>"
  echo "  ./doorway_research.sh watch"
  echo "  ./doorway_research.sh eval [doorway_bags目录]"
}

validate_label() {
  case "$1" in
    PASSABLE|BLOCKED|UNKNOWN) ;;
    *) echo "标签必须是 PASSABLE、BLOCKED 或 UNKNOWN" >&2; exit 2 ;;
  esac
}

check_topics() {
  local missing=0
  for topic in /livox/lidar /camera/color/image_raw /camera/aligned_depth_to_color/image_raw; do
    local count
    count="$(set +o pipefail; ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}' || true)"
    local echo_args=("$topic" --once)
    if [[ "$topic" == /camera/* ]]; then
      # RealSense image publishers commonly offer sensor-data/best-effort QoS.
      echo_args+=(--qos-reliability best_effort --qos-durability volatile)
    fi
    if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )) && \
        timeout 5 ros2 topic echo "${echo_args[@]}" >/dev/null 2>&1; then
      printf '  [LIVE] %s\n' "$topic"
    else
      printf '  [--] %s (5秒内没有收到消息；publisher=%s)\n' "$topic" "${count:-0}"
      missing=$((missing + 1))
    fi
  done
  echo "缺少 $missing/3 个门洞感知输入；不需要 FAST-LIO 或 Nav2。"
}

command="${1:-}"
case "$command" in
  check)
    check_topics
    ;;
  run)
    label="${2:-UNSET}"
    distance="${3:-2.0}"
    validate_label "$label"
    exec ros2 launch ranger_nav doorway_perception.launch.py \
      ground_truth:="$label" door_distance:="$distance"
    ;;
  label)
    label="${2:-}"
    validate_label "$label"
    exec ros2 topic pub --once /doorway/ground_truth std_msgs/msg/String \
      "{data: '$label'}"
    ;;
  watch)
    exec ros2 topic echo /doorway/evidence std_msgs/msg/String --field data
    ;;
  eval)
    input="${2:-$data_root}"
    exec ros2 run ranger_nav doorway_eval.py "$input" \
      --output "$data_root/doorway_metrics.csv"
    ;;
  record)
    label="${2:-}"
    distance="${3:-}"
    repeat="${4:-}"
    validate_label "$label"
    if [[ ! "$distance" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ ! "$repeat" =~ ^[0-9]+$ ]]; then
      usage
      exit 2
    fi
    stamp="$(date +%Y%m%d_%H%M%S)"
    session="$data_root/${stamp}_${label}_d${distance}_r${repeat}"
    mkdir -p "$session"
    {
      printf 'schema_version=1\n'
      printf 'ground_truth=%s\n' "$label"
      printf 'door_distance_m=%s\n' "$distance"
      printf 'repeat=%s\n' "$repeat"
      printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
      printf 'hardware=RangerMini_2.0,MID-360S,D435i,Jetson_Orin_Nano\n'
    } > "$session/session.env"
    echo "开始门洞实验录制：$session/bag"
    echo "本次真值=$label，门距离=${distance}m；完成后 Ctrl-C。"
    ros2 topic pub --once /doorway/ground_truth std_msgs/msg/String \
      "{data: '$label'}" >/dev/null
    set +e
    ros2 bag record --compression-mode file --compression-format zstd \
      -o "$session/bag" \
      /livox/lidar /livox/imu \
      /camera/color/image_raw /camera/color/camera_info \
      /camera/aligned_depth_to_color/image_raw \
      /camera/aligned_depth_to_color/camera_info \
      /doorway/evidence /doorway/ground_truth /tf_static
    status=$?
    set -e
    printf 'finished_at=%s\n' "$(date --iso-8601=seconds)" >> "$session/session.env"
    [[ -f "$session/bag/metadata.yaml" ]] && ros2 bag info "$session/bag" > "$session/bag_info.txt"
    echo "数据保存在：$session"
    exit "$status"
    ;;
  *)
    usage
    exit 2
    ;;
esac
