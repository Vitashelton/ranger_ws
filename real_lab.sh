#!/usr/bin/env bash
set -eo pipefail

WAYPOINT_FILE="${RANGER_REAL_LAB_FILE:-$HOME/.config/ranger_nav/real_lab.yaml}"
MAP_ID_FILE="${RANGER_LAB_MAP_ID_FILE:-$HOME/.config/ranger_nav/maps/real_lab.map_id}"

source "$HOME/ros2_comm_pc.sh"
source "$HOME/ranger_ws/install/setup.bash"

waypoint_tool() {
  ros2 run ranger_nav lab_waypoint_demo.py --file "$WAYPOINT_FILE" \
    --map-id-file "$MAP_ID_FILE" "$@"
}

usage() {
  echo "用法："
  echo "  ./real_lab.sh start       # 记录/覆盖真实起点"
  echo "  ./real_lab.sh reset       # 归档旧航点并绑定当前固定地图"
  echo "  ./real_lab.sh room1       # 记录/覆盖真实房间1门口"
  echo "  ./real_lab.sh room2       # 记录/覆盖真实房间2门口"
  echo "  ./real_lab.sh mark NAME   # 记录任意长期地图位姿"
  echo "  ./real_lab.sh make-route NAME WP... # 保存有序路线"
  echo "  ./real_lab.sh list        # 查看航点"
  echo "  ./real_lab.sh route       # 保存 room1 -> room2 路线"
  echo "  ./real_lab.sh markers     # 在RViz发布真实航点标记（保持运行）"
  echo "  ./real_lab.sh go room1    # 发送导航目标（CAN模式下会直接运动）"
  echo "  ./real_lab.sh go room2"
  echo "  ./real_lab.sh go to_room1  # 执行长期分阶段路线"
  echo "  ./real_lab.sh go to_room2"
  echo "  ./real_lab.sh go to_elevator"
  echo "  ./real_lab.sh go route      # 旧版 room1 -> room2 路线"
}

case "${1:-}" in
  reset)
    waypoint_tool reset
    ;;
  start)
    waypoint_tool record lab_start --timeout 10
    ;;
  room1)
    waypoint_tool record room1 --timeout 10
    ;;
  room2)
    waypoint_tool record room2 --timeout 10
    ;;
  mark)
    if [[ -z "${2:-}" ]]; then
      usage
      exit 2
    fi
    waypoint_tool record "$2" --timeout 10
    ;;
  make-route)
    if (( $# < 4 )); then
      usage
      exit 2
    fi
    route_name="$2"
    shift 2
    waypoint_tool route "$route_name" "$@"
    ;;
  list)
    waypoint_tool list
    ;;
  route)
    waypoint_tool route two_rooms room1 room2
    ;;
  markers)
    waypoint_tool markers
    ;;
  go)
    case "${2:-}" in
      room1|room2)
        waypoint_tool go "$2"
        ;;
      to_room1|to_room2|to_elevator)
        waypoint_tool go "$2"
        ;;
      route)
        waypoint_tool go two_rooms
        ;;
      *)
        usage
        exit 2
        ;;
    esac
    ;;
  *)
    usage
    exit 2
    ;;
esac
