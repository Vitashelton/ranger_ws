#!/usr/bin/env bash
set -eo pipefail

MAP_DIR="${RANGER_LAB_MAP_DIR:-$HOME/.config/ranger_nav/maps}"
MAP_NAME="${RANGER_LAB_MAP_NAME:-real_lab}"
PCD_PATH="$MAP_DIR/${MAP_NAME}_3d.pcd"

source "$HOME/ros2_comm_pc.sh"
source "$HOME/ranger_ws/install/setup.bash"

case "${1:-}" in
  start)
    mkdir -p "$MAP_DIR"
    exec ros2 launch ranger_nav ranger_lab_3d_mapping.launch.py \
      pcd_path:="$PCD_PATH"
    ;;
  save)
    ros2 service call /map_save std_srvs/srv/Trigger "{}"
    stamp="$(date +%Y%m%d-%H%M%S)"
    snapshot="$MAP_DIR/${MAP_NAME}_3d_${stamp}.pcd"
    leveled="$MAP_DIR/${MAP_NAME}_3d_level_${stamp}.pcd"
    leveled_latest="$MAP_DIR/${MAP_NAME}_3d_level.pcd"
    cp --reflink=auto "$PCD_PATH" "$snapshot"
    echo "已保存时间戳快照：$snapshot"
    ros2 run ranger_nav level_fastlio_pcd.py "$snapshot" "$leveled"
    cp --reflink=auto "$leveled" "$leveled_latest"
    sha256sum "$leveled" | awk '{print $1}' > "$MAP_DIR/${MAP_NAME}.map_id"
    echo "三维定位地图：$leveled"
    echo "当前地图副本：$leveled_latest"
    ;;
  check)
    if [[ -s "$PCD_PATH" ]]; then
      ls -lh "$PCD_PATH"
      find "$MAP_DIR" -maxdepth 1 -type f \
        -name "${MAP_NAME}_3d_????????-??????.pcd" -printf '%TY-%Tm-%Td %TH:%TM %9s %p\n' \
        | sort
      find "$MAP_DIR" -maxdepth 1 -type f \
        -name "${MAP_NAME}_3d_level_????????-??????.pcd" -printf '%TY-%Tm-%Td %TH:%TM %9s %p\n' \
        | sort
    else
      echo "缺少三维地图：$PCD_PATH"
      exit 1
    fi
    ;;
  *)
    echo "用法：./lab_map.sh {start|save|check}"
    echo "这里只生成完整3D PCD；三维重定位接入完成前不会启动全局导航。"
    exit 2
    ;;
esac
