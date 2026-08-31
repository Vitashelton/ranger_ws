#!/bin/bash
set -e

source /home/zbx/ros2_comm_pc.sh
source /home/zbx/ranger_ws/install/setup.bash

echo "默认全局导航已暂停：二维 /scan 重定位方案已撤销。"
echo "请先使用 ./lab_map.sh start 生成3D PCD，等待三维定位器接入。"
exit 1
