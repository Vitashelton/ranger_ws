#!/bin/bash
# Source all required ROS2 workspaces for Ranger navigation
# Usage: source setup_ranger_nav.sh

echo "[ranger_nav] Setting up ROS2 Humble workspace overlay..."

# Base ROS2
source /opt/ros/humble/setup.bash

# Livox driver workspace (sensors)
if [ -f /home/robot/livox_ws/install/setup.bash ]; then
    source /home/robot/livox_ws/install/setup.bash
    echo "[ranger_nav]   -> livox_ws loaded"
else
    echo "[ranger_nav]   WARNING: livox_ws/install/setup.bash not found"
fi

# AgileX chassis workspace
if [ -f /home/robot/agilex_ws/install/setup.bash ]; then
    source /home/robot/agilex_ws/install/setup.bash
    echo "[ranger_nav]   -> agilex_ws loaded"
else
    echo "[ranger_nav]   WARNING: agilex_ws/install/setup.bash not found"
fi

# Ranger nav workspace (this package)
if [ -f /home/robot/ranger_nav_ws/install/setup.bash ]; then
    source /home/robot/ranger_nav_ws/install/setup.bash
    echo "[ranger_nav]   -> ranger_nav_ws loaded"
fi

echo "[ranger_nav] Ready."
echo "  SLAM:  ros2 launch ranger_nav ranger_slam.launch.py"
echo "  Nav:   ros2 launch ranger_nav ranger_nav.launch.py map:=/home/robot/maps/ranger_map.yaml"
echo "  Full:  ros2 launch ranger_nav ranger_full.launch.py mode:=mapping"
