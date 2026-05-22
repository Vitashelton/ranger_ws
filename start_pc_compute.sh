#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source ~/ranger_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch ranger_nav ubuntu_compute.launch.py
