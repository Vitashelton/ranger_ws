#!/usr/bin/python3
"""
Launch FAST-LIVO2 (LiDAR-Inertial-Visual Odometry) for Ranger Mini 2.0.

Hardware:
  - Livox MID360S LiDAR  (topic: /livox/lidar, IMU: /livox/imu)
  - Intel RealSense D435i (topic: /camera/color/image_raw)

Prerequisites:
  Sensors must already be running (via ranger_sensors.launch.py or equivalent).
  This launch file only starts the FAST-LIVO2 mapping node.

Usage:
  # Standalone (sensors already running):
  ros2 launch fast_livo ranger_fast_livo2.launch.py

  # With RViz:
  ros2 launch fast_livo ranger_fast_livo2.launch.py use_rviz:=true

  # With custom configs:
  ros2 launch fast_livo ranger_fast_livo2.launch.py \
      lio_params_file:=/path/to/mid360_d435i.yaml \
      camera_params_file:=/path/to/camera_d435i.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fast_livo')

    # ---- Paths ----
    lio_config_default = os.path.join(pkg_dir, 'config', 'mid360_d435i.yaml')
    camera_config_default = os.path.join(pkg_dir, 'config', 'camera_d435i.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz_cfg', 'fast_livo2.rviz')

    # ---- Arguments ----
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Launch RViz2',
    )
    lio_config_arg = DeclareLaunchArgument(
        'lio_params_file',
        default_value=lio_config_default,
        description='Path to LiDAR-IMU config YAML',
    )
    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_default,
        description='Path to camera config YAML',
    )
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Respawn if node crashes',
    )

    # ---- Nodes ----
    fast_livo_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            LaunchConfiguration('lio_params_file'),
            LaunchConfiguration('camera_params_file'),
        ],
        arguments=['--ros-args', '--log-level', 'WARN'],
        respawn=LaunchConfiguration('use_respawn'),
    )


    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        use_rviz_arg,
        lio_config_arg,
        camera_config_arg,
        use_respawn_arg,
        fast_livo_node,
        rviz_node,
    ])
