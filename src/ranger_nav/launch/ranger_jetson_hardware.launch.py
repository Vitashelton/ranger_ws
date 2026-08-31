"""Jetson-only hardware bringup: chassis, MID-360S and optional D435i.

FAST-LIO, Nav2, the task layer and RViz intentionally run on the PC.
This launch never starts a planner and never sends a velocity command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(path, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    launch_dir = os.path.join(pkg, 'launch')
    start_base = LaunchConfiguration('start_base')
    start_d435i = LaunchConfiguration('start_d435i')
    livox_config = LaunchConfiguration('livox_config')

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_d435i', default_value='true'),
        DeclareLaunchArgument(
            'livox_config',
            default_value=(
                '/home/robot/livox_ws/src/livox_ros_driver2/'
                'config/MID360s_config.json'),
        ),
        include(
            os.path.join(launch_dir, 'ranger_base.launch.py'),
            {'port_name': 'can0', 'publish_odom_tf': 'true'},
            condition=IfCondition(start_base),
        ),
        include(
            os.path.join(launch_dir, 'ranger_sensors.launch.py'),
            {
                'livox_config': livox_config,
                'xfer_format': '1',
                'start_d435i': start_d435i,
            },
        ),
    ])
