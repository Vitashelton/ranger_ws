"""Bring up the existing Ranger LIO and PointCloud2 Nav2 demo stack.

This launch does not send a goal. Once a goal is submitted, Nav2 publishes
directly to /cmd_vel, so the chassis can move immediately when CAN mode is on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    launch_dir = os.path.join(pkg, 'launch')

    start_base = LaunchConfiguration('start_base')
    start_d435i = LaunchConfiguration('start_d435i')
    use_rviz = LaunchConfiguration('use_rviz')
    livox_config = LaunchConfiguration('livox_config')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    waypoint_file = LaunchConfiguration('waypoint_file')

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_d435i', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('nav2_autostart', default_value='true'),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='/home/robot/.config/ranger_nav/lab_waypoints.yaml',
        ),
        DeclareLaunchArgument(
            'livox_config',
            default_value=(
                '/home/robot/livox_ws/src/livox_ros_driver2/'
                'config/MID360s_config.json'
            ),
        ),
        include(os.path.join(launch_dir, 'ranger_lio_bringup.launch.py'), {
            'start_base': start_base,
            'start_d435i': start_d435i,
            'use_rviz': use_rviz,
            'livox_config': livox_config,
        }),
        include(os.path.join(launch_dir, 'ranger_nav_pointcloud.launch.py'), {
            'autostart': nav2_autostart,
        }),
        Node(
            package='ranger_nav',
            executable='lab_waypoint_demo.py',
            name='lab_waypoint_markers',
            arguments=['--file', waypoint_file, 'markers'],
            output='screen',
        ),
    ])
