"""Nav2 servers with rolling PointCloud2 voxel costmaps and no 2D map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    params = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[params],
             remappings=[('/cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params],
             remappings=[('/cmd_vel', '/cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[params]),
        Node(
            package='ranger_nav', executable='cmd_vel_safety_gate.py',
            name='cmd_vel_safety_gate', output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': autostart,
                'node_names': [
                    'controller_server', 'planner_server', 'behavior_server',
                    'bt_navigator', 'waypoint_follower'],
                'bond_timeout': 10.0,
            }],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_pointcloud_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        *nodes,
    ])
