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
    nav_to_pose_bt = os.path.join(
        pkg, 'behavior_trees', 'navigate_to_pose_no_recovery.xml')
    nav_through_poses_bt = os.path.join(
        pkg, 'behavior_trees', 'navigate_through_poses_no_recovery.xml')
    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[
                 params,
                 {
                     'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                     'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                 },
             ]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[params]),
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
        # Do not activate costmaps before the map->odom->base_link chain exists.
        DeclareLaunchArgument('autostart', default_value='false'),
        *nodes,
    ])
