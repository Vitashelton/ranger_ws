"""PC-only prior-map localization bringup.

The Jetson must already be running the hardware launch.  This launch starts
FAST-LIO, the prior 3D map localizer and optional RViz; it deliberately does
not start Nav2 or any controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    use_rviz = LaunchConfiguration('use_rviz')
    prior_map = LaunchConfiguration('prior_map')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'prior_map',
            default_value='/home/zbx/.config/ranger_nav/maps/real_lab_3d_level_full.pcd'),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fast_lio',
            output='screen', parameters=[
                os.path.join(pkg, 'config', 'fastlio_mid360.yaml'),
                {'use_sim_time': False, 'publish.map_en': False},
            ]),
        Node(
            package='ranger_nav', executable='cloud_frame_relay.py',
            name='cloud_frame_relay', output='screen'),
        Node(
            package='ranger_nav', executable='prior_map_localizer',
            name='prior_map_localizer', output='screen',
            parameters=[{
                'map_path': prior_map,
                'cloud_topic': '/cloud_registered',
                'lio_topic': '/Odometry',
                'wheel_topic': '/odom',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'base_to_body_x': 0.30,
                'base_to_body_y': 0.0,
                'base_to_body_z': 0.70,
                'base_to_body_pitch': 0.523599,
                'initial_x': 0.30,
                'initial_y': 0.0,
                'initial_z': 0.70,
                'initial_pitch': 0.523599,
                'map_voxel': 0.15,
                'scan_voxel': 0.15,
                'ndt_resolution': 0.80,
                'ndt_iterations': 20,
                'process_every_n': 10,
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'pc_monitor.launch.py')),
            condition=IfCondition(use_rviz),
        ),
    ])
