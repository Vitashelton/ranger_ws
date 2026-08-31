"""Accumulate and save a native 3D FAST-LIO map; starts no controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    pcd_path = LaunchConfiguration('pcd_path')
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path',
            default_value=os.path.expanduser(
                '~/.config/ranger_nav/maps/real_lab_3d.pcd')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='fast_lio',
            output='screen', parameters=[
                os.path.join(pkg, 'config', 'fastlio_mid360.yaml'),
                {
                    'use_sim_time': False,
                    # Keep map accumulation and map_save available, but do not
                    # stream the ever-growing /Laser_map to RViz during live
                    # experiments; it can stall RViz over the hotspot.
                    'publish.map_en': False,
                    'pcd_save.pcd_save_en': True,
                    'map_file_path': pcd_path,
                },
            ]),
        # Display-only leveling transform. It restores the previously verified
        # 30 degree MID360S mounting correction without claiming that this
        # session-local frame is the persistent global map.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='mapping_view_to_camera_init_tf', output='screen',
            arguments=[
                '--x', '0.30', '--y', '0.0', '--z', '0.70',
                '--roll', '0.0', '--pitch', '0.523599', '--yaw', '0.0',
                '--frame-id', 'mapping_view',
                '--child-frame-id', 'camera_init',
            ]),
        Node(
            package='rviz2', executable='rviz2', name='ranger_3d_mapping_rviz',
            arguments=['-d', os.path.join(
                pkg, 'config', 'lab_3d_mapping.rviz')],
            condition=IfCondition(use_rviz), output='screen'),
    ])
