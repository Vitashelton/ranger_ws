"""Bounded doorway-perception experiment; no mapping, navigation, or control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('ranger_nav')
    config = os.path.join(package, 'config', 'doorway_perception.yaml')
    rviz_config = os.path.join(package, 'config', 'doorway_monitor.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('door_distance', default_value='2.0'),
        DeclareLaunchArgument('ground_truth', default_value='UNSET'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='ranger_nav',
            executable='doorway_traversability.py',
            name='doorway_traversability',
            output='screen',
            parameters=[config, {
                'door_distance': LaunchConfiguration('door_distance'),
                'ground_truth': LaunchConfiguration('ground_truth'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='doorway_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
