"""
SLAM mapping launch for Ranger Mini 2.0.

Starts: chassis driver + sensors + static TF + slam_toolbox (online async mapping).

Usage:
  ros2 launch ranger_nav ranger_slam.launch.py
  ros2 launch ranger_nav ranger_slam.launch.py slam_params_file:=/path/to/custom.yaml
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
    pkg_dir = get_package_share_directory('ranger_nav')

    # --- Config paths ---
    default_slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox_mapping.yaml')

    slam_params_file = LaunchConfiguration('slam_params_file', default=default_slam_config)

    # --- Include sub-launches ---
    ranger_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_base.launch.py')
        ),
    )

    ranger_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_sensors.launch.py')
        ),
    )

    # --- slam_toolbox (online async) ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # --- RViz2 ---
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ranger_nav.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz', default='true')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_config),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        ranger_base_launch,
        ranger_sensors_launch,
        slam_toolbox_node,
        rviz2_node,
    ])
