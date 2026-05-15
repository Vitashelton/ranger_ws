"""
3D SLAM launch for Ranger Mini 2.0 — FAST-LIO2 LiDAR-inertial odometry + mapping.

Hardware requirements:
  - Livox MID360S LiDAR (built-in IMU used for odometry)
  - Ranger Mini 2.0 base (wheel odometry for map->odom residual)
  - livox_ros_driver2 + Ericsii/FAST_LIO_ROS2 installed

Usage:
  ros2 launch ranger_nav ranger_3d_slam.launch.py

Topics published:
  /Odometry             — 6-DOF LiDAR-inertial odometry (200 Hz)
  /cloud_registered     — registered scan in world frame
  /cloud_registered_body — registered scan in IMU body frame
  /map                  — accumulated map pointcloud
  /path                 — odometry trajectory
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # --- Chassis: Ranger Mini 2.0 base driver ---
    ranger_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_base.launch.py')
        ),
    )

    # --- Sensors: Livox MID360S + static TF base_link->livox_frame ---
    ranger_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_sensors.launch.py')
        ),
    )

    # --- FAST-LIO2: LiDAR-IMU odometry + mapping ---
    fast_lio_config = os.path.join(pkg_dir, 'config', 'fastlio_mid360.yaml')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fast_lio_config],
        remappings=[
            # FAST-LIO2 publishes /Odometry; remap to match nav2 convention
            ('/Odometry', '/odom_lidar'),
        ],
    )

    # --- map -> odom identity TF (FAST-LIO odom is the world frame) ---
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # --- 3D RViz ---
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ranger_3d_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz', default='true')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 with 3D config'),
        ranger_base_launch,
        ranger_sensors_launch,
        fast_lio_node,
        map_to_odom_tf,
        rviz_node,
    ])
