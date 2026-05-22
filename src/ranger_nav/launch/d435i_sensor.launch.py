"""
Launch Intel RealSense D435i RGB-D camera for Ranger Mini 2.0.

Publishes:
  /camera/depth/color/points — Registered depth pointcloud (for obstacles)
  /camera/color/image_raw     — RGB image (for debugging/visualization)
  /camera/depth/image_rect_raw — Depth image

Static TF: base_link -> camera_link (extrinsics: [TBD] — measure after mounting)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    # --- Static TF: base_link -> camera_link ---
    # [TBD] values — must be measured after D435i mounting
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0.35',        # [TBD] forward offset
            '--y', '0.0',         # centered
            '--z', '0.60',        # [TBD] height above ground
            '--roll', '0.0',
            '--pitch', '0.0', # [TBD] -15° downward tilt
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    # --- D435i camera driver ---
    d435i_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        parameters=[{
            'camera_name': 'camera',

            'enable_color': True,
            'enable_depth': True,

            'align_depth.enable': True,
            'pointcloud.enable': True,

            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,

            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }]
    )

    return LaunchDescription([
        static_tf,
        d435i_node,
    ])
