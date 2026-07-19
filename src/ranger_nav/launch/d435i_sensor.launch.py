#!/usr/bin/python3
"""
Launch Intel RealSense D435i RGB-D camera for Ranger Mini 2.0.

Expected point cloud:
  /camera/depth/color/points

TF branch:
  base_link -> camera_link -> camera_*_frame -> camera_*_optical_frame
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Camera mounting transform. Replace these estimated values after calibration.
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_tf',
        arguments=[
            '--x', '0.35',
            '--y', '0.0',
            '--z', '0.60',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    d435i_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        parameters=[{
            'camera_name': 'camera',

            # Streams
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,

            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',

            # Align depth to color
            'align_depth.enable': True,

            # Point cloud
            'pointcloud.enable': True,
            'pointcloud.ordered_pc': False,
            'pointcloud.allow_no_texture_points': True,

            # D435i IMU is not used for now
            'enable_gyro': False,
            'enable_accel': False,

            # The RealSense driver publishes its internal camera TFs.
            # A zero rate keeps fixed transforms on /tf_static.
            'publish_tf': True,
            'tf_publish_rate': 0.0,

            # base_frame_id is a suffix: camera_name + '_' + base_frame_id.
            # Therefore 'link' produces the required root frame camera_link.
            'base_frame_id': 'link',
            'color_frame_id': 'camera_color_frame',
            'color_optical_frame_id': 'camera_color_optical_frame',
            'depth_frame_id': 'camera_depth_frame',
            'depth_optical_frame_id': 'camera_depth_optical_frame',
        }],
    )

    return LaunchDescription([
        camera_static_tf,
        d435i_node,
    ])
