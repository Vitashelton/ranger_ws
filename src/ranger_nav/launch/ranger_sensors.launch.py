"""
Launch Ranger Mini 2.0 sensors: Livox MID360S driver + pointcloud-to-LaserScan + static TF.

Livox driver config: /home/robot/livox_ws/src/livox_ros_driver2/config/MID360s_config.json
Static TF: base_link -> livox_frame (0, 0, 0.35)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=[
            '--x', '0.30',
            '--y', '0.0',
            '--z', '0.70',
            '--roll', '0.0',
            '--pitch', '0.523599',  # 30度
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
        output='screen',
    )
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
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

    # --- Livox MID360S driver ---
    livox_config = os.path.join(
        '/home/robot/livox_ws/src/livox_ros_driver2', 'config', 'MID360s_config.json'
    )
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
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
        lidar_static_tf,   # 雷达TF
        camera_static_tf,  # 相机TF
        livox_driver,      # 雷达驱动
        d435i_node,        # 相机驱动
    ])
