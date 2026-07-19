"""
Launch only the sensors needed by TCA-BEV:
  - base_link -> livox_frame static TF
  - Livox MID360S driver publishing /livox/lidar as CustomMsg
  - D435i driver and base_link -> camera_link static TF

No /scan conversion is launched here. TCA-BEV uses the 3D MID360S data directly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    # LiDAR mounting transform. Keep only one publisher for this transform.
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=[
            '--x', '0.30',
            '--y', '0.0',
            '--z', '0.70',
            '--roll', '0.0',
            '--pitch', '0.523599',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
        output='screen',
    )

    livox_config = os.path.join(
        '/home/robot/livox_ws/src/livox_ros_driver2', 'config', 'MID360s_config.json'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,

            # 0 keeps livox_ros_driver2/msg/CustomMsg, which your current system publishes.
            # TCA-BEV receives converted PointCloud2 from mid360s_adapter, not directly here.
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'd435i_sensor.launch.py')
        ),
    )

    return LaunchDescription([
        lidar_static_tf,
        livox_driver,
        d435i,
    ])