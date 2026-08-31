"""Launch MID360S natively (no PointCloud2-to-LaserScan conversion)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')
    livox_config = LaunchConfiguration('livox_config')
    xfer_format = LaunchConfiguration('xfer_format')
    start_d435i = LaunchConfiguration('start_d435i')

    static_tf = Node(
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

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            # FAST-LIO's Livox path requires CustomMsg (xfer_format=1).
            # Use 0 only when running the driver without FAST-LIO.
            'xfer_format': ParameterValue(xfer_format, value_type=int),
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
            'qos_overrides./livox/lidar.publisher.reliability': 'best_effort',
            'qos_overrides./livox/lidar.publisher.history': 'keep_last',
            'qos_overrides./livox/lidar.publisher.depth': 5,
        }],
    )

    d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'd435i_sensor.launch.py')
        ),
        condition=IfCondition(start_d435i),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'livox_config',
            default_value=(
                '/home/robot/livox_ws/src/livox_ros_driver2/'
                'config/MID360s_config.json'
            ),
        ),
        DeclareLaunchArgument('xfer_format', default_value='1'),
        DeclareLaunchArgument('start_d435i', default_value='false'),
        static_tf,
        livox_driver,
        d435i,
    ])
