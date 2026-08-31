"""Safe physical bringup: base + MID360S CustomMsg + FAST-LIO + Nav2 TF adapters.

No controller or task node is launched, so the robot cannot move by itself.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    launch_dir = os.path.join(pkg, 'launch')
    start_base = LaunchConfiguration('start_base')
    start_d435i = LaunchConfiguration('start_d435i')
    use_rviz = LaunchConfiguration('use_rviz')
    livox_config = LaunchConfiguration('livox_config')

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_d435i', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'livox_config',
            default_value='/home/robot/livox_ws/src/livox_ros_driver2/config/MID360s_config.json'),
        include(os.path.join(launch_dir, 'ranger_base.launch.py'),
                condition=IfCondition(start_base)),
        include(os.path.join(launch_dir, 'ranger_sensors.launch.py'), arguments={
            'livox_config': livox_config,
            'xfer_format': '1',
            'start_d435i': start_d435i,
        }),
        include(os.path.join(launch_dir, 'ranger_3d_slam.launch.py'), arguments={
            'use_rviz': use_rviz,
        }),
        Node(
            package='ranger_nav', executable='cloud_frame_relay.py',
            name='cloud_frame_relay', output='screen'),
        Node(
            package='ranger_nav', executable='fastlio_nav_tf.py',
            name='fastlio_nav_tf', output='screen',
            parameters=[{
                'sensor_x': 0.30, 'sensor_y': 0.0, 'sensor_z': 0.70,
                'sensor_roll': 0.0, 'sensor_pitch': 0.523599,
                'sensor_yaw': 0.0, 'planarize': True,
            }]),
    ])
