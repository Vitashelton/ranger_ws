"""
Launch Ranger Mini 2.0 base driver with odom TF enabled.

Equivalent to:
  ros2 launch ranger_bringup ranger_mini_v2.launch.py \\
      port_name:=can1 robot_model:=ranger_mini_v2 publish_odom_tf:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    # --- Arguments ---
    port_name = LaunchConfiguration('port_name', default='can1')
    robot_model = LaunchConfiguration('robot_model', default='ranger_mini_v2')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='true')
    update_rate = LaunchConfiguration('update_rate', default='50')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    base_frame = LaunchConfiguration('base_frame', default='base_link')
    odom_topic_name = LaunchConfiguration('odom_topic_name', default='odom')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    simulated_robot = LaunchConfiguration('simulated_robot', default='false')

    # --- Nodes ---
    ranger_base_node = Node(
        package='ranger_base',
        executable='ranger_base_node',
        name='ranger_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port_name': port_name,
            'robot_model': robot_model,
            'publish_odom_tf': publish_odom_tf,
            'update_rate': update_rate,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'odom_topic_name': odom_topic_name,
            'use_sim_time': use_sim_time,
            'simulated_robot': simulated_robot,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='can1'),
        DeclareLaunchArgument('robot_model', default_value='ranger_mini_v2'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('update_rate', default_value='50'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic_name', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('simulated_robot', default_value='false'),
        ranger_base_node,
    ])
