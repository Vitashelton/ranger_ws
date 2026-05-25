"""
Launch Gazebo lab world + Ranger Mini 2.0 simulation model + sensors + RViz.

Usage:
  ros2 launch ranger_gazebo_experiments ranger_lab_world.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_gazebo_experiments')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Arguments
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    # Gazebo — uses ranger_lab_corridor.world by default
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'ranger_lab_corridor.world'),
        }.items(),
    )

    # Robot state publisher
    urdf_path = os.path.join(pkg_dir, 'urdf', 'ranger_mini_sim.urdf.xacro')
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ['xacro ', urdf_path]}],
    )

    # Joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        name='spawn_ranger', output='screen',
        arguments=['-entity', 'ranger_mini_sim', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1', '-timeout', '30.0'],
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ranger_lab_fusion.rviz')],
    )

    return LaunchDescription([
        declare_gui, declare_rviz, gazebo, robot_state_pub,
        joint_state_pub, spawn_robot, rviz_node,
    ])
