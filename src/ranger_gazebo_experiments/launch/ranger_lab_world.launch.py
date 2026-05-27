"""
Launch Gazebo lab world + Ranger Mini 2.0 simulation model + sensors + RViz.

Usage:
  ros2 launch ranger_gazebo_experiments ranger_lab_world.launch.py
  ros2 launch ranger_gazebo_experiments ranger_lab_world.launch.py gui:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_gazebo_experiments')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo client GUI'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'ranger_lab_corridor.world'),
        description='Gazebo world file'
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
        }.items(),
    )

    # URDF / Xacro
    urdf_path = os.path.join(pkg_dir, 'urdf', 'ranger_mini_sim.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            urdf_path,
        ]),
        value_type=str
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )


    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ranger',
        output='screen',
        arguments=[
            '-entity', 'ranger_mini_sim',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-timeout', '30.0',
        ],
    )

    # RViz
    # 建议你新建/修改这个 rviz 文件，让 Fixed Frame = odom，不要用 camera_init
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ranger_lab_fusion.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True,
        }],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        declare_world,

        gazebo,
        robot_state_pub,

        # 延迟 spawn，避免 Gazebo 和 robot_description 还没准备好
        TimerAction(
            period=3.0,
            actions=[spawn_robot],
        ),

        rviz_node,
    ])
