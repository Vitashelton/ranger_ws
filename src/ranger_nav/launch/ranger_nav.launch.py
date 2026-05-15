"""
Nav2 navigation launch for Ranger Mini 2.0.

Requires a pre-built map. Runs localization + planning + control.

Usage:
  ros2 launch ranger_nav ranger_nav.launch.py map:=/home/robot/maps/ranger_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    # --- Arguments ---
    map_yaml = LaunchConfiguration('map', default='/home/robot/maps/ranger_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    nav2_params_file = LaunchConfiguration(
        'nav2_params_file',
        default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    )
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_dir, 'config', 'slam_toolbox_localization.yaml')
    )

    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='false')

    # --- Include sub-launches ---
    ranger_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_base.launch.py')
        ),
    )

    ranger_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ranger_sensors.launch.py')
        ),
    )

    # --- Localization (slam_toolbox in localization mode) ---
    localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # --- Map server ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # --- Nav2 lifecycle nodes ---
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config],
    )

    # Lifecycle manager: auto-activates all nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
            'bond_timeout': 10.0,
        }],
    )

    # --- RViz2 ---
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ranger_nav.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/robot/maps/ranger_map.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_config),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(pkg_dir, 'config',
                                                         'slam_toolbox_localization.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='false'),

        # Chassis + sensors
        ranger_base_launch,
        ranger_sensors_launch,

        # Localization
        localization_node,
        map_server_node,

        # Nav2 stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,

        # RViz
        rviz2_node,
    ])
