"""PC compute nodes for FAST-LIO, Nav2 and bounded camera evidence.

The Jetson publishes Livox, IMU, wheel odometry and camera topics over DDS and
subscribes to the final /cmd_vel.  No hardware driver is started here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    pkg = get_package_share_directory('ranger_nav')
    launch_dir = os.path.join(pkg, 'launch')
    fast_lio_config = os.path.join(pkg, 'config', 'fastlio_mid360.yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    start_camera_evidence = LaunchConfiguration('start_camera_evidence')
    use_prior_localization = LaunchConfiguration('use_prior_localization')
    prior_map = LaunchConfiguration('prior_map')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # FAST-LIO must publish map->odom before Nav2 costmaps are activated.
        # Start Nav2 explicitly through the lifecycle manager after TF is ready.
        DeclareLaunchArgument('nav2_autostart', default_value='false'),
        DeclareLaunchArgument('start_camera_evidence', default_value='false'),
        DeclareLaunchArgument('use_prior_localization', default_value='false'),
        DeclareLaunchArgument(
            'prior_map',
            default_value='/home/zbx/.config/ranger_nav/maps/real_lab_3d_level_full.pcd'),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',
            output='screen',
            parameters=[fast_lio_config, {
                'use_sim_time': False,
                # The prior-map localizer publishes a bounded downsampled map.
                # Avoid FAST-LIO's unbounded /Laser_map because it can freeze RViz.
                'publish.map_en': False,
            }],
        ),
        # Level the FAST-LIO session frame using the verified MID-360S mount.
        # This is display/TF plumbing only; it does not alter point data.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            output='screen',
            condition=UnlessCondition(use_prior_localization),
            arguments=[
                '--x', '0.30', '--y', '0.0', '--z', '0.70',
                '--roll', '0.0', '--pitch', '0.523599', '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'camera_init',
            ],
        ),
        Node(
            package='ranger_nav',
            executable='cloud_frame_relay.py',
            name='cloud_frame_relay',
            output='screen',
        ),
        Node(
            package='ranger_nav',
            executable='fastlio_nav_tf.py',
            name='fastlio_nav_tf',
            output='screen',
            condition=UnlessCondition(use_prior_localization),
            parameters=[{
                'sensor_x': 0.30,
                'sensor_y': 0.0,
                'sensor_z': 0.70,
                'sensor_roll': 0.0,
                'sensor_pitch': 0.523599,
                'sensor_yaw': 0.0,
                'planarize': True,
            }],
        ),
        Node(
            package='ranger_nav',
            executable='prior_map_localizer',
            name='prior_map_localizer',
            output='screen',
            condition=IfCondition(use_prior_localization),
            parameters=[{
                'map_path': prior_map,
                'cloud_topic': '/cloud_registered',
                'lio_topic': '/Odometry',
                'wheel_topic': '/odom',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'base_to_body_x': 0.30,
                'base_to_body_y': 0.0,
                'base_to_body_z': 0.70,
                'base_to_body_pitch': 0.523599,
                'initial_x': 0.30,
                'initial_y': 0.0,
                'initial_z': 0.70,
                'initial_pitch': 0.523599,
                'map_voxel': 0.15,
                'scan_voxel': 0.15,
                'ndt_resolution': 0.80,
                'ndt_iterations': 20,
                'process_every_n': 10,
            }],
        ),
        Node(
            package='ranger_nav',
            executable='d435i_marker_evidence.py',
            name='d435i_marker_evidence',
            output='screen',
            condition=IfCondition(start_camera_evidence),
        ),
        include(
            os.path.join(launch_dir, 'ranger_nav_pointcloud.launch.py'),
            {'autostart': nav2_autostart},
        ),
        include(
            os.path.join(launch_dir, 'pc_monitor.launch.py'),
            condition=IfCondition(use_rviz),
        ),
    ])
