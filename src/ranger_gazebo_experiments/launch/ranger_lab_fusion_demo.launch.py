"""
Launch lab world + sensor fusion pipeline demo.

Nodes:
  - Gazebo lab world + Ranger robot
  - obstacle_cluster_node (MID360 3D LiDAR → /obstacles_mid360)
  - d435i_obstacle_node (depth camera → /obstacles_d435i)
  - sensor_fusion_node (Hungarian fusion → /fused_obstacles, /risk_markers)
  - RViz

Usage:
  ros2 launch ranger_gazebo_experiments ranger_lab_fusion_demo.launch.py
  ros2 launch ranger_gazebo_experiments ranger_lab_fusion_demo.launch.py \
    use_lidar:=true use_depth:=true use_yolo:=false mode:=lidar_depth_fusion
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
    mode = LaunchConfiguration('mode')
    use_lidar = LaunchConfiguration('use_lidar')
    use_depth = LaunchConfiguration('use_depth')
    use_yolo = LaunchConfiguration('use_yolo')

    args = [
        DeclareLaunchArgument('mode', default_value='lidar_depth_fusion'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('use_yolo', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='ranger_lab_corridor.world'),
    ]

    # Gazebo + robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'ranger_lab_corridor.world'),
        }.items(),
    )

    urdf_path = os.path.join(pkg_dir, 'urdf', 'ranger_mini_sim.urdf.xacro')
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ['xacro ', urdf_path]}],
    )

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        name='spawn_ranger', output='screen',
        arguments=['-entity', 'ranger_mini_sim', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
    )

    # MID360 obstacle clustering (configured for simulation LiDAR topic)
    obstacle_cluster = Node(
        package='ranger_sensor_fusion', executable='obstacle_cluster_node',
        name='obstacle_cluster_node', output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'frame_id': 'odom',
            'roi_x_min': -20.0, 'roi_x_max': 20.0,
            'roi_y_min': -20.0, 'roi_y_max': 20.0,
            'roi_z_min': -3.0, 'roi_z_max': 5.0,
            'voxel_leaf_size': 0.25, 'cluster_tolerance': 0.5,
            'min_cluster_size': 3, 'max_obstacles': 50,
        }],
    )

    # D435i depth obstacle node
    d435i_obstacle = Node(
        package='ranger_sensor_fusion', executable='d435i_obstacle_node',
        name='d435i_obstacle_node', output='screen',
        parameters=[{
            'input_topic': '/camera/depth/color/points',
            'frame_id': 'odom',
            'camera_optical_to_robot_frame': True,
            'max_range': 4.0, 'min_range': 0.2,
            'min_height': -0.2, 'max_height': 1.5,
            'cluster_tolerance': 0.08, 'min_cluster_size': 10,
        }],
    )

    # Sensor fusion
    sensor_fusion = Node(
        package='ranger_sensor_fusion', executable='sensor_fusion_node',
        name='sensor_fusion_node', output='screen',
        parameters=[{
            'mid360_obstacles_topic': '/obstacles_mid360',
            'd435i_obstacles_topic': '/obstacles_d435i',
            'fused_obstacles_topic': '/fused_obstacles',
            'risk_markers_topic': '/risk_markers',
            'frame_id': 'odom',
            'risk_enabled': True,
            'yolo_obstacles_topic': '/obstacles_yolo_person',
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ranger_lab_fusion.rviz')],
    )

    return LaunchDescription([
        *args, gazebo, robot_state_pub, spawn_robot,
        obstacle_cluster, d435i_obstacle, sensor_fusion, rviz_node,
    ])
