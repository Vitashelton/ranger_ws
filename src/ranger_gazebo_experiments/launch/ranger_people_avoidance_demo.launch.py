"""
Launch full people avoidance demo: Gazebo world + Ranger robot + animated human actor +
YOLO person detector + sensor fusion + avoidance controller + metrics + RViz.

Usage:
  ros2 launch ranger_gazebo_experiments ranger_people_avoidance_demo.launch.py
  ros2 launch ranger_gazebo_experiments ranger_people_avoidance_demo.launch.py \
    scenario:=crossing_person mode:=fusion_with_risk_avoidance \
    random_seed:=0 goal_x:=8.0 goal_y:=0.0 yolo_device:=cpu
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
    args = [
        DeclareLaunchArgument('scenario', default_value='crossing_person'),
        DeclareLaunchArgument('mode', default_value='fusion_with_risk_avoidance'),
        DeclareLaunchArgument('random_seed', default_value='0'),
        DeclareLaunchArgument('use_yolo', default_value='true'),
        DeclareLaunchArgument('yolo_device', default_value='cpu'),
        DeclareLaunchArgument('use_ground_truth_for_control', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='8.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('trial_timeout', default_value='120.0'),
    ]

    world = LaunchConfiguration('world', default='ranger_people_corridor.world')
    scenario = LaunchConfiguration('scenario')
    mode_lc = LaunchConfiguration('mode')
    seed_lc = LaunchConfiguration('random_seed')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    yolo_device = LaunchConfiguration('yolo_device')
    use_gt_ctrl = LaunchConfiguration('use_ground_truth_for_control')

    # Gazebo + people world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'ranger_people_corridor.world'),
        }.items(),
    )

    # Robot
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

    # Ground truth publisher
    ground_truth = Node(
        package='ranger_gazebo_experiments', executable='simulated_people_ground_truth_node',
        name='simulated_people_ground_truth_node', output='screen',
        parameters=[{
            'scenario': scenario,
            'scenario_config': os.path.join(pkg_dir, 'config', 'people_scenarios.yaml'),
            'world_frame': 'odom',
            'publish_rate': 30.0,
        }],
    )

    # Actor proxy sync
    proxy_sync = Node(
        package='ranger_gazebo_experiments', executable='actor_proxy_sync_node',
        name='actor_proxy_sync_node', output='screen',
        parameters=[{'ground_truth_topic': '/sim/people_ground_truth', 'update_rate': 30.0}],
    )

    # YOLO person detector
    yolo_detector = Node(
        package='ranger_gazebo_experiments', executable='yolo_person_detector_node',
        name='yolo_person_detector_node', output='screen',
        parameters=[{
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.35,
            'iou_threshold': 0.45,
            'device': yolo_device,
            'image_topic': '/camera/color/image_raw',
            'detections_topic': '/yolo/person_detections',
            'debug_image_topic': '/yolo/person_debug_image',
            'markers_topic': '/yolo/person_markers',
            'publish_debug_image': True,
            'publish_markers': True,
        }],
    )

    # YOLO detections to 3D obstacles
    yolo_to_obs = Node(
        package='ranger_gazebo_experiments', executable='person_detection_to_obstacle_node',
        name='person_detection_to_obstacle_node', output='screen',
        parameters=[{
            'detections_topic': '/yolo/person_detections',
            'depth_image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'obstacles_topic': '/obstacles_yolo_person',
            'frame_id': 'camera_color_optical_frame',
        }],
    )

    # MID360 obstacle clustering
    obstacle_cluster = Node(
        package='ranger_sensor_fusion', executable='obstacle_cluster_node',
        name='obstacle_cluster_node', output='screen',
        parameters=[{
            'input_topic': '/livox/lidar', 'frame_id': 'odom',
            'roi_x_min': -20.0, 'roi_x_max': 20.0,
            'roi_y_min': -20.0, 'roi_y_max': 20.0,
            'voxel_leaf_size': 0.25, 'cluster_tolerance': 0.5,
            'min_cluster_size': 3, 'max_obstacles': 50,
        }],
    )

    # D435i depth obstacle
    d435i_obstacle = Node(
        package='ranger_sensor_fusion', executable='d435i_obstacle_node',
        name='d435i_obstacle_node', output='screen',
        parameters=[{
            'input_topic': '/camera/depth/color/points', 'frame_id': 'odom',
            'camera_optical_to_robot_frame': True,
            'max_range': 4.0, 'min_range': 0.2,
        }],
    )

    # Sensor fusion (with optional YOLO input)
    sensor_fusion = Node(
        package='ranger_sensor_fusion', executable='sensor_fusion_node',
        name='sensor_fusion_node', output='screen',
        parameters=[{
            'mid360_obstacles_topic': '/obstacles_mid360',
            'd435i_obstacles_topic': '/obstacles_d435i',
            'fused_obstacles_topic': '/fused_obstacles',
            'risk_markers_topic': '/risk_markers',
            'yolo_obstacles_topic': '/obstacles_yolo_person',
            'frame_id': 'odom', 'risk_enabled': True,
        }],
    )

    # Person avoidance controller
    avoidance_ctrl = Node(
        package='ranger_gazebo_experiments', executable='person_avoidance_controller',
        name='person_avoidance_controller', output='screen',
        parameters=[{
            'max_linear_speed': 0.5, 'max_angular_speed': 0.8,
            'slow_down_distance': 2.0, 'stop_distance': 0.8,
            'safety_radius': 0.5, 'goal_tolerance': 0.5,
            'goal_x': goal_x, 'goal_y': goal_y,
            'use_fused_obstacles': True,
            'use_ground_truth_for_control': use_gt_ctrl,
            'fused_obstacles_topic': '/fused_obstacles',
            'ground_truth_topic': '/sim/people_ground_truth',
            'frame_id': 'odom',
        }],
    )

    # Experiment metrics
    metrics = Node(
        package='ranger_gazebo_experiments', executable='experiment_metrics_node',
        name='experiment_metrics_node', output='screen',
        parameters=[{
            'scenario_name': scenario, 'random_seed': seed_lc,
            'mode': mode_lc,
            'goal_x': goal_x, 'goal_y': goal_y,
            'odom_topic': '/odom', 'cmd_vel_topic': '/cmd_vel',
            'ground_truth_topic': '/sim/people_ground_truth',
            'yolo_detections_topic': '/yolo/person_detections',
            'fused_obstacles_topic': '/fused_obstacles',
            'risk_markers_topic': '/risk_markers',
            'goal_pose_topic': '/local_goal',
            'log_rate': 10.0,
        }],
    )

    # Ablation runner
    ablation_runner = Node(
        package='ranger_gazebo_experiments', executable='ablation_runner_node',
        name='ablation_runner_node', output='screen',
        parameters=[{
            'mode': mode_lc, 'scenario': scenario,
            'random_seed': seed_lc, 'trial_timeout': LaunchConfiguration('trial_timeout'),
            'goal_x': goal_x, 'goal_y': goal_y,
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ranger_people_avoidance.rviz')],
    )

    return LaunchDescription([
        *args,
        gazebo, robot_state_pub, spawn_robot,
        ground_truth, proxy_sync,
        yolo_detector, yolo_to_obs,
        obstacle_cluster, d435i_obstacle, sensor_fusion,
        avoidance_ctrl, metrics, ablation_runner, rviz_node,
    ])
