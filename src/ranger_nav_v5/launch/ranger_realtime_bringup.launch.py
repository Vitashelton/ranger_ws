from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Hardware includes are intentionally optional.
    # Keep enable_drive:=false until /cmd_vel_safe is verified.
    args = [
        DeclareLaunchArgument('use_base', default_value='false',
                              description='Include existing ranger_nav/ranger_base.launch.py. Default false for safety.'),
        DeclareLaunchArgument('use_mid360', default_value='true',
                              description='Include existing ranger_nav/ranger_sensors.launch.py when MID360 is needed.'),
        DeclareLaunchArgument('use_d435i', default_value='true',
                              description='Include existing ranger_nav/ranger_sensors.launch.py when D435i is needed.'),
        DeclareLaunchArgument('use_semantic', default_value='true',
                              description='Run semantic memory / target layer.'),
        DeclareLaunchArgument('use_semantic_stub', default_value='false',
                              description='Publish fake room detections for bench testing without YOLO.'),
        DeclareLaunchArgument('use_shared_control', default_value='true',
                              description='Run risk-aware shared-control safety filter.'),
        DeclareLaunchArgument('use_logger', default_value='true',
                              description='Record /cmd_vel_raw /cmd_vel_safe /odom /risk into CSV.'),
        DeclareLaunchArgument('enable_drive', default_value='false',
                              description='If true, republish /cmd_vel_safe to /cmd_vel through cmd_vel_guard. Keep false first.'),
        DeclareLaunchArgument('target_room', default_value='906',
                              description='Semantic target room id, e.g. 906 or 908.'),
        DeclareLaunchArgument('human_cmd_topic', default_value='/cmd_vel_raw'),
        DeclareLaunchArgument('safe_cmd_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('final_cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('risk_grid_topic', default_value='/local_risk_grid'),
        DeclareLaunchArgument('log_dir', default_value='/tmp/rangermini_v5_logs'),
    ]

    cfg_risk = PathJoinSubstitution([FindPackageShare('ranger_nav_v5'), 'config', 'v5_risk_params.yaml'])
    cfg_ctrl = PathJoinSubstitution([FindPackageShare('ranger_nav_v5'), 'config', 'v5_shared_control.yaml'])
    cfg_sem = PathJoinSubstitution([FindPackageShare('ranger_nav_v5'), 'config', 'v5_semantic_rooms.yaml'])

    # Existing hardware package. This keeps your favorite style:
    # ros2 launch ranger_nav ranger_realtime_bringup.launch.py ...
    # In this standalone package, use ranger_nav_v5; the drop-in launch keeps the package name ranger_nav.
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ranger_nav'), 'launch', 'ranger_base.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('use_base')),
    )

    sensors_needed = PythonExpression([
        "'", LaunchConfiguration('use_mid360'), "' == 'true' or '",
        LaunchConfiguration('use_d435i'), "' == 'true'"
    ])

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ranger_nav'), 'launch', 'ranger_sensors.launch.py'])
        ),
        condition=IfCondition(sensors_needed),
    )

    semantic_stub = Node(
        package='ranger_nav_v5',
        executable='semantic_detector_stub',
        name='semantic_detector_stub',
        condition=IfCondition(LaunchConfiguration('use_semantic_stub')),
        parameters=[cfg_sem, {'target_room': ParameterValue(LaunchConfiguration('target_room'), value_type=str)}],
        output='screen',
    )

    semantic_memory = Node(
        package='ranger_nav_v5',
        executable='semantic_memory_node',
        name='semantic_memory_node',
        condition=IfCondition(LaunchConfiguration('use_semantic')),
        parameters=[cfg_sem, {
            'target_room': ParameterValue(LaunchConfiguration('target_room'), value_type=str),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
        output='screen',
    )

    risk_node = Node(
        package='ranger_nav_v5',
        executable='local_bev_risk_node',
        name='local_bev_risk_node',
        parameters=[cfg_risk, {
            'scan_topic': LaunchConfiguration('scan_topic'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'risk_grid_topic': LaunchConfiguration('risk_grid_topic'),
        }],
        output='screen',
    )

    shared_filter = Node(
        package='ranger_nav_v5',
        executable='shared_control_filter',
        name='shared_control_filter',
        condition=IfCondition(LaunchConfiguration('use_shared_control')),
        parameters=[cfg_ctrl, {
            'human_cmd_topic': LaunchConfiguration('human_cmd_topic'),
            'safe_cmd_topic': LaunchConfiguration('safe_cmd_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'risk_grid_topic': LaunchConfiguration('risk_grid_topic'),
            'target_room': ParameterValue(LaunchConfiguration('target_room'), value_type=str)
        }],
        output='screen',
    )

    guard = Node(
        package='ranger_nav_v5',
        executable='cmd_vel_guard',
        name='cmd_vel_guard',
        condition=IfCondition(LaunchConfiguration('enable_drive')),
        parameters=[{
            'input_topic': LaunchConfiguration('safe_cmd_topic'),
            'output_topic': LaunchConfiguration('final_cmd_topic'),
            'max_vx': 0.25,
            'max_vy': 0.20,
            'max_wz': 0.50,
            'deadman_timeout': 0.35,
        }],
        output='screen',
    )

    logger = Node(
        package='ranger_nav_v5',
        executable='realtime_csv_logger',
        name='realtime_csv_logger',
        condition=IfCondition(LaunchConfiguration('use_logger')),
        parameters=[{
            'log_dir': LaunchConfiguration('log_dir'),
            'human_cmd_topic': LaunchConfiguration('human_cmd_topic'),
            'safe_cmd_topic': LaunchConfiguration('safe_cmd_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
        output='screen',
    )

    return LaunchDescription(args + [
        base_launch,
        sensors_launch,
        risk_node,
        semantic_stub,
        semantic_memory,
        shared_filter,
        guard,
        logger,
    ])
