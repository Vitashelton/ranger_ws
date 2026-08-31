from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params = PathJoinSubstitution([
        get_package_share_directory('semantic_door_nav'),
        'config',
        'semantic_nav.yaml',
    ])
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML parameter file.',
        ),
        Node(
            package='semantic_door_nav',
            executable='aruco_door_observer',
            name='aruco_door_observer',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='semantic_door_nav',
            executable='topology_manager',
            name='topology_manager',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='semantic_door_nav',
            executable='reactive_local_controller',
            name='reactive_local_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
