from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rqt_image', default_value='false'),
        DeclareLaunchArgument('use_plotjuggler', default_value='false'),
        DeclareLaunchArgument('use_foxglove_bridge', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('ranger_nav_v5'), 'config', 'pc_monitor.rviz'
        ])),

        # PC-side RViz. It only subscribes to Jetson topics.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rangermini_v5_pc_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),

        # Optional image view for /debug/bev_image.
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='bev_image_view',
            condition=IfCondition(LaunchConfiguration('use_rqt_image')),
            output='screen',
        ),

        # Optional PlotJuggler for /intervention_score, /min_distance, cmd_vel curves.
        ExecuteProcess(
            cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
            condition=IfCondition(LaunchConfiguration('use_plotjuggler')),
            output='screen',
        ),

        # Optional: run this on Jetson instead if you want browser visualization from PC.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            condition=IfCondition(LaunchConfiguration('use_foxglove_bridge')),
            parameters=[{'port': 8765, 'address': '0.0.0.0'}],
            output='screen',
        ),
    ])
