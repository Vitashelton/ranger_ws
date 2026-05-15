"""
Launch replan manager node.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    replan_manager = Node(
        package='ranger_replan_manager',
        executable='replan_manager_node',
        name='replan_manager_node',
        output='screen',
        parameters=[{
            'stuck_vel_threshold': 0.05,
            'stuck_duration': 5.0,
            'ttc_critical_threshold': 1.0,
            'ttc_critical_duration': 2.0,
            'blockage_duration': 3.0,
            'max_estop_count': 3,
            'estop_window': 30.0,
            'local_failure_max': 3,
            'slowdown_factor': 0.5,
            'recovery_cooldown': 5.0,
        }],
    )

    return LaunchDescription([replan_manager])
