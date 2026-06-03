"""Launch the safety supervisor (the only /cmd_vel publisher)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tca_bev_nav')
    cfg = os.path.join(pkg, 'config', 'safety_supervisor.yaml')
    return LaunchDescription([
        Node(
            package='tca_bev_nav', executable='safety_supervisor_node',
            name='safety_supervisor_node', output='screen',
            parameters=[cfg],
        ),
    ])
