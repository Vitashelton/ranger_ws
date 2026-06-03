"""Launch TCA-BEV perception stack: time_align + calibration + fusion.

Does NOT launch the chassis driver or sensor drivers — those are the official
packages you bring up separately (see README). This launch only starts the
research nodes in this package.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tca_bev_nav')
    cfg = os.path.join(pkg, 'config')

    return LaunchDescription([
        Node(
            package='tca_bev_nav', executable='time_align_node',
            name='time_align_node', output='screen',
            parameters=[os.path.join(cfg, 'time_align.yaml')],
        ),
        Node(
            package='tca_bev_nav', executable='calibration_uncertainty_node',
            name='calibration_uncertainty_node', output='screen',
            parameters=[os.path.join(cfg, 'extrinsics.yaml')],
        ),
        Node(
            package='tca_bev_nav', executable='tca_bev_fusion_node',
            name='tca_bev_fusion_node', output='screen',
            parameters=[os.path.join(cfg, 'tca_bev_params.yaml')],
        ),
    ])
