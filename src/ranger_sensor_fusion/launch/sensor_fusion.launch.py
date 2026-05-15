"""
Launch MID360S + D435i sensor fusion pipeline.

Nodes:
  obstacle_cluster_node  — MID360S pointcloud clustering
  d435i_obstacle_node    — D435i near-field obstacle detection
  sensor_fusion_node     — Multi-sensor fusion + risk markers
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_cluster = Node(
        package='ranger_sensor_fusion',
        executable='obstacle_cluster_node',
        name='obstacle_cluster_node',
        output='screen',
        parameters=[{
            'roi_x_min': 0.3,
            'roi_x_max': 15.0,
            'roi_z_min': 0.1,
            'roi_z_max': 2.0,
            'voxel_leaf_size': 0.1,
            'cluster_tolerance': 0.15,
            'min_cluster_size': 5,
            'frame_id': 'base_link',
        }],
    )

    d435i_obstacle = Node(
        package='ranger_sensor_fusion',
        executable='d435i_obstacle_node',
        name='d435i_obstacle_node',
        output='screen',
        parameters=[{
            'max_range': 4.0,
            'min_range': 0.2,
            'min_height': 0.0,
            'max_height': 1.5,
            'cluster_tolerance': 0.08,
            'min_cluster_size': 10,
            'safety_zone_x_min': 0.1,
            'safety_zone_x_max': 1.0,
            'safety_zone_y_half_width': 0.4,
            'safety_critical_range': 0.3,
            'frame_id': 'base_link',
        }],
    )

    sensor_fusion = Node(
        package='ranger_sensor_fusion',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        output='screen',
        parameters=[{
            'association_max_dist': 0.5,
            'max_timestamp_diff': 0.1,
            'mid360_base_confidence': 0.85,
            'd435i_base_confidence': 0.8,
            'dual_detection_confidence': 0.95,
            'min_confidence_threshold': 0.3,
            'risk_enabled': True,
            'frame_id': 'base_link',
        }],
    )

    return LaunchDescription([
        obstacle_cluster,
        d435i_obstacle,
        sensor_fusion,
    ])
