"""
Launch dynamic obstacle tracking + prediction + risk evaluation.

Nodes:
  obstacle_tracker_node   — Kalman filter multi-object tracking
  obstacle_predictor_node — Constant-velocity trajectory prediction
  risk_evaluator_node     — TTC-based risk evaluation
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tracker = Node(
        package='ranger_dynamic_obstacle',
        executable='obstacle_tracker_node',
        name='obstacle_tracker_node',
        output='screen',
        parameters=[{
            'association_max_dist': 0.5,
            'birth_threshold': 3,
            'death_threshold': 5,
            'confirmed_threshold': 10,
            'process_noise': 0.1,
            'measurement_noise': 0.05,
            'frame_id': 'base_link',
        }],
    )

    predictor = Node(
        package='ranger_dynamic_obstacle',
        executable='obstacle_predictor_node',
        name='obstacle_predictor_node',
        output='screen',
        parameters=[{
            'prediction_horizon': 2.0,
            'prediction_step': 0.2,
            'min_speed_for_prediction': 0.05,
            'frame_id': 'base_link',
        }],
    )

    risk_evaluator = Node(
        package='ranger_dynamic_obstacle',
        executable='risk_evaluator_node',
        name='risk_evaluator_node',
        output='screen',
        parameters=[{
            'ttc_threshold_low': 3.0,
            'ttc_threshold_medium': 1.5,
            'ttc_threshold_high': 0.5,
            'collision_distance': 0.35,
            'robot_radius': 0.35,
            'frame_id': 'base_link',
        }],
    )

    return LaunchDescription([
        tracker,
        predictor,
        risk_evaluator,
    ])
