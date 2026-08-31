"""Ranger Mini hardware profile for the edge-LLM slow/fast overlay.

Fast layer remains ranger_nav's Fast-LIO + Nav2 stack. The slow layer only
publishes verified behavior/task messages and never owns /cmd_vel.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='edge_llm_nav', executable='llm_task_server', name='llm_task_server', output='screen', parameters=[{'model': 'llama3.2:3b'}]),
        Node(package='edge_llm_nav', executable='task_graph_verifier', name='task_graph_verifier', output='screen', parameters=[{'known_targets': ['dock', 'lab', 'charging_station']}]),
        Node(package='edge_llm_nav', executable='task_executor', name='task_executor', output='screen'),
        Node(package='edge_llm_nav', executable='execution_monitor', name='execution_monitor', output='screen'),
        Node(package='edge_llm_nav', executable='llm_recovery_policy', name='llm_recovery_policy', output='screen'),
        Node(package='edge_llm_nav', executable='experiment_logger', name='experiment_logger', output='screen', parameters=[{'csv_path': '/tmp/ranger_edge_llm_metrics.csv'}]),
    ])
