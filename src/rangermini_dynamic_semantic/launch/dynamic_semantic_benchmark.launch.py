#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rangermini_dynamic_semantic")
    world = PathJoinSubstitution([pkg, "worlds", "dynamic_indoor_benchmark.sdf"])
    config = PathJoinSubstitution([pkg, "config", "dynamic_benchmark.yaml"])
    default_task = PathJoinSubstitution([pkg, "config", "task_context.yaml"])
    common = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool),
        "require_sim_time": ParameterValue(
            LaunchConfiguration("require_sim_time"), value_type=bool),
        "trial_id": LaunchConfiguration("trial_id"),
        "scenario_id": LaunchConfiguration("scenario_id"),
        "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
        "method_mode": LaunchConfiguration("method_mode"),
    }
    return LaunchDescription([
        DeclareLaunchArgument("launch_gazebo", default_value="true"),
        DeclareLaunchArgument("enable_trial_reset", default_value="true"),
        DeclareLaunchArgument("enable_demo_controller", default_value="true"),
        DeclareLaunchArgument("enable_disturbances", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="mixed_shift"),
        DeclareLaunchArgument("trial_id", default_value="manual_trial"),
        DeclareLaunchArgument("scenario_id", default_value="S6"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("method_mode", default_value="Ours"),
        DeclareLaunchArgument("task_context_file", default_value=default_task),
        DeclareLaunchArgument("task_context_json", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("require_sim_time", default_value="true"),
        DeclareLaunchArgument("auto_reset_on_start", default_value="true"),
        DeclareLaunchArgument(
            "expected_reset_nodes",
            default_value=("dynamic_scene_manager,streaming_perception_scheduler,"
                           "rgbd_semantic_perception,temporal_semantic_memory,"
                           "dynamic_topology_maintenance,"
                           "benchmark_navigation_controller,dynamic_benchmark_metrics")),
        ExecuteProcess(cmd=["ign", "gazebo", "-r", world], output="screen",
                       condition=IfCondition(LaunchConfiguration("launch_gazebo"))),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="dynamic_benchmark_bridge", output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/model/rangermini_2_0/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                "/model/rangermini_2_0/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
                "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/world/dynamic_indoor_benchmark/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            ],
            remappings=[("/model/rangermini_2_0/cmd_vel", "/cmd_vel_safe")],
        ),
        Node(package="rangermini_dynamic_semantic", executable="gazebo_odom_adapter",
             parameters=[{"input_topic": "/model/rangermini_2_0/pose",
                          "use_sim_time": common["use_sim_time"]}], output="screen"),
        Node(package="rangermini_dynamic_semantic", executable="trial_reset_coordinator",
             parameters=[common, {
                 "task_context_file": LaunchConfiguration("task_context_file"),
                 "task_context_json": LaunchConfiguration("task_context_json"),
                 "auto_reset_on_start": ParameterValue(
                     LaunchConfiguration("auto_reset_on_start"), value_type=bool),
                 "expected_reset_nodes": LaunchConfiguration("expected_reset_nodes"),
             }], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_trial_reset"))),
        Node(package="rangermini_dynamic_semantic", executable="dynamic_scene_manager",
             parameters=[common, {"config_file": config,
                          "scenario": LaunchConfiguration("scenario"),
                          "task_context_file": LaunchConfiguration("task_context_file"),
                          "task_context_json": LaunchConfiguration("task_context_json")}], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_disturbances"))),
        Node(package="rangermini_dynamic_semantic", executable="streaming_perception_scheduler",
             parameters=[common, {"config_file": config,
                          "task_context_file": LaunchConfiguration("task_context_file"),
                          "task_context_json": LaunchConfiguration("task_context_json")}], output="screen"),
        Node(package="rangermini_dynamic_semantic", executable="rgbd_semantic_perception",
             parameters=[common, {"config_file": config}], output="screen"),
        Node(package="rangermini_dynamic_semantic", executable="temporal_semantic_memory",
             parameters=[common, {"config_file": config}], output="screen"),
        Node(package="rangermini_dynamic_semantic", executable="dynamic_topology_maintenance",
             parameters=[common, {"config_file": config}], output="screen"),
        Node(package="rangermini_dynamic_semantic", executable="benchmark_navigation_controller",
             parameters=[common, {"config_file": config}], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_demo_controller"))),
        Node(package="rangermini_dynamic_semantic", executable="dynamic_benchmark_metrics",
             parameters=[common, {"config_file": config}], output="screen"),
    ])
