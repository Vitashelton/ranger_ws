"""RangerWM 控制节点 only (假设底盘 + 传感器 + TF 已由 ranger_nav 启动).

用法:
  # 先启动硬件
  ros2 launch ranger_nav ranger_full.launch.py mode:=mapping
  # 再启动 RangerWM 控制 (另开终端)
  ros2 launch rangerwm_bringup rangerwm_only.launch.py

控制链路:
  /scan + /odom + /goal_pose -> mpc_node -> /policy/mode_aware_cmd
      -> cmd_to_mode_aware -> /policy/cmd_vel_raw -> safety_node -> /cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rangerwm_bringup")
    cfg_dir = os.path.join(pkg_share, "config")

    wm_backend = LaunchConfiguration("wm_backend", default="analytic")

    return LaunchDescription([
        DeclareLaunchArgument("wm_backend", default_value="analytic",
                              description="analytic | torch | onnx | trt"),

        # BEV from scan (stand-in)
        Node(
            package="rangerwm_preprocess",
            executable="bev_from_scan",
            name="bev_from_scan",
            output="screen",
        ),

        # World model (analytic fallback)
        Node(
            package="rangerwm_world_model",
            executable="wm_node",
            name="rangerwm_world_model",
            output="screen",
            parameters=[{"backend": wm_backend}],
        ),

        # MPC reranking
        Node(
            package="rangerwm_mpc",
            executable="mpc_node",
            name="rangerwm_mpc",
            output="screen",
            parameters=[
                os.path.join(cfg_dir, "bringup_params.yaml"),
                {"wm_backend": wm_backend},
            ],
        ),

        # ModeAwareCmd -> Twist
        Node(
            package="rangerwm_preprocess",
            executable="cmd_to_mode_aware",
            name="cmd_to_mode_aware",
            output="screen",
            parameters=[{
                "backend": "twist",
                "input_topic": "/policy/mode_aware_cmd",
                "output_topic": "/policy/cmd_vel_raw",
            }],
        ),

        # Safety supervisor (sole /cmd_vel publisher)
        Node(
            package="rangerwm_safety",
            executable="safety_node",
            name="rangerwm_safety",
            output="screen",
            parameters=[
                os.path.join(cfg_dir, "safety_supervisor.yaml"),
                {"enabled": True, "mode": "rangerwm"},
            ],
        ),
    ])
