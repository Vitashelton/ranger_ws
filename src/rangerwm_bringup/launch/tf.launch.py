"""静态 TF: base_link -> livox_frame / base_link -> camera_link.

外参值与 ranger_nav 保持一致 (来自 ranger_params.hpp + 实测初值, 标定后覆盖):
  livox_frame:  x=0.30, z=0.70, pitch=0.523599 rad (30° 上倾)
  camera_link:  x=0.35, z=0.60

当 hardware_up=true (外部已发布 TF) 或 publish_tfs=false 时跳过。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    hardware_up = LaunchConfiguration("hardware_up", default="false")
    publish_tfs = LaunchConfiguration("publish_tfs", default="true")

    should_publish = PythonExpression([
        "'", publish_tfs, "' == 'true' and '", hardware_up, "' == 'false'"
    ])

    # livox_frame (ranger_nav 确认: x=0.30, y=0, z=0.70, roll=0, pitch=30deg, yaw=0)
    livox_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        arguments=[
            "--x", "0.30", "--y", "0.0", "--z", "0.70",
            "--roll", "0.0", "--pitch", "0.523599", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "livox_frame",
        ],
        condition=IfCondition(should_publish),
    )

    # camera_link (ranger_nav 确认: x=0.35, y=0, z=0.60, 全部旋转 0; NEEDS_PHYSICAL_CONFIRMATION pitch)
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf",
        arguments=[
            "--x", "0.35", "--y", "0.0", "--z", "0.60",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
        condition=IfCondition(should_publish),
    )

    log_skip = LogInfo(
        msg="hardware_up=true or publish_tfs=false: 跳过静态 TF 发布 (外部已处理)。",
        condition=IfCondition(PythonExpression([
            "'", publish_tfs, "' == 'false' or '", hardware_up, "' == 'true'"
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_up", default_value="false"),
        DeclareLaunchArgument("publish_tfs", default_value="true"),
        livox_tf,
        camera_tf,
        log_skip,
    ])
