"""传感器驱动: Livox Mid-360 + RealSense D435i + pointcloud_to_laserscan.

当 hardware_up=false 时 Include ranger_nav 的 ranger_sensors.launch.py,
否则假设传感器已在外部启动 (什么都不做)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hardware_up = LaunchConfiguration("hardware_up", default="false")
    hw_down = PythonExpression(["'", hardware_up, "' == 'false'"])

    # Include ranger_nav's battle-tested sensor launch (Livox + D435i + pointcloud_to_laserscan + livox_frame TF)
    ranger_nav_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ranger_nav"),
                         "launch", "ranger_sensors.launch.py")
        ),
        condition=IfCondition(hw_down),
    )

    log_skip = LogInfo(
        msg="hardware_up=true: 假设传感器已在外部启动, 跳过 RangerWM 自带传感器 launch.",
        condition=IfCondition(PythonExpression(["'", hardware_up, "' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_up", default_value="false",
                              description="true = sensors already running externally"),
        ranger_nav_sensors,
        log_skip,
    ])
