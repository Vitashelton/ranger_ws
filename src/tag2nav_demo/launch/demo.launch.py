from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("layout_file", default_value="selected_layout.json"),
        Node(package="tag2nav_demo", executable="layout_publisher",
             parameters=[{"layout_file": LaunchConfiguration("layout_file")}], output="screen"),
    ])
