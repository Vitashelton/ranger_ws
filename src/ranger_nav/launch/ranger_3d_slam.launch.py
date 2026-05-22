import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    use_rviz = LaunchConfiguration('use_rviz')
    fast_lio_config = os.path.join(pkg_dir, 'config', 'fastlio_mid360.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'ranger_3d_slam.rviz')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fast_lio_config],

        # 先不 remap，保持 FAST-LIO2 默认输出 /Odometry
        # 如果你后面想统一成 /odom_lidar，再打开下面这行
        # remappings=[('/Odometry', '/odom_lidar')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with 3D SLAM config'
        ),

        fast_lio_node,
        rviz_node,
    ])
