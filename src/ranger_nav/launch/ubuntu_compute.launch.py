import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ranger_nav')

    use_sensor_fusion = LaunchConfiguration('use_sensor_fusion')
    use_dynamic_obstacle = LaunchConfiguration('use_dynamic_obstacle')
    use_replan = LaunchConfiguration('use_replan')

    # 1. PointCloud2 -> LaserScan
    # Ubuntu 订阅 Jetson 发来的 /livox/lidar，然后在 Ubuntu 上生成 /scan
    pcl_to_scan_config = os.path.join(
        pkg_dir,
        'config',
        'pointcloud_to_laserscan.yaml'
    )

    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': -1.0,
            'max_height': 2.0,

            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,

            'range_min': 0.2,
            'range_max': 30.0,

            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )


    # 2. MID360 + D435i obstacle fusion
    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ranger_sensor_fusion'),
                'launch',
                'sensor_fusion.launch.py'
            )
        ),
        condition=IfCondition(use_sensor_fusion),
    )

    # 3. Dynamic obstacle tracking / prediction / TTC risk
    dynamic_obstacle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ranger_dynamic_obstacle'),
                'launch',
                'dynamic_obstacle.launch.py'
            )
        ),
        condition=IfCondition(use_dynamic_obstacle),
    )

    # 4. Replan manager
    replan_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ranger_replan_manager'),
                'launch',
                'replan_manager.launch.py'
            )
        ),
        condition=IfCondition(use_replan),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sensor_fusion', default_value='true'),
        DeclareLaunchArgument('use_dynamic_obstacle', default_value='true'),
        DeclareLaunchArgument('use_replan', default_value='false'),

        # pcl_to_scan,
        sensor_fusion,
        dynamic_obstacle,
        replan_manager,
    ])
