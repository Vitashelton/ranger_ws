"""数据采集: sensors + tf + data_recorder (原始另由 ros2 bag record)。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory("rangerwm_bringup")
    inc = lambda f: IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,"launch",f)))
    return LaunchDescription([
        inc("sensors.launch.py"), inc("tf.launch.py"),
        Node(package="rangerwm_data_recorder", executable="recorder", output="screen",
             parameters=[{"task_id":"goto","scene_id":"lab_a"}]),
    ])
