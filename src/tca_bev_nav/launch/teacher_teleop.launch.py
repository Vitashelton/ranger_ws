"""Launch ROS teleop for collecting *teacher* commands.

Critical: teleop_twist_keyboard's /cmd_vel output is remapped to
/teacher/cmd_vel_raw so it can NEVER drive the chassis directly. Only the
safety supervisor publishes /cmd_vel.

Run the safety supervisor separately (safety_supervisor.launch.py) with
source:=teacher to actually move the robot.

Note: teleop_twist_keyboard needs an interactive terminal, so launching it via
ros2 launch can be awkward. The README also gives the plain `ros2 run ...
--remap` one-liner, which is usually more convenient for keyboard teleop.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teacher_teleop',
            output='screen',
            prefix='xterm -e',  # needs a real TTY; adjust to your terminal
            remappings=[('/cmd_vel', '/teacher/cmd_vel_raw')],
        ),
    ])
