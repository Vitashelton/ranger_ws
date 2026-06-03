from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/color/image_raw",
    )

    instruction_topic_arg = DeclareLaunchArgument(
        "instruction_topic",
        default_value="/vla/instruction",
    )

    action_topic_arg = DeclareLaunchArgument(
        "action_topic",
        default_value="/vla/action",
    )

    api_url_arg = DeclareLaunchArgument(
        "api_url",
        default_value="http://127.0.0.1:8000/predict",
    )

    control_rate_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="5.0",
    )

    vla_node = Node(
        package="vla_inference",
        executable="vla_node",
        name="vla_inference_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "instruction_topic": LaunchConfiguration("instruction_topic"),
                "action_topic": LaunchConfiguration("action_topic"),
                "api_url": LaunchConfiguration("api_url"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "image_width": 224,
                "image_height": 224,
                "action_dim": 7,
                "max_abs_action": [0.05, 0.05, 0.05, 0.2, 0.2, 0.2, 1.0],
                "max_delta_action": [0.02, 0.02, 0.02, 0.1, 0.1, 0.1, 1.0],
                "ema_alpha": 0.5,
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            instruction_topic_arg,
            action_topic_arg,
            api_url_arg,
            control_rate_arg,
            vla_node,
        ]
    )
