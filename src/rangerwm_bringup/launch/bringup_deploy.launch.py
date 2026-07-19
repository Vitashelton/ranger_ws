"""RangerWM 部署入口 —— Ranger Mini 2.0 硬件适配版.

支持三种模式 (launch argument `mode`):
  rangerwm  - 世界模型 MPC 控制 (wm_node + mpc_node + cmd_to_mode_aware + safety_node)
  nav2      - Nav2 经典导航 (RangerWM safety_node 退让, 不发布 /cmd_vel)
  none      - 纯传感器/底盘 (录包 / 手动遥控 / 调试)

支持两种硬件启动方式 (launch argument `hardware_up`):
  false (默认) - Include ranger_nav 的 base + sensors launch, 拉起全部硬件驱动
  true         - 假设底盘 + 传感器 + TF 已在外部启动, 只运行 RangerWM 控制节点

控制链路 (mode=rangerwm):
  mpc_node --ModeAwareCmd--> cmd_to_mode_aware --Twist--> safety_node --/cmd_vel--> ranger_base_node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rangerwm_bringup")
    cfg_dir = os.path.join(pkg_share, "config")

    # ---- Launch arguments ----
    mode = LaunchConfiguration("mode", default="rangerwm")
    hardware_up = LaunchConfiguration("hardware_up", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    wm_backend = LaunchConfiguration("wm_backend", default="analytic")
    policy_type = LaunchConfiguration("policy_type", default="none")  # none = MPC-only (no learned policy)

    # ---- Condition expressions ----
    is_rangerwm = PythonExpression(["'", mode, "' == 'rangerwm'"])
    is_nav2 = PythonExpression(["'", mode, "' == 'nav2'"])
    is_none = PythonExpression(["'", mode, "' == 'none'"])
    hw_down = PythonExpression(["'", hardware_up, "' == 'false'"])

    # ---- Hardware: ranger_base (chassis driver) ----
    ranger_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ranger_nav"),
                         "launch", "ranger_base.launch.py")
        ),
        launch_arguments={
            "port_name": "can1",
            "robot_model": "ranger_mini_v2",
            "publish_odom_tf": "true",
            "update_rate": "50",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "odom_topic_name": "odom",
        }.items(),
        condition=IfCondition(hw_down),
    )

    # ---- Hardware: sensors (Livox Mid-360 + D435i + pointcloud_to_laserscan) ----
    ranger_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sensors.launch.py")
        ),
        launch_arguments={"hardware_up": hardware_up}.items(),
    )

    # ---- Static TF (only when hardware TF not already running) ----
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "tf.launch.py")
        ),
        launch_arguments={
            "hardware_up": hardware_up,
            "publish_tfs": PythonExpression(["'", hardware_up, "' == 'false' and 'true' or 'false'"]),
        }.items(),
    )

    # ---- RangerWM: world model ----
    wm_node = Node(
        package="rangerwm_world_model",
        executable="wm_node",
        name="rangerwm_world_model",
        output="screen",
        parameters=[{"backend": wm_backend}],
        condition=IfCondition(is_rangerwm),
    )

    # ---- RangerWM: MPC reranking ----
    mpc_node = Node(
        package="rangerwm_mpc",
        executable="mpc_node",
        name="rangerwm_mpc",
        output="screen",
        parameters=[
            os.path.join(cfg_dir, "bringup_params.yaml"),
            {"wm_backend": wm_backend},
        ],
        condition=IfCondition(is_rangerwm),
    )

    # ---- RangerWM: cmd_to_mode_aware (ModeAwareCmd -> Twist) ----
    cmd_to_mode_aware_node = Node(
        package="rangerwm_preprocess",
        executable="cmd_to_mode_aware",
        name="cmd_to_mode_aware",
        output="screen",
        parameters=[{
            "backend": "twist",
            "input_topic": "/policy/mode_aware_cmd",
            "output_topic": "/policy/cmd_vel_raw",
        }],
        condition=IfCondition(is_rangerwm),
    )

    # ---- RangerWM: safety supervisor ----
    safety_node = Node(
        package="rangerwm_safety",
        executable="safety_node",
        name="rangerwm_safety",
        output="screen",
        parameters=[
            os.path.join(cfg_dir, "safety_supervisor.yaml"),
            {
                "enabled": PythonExpression(["'", mode, "' != 'nav2' and 'true' or 'false'"]),
                "mode": mode,
            },
        ],
    )

    # ---- BEV from scan (stand-in until tca_bev_nav is running) ----
    bev_node = Node(
        package="rangerwm_preprocess",
        executable="bev_from_scan",
        name="bev_from_scan",
        output="screen",
        condition=IfCondition(is_rangerwm),
    )

    # ---- Log mode ----
    log_mode = LogInfo(
        msg=PythonExpression(["'RangerWM bringup mode=[", mode, "] hardware_up=[", hardware_up, "]'"]),
    )

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="rangerwm",
                              description="Control mode: rangerwm | nav2 | none"),
        DeclareLaunchArgument("hardware_up", default_value="false",
                              description="true = base+sensors already running externally"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("wm_backend", default_value="analytic",
                              description="analytic (no torch) | torch | onnx | trt"),
        DeclareLaunchArgument("policy_type", default_value="none",
                              description="none (MPC-only) | action_only | joint | imagined"),

        log_mode,

        # Hardware layer (conditional)
        ranger_base_launch,
        ranger_sensors_launch,
        tf_launch,

        # RangerWM control layer (mode=rangerwm)
        bev_node,
        wm_node,
        mpc_node,
        cmd_to_mode_aware_node,

        # Safety (always launched; disabled in nav2 mode)
        safety_node,
    ])
