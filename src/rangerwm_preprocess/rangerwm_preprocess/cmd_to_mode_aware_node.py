#!/usr/bin/env python3
"""P0 节点: cmd_to_mode_aware

订阅 rangerwm_msgs/ModeAwareCmd (策略/MPC 输出的 mode-aware 动作),
转换为底盘可执行指令并发布:

  backend = "twist"  (默认, 今天即可用):
      转成 geometry_msgs/Twist, 发布到 output_topic (默认 /policy/cmd_vel_raw),
      由现有 safety_supervisor_node 校验后下发到唯一 /cmd_vel。
      => 不破坏 "safety 是唯一 /cmd_vel 发布者" 的架构。

  backend = "direct_can" (NEEDS_PHYSICAL_CONFIRMATION):
      调用 ugv_sdk AgilexBase::SendMotionCommand 或自组 CAN 0x141+0x111。
      绕过 Twist 仲裁; 需先确认固件支持且与 safety 集成 (此处仅打印, 不实发)。

依赖: rangerwm_kinematics (纯 python, 与训练共用同一映射, 保证 sim/real 一致)。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rangerwm_msgs.msg import ModeAwareCmd
from rangerwm_kinematics import mode_aware as MA, params as P


class CmdToModeAware(Node):
    def __init__(self):
        super().__init__("cmd_to_mode_aware")
        self.declare_parameter("backend", "twist")          # twist | direct_can
        self.declare_parameter("input_topic", "/policy/mode_aware_cmd")
        self.declare_parameter("output_topic", "/policy/cmd_vel_raw")
        self.declare_parameter("wheelbase", P.L_NOMINAL)
        self.backend = self.get_parameter("backend").value
        self.L = float(self.get_parameter("wheelbase").value)

        self.sub = self.create_subscription(
            ModeAwareCmd, self.get_parameter("input_topic").value, self.cb, 10)
        if self.backend == "twist":
            self.pub = self.create_publisher(
                Twist, self.get_parameter("output_topic").value, 10)
        else:
            self.pub = None
            self.get_logger().warn(
                "backend=direct_can: CAN 0x141/0x111 直发尚未实接 (NEEDS_CONFIRMATION); 仅日志。")

    def cb(self, msg: ModeAwareCmd):
        if self.backend == "twist":
            vx, vy, wz, info = MA.mode_aware_to_twist(msg.mode, msg.param1, msg.param2, self.L)
            if info.get("clamped"):
                self.get_logger().debug(f"mode {msg.mode} params clamped for twist backend")
            if info.get("note"):
                self.get_logger().warn(info["note"])
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            self.pub.publish(t)
        else:
            frame = MA.mode_aware_to_can(msg.mode, msg.param1, msg.param2)
            # TODO(real): 接 ugv_sdk SendMotionCommand 或自组 CAN; 需与 safety 联动
            self.get_logger().info(f"[direct_can] {frame}")


def main():
    rclpy.init()
    node = CmdToModeAware()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
