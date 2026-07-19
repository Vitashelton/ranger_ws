#!/usr/bin/env python3
"""rangerwm_policy/policy_node — 策略推理 (action_only | joint | imagined)。

订阅 BEV/obs + odom + goal, 发布 ModeAwareCmd 到 /policy/mode_aware_cmd。
torch 缺失或无 ckpt 时 dry-run 发 STOP。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from rangerwm_msgs.msg import ModeAwareCmd

class PolicyNode(Node):
    def __init__(self):
        super().__init__("rangerwm_policy")
        self.declare_parameter("policy_type", "action_only")  # action_only|joint|imagined
        self.declare_parameter("ckpt", "")
        self.declare_parameter("rate_hz", 8.0)
        self.bev = None; self.ego = np.zeros(4, np.float32); self.goal = np.zeros(3, np.float32)
        self.model = self._load()
        self.create_subscription(Float32MultiArray, "/bev/tensor", self.on_bev, 5)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.pub = self.create_publisher(ModeAwareCmd, "/policy/mode_aware_cmd", 10)
        self.create_timer(1.0/float(self.get_parameter("rate_hz").value), self.step)

    def _load(self):
        ckpt = self.get_parameter("ckpt").value
        try:
            import torch  # noqa
        except Exception:
            self.get_logger().warn("no torch; dry-run STOP."); return None
        if not ckpt:
            self.get_logger().warn("no ckpt; dry-run STOP."); return None
        # from learning.models.policy import RangerPolicy ...
        return None

    def on_bev(self, m): self.bev = np.asarray(m.data, np.float32)
    def on_odom(self, m):
        self.ego = np.array([m.twist.twist.linear.x, m.twist.twist.angular.z, 0, 0], np.float32)

    def step(self):
        cmd = ModeAwareCmd(); cmd.header.stamp = self.get_clock().now().to_msg()
        if self.model is None or self.bev is None:
            cmd.mode = ModeAwareCmd.MODE_STOP
        else:
            # TODO: 推理取首步 (mode, p1, p2)
            cmd.mode = ModeAwareCmd.MODE_STOP
        self.pub.publish(cmd)

def main():
    rclpy.init(); n = PolicyNode()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
