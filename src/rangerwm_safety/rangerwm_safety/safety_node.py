#!/usr/bin/env python3
"""rangerwm_safety/safety_node — 唯一 /cmd_vel 发布者 (20-50Hz, 最高优先级), 硬件适配版.

订阅:
  /policy/cmd_vel_raw  (geometry_msgs/Twist)  RangerWM 控制候选 (来自 cmd_to_mode_aware)
  /scan                (sensor_msgs/LaserScan)  前向扇区障碍距离
  /estop               (std_msgs/Bool)          急停 (遥控器/软件)

发布:
  /cmd_vel             (geometry_msgs/Twist)  唯一底盘控制出口
  /cmd_vel_safe        (geometry_msgs/Twist)  安全后的指令 (调试/记录)
  /safety/status       (std_msgs/String JSON) 安全状态

安全逻辑:
  1. 急停 -> 零速
  2. cmd 超时 -> 衰减 -> 硬超时停车
  3. 传感器陈旧 -> 停车 (scan 超时)
  4. 前向障碍距离 -> 硬停/软降速 (从 /scan 前向扇区实时计算)
  5. 模式切换冷却 (Ackermann <-> Spin 防抖)
  6. mode-aware 加速度限制 (Ackermann 1.0 m/s^2, Spin 2.0 rad/s^2, 来自 Nav2)
  7. 速度/角速度硬限幅

模式判别复用 rangerwm_kinematics.arbitration (与底盘仲裁一致).
当 enabled=false (Nav2 模式) 时不发布 /cmd_vel.
"""
import time
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from rangerwm_kinematics import arbitration as A, params as P


class SafetyNode(Node):
    def __init__(self):
        super().__init__("rangerwm_safety")

        # ---- 参数声明 ----
        d = self.declare_parameter
        d("rate_hz", 30.0)
        d("cmd_timeout_s", 0.3)
        d("cmd_hard_timeout_s", 0.6)
        d("a_max_ackermann", 1.0)          # Nav2 acc_lim_x
        d("a_max_spin", 2.0)               # Nav2 acc_lim_theta
        d("a_brake", 2.0)                  # 制动减速度
        d("mode_switch_cooldown_s", 0.4)
        d("d_hard_m", 0.35)                # 含 Mid-360 0.5m 盲区, D435i 近场补盲
        d("d_soft_m", 0.8)
        d("front_sector_deg", 60.0)        # 前向扇区半角
        d("scan_timeout_s", 0.5)           # scan 超时 -> 停车
        d("v_max", P.MAX_LINEAR_SPEED)
        d("wz_max", P.MAX_ANGULAR_SPEED)
        d("enabled", True)                 # false = Nav2 模式, 不发布
        d("mode", "rangerwm")              # rangerwm | nav2

        # ---- 状态 ----
        self.last_cmd = Twist()
        self.candidate = None
        self.cand_t = 0.0
        self.last_mode = P.SDK_PARK
        self.last_mode_switch_t = 0.0
        self.front_dist = math.inf
        self.last_scan_t = 0.0
        self.estop = False

        # ---- 订阅 ----
        self.create_subscription(Twist, "/policy/cmd_vel_raw", self.on_cmd, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 5)
        self.create_subscription(Bool, "/estop", self.on_estop, 1)

        # ---- 发布 ----
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_safe = self.create_publisher(Twist, "/cmd_vel_safe", 1)
        self.pub_status = self.create_publisher(String, "/safety/status", 10)

        # ---- 定时器 ----
        self.dt = 1.0 / float(self._g("rate_hz"))
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"Safety node ready (enabled={self._g('enabled')}, "
            f"Hz={self._g('rate_hz')}, d_hard={self._g('d_hard_m')}m, "
            f"d_soft={self._g('d_soft_m')}m)")

    # ----------------------------------------------------------------
    #  参数快捷方法
    # ----------------------------------------------------------------
    def _g(self, key):
        return self.get_parameter(key).value

    # ----------------------------------------------------------------
    #  回调
    # ----------------------------------------------------------------
    def on_cmd(self, msg: Twist):
        self.candidate = msg
        self.cand_t = time.time()

    def on_scan(self, msg: LaserScan):
        """从 /scan 提取前向扇区最近距离."""
        self.last_scan_t = time.time()
        self.front_dist = self._front_sector_min_dist(msg)

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)
        if self.estop:
            self.get_logger().warn("ESTOP active!", throttle_duration_sec=1.0)

    # ----------------------------------------------------------------
    #  安全主循环
    # ----------------------------------------------------------------
    def on_timer(self):
        now = time.time()
        enabled = bool(self._g("enabled"))

        # Nav2 模式: 不发布 /cmd_vel
        if not enabled:
            return

        # ---- 1) 急停 ----
        if self.estop:
            return self._publish(Twist(), "ESTOP")

        # ---- 2) 命令超时 ----
        if self.candidate is None or (now - self.cand_t) > self._g("cmd_timeout_s"):
            if self.candidate is None or (now - self.cand_t) > self._g("cmd_hard_timeout_s"):
                return self._publish(Twist(), "CMD_HARD_TIMEOUT")
            return self._publish(self._decay(self.last_cmd), "CMD_SOFT_TIMEOUT")

        # ---- 3) 传感器陈旧 ----
        if (now - self.last_scan_t) > self._g("scan_timeout_s"):
            self.get_logger().warn(
                f"Scan timeout ({now - self.last_scan_t:.1f}s), stopping.",
                throttle_duration_sec=2.0)
            return self._publish(Twist(), "SCAN_STALE")

        # ---- 构建候选指令 ----
        out = Twist()
        out.linear.x = max(-self._g("v_max"), min(self._g("v_max"),
                                                  self.candidate.linear.x))
        out.linear.y = self.candidate.linear.y
        out.angular.z = max(-self._g("wz_max"), min(self._g("wz_max"),
                                                     self.candidate.angular.z))

        # ---- 4) 前向障碍 ----
        stop_reason = ""
        if self.front_dist < self._g("d_hard_m"):
            return self._publish(Twist(), "FRONT_HARD_STOP")
        if self.front_dist < self._g("d_soft_m"):
            r = (self.front_dist - self._g("d_hard_m")) / (
                self._g("d_soft_m") - self._g("d_hard_m"))
            out.linear.x *= r
            out.linear.y *= r
            stop_reason = "FRONT_SOFT_SLOW"

        # ---- 5) 模式切换冷却 ----
        mode = A.arbitrate_twist(out.linear.x, out.linear.y, out.angular.z)
        if mode != self.last_mode and (now - self.last_mode_switch_t) < self._g(
                "mode_switch_cooldown_s"):
            out = Twist()
            mode = self.last_mode
            if stop_reason:
                stop_reason += "+COOLDOWN"
            else:
                stop_reason = "COOLDOWN"
        elif mode != self.last_mode:
            self.last_mode_switch_t = now
        self.last_mode = mode

        # ---- 6) mode-aware 加速度限制 ----
        a_max = self._g("a_max_spin") if mode == P.SDK_SPINNING else self._g("a_max_ackermann")
        out.linear.x = self._accel_clamp(out.linear.x, self.last_cmd.linear.x, a_max)

        self._publish(out, stop_reason if stop_reason else "OK")

    # ----------------------------------------------------------------
    #  前向扇区最近距离 (从 LaserScan)
    # ----------------------------------------------------------------
    def _front_sector_min_dist(self, msg: LaserScan) -> float:
        """在 scan 中取前向 ±front_sector_deg/2 扇区的最小有效距离."""
        half_deg = float(self._g("front_sector_deg")) / 2.0
        half_rad = math.radians(half_deg)

        min_d = math.inf
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # 检查是否在前向扇区内
                # 前向 = 0 rad; 扇区 = [-half_rad, +half_rad]
                if -half_rad <= angle <= half_rad:
                    if r < min_d:
                        min_d = r
            angle += msg.angle_increment
        return min_d

    # ----------------------------------------------------------------
    #  辅助方法
    # ----------------------------------------------------------------
    def _accel_clamp(self, v: float, v_prev: float, a_max: float) -> float:
        dv_max = a_max * self.dt
        return max(v_prev - dv_max, min(v_prev + dv_max, v))

    def _decay(self, cmd: Twist, k: float = 0.6) -> Twist:
        out = Twist()
        out.linear.x = cmd.linear.x * k
        out.linear.y = cmd.linear.y * k
        out.angular.z = cmd.angular.z * k
        return out

    def _publish(self, cmd: Twist, reason: str = "OK"):
        self.last_cmd = cmd
        now = time.time()
        self.pub.publish(cmd)
        self.pub_safe.publish(cmd)

        # 发布状态 JSON
        status = json.dumps({
            "t": now,
            "reason": reason,
            "vx": cmd.linear.x,
            "vy": cmd.linear.y,
            "wz": cmd.angular.z,
            "front_dist": self.front_dist,
            "mode": self.last_mode,
            "estop": self.estop,
        })
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
