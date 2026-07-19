import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelGuard(Node):
    """Final safety gate from /cmd_vel_safe to /cmd_vel.

    It is intentionally separated from shared_control_filter so that enable_drive:=false can be used for dry runs.
    """
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.input_topic = self.declare_parameter('input_topic', '/cmd_vel_safe').value
        self.output_topic = self.declare_parameter('output_topic', '/cmd_vel').value
        self.max_vx = float(self.declare_parameter('max_vx', 0.25).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.20).value)
        self.max_wz = float(self.declare_parameter('max_wz', 0.50).value)
        self.deadman_timeout = float(self.declare_parameter('deadman_timeout', 0.35).value)

        self.last = Twist()
        self.last_time = 0.0
        self.create_subscription(Twist, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().warn(f'cmd_vel_guard ENABLED: {self.input_topic} -> {self.output_topic}. Keep robot lifted / low speed first.')

    def cb(self, msg):
        self.last = msg
        self.last_time = time.time()

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def step(self):
        msg = Twist()
        if time.time() - self.last_time <= self.deadman_timeout:
            msg.linear.x = float(self.clamp(self.last.linear.x, -self.max_vx, self.max_vx))
            msg.linear.y = float(self.clamp(self.last.linear.y, -self.max_vy, self.max_vy))
            msg.angular.z = float(self.clamp(self.last.angular.z, -self.max_wz, self.max_wz))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
