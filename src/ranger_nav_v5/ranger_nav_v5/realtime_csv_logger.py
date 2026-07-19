import csv
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class RealtimeCsvLogger(Node):
    def __init__(self):
        super().__init__('realtime_csv_logger')
        self.log_dir = Path(str(self.declare_parameter('log_dir', '/tmp/rangermini_v5_logs').value))
        self.human_cmd_topic = self.declare_parameter('human_cmd_topic', '/cmd_vel_raw').value
        self.safe_cmd_topic = self.declare_parameter('safe_cmd_topic', '/cmd_vel_safe').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.raw = Twist()
        self.safe = Twist()
        self.odom = None
        self.intervention = None
        self.min_distance = None
        self.target = None

        self.create_subscription(Twist, self.human_cmd_topic, lambda m: setattr(self, 'raw', m), 10)
        self.create_subscription(Twist, self.safe_cmd_topic, lambda m: setattr(self, 'safe', m), 10)
        self.create_subscription(Odometry, self.odom_topic, lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Float32, '/intervention_score', lambda m: setattr(self, 'intervention', float(m.data)), 10)
        self.create_subscription(Float32, '/min_distance', lambda m: setattr(self, 'min_distance', float(m.data)), 10)
        self.create_subscription(PoseStamped, '/semantic_target_pose', lambda m: setattr(self, 'target', m), 10)

        self.log_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.path = self.log_dir / f'rangermini_v5_trial_{ts}.csv'
        self.f = self.path.open('w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow([
            't',
            'vx_raw', 'vy_raw', 'wz_raw',
            'vx_safe', 'vy_safe', 'wz_safe',
            'odom_x', 'odom_y', 'odom_yaw',
            'intervention_score', 'min_distance',
            'target_x', 'target_y',
        ])
        self.t0 = time.time()
        self.create_timer(0.05, self.step)
        self.get_logger().info(f'CSV logging to {self.path}')

    def yaw(self, q):
        import math
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def step(self):
        t = time.time() - self.t0
        ox = oy = oyaw = ''
        if self.odom is not None:
            ox = self.odom.pose.pose.position.x
            oy = self.odom.pose.pose.position.y
            oyaw = self.yaw(self.odom.pose.pose.orientation)
        tx = ty = ''
        if self.target is not None:
            tx = self.target.pose.position.x
            ty = self.target.pose.position.y
        self.writer.writerow([
            f'{t:.3f}',
            f'{self.raw.linear.x:.4f}', f'{self.raw.linear.y:.4f}', f'{self.raw.angular.z:.4f}',
            f'{self.safe.linear.x:.4f}', f'{self.safe.linear.y:.4f}', f'{self.safe.angular.z:.4f}',
            ox, oy, oyaw,
            '' if self.intervention is None else f'{self.intervention:.4f}',
            '' if self.min_distance is None else f'{self.min_distance:.4f}',
            tx, ty
        ])
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RealtimeCsvLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
