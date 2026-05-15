#!/usr/bin/env python3
"""
Online navigation metrics logger.

Subscribes to navigation topics and periodically writes metrics to CSV.
Runs alongside navigation; does not interfere with control loop.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('output_dir', '/home/robot/metrics')
        self.declare_parameter('log_interval', 1.0)
        output_dir = self.get_parameter('output_dir').value
        log_interval = self.get_parameter('log_interval').value

        os.makedirs(output_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(output_dir, f'metrics_{timestamp}.csv')

        # Metrics accumulators
        self.episode_start = None
        self.goal_reached = False
        self.collision_detected = False
        self.min_obstacle_dist = float('inf')
        self.path_length = 0.0
        self.last_position = None
        self.replan_count = 0
        self.estop_count = 0
        self.failure_count = 0
        self.vel_samples = []
        self.mode_switch_count = 0
        self.computation_times = {}

        # Subs
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.event_sub = self.create_subscription(String, '/replan_event', self._event_cb, 10)

        # Write CSV header
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'pos_x', 'pos_y', 'yaw', 'vel_x', 'vel_y', 'vel_wz',
                'path_length', 'min_obstacle_dist', 'replan_count', 'estop_count',
                'failure_count', 'vel_smoothness',
            ])

        self.timer = self.create_timer(log_interval, self._log_metrics)
        self.get_logger().info(f'Metrics logger started, output: {self.csv_path}')

    def _odom_cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)

        if self.last_position is not None:
            dx = px - self.last_position[0]
            dy = py - self.last_position[1]
            self.path_length += math.hypot(dx, dy)
        self.last_position = (px, py)

        self.vel_samples.append((
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ))

    def _plan_cb(self, msg):
        pass  # track plan metrics if needed

    def _event_cb(self, msg):
        if 'RECOVERY' in msg.data:
            self.replan_count += 1
        if 'SAFETY STOP' in msg.data:
            self.estop_count += 1
        if 'local_failure' in msg.data or 'stuck' in msg.data:
            self.failure_count += 1

    def _log_metrics(self):
        now = self.get_clock().now().to_msg()
        t = now.sec + now.nanosec * 1e-9

        px, py = self.last_position or (0.0, 0.0)
        vx, vy, wz = self.vel_samples[-1] if self.vel_samples else (0.0, 0.0, 0.0)

        # Smoothness: running std of velocity magnitude
        if len(self.vel_samples) > 1:
            speeds = [math.hypot(v[0], v[1]) for v in self.vel_samples[-20:]]
            smoothness = float(sum(abs(speeds[i] - speeds[i-1]) for i in range(1, len(speeds))) / max(len(speeds)-1, 1))
        else:
            smoothness = 0.0

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f'{t:.3f}', f'{px:.4f}', f'{py:.4f}', '0.0',
                f'{vx:.4f}', f'{vy:.4f}', f'{wz:.4f}',
                f'{self.path_length:.4f}',
                f'{self.min_obstacle_dist:.4f}',
                self.replan_count, self.estop_count,
                self.failure_count, f'{smoothness:.4f}',
            ])

    def __del__(self):
        self.get_logger().info(f'Metrics saved to {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
