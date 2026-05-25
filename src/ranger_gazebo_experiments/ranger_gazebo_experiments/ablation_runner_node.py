#!/usr/bin/env python3
"""
Ablation experiment runner node.

Reads fusion_modes.yaml and configures sensor/controller toggles
based on the selected mode parameter. Monitors experiment progress
and reports when the trial is complete or timed out.
"""
import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory


class AblationRunnerNode(Node):
    def __init__(self):
        super().__init__('ablation_runner_node')

        self.declare_parameter('mode', 'lidar_depth_fusion')
        self.declare_parameter('scenario', 'crossing_person')
        self.declare_parameter('random_seed', 0)
        self.declare_parameter('trial_timeout', 120.0)
        self.declare_parameter('fusion_config', '')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)

        self.mode = self.get_parameter('mode').value
        self.scenario = self.get_parameter('scenario').value
        self.seed = self.get_parameter('random_seed').value
        self.timeout = self.get_parameter('trial_timeout').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # Load mode config
        config_path = self.get_parameter('fusion_config').value
        if not config_path:
            pkg = get_package_share_directory('ranger_gazebo_experiments')
            config_path = os.path.join(pkg, 'config', 'fusion_modes.yaml')
        self.mode_config = self._load_mode_config(config_path)

        # State
        self.trial_started = False
        self.start_time = None
        self.robot_pose = (0.0, 0.0)

        # Subs
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.success_sub = self.create_subscription(Bool, '/experiment_success', self._success_cb, 10)

        # Timer for timeout check
        self.timer = self.create_timer(1.0, self._check_timeout)

        # Mode info publisher
        self.mode_pub = self.create_publisher(String, '/experiment_mode_info', 10)

        self.get_logger().info(
            f'ablation_runner_node: mode={self.mode} scenario={self.scenario} '
            f'seed={self.seed} timeout={self.timeout}s')
        self.get_logger().info(
            f'  sensors: {self.mode_config.get("sensors_enabled", [])}')
        self.get_logger().info(
            f'  fusion={self.mode_config.get("fusion_enabled", False)} '
            f'risk={self.mode_config.get("risk_enabled", False)}')

        self.start_time = self.get_clock().now()
        self.trial_started = True

    def _load_mode_config(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        modes = data.get('modes', {})
        if self.mode not in modes:
            self.get_logger().warn(
                f'Mode "{self.mode}" not found, defaulting to lidar_depth_fusion')
            return modes.get('lidar_depth_fusion', {})
        return modes[self.mode]

    def _odom_cb(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _success_cb(self, msg):
        if msg.data:
            self.get_logger().info('Experiment SUCCESS — goal reached')
        else:
            self.get_logger().info('Experiment FAILURE reported')

    def _check_timeout(self):
        if not self.trial_started or self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        dist_to_goal = math.hypot(
            self.goal_x - self.robot_pose[0], self.goal_y - self.robot_pose[1])

        if elapsed > self.timeout:
            self.get_logger().warn(
                f'Trial TIMEOUT after {elapsed:.1f}s (limit={self.timeout}s) '
                f'dist_to_goal={dist_to_goal:.2f}m')
            # Publish timeout event
            msg = String()
            msg.data = f'timeout:{self.mode}:{self.scenario}:{self.seed}'
            self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AblationRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
