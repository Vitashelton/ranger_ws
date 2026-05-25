#!/usr/bin/env python3
"""
Comprehensive experiment metrics recorder.

Subscribes to navigation and perception topics, records per-tick metrics,
and writes CSV on shutdown. Computes summary statistics at end of trial.

CSV schema follows the spec: 30+ columns covering robot state, perception,
person distance, collision events, and navigation performance.
"""
import csv
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool

from .utils_geometry import quaternion_to_euler
from .utils_metrics import compute_min_distance_to_person


def _get_git_commit():
    try:
        result = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, cwd=os.path.expanduser('~/ranger_ws'))
        return result.stdout.strip() if result.returncode == 0 else 'unknown'
    except Exception:
        return 'unknown'


class ExperimentMetricsNode(Node):
    def __init__(self):
        super().__init__('experiment_metrics_node')

        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/ranger_ws/experiments/results'))
        self.declare_parameter('log_rate', 10.0)
        self.declare_parameter('scenario_name', 'unknown')
        self.declare_parameter('random_seed', 0)
        self.declare_parameter('mode', 'unknown')
        self.declare_parameter('person_count', 0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ground_truth_topic', '/sim/people_ground_truth')
        self.declare_parameter('yolo_detections_topic', '/yolo/person_detections')
        self.declare_parameter('fused_obstacles_topic', '/fused_obstacles')
        self.declare_parameter('risk_markers_topic', '/risk_markers')
        self.declare_parameter('goal_pose_topic', '/local_goal')
        self.declare_parameter('dangerous_close_distance', 0.35)
        self.declare_parameter('collision_distance', 0.25)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('stuck_velocity_threshold', 0.03)
        self.declare_parameter('stuck_duration_threshold', 15.0)

        p = lambda n: self.get_parameter(n).value

        # Config
        self.scenario = p('scenario_name')
        self.seed = p('random_seed')
        self.mode = p('mode')
        self.goal_x = p('goal_x')
        self.goal_y = p('goal_y')
        self.goal_tol = p('goal_tolerance')
        self.dangerous_close_d = p('dangerous_close_distance')
        self.collision_d = p('collision_distance')
        self.stuck_vel = p('stuck_velocity_threshold')
        self.stuck_dur = p('stuck_duration_threshold')

        # State
        self.robot_pose = (0.0, 0.0, 0.0)
        self.robot_vel = (0.0, 0.0)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.gt_people = []
        self.person_count = p('person_count')
        self.yolo_count = 0
        self.yolo_miss_count = 0
        self.fused_obs_count = 0
        self.high_risk_count = 0
        self.stop_count = 0
        self.replan_count = 0
        self.collision_count = 0
        self.dangerous_close_count = 0
        self.path_length = 0.0
        self.last_position = None
        self.vel_samples = []
        self.start_time = self.get_clock().now()
        self.goal_reached = False
        self.is_success = False
        self.failure_reason = ''
        self.stuck_start = None
        self.prev_collision = False
        self.prev_dangerous = False

        # YOLO metrics
        self.detector_fps = 0.0
        self.detector_latency_ms = 0.0

        # Subs
        self.odom_sub = self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, p('cmd_vel_topic'), self._cmd_cb, 10)
        self.gt_sub = self.create_subscription(PoseArray, p('ground_truth_topic'), self._gt_cb, 10)
        self.yolo_sub = self.create_subscription(Detection2DArray, p('yolo_detections_topic'), self._yolo_cb, 10)
        self.fused_sub = self.create_subscription(MarkerArray, p('fused_obstacles_topic'), self._fused_cb, 10)
        self.risk_sub = self.create_subscription(MarkerArray, p('risk_markers_topic'), self._risk_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, p('goal_pose_topic'), self._goal_cb, 10)
        self.success_sub = self.create_subscription(Bool, '/experiment_success', self._success_cb, 10)

        # CSV setup
        self.output_dir = p('output_dir')
        os.makedirs(self.output_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'gazebo_people_avoidance_{self.scenario}_{self.mode}_s{self.seed}_{ts}.csv')
        self._write_header()

        # Timer
        self.timer = self.create_timer(1.0 / p('log_rate'), self._log)
        self.get_logger().info(f'experiment_metrics_node started, output: {self.csv_path}')

    def _write_header(self):
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'timestamp', 'scenario_name', 'random_seed', 'mode', 'person_count',
                'robot_x', 'robot_y', 'robot_yaw', 'goal_x', 'goal_y',
                'distance_to_goal', 'cmd_linear', 'cmd_angular',
                'min_distance_to_person', 'collision_count', 'dangerous_close_count',
                'yolo_detection_count', 'yolo_miss_count',
                'detector_fps', 'detector_latency_ms',
                'fused_obstacle_count', 'high_risk_count',
                'stop_count', 'replan_count', 'path_length',
                'average_speed', 'navigation_time',
                'success', 'failure_reason',
            ])

    def _odom_cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.robot_pose = (px, py, yaw)

        if self.last_position is not None:
            dx = px - self.last_position[0]
            dy = py - self.last_position[1]
            self.path_length += math.hypot(dx, dy)
        self.last_position = (px, py)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_vel = (vx, vy)
        self.vel_samples.append((vx, vy, msg.twist.twist.angular.z))

    def _cmd_cb(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def _gt_cb(self, msg):
        self.gt_people = [(p.position.x, p.position.y) for p in msg.poses]
        self.person_count = max(self.person_count, len(msg.poses))

    def _yolo_cb(self, msg):
        self.yolo_count = len(msg.detections)

    def _fused_cb(self, msg):
        self.fused_obs_count = len(msg.markers)

    def _risk_cb(self, msg):
        self.high_risk_count = sum(
            1 for m in msg.markers
            if hasattr(m, 'ns') and 'risk' in m.ns and m.color.r >= 0.9 and m.color.g < 0.2)

    def _goal_cb(self, msg):
        pass

    def _success_cb(self, msg):
        self.goal_reached = True
        self.is_success = msg.data

    def _log(self):
        now = self.get_clock().now()
        nav_time = (now - self.start_time).nanoseconds * 1e-9

        rx, ry, ryaw = self.robot_pose
        dist_to_goal = math.hypot(self.goal_x - rx, self.goal_y - ry)

        # Min distance to person
        min_dist = compute_min_distance_to_person(self.robot_pose[:2], self.gt_people)

        # Collision / dangerous close
        if min_dist < self.collision_d and not self.prev_collision:
            self.collision_count += 1
            self.prev_collision = True
        elif min_dist >= self.collision_d:
            self.prev_collision = False

        if min_dist < self.dangerous_close_d and not self.prev_dangerous:
            self.dangerous_close_count += 1
            self.prev_dangerous = True
        elif min_dist >= self.dangerous_close_d:
            self.prev_dangerous = False

        # Stuck detection
        speed = math.hypot(self.robot_vel[0], self.robot_vel[1])
        if speed < self.stuck_vel and not self.goal_reached:
            if self.stuck_start is None:
                self.stuck_start = now
            elif (now - self.stuck_start).nanoseconds * 1e-9 > self.stuck_dur:
                self.failure_reason = 'stuck'
                self.is_success = False
        else:
            self.stuck_start = None

        # Goal check
        if dist_to_goal < self.goal_tol:
            self.goal_reached = True
            self.is_success = True

        # Average speed
        avg_speed = float(sum(math.hypot(v[0], v[1]) for v in self.vel_samples[-50:]) /
                          max(len(self.vel_samples[-50:]), 1))

        # Write row
        t = now.to_msg()
        t_float = t.sec + t.nanosec * 1e-9
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                f'{t_float:.3f}', self.scenario, self.seed, self.mode, self.person_count,
                f'{rx:.4f}', f'{ry:.4f}', f'{ryaw:.4f}',
                f'{self.goal_x:.2f}', f'{self.goal_y:.2f}',
                f'{dist_to_goal:.4f}', f'{self.cmd_linear:.4f}', f'{self.cmd_angular:.4f}',
                f'{min_dist:.4f}', self.collision_count, self.dangerous_close_count,
                self.yolo_count, self.yolo_miss_count,
                f'{self.detector_fps:.1f}', f'{self.detector_latency_ms:.1f}',
                self.fused_obs_count, self.high_risk_count,
                self.stop_count, self.replan_count,
                f'{self.path_length:.4f}',
                f'{avg_speed:.4f}', f'{nav_time:.2f}',
                1 if self.is_success else 0, self.failure_reason,
            ])

    def destroy_node(self):
        self._write_summary()
        self.get_logger().info(f'Metrics saved to {self.csv_path}')
        super().destroy_node()

    def _write_summary(self):
        summary_path = self.csv_path.replace('.csv', '_summary.csv')
        try:
            # Read back the CSV and compute summary
            import pandas as pd
            df = pd.read_csv(self.csv_path)
            summary = {
                'scenario': self.scenario,
                'mode': self.mode,
                'seed': self.seed,
                'success': int(self.is_success),
                'failure_reason': self.failure_reason,
                'navigation_time_s': (self.get_clock().now() - self.start_time).nanoseconds * 1e-9,
                'path_length_m': self.path_length,
                'collision_count': self.collision_count,
                'dangerous_close_count': self.dangerous_close_count,
                'total_stop_count': self.stop_count,
                'git_commit': _get_git_commit(),
            }
            with open(summary_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=summary.keys())
                w.writeheader()
                w.writerow(summary)
        except ImportError:
            self.get_logger().warn('pandas not available, skipping summary CSV')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
