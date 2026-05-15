#!/usr/bin/env python3
"""
TTC-based risk evaluator for dynamic obstacles.

Inputs:  /tracked_obstacles (MarkerArray)
         /predicted_obstacles (MarkerArray)
         /odom (Odometry)
Output:  /risk_markers (MarkerArray)

Computes Time-To-Collision between robot and each tracked obstacle.
Risk levels:
  TTC > 3.0s  → LOW (green)
  1.5-3.0s     → MEDIUM (yellow)
  0.5-1.5s     → HIGH (orange)
  < 0.5s       → CRITICAL (red)

Risk cost for trajectory evaluation:
  J_ttc = Σ 1/(TTC + ε)  for all obstacles with TTC < T_thresh
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class RiskEvaluatorNode(Node):
    def __init__(self):
        super().__init__('risk_evaluator_node')

        # --- Params ---
        self.declare_parameter('ttc_threshold_low', 3.0)
        self.declare_parameter('ttc_threshold_medium', 1.5)
        self.declare_parameter('ttc_threshold_high', 0.5)
        self.declare_parameter('min_distance_threshold', 0.5)
        self.declare_parameter('collision_distance', 0.35)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('frame_id', 'base_link')
        self._load_params()

        # State
        self.tracked_obs = {}    # id -> (px, py, vx, vy)
        self.predicted_obs = {}  # id -> [(px, py), ...]
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_wz = 0.0

        # Subs
        self.tracked_sub = self.create_subscription(
            MarkerArray, '/tracked_obstacles', self._tracked_cb, 10)
        self.predicted_sub = self.create_subscription(
            MarkerArray, '/predicted_obstacles', self._predicted_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # Pub
        self.pub = self.create_publisher(MarkerArray, '/risk_markers', 10)

        # Timer
        self.timer = self.create_timer(0.1, self._evaluate)

        self.get_logger().info('risk_evaluator_node started')

    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.ttc_low = p('ttc_threshold_low')
        self.ttc_med = p('ttc_threshold_medium')
        self.ttc_high = p('ttc_threshold_high')
        self.min_dist_thresh = p('min_distance_threshold')
        self.collision_dist = p('collision_distance')
        self.robot_r = p('robot_radius')
        self.frame = p('frame_id')

    def _tracked_cb(self, msg):
        self.tracked_obs = {}
        for m in msg.markers:
            if m.ns == 'tracked':
                self.tracked_obs[m.id] = {
                    'px': m.pose.position.x,
                    'py': m.pose.position.y,
                }
            elif m.ns == 'tracked_vel':
                # velocity arrow: orientation = yaw, scale.x = speed
                qw = m.pose.orientation.w
                qz = m.pose.orientation.z
                yaw = 2.0 * math.atan2(qz, qw) if abs(qw) > 1e-9 else 0.0
                v_mag = m.scale.x
                if m.id in self.tracked_obs:
                    self.tracked_obs[m.id]['vx'] = v_mag * math.cos(yaw)
                    self.tracked_obs[m.id]['vy'] = v_mag * math.sin(yaw)

    def _predicted_cb(self, msg):
        self.predicted_obs = {}
        for m in msg.markers:
            if m.ns == 'predicted':
                pts = [(p.x, p.y) for p in m.points]
                self.predicted_obs[m.id] = pts

    def _odom_cb(self, msg):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_vy = msg.twist.twist.linear.y
        self.robot_wz = msg.twist.twist.angular.z

    def _evaluate(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for obs_id, obs in self.tracked_obs.items():
            px = obs.get('px', 0.0)
            py = obs.get('py', 0.0)
            vx = obs.get('vx', 0.0)
            vy = obs.get('vy', 0.0)

            # Distance from robot (robot at origin in base_link)
            dist = math.hypot(px, py)

            # Relative velocity (closing speed along line-of-sight)
            if dist > 1e-6:
                los_x = px / dist
                los_y = py / dist
                rel_vx = vx - self.robot_vx
                rel_vy = vy - self.robot_vy
                closing_speed = -(rel_vx * los_x + rel_vy * los_y)
            else:
                closing_speed = math.hypot(vx - self.robot_vx, vy - self.robot_vy)

            # TTC
            effective_dist = max(dist - self.robot_r - self.collision_dist, 0.01)
            ttc = effective_dist / max(closing_speed, 0.01)

            if ttc > 10.0:
                ttc = 10.0  # cap for visualization

            # Risk level
            if ttc <= self.ttc_high or dist <= self.collision_dist:
                r, g, b = 1.0, 0.0, 0.0  # red
                level = 'CRITICAL'
            elif ttc <= self.ttc_med:
                r, g, b = 1.0, 0.5, 0.0  # orange
                level = 'HIGH'
            elif ttc <= self.ttc_low:
                r, g, b = 1.0, 1.0, 0.0  # yellow
                level = 'MEDIUM'
            else:
                r, g, b = 0.0, 1.0, 0.0  # green
                level = 'LOW'

            # Risk sphere around obstacle
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = now
            m.ns = 'risk'
            m.id = obs_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.2
            m.scale.x = m.scale.y = m.scale.z = max(self.collision_dist * 2.0, 0.7)
            m.color.r, m.color.g, m.color.b = r, g, b
            m.color.a = 0.3
            markers.markers.append(m)

            # TTC text marker (use LINE_STRIP from obstacle to center as visual indicator)
            # Also draw a line from robot origin to obstacle
            line = Marker()
            line.header.frame_id = self.frame
            line.header.stamp = now
            line.ns = 'risk_line'
            line.id = obs_id + 10000
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02
            line.color.r, line.color.g, line.color.b = r, g, b
            line.color.a = 0.6
            line.points.append(Point(x=0.0, y=0.0, z=0.1))
            line.points.append(Point(x=px, y=py, z=0.1))
            markers.markers.append(line)

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RiskEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
