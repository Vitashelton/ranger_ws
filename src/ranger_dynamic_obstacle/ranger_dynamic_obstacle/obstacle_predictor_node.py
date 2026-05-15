#!/usr/bin/env python3
"""
Short-term trajectory prediction for tracked obstacles.

Input:  /tracked_obstacles (MarkerArray)
Output: /predicted_obstacles (MarkerArray) — LINE_STRIP of future positions

Method: constant-velocity linear extrapolation with growing uncertainty.
Prediction horizon: 2.0s at 0.2s steps.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ObstaclePredictorNode(Node):
    def __init__(self):
        super().__init__('obstacle_predictor_node')

        self.declare_parameter('prediction_horizon', 2.0)
        self.declare_parameter('prediction_step', 0.2)
        self.declare_parameter('min_speed_for_prediction', 0.05)
        self.declare_parameter('frame_id', 'base_link')
        self._load_params()

        self.sub = self.create_subscription(
            MarkerArray, '/tracked_obstacles', self._callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/predicted_obstacles', 10)

        self.get_logger().info('obstacle_predictor_node started')

    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.horizon = p('prediction_horizon')
        self.step = p('prediction_step')
        self.min_speed = p('min_speed_for_prediction')
        self.frame = p('frame_id')

    def _callback(self, msg):
        if not msg.markers:
            self.pub.publish(MarkerArray())
            return

        stamp = msg.markers[0].header.stamp
        markers = MarkerArray()

        # Each tracked obstacle has 2 markers: centroid (ns='tracked') + velocity arrow (ns='tracked_vel')
        # Group by obstacle ID
        centroids = {}
        velocities = {}
        for m in msg.markers:
            if m.ns == 'tracked':
                centroids[m.id] = (m.pose.position.x, m.pose.position.y)
            elif m.ns == 'tracked_vel':
                # Velocity direction from orientation, magnitude from scale
                qw = m.pose.orientation.w
                qz = m.pose.orientation.z
                yaw = 2.0 * math.atan2(qz, qw) if abs(qw) > 1e-9 else 0.0
                v_mag = m.scale.x
                vx = v_mag * math.cos(yaw)
                vy = v_mag * math.sin(yaw)
                velocities[m.id] = (vx, vy)

        for obs_id, (px, py) in centroids.items():
            vx, vy = velocities.get(obs_id, (0.0, 0.0))
            speed = math.hypot(vx, vy)

            if speed < self.min_speed:
                continue

            # Build prediction LINE_STRIP
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns = 'predicted'
            m.id = obs_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0
            m.color.a = 0.8

            steps = int(self.horizon / self.step)
            for k in range(steps + 1):
                t = k * self.step
                pred_x = px + vx * t
                pred_y = py + vy * t
                m.points.append(Point(x=pred_x, y=pred_y, z=0.2))

            # Uncertainty ellipses at 1.0s and 2.0s
            sigma_v = 0.2 * speed
            for t_mark in [1.0, 2.0]:
                if t_mark > self.horizon:
                    continue
                sigma_pos = sigma_v * t_mark
                ell = Marker()
                ell.header.frame_id = self.frame
                ell.header.stamp = stamp
                ell.ns = 'predicted_uncertainty'
                ell.id = obs_id * 100 + int(t_mark * 10)
                ell.type = Marker.SPHERE
                ell.action = Marker.ADD
                ell.pose.position.x = px + vx * t_mark
                ell.pose.position.y = py + vy * t_mark
                ell.pose.position.z = 0.2
                ell.scale.x = ell.scale.y = sigma_pos * 2.0
                ell.scale.z = 0.05
                ell.color.r, ell.color.g, ell.color.b = 1.0, 0.5, 0.0
                ell.color.a = 0.15
                markers.markers.append(ell)

            markers.markers.append(m)

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
