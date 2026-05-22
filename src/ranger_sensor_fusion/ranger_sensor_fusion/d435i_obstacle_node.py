#!/usr/bin/env python3
"""
D435i depth-based near-field obstacle detection node.

Input:  /camera/depth/color/points (PointCloud2)
Output: /obstacles_d435i (MarkerArray), /near_field_safety_zone (Marker)

Focus: near-field (0.2-4m), low obstacles (0-1.5m height).
Provides blind-zone coverage for MID360S and low-obstacle detection.
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Polygon

# Re-use pointcloud helper
from ranger_sensor_fusion.obstacle_cluster_node import (
    _pointcloud2_to_xyz,
    _euler_to_quaternion,
)


class D435iObstacleNode(Node):
    def __init__(self):
        super().__init__('d435i_obstacle_node')

        # --- Params ---
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('min_height', 0.0)
        self.declare_parameter('max_height', 1.5)
        self.declare_parameter('cluster_tolerance', 0.08)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('low_obstacle_threshold', 0.3)
        self.declare_parameter('safety_zone_x_min', 0.1)
        self.declare_parameter('safety_zone_x_max', 1.0)
        self.declare_parameter('safety_zone_y_half_width', 0.4)
        self.declare_parameter('safety_critical_range', 0.3)
        self.declare_parameter('max_obstacles', 30)
        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('camera_optical_to_robot_frame', True)


        self._load_params()

        # --- Sub / Pub ---
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._callback, 10)
        self.obs_pub = self.create_publisher(MarkerArray, '/obstacles_d435i', 10)
        self.zone_pub = self.create_publisher(Marker, '/near_field_safety_zone', 10)
        self.get_logger().info(f'd435i_obstacle_node started, input={self.input_topic}, frame={self.frame}')


    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.input_topic = p('input_topic')
        self.optical_to_robot = p('camera_optical_to_robot_frame')
        self.max_range = p('max_range')
        self.min_range = p('min_range')
        self.min_h = p('min_height')
        self.max_h = p('max_height')
        self.cluster_tol = p('cluster_tolerance')
        self.min_cluster = p('min_cluster_size')
        self.max_cluster = p('max_cluster_size')
        self.low_thresh = p('low_obstacle_threshold')
        self.sz_x = (p('safety_zone_x_min'), p('safety_zone_x_max'))
        self.sz_y_half = p('safety_zone_y_half_width')
        self.critical_range = p('safety_critical_range')
        self.max_obs = p('max_obstacles')
        self.frame = p('frame_id')

    def _callback(self, msg):
        t0 = time.time()

        pts = _pointcloud2_to_xyz(msg)
        if len(pts) == 0:
            self.obs_pub.publish(MarkerArray())
            return
        # D435i optical frame: x right, y down, z forward
        # Robot frame: x forward, y left, z up
        if self.optical_to_robot:
            x = pts[:, 2].copy()
            y = -pts[:, 0].copy()
            z = -pts[:, 1].copy()
            pts = np.stack((x, y, z), axis=1)

        # 1. ROI filter: range + height
        r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2 + pts[:, 2]**2)
        mask = (
            (r >= self.min_range) & (r <= self.max_range) &
            (pts[:, 2] >= self.min_h) & (pts[:, 2] <= self.max_h)
        )
        pts = pts[mask]
        if len(pts) < self.min_cluster:
            self.obs_pub.publish(MarkerArray())
            return

        # 2. Simple clustering (KDTree from obstacle_cluster_node)
        from scipy.spatial import KDTree
        tree = KDTree(pts[:, :2])
        visited = np.zeros(len(pts), dtype=bool)
        clusters = []

        for i in range(len(pts)):
            if visited[i]:
                continue
            queue = [i]
            visited[i] = True
            cluster_idx = []
            while queue:
                idx = queue.pop()
                cluster_idx.append(idx)
                neighbors = tree.query_ball_point(pts[idx, :2], self.cluster_tol)
                for nb in neighbors:
                    if not visited[nb]:
                        visited[nb] = True
                        queue.append(nb)
            if self.min_cluster <= len(cluster_idx) <= self.max_cluster:
                clusters.append(pts[cluster_idx])
            if len(clusters) >= self.max_obs:
                break

        # 3. Build markers
        stamp = msg.header.stamp
        obstacles, has_critical = self._clusters_to_obstacles(clusters, stamp)
        self.obs_pub.publish(obstacles)

        # 4. Publish near-field safety zone
        self.zone_pub.publish(self._build_safety_zone(stamp, has_critical))

        dt = (time.time() - t0) * 1000.0
        self.get_logger().debug(f'D435i: {len(clusters)} clusters in {dt:.1f} ms')

    def _clusters_to_obstacles(self, clusters, stamp):
        markers = MarkerArray()
        has_critical = False
        for i, c in enumerate(clusters):
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            cz = float(np.mean(c[:, 2]))
            h = float(np.ptp(c[:, 2]) + 0.05)

            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns = 'd435i_obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz
            q = _euler_to_quaternion(0.0, 0.0, 0.0)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = max(h, 0.05)

            is_low = h < self.low_thresh
            is_critical = (
                self.sz_x[0] <= cx <= self.sz_x[1] and
                abs(cy) <= self.sz_y_half and
                np.sqrt(cx**2 + cy**2) < self.critical_range
            )

            if is_critical:
                has_critical = True
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.9
            elif is_low:
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 0.8
            else:
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0
                m.color.a = 0.7
            markers.markers.append(m)
        return markers, has_critical

    def _build_safety_zone(self, stamp, has_critical):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.ns = 'safety_zone'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03

        # Draw polygon: (x_min, -half), (x_min, +half), (x_max, +half), (x_max, -half), close
        pts = [
            (self.sz_x[0], -self.sz_y_half, 0.1),
            (self.sz_x[0], self.sz_y_half, 0.1),
            (self.sz_x[1], self.sz_y_half, 0.1),
            (self.sz_x[1], -self.sz_y_half, 0.1),
            (self.sz_x[0], -self.sz_y_half, 0.1),
        ]
        for px, py, pz in pts:
            p = Point(x=px, y=py, z=pz)
            m.points.append(p)

        if has_critical:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.4
        return m


def main(args=None):
    rclpy.init(args=args)
    node = D435iObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
