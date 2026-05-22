#!/usr/bin/env python3
"""
MID360S pointcloud obstacle clustering node.
Input:  /cloud_registered (PointCloud2, from FAST-LIO)
Output: /obstacles_mid360 (MarkerArray)


Pipeline: ROI filter -> voxel downsample -> ground removal -> Euclidean clustering.

No deep learning. All parameters configurable via ROS params.
"""
import math
import time

import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def _pointcloud2_to_xyz(pc_msg):
    """Read PointCloud2 as normal Nx3 float32 array."""
    field_names = {f.name for f in pc_msg.fields}
    if not {'x', 'y', 'z'}.issubset(field_names):
        return np.empty((0, 3), dtype=np.float32)

    try:
        pts = point_cloud2.read_points_numpy(
            pc_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )
        pts = np.asarray(pts, dtype=np.float32)
        pts = pts.reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        return pts
    except Exception:
        pts = point_cloud2.read_points(
            pc_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        if isinstance(pts, np.ndarray):
            if pts.dtype.names is not None:
                arr = np.column_stack((pts['x'], pts['y'], pts['z'])).astype(np.float32)
            else:
                arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            arr = arr[np.isfinite(arr).all(axis=1)]
            return arr

        data = []
        for p in pts:
            data.append((float(p[0]), float(p[1]), float(p[2])))

        if len(data) == 0:
            return np.empty((0, 3), dtype=np.float32)

        arr = np.asarray(data, dtype=np.float32)
        arr = arr[np.isfinite(arr).all(axis=1)]
        return arr



def _euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ObstacleClusterNode(Node):
    def __init__(self):
        super().__init__('obstacle_cluster_node')

        # --- Params ---
        self.declare_parameter('roi_x_min', -5.0)
        self.declare_parameter('roi_x_max', 15.0)
        self.declare_parameter('roi_y_min', -8.0)
        self.declare_parameter('roi_y_max', 8.0)
        self.declare_parameter('roi_z_min', -1.0)
        self.declare_parameter('roi_z_max', 2.0)
        self.declare_parameter('voxel_leaf_size', 0.1)
        self.declare_parameter('ground_ransac_dist_thresh', 0.05)
        self.declare_parameter('cluster_tolerance', 0.15)
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('max_obstacles', 50)
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('frame_id', 'camera_init')


        self._load_params()

        # --- Sub / Pub ---
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/obstacles_mid360', 10)
        self.get_logger().info(f'obstacle_cluster_node started, input={self.input_topic}, frame={self.frame}')

    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.input_topic = p('input_topic')
        self.roi_x = (p('roi_x_min'), p('roi_x_max'))
        self.roi_y = (p('roi_y_min'), p('roi_y_max'))
        self.roi_z = (p('roi_z_min'), p('roi_z_max'))
        self.voxel_size = p('voxel_leaf_size')
        self.ground_thresh = p('ground_ransac_dist_thresh')
        self.cluster_tol = p('cluster_tolerance')
        self.min_cluster = p('min_cluster_size')
        self.max_cluster = p('max_cluster_size')
        self.max_obs = p('max_obstacles')
        self.frame = p('frame_id')

    def _callback(self, msg):
        t0 = time.time()

        pts = _pointcloud2_to_xyz(msg)
        if len(pts) == 0:
            self.pub.publish(MarkerArray())
            return

        # 1. ROI filter
        mask = (
            (pts[:, 0] >= self.roi_x[0]) & (pts[:, 0] <= self.roi_x[1]) &
            (pts[:, 1] >= self.roi_y[0]) & (pts[:, 1] <= self.roi_y[1]) &
            (pts[:, 2] >= self.roi_z[0]) & (pts[:, 2] <= self.roi_z[1])
        )
        pts = pts[mask]
        if len(pts) < self.min_cluster:
            self.pub.publish(MarkerArray())
            return

        # 2. Voxel downsample
        pts = self._voxel_downsample(pts)

        # 3. Ground removal (simple RANSAC plane)
        # pts = self._remove_ground(pts)

        if len(pts) < self.min_cluster:
            self.pub.publish(MarkerArray())
            return

        # 4. Euclidean clustering (KDTree + BFS)
        clusters = self._euclidean_cluster(pts)

        # 5. Publish
        markers = self._clusters_to_markers(clusters, msg.header.stamp)
        self.pub.publish(markers)
        dt = (time.time() - t0) * 1000.0
        self.get_logger().debug(f'Processed {len(pts)} pts -> {len(clusters)} clusters in {dt:.1f} ms')

    def _voxel_downsample(self, pts):
        """Simple voxel grid: keep one point per occupied voxel."""
        if self.voxel_size <= 0:
            return pts

        scaled = np.floor(pts / self.voxel_size).astype(np.int64)

        _, idx = np.unique(
            scaled,
            axis=0,
            return_index=True
        )

        return pts[idx]

    def _remove_ground(self, pts):
        """Simple RANSAC ground plane removal. Returns non-ground points."""
        if len(pts) < 10:
            return pts
        best_inliers = 0
        best_mask = np.ones(len(pts), dtype=bool)
        target_z = 0.0  # expect ground at z=0
        for _ in range(30):
            # Sample 3 random points
            idx = np.random.choice(len(pts), 3, replace=False)
            # Fit plane z = ax + by + c (assuming nearly horizontal)
            # Simplified: just check if points are within threshold of z=target
            # (Ground is approximately horizontal in base_link frame)
            dist = np.abs(pts[idx, 2] - target_z)
            if np.all(dist < self.ground_thresh * 3):
                # All 3 near ground -> treat as ground candidate
                dz = np.abs(pts[:, 2] - target_z)
                inliers = np.sum(dz < self.ground_thresh)
                if inliers > best_inliers:
                    best_inliers = inliers
                    best_mask = dz >= self.ground_thresh
        if best_inliers > len(pts) * 0.3:
            return pts[best_mask]
        return pts  # not enough ground -> keep all

    def _euclidean_cluster(self, pts):
        """KDTree-based 3D Euclidean clustering."""
        tree = KDTree(pts)
        visited = np.zeros(len(pts), dtype=bool)
        clusters = []

        for i in range(len(pts)):
            if visited[i]:
                continue

            queue = [i]
            visited[i] = True
            cluster_pts = []

            while queue:
                idx = queue.pop()
                cluster_pts.append(idx)

                neighbors = tree.query_ball_point(pts[idx], self.cluster_tol)
                for nb in neighbors:
                    if not visited[nb]:
                        visited[nb] = True
                        queue.append(nb)

            if self.min_cluster <= len(cluster_pts) <= self.max_cluster:
                clusters.append(pts[cluster_pts])

            if len(clusters) >= self.max_obs:
                break

        return clusters


    def _clusters_to_markers(self, clusters, stamp):
        markers = MarkerArray()
        for i, c in enumerate(clusters):
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            cz = float(np.mean(c[:, 2]))

            w = float(np.ptp(c[:, 0]) + 0.1)
            d = float(np.ptp(c[:, 1]) + 0.1)
            h = float(np.ptp(c[:, 2]) + 0.1)

            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns = 'mid360_obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = _euler_to_quaternion(0.0, 0.0, 0.0)
            m.scale.x = max(w, 0.15)
            m.scale.y = max(d, 0.15)
            m.scale.z = max(h, 0.15)
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.7
            markers.markers.append(m)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
