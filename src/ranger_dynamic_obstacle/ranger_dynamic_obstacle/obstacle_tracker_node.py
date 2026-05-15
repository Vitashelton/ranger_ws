#!/usr/bin/env python3
"""
Multi-object Kalman filter tracker for fused obstacles.

Input:  /fused_obstacles (MarkerArray)
Output: /tracked_obstacles (MarkerArray) — with persistent IDs and velocity estimates

State per track: [px, py, vx, vy]^T
Measurement:     [px, py]^T (from fused obstacle centroid)
Model:           constant velocity with Gaussian noise
Association:     Hungarian algorithm on Euclidean distance
"""
import math
import time

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def _euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class KalmanTrack:
    """4-state Kalman filter for 2D obstacle tracking."""
    __slots__ = ('id', 'x', 'P', 'F', 'H', 'Q', 'R',
                 'age', 'missed', 'label', 'color')

    def __init__(self, track_id, px, py, dt=0.1,
                 process_noise=0.1, meas_noise=0.05):
        self.id = track_id
        self.x = np.array([px, py, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4) * 0.5
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1,  0],
                           [0, 0, 0,  1]], dtype=np.float64)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        q = process_noise
        self.Q = np.eye(4) * q
        self.R = np.eye(2) * meas_noise
        self.age = 0
        self.missed = 0
        self.label = ''
        self.color = (1.0, 1.0, 1.0)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1

    def update(self, z):
        """z = [px_meas, py_meas]"""
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.missed = 0

    @property
    def position(self):
        return self.x[0:2]

    @property
    def velocity(self):
        return self.x[2:4]

    @property
    def speed(self):
        return float(np.linalg.norm(self.x[2:4]))


class ObstacleTrackerNode(Node):
    def __init__(self):
        super().__init__('obstacle_tracker_node')

        # --- Params ---
        self.declare_parameter('association_max_dist', 0.5)
        self.declare_parameter('birth_threshold', 3)
        self.declare_parameter('death_threshold', 5)
        self.declare_parameter('confirmed_threshold', 10)
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.05)
        self.declare_parameter('max_tracks', 100)
        self.declare_parameter('frame_id', 'base_link')
        self._load_params()

        # Track list
        self.tracks = []
        self.next_id = 0
        self.last_time = self.get_clock().now()

        # Color palette for tracks
        self.colors = [
            (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0),
            (1.0, 0.5, 0.0), (0.5, 0.0, 1.0), (0.0, 0.5, 1.0),
            (1.0, 0.8, 0.0),
        ]

        self.sub = self.create_subscription(
            MarkerArray, '/fused_obstacles', self._callback, 10)
        self.pub = self.create_publisher(MarkerArray, '/tracked_obstacles', 10)

        self.get_logger().info('obstacle_tracker_node started')

    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.assoc_dist = p('association_max_dist')
        self.birth_thresh = p('birth_threshold')
        self.death_thresh = p('death_threshold')
        self.confirmed_thresh = p('confirmed_threshold')
        self.proc_noise = p('process_noise')
        self.meas_noise = p('measurement_noise')
        self.max_tracks = p('max_tracks')
        self.frame = p('frame_id')

    def _callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            dt = 0.1
        self.last_time = now

        # Update transition matrix with current dt
        for t in self.tracks:
            t.F[0, 2] = dt
            t.F[1, 3] = dt

        # Extract measurements
        measurements = []
        for m in msg.markers:
            z = np.array([m.pose.position.x, m.pose.position.y])
            measurements.append(z)

        # 1. Predict all tracks
        for t in self.tracks:
            t.predict()

        # 2. Association via Hungarian
        N_tracks = len(self.tracks)
        N_meas = len(measurements)

        if N_tracks > 0 and N_meas > 0:
            cost = np.full((N_tracks, N_meas), 1e9)
            for i, t in enumerate(self.tracks):
                for j, z in enumerate(measurements):
                    d = float(np.linalg.norm(t.position - z))
                    if d < self.assoc_dist:
                        cost[i, j] = d
            row_idx, col_idx = linear_sum_assignment(cost)
        else:
            row_idx, col_idx = [], []

        matched_tracks = set()
        matched_meas = set()
        for ri, ci in zip(row_idx, col_idx):
            if cost[ri, ci] < 1e8:
                self.tracks[ri].update(measurements[ci])
                matched_tracks.add(ri)
                matched_meas.add(ci)

        # 3. Unmatched measurements → init potential new tracks
        for j in range(N_meas):
            if j not in matched_meas:
                z = measurements[j]
                # Tentative track (will be pruned if not confirmed)
                new_t = KalmanTrack(self.next_id, z[0], z[1], dt,
                                    self.proc_noise, self.meas_noise)
                new_t.label = f'track_{self.next_id}'
                color = self.colors[self.next_id % len(self.colors)]
                new_t.color = color
                self.tracks.append(new_t)
                self.next_id += 1

        # 4. Mark unmatched tracks
        for i in range(N_tracks):
            if i not in matched_tracks:
                self.tracks[i].missed += 1

        # 5. Prune dead tracks
        self.tracks = [t for t in self.tracks
                       if t.missed < self.death_thresh
                       and (t.age < self.birth_thresh or t.missed < self.death_thresh)]

        # Actually: keep tracks with birth ≤ confirmed or already confirmed
        # Remove: tracks that died (missed >= death_thresh)
        survived = []
        for t in self.tracks:
            if t.age >= self.birth_thresh and t.missed >= self.death_thresh:
                continue  # confirmed but stale → delete
            if t.age < self.birth_thresh and t.missed > 0:
                continue  # tentative and missed → delete
            survived.append(t)
        self.tracks = survived[:self.max_tracks]

        # 6. Publish
        self.pub.publish(self._tracks_to_markers(msg.markers[0].header.stamp if msg.markers else now.to_msg()))

    def _tracks_to_markers(self, stamp):
        markers = MarkerArray()
        for t in self.tracks:
            if t.age < self.birth_thresh:
                continue  # don't show tentative tracks

            px, py = float(t.x[0]), float(t.x[1])
            vx, vy = float(t.x[2]), float(t.x[3])

            # Centroid marker
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = stamp
            m.ns = 'tracked'
            m.id = t.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.3
            q = _euler_to_quaternion(0.0, 0.0, 0.0)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x = m.scale.y = 0.25
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b = t.color
            m.color.a = 0.8
            markers.markers.append(m)

            # Velocity arrow
            speed = t.speed
            if speed > 0.05:
                arr = Marker()
                arr.header.frame_id = self.frame
                arr.header.stamp = stamp
                arr.ns = 'tracked_vel'
                arr.id = t.id + 10000
                arr.type = Marker.ARROW
                arr.action = Marker.ADD
                arr.pose.position.x = px
                arr.pose.position.y = py
                arr.pose.position.z = 0.3
                # Arrow orientation: point along velocity direction
                yaw = math.atan2(vy, vx)
                q = _euler_to_quaternion(0.0, 0.0, yaw)
                arr.pose.orientation.x, arr.pose.orientation.y, arr.pose.orientation.z, arr.pose.orientation.w = q
                arr.scale.x = speed * 1.0
                arr.scale.y = 0.05
                arr.scale.z = 0.05
                arr.color.r, arr.color.g, arr.color.b = t.color
                arr.color.a = 0.6
                markers.markers.append(arr)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
