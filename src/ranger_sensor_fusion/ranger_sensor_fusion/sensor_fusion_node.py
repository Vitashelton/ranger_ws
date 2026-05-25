#!/usr/bin/env python3
"""
Multi-sensor obstacle fusion node.

Inputs:  /obstacles_mid360 (MarkerArray), /obstacles_d435i (MarkerArray)
Outputs: /fused_obstacles (MarkerArray)
         /risk_markers (MarkerArray) — if risk_enabled

Fuses MID360S (long-range, full-surround) and D435i (near-field, forward-only)
obstacles via Hungarian assignment + confidence-weighted merging.
"""
import math
import time

import numpy as np
from scipy.optimize import linear_sum_assignment
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


def _euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # --- Params ---
        self.declare_parameter('association_max_dist', 0.5)
        self.declare_parameter('max_timestamp_diff', 0.1)
        self.declare_parameter('mid360_base_confidence', 0.85)
        self.declare_parameter('mid360_range_decay_start', 10.0)
        self.declare_parameter('mid360_range_decay_end', 20.0)
        self.declare_parameter('d435i_base_confidence', 0.8)
        self.declare_parameter('d435i_range_decay_start', 1.0)
        self.declare_parameter('d435i_range_decay_end', 3.0)
        self.declare_parameter('dual_detection_confidence', 0.95)
        self.declare_parameter('temporal_consistency_bonus', 0.05)
        self.declare_parameter('min_confidence_threshold', 0.3)
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('risk_enabled', True)
        self.declare_parameter('mid360_obstacles_topic', '/obstacles_mid360')
        self.declare_parameter('d435i_obstacles_topic', '/obstacles_d435i')
        self.declare_parameter('fused_obstacles_topic', '/fused_obstacles')
        self.declare_parameter('risk_markers_topic', '/risk_markers')
        self.declare_parameter('yolo_obstacles_topic', '')
        self.declare_parameter('yolo_base_confidence', 0.75)

        self._load_params()

        # Buffers for latest messages from each sensor
        self.mid360_obs = None
        self.mid360_stamp = None
        self.d435i_obs = None
        self.yolo_obs = None
        self.yolo_stamp = None
        self.d435i_stamp = None

        # Subscribers
        self.mid360_sub = self.create_subscription(
            MarkerArray, self.mid360_topic, self._mid360_cb, 10)

        self.d435i_sub = self.create_subscription(
            MarkerArray, self.d435i_topic, self._d435i_cb, 10)

        self.yolo_sub = None
        if self.yolo_topic:
            self.yolo_sub = self.create_subscription(
                MarkerArray, self.yolo_topic, self._yolo_cb, 10)
            self.get_logger().info(f'YOLO fusion enabled: subscribing to {self.yolo_topic}')

        self.fused_pub = self.create_publisher(
            MarkerArray, self.fused_topic, 10)

        self.risk_pub = self.create_publisher(
            MarkerArray, self.risk_topic, 10)

        # Publishers
        self.fused_pub = self.create_publisher(MarkerArray, self.fused_topic, 10)
        self.risk_pub = self.create_publisher(MarkerArray, self.risk_topic, 10)


        # Timer: fuse on fixed rate (handles sensor rate mismatch)
        self.timer = self.create_timer(0.1, self._fuse_callback)

        self.get_logger().info(
            f'sensor_fusion_node started: {self.mid360_topic} + {self.d435i_topic} -> {self.fused_topic}, frame={self.frame}'
        )


    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.assoc_dist = p('association_max_dist')
        self.max_ts_diff = p('max_timestamp_diff')
        self.conf_mid_base = p('mid360_base_confidence')
        self.conf_mid_decay_s = p('mid360_range_decay_start')
        self.conf_mid_decay_e = p('mid360_range_decay_end')
        self.conf_d435i_base = p('d435i_base_confidence')
        self.conf_d435i_decay_s = p('d435i_range_decay_start')
        self.conf_d435i_decay_e = p('d435i_range_decay_end')
        self.conf_dual = p('dual_detection_confidence')
        self.conf_temporal_bonus = p('temporal_consistency_bonus')
        self.conf_min = p('min_confidence_threshold')
        self.frame = p('frame_id')
        self.risk_enabled = p('risk_enabled')
        self.frame = p('frame_id')
        self.risk_enabled = p('risk_enabled')
        self.mid360_topic = p('mid360_obstacles_topic')
        self.d435i_topic = p('d435i_obstacles_topic')
        self.fused_topic = p('fused_obstacles_topic')
        self.risk_topic = p('risk_markers_topic')
        self.yolo_topic = p('yolo_obstacles_topic')
        self.conf_yolo_base = p('yolo_base_confidence')


    def _mid360_cb(self, msg):
        self.mid360_obs = msg.markers
        self.mid360_stamp = msg.markers[0].header.stamp if msg.markers else self.get_clock().now()

    def _d435i_cb(self, msg):
        self.d435i_obs = msg.markers
        self.d435i_stamp = msg.markers[0].header.stamp if msg.markers else self.get_clock().now()

    def _yolo_cb(self, msg):
        self.yolo_obs = msg.markers
        self.yolo_stamp = msg.markers[0].header.stamp if msg.markers else self.get_clock().now()

    def _fuse_callback(self):
        mid = self.mid360_obs or []
        d435 = self.d435i_obs or []
        yolo = self.yolo_obs or []

        if not mid and not d435 and not yolo:
            self.fused_pub.publish(MarkerArray())
            if self.risk_enabled:
                self.risk_pub.publish(MarkerArray())
            return

        now = self.get_clock().now()

        # 1. Compute confidence for each obstacle
        mid_conf = [self._mid360_confidence(m) for m in mid]
        d435_conf = [self._d435i_confidence(m) for m in d435]
        yolo_conf = [self._yolo_confidence(m) for m in yolo]

        # 2. Fuse MID360 + D435i via Hungarian
        N, M = len(mid), len(d435)
        if N > 0 and M > 0:
            cost = np.full((N, M), 1e9)
            for i in range(N):
                for j in range(M):
                    dx = mid[i].pose.position.x - d435[j].pose.position.x
                    dy = mid[i].pose.position.y - d435[j].pose.position.y
                    dist = math.hypot(dx, dy)
                    if dist < self.assoc_dist:
                        cost[i, j] = dist
            row_idx, col_idx = linear_sum_assignment(cost)
        else:
            row_idx, col_idx = [], []

        matched = set()
        paired_mid = set()
        paired_d435 = set()
        for ri, ci in zip(row_idx, col_idx):
            if cost[ri, ci] < 1e8:
                matched.add((ri, ci))
                paired_mid.add(ri)
                paired_d435.add(ci)

        # 3. Build fused obstacle list
        fused = []
        stamp = now.to_msg()

        # Matched pairs: merge MID360 + D435i
        for ri, ci in matched:
            m_mid, m_d435 = mid[ri], d435[ci]
            conf = min(self.conf_dual, 1.0 - (1.0 - mid_conf[ri]) * (1.0 - d435_conf[ci]))
            m = self._merge_obstacles(m_mid, m_d435, conf, stamp)
            fused.append(m)

        # Unmatched MID360S obstacles
        for i in range(N):
            if i not in paired_mid and mid_conf[i] >= self.conf_min:
                m = self._copy_marker(mid[i], f'fused_{i}')
                m.ns = 'fused_mid360_only'
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.6
                fused.append(m)

        # Unmatched D435i obstacles
        off_d = N
        for j in range(M):
            if j not in paired_d435 and d435_conf[j] >= self.conf_min:
                m = self._copy_marker(d435[j], f'fused_{off_d + j}')
                m.ns = 'fused_d435i_only'
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 0.6
                fused.append(m)

        # 4. Add YOLO person obstacles (matched against existing fused obstacles)
        matched_yolo = set()
        for yi, ym in enumerate(yolo):
            if yolo_conf[yi] < self.conf_min:
                continue
            # Check if YOLO person matches any existing fused obstacle
            matched_existing = False
            for fi, fm in enumerate(fused):
                dx = ym.pose.position.x - fm.pose.position.x
                dy = ym.pose.position.y - fm.pose.position.y
                if math.hypot(dx, dy) < self.assoc_dist * 1.5:  # wider gate for person matching
                    # Boost confidence of existing obstacle
                    fm.color.a = min(1.0, fm.color.a + 0.2)
                    matched_existing = True
                    matched_yolo.add(yi)
                    break
            if not matched_existing:
                m = self._copy_marker(ym, f'fused_yolo_{yi}')
                m.ns = 'fused_yolo_person'
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 1.0
                m.color.a = float(yolo_conf[yi])
                fused.append(m)

        self.fused_pub.publish(MarkerArray(markers=fused))

        # 5. Risk markers
        if self.risk_enabled:
            self.risk_pub.publish(self._build_risk_markers(fused, stamp))

    def _mid360_confidence(self, m):
        r = math.hypot(m.pose.position.x, m.pose.position.y)
        if r <= self.conf_mid_decay_s:
            return self.conf_mid_base
        if r >= self.conf_mid_decay_e:
            return 0.5
        t = (r - self.conf_mid_decay_s) / (self.conf_mid_decay_e - self.conf_mid_decay_s)
        return self.conf_mid_base + t * (0.5 - self.conf_mid_base)

    def _d435i_confidence(self, m):
        r = math.hypot(m.pose.position.x, m.pose.position.y)
        if r <= self.conf_d435i_decay_s:
            return self.conf_d435i_base
        if r >= self.conf_d435i_decay_e:
            return 0.4
        t = (r - self.conf_d435i_decay_s) / (self.conf_d435i_decay_e - self.conf_d435i_decay_s)
        return self.conf_d435i_base + t * (0.4 - self.conf_d435i_base)

    def _yolo_confidence(self, m):
        """YOLO detection confidence — primarily from marker alpha/color."""
        base = max(self.conf_yolo_base, m.color.a) if hasattr(m.color, 'a') and m.color.a > 0 else self.conf_yolo_base
        r = math.hypot(m.pose.position.x, m.pose.position.y)
        if r > 15.0:
            return base * 0.5
        if r > 8.0:
            return base * 0.7
        return base

    def _merge_obstacles(self, m_mid, m_d435, conf, stamp):
        """Confidence-weighted average of two obstacle markers."""
        # Weights proportional to individual confidences
        c_mid = self._mid360_confidence(m_mid)
        c_d435 = self._d435i_confidence(m_d435)
        w_total = c_mid + c_d435
        if w_total < 1e-6:
            w_mid = w_d435 = 0.5
        else:
            w_mid = c_mid / w_total
            w_d435 = c_d435 / w_total

        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.ns = 'fused_obstacle'
        m.id = m_mid.id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.lifetime = Duration(sec=0, nanosec=500000000)

        m.pose.position.x = w_mid * m_mid.pose.position.x + w_d435 * m_d435.pose.position.x
        m.pose.position.y = w_mid * m_mid.pose.position.y + w_d435 * m_d435.pose.position.y
        m.pose.position.z = max(m_mid.pose.position.z, m_d435.pose.position.z)

        q = _euler_to_quaternion(0.0, 0.0, 0.0)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q

        m.scale.x = max(m_mid.scale.x, m_d435.scale.x, 0.15)
        m.scale.y = max(m_mid.scale.y, m_d435.scale.y, 0.15)
        m.scale.z = max(m_mid.scale.z, m_d435.scale.z, 0.15)

        # Color: cyan (fused) with alpha proportional to confidence
        m.color.r, m.color.g, m.color.b = 0.0, 1.0, 1.0
        m.color.a = float(conf)
        return m

    def _copy_marker(self, src, marker_id):
        m = Marker()
        m.header = src.header
        m.ns = src.ns
        m.id = src.id
        m.type = src.type
        m.action = Marker.ADD
        m.pose = src.pose
        m.scale = src.scale
        m.color = src.color
        return m

    def _build_risk_markers(self, fused, stamp):
        """Build risk zone markers around each fused obstacle.
        Risk = 1 / (distance_from_robot_origin + epsilon)."""
        markers = MarkerArray()
        for i, m in enumerate(fused):
            r = math.hypot(m.pose.position.x, m.pose.position.y)
            risk = 1.0 / max(r, 0.1)
            risk = min(risk, 1.0)  # clamp

            rm = Marker()
            rm.header.frame_id = self.frame
            rm.header.stamp = stamp
            rm.ns = 'risk_zone'
            rm.id = i
            rm.type = Marker.SPHERE
            rm.action = Marker.ADD
            rm.pose.position = m.pose.position
            rm.scale.x = rm.scale.y = rm.scale.z = max(m.scale.x, 0.3) * 1.5

            if risk > 0.7:
                rm.color.r, rm.color.g, rm.color.b = 1.0, 0.0, 0.0  # red
            elif risk > 0.4:
                rm.color.r, rm.color.g, rm.color.b = 1.0, 0.65, 0.0  # orange
            elif risk > 0.2:
                rm.color.r, rm.color.g, rm.color.b = 1.0, 1.0, 0.0  # yellow
            else:
                rm.color.r, rm.color.g, rm.color.b = 0.0, 1.0, 0.0  # green
            rm.color.a = 0.3
            markers.markers.append(rm)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
