#!/usr/bin/env python3
"""Local MID-360S + D435i doorway evidence; never publishes motion commands."""

import json
import time

from livox_ros_driver2.msg import CustomMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from doorway_evidence_core import (
    depth_door_evidence,
    fuse_baselines,
    lidar_door_evidence,
    transform_lidar_to_base,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class DoorwayTraversability(Node):
    def __init__(self):
        super().__init__('doorway_traversability')
        self.declare_parameter('door_distance', 2.0)
        self.declare_parameter('ground_truth', 'UNSET')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter(
            'depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('max_receipt_age', 0.50)
        self.declare_parameter('max_pair_skew', 0.25)
        self.declare_parameter('publish_rate', 5.0)
        self.door_distance = float(self.get_parameter('door_distance').value)
        self.ground_truth = str(self.get_parameter('ground_truth').value).upper()
        self.max_receipt_age = float(self.get_parameter('max_receipt_age').value)
        self.max_pair_skew = float(self.get_parameter('max_pair_skew').value)
        self.lidar = None
        self.depth = None

        self.publisher = self.create_publisher(String, '/doorway/evidence', 20)
        self.create_subscription(
            String, '/doorway/ground_truth', self.label_callback, 10)
        self.create_subscription(
            CustomMsg, self.get_parameter('lidar_topic').value,
            self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self.publish_evidence)
        self.get_logger().info(
            f'Local doorway experiment: distance={self.door_distance:.2f}m, '
            f'ground_truth={self.ground_truth}; no cmd_vel publisher')

    def label_callback(self, message):
        label = message.data.strip().upper()
        if label in {'PASSABLE', 'BLOCKED', 'UNKNOWN', 'UNSET'}:
            self.ground_truth = label
            self.get_logger().info(f'ground truth changed to {label}')
        else:
            self.get_logger().error(f'rejected ground truth label: {label}')

    def lidar_callback(self, message):
        receipt = time.monotonic()
        points = np.asarray(
            [(point.x, point.y, point.z) for point in message.points],
            dtype=np.float32)
        points_base = transform_lidar_to_base(points)
        self.lidar = lidar_door_evidence(
            points_base, self.door_distance,
            source_stamp=stamp_seconds(message.header.stamp),
            receipt_stamp=receipt)

    def depth_callback(self, message):
        receipt = time.monotonic()
        encoding = message.encoding.upper()
        if encoding in {'16UC1', 'MONO16'}:
            dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
            columns = message.step // dtype.itemsize
            raw = np.frombuffer(message.data, dtype=dtype).reshape(
                message.height, columns)
            depth = raw[:, :message.width].astype(np.float32)
            depth *= 0.001
        elif encoding == '32FC1':
            dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
            columns = message.step // dtype.itemsize
            raw = np.frombuffer(message.data, dtype=dtype).reshape(
                message.height, columns)
            depth = raw[:, :message.width].astype(np.float32)
        else:
            self.get_logger().error(f'unsupported depth encoding: {message.encoding}')
            return
        self.depth = depth_door_evidence(
            depth, self.door_distance,
            source_stamp=stamp_seconds(message.header.stamp),
            receipt_stamp=receipt)

    def publish_evidence(self):
        result = fuse_baselines(
            self.lidar, self.depth, time.monotonic(),
            max_receipt_age=self.max_receipt_age,
            max_pair_skew=self.max_pair_skew)
        payload = {
            'schema_version': 1,
            'ground_truth': self.ground_truth,
            'door_distance_m': self.door_distance,
            'lidar': None if self.lidar is None else {
                'score': self.lidar.score,
                'valid': self.lidar.valid,
                'support': self.lidar.support,
                'reason': self.lidar.reason,
                'source_stamp': self.lidar.source_stamp,
            },
            'depth': None if self.depth is None else {
                'score': self.depth.score,
                'valid': self.depth.valid,
                'support': self.depth.support,
                'reason': self.depth.reason,
                'source_stamp': self.depth.source_stamp,
            },
            'methods': {
                'lidar_only': result['lidar_only'],
                'depth_only': result['depth_only'],
                'naive_late_fusion': result['naive_late_fusion'],
                'async_conservative': result['async_conservative'],
            },
            'timing': {
                'lidar_receipt_age': result['lidar_receipt_age'],
                'depth_receipt_age': result['depth_receipt_age'],
                'pair_stamp_skew': result['pair_stamp_skew'],
            },
            'decision_reason': result['async_reason'],
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DoorwayTraversability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
