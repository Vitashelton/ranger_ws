#!/usr/bin/env python3
"""time_align_node.

Estimates inter-modality timestamp offsets and publishes a per-modality time
confidence. It does NOT hardware-synchronise sensors (the platform has none);
it *measures* the de-synchronisation so downstream fusion can stay conservative.

Subscribes:
    /livox/lidar               (sensor_msgs/PointCloud2  OR  livox CustomMsg)
    /camera/depth/image_rect_raw (sensor_msgs/Image)
    /livox/imu                 (sensor_msgs/Imu)
    /odom                      (nav_msgs/Odometry)

Publishes:
    /time_align/status         (std_msgs/String, JSON payload)

The reference clock is the LiDAR stamp (lowest-latency geometric modality).
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tca_bev_nav.bev.confidence import TimeConfidenceParams, time_confidence
from tca_bev_nav.common.ros_utils import stamp_to_sec

# Soft import: Livox CustomMsg may or may not be installed.
try:
    from livox_ros_driver2.msg import CustomMsg  # type: ignore
    _HAS_LIVOX = True
except Exception:  # pragma: no cover
    _HAS_LIVOX = False
from sensor_msgs.msg import Image, Imu, PointCloud2  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


class TimeAlignNode(Node):
    def __init__(self):
        super().__init__('time_align_node')
        self.declare_parameter('tau', 0.10)
        self.declare_parameter('max_dt', 0.40)
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('lidar_is_custom_msg', _HAS_LIVOX)
        self.declare_parameter('publish_period_s', 0.2)

        self._tp = TimeConfidenceParams(
            tau=self.get_parameter('tau').value,
            max_dt=self.get_parameter('max_dt').value,
        )
        self._last = {'lidar': None, 'depth': None, 'imu': None, 'odom': None}

        lidar_topic = self.get_parameter('lidar_topic').value
        if self.get_parameter('lidar_is_custom_msg').value and _HAS_LIVOX:
            self.create_subscription(CustomMsg, lidar_topic,
                                     lambda m: self._set('lidar', m.header), 10)
        else:
            self.create_subscription(PointCloud2, lidar_topic,
                                     lambda m: self._set('lidar', m.header), 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 lambda m: self._set('depth', m.header), 10)
        self.create_subscription(Imu, '/livox/imu',
                                 lambda m: self._set('imu', m.header), 50)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self._set('odom', m.header), 50)

        self._pub = self.create_publisher(String, '/time_align/status', 10)
        self.create_timer(self.get_parameter('publish_period_s').value,
                          self._tick)
        self.get_logger().info('time_align_node up (reference clock = lidar).')

    def _set(self, key: str, header):
        self._last[key] = stamp_to_sec(header.stamp)

    def _tick(self):
        ref = self._last['lidar']
        out = {'reference': 'lidar', 'have_lidar': ref is not None}
        if ref is not None:
            for key in ('depth', 'imu', 'odom'):
                t = self._last[key]
                if t is None:
                    out[f'dt_{key}'] = None
                    out[f'conf_{key}'] = 0.0
                else:
                    dt = abs(ref - t)
                    out[f'dt_{key}'] = dt
                    out[f'conf_{key}'] = time_confidence(dt, self._tp)
        msg = String()
        msg.data = json.dumps(out)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TimeAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
