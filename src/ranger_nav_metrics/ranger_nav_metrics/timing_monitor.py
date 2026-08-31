#!/usr/bin/env python3
"""Monitor real sensor timing without changing the robot control path."""

import csv
import math
from collections import deque
from pathlib import Path
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # Allows the monitor to report the missing driver cleanly.
    CustomMsg = None


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StreamWindow:
    def __init__(self, maxlen=200):
        self.arrivals = deque(maxlen=maxlen)
        self.stamps = deque(maxlen=maxlen)

    def add(self, arrival, stamp):
        self.arrivals.append(arrival)
        self.stamps.append(stamp)

    def summary(self, now):
        periods = [
            self.arrivals[index] - self.arrivals[index - 1]
            for index in range(1, len(self.arrivals))
            if self.arrivals[index] > self.arrivals[index - 1]
        ]
        mean_period = statistics.fmean(periods) if periods else math.nan
        jitter = statistics.pstdev(periods) if len(periods) > 1 else math.nan
        latest_stamp = self.stamps[-1] if self.stamps else math.nan
        return {
            'samples': len(self.arrivals),
            'rate_hz': 1.0 / mean_period if mean_period > 0.0 else math.nan,
            'mean_period_ms': mean_period * 1000.0,
            'jitter_ms': jitter * 1000.0,
            'age_ms': (now - latest_stamp) * 1000.0,
            'latest_stamp': latest_stamp,
        }


class TimingMonitor(Node):
    def __init__(self):
        super().__init__('ranger_sensor_timing_monitor')
        self.declare_parameter(
            'output_dir', '~/.config/ranger_nav/research/timing')
        self.declare_parameter('report_interval', 1.0)
        output_dir = Path(
            self.get_parameter('output_dir').value).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = output_dir / f'timing_{timestamp}.csv'
        self.streams = {
            'lidar': StreamWindow(),
            'imu': StreamWindow(),
            'rgb': StreamWindow(),
            'depth': StreamWindow(),
            'wheel_odom': StreamWindow(),
            'lio_odom': StreamWindow(),
            'nav_points': StreamWindow(),
        }
        self._subscriptions = []
        if CustomMsg is not None:
            self._subscribe(CustomMsg, '/livox/lidar', 'lidar')
        else:
            self.get_logger().error(
                'livox_ros_driver2 Python messages unavailable; lidar timing disabled')
        self._subscribe(Imu, '/livox/imu', 'imu')
        self._subscribe(Image, '/camera/color/image_raw', 'rgb')
        self._subscribe(
            Image, '/camera/aligned_depth_to_color/image_raw', 'depth')
        self._subscribe(Odometry, '/odom', 'wheel_odom')
        self._subscribe(Odometry, '/lio/base_odom', 'lio_odom')
        self._subscribe(PointCloud2, '/nav/points', 'nav_points')

        with self.csv_path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'wall_time', 'stream', 'samples', 'rate_hz',
                'mean_period_ms', 'jitter_ms', 'age_ms',
                'latest_stamp', 'lidar_rgb_skew_ms',
                'lidar_depth_skew_ms',
            ])
        interval = float(self.get_parameter('report_interval').value)
        self.create_timer(interval, self._report)
        self.get_logger().info(f'timing log: {self.csv_path}')

    def _subscribe(self, message_type, topic, stream):
        subscription = self.create_subscription(
            message_type, topic,
            lambda message, name=stream: self._receive(name, message),
            qos_profile_sensor_data)
        self._subscriptions.append(subscription)

    def _receive(self, stream, message):
        self.streams[stream].add(
            self.get_clock().now().nanoseconds * 1e-9,
            stamp_seconds(message))

    @staticmethod
    def _format(value):
        return 'nan' if not math.isfinite(value) else f'{value:.3f}'

    def _latest_skew_ms(self, first, second):
        a = self.streams[first].stamps
        b = self.streams[second].stamps
        if not a or not b:
            return math.nan
        return (a[-1] - b[-1]) * 1000.0

    def _report(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        rgb_skew = self._latest_skew_ms('lidar', 'rgb')
        depth_skew = self._latest_skew_ms('lidar', 'depth')
        rows = []
        active = []
        for name, window in self.streams.items():
            summary = window.summary(now)
            rows.append([
                f'{now:.6f}', name, summary['samples'],
                self._format(summary['rate_hz']),
                self._format(summary['mean_period_ms']),
                self._format(summary['jitter_ms']),
                self._format(summary['age_ms']),
                self._format(summary['latest_stamp']),
                self._format(rgb_skew), self._format(depth_skew),
            ])
            if summary['samples']:
                active.append(
                    f"{name}={self._format(summary['rate_hz'])}Hz/"
                    f"{self._format(summary['jitter_ms'])}ms-jitter")
        with self.csv_path.open('a', newline='', encoding='utf-8') as handle:
            csv.writer(handle).writerows(rows)
        if active:
            self.get_logger().info(
                ' | '.join(active)
                + f' | lidar-rgb={self._format(rgb_skew)}ms'
                + f' lidar-depth={self._format(depth_skew)}ms')
        else:
            self.get_logger().warning('no monitored messages received yet')


def main(args=None):
    rclpy.init(args=args)
    node = TimingMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
