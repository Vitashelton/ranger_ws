#!/usr/bin/env python3
"""Inject a fixed latency on a topic (time-delay experiment tool).

Buffers incoming messages and republishes them after a configurable delay
(50/100/200/300 ms per the protocol). Used to emulate weak synchronisation in
a controlled, repeatable way so the time-confidence module can be evaluated.

Example:
    ros2 run tca_bev_nav time_delay_injector \
        --ros-args -p in_topic:=/camera/depth/image_rect_raw \
                   -p out_topic:=/camera/depth/image_rect_raw_delayed \
                   -p delay_ms:=150

Run the fusion node against the *_delayed topic to reproduce a delay condition.
Not part of the deployed system; experiment-only.
"""
from __future__ import annotations

import collections

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TimeDelayInjector(Node):
    def __init__(self):
        super().__init__('time_delay_injector')
        self.declare_parameter('in_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('out_topic',
                               '/camera/depth/image_rect_raw_delayed')
        self.declare_parameter('delay_ms', 150.0)
        self._delay = self.get_parameter('delay_ms').value * 1e-3
        self._buf = collections.deque()
        self.create_subscription(
            Image, self.get_parameter('in_topic').value, self._on_msg, 50)
        self._pub = self.create_publisher(
            Image, self.get_parameter('out_topic').value, 50)
        self.create_timer(0.005, self._flush)
        self.get_logger().info(
            f'delaying by {self._delay*1e3:.0f} ms')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_msg(self, msg: Image):
        self._buf.append((self._now(), msg))

    def _flush(self):
        now = self._now()
        while self._buf and (now - self._buf[0][0]) >= self._delay:
            _, msg = self._buf.popleft()
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TimeDelayInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
