#!/usr/bin/env python3
"""wheel_observer_node (READ-ONLY).

Optional. If you want wheel-speed / steering-angle feedback that the official
ranger_ros2 driver does not expose, this node *passively reads* CAN frames in
a read-only fashion and republishes them. It NEVER writes to the bus and does
NOT modify the official SDK or driver.

Two modes:
  * 'topic'  : subscribe to existing chassis feedback topics
               (/motion_state, /actuator_state) and just re-expose a tidy
               observed state. This is the safe default — no CAN access at all.
  * 'socketcan': open a python-can SocketCAN bus in *listen-only* /
               receive-only mode and decode frames. Requires python-can and a
               documented Ranger CAN frame map. DISABLED by default.

Publishes:
    /observer/wheel_state  (std_msgs/String, JSON)

IMPORTANT: socketcan mode must be brought up read-only:
    sudo ip link set can0 type can bitrate 500000 listen-only on
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WheelObserverNode(Node):
    def __init__(self):
        super().__init__('wheel_observer_node')
        self.declare_parameter('mode', 'topic')  # 'topic' | 'socketcan'
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('publish_period_s', 0.05)

        self._state = {}
        self._pub = self.create_publisher(String, '/observer/wheel_state', 10)

        mode = self.get_parameter('mode').value
        if mode == 'topic':
            # Re-expose official feedback topics only. No CAN access.
            self.create_subscription(String, '/motion_state',
                                     lambda m: self._note('motion_state', m.data),
                                     10)
            self.create_subscription(String, '/actuator_state',
                                     lambda m: self._note('actuator_state',
                                                          m.data), 10)
            self.get_logger().info('wheel_observer in topic mode (no CAN).')
        elif mode == 'socketcan':
            self._init_socketcan()
        else:
            self.get_logger().error(f'unknown mode: {mode}')

        self.create_timer(self.get_parameter('publish_period_s').value,
                          self._tick)

    def _note(self, key, value):
        self._state[key] = value

    def _init_socketcan(self):
        try:
            import can  # python-can
        except Exception:
            self.get_logger().error(
                'python-can not installed; socketcan mode unavailable.')
            return
        ch = self.get_parameter('can_channel').value
        # receive_own_messages=False, and the interface itself should be
        # brought up listen-only at the OS level (see module docstring).
        self._bus = can.interface.Bus(channel=ch, interface='socketcan',
                                      receive_own_messages=False)
        self.create_timer(0.005, self._poll_can)
        self.get_logger().warn(
            'wheel_observer in socketcan LISTEN-ONLY mode; never transmits.')
        # TODO(can-map): fill in the Ranger Mini 2.0 CAN frame IDs and decoding
        # for wheel speed / steering angle. Until verified against the official
        # protocol doc, decoding stays empty so we never publish wrong numbers.

    def _poll_can(self):
        msg = self._bus.recv(timeout=0.0)
        if msg is None:
            return
        # TODO: decode msg.arbitration_id / msg.data per Ranger CAN map.
        self._state['last_can_id'] = hex(msg.arbitration_id)

    def _tick(self):
        out = String()
        out.data = json.dumps(self._state)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
