#!/usr/bin/env python3
"""Explicitly armed cmd_vel gate with a short command watchdog."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVelSafetyGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_gate')
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('timeout', 0.30)
        self.declare_parameter('publish_rate', 20.0)
        self.timeout = float(self.get_parameter('timeout').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.enabled = False
        self.last_command = Twist()
        self.last_command_time = None
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.command_callback, 10)
        self.create_service(SetBool, '/navigation/enable_motion', self.enable_callback)
        self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().warn(
            'Motion gate DISABLED. Arm with: ros2 service call '
            '/navigation/enable_motion std_srvs/srv/SetBool "{data: true}"')

    def command_callback(self, msg: Twist):
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    def enable_callback(self, request, response):
        self.enabled = bool(request.data)
        # Never replay a command received before the latest arm/disarm action.
        self.last_command_time = None
        if not self.enabled:
            self.publisher.publish(Twist())
        response.success = True
        response.message = 'motion enabled' if self.enabled else 'motion disabled and stopped'
        self.get_logger().warn(response.message)
        return response

    def timer_callback(self):
        output = Twist()
        if self.enabled and self.last_command_time is not None:
            age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
            if age <= self.timeout:
                output = self.last_command
        self.publisher.publish(output)

    def stop(self):
        for _ in range(3):
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
