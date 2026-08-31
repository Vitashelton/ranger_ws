#!/usr/bin/env python3
"""
/scan + /topo_command -> /cmd_vel

A deliberately simple LiDAR-based controller for proving the semantic
navigation data flow in simulation. It is NOT the final BEV-MPPI-TD-MPC2
controller; later you can replace this node while keeping the same command
and sensor interfaces.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def finite_min(values, fallback=10.0):
    vals = [v for v in values if math.isfinite(v) and v > 0.02]
    return min(vals) if vals else fallback


class ReactiveLocalController(Node):
    def __init__(self):
        super().__init__('reactive_local_controller')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('topo_command_topic', '/topo_command')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('nominal_speed', 0.20)
        self.declare_parameter('search_speed', 0.12)
        self.declare_parameter('stop_distance', 0.42)
        self.declare_parameter('slow_distance', 0.75)
        self.declare_parameter('corridor_kp', 0.65)
        self.declare_parameter('max_yaw_rate', 0.55)
        self.declare_parameter('enter_yaw_rate', 0.38)
        self.declare_parameter('enter_duration_s', 2.7)

        self.command = 'SEARCH_DOOR_904'
        self.scan = None
        self.enter_start_time = None
        self.task_done = False

        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.search_speed = float(self.get_parameter('search_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.corridor_kp = float(self.get_parameter('corridor_kp').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.enter_yaw_rate = float(self.get_parameter('enter_yaw_rate').value)
        self.enter_duration_s = float(self.get_parameter('enter_duration_s').value)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        topo_topic = self.get_parameter('topo_command_topic').value
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.command_sub = self.create_subscription(String, topo_topic, self.on_command, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('control_hz').value), self.on_control)

        self.get_logger().info(f'Reactive controller active. scan={scan_topic}, command={topo_topic}, cmd={cmd_topic}')

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def on_command(self, msg: String):
        new_command = msg.data.strip()
        if new_command != self.command:
            self.get_logger().info(f'Topology command: {new_command}')
            if new_command.startswith('ENTER_ROOM'):
                self.enter_start_time = None
                self.task_done = False
        self.command = new_command

    def sector_minima(self):
        if self.scan is None:
            return 10.0, 10.0, 10.0

        left, front, right = [], [], []
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) <= 0.28:
                front.append(distance)
            elif 0.35 <= angle <= 1.30:
                left.append(distance)
            elif -1.30 <= angle <= -0.35:
                right.append(distance)

        return finite_min(left), finite_min(front), finite_min(right)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def safety_override(self, left, front, right):
        if front < self.stop_distance:
            turn = self.max_yaw_rate if left > right else -self.max_yaw_rate
            return 0.0, turn
        if front < self.slow_distance:
            turn = 0.6 * self.max_yaw_rate if left > right else -0.6 * self.max_yaw_rate
            return 0.05, turn
        return None

    def on_control(self):
        cmd = Twist()
        left, front, right = self.sector_minima()

        safety = self.safety_override(left, front, right)
        if safety is not None:
            cmd.linear.x, cmd.angular.z = safety
            self.cmd_pub.publish(cmd)
            return

        if self.task_done:
            self.cmd_pub.publish(cmd)
            return

        if self.command.startswith('FOLLOW_CORRIDOR'):
            cmd.linear.x = self.nominal_speed
            cmd.angular.z = self.clamp(self.corridor_kp * (left - right),
                                       -self.max_yaw_rate, self.max_yaw_rate)

        elif self.command.startswith('SEARCH_DOOR'):
            cmd.linear.x = self.search_speed
            cmd.angular.z = self.clamp(self.corridor_kp * (left - right),
                                       -0.45 * self.max_yaw_rate, 0.45 * self.max_yaw_rate)

        elif self.command.startswith('ENTER_ROOM_LEFT') or self.command.startswith('ENTER_ROOM_RIGHT'):
            if self.enter_start_time is None:
                self.enter_start_time = time.monotonic()

            elapsed = time.monotonic() - self.enter_start_time
            if elapsed >= self.enter_duration_s:
                self.task_done = True
                self.get_logger().info('Entry primitive complete: TASK_DONE')
            else:
                cmd.linear.x = 0.10
                cmd.angular.z = self.enter_yaw_rate if 'LEFT' in self.command else -self.enter_yaw_rate

        else:
            # STOP_AND_OBSERVE / unknown command
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveLocalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish one explicit stop on shutdown.
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
