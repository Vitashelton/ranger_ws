#!/usr/bin/env python3
"""safety_supervisor_node.

THE ONLY NODE PERMITTED TO PUBLISH /cmd_vel.

Policies / teleop publish raw velocity commands. The supervisor clamps them and
republishes a guarded command on both /cmd_vel_safe and /cmd_vel. If any
required input goes stale, or a front-obstacle stop is requested, it outputs
zero velocity.

Subscribes:
    /teacher/cmd_vel_raw   (geometry_msgs/Twist)  teleop / rule controller
    /policy/cmd_vel_raw    (geometry_msgs/Twist)  learned student (optional)
    /bev/status            (std_msgs/String)      front-obstacle / freshness
Publishes:
    /cmd_vel_safe          (geometry_msgs/Twist)
    /cmd_vel               (geometry_msgs/Twist)  forwarded guarded command
    /safety/status         (std_msgs/String, JSON)
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from tca_bev_nav.common.ros_utils import RateWatch, clamp


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor_node')
        self.declare_parameter('max_linear_mps', 0.6)
        self.declare_parameter('max_angular_rps', 0.8)
        self.declare_parameter('max_lin_accel_mps2', 0.5)
        self.declare_parameter('max_ang_accel_rps2', 1.0)
        self.declare_parameter('control_period_s', 0.05)
        self.declare_parameter('cmd_timeout_s', 0.3)
        self.declare_parameter('bev_timeout_s', 0.5)
        self.declare_parameter('source', 'teacher')  # 'teacher' or 'policy'

        self._dt = self.get_parameter('control_period_s').value
        self._last_cmd = Twist()
        self._raw = Twist()
        self._raw_watch = RateWatch()
        self._bev_watch = RateWatch()
        self._front_stop = False  # set from /bev/status front-obstacle flag

        src = self.get_parameter('source').value
        topic = '/teacher/cmd_vel_raw' if src == 'teacher' \
            else '/policy/cmd_vel_raw'
        self.create_subscription(Twist, topic, self._on_raw, 10)
        # Always also listen to the other one so switching source is hot.
        other = '/policy/cmd_vel_raw' if src == 'teacher' \
            else '/teacher/cmd_vel_raw'
        self.create_subscription(Twist, other, self._on_raw, 10)
        self.create_subscription(String, '/bev/status', self._on_bev, 10)

        self.pub_safe = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/safety/status', 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().warn(
            'safety_supervisor_node is the ONLY publisher of /cmd_vel.')

    def _now_wall(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg: Twist):
        self._raw = msg
        self._raw_watch.update(self._now_wall(), self._now_wall())

    def _on_bev(self, msg: String):
        self._bev_watch.update(self._now_wall(), self._now_wall())
        try:
            d = json.loads(msg.data)
            # TODO(front-stop): replace with a proper front-sector occupancy
            # query on /bev/tensor. Placeholder keeps the interface explicit.
            self._front_stop = bool(d.get('front_obstacle', False))
        except Exception:
            self._front_stop = False

    def _tick(self):
        now = self._now_wall()
        reasons = []

        cmd_stale = self._raw_watch.staleness(now) > \
            self.get_parameter('cmd_timeout_s').value
        bev_stale = self._bev_watch.staleness(now) > \
            self.get_parameter('bev_timeout_s').value
        if cmd_stale:
            reasons.append('cmd_timeout')
        if bev_stale:
            reasons.append('bev_timeout')
        if self._front_stop:
            reasons.append('front_obstacle')

        target = Twist()
        if not reasons:
            target.linear.x = clamp(
                self._raw.linear.x,
                -self.get_parameter('max_linear_mps').value,
                self.get_parameter('max_linear_mps').value)
            target.angular.z = clamp(
                self._raw.angular.z,
                -self.get_parameter('max_angular_rps').value,
                self.get_parameter('max_angular_rps').value)
        # else target stays zero (hard stop)

        guarded = self._rate_limit(target)
        self._last_cmd = guarded

        self.pub_safe.publish(guarded)
        self.pub_cmd.publish(guarded)
        st = String()
        st.data = json.dumps({
            'stop_reasons': reasons,
            'out_linear': guarded.linear.x,
            'out_angular': guarded.angular.z,
        })
        self.pub_status.publish(st)

    def _rate_limit(self, target: Twist) -> Twist:
        dv = self.get_parameter('max_lin_accel_mps2').value * self._dt
        dw = self.get_parameter('max_ang_accel_rps2').value * self._dt
        out = Twist()
        out.linear.x = clamp(target.linear.x,
                             self._last_cmd.linear.x - dv,
                             self._last_cmd.linear.x + dv)
        out.angular.z = clamp(target.angular.z,
                             self._last_cmd.angular.z - dw,
                             self._last_cmd.angular.z + dw)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
