#!/usr/bin/env python3
import math
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import json
import time


class CmdVelWatchdog(Node):
    """Fail-safe boundary between planning/filtering and the simulated chassis."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel_safe_raw")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("timeout", 0.35)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("enable_pointcloud_guard", False)
        self.declare_parameter("pointcloud_topic", "/camera/points")
        self.declare_parameter("sensor_timeout", 0.80)
        self.declare_parameter("zone_x_min", 0.15)
        self.declare_parameter("zone_x_stop", 1.05)
        self.declare_parameter("zone_x_clear", 1.20)
        self.declare_parameter("zone_y_half", 0.62)
        self.declare_parameter("zone_z_min", 0.05)
        self.declare_parameter("zone_z_max", 1.80)
        self.declare_parameter("point_stride", 8)
        self.declare_parameter("min_obstacle_points", 3)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("clear_frames", 4)
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self.on_cmd, 10)
        self.create_subscription(String, "/task_control", self.on_task_control, 10)
        self.pointcloud_guard = bool(
            self.get_parameter("enable_pointcloud_guard").value)
        self.safety_pub = self.create_publisher(
            String, "/office_rpg/safety_status", 10)
        self.event_pub = self.create_publisher(
            String, "/office_rpg/event_log", 50)
        # Topology events are deliberately anonymous: this node only knows
        # that the current forward corridor is blocked, not which person or
        # world object caused it.  The search executor maps the event to its
        # active semantic target and requests a new plan.
        self.dynamic_observation_pub = self.create_publisher(
            String, "/office_rpg/dynamic_observation", 10)
        if self.pointcloud_guard:
            self.create_subscription(
                PointCloud2, str(self.get_parameter("pointcloud_topic").value),
                self.on_pointcloud, 10)
        self.last_cmd = self.get_clock().now()
        self.last_sensor = self.get_clock().now()
        self.cmd = Twist()
        self.have_cmd = False
        self.have_sensor = False
        self.sensor_blocked = False
        self.obstacle_frames = 0
        self.clear_frames = 0
        self.nearest_obstacle = None
        self.protection_points = 0
        self.stopped = True
        self.last_state = "INITIALIZING"
        self.yield_count = 0
        self.explicit_stop = True
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd = self.get_clock().now()
        self.have_cmd = True

    def on_task_control(self, msg):
        command = msg.data.strip().upper()
        if command == "STOP":
            self.explicit_stop = True
            self.pub.publish(Twist())
        elif command == "START":
            self.explicit_stop = False

    def on_pointcloud(self, msg):
        """Three-dimensional sensor gate; no entity names or world truth."""
        self.last_sensor = self.get_clock().now()
        self.have_sensor = True
        if msg.height == 0 or msg.width == 0 or not msg.data or msg.point_step <= 0:
            self.obstacle_frames += 1
            return
        try:
            offsets = {field.name: field.offset for field in msg.fields}
            xyz = (offsets["x"], offsets["y"], offsets["z"])
            endian = ">" if msg.is_bigendian else "<"
            limit = float(self.get_parameter(
                "zone_x_clear" if self.sensor_blocked else "zone_x_stop").value)
            xmin = float(self.get_parameter("zone_x_min").value)
            yhalf = float(self.get_parameter("zone_y_half").value)
            zmin = float(self.get_parameter("zone_z_min").value)
            zmax = float(self.get_parameter("zone_z_max").value)
            stride = max(1, int(self.get_parameter("point_stride").value))
            occupied = []
            total = int(msg.width) * int(msg.height)
            for index in range(0, total, stride):
                base = (index // msg.width) * msg.row_step + \
                       (index % msg.width) * msg.point_step
                x = struct.unpack_from(endian + "f", msg.data, base + xyz[0])[0]
                y = struct.unpack_from(endian + "f", msg.data, base + xyz[1])[0]
                z = struct.unpack_from(endian + "f", msg.data, base + xyz[2])[0]
                if (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
                        and xmin <= x <= limit and abs(y) <= yhalf
                        and zmin <= z <= zmax):
                    occupied.append(math.hypot(x, y))
        except (KeyError, struct.error, ValueError, TypeError) as exc:
            self.get_logger().warn(
                f"Malformed PointCloud2 frame: {exc}; fail-safe stop",
                throttle_duration_sec=5.0)
            self.obstacle_frames += 1
            return
        self.nearest_obstacle = min(occupied) if occupied else None
        self.protection_points = len(occupied)
        if len(occupied) >= int(self.get_parameter("min_obstacle_points").value):
            self.obstacle_frames += 1
            self.clear_frames = 0
            if self.obstacle_frames >= int(self.get_parameter("confirm_frames").value):
                self.sensor_blocked = True
        else:
            self.obstacle_frames = 0
            self.clear_frames += 1
            if self.clear_frames >= int(self.get_parameter("clear_frames").value):
                self.sensor_blocked = False

    def publish_status(self, state, age):
        payload = {
            "state": state,
            "source": "pointcloud2_3d_guard" if self.pointcloud_guard else "watchdog_only",
            "nearest_obstacle_distance": self.nearest_obstacle,
            "protection_point_count": self.protection_points,
            "last_sensor_age": round(age, 3) if math.isfinite(age) else None,
            "ground_truth_used": False,
            "yield_count": self.yield_count,
        }
        self.safety_pub.publish(String(data=json.dumps(payload)))

    def transition_event(self, state):
        if state == self.last_state:
            return
        previous = self.last_state
        self.last_state = state
        event_type = None
        message = ""
        if state == "YIELDING":
            self.yield_count += 1
            event_type = "SAFETY_YIELD_START"
            message = "PointCloud2 三维安全区检测到横穿障碍，/cmd_vel_safe 已置零"
            self.dynamic_observation_pub.publish(String(data=json.dumps({
                "event_type": "EDGE_BLOCKED",
                "timestamp": time.time(),
                "duration": 0.0,
                "confidence": 1.0,
                "source": "cmd_vel_watchdog_pointcloud",
            }, ensure_ascii=False)))
        elif previous == "YIELDING" and state == "CLEAR":
            event_type = "SAFETY_RESUME"
            message = "三维安全区清空，自动恢复原导航任务"
        elif state == "SENSOR_TIMEOUT":
            event_type = "SAFETY_FAIL_SAFE_STOP"
            message = "PointCloud2 不可用，执行 fail-safe stop"
        if event_type:
            payload = {"event_type": event_type, "message": message,
                       "timestamp": time.time(), "source": "cmd_vel_watchdog",
                       "safety_state": state, "ground_truth_used": False}
            self.event_pub.publish(String(data=json.dumps(payload,
                                                           ensure_ascii=False)))

    def tick(self):
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        timed_out = (not self.have_cmd) or age > float(self.get_parameter("timeout").value)
        sensor_age = (self.get_clock().now() - self.last_sensor).nanoseconds * 1e-9
        sensor_timed_out = self.pointcloud_guard and (
            not self.have_sensor or sensor_age > float(
                self.get_parameter("sensor_timeout").value))
        if self.explicit_stop or timed_out or sensor_timed_out or self.sensor_blocked:
            safe_stop = Twist()
            # A front obstacle must never permit translation.  A pure spin
            # command from the existing retask controller is safe at the
            # measured clearance and lets the forward camera turn away instead
            # of deadlocking forever.  STOP remains a zero command upstream.
            if (not self.explicit_stop and self.sensor_blocked and
                    not timed_out and not sensor_timed_out and
                    abs(self.cmd.linear.x) < 1.0e-3 and
                    abs(self.cmd.linear.y) < 1.0e-3):
                safe_stop.angular.z = self.cmd.angular.z
            self.pub.publish(safe_stop)
            if not self.stopped:
                reason = ("explicit STOP" if self.explicit_stop else
                          "upstream command timeout" if timed_out else
                          "PointCloud2 sensor timeout" if sensor_timed_out else
                          "near obstacle in 3-D protection zone")
                self.get_logger().warn(f"Safety stop: {reason}")
            self.stopped = True
            state = ("STOPPED" if self.explicit_stop else
                     "SENSOR_TIMEOUT" if sensor_timed_out else
                     "YIELDING" if self.sensor_blocked else "STOPPED")
        else:
            self.pub.publish(self.cmd)
            self.stopped = False
            state = "CLEAR"
        self.transition_event(state)
        self.publish_status(state, sensor_age if self.have_sensor else math.inf)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Publish several zero frames for transports/controllers that sample commands.
        if rclpy.ok():
            for _ in range(3):
                node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
