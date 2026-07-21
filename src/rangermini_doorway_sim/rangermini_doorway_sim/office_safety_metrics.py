"""Independent simulation-only clearance/collision audit.

This node may consume Gazebo truth because it is an evaluator. Its output is
never subscribed by the planner, navigation controller, or safety gate.
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


NPC_IDS = {"teacher_zhang", "student_li", "visitor"}


class OfficeSafetyMetrics(Node):
    def __init__(self):
        super().__init__("office_rpg_safety_metrics")
        self.robot = None
        self.npcs = {}
        self.minimum_clearance = math.inf
        self.collision_count = 0
        self.in_collision = set()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            String, "/office_rpg/safety_metrics", qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(
            TFMessage, "/world/corridor_902_904_906_908/pose/info",
            self.on_world_poses, 10)
        self.create_subscription(String, "/office_rpg/schedule_reset",
                                 self.on_reset, 10)
        self.create_timer(0.1, self.tick)

    def on_reset(self, _msg):
        self.minimum_clearance = math.inf
        self.collision_count = 0
        self.in_collision.clear()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot = (float(msg.pose.pose.position.x),
                      float(msg.pose.pose.position.y), yaw)

    def on_world_poses(self, msg):
        for transform in msg.transforms:
            identity = transform.child_frame_id.rsplit("/", 1)[-1]
            if identity in NPC_IDS:
                self.npcs[identity] = (
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y))

    @staticmethod
    def clearance(robot, npc):
        rx, ry, yaw = robot
        dx, dy = npc[0] - rx, npc[1] - ry
        # Point-to-oriented Ranger footprint, then subtract NPC collision radius.
        local_x = abs(math.cos(yaw) * dx + math.sin(yaw) * dy)
        local_y = abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)
        outside_x = max(local_x - 0.369, 0.0)
        outside_y = max(local_y - 0.250, 0.0)
        return math.hypot(outside_x, outside_y) - 0.20

    def tick(self):
        if self.robot is None or not self.npcs:
            return
        current = {identity: self.clearance(self.robot, pose)
                   for identity, pose in self.npcs.items()}
        self.minimum_clearance = min(self.minimum_clearance, min(current.values()))
        colliding = {identity for identity, distance in current.items()
                     if distance <= 0.0}
        self.collision_count += len(colliding - self.in_collision)
        self.in_collision = colliding
        payload = {
            "source": "simulation_ground_truth_audit_only",
            "not_available_to_planner": True,
            "timestamp": time.time(),
            "minimum_npc_clearance_m": (round(self.minimum_clearance, 3)
                                         if math.isfinite(self.minimum_clearance)
                                         else None),
            "current_minimum_npc_clearance_m": round(min(current.values()), 3),
            "collision_count": self.collision_count,
            "collision_free": self.collision_count == 0,
        }
        self.pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = OfficeSafetyMetrics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
