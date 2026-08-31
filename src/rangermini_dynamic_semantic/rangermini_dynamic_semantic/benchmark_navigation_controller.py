"""Executable holonomic benchmark controller with graph-level rerouting.

This controller is for repeatable simulation smoke tests.  Real experiments
replace it with Nav2 while retaining the goal, failure, success and graph
contracts used by the semantic-maintenance stack.
"""
import heapq
import json
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BenchmarkNavigationController(Node):
    def __init__(self):
        super().__init__("benchmark_navigation_controller")
        default = str(Path(get_package_share_directory(
            "rangermini_dynamic_semantic")) / "config" / "dynamic_benchmark.yaml")
        self.declare_parameter("config_file", default)
        self.declare_parameter("max_speed", 0.45)
        self.declare_parameter("arrival_radius", 0.55)
        self.declare_parameter("obstacle_distance", 0.72)
        self.declare_parameter("blocked_timeout_sec", 2.5)
        self.declare_parameter("require_sim_time", True)
        self.declare_parameter("trial_id", "manual_trial")
        self.declare_parameter("scenario_id", "S6")
        self.declare_parameter("seed", 0)
        self.declare_parameter("method_mode", "Ours")
        self.assert_time_contract()
        with open(str(self.get_parameter("config_file").value), encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        self.regions = cfg.get("regions", {})
        self.edges = {}
        for edge in cfg.get("topology_edges", []):
            key = self.key(edge["from"], edge["to"])
            self.edges[key] = dict(edge, state="FREE", effective_cost=float(edge["cost"]))
        self.odom = None
        self.scan = None
        self.current_region = "lobby"
        self.goal_region = None
        self.path = []
        self.active_edge = None
        self.blocked_since = None
        self.failed_edge = None
        self.task_context_value = {}
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.failure_pub = self.create_publisher(String, "/navigation_failure", 20)
        self.success_pub = self.create_publisher(String, "/navigation_success", 20)
        self.result_pub = self.create_publisher(String, "/benchmark/navigation_result", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(String, "/dynamic_semantic_graph", self.on_graph, 10)
        self.create_subscription(String, "/benchmark/goal", self.on_goal, 10)
        task_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/task_context/current", self.on_task_context, task_qos)
        self.create_timer(0.05, self.tick)

    def assert_time_contract(self):
        if (bool(self.get_parameter("require_sim_time").value) and
                not bool(self.get_parameter("use_sim_time").value)):
            raise RuntimeError("benchmark_navigation_controller requires use_sim_time=true")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def trial_context(self):
        return {
            "trial_id": str(self.get_parameter("trial_id").value),
            "scenario_id": str(self.get_parameter("scenario_id").value),
            "seed": int(self.get_parameter("seed").value),
            "method_mode": str(self.get_parameter("method_mode").value),
            "task_context": dict(self.task_context_value),
        }

    def on_task_context(self, msg):
        try:
            payload = json.loads(msg.data)
            self.task_context_value = payload.get("task_context", {})
        except ValueError:
            return

    @staticmethod
    def key(a, b):
        return "|".join(sorted((str(a), str(b))))

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_graph(self, msg):
        try:
            graph = json.loads(msg.data)
        except ValueError:
            return
        for edge in graph.get("edges", []):
            key = self.key(edge.get("from"), edge.get("to"))
            if key in self.edges:
                self.edges[key].update({
                    "state": edge.get("state", "FREE"),
                    "effective_cost": float(edge.get("effective_cost", edge.get("base_cost", 1.0))),
                })

    def on_goal(self, msg):
        try:
            payload = json.loads(msg.data)
            goal = payload.get("target_region", payload.get("region", ""))
        except ValueError:
            goal = msg.data.strip()
        if goal not in self.regions:
            self.get_logger().warn(f"Unknown benchmark region: {goal}")
            return
        self.goal_region = goal
        self.failed_edge = None
        self.plan()

    def plan(self):
        if not self.goal_region:
            return
        queue = [(0.0, self.current_region, [])]
        best = {self.current_region: 0.0}
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node == self.goal_region:
                self.path = path
                self.get_logger().info(f"Graph route: {self.current_region} -> {' -> '.join(path)}")
                return
            if cost > best.get(node, float("inf")):
                continue
            for edge in self.edges.values():
                if edge["state"] == "TEMP_BLOCKED":
                    continue
                if edge["from"] == node:
                    nxt = edge["to"]
                elif edge["to"] == node:
                    nxt = edge["from"]
                else:
                    continue
                new_cost = cost + float(edge.get("effective_cost", edge["cost"]))
                if new_cost < best.get(nxt, float("inf")):
                    best[nxt] = new_cost
                    heapq.heappush(queue, (new_cost, nxt, path + [nxt]))
        self.path = []
        self.result_pub.publish(String(data=json.dumps({
            "status": "NO_ROUTE", "from": self.current_region,
            "target_region": self.goal_region, "timestamp": self.now_sec(),
            "trial_context": self.trial_context()})))

    def obstacle_in_direction(self, angle):
        if self.scan is None or not self.scan.ranges:
            return False
        threshold = float(self.get_parameter("obstacle_distance").value)
        half_width = math.radians(28.0)
        for index, value in enumerate(self.scan.ranges):
            if not math.isfinite(value) or value <= 0.0:
                continue
            ray = self.scan.angle_min + index * self.scan.angle_increment
            delta = math.atan2(math.sin(ray - angle), math.cos(ray - angle))
            if abs(delta) <= half_width and value < threshold:
                return True
        return False

    def stop(self):
        self.cmd_pub.publish(Twist())

    def tick(self):
        if self.odom is None or not self.goal_region or not self.path:
            self.stop()
            return
        target = self.path[0]
        target_cfg = self.regions[target]
        pose = self.odom.pose.pose
        dx = float(target_cfg["x"]) - float(pose.position.x)
        dy = float(target_cfg["y"]) - float(pose.position.y)
        distance = math.hypot(dx, dy)
        edge_key = self.key(self.current_region, target)
        self.active_edge = edge_key
        if distance <= float(self.get_parameter("arrival_radius").value):
            previous = self.current_region
            self.current_region = target
            self.path.pop(0)
            self.blocked_since = None
            self.failed_edge = None
            self.success_pub.publish(String(data=json.dumps({
                "event_type": "TRAVERSAL_SUCCESS", "from": previous, "to": target,
                "edge": edge_key.replace("|", "-"), "timestamp": self.now_sec(),
                "trial_context": self.trial_context()})))
            if self.current_region == self.goal_region:
                self.stop()
                self.result_pub.publish(String(data=json.dumps({
                    "status": "SUCCEEDED", "target_region": self.goal_region,
                    "timestamp": self.now_sec(),
                    "trial_context": self.trial_context()})))
                self.goal_region = None
            return
        yaw = yaw_from_quaternion(pose.orientation)
        world_angle = math.atan2(dy, dx)
        body_angle = math.atan2(math.sin(world_angle - yaw), math.cos(world_angle - yaw))
        if self.obstacle_in_direction(body_angle):
            self.stop()
            now = self.now_sec()
            self.blocked_since = self.blocked_since or now
            if (now - self.blocked_since >=
                    float(self.get_parameter("blocked_timeout_sec").value) and
                    self.failed_edge != edge_key):
                self.failed_edge = edge_key
                edge = self.edges[edge_key]
                edge["state"] = "TEMP_BLOCKED"
                self.failure_pub.publish(String(data=json.dumps({
                    "event_type": "PATH_BLOCKED", "from": self.current_region,
                    "to": target, "edge": edge_key.replace("|", "-"),
                    "confidence": 0.9, "timestamp": now,
                    "trial_context": self.trial_context()})))
                self.plan()
            return
        self.blocked_since = None
        speed = min(float(self.get_parameter("max_speed").value), 0.7 * distance)
        cmd = Twist()
        cmd.linear.x = speed * math.cos(body_angle)
        cmd.linear.y = speed * math.sin(body_angle)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
