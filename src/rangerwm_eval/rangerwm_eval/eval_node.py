#!/usr/bin/env python3
"""rangerwm_eval/eval_node — 在线评测: success/collision/path_eff/intervention/safety_stop。"""
import math, json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

class EvalNode(Node):
    def __init__(self):
        super().__init__("rangerwm_eval")
        self.declare_parameter("task", "go_to_target")
        self.declare_parameter("goal_x", 5.0); self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("success_dist", 0.4)
        self.path_len = 0.0; self.prev = None; self.collisions = 0; self.stops = 0
        self.min_dist = math.inf; self.start = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Bool, "/collision", self.on_coll, 5)
        self.create_subscription(Float32, "/front_min_dist", self.on_front, 5)
        self.create_timer(1.0, self.report)

    def on_odom(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.prev: self.path_len += math.hypot(p[0]-self.prev[0], p[1]-self.prev[1])
        if self.start is None: self.start = p
        self.prev = p
        gx, gy = self.get_parameter("goal_x").value, self.get_parameter("goal_y").value
        self.min_dist = min(self.min_dist, math.hypot(gx-p[0], gy-p[1]))

    def on_coll(self, m):
        if m.data: self.collisions += 1
    def on_front(self, m):
        if m.data < 0.05: self.stops += 1

    def report(self):
        if not self.prev: return
        gx, gy = self.get_parameter("goal_x").value, self.get_parameter("goal_y").value
        d = math.hypot(gx-self.prev[0], gy-self.prev[1])
        success = d < self.get_parameter("success_dist").value and self.collisions == 0
        straight = math.hypot(gx-self.start[0], gy-self.start[1]) if self.start else 0.0
        rec = dict(task=self.get_parameter("task").value, success=success,
                   final_dist=round(d,3), collisions=self.collisions,
                   path_efficiency=round(straight/max(self.path_len,1e-6),3),
                   safety_stops=self.stops)
        self.get_logger().info(json.dumps(rec))

def main():
    rclpy.init(); n = EvalNode()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
