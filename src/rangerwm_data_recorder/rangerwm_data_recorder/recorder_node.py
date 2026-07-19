#!/usr/bin/env python3
"""rangerwm_data_recorder — 多源时间对齐 + 写 index.jsonl (原始仍由 ros2 bag 录)。"""
import json, time
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

def yaw_of(q):
    import math
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class RecorderNode(Node):
    def __init__(self):
        super().__init__("rangerwm_recorder")
        self.declare_parameter("task_id", "goto")
        self.declare_parameter("scene_id", "lab_a")
        self.declare_parameter("slop", 0.03)
        self.declare_parameter("out", "/data/index.jsonl")
        rgb = Subscriber(self, Image, "/camera/color/image_raw")
        depth = Subscriber(self, Image, "/camera/depth/image_rect_raw")
        odom = Subscriber(self, Odometry, "/odom")
        cmd = Subscriber(self, TwistStamped, "/cmd_vel_stamped")  # 若 /cmd_vel 无 stamp 需包装
        self.sync = ApproximateTimeSynchronizer([rgb, depth, odom, cmd],
                        queue_size=30, slop=float(self.get_parameter("slop").value))
        self.sync.registerCallback(self.cb)
        self.f = open(self.get_parameter("out").value, "a")
        self.get_logger().info("recorder writing aligned index; raw via ros2 bag record.")

    def cb(self, rgb, depth, odom, cmd):
        rec = {"t": time.time(),
               "task_id": self.get_parameter("task_id").value,
               "scene_id": self.get_parameter("scene_id").value,
               "rgb_stamp": rgb.header.stamp.sec + rgb.header.stamp.nanosec*1e-9,
               "depth_stamp": depth.header.stamp.sec + depth.header.stamp.nanosec*1e-9,
               "odom": [odom.pose.pose.position.x, odom.pose.pose.position.y,
                        yaw_of(odom.pose.pose.orientation),
                        odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.angular.z],
               "cmd": [cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z]}
        self.f.write(json.dumps(rec)+"\n"); self.f.flush()

def main():
    rclpy.init(); n = RecorderNode()
    try: rclpy.spin(n)
    finally: n.f.close(); n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
