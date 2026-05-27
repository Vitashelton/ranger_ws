#!/usr/bin/env python3
"""
TF bridge for Gazebo simulation.

Problem:
  robot_state_publisher publishes fixed joints (base_footprint->base_link,
  base_link->livox_frame) on /tf_static.  RViz2's tf2_ros::MessageFilter
  sometimes fails to chain /tf_static transforms with /tf transforms, so
  PointCloud2 messages with frame_id=livox_frame can never be resolved to
  the Fixed Frame (odom).

  Additionally, the gazebo_ros_diff_drive plugin may not publish the
  odom->base_footprint TF at all in some ROS 2 Humble configurations.

Solution:
  Publish the ENTIRE required TF chain on /tf at 20 Hz:
    odom -> base_footprint -> base_link -> livox_frame
         -> base_link -> camera_link -> camera_depth_optical_frame

  - odom->base_footprint: starts as identity; replaced by real odometry
    whenever a /odom message arrives.
  - All static links: fixed transforms from the URDF.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OdomToTFBridge(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Latest odometry — None until first /odom message arrives
        self._odom = None

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publish ALL transforms on /tf at 20 Hz so they never leave
        # the TF buffer cache (default 10 s).
        self.timer = self.create_timer(0.05, self.publish_all)

        # Pre-compute static quaternions
        # livox_frame:  rpy = (0, 30°, 0)
        self.livox_q = quaternion_from_rpy(0.0, 0.523599, 0.0)
        # camera_depth_optical_frame:  rpy = (-π/2, 0, -π/2)
        self.depth_q = quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)

        self.get_logger().info(
            'odom_to_tf_bridge started — publishing full TF chain on /tf at 20 Hz')

    def odom_callback(self, msg: Odometry):
        self._odom = msg

    def publish_all(self):
        now = self.get_clock().now().to_msg()

        # ---- odom -> base_footprint (dynamic; identity fallback) ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        if self._odom is not None:
            t.transform.translation.x = self._odom.pose.pose.position.x
            t.transform.translation.y = self._odom.pose.pose.position.y
            t.transform.translation.z = self._odom.pose.pose.position.z
            t.transform.rotation = self._odom.pose.pose.orientation
        else:
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # ---- base_footprint -> base_link (identity) ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # ---- base_link -> livox_frame (0.30, 0, 0.70)  pitch 30° ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'livox_frame'
        t.transform.translation.x = 0.30
        t.transform.translation.z = 0.70
        t.transform.rotation.x = self.livox_q[0]
        t.transform.rotation.y = self.livox_q[1]
        t.transform.rotation.z = self.livox_q[2]
        t.transform.rotation.w = self.livox_q[3]
        self.tf_broadcaster.sendTransform(t)

        # ---- base_link -> camera_link (0.35, 0, 0.60) ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.35
        t.transform.translation.z = 0.60
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # ---- camera_link -> camera_depth_optical_frame  (rpy -π/2 0 -π/2) ----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'camera_depth_optical_frame'
        t.transform.rotation.x = self.depth_q[0]
        t.transform.rotation.y = self.depth_q[1]
        t.transform.rotation.z = self.depth_q[2]
        t.transform.rotation.w = self.depth_q[3]
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
