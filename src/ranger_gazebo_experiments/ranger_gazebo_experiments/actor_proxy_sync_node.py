#!/usr/bin/env python3
"""
Actor proxy sync node.

Subscribes to /sim/people_ground_truth (PoseArray) and moves
invisible collision proxies in Gazebo to match actor positions.
This ensures LiDAR rays hit the proxies for obstacle detection.

Uses Gazebo's /gazebo/set_model_state service to move proxies.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class ActorProxySyncNode(Node):
    def __init__(self):
        super().__init__('actor_proxy_sync_node')

        self.declare_parameter('ground_truth_topic', '/sim/people_ground_truth')
        self.declare_parameter('proxy_prefix', 'person_')
        self.declare_parameter('proxy_suffix', '_proxy')
        self.declare_parameter('update_rate', 30.0)

        self.gt_topic = self.get_parameter('ground_truth_topic').value
        self.proxy_prefix = self.get_parameter('proxy_prefix').value
        self.proxy_suffix = self.get_parameter('proxy_suffix').value
        update_rate = self.get_parameter('update_rate').value

        # Subscriber
        self.gt_sub = self.create_subscription(
            PoseArray, self.gt_topic, self._gt_cb, 10)

        # Gazebo set model state client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        self.last_poses = []
        self.timer = self.create_timer(1.0 / update_rate, self._sync_proxies)
        self.get_logger().info(
            f'actor_proxy_sync_node started: {self.gt_topic} -> Gazebo proxies')

    def _gt_cb(self, msg):
        self.last_poses = msg.poses

    def _sync_proxies(self):
        if not self.last_poses:
            return
        for i, pose in enumerate(self.last_poses):
            proxy_name = f'{self.proxy_prefix}{i + 1}{self.proxy_suffix}'
            req = SetEntityState.Request()
            req.state = EntityState()
            req.state.name = proxy_name
            req.state.pose.position.x = pose.position.x
            req.state.pose.position.y = pose.position.y
            req.state.pose.position.z = 0.875  # proxy half-height
            req.state.pose.orientation.w = 1.0
            # Fire and forget — don't block on response
            self.set_state_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ActorProxySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
