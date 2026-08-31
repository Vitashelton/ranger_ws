#!/usr/bin/env python3
"""
/door_observation + target room -> /topo_command

This node never subscribes to /map, /amcl_pose, or global waypoints.
It provides a small semantic/topological layer whose output is a discrete
command, not a global coordinate target.
"""
from collections import Counter, deque
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TopologyManager(Node):
    def __init__(self):
        super().__init__('topology_manager')

        self.declare_parameter('target_room', '904')
        self.declare_parameter('door_observation_topic', '/door_observation')
        self.declare_parameter('topo_command_topic', '/topo_command')
        self.declare_parameter('confirmation_window', 5)
        self.declare_parameter('confirmation_votes', 3)
        self.declare_parameter('min_confidence', 0.70)
        self.declare_parameter('target_side_fallback', 'right')

        self.target_room = str(self.get_parameter('target_room').value)
        self.window = int(self.get_parameter('confirmation_window').value)
        self.required_votes = int(self.get_parameter('confirmation_votes').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.target_side_fallback = str(self.get_parameter('target_side_fallback').value).lower()

        self.votes = deque(maxlen=max(1, self.window))
        self.current_command = f'SEARCH_DOOR_{self.target_room}'
        self.confirmed = False

        in_topic = self.get_parameter('door_observation_topic').value
        out_topic = self.get_parameter('topo_command_topic').value
        self.pub = self.create_publisher(String, out_topic, 10)
        self.sub = self.create_subscription(String, in_topic, self.on_door_observation, 10)

        self.timer = self.create_timer(0.5, self.publish_current)
        self.get_logger().info(
            f'Topology manager target={self.target_room}, input={in_topic}, output={out_topic}'
        )

    def publish_current(self):
        msg = String()
        msg.data = self.current_command
        self.pub.publish(msg)

    def on_door_observation(self, msg: String):
        if self.confirmed:
            return

        try:
            obs = json.loads(msg.data)
            door_id = str(obs.get('door_id', ''))
            confidence = float(obs.get('confidence', 0.0))
            side = str(obs.get('side', self.target_side_fallback)).lower()
        except (ValueError, TypeError, json.JSONDecodeError):
            self.get_logger().warn(f'Invalid door observation: {msg.data}')
            return

        if confidence < self.min_confidence:
            return

        self.votes.append(door_id)
        counts = Counter(self.votes)

        if door_id == self.target_room and counts[self.target_room] >= self.required_votes:
            if side not in ('left', 'right'):
                side = self.target_side_fallback
            self.current_command = f'ENTER_ROOM_{side.upper()}_{self.target_room}'
            self.confirmed = True
            self.get_logger().info(
                f'Target room {self.target_room} confirmed by {counts[self.target_room]}/{len(self.votes)} observations; '
                f'command={self.current_command}'
            )
        else:
            self.current_command = f'SEARCH_DOOR_{self.target_room}'


def main(args=None):
    rclpy.init(args=args)
    node = TopologyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
