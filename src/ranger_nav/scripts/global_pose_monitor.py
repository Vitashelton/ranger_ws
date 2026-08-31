#!/usr/bin/env python3
"""Expose the persistent-map robot pose and conservative TF availability."""

import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalPoseMonitor(Node):
    def __init__(self):
        super().__init__('global_pose_monitor')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('health_topic', '/localization/health')
        self.declare_parameter(
            'map_id_file', '~/.config/ranger_nav/maps/real_lab.map_id')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        map_id_path = Path(
            self.get_parameter('map_id_file').value).expanduser()
        self.map_id = (
            map_id_path.read_text(encoding='utf-8').strip()
            if map_id_path.exists() else None
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.health_pub = self.create_publisher(
            String, self.get_parameter('health_topic').value, 10)
        self.create_timer(0.5, self.publish_state)

    def publish_health(self, state, detail=None, stamp=None):
        payload = {
            'schema_version': 1,
            'state': state,
            'map_id': self.map_id,
            'map_frame': self.map_frame,
            'base_frame': self.base_frame,
        }
        if detail:
            payload['detail'] = detail
        if stamp is not None:
            payload['transform_stamp'] = {
                'sec': stamp.sec, 'nanosec': stamp.nanosec}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.health_pub.publish(message)

    def publish_state(self):
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as error:
            self.publish_health('UNAVAILABLE', str(error))
            return
        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)
        # TF availability is not the same as verified scan-match quality. The
        # task layer may require camera/map evidence before accepting a fact.
        self.publish_health('TF_AVAILABLE', stamp=transform.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
