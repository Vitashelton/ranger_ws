#!/usr/bin/env python3
"""Relay FAST-LIO's body-frame cloud into the physical MID360S TF frame."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudFrameRelay(Node):
    def __init__(self):
        super().__init__('cloud_frame_relay')
        self.declare_parameter('input_topic', '/cloud_registered_body')
        self.declare_parameter('output_topic', '/nav/points')
        self.declare_parameter('output_frame', 'livox_frame')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} as frame {self.output_frame}')

    def callback(self, msg: PointCloud2):
        # FAST-LIO's body cloud is expressed in the co-located LiDAR/IMU frame.
        msg.header.frame_id = self.output_frame
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
