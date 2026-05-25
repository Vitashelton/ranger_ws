#!/usr/bin/env python3
"""
Simulation LiDAR to obstacle pipeline adapter (stub).

In simulation, the obstacle_cluster_node is configured directly to subscribe
to /livox/lidar via the 'input_topic' parameter. This adapter exists as a
bridge for cases where topic republishing or format conversion is needed.

Currently a pass-through: subscribes /livox/lidar and can republish to
/cloud_registered for backward compatibility.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class SimulatedLidarObstacleAdapter(Node):
    def __init__(self):
        super().__init__('simulated_lidar_obstacle_adapter')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/cloud_registered')
        self.declare_parameter('passthrough', True)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.passthrough = self.get_parameter('passthrough').value

        self.sub = self.create_subscription(PointCloud2, in_topic, self._cb, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.get_logger().info(
            f'Adapter: {in_topic} -> {out_topic} (passthrough={self.passthrough})')

    def _cb(self, msg):
        if self.passthrough:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedLidarObstacleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
