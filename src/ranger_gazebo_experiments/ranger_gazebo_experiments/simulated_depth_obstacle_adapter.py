#!/usr/bin/env python3
"""
Simulation depth camera to obstacle pipeline adapter (stub).

In simulation, the d435i_obstacle_node can be configured directly via parameters.
This adapter exists for cases where format conversion or topic bridging is needed.

Currently a pass-through for depth pointcloud data.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class SimulatedDepthObstacleAdapter(Node):
    def __init__(self):
        super().__init__('simulated_depth_obstacle_adapter')
        self.declare_parameter('input_topic', '/camera/depth/color/points')
        self.declare_parameter('output_topic', '/camera/depth/color/points')
        self.declare_parameter('passthrough', True)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(PointCloud2, in_topic, self._cb, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.get_logger().info(f'Depth adapter: {in_topic} -> {out_topic}')

    def _cb(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDepthObstacleAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
