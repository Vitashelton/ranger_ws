#!/usr/bin/env python3
"""最小 BEV 生成节点 —— 从 /scan (LaserScan) 投影到 2D 占据 grid.

在 tca_bev_nav 跑通之前的 stand-in 实现:
  订阅 /scan -> 投影到 base_link 中心 grid -> Float32MultiArray -> /bev/tensor

Grid 参数 (可配):
  range_m:       8.0 m (half-extent, 对应 16m x 16m 视场)
  resolution_m:  0.05 m/px -> 320x320 grid
  3-channel: [occ, free, unknown] (float32 [0,1])

注意: 这是极简实现, 仅将 scan 点标记为 occupied=1.0, 不做 ray-casting free-space.
未来替换为 tca_bev_nav 的完整 LiDAR + depth BEV 融合。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Header


class BevFromScan(Node):
    def __init__(self):
        super().__init__("bev_from_scan")

        # ---- 参数 ----
        self.declare_parameter("range_m", 8.0)
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("publish_topic", "/bev/tensor")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("min_range", 0.3)
        self.declare_parameter("max_range", 15.0)
        self.declare_parameter("rate_hz", 10.0)

        self.range_m = float(self.get_parameter("range_m").value)
        self.res_m = float(self.get_parameter("resolution_m").value)

        # grid 尺寸
        self.grid_size = int(2.0 * self.range_m / self.res_m)
        self.get_logger().info(
            f"BEV grid: {self.grid_size}x{self.grid_size} "
            f"({self.range_m*2:.0f}m x {self.range_m*2:.0f}m @ {self.res_m:.2f}m/px)")

        # ---- 订阅 & 发布 ----
        self.create_subscription(LaserScan, "/scan", self.on_scan, 5)
        self.pub = self.create_publisher(
            Float32MultiArray, self.get_parameter("publish_topic").value, 1)

        if self.get_parameter("publish_debug_image").value:
            self.pub_img = self.create_publisher(
                # sensor_msgs/Image would need cv_bridge; skip for MVP
                Float32MultiArray, "/bev/image_debug", 1)
        else:
            self.pub_img = None

    def on_scan(self, msg: LaserScan):
        """投影 LaserScan -> 3-channel BEV tensor."""
        min_r = float(self.get_parameter("min_range").value)
        max_r = float(self.get_parameter("max_range").value)

        # 3 channels: occ, free, unknown
        grid = np.zeros((3, self.grid_size, self.grid_size), dtype=np.float32)
        # unknown = 1.0 everywhere initially
        grid[2, :, :] = 1.0

        center = self.grid_size // 2

        angle = msg.angle_min
        for r in msg.ranges:
            if min_r < r < max_r:
                # base_link 系: x 向前, y 向左
                bx = r * math.cos(angle)
                by = r * math.sin(angle)

                # grid 坐标: row = y (从 center 向上为负), col = x (从 center 向右为正)
                col = int(center + bx / self.res_m)
                row = int(center - by / self.res_m)  # grid row: +y -> -row

                if 0 <= col < self.grid_size and 0 <= row < self.grid_size:
                    grid[0, row, col] = 1.0   # occ
                    grid[2, row, col] = 0.0   # not unknown

            angle += msg.angle_increment

        # 发布 flatten tensor
        msg_out = Float32MultiArray()
        msg_out.data = grid.flatten().tolist()
        # 布局信息: 在 data 前加一小段 header (不是标准做法, 但保持一致)
        # tca_bev_nav 的 /bev/tensor 约定为 [C*H*W] flatten
        self.pub.publish(msg_out)


def main():
    rclpy.init()
    node = BevFromScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
