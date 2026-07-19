import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, Image
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None


class LocalBevRiskNode(Node):
    """Build a lightweight local BEV risk grid from /scan and/or /livox/lidar.

    This is the real-sensor-ready replacement for the fake S1/S2/S3 obstacles used in v4.x.
    It is deliberately simple and robust for early RangerMini tests:
    - LaserScan points are assumed in base_link or approximately aligned with base_link.
    - PointCloud2 uses XYZ directly; add TF compensation later when extrinsics are finalized.
    """

    def __init__(self):
        super().__init__('local_bev_risk_node')

        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/livox/lidar').value
        self.risk_grid_topic = self.declare_parameter('risk_grid_topic', '/local_risk_grid').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.grid_width_m = float(self.declare_parameter('grid_width_m', 8.0).value)
        self.grid_height_m = float(self.declare_parameter('grid_height_m', 8.0).value)
        self.resolution = float(self.declare_parameter('resolution', 0.05).value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 10.0).value)
        self.use_scan = bool(self.declare_parameter('use_scan', True).value)
        self.use_cloud = bool(self.declare_parameter('use_cloud', True).value)
        self.max_range_m = float(self.declare_parameter('max_range_m', 6.0).value)
        self.min_range_m = float(self.declare_parameter('min_range_m', 0.08).value)
        self.hmin = float(self.declare_parameter('obstacle_height_min', 0.05).value)
        self.hmax = float(self.declare_parameter('obstacle_height_max', 1.80).value)
        self.downsample = max(1, int(self.declare_parameter('cloud_downsample_step', 5).value))
        self.inflation_radius_m = float(self.declare_parameter('inflation_radius_m', 0.28).value)
        self.lethal_radius_m = float(self.declare_parameter('lethal_radius_m', 0.16).value)
        self.temporal_decay = float(self.declare_parameter('temporal_decay', 0.92).value)
        self.publish_bev_image = bool(self.declare_parameter('publish_bev_image', True).value)
        self.publish_markers = bool(self.declare_parameter('publish_markers', True).value)

        self.w = int(round(self.grid_width_m / self.resolution))
        self.h = int(round(self.grid_height_m / self.resolution))
        self.cx = self.w // 2
        self.cy = self.h // 2
        self.grid = [0.0] * (self.w * self.h)
        self.obstacle_points: List[Tuple[float, float]] = []
        self.min_distance = 99.0

        if self.use_scan:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        if self.use_cloud:
            self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, 5)

        self.grid_pub = self.create_publisher(OccupancyGrid, self.risk_grid_topic, 10)
        self.min_pub = self.create_publisher(Float32, '/min_distance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/risk_markers', 10)
        self.image_pub = self.create_publisher(Image, '/debug/bev_image', 5)

        self.create_timer(1.0 / self.publish_hz, self.publish)
        self.get_logger().info(
            f'Local BEV risk node ready: scan={self.scan_topic}, cloud={self.cloud_topic}, grid={self.grid_width_m}x{self.grid_height_m}m'
        )

    def world_to_cell(self, x: float, y: float):
        ix = int(self.cx + x / self.resolution)
        iy = int(self.cy + y / self.resolution)
        if 0 <= ix < self.w and 0 <= iy < self.h:
            return ix, iy
        return None

    def cell_index(self, ix: int, iy: int):
        return iy * self.w + ix

    def decay_grid(self):
        d = self.temporal_decay
        self.grid = [v * d for v in self.grid]

    def add_obstacle(self, x: float, y: float):
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        r = math.hypot(x, y)
        if r < self.min_range_m or r > self.max_range_m:
            return
        cell = self.world_to_cell(x, y)
        if cell is None:
            return
        self.obstacle_points.append((x, y))
        self.min_distance = min(self.min_distance, r)

        ix0, iy0 = cell
        infl_cells = max(1, int(self.inflation_radius_m / self.resolution))
        lethal_cells = max(1, int(self.lethal_radius_m / self.resolution))
        for dy in range(-infl_cells, infl_cells + 1):
            for dx in range(-infl_cells, infl_cells + 1):
                ix = ix0 + dx
                iy = iy0 + dy
                if not (0 <= ix < self.w and 0 <= iy < self.h):
                    continue
                dist = math.hypot(dx, dy)
                if dist > infl_cells:
                    continue
                if dist <= lethal_cells:
                    val = 100.0
                else:
                    # smooth inflation: high near obstacle, lower outward
                    val = 35.0 + 60.0 * (1.0 - (dist - lethal_cells) / max(1.0, infl_cells - lethal_cells))
                idx = self.cell_index(ix, iy)
                if val > self.grid[idx]:
                    self.grid[idx] = min(100.0, val)

    def scan_cb(self, msg: LaserScan):
        self.decay_grid()
        self.obstacle_points = []
        self.min_distance = 99.0
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                self.add_obstacle(x, y)
            angle += msg.angle_increment

    def cloud_cb(self, msg: PointCloud2):
        if point_cloud2 is None:
            self.get_logger().warn('sensor_msgs_py.point_cloud2 is not available; cloud input ignored.', throttle_duration_sec=5.0)
            return
        count = 0
        try:
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                count += 1
                if count % self.downsample != 0:
                    continue
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if self.hmin <= z <= self.hmax:
                    self.add_obstacle(x, y)
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 parse failed: {e}', throttle_duration_sec=2.0)

    def publish(self):
        now = self.get_clock().now().to_msg()

        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.base_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = -self.grid_width_m / 2.0
        msg.info.origin.position.y = -self.grid_height_m / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(max(0, min(100, round(v)))) for v in self.grid]
        self.grid_pub.publish(msg)

        md = Float32()
        md.data = float(self.min_distance if self.min_distance < 90 else -1.0)
        self.min_pub.publish(md)

        if self.publish_markers:
            self.publish_marker(now)
        if self.publish_bev_image:
            self.publish_image(now)

    def publish_marker(self, stamp):
        arr = MarkerArray()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.base_frame
        m.ns = 'risk_points'
        m.id = 1
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.045
        m.scale.y = 0.045
        m.color.r = 1.0
        m.color.g = 0.15
        m.color.b = 0.10
        m.color.a = 0.70
        for x, y in self.obstacle_points[:2000]:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.04
            m.points.append(p)
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def publish_image(self, stamp):
        # RGB image, robot centered. Red = risk, green = free.
        data = bytearray(self.w * self.h * 3)
        for iy in range(self.h):
            for ix in range(self.w):
                v = int(max(0, min(100, self.grid[self.cell_index(ix, iy)])))
                off = ((self.h - 1 - iy) * self.w + ix) * 3
                data[off + 0] = min(255, int(v * 2.55))
                data[off + 1] = min(255, int((100 - v) * 1.2))
                data[off + 2] = 20
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.base_frame
        img.height = self.h
        img.width = self.w
        img.encoding = 'rgb8'
        img.is_bigendian = False
        img.step = self.w * 3
        img.data = bytes(data)
        self.image_pub.publish(img)


def main():
    rclpy.init()
    node = LocalBevRiskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
