#!/usr/bin/env python3
"""tca_bev_fusion_node.

v1 path: LiDAR-only BEV with an explicit unknown mask and the conservative
fusion rule already wired in (depth channels stay empty until the RGB-D path is
implemented). This lets us validate the full plumbing — grid, confidence,
fusion, debug image, status — before adding the harder depth projection.

Subscribes:
    /livox/lidar    (PointCloud2 or livox CustomMsg)
    /odom           (nav_msgs/Odometry)        -> pose-anchor quality
    /time_align/status      (std_msgs/String)  -> time confidence
    /calibration/status     (std_msgs/String)  -> calib confidence + inflation
    (tf via tf2 for depth->base once RGB-D path lands)

Publishes:
    /bev/tensor        (std_msgs/Float32MultiArray)  HxWxC fused stack
    /bev/image_debug   (sensor_msgs/Image, bgr8)     colourised occ/free/unknown
    /bev/status        (std_msgs/String, JSON)
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry

from tca_bev_nav.bev.bev_grid import (BEVConfig, BEVGrid, OCC_LIDAR,
                                      points_to_lidar_evidence)
from tca_bev_nav.bev.conservative_fusion import (FusionParams,
                                                 ModalityConfidence,
                                                 conservative_fuse)
from tca_bev_nav.bev.confidence import PoseAnchorParams, pose_anchor_quality

try:
    from livox_ros_driver2.msg import CustomMsg  # type: ignore
    _HAS_LIVOX = True
except Exception:  # pragma: no cover
    _HAS_LIVOX = False

try:
    from cv_bridge import CvBridge
    _HAS_CVBRIDGE = True
except Exception:  # pragma: no cover
    _HAS_CVBRIDGE = False


class TCABEVFusionNode(Node):
    def __init__(self):
        super().__init__('tca_bev_fusion_node')
        self.declare_parameter('range_m', 8.0)
        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('update_period_s', 0.1)
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('lidar_is_custom_msg', _HAS_LIVOX)
        self.declare_parameter('publish_debug_image', True)

        self.cfg = BEVConfig(
            range_m=self.get_parameter('range_m').value,
            resolution_m=self.get_parameter('resolution_m').value,
        )
        self.grid = BEVGrid(self.cfg)
        self.fusion_p = FusionParams()
        self.pose_p = PoseAnchorParams()

        # Latest auxiliary state.
        self._latest_points = None
        self._time_conf = {'conf_depth': 1.0, 'conf_odom': 1.0}
        self._calib_conf = 1.0
        self._inflation_m = self.cfg.resolution_m
        self._odom_v = 0.0
        self._odom_w = 0.0

        lidar_topic = self.get_parameter('lidar_topic').value
        if self.get_parameter('lidar_is_custom_msg').value and _HAS_LIVOX:
            self.create_subscription(CustomMsg, lidar_topic,
                                     self._on_custom_lidar, 10)
        else:
            self.create_subscription(PointCloud2, lidar_topic,
                                     self._on_pc2_lidar, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(String, '/time_align/status',
                                 self._on_time, 10)
        self.create_subscription(String, '/calibration/status',
                                 self._on_calib, 10)

        self.pub_tensor = self.create_publisher(Float32MultiArray,
                                                '/bev/tensor', 1)
        self.pub_status = self.create_publisher(String, '/bev/status', 1)
        self.pub_debug = self.create_publisher(Image, '/bev/image_debug', 1)
        self._bridge = CvBridge() if _HAS_CVBRIDGE else None

        self.create_timer(self.get_parameter('update_period_s').value,
                          self._update)
        self.get_logger().info('tca_bev_fusion_node up (LiDAR-only v1).')

    # -- callbacks ----------------------------------------------------------
    def _on_pc2_lidar(self, msg: PointCloud2):
        pts = np.array(
            [[p[0], p[1], p[2]] for p in
             pc2.read_points(msg, field_names=('x', 'y', 'z'),
                             skip_nans=True)],
            dtype=np.float32,
        ) if msg.width * msg.height else np.empty((0, 3), np.float32)
        self._latest_points = pts

    def _on_custom_lidar(self, msg):
        # Livox CustomMsg: list of CustomPoint with x,y,z in metres.
        pts = np.array([[pt.x, pt.y, pt.z] for pt in msg.points],
                       dtype=np.float32) if msg.points \
            else np.empty((0, 3), np.float32)
        self._latest_points = pts

    def _on_odom(self, msg: Odometry):
        self._odom_v = abs(msg.twist.twist.linear.x)
        self._odom_w = abs(msg.twist.twist.angular.z)

    def _on_time(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._time_conf['conf_depth'] = float(d.get('conf_depth') or 0.0)
            self._time_conf['conf_odom'] = float(d.get('conf_odom') or 1.0)
        except Exception:
            pass

    def _on_calib(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._calib_conf = float(d.get('calib_conf', 1.0))
            self._inflation_m = float(d.get('inflation_radius_m',
                                            self.cfg.resolution_m))
        except Exception:
            pass

    # -- main loop ----------------------------------------------------------
    def _update(self):
        self.grid.reset()
        if self._latest_points is not None:
            points_to_lidar_evidence(self._latest_points, self.grid)
        # TODO(rgb-d): project /camera/depth/image_rect_raw into OCC_DEPTH /
        # FREE_DEPTH using camera_info + depth->base tf. Gate with calib_conf.

        pa = pose_anchor_quality(self._odom_v, self._odom_w,
                                 tf_latency_s=0.0, p=self.pose_p)
        conf = ModalityConfidence(
            time_lidar=1.0,
            time_depth=self._time_conf['conf_depth'],
            calib_depth=self._calib_conf,
            pose_anchor=pa,
        )
        fused = conservative_fuse(self.grid, conf, self.fusion_p)
        fused = self._apply_inflation(fused)

        self._publish_tensor(fused)
        self._publish_status(fused, conf, pa)
        if self.get_parameter('publish_debug_image').value:
            self._publish_debug(fused)

    def _apply_inflation(self, fused: dict) -> dict:
        """Dilate occupied cells by the calibration-driven inflation radius.

        Pure-numpy box dilation to avoid an OpenCV hard dependency in v1.
        TODO: replace with a circular structuring element for exact radius.
        """
        rad_cells = max(1, int(round(self._inflation_m / self.cfg.resolution_m)))
        occ = fused['occ']
        mask = occ >= 0.5
        if rad_cells > 0 and mask.any():
            dil = mask.copy()
            for _ in range(rad_cells):
                d = dil.copy()
                d[1:, :] |= dil[:-1, :]
                d[:-1, :] |= dil[1:, :]
                d[:, 1:] |= dil[:, :-1]
                d[:, :-1] |= dil[:, 1:]
                dil = d
            inflated = np.where(dil & (occ < 0.5), 0.6, occ).astype(np.float32)
            # Inflation must not turn unknown into free.
            fused['occ'] = np.maximum(occ, inflated)
        return fused

    def _publish_tensor(self, fused: dict):
        stack = np.stack([fused['occ'], fused['free'], fused['unknown']],
                         axis=-1).astype(np.float32)
        m = Float32MultiArray()
        h, w, c = stack.shape
        for label, n in (('h', h), ('w', w), ('c', c)):
            dim = MultiArrayDimension()
            dim.label, dim.size = label, n
            m.layout.dim.append(dim)
        m.data = stack.reshape(-1).tolist()
        self.pub_tensor.publish(m)

    def _publish_status(self, fused, conf, pa):
        occ_cells = int((fused['occ'] >= 0.5).sum())
        unk_ratio = float((fused['unknown'] > 0.5).mean())
        msg = String()
        msg.data = json.dumps({
            'occ_cells': occ_cells,
            'unknown_ratio': unk_ratio,
            'calib_conf': self._calib_conf,
            'inflation_m': self._inflation_m,
            'pose_anchor': pa,
            'time_depth_conf': conf.time_depth,
            'mode': 'lidar_only_v1',
        })
        self.pub_status.publish(msg)

    def _publish_debug(self, fused):
        if self._bridge is None:
            return
        n = self.cfg.size
        img = np.zeros((n, n, 3), dtype=np.uint8)
        img[fused['unknown'] > 0.5] = (60, 60, 60)      # grey = unknown
        img[fused['free'] >= 0.5] = (0, 140, 0)          # green = free
        img[fused['occ'] >= 0.5] = (0, 0, 220)           # red = occupied
        img = np.flipud(img)  # x forward shown upward
        self.pub_debug.publish(self._bridge.cv2_to_imgmsg(img, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = TCABEVFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
