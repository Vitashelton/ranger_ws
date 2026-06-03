#!/usr/bin/env python3
"""calibration_uncertainty_node.

Loads hand-measured extrinsics and their *honest* uncertainties from
config/extrinsics.yaml, computes a calibration confidence, and publishes it so
the fusion node can grow its inflation radius when trust is low.

This node makes NO claim of accurate calibration-free fusion. It encodes how
poorly we know the extrinsics and lets the system behave conservatively.

Publishes:
    /calibration/status  (std_msgs/String, JSON payload)
"""
from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tca_bev_nav.bev.confidence import (CalibConfidenceParams,
                                        calibration_confidence,
                                        inflation_radius)


class CalibrationUncertaintyNode(Node):
    def __init__(self):
        super().__init__('calibration_uncertainty_node')
        # Extrinsic uncertainty of the camera w.r.t. the LiDAR / base frame.
        # These are operator-provided estimates from hand measurement.
        self.declare_parameter('cam_rot_std_deg', 5.0)
        self.declare_parameter('cam_trans_std_m', 0.05)
        self.declare_parameter('sigma_rot_deg', 5.0)
        self.declare_parameter('sigma_trans_m', 0.05)
        self.declare_parameter('base_inflation_m', 0.15)
        self.declare_parameter('max_extra_inflation_m', 0.35)
        self.declare_parameter('publish_period_s', 0.5)

        self._cp = CalibConfidenceParams(
            sigma_rot=math.radians(self.get_parameter('sigma_rot_deg').value),
            sigma_trans=self.get_parameter('sigma_trans_m').value,
        )
        self._pub = self.create_publisher(String, '/calibration/status', 10)
        self.create_timer(self.get_parameter('publish_period_s').value,
                          self._tick)
        self.get_logger().info('calibration_uncertainty_node up.')

    def compute(self):
        rot_std = math.radians(self.get_parameter('cam_rot_std_deg').value)
        trans_std = self.get_parameter('cam_trans_std_m').value
        c_c = calibration_confidence(rot_std, trans_std, self._cp)
        r = inflation_radius(
            self.get_parameter('base_inflation_m').value,
            c_c,
            self.get_parameter('max_extra_inflation_m').value,
        )
        return c_c, r

    def _tick(self):
        c_c, r = self.compute()
        msg = String()
        msg.data = json.dumps({
            'calib_conf': c_c,
            'inflation_radius_m': r,
            'cam_rot_std_deg': self.get_parameter('cam_rot_std_deg').value,
            'cam_trans_std_m': self.get_parameter('cam_trans_std_m').value,
        })
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationUncertaintyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
