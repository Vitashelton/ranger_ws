#!/usr/bin/env python3
"""Publish bounded RGB-D ArUco observations as task evidence.

This node recognizes physical marker IDs and estimates their center depth. It
does not publish velocity, alter TF, infer arbitrary objects, or treat a missed
frame as evidence that an entity is absent.
"""

import json
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class D435iMarkerEvidence(Node):
    def __init__(self):
        super().__init__('d435i_marker_evidence')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('evidence_topic', '/evidence/room_marker')
        self.declare_parameter(
            'marker_config', '~/.config/ranger_nav/room_markers.yaml')
        self.declare_parameter('dictionary', 'DICT_6X6_1000')
        self.declare_parameter('min_marker_area_px', 250.0)
        self.declare_parameter('max_depth_age_s', 0.25)
        self.declare_parameter('publish_unmapped', True)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = 0.0
        self.min_area = float(self.get_parameter('min_marker_area_px').value)
        self.max_depth_age = float(self.get_parameter('max_depth_age_s').value)
        self.publish_unmapped = bool(
            self.get_parameter('publish_unmapped').value)

        config_path = Path(
            self.get_parameter('marker_config').value).expanduser()
        config = {}
        if config_path.exists():
            with config_path.open('r', encoding='utf-8') as stream:
                config = yaml.safe_load(stream) or {}
        else:
            self.get_logger().warn(
                f'{config_path} not found; publishing observed marker IDs unmapped')

        self.marker_map = {
            str(key): value for key, value in (config.get('markers') or {}).items()
        }
        dictionary_name = str(
            config.get('dictionary')
            or self.get_parameter('dictionary').value)
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f'unknown ArUco dictionary: {dictionary_name}')
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name))
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.dictionary_name = dictionary_name

        self.publisher = self.create_publisher(
            String, self.get_parameter('evidence_topic').value, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self.on_color, qos_profile_sensor_data)
        self.get_logger().info(
            f'RGB-D marker evidence active; dictionary={dictionary_name}, '
            f'configured_markers={len(self.marker_map)}')

    def on_depth(self, message):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='passthrough')
            self.latest_depth_stamp = stamp_seconds(message.header.stamp)
        except Exception as error:
            self.get_logger().warn(f'depth conversion failed: {error}')

    def center_depth_m(self, center_x, center_y, color_stamp):
        if self.latest_depth is None:
            return None, 0
        if abs(color_stamp - self.latest_depth_stamp) > self.max_depth_age:
            return None, 0
        image = self.latest_depth
        x = int(round(center_x))
        y = int(round(center_y))
        if x < 0 or y < 0 or y >= image.shape[0] or x >= image.shape[1]:
            return None, 0
        y0, y1 = max(0, y - 3), min(image.shape[0], y + 4)
        x0, x1 = max(0, x - 3), min(image.shape[1], x + 4)
        values = np.asarray(image[y0:y1, x0:x1], dtype=np.float64)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            return None, 0
        # RealSense 16UC1 is millimetres; 32FC1 is metres.
        if image.dtype == np.uint16:
            values = values * 0.001
        values = values[(values >= 0.15) & (values <= 20.0)]
        if values.size == 0:
            return None, 0
        return round(float(np.median(values)), 3), int(values.size)

    def on_color(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warn(f'color conversion failed: {error}')
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return
        color_stamp = stamp_seconds(message.header.stamp)
        for marker_corners, marker_array in zip(corners, ids):
            marker_id = int(marker_array[0])
            metadata = self.marker_map.get(str(marker_id))
            if metadata is None and not self.publish_unmapped:
                continue
            quad = marker_corners.reshape(4, 2)
            area = abs(float(cv2.contourArea(quad.astype(np.float32))))
            if not math.isfinite(area) or area < self.min_area:
                continue
            center_x, center_y = np.mean(quad, axis=0)
            depth_m, depth_samples = self.center_depth_m(
                center_x, center_y, color_stamp)
            evidence = {
                'schema_version': 1,
                'predicate': 'marker_visible',
                'value': True,
                'source': 'd435i_aruco',
                'stamp': {
                    'sec': message.header.stamp.sec,
                    'nanosec': message.header.stamp.nanosec,
                },
                'marker_id': marker_id,
                'entity_id': (metadata or {}).get('entity_id'),
                'role': (metadata or {}).get('role'),
                'range_m': depth_m,
                'quality': {
                    'area_px': round(area, 1),
                    'depth_samples': depth_samples,
                    'dictionary': self.dictionary_name,
                },
            }
            output = String()
            output.data = json.dumps(evidence, ensure_ascii=False)
            self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = D435iMarkerEvidence()
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
