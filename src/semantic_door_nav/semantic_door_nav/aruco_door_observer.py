#!/usr/bin/env python3
"""
Camera -> ArUco room-tag observer -> /door_observation

Output format (std_msgs/String JSON):
{"door_id":"904","confidence":0.94,"side":"right","marker_id":904}

The marker IDs in simulation can equal the room numbers for a simple demo.
On real hardware, replace this node with a door-plate detector + OCR node
that publishes the same JSON contract.
"""
import json
import math

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoDoorObserver(Node):
    def __init__(self):
        super().__init__('aruco_door_observer')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('door_observation_topic', '/door_observation')
        self.declare_parameter('dictionary_name', 'DICT_6X6_1000')
        self.declare_parameter('marker_to_room_json', '{"902":"902","904":"904","100":"START","101":"CORRIDOR_TURN"}')
        self.declare_parameter('min_marker_area_px', 350.0)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('door_observation_topic').value
        dictionary_name = self.get_parameter('dictionary_name').value
        self.min_marker_area_px = float(self.get_parameter('min_marker_area_px').value)

        try:
            self.marker_to_room = json.loads(self.get_parameter('marker_to_room_json').value)
        except json.JSONDecodeError:
            self.get_logger().warn('marker_to_room_json is invalid; using an empty map.')
            self.marker_to_room = {}

        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f'OpenCV ArUco dictionary not found: {dictionary_name}')

        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.subscription = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.get_logger().info(
            f'ArUco observer active. image={image_topic}, output={output_topic}, map={self.marker_to_room}'
        )

    @staticmethod
    def polygon_area(corners: np.ndarray) -> float:
        return abs(cv2.contourArea(corners.astype(np.float32)))

    def on_image(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        h, w = gray.shape[:2]
        candidates = []

        for c, marker_id_arr in zip(corners, ids):
            marker_id = int(marker_id_arr[0])
            room = self.marker_to_room.get(str(marker_id))
            if room is None:
                continue

            quad = c.reshape(4, 2)
            area = self.polygon_area(quad)
            if area < self.min_marker_area_px:
                continue

            cx = float(np.mean(quad[:, 0]))
            if cx < 0.45 * w:
                side = 'left'
            elif cx > 0.55 * w:
                side = 'right'
            else:
                side = 'center'

            # A deliberately simple confidence surrogate for a simulation demo.
            # Real OCR should publish its own calibrated recognition confidence.
            normalized = min(1.0, area / max(1.0, 0.10 * w * h))
            confidence = float(max(0.55, min(0.99, 0.55 + 0.44 * normalized)))
            candidates.append((area, room, confidence, side, marker_id))

        if not candidates:
            return

        _, room, confidence, side, marker_id = max(candidates, key=lambda x: x[0])
        out = {
            'door_id': str(room),
            'confidence': round(confidence, 3),
            'side': side,
            'marker_id': marker_id,
        }
        ros_msg = String()
        ros_msg.data = json.dumps(out, ensure_ascii=False)
        self.publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDoorObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
