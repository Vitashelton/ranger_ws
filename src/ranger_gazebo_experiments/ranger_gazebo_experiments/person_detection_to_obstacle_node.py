#!/usr/bin/env python3
"""
Converts YOLO 2D person detections to 3D obstacle markers for fusion pipeline.

Subscribes to /yolo/person_detections and uses depth image + camera info
to project 2D bbox centers to 3D camera coordinates.

Publishes /obstacles_yolo_person (MarkerArray) — compatible with sensor_fusion_node.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from .utils_geometry import project_bbox_to_3d, euler_to_quaternion


class PersonDetectionToObstacleNode(Node):
    def __init__(self):
        super().__init__('person_detection_to_obstacle_node')

        self.declare_parameter('detections_topic', '/yolo/person_detections')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('obstacles_topic', '/obstacles_yolo_person')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('use_ground_truth_depth', False)
        self.declare_parameter('default_person_confidence', 0.75)

        p = lambda n: self.get_parameter(n).value
        self.frame_id = p('frame_id')
        self.obstacles_topic = p('obstacles_topic')
        self.use_gt_depth = p('use_ground_truth_depth')
        self.confidence = p('default_person_confidence')

        self.bridge = CvBridge()
        self.latest_detections = None
        self.latest_depth = None
        self.latest_camera_info = None

        self.det_sub = self.create_subscription(
            Detection2DArray, p('detections_topic'), self._det_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, p('depth_image_topic'), self._depth_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, p('camera_info_topic'), self._info_cb, 10)

        self.obs_pub = self.create_publisher(
            MarkerArray, self.obstacles_topic, 10)
        self.timer = self.create_timer(
            1.0 / p('publish_rate'), self._publish_obstacles)

        self.get_logger().info(
            f'person_detection_to_obstacle_node started -> {self.obstacles_topic}')

    def _det_cb(self, msg):
        self.latest_detections = msg

    def _depth_cb(self, msg):
        self.latest_depth = msg

    def _info_cb(self, msg):
        self.latest_camera_info = msg

    def _publish_obstacles(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        if self.latest_detections is None:
            self.obs_pub.publish(markers)
            return

        for i, det in enumerate(self.latest_detections.detections):
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            sx = det.bbox.size_x
            sy = det.bbox.size_y
            conf = det.results[0].hypothesis.score if det.results else self.confidence

            x1, y1 = cx - sx / 2, cy - sy / 2
            x2, y2 = cx + sx / 2, cy + sy / 2

            pos_3d = None
            if self.latest_depth is not None and self.latest_camera_info is not None:
                try:
                    depth_img = self.bridge.imgmsg_to_cv2(
                        self.latest_depth, desired_encoding='passthrough')
                    bbox_xyxy = [x1, y1, x2, y2]
                    pos_3d = project_bbox_to_3d(
                        bbox_xyxy, depth_img, self.latest_camera_info)
                except Exception as e:
                    self.get_logger().debug(f'Depth projection failed: {e}')

            if pos_3d is None:
                continue

            px, py, pz = pos_3d

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'yolo_person_obstacle'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            q = euler_to_quaternion(0.0, 0.0, 0.0)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 1.75
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = float(conf)
            markers.markers.append(m)

        self.obs_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionToObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
