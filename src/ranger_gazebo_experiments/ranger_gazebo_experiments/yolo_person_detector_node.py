#!/usr/bin/env python3
"""
YOLOv8 person detector node for Gazebo simulation experiments.

Subscribes to /camera/color/image_raw and publishes:
  - /yolo/person_detections (Detection2DArray)
  - /yolo/person_debug_image (Image, with bbox overlay)
  - /yolo/person_markers (MarkerArray)

Supports CPU/CUDA, configurable confidence threshold, FPS tracking.
"""
import math
import os
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class YOLOPersonDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_person_detector_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/yolo/person_detections')
        self.declare_parameter('debug_image_topic', '/yolo/person_debug_image')
        self.declare_parameter('markers_topic', '/yolo/person_markers')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('tracker_enabled', False)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('max_image_width', 640)

        p = lambda name: self.get_parameter(name).value
        self.model_path = p('model_path')
        self.conf_thresh = p('confidence_threshold')
        self.iou_thresh = p('iou_threshold')
        self.device = p('device')
        self.image_topic = p('image_topic')
        self.detections_topic = p('detections_topic')
        self.debug_image_topic = p('debug_image_topic')
        self.markers_topic = p('markers_topic')
        self.publish_markers_flag = p('publish_markers')
        self.publish_debug_flag = p('publish_debug_image')
        self.target_class = p('target_class')
        self.tracker_enabled = p('tracker_enabled')
        self.frame_id = p('frame_id')
        self.max_image_width = p('max_image_width')

        self.bridge = CvBridge()
        self._model = None
        self._load_model()

        # State
        self.latest_image = None
        self.latest_camera_info = None
        self.detector_fps = 0.0
        self.detector_latency_ms = 0.0
        self.last_fps_update = time.time()
        self.fps_frame_count = 0
        self.detection_count = 0

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)

        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.debug_pub = None
        if self.publish_debug_flag:
            self.debug_pub = self.create_publisher(
                Image, self.debug_image_topic, 10)
        self.markers_pub = None
        if self.publish_markers_flag:
            self.markers_pub = self.create_publisher(
                MarkerArray, self.markers_topic, 10)

        self.get_logger().info(
            f'yolo_person_detector_node started: model={self.model_path} '
            f'device={self.device} conf={self.conf_thresh}')

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal(
                'ultralytics not installed. Run: pip install ultralytics')
            raise

        if not os.path.isfile(self.model_path):
            self.get_logger().warn(
                f'Model file {self.model_path} not found — YOLO will auto-download')
        self._model = YOLO(self.model_path)
        self._model.to(self.device)
        self.get_logger().info(f'YOLOv8 model loaded on {self.device}')

    def _image_cb(self, msg):
        self.latest_image = msg
        self._process_frame(msg)

    def _camera_info_cb(self, msg):
        self.latest_camera_info = msg

    def _process_frame(self, img_msg):
        if self._model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}', throttle_duration_sec=5.0)
            return

        # Resize if needed
        if self.max_image_width > 0 and cv_image.shape[1] > self.max_image_width:
            scale = self.max_image_width / cv_image.shape[1]
            new_h = int(cv_image.shape[0] * scale)
            cv_image = cv2.resize(cv_image, (self.max_image_width, new_h))

        t0 = time.time()
        results = self._model.predict(
            cv_image, conf=self.conf_thresh, iou=self.iou_thresh,
            device=self.device, verbose=False, classes=[0],  # 0 = person in COCO
        )
        dt = (time.time() - t0) * 1000.0
        self.detector_latency_ms = dt

        # FPS tracking
        self.fps_frame_count += 1
        now = time.time()
        if now - self.last_fps_update >= 1.0:
            self.detector_fps = self.fps_frame_count / (now - self.last_fps_update)
            self.fps_frame_count = 0
            self.last_fps_update = now

        # Parse detections
        detections_array = Detection2DArray()
        detections_array.header = img_msg.header
        detections_array.header.frame_id = self.frame_id

        debug_image = cv_image.copy()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                conf = float(boxes.conf[i].item())
                xyxy = boxes.xyxy[i].tolist()
                x1, y1, x2, y2 = [int(v) for v in xyxy]

                det = Detection2D()
                det.header = img_msg.header
                det.header.frame_id = self.frame_id
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'person'
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det.id = str(i)
                detections_array.detections.append(det)

                # Draw on debug image
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'person {conf:.2f}'
                cv2.putText(debug_image, label, (x1, max(y1 - 5, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.detection_count = len(detections_array.detections)
        self.detections_pub.publish(detections_array)

        # Publish debug image
        if self.debug_pub is not None:
            # Overlay FPS
            cv2.putText(debug_image, f'FPS: {self.detector_fps:.1f} Latency: {dt:.1f}ms',
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = img_msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f'debug image publish failed: {e}', throttle_duration_sec=5.0)

        # Publish markers
        if self.markers_pub is not None:
            self._publish_markers(detections_array, img_msg.header.stamp)

    def _publish_markers(self, detections_array, stamp):
        markers = MarkerArray()
        for i, det in enumerate(detections_array.detections):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'yolo_person'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = det.bbox.center.position.x * 0.001
            m.pose.position.y = det.bbox.center.position.y * 0.001
            m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = det.bbox.size_x * 0.001
            m.scale.y = det.bbox.size_y * 0.001
            m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 0.5
            markers.markers.append(m)
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
