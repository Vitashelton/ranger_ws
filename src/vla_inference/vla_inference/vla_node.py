#!/usr/bin/env python3

import base64
import time
from typing import Optional

import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray


class VLAInferenceNode(Node):
    def __init__(self):
        super().__init__("vla_inference_node")

        # -------- Parameters --------
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("instruction_topic", "/vla/instruction")
        self.declare_parameter("action_topic", "/vla/action")
        self.declare_parameter("api_url", "http://127.0.0.1:8000/predict")
        self.declare_parameter("control_rate_hz", 5.0)
        self.declare_parameter("image_width", 224)
        self.declare_parameter("image_height", 224)
        self.declare_parameter("request_timeout", 5.0)

        # 假设动作维度为 7: dx, dy, dz, droll, dpitch, dyaw, gripper
        self.declare_parameter("action_dim", 7)

        # 简单安全限制，实际部署时按你的机械臂/底盘修改
        self.declare_parameter("max_abs_action", [0.05, 0.05, 0.05, 0.2, 0.2, 0.2, 1.0])
        self.declare_parameter("max_delta_action", [0.02, 0.02, 0.02, 0.1, 0.1, 0.1, 1.0])
        self.declare_parameter("ema_alpha", 0.5)

        self.image_topic = self.get_parameter("image_topic").value
        self.instruction_topic = self.get_parameter("instruction_topic").value
        self.action_topic = self.get_parameter("action_topic").value
        self.api_url = self.get_parameter("api_url").value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.request_timeout = float(self.get_parameter("request_timeout").value)
        self.action_dim = int(self.get_parameter("action_dim").value)

        self.max_abs_action = np.array(
            self.get_parameter("max_abs_action").value,
            dtype=np.float32,
        )
        self.max_delta_action = np.array(
            self.get_parameter("max_delta_action").value,
            dtype=np.float32,
        )
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)

        if len(self.max_abs_action) != self.action_dim:
            raise ValueError("max_abs_action length must equal action_dim")
        if len(self.max_delta_action) != self.action_dim:
            raise ValueError("max_delta_action length must equal action_dim")

        # -------- State --------
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_instruction: str = ""
        self.last_action = np.zeros(self.action_dim, dtype=np.float32)
        self.last_request_time = 0.0

        # 图像一般用 best_effort，避免网络差时阻塞
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )

        self.instruction_sub = self.create_subscription(
            String,
            self.instruction_topic,
            self.instruction_callback,
            10,
        )

        self.action_pub = self.create_publisher(
            Float32MultiArray,
            self.action_topic,
            10,
        )

        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("VLA inference node started")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Instruction topic: {self.instruction_topic}")
        self.get_logger().info(f"Action topic: {self.action_topic}")
        self.get_logger().info(f"OpenVLA API: {self.api_url}")

    def image_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.latest_image = image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def instruction_callback(self, msg: String):
        self.latest_instruction = msg.data.strip()
        self.get_logger().info(f"Received instruction: {self.latest_instruction}")

    def timer_callback(self):
        if self.latest_image is None:
            self.get_logger().warn("Waiting for image...", throttle_duration_sec=2.0)
            return

        if not self.latest_instruction:
            self.get_logger().warn("Waiting for instruction...", throttle_duration_sec=2.0)
            return

        try:
            raw_action = self.call_openvla_api(
                image=self.latest_image,
                instruction=self.latest_instruction,
            )
            safe_action = self.safety_filter(raw_action)
            self.publish_action(safe_action)

        except Exception as e:
            self.get_logger().error(f"VLA inference failed: {e}")

    def encode_image(self, image: np.ndarray) -> str:
        resized = cv2.resize(image, (self.image_width, self.image_height))
        bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)

        ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            raise RuntimeError("Failed to encode image")

        return base64.b64encode(buffer).decode("utf-8")

    def call_openvla_api(self, image: np.ndarray, instruction: str) -> np.ndarray:
        image_b64 = self.encode_image(image)

        payload = {
            "instruction": instruction,
            "image": image_b64,
            "timestamp": time.time(),
        }

        response = requests.post(
            self.api_url,
            json=payload,
            timeout=self.request_timeout,
        )
        response.raise_for_status()

        data = response.json()

        if "action" not in data:
            raise RuntimeError(f"API response has no action field: {data}")

        action = np.array(data["action"], dtype=np.float32)

        if action.shape[0] != self.action_dim:
            raise RuntimeError(
                f"Invalid action dim: got {action.shape[0]}, expected {self.action_dim}"
            )

        if not np.all(np.isfinite(action)):
            raise RuntimeError("Action contains NaN or Inf")

        return action

    def safety_filter(self, action: np.ndarray) -> np.ndarray:
        # 1. 绝对动作限幅
        action = np.clip(action, -self.max_abs_action, self.max_abs_action)

        # 2. 单步变化限幅，避免动作突变
        delta = action - self.last_action
        delta = np.clip(delta, -self.max_delta_action, self.max_delta_action)
        action = self.last_action + delta

        # 3. EMA 平滑
        action = self.ema_alpha * action + (1.0 - self.ema_alpha) * self.last_action

        self.last_action = action.astype(np.float32)
        return self.last_action

    def publish_action(self, action: np.ndarray):
        msg = Float32MultiArray()
        msg.data = action.astype(np.float32).tolist()
        self.action_pub.publish(msg)

        self.get_logger().info(
            f"Published action: {np.array2string(action, precision=3)}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = VLAInferenceNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
