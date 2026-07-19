import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SemanticDetectorStub(Node):
    """Stub that imitates a YOLO+localizer output.

    Replace this with your D435i RGB + YOLO detector later.
    Output contract:
      /semantic_detections_json: [{"room_id":"906","class":"door_front","material":"glass","pose":[x,y,yaw], "conf":0.92}, ...]
    """
    def __init__(self):
        super().__init__('semantic_detector_stub')
        self.target_room = str(self.declare_parameter('target_room', '906').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 1.0).value)
        self.pub = self.create_publisher(String, '/semantic_detections_json', 10)
        self.create_timer(1.0 / self.publish_hz, self.step)
        self.get_logger().info('Semantic detector stub enabled. Disable use_semantic_stub when real YOLO is connected.')

    def step(self):
        rooms = {
            "902": {"material": "wood",  "pose": [2.0, -1.2, 0.0]},
            "904": {"material": "wood",  "pose": [4.0, -1.2, 0.0]},
            "906": {"material": "glass", "pose": [6.0,  1.2, 0.0]},
            "908": {"material": "glass", "pose": [8.0,  1.2, 0.0]},
        }
        detections = []
        for rid, info in rooms.items():
            detections.append({
                "room_id": rid,
                "class": "door_front",
                "material": info["material"],
                "pose": info["pose"],
                "conf": 0.93 if rid == self.target_room else 0.80
            })
        msg = String()
        msg.data = json.dumps(detections, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SemanticDetectorStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
