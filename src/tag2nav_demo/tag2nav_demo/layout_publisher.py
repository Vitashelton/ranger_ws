import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LayoutPublisher(Node):
    def __init__(self):
        super().__init__("tag2nav_layout_publisher")
        self.declare_parameter("layout_file", "selected_layout.json")
        path = self.get_parameter("layout_file").value
        with open(path, "r", encoding="utf-8") as f:
            self.layout = json.load(f)
        self.pub = self.create_publisher(String, "/tag2nav/selected_layout", 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.layout, ensure_ascii=False)))


def main():
    rclpy.init()
    node = LayoutPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
