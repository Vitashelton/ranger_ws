import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw):
    import math
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class SemanticMemoryNode(Node):
    """Lightweight semantic target memory.

    Real input can be YOLO/localizer JSON. Without detections, fallback rooms_json is used.
    """

    def __init__(self):
        super().__init__('semantic_memory_node')
        self.target_room = str(self.declare_parameter('target_room', '906').value)
        self.target_frame = str(self.declare_parameter('target_frame', 'odom').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 5.0).value)
        self.rooms_json = str(self.declare_parameter('rooms_json', '{}').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.rooms = {}
        self.detections = {}
        self.last_odom = None

        try:
            self.rooms = json.loads(self.rooms_json)
        except Exception:
            self.get_logger().warn('rooms_json parse failed, using empty fallback.')

        self.create_subscription(String, '/semantic_detections_json', self.det_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.target_pub = self.create_publisher(PoseStamped, '/semantic_target_pose', 10)
        self.debug_pub = self.create_publisher(String, '/semantic_memory_debug', 10)
        self.create_timer(1.0 / self.publish_hz, self.publish_target)
        self.get_logger().info(f'Semantic memory ready, target_room={self.target_room}')

    def odom_cb(self, msg):
        self.last_odom = msg

    def det_cb(self, msg):
        try:
            items = json.loads(msg.data)
            if isinstance(items, dict):
                items = [items]
            for d in items:
                rid = str(d.get('room_id', ''))
                if rid:
                    self.detections[rid] = d
        except Exception as e:
            self.get_logger().warn(f'semantic JSON parse failed: {e}', throttle_duration_sec=2.0)

    def select_target(self):
        # Prefer latest detector/localizer pose, then fallback configured semantic map.
        if self.target_room in self.detections:
            d = self.detections[self.target_room]
            pose = d.get('pose', None)
            if pose and len(pose) >= 2:
                yaw = float(pose[2]) if len(pose) > 2 else 0.0
                return float(pose[0]), float(pose[1]), yaw, d

        if self.target_room in self.rooms:
            r = self.rooms[self.target_room]
            pose = r.get('door_front', [0.0, 0.0, 0.0])
            return float(pose[0]), float(pose[1]), float(pose[2] if len(pose) > 2 else 0.0), r

        # last resort: 3 m ahead in odom frame
        return 3.0, 0.0, 0.0, {"label": "fallback"}

    def publish_target(self):
        x, y, yaw, source = self.select_target()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.target_pub.publish(msg)

        dbg = String()
        dbg.data = json.dumps({
            "target_room": self.target_room,
            "target_pose": [x, y, yaw],
            "source": source
        }, ensure_ascii=False)
        self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = SemanticMemoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
