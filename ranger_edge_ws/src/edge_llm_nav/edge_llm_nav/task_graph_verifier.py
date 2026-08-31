from rclpy.node import Node
from std_msgs.msg import String
from .task_schema import verify_graph
import json

class TaskGraphVerifier(Node):
    def __init__(self):
        super().__init__('task_graph_verifier')
        self.declare_parameter('known_targets', [])
        self.pub = self.create_publisher(String, '~/verified_graph', 10)
        self.reject = self.create_publisher(String, '~/rejection', 10)
        self.create_subscription(String, '/llm_task_server/task_graph', self._cb, 10)
    def _cb(self, msg):
        try: graph = json.loads(msg.data)
        except json.JSONDecodeError as exc: self._reject([str(exc)]); return
        ok, errors, graph = verify_graph(graph, self.get_parameter('known_targets').value)
        if ok:
            out = String(); out.data = json.dumps(graph, ensure_ascii=False); self.pub.publish(out)
        else: self._reject(errors)
    def _reject(self, errors):
        out = String(); out.data = json.dumps({'valid': False, 'errors': errors}, ensure_ascii=False); self.reject.publish(out)

def main(args=None):
    import rclpy
    rclpy.init(args=args); n=TaskGraphVerifier(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
