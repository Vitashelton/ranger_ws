"""Slow-layer language task parsing through the local Ollama HTTP API."""
import json
import urllib.request
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .task_schema import verify_graph

SYSTEM = """Return JSON only: {version:'1',task_id:string,nodes:[{id,action,params,next}]}.
Allowed actions are navigate(target), wait(timeout_s), search(target), confirm(target), ask(question).
Never output cmd_vel, shell commands, or arbitrary actions."""

class LLMTaskServer(Node):
    def __init__(self):
        super().__init__('llm_task_server')
        self.declare_parameter('ollama_url', 'http://127.0.0.1:11434/api/generate')
        self.declare_parameter('model', 'llama3.2:3b')
        self.task_pub = self.create_publisher(String, '~/task_graph', 10)
        self.create_subscription(String, '~/task_request', self._request, 10)
        self.service = self.create_service(Trigger, '~/parse_last', self._parse_last)
        self.last_request = ''

    def _request(self, msg):
        self.last_request = msg.data
        self._parse(msg.data)

    def _parse_last(self, req, res):
        if not self.last_request:
            res.success, res.message = False, 'no task_request received'
        else:
            res.success = self._parse(self.last_request)
            res.message = 'published task graph' if res.success else 'LLM output rejected'
        return res

    def _parse(self, text):
        payload = {'model': self.get_parameter('model').value,
                   'system': SYSTEM, 'prompt': text, 'stream': False, 'format': 'json'}
        try:
            req = urllib.request.Request(self.get_parameter('ollama_url').value,
                data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=30) as response:
                body = json.loads(response.read().decode())
            graph = json.loads(body.get('response', '{}'))
            ok, errors, graph = verify_graph(graph)
            if not ok:
                self.get_logger().error('verifier rejected LLM graph: %s', '; '.join(errors)); return False
            out = String(); out.data = json.dumps(graph, ensure_ascii=False); self.task_pub.publish(out); return True
        except Exception as exc:
            self.get_logger().error('Ollama request failed: %s', exc); return False

def main(args=None):
    import rclpy
    rclpy.init(args=args); node = LLMTaskServer(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
