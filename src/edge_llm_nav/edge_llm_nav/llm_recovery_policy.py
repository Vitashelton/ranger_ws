import json, urllib.request
from rclpy.node import Node
from std_msgs.msg import String
from .task_schema import verify_graph

class LLMRecoveryPolicy(Node):
    def __init__(self):
        super().__init__('llm_recovery_policy'); self.declare_parameter('ollama_url','http://127.0.0.1:11434/api/generate'); self.declare_parameter('model','llama3.2:3b'); self.pub=self.create_publisher(String,'~/recovery_graph',10); self.create_subscription(String,'/execution_monitor/state_summary',self._cb,10)
    def _cb(self,msg):
        try:
            state=json.loads(msg.data)
            if state.get('nav_status') not in ('FAILED','ABORTED') and state.get('localization')!='LOST': return
            payload={'model':self.get_parameter('model').value,'prompt':'Return JSON task graph with only search, confirm, wait, ask, navigate. Recover from: '+msg.data,'stream':False,'format':'json'}
            req=urllib.request.Request(self.get_parameter('ollama_url').value,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(req,timeout=30) as r: graph=json.loads(json.loads(r.read().decode()).get('response','{}'))
            ok, errors, graph = verify_graph(graph)
            if not ok:
                self.get_logger().warning('recovery graph rejected: %s', '; '.join(errors)); return
            out=String(); out.data=json.dumps(graph); self.pub.publish(out)
        except Exception as exc: self.get_logger().warning('recovery unavailable: %s',exc)
def main(args=None):
    import rclpy
    rclpy.init(args=args); n=LLMRecoveryPolicy(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
