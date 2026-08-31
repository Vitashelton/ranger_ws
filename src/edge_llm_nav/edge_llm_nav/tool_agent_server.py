import json
from rclpy.node import Node
from std_msgs.msg import String
from .tool_schema import verify_tool_calls

class ToolAgentServer(Node):
    def __init__(self):
        super().__init__('tool_agent_server'); self.pub=self.create_publisher(String,'~/verified_tool_calls',10); self.grounded=set(); self.create_subscription(String,'~/tool_calls',self._cb,10); self.create_subscription(String,'/semantic_grounder/grounded_task',self._grounded,10)
    def _grounded(self,msg):
        try: self.grounded={x['node_id'] for x in json.loads(msg.data).get('task',[]) if x.get('grounding')=='grounded'}
        except Exception: self.grounded=set()
    def _cb(self,msg):
        try: graph=json.loads(msg.data); ok,errors=verify_tool_calls(graph,self.grounded)
        except Exception as exc: ok,errors=False,[str(exc)]
        if ok: out=String(); out.data=json.dumps(graph,ensure_ascii=False); self.pub.publish(out)
        else: self.get_logger().warning('tool calls rejected: %s','; '.join(errors))
def main(args=None):
    import rclpy; rclpy.init(args=args); n=ToolAgentServer(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
