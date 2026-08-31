import json
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
except ImportError:  # schema tests can run without a Nav2 installation
    ActionClient = NavigateToPose = None

class TaskExecutor(Node):
    """Deterministic dispatcher; navigation is delegated to Nav2, never cmd_vel."""
    def __init__(self):
        super().__init__('task_executor'); self.pub = self.create_publisher(String, '~/behavior', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') if ActionClient else None
        self.create_subscription(String, '/semantic_grounder/grounded_task', self._cb, 10)
        self.create_service(Trigger, '~/start', self._start); self.graph=None; self.index=0
    def _cb(self,msg): self.graph=json.loads(msg.data); self.index=0
    def _start(self,req,res):
        if not self.graph: res.success=False; res.message='no verified graph'; return res
        for node in self.graph['task']:
            if node.get('grounding') not in ('grounded', 'not_required'):
                continue
            # The adapter owns the fast-layer action boundary. Only a verified
            # navigate node with an explicit pose is sent to Nav2 directly.
            if node.get('intent') == 'navigate' and self.nav_client and 'pose' in node:
                goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
                pose = node['pose']; goal.pose.pose.position.x = float(pose['x']); goal.pose.pose.position.y = float(pose['y'])
                self.nav_client.send_goal_async(goal)
            else:
                out=String(); out.data=json.dumps(node, ensure_ascii=False); self.pub.publish(out)
        res.success=True; res.message='dispatched behaviors'; return res

def main(args=None):
    import rclpy
    rclpy.init(args=args); n=TaskExecutor(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
