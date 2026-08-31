import json, time
from rclpy.node import Node
from std_msgs.msg import String

class ExecutionMonitor(Node):
    def __init__(self):
        super().__init__('execution_monitor'); self.state={'stamp':0.0,'nav_status':'UNKNOWN','localization':'UNKNOWN','detections':[],'tf_ok':False}
        self.pub=self.create_publisher(String,'~/state_summary',10); self.create_timer(0.5,self._tick)
        for topic,key in [('/navigate_to_pose/_action/status','nav_status'),('/localization/health','localization'),('/detections','detections')]:
            self.create_subscription(String,topic,lambda m,k=key:self._set(k,m.data),10)
    def _set(self,key,val): self.state[key]=val
    def _tick(self): self.state['stamp']=time.time(); m=String(); m.data=json.dumps(self.state,ensure_ascii=False); self.pub.publish(m)
def main(args=None):
    import rclpy
    rclpy.init(args=args); n=ExecutionMonitor(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
