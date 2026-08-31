import csv, os, time
from rclpy.node import Node
from std_msgs.msg import String

class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger'); self.declare_parameter('csv_path','/tmp/edge_llm_nav_metrics.csv'); p=self.get_parameter('csv_path').value; self.f=open(p,'a',newline=''); self.w=csv.writer(self.f); self.w.writerow(['stamp','event','payload']); self.create_subscription(String,'/execution_monitor/state_summary',lambda m:self._log('state',m.data),10); self.create_subscription(String,'/task_graph_verifier/rejection',lambda m:self._log('verifier_rejection',m.data),10); self.create_subscription(String,'/llm_recovery_policy/recovery_graph',lambda m:self._log('recovery',m.data),10)
    def _log(self,e,p): self.w.writerow([time.time(),e,p]); self.f.flush()
    def destroy_node(self): self.f.close(); return super().destroy_node()
def main(args=None):
    import rclpy
    rclpy.init(args=args); n=ExperimentLogger(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
