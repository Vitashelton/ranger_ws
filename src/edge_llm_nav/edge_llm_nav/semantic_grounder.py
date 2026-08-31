import json, os, yaml
from rclpy.node import Node
from std_msgs.msg import String
def ground(graph, nodes):
    result=[]
    for item in graph.get('task',[]):
        item=dict(item); q=item.get('goal_query',{}); text=(q.get('semantic_label','')+' '+q.get('spatial_hint','')).lower()
        hits=[n for n in nodes if any(a.lower() in text for a in [n['semantic_label']]+n.get('aliases',[]))]
        if item.get('intent') in {'navigate','verify'}:
            if not hits: item['grounding']='unresolved'
            elif len(hits)>1: item['grounding']='ambiguous'
            elif not hits[0].get('nav_reachable',False): item['grounding']='unreachable'
            else: item.update(grounding='grounded',node_id=hits[0]['node_id'],pose=hits[0]['pose'],confirmation_target=hits[0].get('confirmation_target'))
        else: item['grounding']='not_required'
        result.append(item)
    return {'task':result}

class SemanticGrounder(Node):
    def __init__(self):
        super().__init__('semantic_grounder'); self.declare_parameter('semantic_map',''); self.pub=self.create_publisher(String,'~/grounded_task',10); self.create_subscription(String,'/task_graph_verifier/verified_graph',self._cb,10); self.nodes=self._load()
    def _load(self):
        p=self.get_parameter('semantic_map').value or os.path.join(os.path.dirname(__file__),'../config/semantic_map.yaml')
        try:
            with open(p,encoding='utf-8') as f: return yaml.safe_load(f).get('nodes',[])
        except Exception as e: self.get_logger().error('semantic map load failed: %s',e); return []
    def _cb(self,msg):
        out=String(); out.data=json.dumps(ground(json.loads(msg.data),self.nodes),ensure_ascii=False); self.pub.publish(out)
def main(args=None):
    import rclpy; rclpy.init(args=args); n=SemanticGrounder(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
