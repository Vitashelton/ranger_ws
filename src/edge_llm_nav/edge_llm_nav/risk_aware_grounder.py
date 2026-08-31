import json, os, yaml
from rclpy.node import Node
from std_msgs.msg import String

def ground(query, nodes):
    text=(query.get('semantic_label','')+' '+query.get('spatial_hint','')).lower()
    hits=[n for n in nodes if any(a.lower() in text for a in [n['semantic_label']]+n.get('aliases',[]))]
    candidates=[{'node_id':n['node_id'],'alias':n['semantic_label'],'confidence':round(1.0/len(hits),3)} for n in hits]
    if not hits: status='unresolved'; confidence=0.0
    elif len(hits)>1: status='ambiguous'; confidence=max(x['confidence'] for x in candidates)
    elif not hits[0].get('nav_reachable',False): status='unreachable'; confidence=1.0
    else: status='grounded'; confidence=1.0
    return {'status':status,'confidence':confidence,'risk_score':round(1.0-confidence,3),'candidates':candidates}

class RiskAwareGrounder(Node):
    def __init__(self):
        super().__init__('risk_aware_grounder'); self.declare_parameter('semantic_map',''); self.pub=self.create_publisher(String,'~/risk_grounding',10); self.create_subscription(String,'/tool_agent_server/semantic_query',self._cb,10); self.nodes=self._load()
    def _load(self):
        p=self.get_parameter('semantic_map').value or os.path.join(os.path.dirname(__file__),'../config/semantic_map.yaml')
        with open(p,encoding='utf-8') as f: return yaml.safe_load(f).get('nodes',[])
    def _cb(self,msg):
        q=json.loads(msg.data); out=String(); out.data=json.dumps({'query':q,'grounding':ground(q,self.nodes)},ensure_ascii=False); self.pub.publish(out)
def main(args=None):
    import rclpy; rclpy.init(args=args); n=RiskAwareGrounder(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
