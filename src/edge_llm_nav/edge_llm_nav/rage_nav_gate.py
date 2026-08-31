"""RAGE-Nav: Risk-Aware Grounding and Execution Gate.

The gate is a pure decision layer. It never publishes /cmd_vel; only an
`execute` decision may be consumed by the Nav2 adapter.
"""
import json, time
from dataclasses import dataclass, asdict
from typing import Any, Dict, List
try:
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pure decision tests can run without ROS installation
    class Node: pass
    class String: pass

@dataclass
class RiskTerms:
    ambiguity_score: float
    reachability_risk: float
    confirmation_risk: float
    hallucination_risk: float
    task_criticality: float
    history_failure_risk: float
    semantic_confidence_risk: float = 0.0
    freshness_risk: float = 0.0
    budget_risk: float = 0.0
    hallucination_mismatch_risk: float = 0.0

    @property
    def total_risk(self):
        weights=(.22,.18,.18,.18,.12,.12)
        values=(self.ambiguity_score,self.reachability_risk,self.confirmation_risk,self.hallucination_risk,self.task_criticality,self.history_failure_risk,self.semantic_confidence_risk,self.freshness_risk,self.budget_risk,self.hallucination_mismatch_risk)
        weights=(.14,.12,.12,.12,.08,.08,.10,.10,.06,.08)
        return round(sum(w*v for w,v in zip(weights,values)),4)

def evaluate(original_command: str, llm_tool_call: Dict[str,Any], candidates: List[Dict[str,Any]], robot_state: Dict[str,Any], nav_reachability: Dict[str,bool], confirmation_target_available: Dict[str,bool], history_stats: Dict[str,Any], risk_threshold=.45) -> Dict[str,Any]:
    tool=llm_tool_call.get('tool'); args=llm_tool_call.get('arguments',{}); node_id=args.get('node_id')
    count=len(candidates); best=candidates[0] if count==1 else None
    terms=RiskTerms(
        ambiguity_score=1.0 if count==0 else min(1.0,(count-1)/2),
        reachability_risk=0.0 if node_id and nav_reachability.get(node_id,False) else 1.0,
        confirmation_risk=0.0 if node_id and confirmation_target_available.get(node_id,False) else 0.8,
        hallucination_risk=0.0 if node_id and any(c.get('node_id')==node_id for c in candidates) else 1.0,
        task_criticality=1.0 if any(x in original_command for x in ('紧急','危险','停止','充电')) else .3,
        history_failure_risk=min(1.0,float(history_stats.get(node_id,{}).get('failure_rate',0.0))) if node_id else .5,
        semantic_confidence_risk=1.0-float(best.get('confidence',1.0)) if best else 1.0,
        freshness_risk=float(robot_state.get('freshness_risk',0.0)),
        budget_risk=float(robot_state.get('budget_risk',0.0)),
        hallucination_mismatch_risk=1.0 if node_id and best and best.get('node_id')!=node_id else 0.0)
    if tool=='navigate_to':
        if count==0 or terms.hallucination_risk>=1.0: decision='ask_user'
        elif count>1 or terms.ambiguity_score>.0: decision='ask_user'
        elif robot_state.get('blocked',False): decision='recover' if robot_state.get('safe_wait_area') else 'ask_user'
        elif terms.history_failure_risk>.7: decision='recover' if robot_state.get('safe_wait_area') else 'verify_first'
        elif terms.freshness_risk>.7 or terms.semantic_confidence_risk>.7: decision='ask_user'
        elif terms.total_risk>risk_threshold: decision='ask_user'
        elif not confirmation_target_available.get(node_id,False): decision='verify_first'
        elif not nav_reachability.get(node_id,False): decision='recover'
        else: decision='execute'
    elif tool in {'verify_target','get_robot_state'}: decision='execute'
    elif tool in {'ask_user','wait','stop'}: decision='execute'
    else: decision='reject'
    return {'command':original_command,'tool_call':llm_tool_call,'candidates':candidates,'risk_terms':asdict(terms),'total_risk':terms.total_risk,'decision':decision,'executed':decision=='execute','wrong_execution':False,'stamp':time.time(),'robot_state':robot_state}

class RAGENavGate(Node):
    def __init__(self):
        super().__init__('rage_nav_gate'); self.declare_parameter('risk_threshold',.45); self.pub=self.create_publisher(String,'~/decision',10); self.create_subscription(String,'~/request',self._cb,10)
    def _cb(self,msg):
        try:
            req=json.loads(msg.data); out=evaluate(req.get('original_command',''),req.get('llm_tool_call',{}),req.get('candidates',[]),req.get('robot_state',{}),req.get('nav_reachability',{}),req.get('confirmation_target_availability',{}),req.get('history_stats',{}),self.get_parameter('risk_threshold').value); m=String(); m.data=json.dumps(out,ensure_ascii=False); self.pub.publish(m)
        except Exception as exc: self.get_logger().error('RAGE-Nav request rejected: %s',exc)
def main(args=None):
    import rclpy; rclpy.init(args=args); n=RAGENavGate(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
