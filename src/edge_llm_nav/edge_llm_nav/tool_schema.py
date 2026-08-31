import json
from typing import Any, Dict, List, Tuple

TOOLS={"query_semantic_map","resolve_goal","navigate_to","verify_target","ask_user","wait","stop","get_robot_state"}
FORBIDDEN={"/cmd_vel","cmd_vel","velocity","twist","linear.x","angular.z","pose"}

def verify_tool_calls(graph: Dict[str,Any], grounded_ids=(), risk_state=None, risk_threshold=0.7) -> Tuple[bool,List[str]]:
    errors=[]
    if not isinstance(graph,dict) or set(graph)!={"tool_calls"} or not isinstance(graph.get('tool_calls'),list): return False,['top-level must contain only tool_calls list']
    raw=json.dumps(graph,ensure_ascii=False).lower(); errors += [f'forbidden token: {x}' for x in FORBIDDEN if x in raw]
    grounded=set(grounded_ids)
    for i,call in enumerate(graph['tool_calls']):
        if not isinstance(call,dict) or set(call)!={'tool','arguments'}: errors.append(f'tool_calls[{i}] malformed'); continue
        tool,args=call.get('tool'),call.get('arguments')
        if tool not in TOOLS: errors.append(f'tool_calls[{i}] unknown tool')
        if not isinstance(args,dict): errors.append(f'tool_calls[{i}] arguments must be object'); continue
        if tool=='query_semantic_map' and not isinstance(args.get('query'),str): errors.append(f'tool_calls[{i}] query must be string')
        if tool in {'resolve_goal','navigate_to','verify_target'}:
            key='candidate_id' if tool=='resolve_goal' else 'node_id'
            if tool=='verify_target': key='node_id'
            if not isinstance(args.get(key),str) or not args[key]: errors.append(f'tool_calls[{i}] {key} required')
            if tool=='navigate_to' and grounded and args.get('node_id') not in grounded: errors.append(f'tool_calls[{i}] node_id not grounded')
            if tool=='navigate_to' and risk_state and (args.get('node_id') not in risk_state or risk_state[args['node_id']].get('status')!='grounded'): errors.append(f'tool_calls[{i}] goal not safely grounded')
            if tool=='navigate_to' and risk_state and risk_state.get(args.get('node_id'),{}).get('risk_score',0)>risk_threshold: errors.append(f'tool_calls[{i}] risk exceeds threshold')
        if tool=='ask_user' and not isinstance(args.get('message'),str): errors.append(f'tool_calls[{i}] message required')
        if tool=='wait' and not isinstance(args.get('duration_sec'),(int,float)): errors.append(f'tool_calls[{i}] duration_sec required')
    return not errors,errors

contract_verifier = verify_tool_calls
