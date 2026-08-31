"""Strict intent-level task graph validation."""
import json
from typing import Any, Dict, List, Tuple
ALLOWED_INTENTS={"navigate","verify","ask","wait","stop"}
ALLOWED_CONDITIONS={"always","arrived","target_visible","target_not_visible","nav_failed","ambiguous","unresolved"}
ITEM_KEYS={"intent","goal_query","condition"}; QUERY_KEYS={"semantic_label","spatial_hint","requires_confirmation"}
FORBIDDEN={"/cmd_vel","cmd_vel","twist","velocity","linear.x","angular.z"}

def normalize_intent_graph(graph, command=''):
    """Fill safe defaults; unresolved semantics remain non-executable."""
    if not isinstance(graph, dict): return graph
    if isinstance(graph.get('task'), list): tasks=graph['task']
    elif isinstance(graph.get('tasks'), list): tasks=graph['tasks']
    else: return {'task': []}
    out={'task': []}
    for item in tasks:
        if not isinstance(item, dict): out['task'].append(item); continue
        q=item.get('goal_query') if isinstance(item.get('goal_query'), dict) else {}
        q=dict(q); q.setdefault('semantic_label','unresolved'); q.setdefault('spatial_hint',''); q.setdefault('requires_confirmation',True)
        condition=item.get('condition') or 'always'
        if '|' in condition: condition='always'
        normalized=dict(item); normalized.pop('explanation',None); normalized['goal_query']=q; normalized['condition']=condition
        intent=normalized.get('intent','')
        if '|' in intent and any(x in command for x in ('去','前往','到')): normalized['intent']='navigate'
        elif intent=='navigate_to': normalized['intent']='navigate'
        elif intent=='verify_target': normalized['intent']='verify'
        elif intent=='ask_user': normalized['intent']='ask'
        label=q.get('semantic_label')
        if label in {'string','go_to','target','object','place'}: normalized['goal_query']['semantic_label']='unresolved'
        out['task'].append(normalized)
    return out

normalize_graph = normalize_intent_graph
def verify_graph(graph:Dict[str,Any],known_targets=())->Tuple[bool,List[str],Dict[str,Any]]:
    errors=[]
    if not isinstance(graph,dict) or set(graph)!={"task"} or not isinstance(graph.get("task"),list): return False,["top-level must contain only task list"],graph
    # Match complete dangerous control strings only; never inspect individual
    # characters (e.g. the `y` in goal_query).
    raw=json.dumps(graph,ensure_ascii=False).lower()
    errors += [f"forbidden token: {x}" for x in FORBIDDEN if x.lower() in raw]
    if not graph['task']: errors.append('task must not be empty')
    for i,item in enumerate(graph['task']):
        if not isinstance(item,dict): errors.append(f"task[{i}] must be object"); continue
        if set(item)!=ITEM_KEYS: errors.append(f"task[{i}] keys must be intent,goal_query,condition")
        if item.get('intent') not in ALLOWED_INTENTS: errors.append(f"task[{i}] unknown intent")
        if item.get('condition') not in ALLOWED_CONDITIONS: errors.append(f"task[{i}] invalid condition")
        q=item.get('goal_query')
        if not isinstance(q,dict) or set(q)!=QUERY_KEYS: errors.append(f"task[{i}] invalid goal_query keys")
        elif not isinstance(q.get('semantic_label'),str) or not isinstance(q.get('spatial_hint'),str) or not isinstance(q.get('requires_confirmation'),bool): errors.append(f"task[{i}] invalid goal_query types")
        elif item.get('intent') in {'navigate','verify'} and not q['semantic_label'].strip(): errors.append(f"task[{i}] semantic_label required")
    return not errors,errors,graph

def verify_intent_graph(graph, original_command=''):
    return verify_graph(graph)[0:2]
