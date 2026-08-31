#!/usr/bin/env python3
from edge_llm_nav.task_schema import verify_graph

VALID={"task":[{"intent":"navigate","goal_query":{"semantic_label":"charging_dock","spatial_hint":"","requires_confirmation":True},"condition":"always"}]}
UNSAFE={"task":[{"intent":"navigate","goal_query":{"semantic_label":"charging_dock /cmd_vel","spatial_hint":"","requires_confirmation":True},"condition":"always"}]}

def main():
    ok, errors, _=verify_graph(VALID); print('goal_query_valid:', 'PASS' if ok else f'FAIL {errors}')
    bad, _, _=verify_graph(UNSAFE); print('cmd_vel_rejected:', 'PASS' if not bad else 'FAIL')
    raise SystemExit(0 if ok and not bad else 1)

if __name__=='__main__': main()
