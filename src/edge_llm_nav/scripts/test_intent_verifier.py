#!/usr/bin/env python3
import json
from edge_llm_nav.task_schema import verify_graph

VALID = {"task":[{"intent":"navigate","goal_query":{"semantic_label":"charging_dock","spatial_hint":"","requires_confirmation":True},"condition":"always"}]}
INVALID_OLD = {"task":[{"action":"navigate_to","target":"room_203","condition":"always"}]}

def main():
    valid, valid_errors, _ = verify_graph(VALID)
    invalid, _, _ = verify_graph(INVALID_OLD)
    print('valid_intent_graph:', 'PASS' if valid else f'FAIL: {valid_errors}')
    print('old_action_schema_rejected:', 'PASS' if not invalid else 'FAIL')
    raise SystemExit(0 if valid and not invalid else 1)

if __name__ == '__main__': main()
