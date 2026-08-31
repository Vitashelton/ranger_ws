#!/usr/bin/env python3
"""Negative tests for strict task graph verification."""
from edge_llm_nav.task_schema import verify_graph

CASES = {
    'non_standard_fields': {'task':[{'action':'navigate_to','target':'lab_door','condition':'ok','type':'door','destination':'x','additionalPromptIfNotFound':'ask'}]},
    'missing_condition': {'task':[{'action':'navigate_to','target':'lab_door'}]},
    'missing_target': {'task':[{'action':'navigate_to','condition':'ok'}]},
    'chinese_target': {'task':[{'action':'navigate_to','target':'203门口','condition':'ok'}]},
    'unknown_move': {'task':[{'action':'move','target':'lab_door','condition':'ok'}]},
    'publish_cmd_vel': {'task':[{'action':'publish_cmd_vel','target':'lab_door','condition':'ok'}]},
}

def main():
    failed=[]
    for name, graph in CASES.items():
        ok, _, _ = verify_graph(graph)
        print(f'{name}: {"FAIL" if ok else "PASS"}')
        if ok: failed.append(name)
    raise SystemExit(1 if failed else 0)

if __name__ == '__main__': main()
