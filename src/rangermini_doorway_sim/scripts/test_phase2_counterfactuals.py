#!/usr/bin/env python3
"""Reproducible counterfactual checks for the Phase 2 semantic graph.

These checks isolate the graph/planner boundary from ROS, Gazebo and network
latency.  They are useful as a small paper/demo artifact: each case changes
the graph evidence while keeping the prior beliefs identical.
"""
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT.parent) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT.parent))

from rangermini_doorway_sim.graph_manager import GraphManager
from rangermini_doorway_sim.llm_providers import OfflineSemanticProvider


BELIEFS = {"room_904": 0.7, "room_906": 0.2, "lobby": 0.1}
MISSION = {
    "mission_id": "counterfactual_demo",
    "mission_type": "SEARCH_PERSON",
    "input_text": "找张老师",
    "target_person": "teacher_zhang",
}


def request(graph):
    return {
        "mission": MISSION,
        "dynamic_graph": graph.snapshot("PLAN_SEARCH"),
        "execution_history": [],
        "last_failure": None,
    }


def run_case(name, mutate):
    graph = GraphManager()
    graph.reset(entity="teacher_zhang", beliefs=BELIEFS)
    before = graph.graph_revision
    mutate(graph)
    plan = OfflineSemanticProvider().plan(request(graph))
    return {
        "case": name,
        "graph_revision": graph.graph_revision,
        "revision_changed": graph.graph_revision != before,
        "beliefs": graph.entity_beliefs["teacher_zhang"],
        "decision": plan,
    }


def main():
    cases = [
        run_case(
            "highest_belief_unreachable",
            lambda graph: graph.update_edge_blocked(
                "room_904", {"source": "counterfactual_demo", "confidence": 1.0}),
        ),
        run_case(
            "search_failed_then_replan",
            lambda graph: graph.update_target_not_found(
                "teacher_zhang", "room_904", "negative_observation"),
        ),
    ]
    expected = ["room_906", "room_906"]
    for result, target in zip(cases, expected):
        decision = result["decision"]
        assert decision["action"] == "SEARCH_REGION", result
        assert decision["target_region"] == target, result
        assert result["revision_changed"], result
    print(json.dumps({"status": "PASS", "cases": cases}, ensure_ascii=False,
                     indent=2))


if __name__ == "__main__":
    main()
