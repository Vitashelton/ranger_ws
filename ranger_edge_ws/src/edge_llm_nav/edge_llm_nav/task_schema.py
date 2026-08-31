"""Schema and validation helpers for the slow-layer task graph."""
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Tuple

ALLOWED_ACTIONS = {"navigate", "wait", "search", "confirm", "ask"}
PARAM_RANGES = {"timeout_s": (0.1, 3600.0), "radius_m": (0.05, 10.0), "retries": (0, 5)}


def normalize_graph(graph: Dict[str, Any]) -> Dict[str, Any]:
    return {"version": str(graph.get("version", "1")),
            "task_id": str(graph.get("task_id", "llm-task")),
            "nodes": list(graph.get("nodes", [])),
            "metadata": dict(graph.get("metadata", {}))}


def verify_graph(graph: Dict[str, Any], known_targets: Iterable[str] = ()) -> Tuple[bool, List[str], Dict[str, Any]]:
    errors: List[str] = []
    graph = normalize_graph(graph)
    nodes = graph["nodes"]
    if not nodes:
        errors.append("nodes must be a non-empty list")
    if len(nodes) > 64:
        errors.append("too many nodes (max 64)")
    ids = set()
    targets = set(known_targets)
    for i, node in enumerate(nodes):
        if not isinstance(node, dict):
            errors.append(f"node {i} is not an object")
            continue
        node_id, action = node.get("id"), node.get("action")
        if not node_id or node_id in ids:
            errors.append(f"node {i} has missing or duplicate id")
        ids.add(node_id)
        if action not in ALLOWED_ACTIONS:
            errors.append(f"node {node_id}: action {action!r} is not allowed")
        params = node.get("params", {})
        if not isinstance(params, dict):
            errors.append(f"node {node_id}: params must be an object")
            continue
        for key, (low, high) in PARAM_RANGES.items():
            if key in params and (not isinstance(params[key], (int, float)) or not low <= params[key] <= high):
                errors.append(f"node {node_id}: {key} outside [{low}, {high}]")
        if action == "navigate":
            target = params.get("target")
            if not target:
                errors.append(f"node {node_id}: navigate requires params.target")
            elif targets and target not in targets:
                errors.append(f"node {node_id}: unknown target {target!r}")
        if action in {"confirm", "search"} and not params.get("target"):
            errors.append(f"node {node_id}: {action} requires params.target")
        if action == "ask" and not params.get("question"):
            errors.append(f"node {node_id}: ask requires params.question")
        for nxt in node.get("next", []):
            if nxt not in ids and nxt not in {n.get("id") for n in nodes if isinstance(n, dict)}:
                errors.append(f"node {node_id}: next references missing node {nxt!r}")
    return not errors, errors, graph
