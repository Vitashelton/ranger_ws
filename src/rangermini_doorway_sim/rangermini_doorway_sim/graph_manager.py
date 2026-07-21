"""Minimal dynamic semantic topology for the existing Office RPG corridor.

The graph deliberately contains no metric poses and no Gazebo/NPC truth.  It
stores only semantic connectivity, reachability, observations and entity
beliefs suitable for a high-level planner.
"""
from copy import deepcopy
import time


ALLOWED_ACTIONS = [
    "SEARCH_REGION", "NOTIFY_PERSON", "PATROL_REGION", "WAIT",
    "COMPLETE_TASK", "ABORT_TASK",
]

NODE_TYPES = {
    "lobby": "lobby",
    "junction": "junction",
    "room_904": "room",
    "room_906": "room",
    "room_908": "room",
}

# Costs are semantic route costs in metres, derived from the existing corridor
# observation-point layout.  They are not commands or physical paths.
BASE_EDGES = [
    {"from": "lobby", "to": "junction", "state": "FREE", "cost": 9.8,
     "evidence_source": "static_map"},
    {"from": "junction", "to": "room_904", "state": "FREE", "cost": 3.4,
     "evidence_source": "static_map"},
    {"from": "junction", "to": "room_906", "state": "FREE", "cost": 3.2,
     "evidence_source": "static_map"},
    {"from": "junction", "to": "room_908", "state": "FREE", "cost": 7.7,
     "evidence_source": "static_map"},
]

KNOWN_ENTITIES = ("teacher_zhang", "student_li", "visitor")


class GraphManager:
    def __init__(self):
        self.reset()

    @staticmethod
    def graph_region(region):
        return "junction" if region == "corridor_junction" else region

    @staticmethod
    def navigation_region(region):
        return "corridor_junction" if region == "junction" else region

    def reset(self, robot_region="lobby", entity=None, beliefs=None):
        self.graph_revision = 1
        self.robot_region = self.graph_region(robot_region or "lobby")
        self.nodes = {
            node_id: {"id": node_id, "type": node_type,
                      "searched": False, "reachable": True,
                      "last_observed": None}
            for node_id, node_type in NODE_TYPES.items()
        }
        self.edges = deepcopy(BASE_EDGES)
        self.entity_beliefs = {identity: {} for identity in KNOWN_ENTITIES}
        self.entity_last_seen = {identity: None for identity in KNOWN_ENTITIES}
        if entity and beliefs:
            self.set_beliefs(entity, beliefs, increment=False)
        self.recent_events = []

    def _increment(self):
        self.graph_revision += 1

    def set_beliefs(self, entity, beliefs, increment=True, clear_searched=False):
        if entity not in self.entity_beliefs:
            raise ValueError(f"Unknown entity: {entity}")
        filtered = {
            self.graph_region(region): max(0.0, float(value))
            for region, value in beliefs.items()
            if self.graph_region(region) in self.nodes
        }
        total = sum(filtered.values()) or 1.0
        self.entity_beliefs[entity] = {
            region: value / total for region, value in filtered.items()
        }
        if clear_searched:
            for region in filtered:
                self.nodes[region]["searched"] = False
        if increment:
            self._increment()

    def set_robot_region(self, region):
        mapped = self.graph_region(region)
        if mapped in self.nodes and mapped != self.robot_region:
            self.robot_region = mapped
            self._increment()

    def update_target_not_found(self, entity, region, reason="target_not_found"):
        mapped = self.graph_region(region)
        if mapped not in self.nodes or entity not in self.entity_beliefs:
            raise ValueError("TARGET_NOT_FOUND references unknown graph data")
        self.nodes[mapped]["searched"] = True
        self.nodes[mapped]["last_observed"] = time.time()
        beliefs = self.entity_beliefs[entity]
        if mapped in beliefs:
            beliefs[mapped] *= 0.05
            total = sum(beliefs.values()) or 1.0
            self.entity_beliefs[entity] = {
                key: value / total for key, value in beliefs.items()
            }
        self._append_event({
            "event_type": "TARGET_NOT_FOUND", "target_entity": entity,
            "region": mapped, "reason": reason, "timestamp": time.time(),
            "source": "local_perception",
        })
        self._increment()
        return dict(self.entity_beliefs[entity])

    def update_target_detected(self, entity, region):
        mapped = self.graph_region(region)
        if mapped not in self.nodes or entity not in self.entity_beliefs:
            return False
        self.nodes[mapped]["searched"] = True
        self.nodes[mapped]["last_observed"] = time.time()
        self.entity_last_seen[entity] = {
            "region": mapped, "timestamp": time.time(),
            "source": "local_perception"}
        self._append_event({
            "event_type": "TARGET_DETECTED", "target_entity": entity,
            "region": mapped, "timestamp": time.time(),
        })
        self._increment()
        return True

    def update_edge_blocked(self, target_region, event):
        mapped = self.graph_region(target_region)
        if mapped not in self.nodes:
            return False
        self.nodes[mapped]["reachable"] = False
        for edge in self.edges:
            if mapped in (edge["from"], edge["to"]):
                edge["state"] = "BLOCKED"
        saved = {
            "event_type": "EDGE_BLOCKED", "target_region": mapped,
            "timestamp": float(event.get("timestamp", time.time())),
            "duration": float(event.get("duration", 0.0)),
            "confidence": float(event.get("confidence", 0.0)),
            "source": event.get("source", "external_safety_interface"),
        }
        self._append_event(saved)
        self._increment()
        return True

    def _append_event(self, event):
        self.recent_events.append(deepcopy(event))
        self.recent_events = self.recent_events[-20:]

    def snapshot(self, mission_state="IDLE"):
        return {
            "schema_version": "ega_office_nav.v1",
            "graph_revision": self.graph_revision,
            "robot": {"current_region": self.robot_region,
                      "mission_state": mission_state},
            "nodes": [deepcopy(self.nodes[node]) for node in NODE_TYPES],
            "edges": deepcopy(self.edges),
            "entity_beliefs": [
                {"entity_id": entity, "candidates": deepcopy(candidates),
                 "last_seen": deepcopy(self.entity_last_seen[entity])}
                for entity, candidates in self.entity_beliefs.items()
                if candidates
            ],
            "recent_events": deepcopy(self.recent_events),
            "allowed_actions": list(ALLOWED_ACTIONS),
        }
