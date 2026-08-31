from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Dict, Iterable, List, Sequence, Set, Tuple


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Anchor:
    id: int
    name: str
    x: float
    y: float
    yaw: float
    quality: float = 1.0


@dataclass
class Task:
    name: str
    weight: float
    route: List[Pose]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def visible_probability(pose: Pose, anchor: Anchor, max_range: float, half_fov: float) -> float:
    dx, dy = anchor.x - pose.x, anchor.y - pose.y
    distance = math.hypot(dx, dy)
    if distance > max_range or distance < 0.15:
        return 0.0
    bearing = abs(wrap(math.atan2(dy, dx) - pose.yaw))
    if bearing > half_fov:
        return 0.0
    # Smooth proxy: closer, centered, high-quality tags are more detectable.
    range_term = max(0.0, 1.0 - (distance / max_range) ** 1.5)
    angle_term = max(0.0, math.cos(0.5 * math.pi * bearing / half_fov))
    return min(0.995, anchor.quality * range_term * angle_term)


def zone_severity(pose: Pose, zones: Sequence[dict]) -> float:
    for z in zones:
        if z["xmin"] <= pose.x <= z["xmax"] and z["ymin"] <= pose.y <= z["ymax"]:
            return float(z["severity"])
    return 0.15


def task_pose_weights(tasks: Sequence[Task], zones: Sequence[dict]) -> List[Tuple[Pose, float]]:
    weighted = []
    for task in tasks:
        normalizer = max(len(task.route), 1)
        for pose in task.route:
            # A tag matters more on frequent tasks and in geometry-degenerate regions.
            w = task.weight * (0.35 + 0.65 * zone_severity(pose, zones)) / normalizer
            weighted.append((pose, w))
    return weighted


def layout_utility(layout: Set[int], anchors: Dict[int, Anchor], weighted_poses,
                   max_range: float, half_fov: float) -> float:
    total = 0.0
    for pose, weight in weighted_poses:
        miss = 1.0
        for aid in layout:
            miss *= 1.0 - visible_probability(pose, anchors[aid], max_range, half_fov)
        total += weight * (1.0 - miss)
    return total


def greedy_layout(anchors: Dict[int, Anchor], weighted_poses, budget: int,
                  max_range: float, half_fov: float) -> Set[int]:
    selected: Set[int] = set()
    while len(selected) < min(budget, len(anchors)):
        base = layout_utility(selected, anchors, weighted_poses, max_range, half_fov)
        best = max(
            (aid for aid in anchors if aid not in selected),
            key=lambda aid: layout_utility(selected | {aid}, anchors, weighted_poses,
                                           max_range, half_fov) - base,
        )
        selected.add(best)
    return selected


def uniform_layout(anchors: Dict[int, Anchor], budget: int) -> Set[int]:
    ordered = sorted(anchors.values(), key=lambda a: (a.x, a.y))
    if budget >= len(ordered):
        return {a.id for a in ordered}
    indices = [round(i * (len(ordered) - 1) / max(budget - 1, 1)) for i in range(budget)]
    return {ordered[i].id for i in indices}


def manual_layout(anchors: Dict[int, Anchor], budget: int) -> Set[int]:
    # Plausible human heuristic: prefer junctions and high-quality mounting sites.
    ordered = sorted(anchors.values(), key=lambda a: ("junction" in a.name, a.quality), reverse=True)
    return {a.id for a in ordered[:budget]}


def random_layout(anchors: Dict[int, Anchor], budget: int, rng: random.Random) -> Set[int]:
    return set(rng.sample(list(anchors), min(budget, len(anchors))))


def relocalization_trial(layout: Set[int], anchors: Dict[int, Anchor], pose: Pose,
                         max_range: float, half_fov: float, rng: random.Random,
                         search_headings: int = 8) -> Tuple[bool, int]:
    # The robot spins in place after localization loss. First successful tag fixes pose.
    for step in range(search_headings):
        view = Pose(pose.x, pose.y, pose.yaw + step * 2.0 * math.pi / search_headings)
        probs = [visible_probability(view, anchors[aid], max_range, half_fov) for aid in layout]
        detect_p = 1.0
        for p in probs:
            detect_p *= 1.0 - p
        if rng.random() < 1.0 - detect_p:
            return True, step + 1
    return False, search_headings
