#!/usr/bin/env python3
"""Metrics calculation utilities."""

import math
import numpy as np


def compute_min_distance_to_person(robot_pose, people_poses):
    """Compute minimum Euclidean distance from robot to any person."""
    if not people_poses:
        return float('inf')
    rx, ry = robot_pose[0], robot_pose[1]
    min_dist = float('inf')
    for p in people_poses:
        px, py = p[0], p[1]
        d = math.hypot(rx - px, ry - py)
        if d < min_dist:
            min_dist = d
    return min_dist


def compute_ttc(robot_pose, robot_vel, person_pose, person_vel, robot_radius=0.35):
    """Compute Time-To-Collision with a single person."""
    rx, ry = robot_pose[0], robot_pose[1]
    px, py = person_pose[0], person_pose[1]
    rvx, rvy = robot_vel[0], robot_vel[1]
    pvx, pvy = person_vel[0], person_vel[1]

    dist = math.hypot(rx - px, ry - py)
    if dist < 1e-6:
        return 0.0

    rel_vx = pvx - rvx
    rel_vy = pvy - rvy
    los_x = (px - rx) / dist
    los_y = (py - ry) / dist
    closing_speed = -(rel_vx * los_x + rel_vy * los_y)

    effective_dist = max(dist - robot_radius - 0.25, 0.01)
    if closing_speed > 0.01:
        return min(effective_dist / closing_speed, 100.0)
    return float('inf')


def is_goal_reached(robot_pose, goal_pose, tolerance=0.5):
    """Check if robot is within tolerance of goal."""
    return math.hypot(robot_pose[0] - goal_pose[0],
                      robot_pose[1] - goal_pose[1]) < tolerance


def compute_path_smoothness(vel_samples):
    """Compute velocity change rate as smoothness metric (lower = smoother)."""
    if len(vel_samples) < 2:
        return 0.0
    speeds = [math.hypot(v[0], v[1]) for v in vel_samples]
    diffs = [abs(speeds[i] - speeds[i - 1]) for i in range(1, len(speeds))]
    return float(np.mean(diffs)) if diffs else 0.0


def compute_iou(box_a, box_b):
    """Compute IOU between two bounding boxes [x1, y1, x2, y2]."""
    xa = max(box_a[0], box_b[0])
    ya = max(box_a[1], box_b[1])
    xb = min(box_a[2], box_b[2])
    yb = min(box_a[3], box_b[3])
    inter_area = max(0, xb - xa) * max(0, yb - ya)
    box_a_area = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    box_b_area = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    denom = box_a_area + box_b_area - inter_area
    return inter_area / denom if denom > 0 else 0.0
