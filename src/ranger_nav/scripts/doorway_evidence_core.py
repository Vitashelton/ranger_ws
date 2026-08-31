#!/usr/bin/env python3
"""Pure functions for local doorway traversability evidence and baselines."""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


PASSABLE = 'PASSABLE'
BLOCKED = 'BLOCKED'
UNKNOWN = 'UNKNOWN'


@dataclass
class Evidence:
    score: float
    valid: bool
    source_stamp: float
    receipt_stamp: float
    support: int
    reason: str = ''


def transform_lidar_to_base(points, xyz=(0.30, 0.0, 0.70), pitch=0.523599):
    """Apply the verified base_link<-livox_frame rigid transform."""
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    cosine, sine = math.cos(pitch), math.sin(pitch)
    rotation = np.array([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ], dtype=np.float32)
    return points @ rotation.T + np.asarray(xyz, dtype=np.float32)


def lidar_door_evidence(
        points_base, door_distance, source_stamp=0.0, receipt_stamp=0.0,
        half_width=0.48, min_height=0.15, max_height=1.65,
        slab_half_depth=0.25, cell_size=0.08, min_view_points=100):
    """Estimate blocked-door probability from occupied y-z cells at the door plane."""
    points = np.asarray(points_base, dtype=np.float32).reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    view = points[
        finite & (points[:, 0] > 0.30) & (points[:, 0] < 6.0) &
        (np.abs(points[:, 1]) < 2.5) &
        (points[:, 2] > -0.3) & (points[:, 2] < 2.5)]
    if len(view) < min_view_points:
        return Evidence(0.5, False, source_stamp, receipt_stamp, len(view),
                        'insufficient_lidar_view')

    slab = view[
        (np.abs(view[:, 0] - float(door_distance)) <= slab_half_depth) &
        (np.abs(view[:, 1]) <= half_width) &
        (view[:, 2] >= min_height) & (view[:, 2] <= max_height)]
    y_bins = max(1, int(math.ceil(2.0 * half_width / cell_size)))
    z_bins = max(1, int(math.ceil((max_height - min_height) / cell_size)))
    if len(slab):
        yi = np.clip(
            ((slab[:, 1] + half_width) / cell_size).astype(int), 0, y_bins - 1)
        zi = np.clip(
            ((slab[:, 2] - min_height) / cell_size).astype(int), 0, z_bins - 1)
        occupied_cells = len(np.unique(zi * y_bins + yi))
    else:
        occupied_cells = 0
    coverage = occupied_cells / float(y_bins * z_bins)
    score = float(np.clip((coverage - 0.03) / 0.27, 0.0, 1.0))
    return Evidence(score, True, source_stamp, receipt_stamp, occupied_cells,
                    f'yz_coverage={coverage:.4f}')


def depth_door_evidence(
        depth_m, door_distance, source_stamp=0.0, receipt_stamp=0.0,
        roi=(0.28, 0.72, 0.20, 0.85), depth_tolerance=0.35,
        min_valid_pixels=500):
    """Estimate blocked-door probability from the central aligned-depth ROI."""
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2 or depth.size == 0:
        return Evidence(0.5, False, source_stamp, receipt_stamp, 0, 'invalid_depth')
    height, width = depth.shape
    u0, u1, v0, v1 = roi
    crop = depth[
        int(v0 * height):max(int(v0 * height) + 1, int(v1 * height)),
        int(u0 * width):max(int(u0 * width) + 1, int(u1 * width))]
    valid = crop[np.isfinite(crop) & (crop > 0.15) & (crop < 10.0)]
    if len(valid) < min_valid_pixels:
        return Evidence(0.5, False, source_stamp, receipt_stamp, len(valid),
                        'insufficient_depth_pixels')
    near = np.abs(valid - float(door_distance)) <= depth_tolerance
    closer = valid < float(door_distance) - depth_tolerance
    blocked_fraction = float(np.mean(near | closer))
    score = float(np.clip((blocked_fraction - 0.08) / 0.52, 0.0, 1.0))
    return Evidence(score, True, source_stamp, receipt_stamp, len(valid),
                    f'blocked_fraction={blocked_fraction:.4f}')


def score_state(evidence: Optional[Evidence], free_threshold=0.25,
                blocked_threshold=0.65):
    if evidence is None or not evidence.valid:
        return UNKNOWN
    if evidence.score >= blocked_threshold:
        return BLOCKED
    if evidence.score <= free_threshold:
        return PASSABLE
    return UNKNOWN


def fuse_baselines(lidar, depth, now, max_receipt_age=0.50,
                   max_pair_skew=0.25):
    """Return four comparable methods without commanding the robot."""
    lidar_state = score_state(lidar)
    depth_state = score_state(depth)
    valid = [item for item in (lidar, depth) if item is not None and item.valid]
    naive_score = float(np.mean([item.score for item in valid])) if valid else 0.5
    naive = score_state(Evidence(naive_score, bool(valid), now, now, len(valid)))

    lidar_age = math.inf if lidar is None else max(0.0, now - lidar.receipt_stamp)
    depth_age = math.inf if depth is None else max(0.0, now - depth.receipt_stamp)
    pair_skew = (
        math.inf if lidar is None or depth is None else
        abs(lidar.source_stamp - depth.source_stamp))
    fresh_pair = (
        lidar is not None and depth is not None and lidar.valid and depth.valid and
        lidar_age <= max_receipt_age and depth_age <= max_receipt_age and
        pair_skew <= max_pair_skew)
    if not fresh_pair:
        conservative = UNKNOWN
        reason = 'stale_missing_or_skewed'
    elif lidar.score >= 0.70 or depth.score >= 0.75:
        conservative = BLOCKED
        reason = 'fresh_high_blocked_evidence'
    elif lidar.score <= 0.25 and depth.score <= 0.25:
        conservative = PASSABLE
        reason = 'fresh_agreed_free_evidence'
    else:
        conservative = UNKNOWN
        reason = 'fresh_but_ambiguous_or_disagreeing'
    return {
        'lidar_only': lidar_state,
        'depth_only': depth_state,
        'naive_late_fusion': naive,
        'async_conservative': conservative,
        'naive_score': naive_score,
        'lidar_receipt_age': lidar_age,
        'depth_receipt_age': depth_age,
        'pair_stamp_skew': pair_skew,
        'async_reason': reason,
    }
