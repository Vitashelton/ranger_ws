import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from doorway_evidence_core import (  # noqa: E402
    BLOCKED,
    PASSABLE,
    UNKNOWN,
    Evidence,
    depth_door_evidence,
    fuse_baselines,
    lidar_door_evidence,
)


def scene_with_plane(closed):
    background = np.column_stack((
        np.full(600, 4.5),
        np.linspace(-2.0, 2.0, 600),
        np.tile(np.linspace(0.0, 2.0, 60), 10),
    ))
    if not closed:
        return background
    y, z = np.meshgrid(np.linspace(-0.46, 0.46, 30),
                       np.linspace(0.16, 1.64, 35))
    door = np.column_stack((np.full(y.size, 2.0), y.ravel(), z.ravel()))
    return np.vstack((background, door))


def test_lidar_closed_and_open_separate():
    closed = lidar_door_evidence(scene_with_plane(True), 2.0)
    opened = lidar_door_evidence(scene_with_plane(False), 2.0)
    assert closed.valid and closed.score > 0.7
    assert opened.valid and opened.score < 0.25


def test_depth_closed_and_open_separate():
    closed = depth_door_evidence(np.full((240, 320), 2.0), 2.0)
    opened = depth_door_evidence(np.full((240, 320), 5.0), 2.0)
    assert closed.valid and closed.score > 0.7
    assert opened.valid and opened.score < 0.25


def test_async_fusion_abstains_on_stale_evidence():
    lidar = Evidence(0.05, True, 10.0, 10.0, 100)
    depth = Evidence(0.05, True, 10.0, 8.0, 1000)
    result = fuse_baselines(lidar, depth, now=10.2, max_receipt_age=0.5)
    assert result['naive_late_fusion'] == PASSABLE
    assert result['async_conservative'] == UNKNOWN


def test_async_fusion_blocks_fresh_obstacle():
    lidar = Evidence(0.90, True, 10.0, 10.0, 100)
    depth = Evidence(0.10, True, 10.1, 10.1, 1000)
    result = fuse_baselines(lidar, depth, now=10.2)
    assert result['async_conservative'] == BLOCKED


def test_async_fusion_passes_only_with_fresh_agreement():
    lidar = Evidence(0.10, True, 10.0, 10.0, 100)
    depth = Evidence(0.12, True, 10.1, 10.1, 1000)
    result = fuse_baselines(lidar, depth, now=10.2)
    assert result['async_conservative'] == PASSABLE


def test_async_fusion_rejects_timestamp_skew():
    lidar = Evidence(0.10, True, 10.0, 20.0, 100)
    depth = Evidence(0.10, True, 11.0, 20.0, 1000)
    result = fuse_baselines(lidar, depth, now=20.1, max_pair_skew=0.25)
    assert math.isclose(result['pair_stamp_skew'], 1.0)
    assert result['async_conservative'] == UNKNOWN
