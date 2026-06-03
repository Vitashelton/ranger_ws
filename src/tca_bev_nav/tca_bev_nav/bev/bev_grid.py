"""BEV grid container for TCA-BEV.

The grid is robot-centric (origin at the base frame), x forward, y left, on a
fixed-resolution square. Each cell carries *evidence* channels rather than a
single occupancy value, so the conservative fusion rule (see
``conservative_fusion.py``) can reason about where free evidence came from and
how much it should be trusted.

Channels (last axis):
    0 OCC_LIDAR   occupied evidence from LiDAR        [0, 1]
    1 OCC_DEPTH   occupied evidence from depth/RGB-D  [0, 1]
    2 FREE_LIDAR  free (ray-passed) evidence, LiDAR   [0, 1]
    3 FREE_DEPTH  free evidence from depth/RGB-D      [0, 1]
    4 UNKNOWN     explicit unknown mask              {0, 1} (1 = never observed)

The unknown channel is *never* collapsed into free space. Downstream consumers
(planner, safety supervisor) must treat unknown != free.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Channel indices
OCC_LIDAR = 0
OCC_DEPTH = 1
FREE_LIDAR = 2
FREE_DEPTH = 3
UNKNOWN = 4
N_CHANNELS = 5


@dataclass
class BEVConfig:
    range_m: float = 8.0          # half-extent forward/back/left/right
    resolution_m: float = 0.05    # cell size
    z_min_m: float = -0.30        # crop below this (relative to base frame)
    z_max_m: float = 1.50         # crop above this
    # Points with |z| within [obstacle_z_lo, obstacle_z_hi] count as obstacles;
    # near-ground points contribute free/ground evidence. TODO: tune on real
    # MID360 data once the LiDAR mount height is measured.
    obstacle_z_lo: float = 0.05
    obstacle_z_hi: float = 1.20

    @property
    def size(self) -> int:
        return int(round(2 * self.range_m / self.resolution_m))


class BEVGrid:
    """A single robot-centric BEV evidence grid."""

    def __init__(self, cfg: BEVConfig):
        self.cfg = cfg
        n = cfg.size
        self.grid = np.zeros((n, n, N_CHANNELS), dtype=np.float32)
        # Start fully unknown until evidence arrives.
        self.grid[..., UNKNOWN] = 1.0

    # -- coordinate helpers -------------------------------------------------
    def world_to_cell(self, x: np.ndarray, y: np.ndarray):
        """Vectorised metric (base-frame) -> integer cell index.

        Returns (ix, iy, valid_mask).
        """
        c = self.cfg
        ix = np.floor((x + c.range_m) / c.resolution_m).astype(np.int32)
        iy = np.floor((y + c.range_m) / c.resolution_m).astype(np.int32)
        n = c.size
        valid = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        return ix, iy, valid

    def reset(self):
        self.grid.fill(0.0)
        self.grid[..., UNKNOWN] = 1.0

    # -- evidence accumulation ---------------------------------------------
    def add_occupied(self, ix, iy, channel: int, weight: float = 1.0):
        np.maximum.at(self.grid[..., channel], (ix, iy), weight)
        # Any observation clears the unknown flag for that cell.
        self.grid[ix, iy, UNKNOWN] = 0.0

    def add_free(self, ix, iy, channel: int, weight: float = 1.0):
        np.maximum.at(self.grid[..., channel], (ix, iy), weight)
        self.grid[ix, iy, UNKNOWN] = 0.0


def points_to_lidar_evidence(points_xyz: np.ndarray, grid: BEVGrid) -> None:
    """Project a LiDAR point cloud (Nx3, base frame) into occupied evidence.

    NOTE: this is the minimal LiDAR-only v1 path. It marks cells containing
    obstacle-height returns as occupied. Proper ray-casting for *free* evidence
    (marking cells the ray traversed before the hit) is left as a clearly
    flagged TODO so we never invent free space we did not measure.
    """
    if points_xyz.size == 0:
        return
    c = grid.cfg
    x, y, z = points_xyz[:, 0], points_xyz[:, 1], points_xyz[:, 2]
    keep = (z >= c.z_min_m) & (z <= c.z_max_m)
    x, y, z = x[keep], y[keep], z[keep]
    obstacle = (z >= c.obstacle_z_lo) & (z <= c.obstacle_z_hi)
    ix, iy, valid = grid.world_to_cell(x, y)
    obs_v = valid & obstacle
    if np.any(obs_v):
        grid.add_occupied(ix[obs_v], iy[obs_v], OCC_LIDAR, weight=1.0)
    # TODO(free-space): implement Bresenham / DDA ray-casting from sensor
    # origin to each return to populate FREE_LIDAR. Until then we DO NOT emit
    # LiDAR free evidence, keeping unobserved cells unknown (conservative).
