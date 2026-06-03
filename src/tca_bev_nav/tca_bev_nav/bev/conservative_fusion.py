"""Conservative multi-modal BEV fusion rule (contribution 2).

Core safety invariant
----------------------
Low-confidence *free* evidence must never erase high-confidence *occupied*
evidence, and cells that nobody observed stay *unknown* (not free). This is the
mechanism that reduces false-free regions under weak calibration / weak
synchronisation.

Output occupancy semantics (single fused channel + masks):
    occ   in [0, 1]  fused occupancy belief
    free  in [0, 1]  fused free belief
    unknown {0, 1}   1 where no modality observed the cell
A planner should treat a cell as drivable only where
``free is high AND occ is low AND unknown == 0``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bev_grid import (BEVGrid, FREE_DEPTH, FREE_LIDAR, OCC_DEPTH, OCC_LIDAR,
                       UNKNOWN)


@dataclass
class FusionParams:
    # Minimum free-confidence required before free evidence may *contest*
    # occupied evidence at all. Below this, free is recorded but cannot clear.
    free_clear_threshold: float = 0.6
    # An occupied cell is only cleared if (free_belief * free_conf) exceeds
    # (occ_belief * occ_conf) by at least this margin -> asymmetric, biased
    # toward keeping obstacles.
    clear_margin: float = 0.25
    # Per-modality occupied confidence priors. LiDAR geometry is trusted more
    # than monocular-ish depth under weak extrinsics. TODO: justify / sweep.
    occ_conf_lidar: float = 0.9
    occ_conf_depth: float = 0.6


@dataclass
class ModalityConfidence:
    """Scalar confidences for one fusion call, from confidence.py."""
    time_lidar: float = 1.0
    time_depth: float = 1.0
    calib_depth: float = 1.0       # depth->base extrinsic trust
    pose_anchor: float = 1.0


def conservative_fuse(grid: BEVGrid, conf: ModalityConfidence,
                      p: FusionParams):
    """Collapse the multi-channel evidence grid into (occ, free, unknown).

    Returns a dict of float32 arrays, each HxW.
    """
    g = grid.grid

    # --- occupied belief: confidence-weighted max across modalities --------
    occ_l = g[..., OCC_LIDAR] * p.occ_conf_lidar * conf.time_lidar
    occ_d = (g[..., OCC_DEPTH] * p.occ_conf_depth
             * conf.time_depth * conf.calib_depth)
    occ = np.maximum(occ_l, occ_d) * max(conf.pose_anchor, 1e-3)

    # --- free belief: confidence-weighted, but depth free is gated ---------
    free_l = g[..., FREE_LIDAR] * conf.time_lidar
    free_d = g[..., FREE_DEPTH] * conf.time_depth * conf.calib_depth
    free = np.maximum(free_l, free_d)

    # --- unknown stays unknown ---------------------------------------------
    unknown = g[..., UNKNOWN].copy()

    # --- conservative clearing rule ----------------------------------------
    # Free may only reduce occupancy where it is both confident enough AND
    # beats occupancy by the asymmetric margin. Otherwise occupancy wins.
    free_conf = np.maximum(conf.time_lidar,
                           conf.time_depth * conf.calib_depth)
    can_clear = (free >= p.free_clear_threshold) & \
                (free >= occ + p.clear_margin) & \
                (free_conf >= p.free_clear_threshold)

    occ_out = np.where(can_clear, np.minimum(occ, 1.0 - free), occ)
    occ_out = np.clip(occ_out, 0.0, 1.0)

    # Where a cell is unknown, force free to 0 (never treat unknown as free).
    free_out = np.where(unknown > 0.5, 0.0, np.clip(free, 0.0, 1.0))

    return {
        'occ': occ_out.astype(np.float32),
        'free': free_out.astype(np.float32),
        'unknown': unknown.astype(np.float32),
    }


def false_free_violations(fused: dict, occ_ref: np.ndarray,
                          occ_thresh: float = 0.5,
                          free_thresh: float = 0.5) -> np.ndarray:
    """Cells declared free by the fusion but occupied in a reference grid.

    Used offline to compute the false-free-rate metric against a higher-trust
    reference (e.g. LiDAR-only occupied evidence). Returns a boolean mask.
    This is an *evaluation* helper, not part of the online pipeline.
    """
    declared_free = fused['free'] >= free_thresh
    ref_occ = occ_ref >= occ_thresh
    return declared_free & ref_occ
