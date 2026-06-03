#!/usr/bin/env python3
"""Metric definitions for TCA-BEV (formulas only; no fabricated values).

Every function takes real arrays / logs you produce from experiments and
returns a number. The module also emits an empty Markdown results table with
TODO cells, to be filled ONLY after real experiments.

Definitions (also written verbatim in the paper, sections/experiments.tex):

  false-free rate (FFR)
      fraction of cells declared FREE by the fused BEV that are OCCUPIED in a
      higher-trust reference grid:
          FFR = |declared_free AND ref_occupied| / |ref_occupied|

  obstacle preservation rate (OPR)
      fraction of reference-occupied cells that remain OCCUPIED after fusion:
          OPR = |fused_occupied AND ref_occupied| / |ref_occupied|

  conservative area ratio (CAR)
      fraction of the BEV kept UNKNOWN or inflated rather than declared free:
          CAR = |unknown OR inflated| / |all cells in sensor FOV|

  BEV update latency
      wall-clock time from triggering sensor stamp to /bev/tensor publish.

  Jetson runtime FPS
      published /bev/tensor messages per second under deployment load.

Counts (from run logs):
  safety_stop_count, near_collision_count, navigation_success_rate,
  average_speed, path_tracking_error.
"""
from __future__ import annotations

import numpy as np


def false_free_rate(declared_free: np.ndarray, ref_occupied: np.ndarray):
    ref = ref_occupied.astype(bool)
    if ref.sum() == 0:
        return float('nan')
    return float((declared_free.astype(bool) & ref).sum() / ref.sum())


def obstacle_preservation_rate(fused_occupied: np.ndarray,
                               ref_occupied: np.ndarray):
    ref = ref_occupied.astype(bool)
    if ref.sum() == 0:
        return float('nan')
    return float((fused_occupied.astype(bool) & ref).sum() / ref.sum())


def conservative_area_ratio(unknown: np.ndarray, inflated: np.ndarray,
                            fov_mask: np.ndarray):
    fov = fov_mask.astype(bool)
    if fov.sum() == 0:
        return float('nan')
    conservative = (unknown.astype(bool) | inflated.astype(bool)) & fov
    return float(conservative.sum() / fov.sum())


def path_tracking_error(actual_xy: np.ndarray, ref_xy: np.ndarray):
    """Mean Euclidean cross-track error (m). Inputs Nx2, aligned by index."""
    n = min(len(actual_xy), len(ref_xy))
    if n == 0:
        return float('nan')
    d = np.linalg.norm(actual_xy[:n] - ref_xy[:n], axis=1)
    return float(d.mean())


def emit_todo_table() -> str:
    rows = [
        'LiDAR-only', 'Depth-only', 'Naive fusion',
        'Fixed inflation', 'TCA-BEV (full)',
    ]
    metrics = ['FFR', 'OPR', 'CAR', 'BEV lat (ms)',
               'safety stops', 'near-collisions', 'success %',
               'avg speed (m/s)', 'track err (m)', 'Jetson FPS']
    header = '| Method | ' + ' | '.join(metrics) + ' |'
    sep = '|' + '---|' * (len(metrics) + 1)
    body = '\n'.join('| ' + r + ' | ' + ' | '.join(['TODO'] * len(metrics))
                     + ' |' for r in rows)
    return '\n'.join([header, sep, body])


if __name__ == '__main__':
    print('# Results (TO BE FILLED FROM REAL EXPERIMENTS)\n')
    print(emit_todo_table())
