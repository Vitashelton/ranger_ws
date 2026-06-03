"""Confidence models for TCA-BEV.

This module is the algorithmic heart of contributions (1) and (2): it turns
*soft* knowledge about timing, extrinsic calibration and pose-anchor quality
into scalar confidence values in [0, 1] that later modulate the conservative
fusion rule.

Design constraints honoured here:
  * No claim of "calibration-free accurate fusion". We only model how *little*
    we trust a modality and let the fusion rule stay conservative.
  * All thresholds / scales are parameters (loaded from YAML), never hidden
    magic numbers, so they can be reported in the paper and ablated.
  * Pure Python / math only -> unit-testable offline.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Time confidence
# ---------------------------------------------------------------------------
@dataclass
class TimeConfidenceParams:
    # Time difference (s) at which confidence has decayed to ~exp(-1).
    tau: float = 0.10
    # Hard cutoff (s): above this the modality is treated as unusable (conf=0).
    max_dt: float = 0.40


def time_confidence(dt_abs: float, p: TimeConfidenceParams) -> float:
    r"""Map an absolute timestamp difference to a confidence in [0, 1].

    We use a monotonically decreasing exponential
    ``c_t = exp(-|dt| / tau)`` clamped to 0 beyond ``max_dt``.

    Larger timestamp gaps between two modalities -> lower confidence, which is
    exactly the behaviour required (a modality measured "long ago" relative to
    the reference frame should not be trusted to clear obstacles).
    """
    if dt_abs >= p.max_dt:
        return 0.0
    return math.exp(-abs(dt_abs) / max(p.tau, 1e-6))


# ---------------------------------------------------------------------------
# Calibration confidence
# ---------------------------------------------------------------------------
@dataclass
class CalibConfidenceParams:
    # Rotational std (rad) at which rotational confidence ~= exp(-1).
    sigma_rot: float = math.radians(5.0)
    # Translational std (m) at which translational confidence ~= exp(-1).
    sigma_trans: float = 0.05


def calibration_confidence(rot_std_rad: float,
                           trans_std_m: float,
                           p: CalibConfidenceParams) -> float:
    r"""Combine rotational + translational extrinsic uncertainty into [0, 1].

    The std values come from the operator's *honest* estimate of how well the
    hand-measured extrinsics are known (stored in config/extrinsics.yaml). This
    is a weak-calibration assumption, not a calibration-free claim.

    ``c_c = exp(-(rot_std/sigma_rot)^2) * exp(-(trans_std/sigma_trans)^2)``
    """
    cr = math.exp(-(rot_std_rad / max(p.sigma_rot, 1e-6)) ** 2)
    ct = math.exp(-(trans_std_m / max(p.sigma_trans, 1e-6)) ** 2)
    return cr * ct


def inflation_radius(base_radius_m: float,
                     calib_conf: float,
                     max_extra_m: float) -> float:
    r"""Obstacle inflation radius that grows as calibration confidence drops.

    ``r = base + (1 - c_c) * max_extra``

    When ``c_c -> 1`` we inflate by ``base`` only; when ``c_c -> 0`` we add the
    full ``max_extra`` safety margin. This realises the requirement
    "外参置信度低时,障碍物膨胀半径自动增大".
    """
    calib_conf = max(0.0, min(1.0, calib_conf))
    return base_radius_m + (1.0 - calib_conf) * max_extra_m


# ---------------------------------------------------------------------------
# Pose-anchor quality
# ---------------------------------------------------------------------------
@dataclass
class PoseAnchorParams:
    # Max odom linear/angular velocity (m/s, rad/s) considered "well anchored".
    v_ref: float = 0.6
    w_ref: float = 0.8
    # Penalty weight for tf lookup latency (s).
    tf_latency_ref: float = 0.10


def pose_anchor_quality(v: float, w: float, tf_latency_s: float,
                        p: PoseAnchorParams) -> float:
    r"""Heuristic pose-anchor quality in [0, 1].

    Fast motion + stale tf both degrade how reliably a BEV cell can be anchored
    to the world frame. This is intentionally a *quality* signal, not an
    absolute uncertainty; it is reported and ablated, never used to fabricate
    accuracy.
    """
    motion = math.exp(-((v / max(p.v_ref, 1e-6)) ** 2
                        + (w / max(p.w_ref, 1e-6)) ** 2))
    tf_term = math.exp(-tf_latency_s / max(p.tf_latency_ref, 1e-6))
    return motion * tf_term
