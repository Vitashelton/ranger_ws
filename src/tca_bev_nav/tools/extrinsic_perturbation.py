#!/usr/bin/env python3
"""Inject controlled extrinsic perturbations (offline experiment tool).

Used for the extrinsic-perturbation ablation: given a measured camera->base
extrinsic, apply a known yaw error (deg) and translation error (m) so we can
measure how the conservative fusion degrades vs naive fusion.

This is an offline analysis utility; it does not run on the live robot. It is
intentionally tiny and dependency-light. No fabricated results are produced —
it only transforms inputs you provide.
"""
from __future__ import annotations

import argparse
import numpy as np


def yaw_matrix(yaw_rad: float) -> np.ndarray:
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def perturb_extrinsic(R: np.ndarray, t: np.ndarray,
                      yaw_deg: float, trans_m: tuple) -> tuple:
    """Return (R', t') with an added yaw error and translation offset."""
    Rp = yaw_matrix(np.radians(yaw_deg)) @ R
    tp = t + np.asarray(trans_m, dtype=float)
    return Rp, tp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw-deg', type=float, default=0.0,
                    help='one of {3,5,8} per the protocol')
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    args = ap.parse_args()

    # Identity baseline; replace with your measured extrinsic.
    R = np.eye(3)
    t = np.zeros(3)
    Rp, tp = perturb_extrinsic(R, t, args.yaw_deg, (args.dx, args.dy, args.dz))
    np.set_printoptions(precision=4, suppress=True)
    print('Perturbed rotation:\n', Rp)
    print('Perturbed translation:', tp)
    print('NOTE: feed this extrinsic into the offline BEV projection to '
          'reproduce the perturbation condition. Results -> TODO table.')


if __name__ == '__main__':
    main()
