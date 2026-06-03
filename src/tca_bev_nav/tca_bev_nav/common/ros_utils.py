"""Shared helpers for the tca_bev_nav package.

Only lightweight, dependency-free utilities live here so that the core
algorithm modules under ``tca_bev_nav.bev`` stay importable in a plain
Python environment (e.g. for offline unit tests on a laptop without a
full ROS2 install).
"""
from __future__ import annotations

from dataclasses import dataclass


def stamp_to_sec(stamp) -> float:
    """Convert a builtin_interfaces/Time stamp to float seconds.

    Works with both ``rclpy`` Time messages and any object exposing
    ``sec`` and ``nanosec`` fields.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class RateWatch:
    """Tracks message arrival rate / staleness for a single topic.

    Used by the safety supervisor and the time-align node to detect sensor
    timeouts without assuming a fixed publishing rate.
    """
    last_stamp_sec: float = 0.0
    last_wall_sec: float = 0.0
    ema_period: float = 0.0
    _alpha: float = 0.2

    def update(self, stamp_sec: float, wall_sec: float) -> None:
        if self.last_wall_sec > 0.0:
            dt = wall_sec - self.last_wall_sec
            if dt > 0:
                self.ema_period = (
                    self._alpha * dt + (1.0 - self._alpha) * self.ema_period
                    if self.ema_period > 0.0 else dt
                )
        self.last_stamp_sec = stamp_sec
        self.last_wall_sec = wall_sec

    def staleness(self, now_wall_sec: float) -> float:
        """Wall-clock seconds since the last message (0 if never seen)."""
        if self.last_wall_sec == 0.0:
            return float('inf')
        return now_wall_sec - self.last_wall_sec
