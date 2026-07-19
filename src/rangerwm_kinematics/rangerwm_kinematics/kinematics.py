"""Ranger Mini 2.0 双阿克曼运动学。

关键修正 (来自 kinematics_model.hpp::DualAckermanModel 与 engineering_decisions.md 第3节):
    双阿克曼 (前后轮反向偏转) 的偏航角速度为
        omega = 2 * v * sin(delta) / L
    而 *不是* 单阿克曼的  omega = v * tan(delta) / L。
    瞬时转弯半径
        R = v / omega = L / (2 * sin(delta))
"""
import math
from . import params as P

EPS = 1e-6


def ackermann_yaw_rate(v: float, delta: float, L: float = P.L_NOMINAL) -> float:
    """(v, 转角 delta) -> 偏航角速度 omega。 omega = 2 v sin(delta)/L。"""
    return 2.0 * v * math.sin(delta) / L


def ackermann_radius_from_delta(delta: float, L: float = P.L_NOMINAL) -> float:
    """转角 -> 瞬时转弯半径 (|delta| 越小半径越大)。"""
    s = math.sin(delta)
    if abs(s) < EPS:
        return math.inf
    return L / (2.0 * s)


def ackermann_radius_from_vw(v: float, wz: float) -> float:
    """(v, wz) -> 转弯半径 R = v / wz。"""
    if abs(wz) < EPS:
        return math.inf
    return v / wz


def ackermann_delta_from_vw(v: float, wz: float, L: float = P.L_NOMINAL) -> float:
    """由 (v, wz) 反解转角 delta。 delta = asin( wz*L / (2v) )。

    v 接近 0 时 Ackermann 无定义 (无法在零速下沿曲线行进), 返回 0 并由上层改用 Spin。
    """
    if abs(v) < EPS:
        return 0.0
    arg = wz * L / (2.0 * v)
    return math.asin(max(-1.0, min(1.0, arg)))


def clamp_ackermann_for_twist(v: float, delta: float):
    """把 (v, delta) 钳进 Twist 后端可执行范围。

    - v 钳到 +-MAX_LINEAR_SPEED
    - delta 钳到 +-DELTA_ACK_TWIST_MAX (~31.2deg), 超出会被底盘仲裁误判为 Spin
    返回 (v_clamped, delta_clamped, was_clamped)。
    """
    vc = max(-P.MAX_LINEAR_SPEED, min(P.MAX_LINEAR_SPEED, v))
    dmax = P.DELTA_ACK_TWIST_MAX
    dc = max(-dmax, min(dmax, delta))
    return vc, dc, (vc != v or dc != delta)


def spin_yaw_rate_clamp(wz: float) -> float:
    return max(-P.MAX_ANGULAR_SPEED, min(P.MAX_ANGULAR_SPEED, wz))
