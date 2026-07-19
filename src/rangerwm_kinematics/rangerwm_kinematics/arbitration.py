"""复刻底盘 Twist->运动模式仲裁逻辑, 并提供反向标注 (cmd_vel -> mode-aware label)。

仲裁逻辑来源 (代码审查):
    src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:410-516
    输入 geometry_msgs/Twist (vx, vy, wz):
        vx==0 && vy==0 && wz==0                  -> Standby
        vy != 0                                  -> Oblique/Parallel  (wz 被静默丢弃!)
        (vx==0 && wz!=0) OR (|vx/wz| < R_min)    -> Spin
        else                                     -> Dual Ackermann
    R_min = 0.4764

数据采集影响 (engineering_decisions.md 2.2):
    Oblique 模式下 wz 被静默丢弃, /motion_state 不报告丢弃分量。
    => 录到的 /cmd_vel 与底盘实际执行动作之间存在不可观测差异。
    反向标注时必须把这种样本标 dropped_wz=True, 供训练做 loss 降权或剔除。
"""
import math
from . import params as P
from . import kinematics as K

EPS_V = 1e-3
EPS_W = 1e-3


def arbitrate_twist(vx: float, vy: float, wz: float, r_min: float = P.MIN_TURN_RADIUS) -> int:
    """返回 SDK MotionMode (kDualAckerman/kParallel/kSpinning/kPark...)。

    注意: 静止映射到 SDK_PARK(驻车) 还是 Standby 取决于上层语义;
    这里把全零判为 PARK 占位 (zero velocity)。
    """
    if abs(vx) < EPS_V and abs(vy) < EPS_V and abs(wz) < EPS_W:
        return P.SDK_PARK
    if abs(vy) >= EPS_V:
        return P.SDK_PARALLEL
    # vy ~ 0
    spin = (abs(vx) < EPS_V and abs(wz) >= EPS_W)
    if not spin and abs(wz) >= EPS_W:
        if abs(vx / wz) < r_min:
            spin = True
    if spin:
        return P.SDK_SPINNING
    return P.SDK_DUAL_ACKERMAN


def twist_to_mode_aware(vx: float, vy: float, wz: float):
    """反向标注: /cmd_vel -> (app_mode, param1, param2, meta)。

    meta 含:
        dropped_wz: bool  (Oblique 模式且原 wz != 0, 表示该样本动作不可观测)
        sdk_mode:   int
    用于训练数据标注 (rangerwm_preprocess/convert_bag.py)。
    """
    sdk_mode = arbitrate_twist(vx, vy, wz)
    meta = {"sdk_mode": sdk_mode, "dropped_wz": False}

    if sdk_mode == P.SDK_PARK:
        return P.APP_MODE_STOP, 0.0, 0.0, meta

    if sdk_mode == P.SDK_PARALLEL:  # oblique
        v = math.hypot(vx, vy)
        theta = math.atan2(vy, vx)
        meta["dropped_wz"] = abs(wz) >= EPS_W
        return P.APP_MODE_OBLIQUE, v, theta, meta

    if sdk_mode == P.SDK_SPINNING:
        return P.APP_MODE_SPIN, 0.0, wz, meta

    # dual ackermann
    v = vx
    delta = K.ackermann_delta_from_vw(v, wz, P.L_NOMINAL)
    return P.APP_MODE_ACKERMANN, v, delta, meta
