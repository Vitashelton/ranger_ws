"""Mode-aware action 的正向转换 (部署用):
    ModeAwareCmd(app_mode, param1, param2)
        -> geometry_msgs/Twist   (默认后端, 今天即可用, 经 safety_supervisor 下发)
        -> CAN 0x141 + 0x111     (直发后端, 绕过 Twist 仲裁; NEEDS_CONFIRMATION)

为什么默认用 Twist 后端:
    底盘 CAN 0x141 直设模式是否被当前固件支持尚未确认 (engineering_decisions.md 2.3/5.2#11);
    用精心构造的 Twist 可以确定性地触发期望仲裁模式, 且保持 safety_supervisor 为
    唯一 /cmd_vel 发布者的架构不变。

各 app_mode 的参数语义:
    ACKERMANN: param1=v[m/s], param2=delta[rad]
    SPIN:      param1=0,      param2=omega[rad/s]
    STOP:      param1=0,      param2=0
    OBLIQUE:   param1=v[m/s], param2=theta_dir[rad]   (第二阶段)
"""
import math
from . import params as P
from . import kinematics as K

EPS = 1e-6


def mode_aware_to_twist(app_mode: int, param1: float, param2: float, L: float = P.L_NOMINAL):
    """返回 (vx, vy, wz, info)。info 含钳位/丢弃提示。"""
    info = {"clamped": False, "note": ""}

    if app_mode == P.APP_MODE_STOP:
        return 0.0, 0.0, 0.0, info

    if app_mode == P.APP_MODE_SPIN:
        wz = K.spin_yaw_rate_clamp(param2)
        info["clamped"] = (wz != param2)
        return 0.0, 0.0, wz, info

    if app_mode == P.APP_MODE_OBLIQUE:
        v = max(-P.MAX_LINEAR_SPEED, min(P.MAX_LINEAR_SPEED, param1))
        theta = param2
        vx = v * math.cos(theta)
        vy = v * math.sin(theta)
        if abs(vy) < 1e-3:
            # 纯前向会被仲裁判成 Ackermann; 提醒上层 oblique 需要非零横向分量
            info["note"] = "oblique with |vy|~0 will arbitrate as Ackermann"
        return vx, vy, 0.0, info

    # ACKERMANN (默认)
    v, delta, was_clamped = K.clamp_ackermann_for_twist(param1, param2)
    info["clamped"] = was_clamped
    if abs(v) < 1e-3:
        # 零速无法 Ackermann; 退化为停车
        return 0.0, 0.0, 0.0, info
    wz = K.ackermann_yaw_rate(v, delta, L)
    return v, 0.0, wz, info


def mode_aware_to_can(app_mode: int, param1: float, param2: float):
    """直发后端: 返回 CAN 0x141(set mode) 与 0x111(motion cmd) 的字段。

    ⚠️ NEEDS_PHYSICAL_CONFIRMATION:
      - 固件是否支持 0x141 直设模式
      - 0x111 各 int16 字段的单位与缩放 (此处给逻辑值, 缩放需对照固件协议)
      - Spin 模式 V2 是否仍有 0.254558 缩放
    0x111 字段: (linear_vel, angular_vel, lateral_vel, steering_angle)
    """
    sdk_mode = P.APP_TO_SDK_MODE[app_mode]
    lin = ang = lat = steer = 0.0

    if app_mode == P.APP_MODE_ACKERMANN:
        v = max(-P.MAX_LINEAR_SPEED, min(P.MAX_LINEAR_SPEED, param1))
        delta = max(-P.MAX_STEER_ACKERMANN, min(P.MAX_STEER_ACKERMANN, param2))
        lin, steer = v, delta            # 直发模式可用全 40deg 转角(待确认)
    elif app_mode == P.APP_MODE_SPIN:
        ang = K.spin_yaw_rate_clamp(param2)
    elif app_mode == P.APP_MODE_OBLIQUE:
        v = max(-P.MAX_LINEAR_SPEED, min(P.MAX_LINEAR_SPEED, param1))
        theta = max(-P.MAX_STEER_PARALLEL, min(P.MAX_STEER_PARALLEL, param2))
        lin, steer = v, theta
    # STOP -> kPark, 全 0

    return {
        "can_141_mode": sdk_mode,
        "can_111": {"linear_vel": lin, "angular_vel": ang,
                    "lateral_vel": lat, "steering_angle": steer},
    }
