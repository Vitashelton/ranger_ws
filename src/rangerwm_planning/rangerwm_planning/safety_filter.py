"""纯 python 安全过滤 (与 rangerwm_safety ROS 节点共享逻辑)。

把候选 (mode,p1,p2) + 当前 clearance + 上一步动作 -> 安全后的 (mode,p1,p2)。
实现: 障碍降速/急停、加速度限制 (mode 相关)、模式切换冷却。
不依赖 ROS, 便于在 sim 与单元测试中复用同一逻辑。
"""
from rangerwm_kinematics import params as P


class SafetyFilter:
    def __init__(self, cfg):
        self.d_hard = cfg.get("d_hard_m", 0.35)
        self.d_soft = cfg.get("d_soft_m", 0.8)
        self.a_ack = cfg.get("a_max_ackermann", 0.5)
        self.a_spin = cfg.get("a_max_spin", 1.0)
        self.cooldown = cfg.get("mode_switch_cooldown_s", 0.4)
        self.dt = cfg.get("dt", 0.1)
        self.last_mode = P.APP_MODE_STOP
        self.last_v = 0.0
        self.since_switch = 1e9

    def filter(self, mode, p1, p2, clearance):
        mode = int(mode)
        # 1) 硬障碍 -> 停
        if clearance <= self.d_hard:
            self.last_mode = P.APP_MODE_STOP; self.last_v = 0.0; self.since_switch = 0.0
            return P.APP_MODE_STOP, 0.0, 0.0, "HARD_STOP"
        flag = "OK"
        # 2) 软障碍 -> Ackermann 降速
        if mode == P.APP_MODE_ACKERMANN and clearance < self.d_soft:
            r = (clearance - self.d_hard) / (self.d_soft - self.d_hard)
            p1 = p1 * r; flag = "SOFT_SLOW"
        # 3) 模式切换冷却
        if mode != self.last_mode and self.since_switch < self.cooldown:
            mode = self.last_mode; p1 = 0.0 if mode == P.APP_MODE_STOP else p1; flag = "COOLDOWN"
        # 4) 加速度限制 (仅对 Ackermann 线速度)
        if mode == P.APP_MODE_ACKERMANN:
            dv_max = self.a_ack * self.dt
            p1 = max(self.last_v - dv_max, min(self.last_v + dv_max, p1))
            self.last_v = p1
        else:
            self.last_v = 0.0
        self.since_switch = 0.0 if mode != self.last_mode else self.since_switch + self.dt
        self.last_mode = mode
        return mode, p1, p2, flag
