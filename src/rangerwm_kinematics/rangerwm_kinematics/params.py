"""Ranger Mini 2.0 底盘参数 (镜像 ranger_params.hpp::RangerMiniV2Params)。

唯一事实来源:
    src/ranger_ros2/ranger_base/include/ranger_base/ranger_params.hpp:49-65
所有数值与 SDK 标称值一致, 凡 SDK 未给出/不自洽的, 在此处显式标注并给出推导。

⚠️ 已知运动学不自洽 (见 engineering_decisions.md 第3节):
    L=0.494, delta_max=0.6981(40deg), R_min=0.4764 三者在 omega=2v sin(delta)/L 下不自洽。
    - 用 L, delta_max 推 R_min  -> 0.384 m
    - 用 R_min, delta_max 推 L  -> 0.612 m
    - 用 L, R_min       推 delta -> 0.5448 rad (31.2deg)
    处理约定:
      * 里程计/名义控制         -> 用 L_NOMINAL = 0.494
      * 安全扫掠/保守包络       -> 用 L_SAFETY  = 0.612
      * Twist 后端 Ackermann 转角 -> 钳到 DELTA_ACK_TWIST_MAX = 0.5448, 否则会被仲裁判成 Spin
"""
import math

# ---- SDK 标称值 (ranger_params.hpp) ----
TRACK = 0.364                      # 轮距 [m]
WHEELBASE = 0.494                  # 物理轴距 [m]  (= L_NOMINAL)
MAX_LINEAR_SPEED = 1.5             # [m/s]
MAX_ANGULAR_SPEED = 4.8            # [rad/s]  (spin)
MAX_STEER_CENTRAL = 0.4782         # ~27.40 deg
MAX_STEER_PARALLEL = 1.570         # 90 deg  (oblique/side)
MAX_STEER_ACKERMANN = 0.6981       # 40 deg  (机械极限)
MIN_TURN_RADIUS = 0.4764           # [m] (实测标称, 见不自洽说明)

# V1 在 spin 模式有角速度缩放因子 0.254558; V2 是否相同 NEEDS_PHYSICAL_CONFIRMATION
SPIN_OMEGA_SCALE_V1 = 0.254558

# ---- 推导/约定值 ----
L_NOMINAL = WHEELBASE              # 0.494, 里程计与名义控制
L_SAFETY = 2.0 * MIN_TURN_RADIUS * math.sin(MAX_STEER_ACKERMANN)  # ~0.612, 保守扫掠
# 在 Twist 后端下, 为保证 R = v/wz >= R_min 不被仲裁误判为 Spin, 转角上界:
DELTA_ACK_TWIST_MAX = math.asin(min(1.0, L_NOMINAL / (2.0 * MIN_TURN_RADIUS)))  # ~0.5448 rad

# 应用层 mode 枚举 (与 rangerwm_msgs/ModeAwareCmd.msg 对齐)
APP_MODE_ACKERMANN = 0
APP_MODE_SPIN = 1
APP_MODE_STOP = 2
APP_MODE_OBLIQUE = 3

# SDK 层 MotionMode 枚举 (ranger_interface.hpp:53-59)
SDK_DUAL_ACKERMAN = 0
SDK_PARALLEL = 1
SDK_SPINNING = 2
SDK_PARK = 3
SDK_SIDESLIP = 4

# 应用层 -> SDK 层 映射
APP_TO_SDK_MODE = {
    APP_MODE_ACKERMANN: SDK_DUAL_ACKERMAN,
    APP_MODE_SPIN: SDK_SPINNING,
    APP_MODE_STOP: SDK_PARK,
    APP_MODE_OBLIQUE: SDK_PARALLEL,
}
