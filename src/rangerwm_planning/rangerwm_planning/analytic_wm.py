"""解析世界模型 (纯 numpy) —— 学习版 world model 的占位/对照实现。

接口与 learning/models/world_model.py 的 rollout 对齐:
    rollout(state, action_chunk, ctx) -> dict(collision, goal_progress, ego, ...)
区别在于这里用真实运动学正向积分 (相当于"完美" WM), 用于:
  - 在没有 torch 的环境里跑通 MPC + 闭环仿真 demo (证明控制架构正确)
  - 作为学习版 WM 的上界对照 (sanity baseline)

注入 RangerMiniV2 真实运动学 (omega = 2 v sin(delta)/L)。
"""
import math
import numpy as np
from rangerwm_kinematics import mode_aware as MA, params as P


def _body_twist(mode, p1, p2, L):
    vx, vy, wz, _ = MA.mode_aware_to_twist(int(mode), float(p1), float(p2), L)
    return vx, vy, wz


def integrate(state, mode, p1, p2, dt, L=P.L_NOMINAL):
    """单步积分。state=(x,y,yaw) 世界系。返回新 state 与 body-frame (dx,dy,dyaw)。"""
    x, y, yaw = state
    vx, vy, wz = _body_twist(mode, p1, p2, L)
    dyaw = wz * dt
    # body -> world
    c, s = math.cos(yaw), math.sin(yaw)
    dx_w = (vx * c - vy * s) * dt
    dy_w = (vx * s + vy * c) * dt
    new = (x + dx_w, y + dy_w, yaw + dyaw)
    return new, (vx * dt, vy * dt, dyaw)


def min_clearance(x, y, obstacles, robot_radius):
    """到最近障碍边缘的距离 (可为负=侵入)。"""
    if not obstacles:
        return math.inf
    d = min(math.hypot(x - ox, y - oy) - orad - robot_radius for ox, oy, orad in obstacles)
    return d


def collision_risk_from_clearance(clear_m, d_soft=0.8, d_hard=0.0):
    """clearance -> [0,1] 碰撞概率 (平滑)。clear<=d_hard -> 1; clear>=d_soft -> ~0。"""
    if clear_m <= d_hard:
        return 1.0
    if clear_m >= d_soft:
        return 0.0
    return float((d_soft - clear_m) / (d_soft - d_hard))


class AnalyticWorldModel:
    def __init__(self, robot_radius=0.35, d_soft=0.8, L=P.L_NOMINAL):
        self.rr = robot_radius; self.d_soft = d_soft; self.L = L

    def rollout(self, state, action_chunk, obstacles, goal, dt=0.1):
        """action_chunk: list[(mode,p1,p2)] 长度 H。返回 dict of np arrays 长度 H。"""
        x, y, yaw = state
        gx, gy = goal
        prev_dist = math.hypot(gx - x, gy - y)
        col, prog, edx, edy, edy_yaw, clears = [], [], [], [], [], []
        st = state
        for (m, p1, p2) in action_chunk:
            st, (dx, dy, dyaw) = integrate(st, m, p1, p2, dt, self.L)
            clr = min_clearance(st[0], st[1], obstacles, self.rr)
            cur_dist = math.hypot(gx - st[0], gy - st[1])
            col.append(collision_risk_from_clearance(clr, self.d_soft))
            prog.append(prev_dist - cur_dist)        # >0 表示接近目标
            edx.append(dx); edy.append(dy); edy_yaw.append(dyaw); clears.append(clr)
            prev_dist = cur_dist
        return {
            "collision": np.array(col), "goal_progress": np.array(prog),
            "ego_dx": np.array(edx), "ego_dy": np.array(edy), "ego_dyaw": np.array(edy_yaw),
            "clearance": np.array(clears), "final_state": st,
        }
