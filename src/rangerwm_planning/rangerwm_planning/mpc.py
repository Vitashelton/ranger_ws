"""MPC 候选采样 + 打分 + 重排 (纯 numpy, 与 WM 后端无关)。

被 sim/run_demo.py 与 ros2 rangerwm_mpc/mpc_node.py 共用。
WM 后端 (analytic 或 learned) 只需提供 .rollout(state, action_chunk, obstacles, goal, dt)。

第一阶段动作集: Ackermann + Spin + Stop (engineering_decisions 推荐)。
"""
import math
import numpy as np
from rangerwm_kinematics import params as P, kinematics as K


def sample_candidates(state, goal, cfg, rng=None):
    """生成 N 个 mode-aware 动作分块, 每块定长 (恒定动作, receding horizon)。
    返回 list[ list[(mode,p1,p2)] ]。
    """
    rng = rng or np.random.default_rng(0)
    H = cfg["H"]; v_max = P.MAX_LINEAR_SPEED
    dmax = P.DELTA_ACK_TWIST_MAX
    cands = []

    # 朝向目标的转角 (用于偏置采样)
    x, y, yaw = state; gx, gy = goal
    bearing = math.atan2(gy - y, gx - x)
    yaw_err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))

    # 1) Ackermann 网格: 速度档 x 转角档 (含朝目标方向偏置)
    default_deltas = [-dmax, -0.6*dmax, -0.3*dmax, 0.0, 0.3*dmax, 0.6*dmax, dmax]
    for v in cfg.get("v_grid", [0.3, 0.6, 1.0]):
        deltas = list(cfg.get("delta_grid") or default_deltas)
        deltas.append(float(np.clip(yaw_err, -dmax, dmax)))   # 目标偏置
        for d in deltas:
            cands.append([(P.APP_MODE_ACKERMANN, v, d)] * H)
    # 2) Spin 档 (调整朝向)
    for w in cfg.get("spin_grid", [-1.5, 1.5]):
        cands.append([(P.APP_MODE_SPIN, 0.0, w)] * H)
    # 3) Stop
    cands.append([(P.APP_MODE_STOP, 0.0, 0.0)] * H)

    # 4) 随机扰动若干 (探索)
    for _ in range(cfg.get("n_random", 8)):
        v = float(rng.uniform(0.0, v_max)); d = float(rng.uniform(-dmax, dmax))
        cands.append([(P.APP_MODE_ACKERMANN, v, d)] * H)
    return cands


def score_rollout(roll, w):
    """综合打分。roll 为 WM.rollout 输出 dict。"""
    goal = float(roll["goal_progress"].sum())
    collision = float(roll["collision"].max())          # 保守: 取最大碰撞概率
    # jerk: 速度差分 (用 ego_dx 近似)
    dx = roll["ego_dx"]
    jerk = float(np.abs(np.diff(dx)).sum()) if len(dx) > 1 else 0.0
    unc = float(roll.get("uncertainty", 0.0))
    return (w["goal"] * goal - w["collision"] * collision
            - w["smooth"] * jerk - w["uncertainty"] * unc)


def plan(state, wm, obstacles, goal, cfg, rng=None):
    """返回 (best_chunk, debug)。best_chunk 为 list[(mode,p1,p2)]。"""
    cands = sample_candidates(state, goal, cfg, rng)
    w = cfg["weights"]; dt = cfg["dt"]
    best, best_s, best_roll = None, -1e18, None
    for c in cands:
        roll = wm.rollout(state, c, obstacles, goal, dt)
        s = score_rollout(roll, w)
        if s > best_s:
            best, best_s, best_roll = c, s, roll
    exec_len = cfg.get("exec_len", 3)
    return best[:exec_len], {"score": best_s, "n_cands": len(cands),
                             "max_collision": float(best_roll["collision"].max())}
