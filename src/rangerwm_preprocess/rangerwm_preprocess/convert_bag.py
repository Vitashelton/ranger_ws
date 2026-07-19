#!/usr/bin/env python3
"""rosbag2 -> 训练样本 (Zarr 张量 + Parquet 索引)。

关键点:
  - 用 rangerwm_kinematics.arbitration.twist_to_mode_aware 把录到的 /cmd_vel
    反推成 mode-aware label, 并标注 dropped_wz (Oblique 静默丢弃 wz 的样本)。
  - dropped_wz=True 的样本动作不可观测 -> 写入 sample_weight 降权 (默认 0.1) 或剔除。
  - BEV 投影与部署节点 (tca_bev_fusion_node) 用同一参数, 保证 sim/real 一致。

CLI:
  python convert_bag.py --bag data/raw/goto_lab_a --out data/proc/ \
      --config configs/deploy.yaml
"""
import argparse
import numpy as np
import math
from rangerwm_kinematics import arbitration as A, params as P

# 占位的 IO; 真实实现用 rosbags / rosbag2_py + zarr + pyarrow
try:
    import zarr
    import pandas as pd
except Exception:  # 容器无依赖时仍可读懂结构
    zarr = None
    pd = None

TOPICS = {
    "rgb": "/camera/color/image_raw",
    "depth": "/camera/depth/image_rect_raw",
    "cloud": "/cloud_registered",            # FAST-LIVO2 配准点云
    "odom": "/odom",
    "cmd": "/cmd_vel",                       # 实际下发 (safety 之后)
    "obstacles": "/fused_obstacles",
    "motion_state": "/motion_state",         # 当前模式, 用于校验反推
}


def cmd_chunk_to_mode_aware(cmd_chunk):
    """cmd_chunk: [Ha,3] (vx,vy,wz) -> (mode[Ha], params[Ha,2], weight[Ha])。"""
    Ha = cmd_chunk.shape[0]
    mode = np.zeros(Ha, np.int64)
    params = np.zeros((Ha, 2), np.float32)
    weight = np.ones(Ha, np.float32)
    for i, (vx, vy, wz) in enumerate(cmd_chunk):
        m, p1, p2, meta = A.twist_to_mode_aware(float(vx), float(vy), float(wz))
        mode[i] = m
        params[i] = (p1, p2)
        if meta["dropped_wz"]:
            weight[i] = 0.1     # 动作不可观测, 降权 (可在 config 调或设 0 直接剔除)
    return mode, params, weight


def pointcloud_to_bev(points, cfg):
    """点云 -> BEV occupancy [3, Hb, Wb] (占据/高度/密度)。占位实现。"""
    rng = cfg["bev"]["range_m"]; res = cfg["bev"]["res_m"]
    n = int(round(rng / res))
    grid = np.zeros((3, n, n), np.float32)
    # TODO(real): 高度过滤 + base_link 居中投影 (与 tca_bev_fusion_node 完全一致)
    return grid


def convert(bag, out_zarr, out_parquet, cfg):
    # 伪流程; 真实需 rosbag2_py 读取并按 10Hz 重采样、按 episode 切分。
    raise NotImplementedError(
        "接 rosbag2_py/rosbags 读取后填充: resample(10Hz)->split_episodes->"
        "cmd_chunk_to_mode_aware/pointcloud_to_bev/diff_odom/collision_labels/goal_progress"
        "->write_zarr/write_parquet")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", default="configs/deploy.yaml")
    args = ap.parse_args()
    # demo: 验证反推逻辑可用
    demo = np.array([[1.0, 0.0, 0.3], [0.3, 0.1, 0.5], [0.0, 0.0, 1.2], [0, 0, 0]], np.float32)
    print("mode/params/weight:\n", cmd_chunk_to_mode_aware(demo))
