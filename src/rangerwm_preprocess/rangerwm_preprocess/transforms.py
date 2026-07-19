"""部署/训练共用的预处理变换 (图像/深度/BEV)。保持 sim==real。"""
import numpy as np

def resize_rgb(img, size):          # -> [3,H,W] float32 in [0,1]
    raise NotImplementedError

def normalize_depth(depth, dmax=8.0, dmin=0.3):
    d = np.clip(depth, dmin, dmax) / dmax
    mask = ((depth >= dmin) & (depth <= dmax)).astype(np.uint8)
    return d.astype(np.float32), mask
