#!/usr/bin/env python3
"""rangerwm_world_model/wm_node — 板载世界模型推理服务 (供 mpc 批量 rollout)。

backend: torch | onnx | trt | analytic(fallback)。部署关 RGB decoder。
此处给节点骨架: 加载模型 + 暴露 rollout (进程内供 mpc 调用, 或自定义 srv)。
"""
import rclpy
from rclpy.node import Node

class WMNode(Node):
    def __init__(self):
        super().__init__("rangerwm_world_model")
        self.declare_parameter("backend", "analytic")
        self.declare_parameter("engine_path", "")
        self.backend = self.get_parameter("backend").value
        self.model = self._load()
        self.get_logger().info(f"world model backend={self.backend}")

    def _load(self):
        if self.backend == "torch":
            import torch
            from learning.models.world_model import RangerWorldModel  # noqa
            m = RangerWorldModel(); 
            p = self.get_parameter("engine_path").value
            if p: m.load_state_dict(torch.load(p, map_location="cpu"))
            m.eval(); return m
        if self.backend in ("onnx", "trt"):
            # TODO(deploy): onnxruntime / TensorRT engine 包装
            return None
        from rangerwm_planning.analytic_wm import AnalyticWorldModel
        return AnalyticWorldModel()

def main():
    rclpy.init(); n = WMNode()
    try: rclpy.spin(n)
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
