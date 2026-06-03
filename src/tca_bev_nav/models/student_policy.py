#!/usr/bin/env python3
"""Student policy skeleton (Teacher-Student extension; NOT the core paper).

Input : BEV tensor (C x H x W) from /bev/tensor (occ, free, unknown).
Output: a 2-D velocity command (linear.x, angular.z).
Labels: /cmd_vel_safe recorded during teacher (ROS teleop / rule controller)
        runs — NEVER the physical RC remote (those commands do not enter ROS).

This is a deliberately small CNN so it can run on a Jetson Orin Nano. Training
loop is a placeholder; real training waits for collected bags (TODO).
"""
from __future__ import annotations

try:
    import torch
    import torch.nn as nn
    _HAS_TORCH = True
except Exception:  # pragma: no cover
    _HAS_TORCH = False


if _HAS_TORCH:
    class StudentPolicy(nn.Module):
        def __init__(self, in_channels: int = 3):
            super().__init__()
            self.backbone = nn.Sequential(
                nn.Conv2d(in_channels, 16, 5, stride=2, padding=2),
                nn.ReLU(inplace=True),
                nn.Conv2d(16, 32, 3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.Conv2d(32, 64, 3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d(1),
            )
            self.head = nn.Sequential(
                nn.Flatten(),
                nn.Linear(64, 64), nn.ReLU(inplace=True),
                nn.Linear(64, 2),  # (linear.x, angular.z)
            )

        def forward(self, bev):
            return self.head(self.backbone(bev))

    def train_step(model, batch, optimizer, loss_fn):
        """One supervised step against /cmd_vel_safe labels.

        batch = (bev[B,C,H,W], cmd[B,2]). Loss is MSE on the safe command.
        TODO: add unknown-mask-aware loss weighting so the student is penalised
        more for driving into unknown/occupied regions.
        """
        model.train()
        bev, cmd = batch
        pred = model(bev)
        loss = loss_fn(pred, cmd)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        return float(loss.item())
else:  # pragma: no cover
    class StudentPolicy:  # type: ignore
        def __init__(self, *a, **k):
            raise ImportError('PyTorch not installed; install torch to use '
                              'the student policy.')
