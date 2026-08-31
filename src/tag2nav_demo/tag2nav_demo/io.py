import math
from pathlib import Path
import yaml

from .core import Anchor, Pose, Task


def load_config(path):
    with Path(path).open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    anchors = {
        int(a["id"]): Anchor(int(a["id"]), a["name"], float(a["x"]), float(a["y"]),
                             math.radians(float(a["yaw_deg"])), float(a.get("quality", 1.0)))
        for a in raw["candidates"]
    }
    tasks = [
        Task(t["name"], float(t["weight"]),
             [Pose(float(p[0]), float(p[1]), math.radians(float(p[2]))) for p in t["route"]])
        for t in raw["tasks"]
    ]
    return raw, anchors, tasks
