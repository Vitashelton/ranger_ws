import argparse
import json
import math

from .core import greedy_layout, layout_utility, task_pose_weights
from .io import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", default="selected_layout.json")
    args = parser.parse_args()
    cfg, anchors, tasks = load_config(args.config)
    weighted = task_pose_weights(tasks, cfg["degeneracy_zones"])
    half_fov = math.radians(float(cfg["visibility_half_fov_deg"]))
    selected = greedy_layout(anchors, weighted, int(cfg["tag_budget"]),
                             float(cfg["visibility_range_m"]), half_fov)
    result = {
        "method": "task_aware_greedy",
        "tag_budget": int(cfg["tag_budget"]),
        "selected": [anchors[i].__dict__ for i in sorted(selected)],
        "utility": layout_utility(selected, anchors, weighted,
                                  float(cfg["visibility_range_m"]), half_fov),
    }
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
