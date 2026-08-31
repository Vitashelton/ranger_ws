import argparse
import csv
import math
import random

from .core import (greedy_layout, manual_layout, random_layout, relocalization_trial,
                   task_pose_weights, uniform_layout)
from .io import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--output", default="layout_results.csv")
    args = parser.parse_args()
    cfg, anchors, tasks = load_config(args.config)
    rng = random.Random(int(cfg.get("random_seed", 7)))
    budget = int(cfg["tag_budget"])
    max_range = float(cfg["visibility_range_m"])
    half_fov = math.radians(float(cfg["visibility_half_fov_deg"]))
    weighted = task_pose_weights(tasks, cfg["degeneracy_zones"])
    layouts = {
        "random": random_layout(anchors, budget, rng),
        "uniform": uniform_layout(anchors, budget),
        "manual": manual_layout(anchors, budget),
        "proposed": greedy_layout(anchors, weighted, budget, max_range, half_fov),
    }
    task_population = [t for t in tasks]
    task_weights = [t.weight for t in tasks]
    rows = []
    for method, layout in layouts.items():
        successes, steps = 0, 0
        for _ in range(args.trials):
            task = rng.choices(task_population, weights=task_weights, k=1)[0]
            pose = rng.choice(task.route)
            ok, used = relocalization_trial(layout, anchors, pose, max_range, half_fov, rng)
            successes += int(ok)
            steps += used
        rows.append({
            "method": method,
            "tag_ids": " ".join(map(str, sorted(layout))),
            "success_rate": successes / args.trials,
            "mean_search_steps": steps / args.trials,
        })
    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    for row in rows:
        print(f"{row['method']:>8} tags=[{row['tag_ids']}] success={row['success_rate']:.3f} "
              f"search_steps={row['mean_search_steps']:.2f}")


if __name__ == "__main__":
    main()
