#!/usr/bin/env python3
"""
run_all_ablation_trials.py — Batch experiment automation.

Iterates over scenarios × modes × random_seeds, launches each trial,
monitors for completion/timeout, and collects CSV results.

Usage:
  python3 scripts/run_all_ablation_trials.py \
    --scenarios crossing_person doorway_person \
    --modes lidar_only lidar_depth_fusion \
    --seeds 0 1 2 \
    --timeout 120 \
    --output-dir ~/ranger_ws/experiments/results

Requires: ros2 launch to be available in PATH.
"""
import argparse
import csv
import os
import subprocess
import signal
import sys
import time
from datetime import datetime


def run_trial(scenario, mode, seed, timeout, output_dir, gui=False):
    """Launch a single ablation trial and wait for completion."""
    trial_name = f'{scenario}_{mode}_s{seed}'
    print(f'\n{"="*60}')
    print(f'Starting trial: {trial_name}')
    print(f'{"="*60}')

    launch_args = [
        'ros2', 'launch', 'ranger_gazebo_experiments',
        'ranger_ablation_experiment.launch.py',
        f'scenario:={scenario}',
        f'mode:={mode}',
        f'random_seed:={seed}',
        f'trial_timeout:={timeout}',
        f'output_dir:={output_dir}',
        f'gui:={"true" if gui else "false"}',
        f'rviz:={"true" if gui else "false"}',
        'record_bag:=false',
    ]

    start_time = time.time()
    proc = subprocess.Popen(launch_args)
    try:
        proc.wait(timeout=timeout + 30)  # extra margin for startup
    except subprocess.TimeoutExpired:
        print(f'  [TIMEOUT] Killing trial {trial_name}')
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    except KeyboardInterrupt:
        print(f'  [INTERRUPTED] Stopping trial {trial_name}')
        proc.send_signal(signal.SIGINT)
        proc.wait()
        return None

    elapsed = time.time() - start_time
    print(f'  Trial {trial_name} finished in {elapsed:.1f}s')
    return elapsed


def main():
    parser = argparse.ArgumentParser(description='Run ablation experiment trials')
    parser.add_argument('--scenarios', nargs='+',
                        default=['crossing_person', 'doorway_person',
                                 'same_direction_person', 'opposite_direction_person',
                                 'multi_person_corridor'])
    parser.add_argument('--modes', nargs='+',
                        default=['lidar_only', 'depth_only', 'yolo_only',
                                 'lidar_depth_fusion', 'lidar_depth_yolo_fusion',
                                 'fusion_with_risk_avoidance'])
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2, 3, 4])
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--output-dir', default=os.path.expanduser('~/ranger_ws/experiments/results'))
    parser.add_argument('--gui', action='store_true', default=False)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    total_trials = len(args.scenarios) * len(args.modes) * len(args.seeds)
    print(f'Running {total_trials} trials: '
          f'{len(args.scenarios)} scenarios x {len(args.modes)} modes x {len(args.seeds)} seeds')
    print(f'Timeout per trial: {args.timeout}s')
    print(f'Output dir: {args.output_dir}')

    results = []
    trial_idx = 0

    for scenario in args.scenarios:
        for mode in args.modes:
            for seed in args.seeds:
                trial_idx += 1
                print(f'\n[{trial_idx}/{total_trials}] {scenario} / {mode} / seed={seed}')
                t0 = time.time()
                run_trial(scenario, mode, seed, args.timeout, args.output_dir, gui=args.gui)
                dt = time.time() - t0
                results.append({
                    'scenario': scenario,
                    'mode': mode,
                    'seed': seed,
                    'duration_s': f'{dt:.1f}',
                })
                # Small pause between trials
                time.sleep(2)

    # Write aggregate results
    agg_path = os.path.join(args.output_dir, f'experiment_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
    with open(agg_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['scenario', 'mode', 'seed', 'duration_s'])
        w.writeheader()
        for r in results:
            w.writerow(r)

    print(f'\nAll {total_trials} trials complete. Log saved to {agg_path}')


if __name__ == '__main__':
    main()
