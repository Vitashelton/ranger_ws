#!/usr/bin/env python3
"""
summarize_experiment_csv.py — Aggregate experiment CSVs into paper-ready tables.

Reads all CSV files in a results directory and produces:
  1. summary_ablation.csv — aggregated by (scenario, mode)
  2. Printed LaTeX table rows for easy paper inclusion

Usage:
  python3 scripts/summarize_experiment_csv.py --results-dir ~/ranger_ws/experiments/results/
"""
import argparse
import csv
import glob
import os
import sys
from collections import defaultdict


def aggregate_csvs(results_dir):
    """Read all trial CSVs and aggregate by scenario+mode."""
    csv_files = sorted(glob.glob(os.path.join(results_dir, 'gazebo_people_avoidance_*.csv')))
    # Filter out summary files
    csv_files = [f for f in csv_files if '_summary' not in f]

    if not csv_files:
        print(f'No experiment CSV files found in {results_dir}')
        return []

    print(f'Found {len(csv_files)} trial CSV files')

    # Group data by (scenario, mode)
    grouped = defaultdict(list)

    for fpath in csv_files:
        basename = os.path.basename(fpath)
        # Parse filename: gazebo_people_avoidance_<scenario>_<mode>_s<seed>_<timestamp>.csv
        try:
            parts = basename.replace('.csv', '').split('_')
            # Find indices: gazebo(0) people(1) avoidance(2) <scenario...> _ <mode...> _ s<seed> _ <ts>
            # Heuristic: scenario is parts[3:until mode], mode is [until s<seed>], seed is after s
            # Simpler: parse from known patterns
            s_idx = next(i for i, p in enumerate(parts) if p.startswith('s') and p[1:].isdigit())
            scenario = '_'.join(parts[3:s_idx - 2])  # from 'gazebo_people_avoidance_' to before mode
            mode_parts = parts[s_idx - 2:s_idx]
            mode = '_'.join(mode_parts) if isinstance(mode_parts, list) else '_'.join(mode_parts)
            seed = parts[s_idx]
        except (StopIteration, ValueError, IndexError):
            # Fallback: use raw filename
            scenario = 'unknown'
            mode = 'unknown'
            seed = 'unknown'

        # Parse CSV content
        try:
            with open(fpath, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                if not rows:
                    continue
                last = rows[-1]
                entry = {
                    'scenario': scenario,
                    'mode': mode,
                    'seed': seed,
                    'file': basename,
                    'success': int(last.get('success', 0)),
                    'failure_reason': last.get('failure_reason', ''),
                    'navigation_time': float(last.get('navigation_time', 0)),
                    'path_length': float(last.get('path_length', 0)),
                    'collision_count': int(last.get('collision_count', 0)),
                    'dangerous_close_count': int(last.get('dangerous_close_count', 0)),
                    'min_distance': float(last.get('min_distance_to_person', float('inf'))),
                    'avg_speed': float(last.get('average_speed', 0)),
                }
                key = (scenario, mode)
                grouped[key].append(entry)
        except Exception as e:
            print(f'  Warning: failed to parse {basename}: {e}')
            continue

    return grouped


def compute_summary(grouped):
    """Compute per-(scenario, mode) summary statistics."""
    summary = []
    for (scenario, mode), entries in sorted(grouped.items()):
        n = len(entries)
        successes = [e['success'] for e in entries]
        collisions = [e['collision_count'] for e in entries]
        dangerous = [e['dangerous_close_count'] for e in entries]
        times = [e['navigation_time'] for e in entries]
        paths = [e['path_length'] for e in entries]
        min_dists = [e['min_distance'] for e in entries if e['min_distance'] != float('inf')]

        summary.append({
            'scenario': scenario,
            'mode': mode,
            'trials': n,
            'success_rate': sum(successes) / n if n > 0 else 0,
            'avg_nav_time_s': sum(times) / n if n > 0 else 0,
            'avg_path_length_m': sum(paths) / n if n > 0 else 0,
            'avg_min_distance_m': sum(min_dists) / len(min_dists) if min_dists else 0,
            'total_collisions': sum(collisions),
            'total_dangerous_close': sum(dangerous),
        })
    return summary


def print_latex_table(summary):
    """Print a LaTeX-formatted table."""
    print('\n% LaTeX table for paper')
    print(r'\begin{table}[htbp]')
    print(r'  \centering')
    print(r'  \caption{Ablation experiment results.}')
    print(r'  \label{tab:ablation}')
    print(r'  \begin{tabular}{l l c c c c c c}')
    print(r'    \toprule')
    print(r'    Scenario & Mode & Trials & Success \% & Nav Time (s) & Path Len (m) & Min Dist (m) & Collisions \\')
    print(r'    \midrule')
    for row in summary:
        print(f'    {row["scenario"]} & {row["mode"]} & '
              f'{row["trials"]} & '
              f'{row["success_rate"]:.1%} & '
              f'{row["avg_nav_time_s"]:.1f} & '
              f'{row["avg_path_length_m"]:.2f} & '
              f'{row["avg_min_distance_m"]:.2f} & '
              f'{row["total_collisions"]} \\\\')
    print(r'    \bottomrule')
    print(r'  \end{tabular}')
    print(r'\end{table}')


def main():
    parser = argparse.ArgumentParser(description='Summarize experiment CSV results')
    parser.add_argument('--results-dir', required=True,
                        help='Directory containing experiment CSV files')
    parser.add_argument('--output', default=None,
                        help='Output CSV path for summary (default: results_dir/summary_ablation.csv)')
    parser.add_argument('--latex', action='store_true', default=True,
                        help='Print LaTeX table to stdout')
    args = parser.parse_args()

    grouped = aggregate_csvs(args.results_dir)
    if not grouped:
        sys.exit(1)

    summary = compute_summary(grouped)

    # Write output CSV
    output = args.output or os.path.join(args.results_dir, 'summary_ablation.csv')
    with open(output, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=[
            'scenario', 'mode', 'trials', 'success_rate',
            'avg_nav_time_s', 'avg_path_length_m', 'avg_min_distance_m',
            'total_collisions', 'total_dangerous_close'])
        w.writeheader()
        for row in summary:
            w.writerow(row)
    print(f'Summary written to {output}')

    # LaTeX output
    if args.latex:
        print_latex_table(summary)


if __name__ == '__main__':
    main()
