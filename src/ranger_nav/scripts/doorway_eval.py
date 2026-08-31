#!/usr/bin/env python3
"""Aggregate fixed-label doorway bags into frame- and trial-level CSV metrics."""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


METHODS = (
    'lidar_only',
    'depth_only',
    'naive_late_fusion',
    'async_conservative',
)
KNOWN = {'PASSABLE', 'BLOCKED'}


def majority_prediction(predictions):
    counts = Counter(item for item in predictions if item in KNOWN)
    if not counts:
        return 'UNKNOWN'
    if counts['PASSABLE'] == counts['BLOCKED']:
        return 'UNKNOWN'
    return counts.most_common(1)[0][0]


def metric_row(method, level, pairs):
    total = len(pairs)
    known = [(truth, pred) for truth, pred in pairs if pred in KNOWN]
    correct = sum(truth == pred for truth, pred in known)
    blocked_total = sum(truth == 'BLOCKED' for truth, _ in pairs)
    passable_total = sum(truth == 'PASSABLE' for truth, _ in pairs)
    false_pass = sum(
        truth == 'BLOCKED' and pred == 'PASSABLE' for truth, pred in pairs)
    false_block = sum(
        truth == 'PASSABLE' and pred == 'BLOCKED' for truth, pred in pairs)
    return {
        'method': method,
        'level': level,
        'samples': total,
        'coverage': len(known) / total if total else 0.0,
        'selective_accuracy': correct / len(known) if known else 0.0,
        'unknown_rate': 1.0 - len(known) / total if total else 0.0,
        'false_pass_rate': false_pass / blocked_total if blocked_total else 0.0,
        'false_block_rate': false_block / passable_total if passable_total else 0.0,
    }


def read_evidence(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()}
    message_type = get_message(topic_types['/doorway/evidence'])
    rows = []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != '/doorway/evidence':
            continue
        message = deserialize_message(serialized, message_type)
        rows.append(json.loads(message.data))
    return rows


def resolve_bags(paths):
    bags = []
    for raw in paths:
        path = Path(raw).expanduser()
        if (path / 'metadata.yaml').exists():
            bags.append(path)
        elif (path / 'bag' / 'metadata.yaml').exists():
            bags.append(path / 'bag')
        elif path.is_dir():
            bags.extend(sorted(path.glob('*/bag')))
    return [path for path in bags if (path / 'metadata.yaml').exists()]


def evaluate(bag_paths):
    frame_pairs = defaultdict(list)
    trial_pairs = defaultdict(list)
    trial_details = []
    for bag in bag_paths:
        payloads = read_evidence(bag)
        valid = [
            item for item in payloads
            if item.get('ground_truth') in KNOWN and isinstance(item.get('methods'), dict)]
        if not valid:
            continue
        truth = Counter(item['ground_truth'] for item in valid).most_common(1)[0][0]
        detail = {'bag': str(bag), 'ground_truth': truth, 'frames': len(valid)}
        for method in METHODS:
            predictions = [item['methods'].get(method, 'UNKNOWN') for item in valid]
            frame_pairs[method].extend((truth, prediction) for prediction in predictions)
            trial_prediction = majority_prediction(predictions)
            trial_pairs[method].append((truth, trial_prediction))
            detail[method] = trial_prediction
        trial_details.append(detail)
    summary = []
    for method in METHODS:
        summary.append(metric_row(method, 'frame', frame_pairs[method]))
        summary.append(metric_row(method, 'trial', trial_pairs[method]))
    return summary, trial_details


def write_csv(path, rows):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', nargs='+', help='bag、session或doorway_bags根目录')
    parser.add_argument('--output', type=Path, default=Path('doorway_metrics.csv'))
    args = parser.parse_args()
    bags = resolve_bags(args.paths)
    if not bags:
        raise SystemExit('没有找到包含 metadata.yaml 的 doorway bag')
    summary, trials = evaluate(bags)
    write_csv(args.output, summary)
    write_csv(args.output.with_name(args.output.stem + '_trials.csv'), trials)
    for row in summary:
        print(
            f"{row['method']:24s} {row['level']:5s} n={row['samples']:5d} "
            f"coverage={row['coverage']:.3f} acc={row['selective_accuracy']:.3f} "
            f"false_pass={row['false_pass_rate']:.3f} unknown={row['unknown_rate']:.3f}")
    print(f'指标表：{args.output}')


if __name__ == '__main__':
    main()
