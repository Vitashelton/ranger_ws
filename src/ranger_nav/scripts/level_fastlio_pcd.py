#!/usr/bin/env python3
"""Transform a binary FAST-LIO PCD from camera_init into mapping_view."""

import argparse
import math
import os
from pathlib import Path
import tempfile

import numpy as np


TYPE_MAP = {
    'F': {4: '<f4', 8: '<f8'},
    'U': {1: 'u1', 2: '<u2', 4: '<u4'},
    'I': {1: 'i1', 2: '<i2', 4: '<i4'},
}


def load_binary_pcd(path):
    header_lines = []
    with path.open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError('PCD header has no DATA line')
            decoded = line.decode('ascii').strip()
            header_lines.append(decoded)
            if decoded.startswith('DATA '):
                break
        metadata = {
            line.split()[0]: line.split()[1:]
            for line in header_lines if line and not line.startswith('#')
        }
        if metadata.get('DATA') != ['binary']:
            raise RuntimeError('only DATA binary PCD files are supported')
        fields = metadata['FIELDS']
        sizes = list(map(int, metadata['SIZE']))
        types = metadata['TYPE']
        counts = list(map(int, metadata.get('COUNT', ['1'] * len(fields))))
        description = []
        for name, size, field_type, count in zip(
                fields, sizes, types, counts):
            scalar_type = TYPE_MAP[field_type][size]
            description.append(
                (name, scalar_type) if count == 1
                else (name, scalar_type, (count,)))
        points = np.fromfile(
            stream, dtype=np.dtype(description),
            count=int(metadata['POINTS'][0]))
    return header_lines, points


def transform(path_in, path_out, pitch_deg, translation):
    header, points = load_binary_pcd(path_in)
    angle = math.radians(pitch_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    rotation = np.array([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ], dtype=np.float64)
    xyz = np.column_stack(
        [points['x'], points['y'], points['z']]).astype(np.float64)
    xyz = xyz @ rotation.T + np.asarray(translation, dtype=np.float64)
    points['x'], points['y'], points['z'] = xyz.T

    names = points.dtype.names or ()
    if all(name in names for name in ('normal_x', 'normal_y', 'normal_z')):
        normals = np.column_stack([
            points['normal_x'], points['normal_y'], points['normal_z'],
        ]).astype(np.float64)
        normals = normals @ rotation.T
        points['normal_x'], points['normal_y'], points['normal_z'] = normals.T

    path_out.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(
        prefix=path_out.name + '.', suffix='.tmp', dir=str(path_out.parent))
    try:
        with os.fdopen(handle, 'wb') as stream:
            stream.write(('\n'.join(header) + '\n').encode('ascii'))
            points.tofile(stream)
        os.replace(temporary, path_out)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    low = np.percentile(xyz, 0.1, axis=0)
    high = np.percentile(xyz, 99.9, axis=0)
    print(f'leveled PCD: {path_out}')
    print(f'points={len(points)} bounds_0.1={low.round(3)} bounds_99.9={high.round(3)}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--pitch-deg', type=float, default=30.0)
    parser.add_argument('--translation', type=float, nargs=3,
                        default=(0.30, 0.0, 0.70),
                        metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    transform(
        args.input.expanduser(), args.output.expanduser(),
        args.pitch_deg, args.translation)


if __name__ == '__main__':
    main()
