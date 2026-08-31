#!/usr/bin/env python3
"""
Generate printable / Gazebo-texture ArUco markers for room IDs.
Example:
  python3 generate_aruco_room_markers.py --ids 100 101 902 904 --out marker_images
"""
import argparse
from pathlib import Path
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ids', type=int, nargs='+', default=[100, 101, 902, 904])
    parser.add_argument('--out', default='marker_images')
    parser.add_argument('--size', type=int, default=700)
    args = parser.parse_args()

    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('OpenCV was built without cv2.aruco.')

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    for marker_id in args.ids:
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, args.size)
        filename = out / f'aruco_6x6_1000_{marker_id}.png'
        cv2.imwrite(str(filename), marker)
        print(filename)


if __name__ == '__main__':
    main()
