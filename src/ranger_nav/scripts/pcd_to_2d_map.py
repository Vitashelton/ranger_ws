#!/usr/bin/env python3
"""
Convert FAST-LIO2 3D pointcloud map (PCD) to a 2D occupancy grid map
suitable for Nav2 navigation.

Usage:
  # Step 1: While FAST-LIO2 is running, call the save PCD service:
  ros2 service call /fast_lio/save_map fast_lio/srv/SaveMap

  # Step 2: Convert the saved PCD to a 2D grid map:
  python3 pcd_to_2d_map.py input.pcd ranger_map --resolution 0.05 --z_min 0.2 --z_max 2.0

  # Output: ranger_map.pgm + ranger_map.yaml (Nav2-compatible)

Dependencies:
  pip install open3d numpy pyyaml
"""
import argparse
import os
import sys

import numpy as np
import yaml

try:
    import open3d as o3d
except ImportError:
    print("ERROR: open3d not installed. Run: pip install open3d")
    sys.exit(1)


def pcd_to_grid(
    pcd_path: str,
    output_prefix: str,
    resolution: float = 0.05,
    z_min: float = 0.2,
    z_max: float = 2.0,
    ground_z: float = -99.0,
    dilate_radius: int = 1,
):
    """
    Convert a 3D PCD pointcloud to a 2D occupancy grid map.

    Parameters
    ----------
    pcd_path : str
        Path to input PCD file.
    output_prefix : str
        Output file prefix (produces <prefix>.pgm and <prefix>.yaml).
    resolution : float
        Grid cell size in meters.
    z_min, z_max : float
        Height band to consider. Points outside this range are ignored.
    ground_z : float
        If > -99, treat points below this z as ground (free space).
        Points between ground_z and z_max are obstacles.
    dilate_radius : int
        Number of cells to dilate obstacles (inflates walls for safety).
    """
    # --- Load pointcloud ---
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"ERROR: {pcd_path} is empty or could not be read.")
        sys.exit(1)

    pts = np.asarray(pcd.points)
    print(f"Loaded {len(pts)} points from {pcd_path}")

    # --- Height filter ---
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pts_filt = pts[mask]
    n_rejected = len(pts) - len(pts_filt)
    print(f"Height filter [{z_min}, {z_max}] m: kept {len(pts_filt)}, rejected {n_rejected}")

    if len(pts_filt) == 0:
        print("ERROR: No points remain after height filter. Check z_min/z_max.")
        sys.exit(1)

    # --- Ground removal (optional) ---
    if ground_z > -99:
        ground_mask = pts_filt[:, 2] <= ground_z
        pts_obstacle = pts_filt[~ground_mask]
        print(f"Ground <= {ground_z}m: {ground_mask.sum()} points removed as free space")
    else:
        pts_obstacle = pts_filt
        # If no explicit ground, treat all points as obstacle
        print("No ground filter; all in-band points treated as obstacles.")

    # --- 2D projection ---
    x = pts_obstacle[:, 0]
    y = pts_obstacle[:, 1]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    print(f"Bounds: x=[{x_min:.2f}, {x_max:.2f}], y=[{y_min:.2f}, {y_max:.2f}]")

    cols = int(np.ceil((x_max - x_min) / resolution)) + 1
    rows = int(np.ceil((y_max - y_min) / resolution)) + 1
    print(f"Grid: {cols} x {rows} cells")

    grid = np.zeros((rows, cols), dtype=np.int32)

    for px, py in zip(x, y):
        ci = int((px - x_min) / resolution)
        ri = int((py - y_min) / resolution)
        if 0 <= ri < rows and 0 <= ci < cols:
            grid[ri, ci] += 1

    # Binarize: hit if >= 1 point in cell
    occupied = grid >= 1

    # --- Dilate ---
    if dilate_radius > 0:
        from scipy.ndimage import binary_dilation
        occupied = binary_dilation(occupied, iterations=dilate_radius)

    # Map origin: lower-left corner of grid in world coordinates
    origin_x = x_min
    origin_y = y_min

    # --- PGM output (Nav2 convention: white=free, black=occupied, gray=unknown) ---
    # 0   = occupied (black)
    # 205 = unknown  (gray,  205/255 ~ 0.8)
    # 254 = free     (white, 254/255 ~ 1.0)
    # Values from OccupancyGrid spec: free=0, occupied=100, unknown=-1
    # In PGM: black (low value) = obstacle, white (high value) = free
    pgm = np.full((rows, cols), 205, dtype=np.uint8)  # unknown (gray)
    pgm[occupied] = 0      # obstacle (black)
    # Free space: everywhere within bounds that isn't occupied
    pgm[pgm == 205] = 254  # free (near-white, but distinguishable from unknown)

    pgm_path = f"{output_prefix}.pgm"
    yaml_path = f"{output_prefix}.yaml"

    # Write PGM (binary format per Nav2 spec)
    from pathlib import Path
    pgm_data = f"P5\n{cols} {rows}\n255\n".encode() + pgm.tobytes()
    Path(pgm_path).write_bytes(pgm_data)
    print(f"Wrote {pgm_path} ({cols}x{rows})")

    # --- YAML metadata ---
    map_metadata = {
        "image": os.path.basename(pgm_path),
        "mode": "trinary",
        "resolution": resolution,
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.25,   # 0~64 gray → occupied
        "free_thresh": 0.65,       # 166~255 gray → free
    }

    with open(yaml_path, "w") as f:
        yaml.dump(map_metadata, f, default_flow_style=False)
    print(f"Wrote {yaml_path}")

    occupied_pct = 100.0 * occupied.sum() / occupied.size
    print(f"\nMap stats: {occupied.sum()} occupied cells ({occupied_pct:.1f}% of {rows*cols})")

    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(
        description="Convert 3D PCD pointcloud to 2D Nav2 occupancy grid map"
    )
    parser.add_argument("pcd", help="Input PCD file path")
    parser.add_argument("output", help="Output prefix (e.g. ranger_map → ranger_map.pgm + .yaml)")
    parser.add_argument("--resolution", type=float, default=0.05, help="Grid resolution (m)")
    parser.add_argument("--z_min", type=float, default=0.2, help="Min height to keep (m)")
    parser.add_argument("--z_max", type=float, default=2.0, help="Max height to keep (m)")
    parser.add_argument("--ground_z", type=float, default=-99.0,
                        help="Height below which points are ground/free (m). "
                             "Default -99 disables ground filtering.")
    parser.add_argument("--dilate", type=int, default=2, help="Obstacle dilation radius (cells)")
    args = parser.parse_args()

    pcd_to_grid(
        pcd_path=args.pcd,
        output_prefix=args.output,
        resolution=args.resolution,
        z_min=args.z_min,
        z_max=args.z_max,
        ground_z=args.ground_z,
        dilate_radius=args.dilate,
    )


if __name__ == "__main__":
    main()
