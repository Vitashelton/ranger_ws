#!/usr/bin/env python3
"""Geometry utilities shared across nodes."""

import math


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_euler(qx, qy, qz, qw):
    """Return (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def project_bbox_to_3d(bbox_xyxy, depth_image, camera_info, depth_scale=0.001):
    """
    Project a 2D bounding box center to 3D using depth image and camera intrinsics.

    Args:
        bbox_xyxy: [x1, y1, x2, y2] in pixel coordinates
        depth_image: numpy uint16 depth image
        camera_info: sensor_msgs/CameraInfo
        depth_scale: meters per depth unit

    Returns:
        (x, y, z) in camera optical frame, or None if projection fails
    """
    import numpy as np
    x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
    cx = int((x1 + x2) // 2)
    cy = int((y1 + y2) // 2)
    h_img, w_img = depth_image.shape[:2]
    if cx < 0 or cy < 0 or cx >= w_img or cy >= h_img:
        return None
    # Take median depth in a small window around center
    r = 5
    x0 = max(0, cx - r)
    x1_w = min(w_img, cx + r)
    y0 = max(0, cy - r)
    y1_w = min(h_img, cy + r)
    patch = depth_image[y0:y1_w, x0:x1_w].astype(np.float32)
    if patch.size == 0:
        return None
    patch[patch <= 0] = np.inf
    depth = np.median(patch) * depth_scale
    if depth <= 0.01 or depth > 50.0:
        return None
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx_i = camera_info.k[2]
    cy_i = camera_info.k[5]
    z = depth
    x = (cx - cx_i) * z / fx
    y = (cy - cy_i) * z / fy
    return (float(x), float(y), float(z))
