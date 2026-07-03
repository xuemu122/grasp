#!/usr/bin/env python3
"""Depth-based shelf collision guard.

The guard is intentionally simple: convert a depth snapshot into base-frame
obstacle points, then check whether a coarse hand capsule intersects those
points along a planned flange path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class DepthGuardResult:
    ok: bool
    min_distance_m: float
    hit_count: int
    checked_points: int


def depth_to_robot_obstacles(
    depth_img: np.ndarray,
    camera,
    transformer,
    flange_pose_mm: Sequence[float],
    stride: int = 8,
    min_depth_m: float = 0.08,
    max_depth_m: float = 1.20,
    exclude_bbox: Optional[Tuple[float, float, float, float]] = None,
    exclude_margin_px: int = 24,
) -> np.ndarray:
    if depth_img is None:
        return np.zeros((0, 3), dtype=np.float32)

    h, w = depth_img.shape[:2]
    ys = np.arange(0, h, max(1, int(stride)), dtype=np.int32)
    xs = np.arange(0, w, max(1, int(stride)), dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)

    depth_scale = getattr(camera, "_depth_scale", None) or 0.001
    depth_m = depth_img[grid_y, grid_x].astype(np.float32) * float(depth_scale)
    valid = (depth_m >= min_depth_m) & (depth_m <= max_depth_m)

    if exclude_bbox is not None:
        x1, y1, x2, y2 = exclude_bbox
        x1 = max(0, int(x1) - exclude_margin_px)
        y1 = max(0, int(y1) - exclude_margin_px)
        x2 = min(w - 1, int(x2) + exclude_margin_px)
        y2 = min(h - 1, int(y2) + exclude_margin_px)
        in_target = (grid_x >= x1) & (grid_x <= x2) & (grid_y >= y1) & (grid_y <= y2)
        valid &= ~in_target

    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    u = grid_x[valid].astype(np.float32)
    v = grid_y[valid].astype(np.float32)
    z = depth_m[valid]
    x = (u - float(transformer.cx)) * z / float(transformer.fx)
    y = (v - float(transformer.cy)) * z / float(transformer.fy)
    cam_pts = np.stack([x, y, z, np.ones_like(z)], axis=1)

    pose = tuple(float(x) for x in flange_pose_mm)
    T_base_flange = np.eye(4, dtype=np.float32)
    T_base_flange[:3, :3] = transformer._rpy_to_rot(pose[3], pose[4], pose[5]).astype(np.float32)
    T_base_flange[:3, 3] = np.array(pose[:3], dtype=np.float32) / 1000.0
    T = T_base_flange @ transformer.T_flange_to_cam.astype(np.float32)
    base_pts = (T @ cam_pts.T).T[:, :3]
    return base_pts.astype(np.float32, copy=False)


def _point_segment_distances(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return np.linalg.norm(points - a[None, :], axis=1)
    t = ((points - a[None, :]) @ ab) / denom
    t = np.clip(t, 0.0, 1.0)
    closest = a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(points - closest, axis=1)


def check_hand_path(
    obstacle_points_m: np.ndarray,
    flange_start_m: Sequence[float],
    flange_end_m: Sequence[float],
    hand_dir_m: Sequence[float],
    hand_length_m: float,
    hand_radius_m: float,
    samples: int = 8,
) -> DepthGuardResult:
    points = np.asarray(obstacle_points_m, dtype=np.float32)
    if points.size == 0:
        return DepthGuardResult(True, float("inf"), 0, 0)

    start = np.asarray(flange_start_m, dtype=np.float32)
    end = np.asarray(flange_end_m, dtype=np.float32)
    hand_dir = np.asarray(hand_dir_m, dtype=np.float32)
    hand_dir = hand_dir / max(float(np.linalg.norm(hand_dir)), 1e-9)

    min_dist = float("inf")
    hit_count = 0
    for alpha in np.linspace(0.0, 1.0, max(2, int(samples))):
        flange = (1.0 - alpha) * start + alpha * end
        fingertip = flange + float(hand_length_m) * hand_dir
        d = _point_segment_distances(points, flange, fingertip)
        if d.size:
            local_min = float(np.min(d))
            min_dist = min(min_dist, local_min)
            hit_count += int(np.count_nonzero(d < hand_radius_m))

    return DepthGuardResult(hit_count == 0, min_dist, hit_count, int(points.shape[0]))
