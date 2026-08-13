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
    hand_hit_count: int = 0
    camera_hit_count: int = 0
    hand_axis_hit_count: int = 0
    hand_box_hit_count: int = 0
    hand_axis_min_distance_m: float = float("inf")


def depth_to_robot_obstacles(
    depth_img: np.ndarray,
    camera,
    transformer,
    flange_pose_mm: Sequence[float],
    stride: int = 8,
    min_depth_m: float = 0.08,
    max_depth_m: float = 1.20,
    target_depth_m: Optional[float] = None,
    target_depth_margin_m: float = 0.06,
    exclude_bbox: Optional[Tuple[float, float, float, float]] = None,
    exclude_bboxes: Optional[Sequence[Tuple[float, float, float, float]]] = None,
    exclude_margin_px: int = 24,
    exclude_self_hand: bool = False,
    self_hand_dir_m: Optional[Sequence[float]] = None,
    self_hand_length_m: float = 0.25,
    self_hand_radius_m: float = 0.07,
    self_hand_radius_profile_m: Optional[Sequence[Tuple[float, float, float]]] = None,
    self_hand_backoff_m: float = 0.03,
    self_camera_box_min_local_m: Optional[Sequence[float]] = None,
    self_camera_box_max_local_m: Optional[Sequence[float]] = None,
    noise_filter_radius_px: int = 0,
    noise_filter_min_support: int = 1,
    noise_filter_tolerance_m: float = 0.025,
    return_pixels: bool = False,
    self_hand_boxes_local_m: Optional[
        Sequence[Tuple[Sequence[float], Sequence[float]]]
    ] = None,
) -> np.ndarray:
    if depth_img is None:
        points = np.zeros((0, 3), dtype=np.float32)
        pixels = np.zeros((0, 2), dtype=np.int32)
        return (points, pixels) if return_pixels else points

    h, w = depth_img.shape[:2]
    ys = np.arange(0, h, max(1, int(stride)), dtype=np.int32)
    xs = np.arange(0, w, max(1, int(stride)), dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)

    depth_scale = getattr(camera, "_depth_scale", None) or 0.001
    depth_m = depth_img[grid_y, grid_x].astype(np.float32) * float(depth_scale)
    valid = (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    if target_depth_m is not None and np.isfinite(float(target_depth_m)):
        # Points materially farther than the selected product are usually the
        # rear shelf/back wall. They must not veto the product's approach path.
        valid &= depth_m <= float(target_depth_m) + max(0.0, float(target_depth_margin_m))

    # Reject isolated depth speckles before projecting them into the robot frame.
    # A real shelf or bottle surface has nearby pixels at a similar depth; a
    # single flying pixel should not become an obstacle capsule hit.
    filter_radius = max(0, int(noise_filter_radius_px))
    min_support = max(1, int(noise_filter_min_support))
    if filter_radius > 0 and min_support > 1:
        support = np.zeros_like(valid, dtype=np.int16)
        for dy in range(-filter_radius, filter_radius + 1):
            for dx in range(-filter_radius, filter_radius + 1):
                sample_x = grid_x + dx
                sample_y = grid_y + dy
                inside = (
                    (sample_x >= 0) & (sample_x < w)
                    & (sample_y >= 0) & (sample_y < h)
                )
                neighbor = depth_img[
                    np.clip(sample_y, 0, h - 1),
                    np.clip(sample_x, 0, w - 1),
                ].astype(np.float32) * float(depth_scale)
                neighbor_valid = (neighbor >= min_depth_m) & (neighbor <= max_depth_m)
                support += (
                    inside & neighbor_valid
                    & (np.abs(neighbor - depth_m) <= float(noise_filter_tolerance_m))
                ).astype(np.int16)
        valid &= support >= min_support

    boxes = []
    if exclude_bbox is not None:
        boxes.append(exclude_bbox)
    if exclude_bboxes is not None:
        boxes.extend(b for b in exclude_bboxes if b is not None)
    if boxes:
        in_excluded = np.zeros_like(valid, dtype=bool)
        margin = max(0, int(exclude_margin_px))
        for bbox in boxes:
            x1, y1, x2, y2 = bbox
            x1 = max(0, int(x1) - margin)
            y1 = max(0, int(y1) - margin)
            x2 = min(w - 1, int(x2) + margin)
            y2 = min(h - 1, int(y2) + margin)
            if x2 < x1 or y2 < y1:
                continue
            in_excluded |= (grid_x >= x1) & (grid_x <= x2) & (grid_y >= y1) & (grid_y <= y2)
        valid &= ~in_excluded

    if not np.any(valid):
        points = np.zeros((0, 3), dtype=np.float32)
        pixels = np.zeros((0, 2), dtype=np.int32)
        return (points, pixels) if return_pixels else points

    u = grid_x[valid].astype(np.float32)
    v = grid_y[valid].astype(np.float32)
    z = depth_m[valid]

    # Keep obstacle projection identical to grasp-point projection.  The
    # normal hand-eye transformer may apply calibrated lens/perspective
    # compensation (currently on the image Y axis); rebuilding the pinhole
    # equations here made the guard drift increasingly for off-centre targets.
    pixel_to_camera = getattr(transformer, "pixel_to_camera", None)
    if callable(pixel_to_camera):
        try:
            cam_xyz = np.asarray(pixel_to_camera(u, v, z), dtype=np.float32)
            if cam_xyz.shape == (len(z), 3):
                cam_pts = np.concatenate(
                    [cam_xyz, np.ones((len(z), 1), dtype=np.float32)], axis=1
                )
            else:
                raise ValueError("unexpected pixel_to_camera shape")
        except (TypeError, ValueError):
            # Existing lightweight test/dummy transformers often implement
            # only scalar conversion; retain a compatible scalar fallback.
            cam_pts = np.asarray(
                [pixel_to_camera(float(px), float(py), float(pz)) for px, py, pz in zip(u, v, z)],
                dtype=np.float32,
            )
            cam_pts = np.concatenate(
                [cam_pts.reshape(-1, 3), np.ones((len(z), 1), dtype=np.float32)], axis=1
            )
    else:
        x = (u - float(transformer.cx)) * z / float(transformer.fx)
        y = (v - float(transformer.cy)) * z / float(transformer.fy)
        cam_pts = np.stack([x, y, z, np.ones_like(z)], axis=1)

    pose = tuple(float(x) for x in flange_pose_mm)
    T_base_flange = np.eye(4, dtype=np.float32)
    T_base_flange[:3, :3] = transformer._rpy_to_rot(pose[3], pose[4], pose[5]).astype(np.float32)
    T_base_flange[:3, 3] = np.array(pose[:3], dtype=np.float32) / 1000.0
    T = T_base_flange @ transformer.T_flange_to_cam.astype(np.float32)
    base_pts = (T @ cam_pts.T).T[:, :3]
    base_pts = base_pts.astype(np.float32, copy=False)
    pixels = np.stack([u.astype(np.int32), v.astype(np.int32)], axis=1)

    if exclude_self_hand and base_pts.size:
        flange_xyz = np.asarray(pose[:3], dtype=np.float32) / 1000.0
        if self_hand_dir_m is not None:
            hand_dir = np.asarray(self_hand_dir_m, dtype=np.float32)
            hand_dir = hand_dir / max(float(np.linalg.norm(hand_dir)), 1e-9)
            hand_start = flange_xyz - float(self_hand_backoff_m) * hand_dir
            hand_end = flange_xyz + float(self_hand_length_m) * hand_dir
            self_dist, self_closest = _point_segment_distances_and_closest(base_pts, hand_start, hand_end)
            self_axial = (self_closest - flange_xyz[None, :]) @ hand_dir
            self_radius = _radius_for_axial(self_axial, float(self_hand_radius_m), self_hand_radius_profile_m)
            keep = self_dist > self_radius
            base_pts = base_pts[keep]
            pixels = pixels[keep]

        if base_pts.size and self_hand_boxes_local_m:
            for box_min, box_max in self_hand_boxes_local_m:
                hand_box_dist, _ = _point_oriented_box_distances_and_closest(
                    base_pts,
                    flange_xyz,
                    T_base_flange[:3, :3],
                    box_min,
                    box_max,
                )
                keep = hand_box_dist > 1e-6
                base_pts = base_pts[keep]
                pixels = pixels[keep]
                if not base_pts.size:
                    break

        if (
            base_pts.size
            and self_camera_box_min_local_m is not None
            and self_camera_box_max_local_m is not None
        ):
            camera_dist, _ = _point_oriented_box_distances_and_closest(
                base_pts,
                flange_xyz,
                T_base_flange[:3, :3],
                self_camera_box_min_local_m,
                self_camera_box_max_local_m,
            )
            keep = camera_dist > 1e-6
            base_pts = base_pts[keep]
            pixels = pixels[keep]

    if return_pixels:
        return base_pts, pixels
    return base_pts


def _point_segment_distances_and_closest(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        closest = np.repeat(a[None, :], points.shape[0], axis=0)
        return np.linalg.norm(points - closest, axis=1), closest
    t = ((points - a[None, :]) @ ab) / denom
    t = np.clip(t, 0.0, 1.0)
    closest = a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(points - closest, axis=1), closest


def _point_segment_distances(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    distances, _ = _point_segment_distances_and_closest(points, a, b)
    return distances


def _point_oriented_box_distances_and_closest(
    points: np.ndarray,
    origin: np.ndarray,
    rotation: np.ndarray,
    box_min_local_m: Sequence[float],
    box_max_local_m: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return point-to-OBB surface distances and closest world points."""
    points = np.asarray(points, dtype=np.float32)
    origin = np.asarray(origin, dtype=np.float32)
    rotation = np.asarray(rotation, dtype=np.float32).reshape(3, 3)
    box_min = np.asarray(box_min_local_m, dtype=np.float32)
    box_max = np.asarray(box_max_local_m, dtype=np.float32)
    lo = np.minimum(box_min, box_max)
    hi = np.maximum(box_min, box_max)
    local = (points - origin[None, :]) @ rotation
    closest_local = np.clip(local, lo, hi)
    closest_world = origin[None, :] + closest_local @ rotation.T
    distances = np.linalg.norm(points - closest_world, axis=1)
    return distances.astype(np.float32), closest_world.astype(np.float32)


def _radius_for_axial(
    axial_m: np.ndarray,
    default_radius_m: float,
    radius_profile_m: Optional[Sequence[Tuple[float, float, float]]],
) -> np.ndarray:
    radius = np.full(axial_m.shape, float(default_radius_m), dtype=np.float32)
    if not radius_profile_m:
        return radius
    for start_m, end_m, radius_m in radius_profile_m:
        lo = min(float(start_m), float(end_m))
        hi = max(float(start_m), float(end_m))
        mask = (axial_m >= lo) & (axial_m <= hi)
        radius[mask] = float(radius_m)
    return radius


def _camera_depth_aligned(
    points: np.ndarray,
    closest: np.ndarray,
    camera_depth_axis_m: Optional[Sequence[float]],
    tolerance_m: Optional[float],
) -> np.ndarray:
    if camera_depth_axis_m is None or tolerance_m is None or tolerance_m <= 0:
        return np.ones((points.shape[0],), dtype=bool)
    axis = np.asarray(camera_depth_axis_m, dtype=np.float32)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)
    depth_delta = np.abs((points - closest) @ axis)
    return depth_delta <= float(tolerance_m)


def check_hand_path(
    obstacle_points_m: np.ndarray,
    flange_start_m: Sequence[float],
    flange_end_m: Sequence[float],
    hand_dir_m: Sequence[float],
    hand_length_m: float,
    hand_radius_m: float,
    hand_radius_profile_m: Optional[Sequence[Tuple[float, float, float]]] = None,
    samples: int = 8,
    camera_depth_axis_m: Optional[Sequence[float]] = None,
    camera_depth_tolerance_m: Optional[float] = None,
    axis_min_m: Optional[float] = None,
    axis_max_m: Optional[float] = None,
    flange_rotation_m: Optional[np.ndarray] = None,
    camera_box_min_local_m: Optional[Sequence[float]] = None,
    camera_box_max_local_m: Optional[Sequence[float]] = None,
    min_hit_points: int = 1,
    hand_box_min_hit_points: Optional[int] = None,
    camera_min_hit_points: Optional[int] = None,
    hand_boxes_local_m: Optional[
        Sequence[Tuple[Sequence[float], Sequence[float]]]
    ] = None,
) -> DepthGuardResult:
    points = np.asarray(obstacle_points_m, dtype=np.float32)
    if points.size == 0:
        return DepthGuardResult(True, float("inf"), 0, 0)

    start = np.asarray(flange_start_m, dtype=np.float32)
    end = np.asarray(flange_end_m, dtype=np.float32)
    hand_dir = np.asarray(hand_dir_m, dtype=np.float32)
    hand_dir = hand_dir / max(float(np.linalg.norm(hand_dir)), 1e-9)
    flange_rotation = (
        np.asarray(flange_rotation_m, dtype=np.float32).reshape(3, 3)
        if flange_rotation_m is not None
        else np.eye(3, dtype=np.float32)
    )
    check_camera = camera_box_min_local_m is not None and camera_box_max_local_m is not None

    min_dist = float("inf")
    hand_axis_min_dist = float("inf")
    hand_axis_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    hand_box_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    camera_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    for alpha in np.linspace(0.0, 1.0, max(2, int(samples))):
        flange = (1.0 - alpha) * start + alpha * end
        fingertip = flange + float(hand_length_m) * hand_dir
        d, closest = _point_segment_distances_and_closest(points, flange, fingertip)
        if d.size:
            depth_ok = _camera_depth_aligned(points, closest, camera_depth_axis_m, camera_depth_tolerance_m)
            point_axial = (points - flange[None, :]) @ hand_dir
            closest_axial = (closest - flange[None, :]) @ hand_dir
            if axis_min_m is not None or axis_max_m is not None:
                if axis_min_m is not None:
                    depth_ok &= point_axial >= float(axis_min_m)
                if axis_max_m is not None:
                    depth_ok &= point_axial <= float(axis_max_m)
            if np.any(depth_ok):
                local_min = float(np.min(d[depth_ok]))
                min_dist = min(min_dist, local_min)
                hand_axis_min_dist = min(hand_axis_min_dist, local_min)
            radius = _radius_for_axial(closest_axial, float(hand_radius_m), hand_radius_profile_m)
            sample_axis_hits = (d < radius) & depth_ok
            sample_box_hits = np.zeros((points.shape[0],), dtype=bool)
            sample_camera_hits = np.zeros((points.shape[0],), dtype=bool)

            for box_min, box_max in hand_boxes_local_m or ():
                hand_box_d, hand_box_closest = _point_oriented_box_distances_and_closest(
                    points,
                    flange,
                    flange_rotation,
                    box_min,
                    box_max,
                )
                hand_box_ok = _camera_depth_aligned(
                    points,
                    hand_box_closest,
                    camera_depth_axis_m,
                    camera_depth_tolerance_m,
                )
                if axis_min_m is not None:
                    hand_box_ok &= point_axial >= float(axis_min_m)
                if axis_max_m is not None:
                    hand_box_ok &= point_axial <= float(axis_max_m)
                if np.any(hand_box_ok):
                    min_dist = min(min_dist, float(np.min(hand_box_d[hand_box_ok])))
                sample_box_hits |= (hand_box_d <= 1e-6) & hand_box_ok

            if check_camera:
                camera_d, camera_closest = _point_oriented_box_distances_and_closest(
                    points,
                    flange,
                    flange_rotation,
                    camera_box_min_local_m,
                    camera_box_max_local_m,
                )
                camera_ok = _camera_depth_aligned(
                    points,
                    camera_closest,
                    camera_depth_axis_m,
                    camera_depth_tolerance_m,
                )
                if axis_min_m is not None:
                    camera_ok &= point_axial >= float(axis_min_m)
                if axis_max_m is not None:
                    camera_ok &= point_axial <= float(axis_max_m)
                if np.any(camera_ok):
                    min_dist = min(min_dist, float(np.min(camera_d[camera_ok])))
                sample_camera_hits = (camera_d <= 1e-6) & camera_ok

            hand_axis_hit_mask |= sample_axis_hits
            hand_box_hit_mask |= sample_box_hits
            camera_hit_mask |= sample_camera_hits

    hand_hit_mask = hand_axis_hit_mask | hand_box_hit_mask
    hit_mask = hand_hit_mask | camera_hit_mask
    hit_count = int(np.count_nonzero(hit_mask))
    hand_hit_count = int(np.count_nonzero(hand_hit_mask))
    hand_axis_hit_count = int(np.count_nonzero(hand_axis_hit_mask))
    hand_box_hit_count = int(np.count_nonzero(hand_box_hit_mask))
    camera_hit_count = int(np.count_nonzero(camera_hit_mask))
    hand_required = max(1, int(min_hit_points))
    hand_box_required = max(
        1,
        int(hand_required if hand_box_min_hit_points is None else hand_box_min_hit_points),
    )
    camera_required = max(
        1,
        int(hand_required if camera_min_hit_points is None else camera_min_hit_points),
    )
    ok = (
        hand_axis_hit_count < hand_required
        and hand_box_hit_count < hand_box_required
        and camera_hit_count < camera_required
    )
    return DepthGuardResult(
        ok,
        min_dist,
        hit_count,
        int(points.shape[0]),
        hand_hit_count,
        camera_hit_count,
        hand_axis_hit_count,
        hand_box_hit_count,
        hand_axis_min_dist,
    )


def hand_path_hit_mask(
    obstacle_points_m: np.ndarray,
    flange_start_m: Sequence[float],
    flange_end_m: Sequence[float],
    hand_dir_m: Sequence[float],
    hand_length_m: float,
    hand_radius_m: float,
    hand_radius_profile_m: Optional[Sequence[Tuple[float, float, float]]] = None,
    samples: int = 8,
    camera_depth_axis_m: Optional[Sequence[float]] = None,
    camera_depth_tolerance_m: Optional[float] = None,
    axis_min_m: Optional[float] = None,
    axis_max_m: Optional[float] = None,
    flange_rotation_m: Optional[np.ndarray] = None,
    camera_box_min_local_m: Optional[Sequence[float]] = None,
    camera_box_max_local_m: Optional[Sequence[float]] = None,
    return_components: bool = False,
    return_hand_components: bool = False,
    hand_boxes_local_m: Optional[
        Sequence[Tuple[Sequence[float], Sequence[float]]]
    ] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    points = np.asarray(obstacle_points_m, dtype=np.float32)
    if points.size == 0:
        empty_mask = np.zeros((0,), dtype=bool)
        empty_distance = np.zeros((0,), dtype=np.float32)
        if return_components and return_hand_components:
            return (
                empty_mask,
                empty_distance,
                empty_mask.copy(),
                empty_mask.copy(),
                empty_mask.copy(),
                empty_mask.copy(),
                empty_distance.copy(),
            )
        if return_components:
            return empty_mask, empty_distance, empty_mask.copy(), empty_mask.copy()
        return empty_mask, empty_distance

    start = np.asarray(flange_start_m, dtype=np.float32)
    end = np.asarray(flange_end_m, dtype=np.float32)
    hand_dir = np.asarray(hand_dir_m, dtype=np.float32)
    hand_dir = hand_dir / max(float(np.linalg.norm(hand_dir)), 1e-9)
    flange_rotation = (
        np.asarray(flange_rotation_m, dtype=np.float32).reshape(3, 3)
        if flange_rotation_m is not None
        else np.eye(3, dtype=np.float32)
    )
    check_camera = camera_box_min_local_m is not None and camera_box_max_local_m is not None

    min_dist = np.full((points.shape[0],), np.inf, dtype=np.float32)
    hand_axis_min_dist = np.full((points.shape[0],), np.inf, dtype=np.float32)
    hand_axis_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    hand_box_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    camera_hit_mask = np.zeros((points.shape[0],), dtype=bool)
    for alpha in np.linspace(0.0, 1.0, max(2, int(samples))):
        flange = (1.0 - alpha) * start + alpha * end
        fingertip = flange + float(hand_length_m) * hand_dir
        d, closest = _point_segment_distances_and_closest(points, flange, fingertip)
        d = d.astype(np.float32)
        depth_ok = _camera_depth_aligned(points, closest, camera_depth_axis_m, camera_depth_tolerance_m)
        point_axial = (points - flange[None, :]) @ hand_dir
        closest_axial = (closest - flange[None, :]) @ hand_dir
        if axis_min_m is not None or axis_max_m is not None:
            if axis_min_m is not None:
                depth_ok &= point_axial >= float(axis_min_m)
            if axis_max_m is not None:
                depth_ok &= point_axial <= float(axis_max_m)
        min_dist[depth_ok] = np.minimum(min_dist[depth_ok], d[depth_ok])
        hand_axis_min_dist[depth_ok] = np.minimum(hand_axis_min_dist[depth_ok], d[depth_ok])
        radius = _radius_for_axial(closest_axial, float(hand_radius_m), hand_radius_profile_m)
        hand_axis_hit_mask |= (d < radius) & depth_ok
        for box_min, box_max in hand_boxes_local_m or ():
            hand_box_d, hand_box_closest = _point_oriented_box_distances_and_closest(
                points,
                flange,
                flange_rotation,
                box_min,
                box_max,
            )
            hand_box_ok = _camera_depth_aligned(
                points,
                hand_box_closest,
                camera_depth_axis_m,
                camera_depth_tolerance_m,
            )
            if axis_min_m is not None:
                hand_box_ok &= point_axial >= float(axis_min_m)
            if axis_max_m is not None:
                hand_box_ok &= point_axial <= float(axis_max_m)
            min_dist[hand_box_ok] = np.minimum(min_dist[hand_box_ok], hand_box_d[hand_box_ok])
            hand_box_hit_mask |= (hand_box_d <= 1e-6) & hand_box_ok
        if check_camera:
            camera_d, camera_closest = _point_oriented_box_distances_and_closest(
                points,
                flange,
                flange_rotation,
                camera_box_min_local_m,
                camera_box_max_local_m,
            )
            camera_ok = _camera_depth_aligned(
                points,
                camera_closest,
                camera_depth_axis_m,
                camera_depth_tolerance_m,
            )
            if axis_min_m is not None:
                camera_ok &= point_axial >= float(axis_min_m)
            if axis_max_m is not None:
                camera_ok &= point_axial <= float(axis_max_m)
            min_dist[camera_ok] = np.minimum(min_dist[camera_ok], camera_d[camera_ok])
            camera_hit_mask |= (camera_d <= 1e-6) & camera_ok
    hand_hit_mask = hand_axis_hit_mask | hand_box_hit_mask
    hit_mask = hand_hit_mask | camera_hit_mask
    if return_components and return_hand_components:
        return (
            hit_mask,
            min_dist,
            hand_hit_mask,
            camera_hit_mask,
            hand_axis_hit_mask,
            hand_box_hit_mask,
            hand_axis_min_dist,
        )
    if return_components:
        return hit_mask, min_dist, hand_hit_mask, camera_hit_mask
    return hit_mask, min_dist
