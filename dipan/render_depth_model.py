#!/usr/bin/env python3
"""Render the DH116 collision model for the last logged flange pose."""

import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import cv2
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from hand_eye import HandEyeTransformer
from tool_geometry import HAND_LENGTH_Z_M, camera_box_bounds, hand_box_bounds


POSE_MM_DEG = (-476.3, 245.9, 689.8, 109.0, 82.9, -73.2)
INPUT_IMAGE = Path("/tmp/depth_guard_探入路径.jpg")
_REPO_ROOT = Path(os.environ["GRASP_ROOT"]) if os.environ.get("GRASP_ROOT") else Path(__file__).resolve().parents[1]
OUTPUT_IMAGE = _REPO_ROOT / "depth_guard_model_last_pose.png"


def transform_pose(pose):
    transformer = HandEyeTransformer(fx=390.4, fy=390.0, cx=324.5, cy=241.2)
    rotation = transformer._rpy_to_rot(*pose[3:])
    translation = np.asarray(pose[:3], dtype=float) / 1000.0
    return transformer, rotation, translation


def box_corners(bounds):
    box_min, box_max = [np.asarray(item, dtype=float) for item in bounds]
    return np.asarray([
        [x, y, z]
        for x in (box_min[0], box_max[0])
        for y in (box_min[1], box_max[1])
        for z in (box_min[2], box_max[2])
    ])


def draw_box(ax, corners, color, label, alpha=0.16):
    faces = (
        (0, 1, 3, 2), (4, 5, 7, 6),
        (0, 1, 5, 4), (2, 3, 7, 6),
        (0, 2, 6, 4), (1, 3, 7, 5),
    )
    ax.add_collection3d(
        Poly3DCollection(
            [[corners[index] for index in face] for face in faces],
            facecolors=color,
            edgecolors=color,
            linewidths=0.8,
            alpha=alpha,
        )
    )
    center = corners.mean(axis=0)
    ax.text(*center, label, color=color, fontsize=8)


def world_box(rotation, translation, bounds):
    return box_corners(bounds) @ rotation.T + translation


def draw_frame(ax, origin, rotation, length=0.07):
    colors = ("#d62728", "#2ca02c", "#1f77b4")
    labels = ("X", "Y", "Z")
    for index, (color, label) in enumerate(zip(colors, labels)):
        endpoint = origin + rotation[:, index] * length
        ax.plot(
            [origin[0], endpoint[0]],
            [origin[1], endpoint[1]],
            [origin[2], endpoint[2]],
            color=color,
            linewidth=2,
        )
        ax.text(*endpoint, label, color=color, fontsize=9, weight="bold")


def project_base_points(points, transformer, rotation, translation, image_shape):
    base_flange = np.eye(4)
    base_flange[:3, :3] = rotation
    base_flange[:3, 3] = translation
    base_cam = base_flange @ transformer.T_flange_to_cam
    cam_base = np.linalg.inv(base_cam)
    points = np.asarray(points, dtype=float)
    points_h = np.concatenate([points, np.ones((len(points), 1))], axis=1)
    cam = (cam_base @ points_h.T).T[:, :3]
    z = cam[:, 2]
    pixels = np.stack([
        cam[:, 0] * transformer.fx / np.maximum(z, 1e-6) + transformer.cx,
        cam[:, 1] * transformer.fy / np.maximum(z, 1e-6) + transformer.cy,
    ], axis=1)
    height, width = image_shape[:2]
    valid = (z > 0.02) & (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return pixels.astype(np.int32), valid


def draw_projected_model(image, transformer, rotation, translation, fingertip):
    overlay = image.copy()
    box_colors = ((148, 103, 189), (23, 190, 207), (152, 223, 138), (0, 127, 255), (227, 119, 194))
    box_labels = ("adapter", "adapter-camera", "palm", "thumb", "fingers")
    for bounds, color, label in zip(hand_box_bounds(0.002), box_colors, box_labels):
        world = world_box(rotation, translation, bounds)
        pixels, valid = project_base_points(world, transformer, rotation, translation, overlay.shape)
        if int(valid.sum()) < 4:
            continue
        hull = cv2.convexHull(pixels[valid].reshape(-1, 1, 2))
        cv2.polylines(overlay, [hull], True, color, 2)
        x, y, width, height = cv2.boundingRect(hull)
        cv2.putText(overlay, label, (x, max(16, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    path_pixels, path_valid = project_base_points(
        np.asarray([translation, fingertip]), transformer, rotation, translation, overlay.shape
    )
    if path_valid.all():
        cv2.line(overlay, tuple(path_pixels[0]), tuple(path_pixels[1]), (255, 255, 0), 2)
    return overlay


def main():
    os.environ.setdefault("TOOL_HAND_MODEL", "dh116")
    transformer, rotation, flange = transform_pose(POSE_MM_DEG)
    camera = np.eye(4)
    camera[:3, :3] = rotation @ transformer.T_flange_to_cam[:3, :3]
    camera[:3, 3] = flange + rotation @ transformer.T_flange_to_cam[:3, 3]

    fig = plt.figure(figsize=(16, 9), dpi=120)
    grid = fig.add_gridspec(1, 2, width_ratios=(1.15, 1.0), wspace=0.04)
    ax = fig.add_subplot(grid[0, 0], projection="3d")
    ax.set_facecolor("#f7f8fa")

    box_colors = ("#9467bd", "#17becf", "#ff9896", "#ff7f0e", "#e377c2")
    box_labels = ("adapter", "adapter-camera", "palm", "thumb", "fingers")
    for bounds, color, label in zip(hand_box_bounds(0.002), box_colors, box_labels):
        draw_box(ax, world_box(rotation, flange, bounds), color, label)
    draw_box(ax, world_box(rotation, flange, camera_box_bounds(0.002)), "#1f77b4", "camera", alpha=0.12)

    fingertip = flange + rotation[:, 2] * HAND_LENGTH_Z_M
    ax.plot(
        [flange[0], fingertip[0]],
        [flange[1], fingertip[1]],
        [flange[2], fingertip[2]],
        color="#00a6d6",
        linewidth=3,
        label="hand path",
    )
    ax.scatter(*flange, color="black", s=35, label="flange root")
    ax.scatter(*fingertip, color="#00a6d6", s=45, label="fingertip axis")
    draw_frame(ax, flange, rotation)
    draw_frame(ax, camera[:3, 3], camera[:3, :3], length=0.045)

    all_points = np.vstack([
        world_box(rotation, flange, bounds)
        for bounds in (*hand_box_bounds(0.002), camera_box_bounds(0.002))
    ])
    lower = all_points.min(axis=0) - 0.06
    upper = all_points.max(axis=0) + 0.06
    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])
    ax.set_box_aspect(np.maximum(upper - lower, 0.01))
    ax.set_xlabel("base X (m)")
    ax.set_ylabel("base Y (m)")
    ax.set_zlabel("base Z (m)")
    ax.view_init(elev=22, azim=-58)
    ax.set_title("DH116 collision model in base frame", fontsize=13, weight="bold")
    ax.legend(loc="upper left", fontsize=8)

    image_ax = fig.add_subplot(grid[0, 1])
    if INPUT_IMAGE.exists():
        raw_image = cv2.imread(str(INPUT_IMAGE), cv2.IMREAD_COLOR)
        projected_image = draw_projected_model(raw_image, transformer, rotation, flange, fingertip)
        image_ax.imshow(cv2.cvtColor(projected_image, cv2.COLOR_BGR2RGB))
        image_ax.axis("off")
        image_ax.set_title("Depth-guard + projected model", fontsize=12, weight="bold")
    else:
        image_ax.text(0.5, 0.5, f"Missing {INPUT_IMAGE}", ha="center", va="center")
        image_ax.axis("off")

    pose_text = (
        "LAST LOGGED FLANGE POSE\n"
        f"x y z = {POSE_MM_DEG[0]:.1f}, {POSE_MM_DEG[1]:.1f}, {POSE_MM_DEG[2]:.1f} mm\n"
        f"r p y = {POSE_MM_DEG[3]:.1f}, {POSE_MM_DEG[4]:.1f}, {POSE_MM_DEG[5]:.1f} deg\n"
        "Boxes: 2 adapter + palm + thumb + fingers\n"
        f"hand axis length = {HAND_LENGTH_Z_M * 1000:.0f} mm, clearance = 2 mm"
    )
    fig.text(
        0.03,
        0.02,
        pose_text,
        fontsize=10,
        family="monospace",
        bbox={"facecolor": "white", "edgecolor": "#888", "alpha": 0.95},
    )
    fig.suptitle(
        "Depth-guard model check (pose is last logged, live TCP pose was occupied by worker)",
        fontsize=15,
        weight="bold",
    )
    fig.savefig(OUTPUT_IMAGE, bbox_inches="tight")
    print(OUTPUT_IMAGE)


if __name__ == "__main__":
    main()
