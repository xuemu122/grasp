#!/usr/bin/env python3
"""Render DH116 tool geometry in the flange-local top and side views."""

from pathlib import Path
import os

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from tool_geometry import camera_box_bounds, hand_box_bounds


_REPO_ROOT = Path(os.environ["GRASP_ROOT"]) if os.environ.get("GRASP_ROOT") else Path(__file__).resolve().parents[1]
OUTPUT_IMAGE = _REPO_ROOT / "depth_guard_model_local_views.png"


def bounds_2d(bounds, axis_a, axis_b):
    lo, hi = bounds
    x0, x1 = float(lo[axis_a]), float(hi[axis_a])
    y0, y1 = float(lo[axis_b]), float(hi[axis_b])
    return x0, y0, x1 - x0, y1 - y0


def add_boxes(ax, boxes, axis_a, axis_b, colors, labels, annotate=True):
    for bounds, color, label in zip(boxes, colors, labels):
        x, y, width, height = bounds_2d(bounds, axis_a, axis_b)
        ax.add_patch(Rectangle(
            (x * 1000, y * 1000),
            width * 1000,
            height * 1000,
            facecolor=color,
            edgecolor=color,
            alpha=0.25,
            linewidth=2,
            label=label,
        ))
        if annotate:
            ax.text(x * 1000 + width * 500, y * 1000 + height * 500, label,
                    ha="center", va="center", fontsize=9, color=color, weight="bold")


def format_view(ax, title, xlabel, ylabel, xlim, ylim):
    ax.axhline(0, color="#555", linewidth=0.8)
    ax.axvline(0, color="#555", linewidth=0.8)
    ax.set_title(title, fontsize=14, weight="bold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")


def main():
    boxes = hand_box_bounds(0.002)
    camera = camera_box_bounds(0.002)
    colors = ("#9467bd", "#17becf", "#ff9896", "#ff7f0e", "#e377c2")
    labels = ("adapter flange", "adapter camera plate", "palm", "thumb", "fingers")

    fig, axes = plt.subplots(1, 3, figsize=(17, 7), dpi=140)
    fig.suptitle(
        "DH116 collision geometry in flange-local coordinates\n"
        "+Z = hand/finger direction, camera body is above the adapter plate",
        fontsize=16,
        weight="bold",
    )

    # Top view: local X/Y, looking down +Z.
    add_boxes(axes[0], boxes, 0, 1, colors, labels, annotate=False)
    x, y, width, height = bounds_2d(camera, 0, 1)
    axes[0].add_patch(Rectangle((x * 1000, y * 1000), width * 1000, height * 1000,
                                facecolor="#1f77b4", edgecolor="#1f77b4", alpha=0.18,
                                linewidth=2, label="camera body"))
    axes[0].text((x + width / 2) * 1000, (y + height / 2) * 1000, "camera", color="#1f77b4",
                 ha="center", va="center", fontsize=9, weight="bold")
    axes[0].scatter([0], [0], s=70, facecolors="none", edgecolors="#00a6d6", linewidths=2)
    axes[0].text(8, 15, "+Z out of page\n(hand direction)", color="#008cab", fontsize=9)
    axes[0].legend(loc="lower left", fontsize=8, framealpha=0.9)
    format_view(axes[0], "Top view: local X-Y", "local X (mm)", "local Y (mm)",
                (-210, 110), (-90, 270))

    # Side view: local X/Z, looking along +Y.
    add_boxes(axes[1], boxes, 0, 2, colors, labels)
    x, y, width, height = bounds_2d(camera, 0, 2)
    axes[1].add_patch(Rectangle((x * 1000, y * 1000), width * 1000, height * 1000,
                                facecolor="#1f77b4", edgecolor="#1f77b4", alpha=0.18,
                                linewidth=2, label="camera body"))
    axes[1].text((x + width / 2) * 1000, (y + height / 2) * 1000, "camera", color="#1f77b4",
                 ha="center", va="center", fontsize=9, weight="bold")
    axes[1].arrow(0, 55, 0, 170, color="#00a6d6", width=1.5, head_width=8, length_includes_head=True)
    axes[1].text(8, 185, "+Z\n(hand direction)", color="#008cab", fontsize=9)
    axes[1].annotate("camera/plate above adapter\nZ=20..45.5 mm", xy=(-120, 32), xytext=(-235, 115),
                     arrowprops={"arrowstyle": "->", "color": "#1f77b4"}, color="#1f77b4", fontsize=9)
    format_view(axes[1], "Side view: local X-Z", "local X (mm)", "local Z (mm)",
                (-260, 110), (-10, 260))

    # Front view: local Y/Z, showing the lateral hand opening.
    add_boxes(axes[2], boxes, 1, 2, colors, labels)
    format_view(axes[2], "Front view: local Y-Z", "local Y (mm)", "local Z (mm)",
                (-100, 100), (-10, 260))

    fig.text(
        0.5,
        0.015,
        "The boxes are collision envelopes, not a photorealistic CAD mesh.\n"
        "The hand path is +local Z; camera/adapter are offset toward local -X and raised in local Z.",
        ha="center",
        fontsize=10,
        family="monospace",
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.9))
    fig.savefig(OUTPUT_IMAGE, bbox_inches="tight")
    print(OUTPUT_IMAGE)


if __name__ == "__main__":
    main()
