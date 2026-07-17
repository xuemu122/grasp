#!/usr/bin/env python3
"""Shared flange-frame geometry for the Aero Hand and wrist camera."""

import math
import os


CAMERA_LENGTH_X_M = float(os.getenv("TOOL_CAMERA_LENGTH_X_M", "0.145"))
CAMERA_WIDTH_Y_M = float(os.getenv("TOOL_CAMERA_WIDTH_Y_M", "0.060"))
CAMERA_HEIGHT_Z_M = float(os.getenv("TOOL_CAMERA_HEIGHT_Z_M", "0.025"))
CAMERA_Z_MIN_M = float(os.getenv("TOOL_CAMERA_Z_MIN_M", "0.0"))


def camera_box_bounds(clearance_m=0.0):
    """Return local AABB bounds; camera runs from the flange toward local -X."""
    clearance = max(0.0, float(clearance_m))
    return (
        (
            -CAMERA_LENGTH_X_M - clearance,
            -CAMERA_WIDTH_Y_M * 0.5 - clearance,
            CAMERA_Z_MIN_M - clearance,
        ),
        (
            clearance,
            CAMERA_WIDTH_Y_M * 0.5 + clearance,
            CAMERA_Z_MIN_M + CAMERA_HEIGHT_Z_M + clearance,
        ),
    )


def camera_capsule_strips(count=4):
    """Cover the camera box with thin X-axis capsules for self-collision checks."""
    count = max(1, int(count))
    strip_width = CAMERA_WIDTH_Y_M / count
    radius = math.hypot(strip_width * 0.5, CAMERA_HEIGHT_Z_M * 0.5)
    z_center = CAMERA_Z_MIN_M + CAMERA_HEIGHT_Z_M * 0.5
    y_min = -CAMERA_WIDTH_Y_M * 0.5
    strips = []
    for index in range(count):
        y_center = y_min + (index + 0.5) * strip_width
        strips.append((
            (-CAMERA_LENGTH_X_M, y_center, z_center),
            (0.0, y_center, z_center),
            radius,
        ))
    return tuple(strips)


def camera_size_label_mm():
    return (
        f"{CAMERA_LENGTH_X_M*1000:.0f}x"
        f"{CAMERA_WIDTH_Y_M*1000:.0f}x"
        f"{CAMERA_HEIGHT_Z_M*1000:.0f}mm"
    )
