#!/usr/bin/env python3
"""Shared flange-frame geometry for the Aero Hand and wrist camera."""

import math
import os


CAMERA_LENGTH_X_M = float(os.getenv("TOOL_CAMERA_LENGTH_X_M", "0.145"))
CAMERA_WIDTH_Y_M = float(os.getenv("TOOL_CAMERA_WIDTH_Y_M", "0.060"))
CAMERA_HEIGHT_Z_M = float(os.getenv("TOOL_CAMERA_HEIGHT_Z_M", "0.025"))
CAMERA_Z_MIN_M = float(os.getenv("TOOL_CAMERA_Z_MIN_M", "0.0"))
HAND_LENGTH_Z_M = float(os.getenv("TOOL_HAND_LENGTH_Z_M", "0.250"))
HAND_PALM_LENGTH_Z_M = float(os.getenv("HAND_PALM_LENGTH_M", "0.157"))
HAND_BODY_DEPTH_X_M = float(os.getenv("TOOL_HAND_BODY_DEPTH_X_M", "0.080"))
HAND_PALM_WIDTH_Y_M = float(os.getenv("TOOL_HAND_PALM_WIDTH_Y_M", "0.095"))
HAND_OPEN_WIDTH_Y_M = float(os.getenv("TOOL_HAND_OPEN_WIDTH_Y_M", "0.150"))
HAND_MOUNT_EXCLUSION_Z_M = float(os.getenv("TOOL_HAND_MOUNT_EXCLUSION_Z_M", "0.050"))


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


def _box_capsule_grid(size_x_m, size_y_m, z_start_m, z_end_m, x_count, y_count):
    """Cover a flange-frame box with Z-axis capsules."""
    x_count = max(1, int(x_count))
    y_count = max(1, int(y_count))
    cell_x = float(size_x_m) / x_count
    cell_y = float(size_y_m) / y_count
    radius = math.hypot(cell_x * 0.5, cell_y * 0.5)
    x_min = -float(size_x_m) * 0.5
    y_min = -float(size_y_m) * 0.5
    return tuple(
        (
            (x_min + (x_index + 0.5) * cell_x, y_min + (y_index + 0.5) * cell_y, float(z_start_m)),
            (x_min + (x_index + 0.5) * cell_x, y_min + (y_index + 0.5) * cell_y, float(z_end_m)),
            radius,
        )
        for x_index in range(x_count)
        for y_index in range(y_count)
    )


def hand_capsule_parts():
    """Return palm and open-finger envelope capsules in the Link6 frame."""
    palm_end = max(HAND_MOUNT_EXCLUSION_Z_M, min(HAND_PALM_LENGTH_Z_M, HAND_LENGTH_Z_M))
    palm = _box_capsule_grid(
        HAND_BODY_DEPTH_X_M,
        HAND_PALM_WIDTH_Y_M,
        HAND_MOUNT_EXCLUSION_Z_M,
        palm_end,
        3,
        4,
    )
    fingers = _box_capsule_grid(
        HAND_BODY_DEPTH_X_M,
        HAND_OPEN_WIDTH_Y_M,
        max(HAND_MOUNT_EXCLUSION_Z_M, palm_end - 0.025),
        HAND_LENGTH_Z_M,
        3,
        5,
    )
    return {"tool_hand_palm": palm, "tool_hand_fingers": fingers}


def camera_size_label_mm():
    return (
        f"{CAMERA_LENGTH_X_M*1000:.0f}x"
        f"{CAMERA_WIDTH_Y_M*1000:.0f}x"
        f"{CAMERA_HEIGHT_Z_M*1000:.0f}mm"
    )


def tool_size_label_mm():
    return (
        f"hand={HAND_LENGTH_Z_M*1000:.0f}mm/"
        f"palm={HAND_BODY_DEPTH_X_M*1000:.0f}x{HAND_PALM_WIDTH_Y_M*1000:.0f}mm/"
        f"open={HAND_OPEN_WIDTH_Y_M*1000:.0f}mm, camera={camera_size_label_mm()}"
    )
