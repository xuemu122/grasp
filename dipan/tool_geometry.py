#!/usr/bin/env python3
"""Shared flange-frame geometry for the DH116 hand and wrist camera.

The DH116 defaults below combine the hand URDF swept envelope with the printed
adapter from ``urdf/new.3mf``.  The round adapter face starts at the Nova tool
flange, its camera plate points along flange -X, and the hand starts after the
adapter along flange +Z.  Mount dimensions remain overrideable.
"""

import math
import os


# The camera's 145 mm side runs laterally across the wrist.  Its optical
# center is about 120 mm along flange -X, so the 60 mm body depth is centered
# there instead of stretching the whole 145 mm from the flange origin.
CAMERA_LONG_SIDE_Y_M = float(os.getenv(
    "TOOL_CAMERA_LONG_SIDE_Y_M",
    os.getenv("TOOL_CAMERA_LENGTH_X_M", "0.145"),
))
CAMERA_BODY_DEPTH_X_M = float(os.getenv(
    "TOOL_CAMERA_BODY_DEPTH_X_M",
    os.getenv("TOOL_CAMERA_WIDTH_Y_M", "0.060"),
))
CAMERA_HEIGHT_Z_M = float(os.getenv("TOOL_CAMERA_HEIGHT_Z_M", "0.025"))
CAMERA_CENTER_X_M = float(os.getenv("TOOL_CAMERA_CENTER_X_M", "-0.120"))
CAMERA_CENTER_Y_M = float(os.getenv("TOOL_CAMERA_CENTER_Y_M", "0.0"))
# The new adapter's raised camera plate is at Z=20 mm; the camera body sits on
# that plate and extends toward +Z. Keep this overrideable for another camera.
CAMERA_Z_MIN_M = float(os.getenv("TOOL_CAMERA_Z_MIN_M", "0.020"))
# Printed adapter bounds after applying the 3MF component transform:
#   all:          X=-150.5..33.0, Y=-33.5..33.5, Z=0..45.5 mm
#   round flange: X=-33.5..33.0,  Y=-33.5..33.5, Z=0..45.5 mm
#   camera plate: X=-150.5..-33.5,Y=-30.4..29.6, Z=0..20.0 mm
ADAPTER_LENGTH_Z_M = float(os.getenv("TOOL_ADAPTER_LENGTH_Z_M", "0.0455"))
ADAPTER_FLANGE_X_MIN_M = float(os.getenv("TOOL_ADAPTER_FLANGE_X_MIN_M", "-0.0335"))
ADAPTER_FLANGE_X_MAX_M = float(os.getenv("TOOL_ADAPTER_FLANGE_X_MAX_M", "0.0330"))
ADAPTER_FLANGE_Y_MIN_M = float(os.getenv("TOOL_ADAPTER_FLANGE_Y_MIN_M", "-0.0335"))
ADAPTER_FLANGE_Y_MAX_M = float(os.getenv("TOOL_ADAPTER_FLANGE_Y_MAX_M", "0.0335"))
ADAPTER_CAMERA_X_MIN_M = float(os.getenv("TOOL_ADAPTER_CAMERA_X_MIN_M", "-0.1505"))
ADAPTER_CAMERA_X_MAX_M = float(os.getenv("TOOL_ADAPTER_CAMERA_X_MAX_M", "-0.0335"))
ADAPTER_CAMERA_Y_MIN_M = float(os.getenv("TOOL_ADAPTER_CAMERA_Y_MIN_M", "-0.0305"))
ADAPTER_CAMERA_Y_MAX_M = float(os.getenv("TOOL_ADAPTER_CAMERA_Y_MAX_M", "0.0296"))
ADAPTER_CAMERA_Z_MAX_M = float(os.getenv("TOOL_ADAPTER_CAMERA_Z_MAX_M", "0.0200"))

# DH116 swept AABB from the supplied URDF, relative to the hand base:
#   all links: X=-33.9..112.8, Y=-49.6..57.9, Z=0..192.4 mm
#   base:      X=-33.9..28.4,  Y=-41.8..41.6, Z=0..108.4 mm
#   thumb:     X=7.6..112.8,   Y=-49.6..57.9, Z=22.6..119.0 mm
#   fingers:   X=-25.1..70.6,  Y=-45.1..43.8, Z=52.0..192.4 mm
HAND_MOUNT_OFFSET_Z_M = float(os.getenv(
    "TOOL_HAND_MOUNT_OFFSET_Z_M", str(ADAPTER_LENGTH_Z_M)
))
HAND_LENGTH_Z_M = float(os.getenv(
    "TOOL_HAND_LENGTH_Z_M", str(HAND_MOUNT_OFFSET_Z_M + 0.195)
))
HAND_PALM_LENGTH_Z_M = float(os.getenv(
    "HAND_PALM_LENGTH_M", str(HAND_MOUNT_OFFSET_Z_M + 0.110)
))
HAND_BODY_DEPTH_X_M = float(os.getenv("TOOL_HAND_BODY_DEPTH_X_M", "0.062"))
HAND_PALM_WIDTH_Y_M = float(os.getenv("TOOL_HAND_PALM_WIDTH_Y_M", "0.086"))
HAND_OPEN_WIDTH_Y_M = float(os.getenv("TOOL_HAND_OPEN_WIDTH_Y_M", "0.112"))
HAND_MOUNT_EXCLUSION_Z_M = float(os.getenv(
    "TOOL_HAND_MOUNT_EXCLUSION_Z_M", str(HAND_MOUNT_OFFSET_Z_M)
))

HAND_PALM_X_MIN_M = float(os.getenv("TOOL_HAND_PALM_X_MIN_M", "-0.036"))
HAND_PALM_X_MAX_M = float(os.getenv("TOOL_HAND_PALM_X_MAX_M", "0.030"))
HAND_PALM_Y_MIN_M = float(os.getenv("TOOL_HAND_PALM_Y_MIN_M", "-0.044"))
HAND_PALM_Y_MAX_M = float(os.getenv("TOOL_HAND_PALM_Y_MAX_M", "0.044"))
HAND_THUMB_X_MIN_M = float(os.getenv("TOOL_HAND_THUMB_X_MIN_M", "0.006"))
HAND_THUMB_X_MAX_M = float(os.getenv("TOOL_HAND_THUMB_X_MAX_M", "0.115"))
HAND_THUMB_Y_MIN_M = float(os.getenv("TOOL_HAND_THUMB_Y_MIN_M", "-0.052"))
HAND_THUMB_Y_MAX_M = float(os.getenv("TOOL_HAND_THUMB_Y_MAX_M", "0.060"))
HAND_FINGERS_X_MIN_M = float(os.getenv("TOOL_HAND_FINGERS_X_MIN_M", "-0.027"))
HAND_FINGERS_X_MAX_M = float(os.getenv("TOOL_HAND_FINGERS_X_MAX_M", "0.073"))
HAND_FINGERS_Y_MIN_M = float(os.getenv("TOOL_HAND_FINGERS_Y_MIN_M", "-0.047"))
HAND_FINGERS_Y_MAX_M = float(os.getenv("TOOL_HAND_FINGERS_Y_MAX_M", "0.046"))
HAND_THUMB_Z_MAX_M = float(os.getenv(
    "TOOL_HAND_THUMB_Z_MAX_M", str(HAND_MOUNT_OFFSET_Z_M + 0.121)
))


def hand_box_bounds(clearance_m=0.0):
    """Return adapter and DH116 swept AABBs in the Nova flange frame."""
    clearance = max(0.0, float(clearance_m))
    boxes = (
        (
            (ADAPTER_FLANGE_X_MIN_M, ADAPTER_FLANGE_Y_MIN_M, 0.0),
            (ADAPTER_FLANGE_X_MAX_M, ADAPTER_FLANGE_Y_MAX_M, ADAPTER_LENGTH_Z_M),
        ),
        (
            (ADAPTER_CAMERA_X_MIN_M, ADAPTER_CAMERA_Y_MIN_M, 0.0),
            (ADAPTER_CAMERA_X_MAX_M, ADAPTER_CAMERA_Y_MAX_M, ADAPTER_CAMERA_Z_MAX_M),
        ),
        (
            (HAND_PALM_X_MIN_M, HAND_PALM_Y_MIN_M, HAND_MOUNT_EXCLUSION_Z_M),
            (HAND_PALM_X_MAX_M, HAND_PALM_Y_MAX_M, min(HAND_PALM_LENGTH_Z_M, HAND_LENGTH_Z_M)),
        ),
        (
            (
                HAND_THUMB_X_MIN_M,
                HAND_THUMB_Y_MIN_M,
                max(HAND_MOUNT_EXCLUSION_Z_M, HAND_MOUNT_OFFSET_Z_M + 0.020),
            ),
            (HAND_THUMB_X_MAX_M, HAND_THUMB_Y_MAX_M, min(HAND_THUMB_Z_MAX_M, HAND_LENGTH_Z_M)),
        ),
        (
            (
                HAND_FINGERS_X_MIN_M,
                HAND_FINGERS_Y_MIN_M,
                max(HAND_MOUNT_EXCLUSION_Z_M, HAND_MOUNT_OFFSET_Z_M + 0.050),
            ),
            (HAND_FINGERS_X_MAX_M, HAND_FINGERS_Y_MAX_M, HAND_LENGTH_Z_M),
        ),
    )
    return tuple(
        (
            tuple(float(value) - clearance for value in box_min),
            tuple(float(value) + clearance for value in box_max),
        )
        for box_min, box_max in boxes
    )


def camera_box_bounds(clearance_m=0.0):
    """Return the wrist-camera AABB in the flange frame."""
    clearance = max(0.0, float(clearance_m))
    half_depth = CAMERA_BODY_DEPTH_X_M * 0.5
    half_length = CAMERA_LONG_SIDE_Y_M * 0.5
    return (
        (
            CAMERA_CENTER_X_M - half_depth - clearance,
            CAMERA_CENTER_Y_M - half_length - clearance,
            CAMERA_Z_MIN_M - clearance,
        ),
        (
            CAMERA_CENTER_X_M + half_depth + clearance,
            CAMERA_CENTER_Y_M + half_length + clearance,
            CAMERA_Z_MIN_M + CAMERA_HEIGHT_Z_M + clearance,
        ),
    )


def camera_capsule_strips(count=4):
    """Cover the root-mounted camera box with thin X-axis capsules."""
    count = max(1, int(count))
    strip_width = CAMERA_LONG_SIDE_Y_M / count
    radius = math.hypot(strip_width * 0.5, CAMERA_HEIGHT_Z_M * 0.5)
    z_center = CAMERA_Z_MIN_M + CAMERA_HEIGHT_Z_M * 0.5
    x_min = CAMERA_CENTER_X_M - CAMERA_BODY_DEPTH_X_M * 0.5
    x_max = CAMERA_CENTER_X_M + CAMERA_BODY_DEPTH_X_M * 0.5
    y_min = CAMERA_CENTER_Y_M - CAMERA_LONG_SIDE_Y_M * 0.5
    strips = []
    for index in range(count):
        y_center = y_min + (index + 0.5) * strip_width
        strips.append((
            (x_min, y_center, z_center),
            (x_max, y_center, z_center),
            radius,
        ))
    return tuple(strips)


def adapter_camera_mount_capsules(count=5):
    """Cover the adapter's -X camera plate with thin X-axis capsules."""
    count = max(1, int(count))
    strip_width = (ADAPTER_CAMERA_Y_MAX_M - ADAPTER_CAMERA_Y_MIN_M) / count
    radius = math.hypot(strip_width * 0.5, ADAPTER_CAMERA_Z_MAX_M * 0.5)
    z_center = ADAPTER_CAMERA_Z_MAX_M * 0.5
    return tuple(
        (
            (
                ADAPTER_CAMERA_X_MIN_M,
                ADAPTER_CAMERA_Y_MIN_M + (index + 0.5) * strip_width,
                z_center,
            ),
            (
                ADAPTER_CAMERA_X_MAX_M,
                ADAPTER_CAMERA_Y_MIN_M + (index + 0.5) * strip_width,
                z_center,
            ),
            radius,
        )
        for index in range(count)
    )


def _box_capsule_grid(
    size_x_m,
    size_y_m,
    z_start_m,
    z_end_m,
    x_count,
    y_count,
    *,
    x_min_m=None,
    y_min_m=None,
):
    """Cover a flange-frame box with Z-axis capsules."""
    x_count = max(1, int(x_count))
    y_count = max(1, int(y_count))
    cell_x = float(size_x_m) / x_count
    cell_y = float(size_y_m) / y_count
    radius = math.hypot(cell_x * 0.5, cell_y * 0.5)
    x_min = -float(size_x_m) * 0.5 if x_min_m is None else float(x_min_m)
    y_min = -float(size_y_m) * 0.5 if y_min_m is None else float(y_min_m)
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
    """Return adapter and DH116 swept-envelope capsules in the Link6 frame."""
    adapter_flange = _box_capsule_grid(
        ADAPTER_FLANGE_X_MAX_M - ADAPTER_FLANGE_X_MIN_M,
        ADAPTER_FLANGE_Y_MAX_M - ADAPTER_FLANGE_Y_MIN_M,
        0.0,
        ADAPTER_LENGTH_Z_M,
        6,
        6,
        x_min_m=ADAPTER_FLANGE_X_MIN_M,
        y_min_m=ADAPTER_FLANGE_Y_MIN_M,
    )
    palm_end = max(HAND_MOUNT_EXCLUSION_Z_M, min(HAND_PALM_LENGTH_Z_M, HAND_LENGTH_Z_M))
    palm = _box_capsule_grid(
        HAND_PALM_X_MAX_M - HAND_PALM_X_MIN_M,
        HAND_PALM_Y_MAX_M - HAND_PALM_Y_MIN_M,
        HAND_MOUNT_EXCLUSION_Z_M,
        palm_end,
        4,
        6,
        x_min_m=HAND_PALM_X_MIN_M,
        y_min_m=HAND_PALM_Y_MIN_M,
    )
    thumb_start = max(HAND_MOUNT_EXCLUSION_Z_M, HAND_MOUNT_OFFSET_Z_M + 0.020)
    thumb = _box_capsule_grid(
        HAND_THUMB_X_MAX_M - HAND_THUMB_X_MIN_M,
        HAND_THUMB_Y_MAX_M - HAND_THUMB_Y_MIN_M,
        thumb_start,
        min(HAND_LENGTH_Z_M, HAND_THUMB_Z_MAX_M),
        3,
        4,
        x_min_m=HAND_THUMB_X_MIN_M,
        y_min_m=HAND_THUMB_Y_MIN_M,
    )
    fingers = _box_capsule_grid(
        HAND_FINGERS_X_MAX_M - HAND_FINGERS_X_MIN_M,
        HAND_FINGERS_Y_MAX_M - HAND_FINGERS_Y_MIN_M,
        max(HAND_MOUNT_EXCLUSION_Z_M, HAND_MOUNT_OFFSET_Z_M + 0.050),
        HAND_LENGTH_Z_M,
        3,
        5,
        x_min_m=HAND_FINGERS_X_MIN_M,
        y_min_m=HAND_FINGERS_Y_MIN_M,
    )
    return {
        "tool_adapter_flange": adapter_flange,
        "tool_adapter_camera_mount": adapter_camera_mount_capsules(),
        "tool_hand_palm": palm,
        "tool_hand_thumb": thumb,
        "tool_hand_fingers": fingers,
    }


def camera_size_label_mm():
    return (
        f"{CAMERA_LONG_SIDE_Y_M*1000:.0f}x"
        f"{CAMERA_BODY_DEPTH_X_M*1000:.0f}x"
        f"{CAMERA_HEIGHT_Z_M*1000:.0f}mm"
    )


def tool_size_label_mm():
    return (
        f"hand={HAND_LENGTH_Z_M*1000:.0f}mm/adapter={ADAPTER_LENGTH_Z_M*1000:.0f}mm/"
        f"palm={HAND_BODY_DEPTH_X_M*1000:.0f}x{HAND_PALM_WIDTH_Y_M*1000:.0f}mm/"
        f"open={HAND_OPEN_WIDTH_Y_M*1000:.0f}mm, camera={camera_size_label_mm()}"
    )
