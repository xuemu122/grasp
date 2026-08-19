#!/usr/bin/env python3
"""Target selection and edge-grasp planning."""

import os

import numpy as np


TARGET_ROI_Y_MIN = int(os.getenv("TARGET_ROI_Y_MIN", "85"))
TARGET_ROI_Y_MAX = int(os.getenv("TARGET_ROI_Y_MAX", "430"))
SIDE_GRASP_MIN_HEIGHT_M = float(os.getenv("SIDE_GRASP_MIN_HEIGHT_M", "0.080"))
SIDE_GRASP_MAX_HEIGHT_M = float(os.getenv("SIDE_GRASP_MAX_HEIGHT_M", "0.11"))
SIDE_GRASP_LOW_TARGET_HEIGHT_M = float(os.getenv("SIDE_GRASP_LOW_TARGET_HEIGHT_M", "0.068"))
SIDE_GRASP_LOWER_SHIFT_M = float(os.getenv("SIDE_GRASP_LOWER_SHIFT_M", "0.015"))
LAYER_Z_FILTER_ENABLE = os.getenv("LAYER_Z_FILTER_ENABLE", "1").lower() in ("1", "true", "yes", "on")
LAYER_Z_TOLERANCE_M = float(os.getenv("LAYER_Z_TOLERANCE_M", "0.16"))
SHELF_BOARD_FILTER_ENABLE = os.getenv("SHELF_BOARD_FILTER_ENABLE", "1").lower() in ("1", "true", "yes", "on")
SHELF_BOARD_SAMPLE_STRIDE = int(os.getenv("SHELF_BOARD_SAMPLE_STRIDE", "4"))
SHELF_BOARD_MIN_POINTS = int(os.getenv("SHELF_BOARD_MIN_POINTS", "24"))
SHELF_BOARD_MIN_EDGE_POINTS = int(os.getenv("SHELF_BOARD_MIN_EDGE_POINTS", "12"))
SHELF_BOARD_MIN_X_SPAN_M = float(os.getenv("SHELF_BOARD_MIN_X_SPAN_M", "0.18"))
SHELF_BOARD_MAX_Z_SPREAD_M = float(os.getenv("SHELF_BOARD_MAX_Z_SPREAD_M", "0.045"))
SHELF_BOARD_MIN_DEPTH_EDGE_M = float(os.getenv("SHELF_BOARD_MIN_DEPTH_EDGE_M", "0.035"))
SHELF_BOARD_CLUSTER_Z_M = float(os.getenv("SHELF_BOARD_CLUSTER_Z_M", "0.055"))
SHELF_BOARD_ROW_GAP = int(os.getenv("SHELF_BOARD_ROW_GAP", "40"))
SHELF_BOARD_ROW_EXCLUDE_MARGIN_PX = int(os.getenv("SHELF_BOARD_ROW_EXCLUDE_MARGIN_PX", "6"))
SHELF_BOARD_MIN_ROW_SPAN_PX = int(os.getenv("SHELF_BOARD_MIN_ROW_SPAN_PX", "100"))
SHELF_BOARD_EXPECTED_Z_SPAN_M = float(os.getenv("SHELF_BOARD_EXPECTED_Z_SPAN_M", "0.38"))
SHELF_BOARD_MIN_Z_SPAN_M = float(os.getenv("SHELF_BOARD_MIN_Z_SPAN_M", "0.30"))
SHELF_BOARD_MAX_Z_SPAN_M = float(os.getenv("SHELF_BOARD_MAX_Z_SPAN_M", "0.46"))
SHELF_BOARD_MAX_REFERENCE_DISTANCE_M = float(os.getenv("SHELF_BOARD_MAX_REFERENCE_DISTANCE_M", "0.75"))
SHELF_BOARD_OBJECT_MARGIN_M = float(os.getenv("SHELF_BOARD_OBJECT_MARGIN_M", "0.018"))
SHELF_BOARD_ROW_MARGIN_PX = int(os.getenv("SHELF_BOARD_ROW_MARGIN_PX", "8"))
MAX_GRASP_DEPTH_M = float(os.getenv("MAX_GRASP_DEPTH_M", "0.52"))  # 超过此深度的物品不抓取，避免碰撞
ALLOW_NO_DEPTH_GRASP = os.getenv("ALLOW_NO_DEPTH_GRASP", "0").lower() in ("1", "true", "yes", "on")
# Temporary field policy: prefer side grasps for every product unless explicitly disabled.
FORCE_SIDE_GRASP = os.getenv("FORCE_SIDE_GRASP", "1").lower() in ("1", "true", "yes", "on")
# Use the midpoint of the selected vertical bbox edge.  This preserves the
# side-grasp geometry while avoiding the old height shift toward the shelf.
EDGE_MIDPOINT_GRASP_ENABLE = os.getenv("EDGE_MIDPOINT_GRASP_ENABLE", "0").lower() in ("1", "true", "yes", "on")


def side_grasp_bottom_clearance_m(det, grip_pixel, fy):
    """Return grasp-point height above the detection-box bottom in image space."""
    depth = float(det.depth_m or 0.0)
    fy = float(fy)
    if depth <= 0.0 or fy <= 0.0 or not np.isfinite(depth) or not np.isfinite(fy):
        return None
    _, _, _, y2 = det.bbox
    grip_v = float(grip_pixel[1])
    if not np.isfinite(grip_v):
        return None
    return (float(y2) - grip_v) * depth / fy


def clamp_side_grasp_pixel_to_safe_clearance(det, grip_pixel, fy):
    """Move an out-of-range side-grasp pixel into the box-relative safe band."""
    clearance_m = side_grasp_bottom_clearance_m(det, grip_pixel, fy)
    if clearance_m is None:
        return None, None, None
    if SIDE_GRASP_MIN_HEIGHT_M <= clearance_m <= SIDE_GRASP_MAX_HEIGHT_M:
        return tuple(grip_pixel), clearance_m, clearance_m

    _, y1, _, y2 = [float(value) for value in det.bbox]
    depth = float(det.depth_m)
    real_h = max(0.0, (y2 - y1) * depth / float(fy))
    if real_h < SIDE_GRASP_MIN_HEIGHT_M:
        return None, clearance_m, None
    if clearance_m < SIDE_GRASP_MIN_HEIGHT_M:
        target_m = min(
            max(SIDE_GRASP_LOW_TARGET_HEIGHT_M, SIDE_GRASP_MIN_HEIGHT_M),
            SIDE_GRASP_MAX_HEIGHT_M,
            real_h,
        )
    else:
        target_m = min(SIDE_GRASP_MAX_HEIGHT_M, real_h)
    corrected_v = y2 - target_m * float(fy) / depth
    corrected_v = max(y1, min(y2, corrected_v))
    corrected_pixel = (float(grip_pixel[0]), corrected_v)
    corrected_clearance_m = side_grasp_bottom_clearance_m(det, corrected_pixel, fy)
    return corrected_pixel, clearance_m, corrected_clearance_m


def axis_aligned_approach_m(current_xyz_m, grasp_xyz_m, forward_dir, min_backoff_m):
    """Move to a controlled backoff pose on the grasp axis."""
    grasp = np.array(grasp_xyz_m, dtype=float)
    axis = np.array(forward_dir, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-9)
    return grasp - abs(float(min_backoff_m)) * axis


def _shelf_board_span_is_plausible(first, second):
    span = abs(float(second["z"]) - float(first["z"]))
    return SHELF_BOARD_MIN_Z_SPAN_M <= span <= SHELF_BOARD_MAX_Z_SPAN_M


def _det_center_robot_z(det, transformer, flange_pose):
    if not det.depth_m or det.depth_m <= 0:
        return None
    x1, y1, x2, y2 = det.bbox
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2
    p_cam = transformer.pixel_to_camera(u, v, det.depth_m)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=False)
    return float(p_robot[2])


def detect_shelf_board_bounds(
    depth_img,
    camera,
    transformer,
    flange_pose,
    reference_z_m=None,
    exclude_bboxes=(),
):
    """Estimate the current shelf's upper/lower boards from one depth frame.

    A shelf board is treated as a horizontal, wide surface: sampled points on
    the same image row have a small robot-Z spread and a meaningful robot-X
    span. The result is only used to establish filtering bounds; it never
    creates a motion target by itself.
    """
    if not SHELF_BOARD_FILTER_ENABLE or depth_img is None or transformer is None or flange_pose is None:
        return None
    depth = np.asarray(depth_img)
    if depth.ndim != 2 or depth.size == 0:
        return None

    height, width = depth.shape
    stride = max(1, int(SHELF_BOARD_SAMPLE_STRIDE))
    rows = np.arange(0, height, stride, dtype=np.int32)
    cols = np.arange(0, width, stride, dtype=np.int32)
    grid_x, grid_y = np.meshgrid(cols, rows)
    depth_scale = float(getattr(camera, "_depth_scale", None) or 0.001)
    depth_m = depth[grid_y, grid_x].astype(np.float32) * depth_scale
    valid = np.isfinite(depth_m) & (depth_m >= 0.08) & (depth_m <= 1.5)

    excluded_centers = []
    for bbox in exclude_bboxes or ():
        if bbox is None:
            continue
        x1, y1, x2, y2 = [int(v) for v in bbox]
        excluded_centers.append((y1 + y2) / 2.0)
        margin = SHELF_BOARD_ROW_EXCLUDE_MARGIN_PX
        valid &= ~(
            (grid_x >= max(0, x1 - margin)) & (grid_x <= min(width - 1, x2 + margin)) &
            (grid_y >= max(0, y1 - margin)) & (grid_y <= min(height - 1, y2 + margin))
        )

    if int(np.count_nonzero(valid)) < SHELF_BOARD_MIN_POINTS:
        return None

    u = grid_x.astype(np.float32)
    v = grid_y.astype(np.float32)
    z_cam = depth_m
    x_cam = (u - float(transformer.cx)) * z_cam / float(transformer.fx)
    y_cam = (v - float(transformer.cy)) * z_cam / float(transformer.fy)
    cam_points = np.stack([x_cam, y_cam, z_cam, np.ones_like(z_cam)], axis=-1)
    pose = tuple(float(value) for value in flange_pose)
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = transformer._rpy_to_rot(pose[3], pose[4], pose[5]).astype(np.float32)
    transform[:3, 3] = np.asarray(pose[:3], dtype=np.float32) / 1000.0
    transform = transform @ np.asarray(transformer.T_flange_to_cam, dtype=np.float32)
    robot_points = (cam_points @ transform.T)[..., :3]

    row_candidates = []
    for row_index, row in enumerate(rows):
        keep = valid[row_index]
        valid_count = int(np.count_nonzero(keep))
        if valid_count < SHELF_BOARD_MIN_EDGE_POINTS:
            continue
        points = robot_points[row_index, keep]
        z_values = points[:, 2]
        z_low, z_high = np.percentile(z_values, [10, 90])
        x_span = float(np.percentile(points[:, 0], 95) - np.percentile(points[:, 0], 5))
        flat_surface = (
            valid_count >= SHELF_BOARD_MIN_POINTS and
            float(z_high - z_low) <= SHELF_BOARD_MAX_Z_SPREAD_M and
            x_span >= SHELF_BOARD_MIN_X_SPAN_M
        )
        source = "surface"
        selected_points = points
        if not flat_surface:
            if row_index == 0 or row_index == len(rows) - 1:
                continue
            edge_keep = valid[row_index - 1] & valid[row_index + 1]
            edge_delta = np.abs(depth_m[row_index - 1] - depth_m[row_index + 1])
            edge_keep &= edge_delta >= SHELF_BOARD_MIN_DEPTH_EDGE_M
            if int(np.count_nonzero(edge_keep)) < SHELF_BOARD_MIN_EDGE_POINTS:
                continue
            selected_points = robot_points[row_index, edge_keep]
            edge_x_span = float(
                np.percentile(selected_points[:, 0], 95) -
                np.percentile(selected_points[:, 0], 5)
            )
            if edge_x_span < SHELF_BOARD_MIN_X_SPAN_M:
                continue
            z_values = selected_points[:, 2]
            source = "depth_edge"
        row_candidates.append({
            "row": int(row),
            "z": float(np.median(z_values)),
            "points": len(selected_points),
            "source": source,
        })

    if not row_candidates:
        return None
    if reference_z_m is not None:
        row_candidates = [
            item for item in row_candidates
            if abs(item["z"] - float(reference_z_m)) <= SHELF_BOARD_MAX_REFERENCE_DISTANCE_M
        ]
    if not row_candidates:
        return None

    row_candidates.sort(key=lambda item: item["row"])
    groups = []
    for candidate in row_candidates:
        if (
            groups and
            candidate["row"] - groups[-1][-1]["row"] <= SHELF_BOARD_ROW_GAP and
            abs(candidate["z"] - np.median([item["z"] for item in groups[-1]])) <= SHELF_BOARD_CLUSTER_Z_M
        ):
            groups[-1].append(candidate)
        else:
            groups.append([candidate])

    boards = []
    for group in groups:
        weight = np.asarray([item["points"] for item in group], dtype=np.float32)
        boards.append({
            "z": float(np.average([item["z"] for item in group], weights=weight)),
            "row": int(round(np.average([item["row"] for item in group], weights=weight))),
            "rows": len(group),
            "points": int(weight.sum()),
            "source": "depth_edge" if any(item["source"] == "depth_edge" for item in group) else "surface",
        })
    boards.sort(key=lambda item: item["z"])
    if reference_z_m is None:
        return {"boards": boards}

    reference = float(reference_z_m)
    boards_near_reference = [
        board for board in boards
        if abs(board["z"] - reference) <= SHELF_BOARD_MAX_REFERENCE_DISTANCE_M
    ]
    if not boards_near_reference:
        return {"boards": boards}

    lower = [board for board in boards_near_reference if board["z"] <= reference + SHELF_BOARD_CLUSTER_Z_M]
    upper = [board for board in boards_near_reference if board["z"] > reference + SHELF_BOARD_CLUSTER_Z_M]
    pair_candidates = []
    if excluded_centers:
        reference_row = float(np.median(excluded_centers))
        for first in boards_near_reference:
            for second in boards_near_reference:
                if first["row"] >= reference_row or second["row"] <= reference_row:
                    continue
                row_span = second["row"] - first["row"]
                if row_span < SHELF_BOARD_MIN_ROW_SPAN_PX:
                    continue
                if not _shelf_board_span_is_plausible(first, second):
                    continue
                z_span = abs(second["z"] - first["z"])
                row_span_error = abs((second["row"] - first["row"]) - SHELF_BOARD_MIN_ROW_SPAN_PX)
                score = (abs(z_span - SHELF_BOARD_EXPECTED_Z_SPAN_M), row_span_error)
                pair_candidates.append((score, first, second))
    else:
        by_row = sorted(boards_near_reference, key=lambda board: board["row"])
        for index, first in enumerate(by_row):
            for second in by_row[index + 1:]:
                row_span = second["row"] - first["row"]
                if row_span < SHELF_BOARD_MIN_ROW_SPAN_PX:
                    continue
                if not _shelf_board_span_is_plausible(first, second):
                    continue
                z_mid = (first["z"] + second["z"]) / 2.0
                z_span = abs(second["z"] - first["z"])
                score = (abs(z_span - SHELF_BOARD_EXPECTED_Z_SPAN_M), abs(z_mid - reference))
                pair_candidates.append((score, first, second))

    if pair_candidates:
        _, first, second = min(pair_candidates, key=lambda item: item[0])
        lower, upper = sorted((first, second), key=lambda board: board["z"])
    else:
        if not lower or not upper:
            return {"boards": boards}
        lower = max(lower, key=lambda board: board["z"])
        upper = min(upper, key=lambda board: board["z"])
        if not _shelf_board_span_is_plausible(lower, upper):
            return {"boards": boards}
    z_min = lower["z"] + SHELF_BOARD_OBJECT_MARGIN_M
    z_max = upper["z"] - SHELF_BOARD_OBJECT_MARGIN_M
    row_min = min(lower["row"], upper["row"]) + SHELF_BOARD_ROW_MARGIN_PX
    row_max = max(lower["row"], upper["row"]) - SHELF_BOARD_ROW_MARGIN_PX
    # A noisy frame can produce adjacent or inverted board estimates. Keep
    # the result for diagnostics, but let the caller use the established
    # fallback filters instead of rejecting every detection.
    if (
        z_min >= z_max or
        row_min >= row_max or
        abs(upper["row"] - lower["row"]) < SHELF_BOARD_MIN_ROW_SPAN_PX
    ):
        return {"boards": boards}
    return {
        "boards": boards,
        "lower": lower,
        "upper": upper,
        "z_min": z_min,
        "z_max": z_max,
        "row_min": row_min,
        "row_max": row_max,
    }


def explain_target_filter(
    detections,
    transformer=None,
    flange_pose=None,
    layer_z_m=None,
    shelf_board_bounds=None,
):
    """Return per-detection decisions used by the layer filter.

    This is a read-only diagnostic API for preview tools. It intentionally
    reports the strict row/Z decision without applying pick_target_box's
    no-candidate fallback, so an operator can see exactly which detections
    the dynamic bounds would reject.
    """
    dynamic = bool(shelf_board_bounds and shelf_board_bounds.get("row_min") is not None)
    if dynamic:
        row_min = shelf_board_bounds["row_min"]
        row_max = shelf_board_bounds["row_max"]
        z_min = shelf_board_bounds.get("z_min")
        z_max = shelf_board_bounds.get("z_max")
        mode = "dynamic"
    else:
        row_min = TARGET_ROI_Y_MIN
        row_max = TARGET_ROI_Y_MAX
        z_min = z_max = None
        mode = "fallback_roi"

    records = []
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        center_x = (float(x1) + float(x2)) / 2.0
        center_y = (float(y1) + float(y2)) / 2.0
        row_ok = row_min <= center_y <= row_max
        z = _det_center_robot_z(det, transformer, flange_pose) if transformer is not None and flange_pose is not None else None
        if dynamic and z_min is not None and z_max is not None and z is not None:
            z_ok = z_min <= z <= z_max
            z_reason = "" if z_ok else "z_outside_dynamic_bounds"
        elif not dynamic and LAYER_Z_FILTER_ENABLE and layer_z_m is not None and z is not None:
            z_ok = abs(z - layer_z_m) <= LAYER_Z_TOLERANCE_M
            z_reason = "" if z_ok else "z_outside_layer_tolerance"
        else:
            z_ok = True
            z_reason = ""
        depth = float(det.depth_m) if det.depth_m and det.depth_m > 0 else None
        depth_ok = depth is not None and depth <= MAX_GRASP_DEPTH_M
        reasons = []
        if not row_ok:
            reasons.append("row_outside_bounds")
        if not z_ok:
            reasons.append(z_reason)
        if not depth_ok:
            reasons.append("missing_or_too_deep")
        records.append({
            "class_name": str(getattr(det, "class_name", "") or ""),
            "class_id": getattr(det, "class_id", None),
            "bbox": [int(round(float(v))) for v in det.bbox],
            "center": [round(center_x, 1), round(center_y, 1)],
            "depth_m": depth,
            "robot_z_m": round(z, 5) if z is not None else None,
            "row_ok": bool(row_ok),
            "z_ok": bool(z_ok),
            "depth_ok": bool(depth_ok),
            "accepted": not reasons,
            "reasons": reasons,
        })
    return {
        "mode": mode,
        "row_min": row_min,
        "row_max": row_max,
        "z_min": z_min,
        "z_max": z_max,
        "layer_z_m": layer_z_m,
        "detections": records,
    }


def pick_target_box(detections, transformer=None, flange_pose=None, layer_z_m=None, shelf_board_bounds=None):
    """
    货架分层抓取:
      1. 优先按当前帧识别到的层板上下边界过滤上下层
      2. 识别不到层板时回退到画面 ROI 和世界 Z 容差
      3. 按深度分组 (同层盒子深度差 < 0.15m)
      4. 选最前层 (深度最小)
      5. 在该层里从右往左选目标
    """
    roi_filtered = []
    roi_rejected = []
    if shelf_board_bounds and shelf_board_bounds.get("row_min") is not None:
        row_min = shelf_board_bounds["row_min"]
        row_max = shelf_board_bounds["row_max"]
        for d in detections:
            y_center = (d.bbox[1] + d.bbox[3]) / 2
            if row_min <= y_center <= row_max:
                roi_filtered.append(d)
            else:
                roi_rejected.append(d)
        if roi_rejected:
            print(f"  层板行过滤: 跳过上下层/边缘候选 {len(roi_rejected)} 个, y范围={row_min}-{row_max}")
    else:
        for d in detections:
            y_center = (d.bbox[1] + d.bbox[3]) / 2
            if TARGET_ROI_Y_MIN <= y_center <= TARGET_ROI_Y_MAX:
                roi_filtered.append(d)
            else:
                roi_rejected.append(d)
        if roi_rejected:
            print(f"  ROI过滤(回退): 跳过上下层/边缘候选 {len(roi_rejected)} 个, y范围={TARGET_ROI_Y_MIN}-{TARGET_ROI_Y_MAX}")
    if not roi_filtered:
        print("  [警告] ROI 内没有候选, 回退使用全部检测框")
        roi_filtered = list(detections)

    if (
        shelf_board_bounds and
        shelf_board_bounds.get("z_min") is not None and
        transformer is not None and
        flange_pose is not None
    ):
        z_min = shelf_board_bounds["z_min"]
        z_max = shelf_board_bounds["z_max"]
        z_filtered = []
        z_rejected = []
        for d in roi_filtered:
            z = _det_center_robot_z(d, transformer, flange_pose)
            if z is None or z_min <= z <= z_max:
                z_filtered.append(d)
            else:
                z_rejected.append((d, z, z - layer_z_m if layer_z_m is not None else 0.0))
        if z_rejected:
            print(f"  层板高度过滤: 跳过非当前层候选 {len(z_rejected)} 个 (Z范围 {z_min*1000:.0f}..{z_max*1000:.0f}mm)")
        if not z_filtered:
            # The board rows still identify the current shelf reliably here.
            # A bad depth sample or hand-eye Z drift must not reject every
            # visible item in that row and turn a recoverable pick into a
            # cancelled order.
            print("  [警告] 动态层高过滤拒绝全部行内候选, 回退使用当前层板行范围候选")
        else:
            roi_filtered = z_filtered
    elif LAYER_Z_FILTER_ENABLE and transformer is not None and flange_pose is not None and layer_z_m is not None:
        z_filtered = []
        z_rejected = []
        for d in roi_filtered:
            z = _det_center_robot_z(d, transformer, flange_pose)
            if z is None:
                z_filtered.append(d)
                continue
            dz = z - layer_z_m
            if abs(dz) <= LAYER_Z_TOLERANCE_M:
                z_filtered.append(d)
            else:
                z_rejected.append((d, z, dz))
        if z_rejected:
            details = ", ".join(
                f"{d.class_name or '?'} z={z*1000:.0f}mm Δ={dz*1000:+.0f}mm"
                for d, z, dz in z_rejected[:6]
            )
            print(
                f"  层高过滤: 跳过非当前层候选 {len(z_rejected)} 个 "
                f"(当前层Z≈{layer_z_m*1000:.0f}mm, 容差±{LAYER_Z_TOLERANCE_M*1000:.0f}mm): {details}"
            )
        if not z_filtered:
            print("  [停止] 层高过滤后没有当前层候选, 本次不抓取")
            return None, None
        roi_filtered = z_filtered

    valid = [d for d in roi_filtered if d.depth_m and d.depth_m > 0]
    depth_rejected = [d for d in valid if d.depth_m > MAX_GRASP_DEPTH_M]
    if depth_rejected:
        details = ", ".join(
            f"{d.class_name or '?'} depth={d.depth_m:.3f}m"
            for d in depth_rejected[:6]
        )
        print(
            f"  深度过滤: 跳过过深候选 {len(depth_rejected)} 个 "
            f"(阈值 {MAX_GRASP_DEPTH_M*100:.0f}cm): {details}"
        )
    valid = [d for d in valid if d.depth_m <= MAX_GRASP_DEPTH_M]
    if not valid:
        if ALLOW_NO_DEPTH_GRASP:
            valid = sorted(roi_filtered, key=lambda d: ((d.bbox[0] + d.bbox[2]) / 2), reverse=True)
            if valid:
                print("  [警告] 无有效深度, 已按 ALLOW_NO_DEPTH_GRASP=1 回退选 ROI 内最右侧候选")
                return valid[0], "right"
        print("  [停止] ROI 内没有可信深度候选, 本次不抓取")
        return None, None

    valid.sort(key=lambda d: d.depth_m)
    layers = []
    current_layer = [valid[0]]
    for d in valid[1:]:
        if abs(d.depth_m - current_layer[0].depth_m) < 0.15:
            current_layer.append(d)
        else:
            layers.append(current_layer)
            current_layer = [d]
    layers.append(current_layer)

    front_layer = layers[0]
    print(f"  检测到 {len(layers)} 层货架, 选最前层 ({len(front_layer)} 个盒子, 深度~{front_layer[0].depth_m:.2f}m)")
    for i, layer in enumerate(layers):
        depths = [f"{d.depth_m:.2f}" for d in layer]
        print(f"    第{i+1}层: {len(layer)}个盒子 depth=[{', '.join(depths)}]m")

    target = max(front_layer, key=lambda d: ((d.bbox[0] + d.bbox[2]) / 2))
    tx = (target.bbox[0] + target.bbox[2]) / 2
    ty = (target.bbox[1] + target.bbox[3]) / 2
    print(f"  右侧优先: 选 bbox中心=({tx:.0f},{ty:.0f}) 的目标")
    return target, "right"


def plan_edge_grasp(det, edge, transformer, flange_pose, fx=390.4, camera=None, depth_img=None):
    """
    边缘抓取:
      - 盒宽 <= 8cm: 侧边缘 (x1/x2) + 离底部至少 SIDE_GRASP_MIN_HEIGHT_M 的安全高度
      - 盒宽 > 8cm: 上边缘 (y1) + 中心 X; FORCE_SIDE_GRASP=1 时临时强制侧抓
    返回: (world_x, world_y, world_z, grasp_mode, p_cam)
    """
    x1, y1, x2, y2 = det.bbox
    depth = det.depth_m or 0.5
    fy = getattr(transformer, "fy", fx)

    if EDGE_MIDPOINT_GRASP_ENABLE:
        edge_u = float(x1 if edge == "left" else x2)
        edge_v = (y1 + y2) / 2.0
        mode = f"{edge}_side"
        print(f"  → 检测框{edge}侧边线中点抓取, 像素=({edge_u:.0f},{edge_v:.0f})")
    else:
        edge_u = edge_v = None
        mode = None

    pixel_w = x2 - x1
    pixel_h = y2 - y1
    real_w = pixel_w * depth / fx
    real_h = pixel_h * depth / fy
    print(f"  盒宽: {pixel_w}px → {real_w*100:.1f}cm, 盒高: {pixel_h}px → {real_h*100:.1f}cm")

    if EDGE_MIDPOINT_GRASP_ENABLE:
        pass
    elif FORCE_SIDE_GRASP or real_w <= 0.08:
        edge_u = x1 if edge == "left" else x2
        if real_h <= SIDE_GRASP_MIN_HEIGHT_M:
            edge_u = (x1 + x2) / 2
            edge_v = y1
            mode = "top_center"
            print(
                f"  → 检测框高度 {real_h*100:.1f}cm ≤ 侧抓下限 "
                f"{SIDE_GRASP_MIN_HEIGHT_M*100:.1f}cm, 改为上面抓取"
            )
        else:
            original_height_m = real_h / 2.0
            side_height_m = float(np.clip(
                original_height_m - SIDE_GRASP_LOWER_SHIFT_M,
                max(SIDE_GRASP_MIN_HEIGHT_M, SIDE_GRASP_LOW_TARGET_HEIGHT_M),
                SIDE_GRASP_MAX_HEIGHT_M,
            ))
            pixel_ratio = side_height_m / real_h
            pixel_ratio = max(0.0, min(1.0, pixel_ratio))
            edge_v = y2 + pixel_ratio * (y1 - y2)
            edge_v = max(y1, min(y2, edge_v))
            mode = f"{edge}_side"
            measured_clearance_m = side_grasp_bottom_clearance_m(det, (edge_u, edge_v), fy)
            clearance_label = (
                "无效"
                if measured_clearance_m is None
                else f"{measured_clearance_m*100:.1f}cm"
            )
            width_label = "强制侧抓" if FORCE_SIDE_GRASP and real_w > 0.08 else "窄盒"
            print(
                f"  → {width_label} ({real_w*100:.1f}cm), 从{edge}侧边缘抓取, "
                f"原高度 {original_height_m*100:.1f}cm, 下移 {SIDE_GRASP_LOWER_SHIFT_M*100:.1f}cm, "
                f"抓点距检测框底边 {clearance_label}, 像素=({edge_u:.0f},{edge_v:.0f})"
            )
    else:
        edge_u = (x1 + x2) / 2
        edge_v = y1
        mode = "top_center"
        print(f"  → 宽盒 ({real_w*100:.1f}cm > 8cm), 从上边缘中心抓取")

    grasp_depth = depth
    # 只在非侧抓模式下重采样抓取点深度
    # 侧抓点在 bbox 边缘(x1/x2), get_depth_at 的 radius 区域会混入背景,
    # 中位数被拉深。bbox 中心深度与侧抓点在同一前表面, 直接复用。
    if "side" not in mode and camera is not None and depth_img is not None:
        sampled_depth = camera.get_depth_at(depth_img, edge_u, edge_v, radius=5)
        if sampled_depth and sampled_depth > 0:
            grasp_depth = sampled_depth
            if abs(grasp_depth - depth) > 0.015:
                print(f"  抓取点深度修正: bbox中心 {depth:.3f}m → 抓取点 {grasp_depth:.3f}m")

    p_cam = transformer.pixel_to_camera(edge_u, edge_v, grasp_depth)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=True)
    return p_robot[0], p_robot[1], p_robot[2], mode, p_cam, (edge_u, edge_v)
