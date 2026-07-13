#!/usr/bin/env python3
"""Target selection and edge-grasp planning."""

import os

import numpy as np


TARGET_ROI_Y_MIN = int(os.getenv("TARGET_ROI_Y_MIN", "85"))
TARGET_ROI_Y_MAX = int(os.getenv("TARGET_ROI_Y_MAX", "430"))
SIDE_GRASP_MIN_HEIGHT_M = float(os.getenv("SIDE_GRASP_MIN_HEIGHT_M", "0.052"))
SIDE_GRASP_MAX_HEIGHT_M = float(os.getenv("SIDE_GRASP_MAX_HEIGHT_M", "0.11"))  # 侧抓点离底部最大高度，防止磕相机
LAYER_Z_FILTER_ENABLE = os.getenv("LAYER_Z_FILTER_ENABLE", "1").lower() in ("1", "true", "yes", "on")
LAYER_Z_TOLERANCE_M = float(os.getenv("LAYER_Z_TOLERANCE_M", "0.16"))
MAX_GRASP_DEPTH_M = float(os.getenv("MAX_GRASP_DEPTH_M", "0.52"))  # 超过此深度的物品不抓取，避免碰撞
ALLOW_NO_DEPTH_GRASP = os.getenv("ALLOW_NO_DEPTH_GRASP", "0").lower() in ("1", "true", "yes", "on")
FORCE_SIDE_GRASP = os.getenv("FORCE_SIDE_GRASP", "0").lower() in ("1", "true", "yes", "on")


def axis_aligned_approach_m(current_xyz_m, grasp_xyz_m, forward_dir, min_backoff_m):
    """Move to a controlled backoff pose on the grasp axis."""
    grasp = np.array(grasp_xyz_m, dtype=float)
    axis = np.array(forward_dir, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-9)
    return grasp - abs(float(min_backoff_m)) * axis


def _det_center_robot_z(det, transformer, flange_pose):
    if not det.depth_m or det.depth_m <= 0:
        return None
    x1, y1, x2, y2 = det.bbox
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2
    p_cam = transformer.pixel_to_camera(u, v, det.depth_m)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=False)
    return float(p_robot[2])


def pick_target_box(detections, transformer=None, flange_pose=None, layer_z_m=None):
    """
    货架分层抓取:
      1. 先按画面纵向 ROI 过滤上下层
      2. 按当前货架层世界 Z 过滤上下层
      3. 按深度分组 (同层盒子深度差 < 0.15m)
      4. 选最前层 (深度最小)
      5. 在该层里从右往左选目标
    """
    roi_filtered = []
    roi_rejected = []
    for d in detections:
        y_center = (d.bbox[1] + d.bbox[3]) / 2
        if TARGET_ROI_Y_MIN <= y_center <= TARGET_ROI_Y_MAX:
            roi_filtered.append(d)
        else:
            roi_rejected.append(d)
    if roi_rejected:
        print(f"  ROI过滤: 跳过上下层/边缘候选 {len(roi_rejected)} 个, y范围={TARGET_ROI_Y_MIN}-{TARGET_ROI_Y_MAX}")
    if not roi_filtered:
        print("  [警告] ROI 内没有候选, 回退使用全部检测框")
        roi_filtered = list(detections)

    if LAYER_Z_FILTER_ENABLE and transformer is not None and flange_pose is not None and layer_z_m is not None:
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

    pixel_w = x2 - x1
    pixel_h = y2 - y1
    real_w = pixel_w * depth / fx
    real_h = pixel_h * depth / fy
    print(f"  盒宽: {pixel_w}px → {real_w*100:.1f}cm, 盒高: {pixel_h}px → {real_h*100:.1f}cm")

    if FORCE_SIDE_GRASP or real_w <= 0.08:
        edge_u = x1 if edge == "left" else x2
        p_bottom_cam = transformer.pixel_to_camera(edge_u, y2, depth)
        p_top_cam = transformer.pixel_to_camera(edge_u, y1, depth)
        p_bottom_robot = transformer.camera_to_robot(p_bottom_cam, flange_pose, debug=False)
        p_top_robot = transformer.camera_to_robot(p_top_cam, flange_pose, debug=False)
        world_h = p_top_robot[2] - p_bottom_robot[2]

        if not FORCE_SIDE_GRASP and (real_h <= SIDE_GRASP_MIN_HEIGHT_M or world_h <= SIDE_GRASP_MIN_HEIGHT_M):
            edge_u = (x1 + x2) / 2
            edge_v = y1
            mode = "top_center"
            print(
                f"  → 窄盒但高度 {real_h*100:.1f}cm / Z高差 {world_h*100:.1f}cm ≤ 侧抓安全高度 "
                f"{SIDE_GRASP_MIN_HEIGHT_M*100:.1f}cm, 改为上面抓取"
            )
        else:
            if real_h <= SIDE_GRASP_MIN_HEIGHT_M or world_h <= SIDE_GRASP_MIN_HEIGHT_M:
                pixel_ratio = 0.5
                print(
                    f"  → FORCE_SIDE_GRASP=1: 物体高度 {real_h*100:.1f}cm / Z高差 {world_h*100:.1f}cm "
                    f"不足侧抓安全高度, 仍临时从{edge}侧中部抓取"
                )
            else:
                midpoint_height_m = real_h / 2
                pixel_ratio = max(midpoint_height_m / real_h, SIDE_GRASP_MIN_HEIGHT_M / world_h)
            pixel_ratio = max(0.0, min(1.0, pixel_ratio))
            edge_v = y2 + pixel_ratio * (y1 - y2)
            edge_v = max(y1, min(y2, edge_v))
            mode = f"{edge}_side"
            side_height_m = world_h * pixel_ratio
            if side_height_m > SIDE_GRASP_MAX_HEIGHT_M:
                side_height_m = SIDE_GRASP_MAX_HEIGHT_M
                pixel_ratio = side_height_m / max(world_h, 0.001)
                edge_v = y2 + pixel_ratio * (y1 - y2)
                edge_v = max(y1, min(y2, edge_v))
            width_label = "强制侧抓" if FORCE_SIDE_GRASP and real_w > 0.08 else "窄盒"
            print(
                f"  → {width_label} ({real_w*100:.1f}cm), 从{edge}侧边缘抓取, "
                f"抓点离底部 Z高差 {side_height_m*100:.1f}cm, 像素=({edge_u:.0f},{edge_v:.0f})"
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
    if not mode.endswith("_side") and camera is not None and depth_img is not None:
        sampled_depth = camera.get_depth_at(depth_img, edge_u, edge_v, radius=5)
        if sampled_depth and sampled_depth > 0:
            grasp_depth = sampled_depth
            if abs(grasp_depth - depth) > 0.015:
                print(f"  抓取点深度修正: bbox中心 {depth:.3f}m → 抓取点 {grasp_depth:.3f}m")

    p_cam = transformer.pixel_to_camera(edge_u, edge_v, grasp_depth)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=True)
    return p_robot[0], p_robot[1], p_robot[2], mode, p_cam, (edge_u, edge_v)
