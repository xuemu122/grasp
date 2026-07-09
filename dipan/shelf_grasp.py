#!/usr/bin/env python3
"""
货架边缘抓取系统
- 统一入口: 识别、抓取、放置、底盘和菜单流程编排
- 硬件控制/手眼变换/抓取规划拆到独立模块，便于现场调参
"""

import sys
import os
import time
import json
import subprocess
import cv2
import numpy as np
from typing import Optional

# ── 识别模块 ──
sys.path.insert(0, "/home/nvidia/grasp/识别")
from orbbec_detect import MedicineLocatorSystem, RealSenseCamera
from orbbec_detect import _assess_lighting

from aero_gripper import AeroGripper
from dobot_arm import DobotArm, _clamp_cp
from grasp_planner import (
    TARGET_ROI_Y_MIN,
    TARGET_ROI_Y_MAX,
    axis_aligned_approach_m,
    pick_target_box,
    plan_edge_grasp,
)
from hand_eye import FX, FY, HandEyeTransformer
from depth_collision_guard import depth_to_robot_obstacles, check_hand_path, hand_path_hit_mask
from target_selection import (
    bbox_iou as _bbox_iou,
    filter_detections_by_target_query as _filter_detections_by_target_query,
    fuse_detection_tracks as _fuse_detection_tracks,
    is_clear_target_query as _is_clear_target_query,
    print_model_classes as _print_model_classes,
    print_target_filter_result as _print_target_filter_result,
    refine_target_depth_on_frame,
    unique_bboxes as _unique_bboxes,
)


def _parse_float_list_env(name, default):
    raw = os.getenv(name)
    if not raw:
        return tuple(default)
    try:
        values = tuple(float(v.strip()) for v in raw.split(",") if v.strip())
    except ValueError:
        print(f"[配置][警告] {name}={raw!r} 格式错误, 使用默认 {default}")
        return tuple(default)
    return values or tuple(default)


# ============================================================
# 参数
# ============================================================
HAND_LENGTH = 0.25      # 法兰 → 指尖 (米)
HAND_WEIGHT = 0.6       # 末端重量 (kg), Aero Hand
HAND_W_Y = 0.095        # 手 Y 方向宽度 (米)
HAND_W_X = 0.08         # 手 X 方向宽度 (米)

# 抓取参数 (基座系, 米): -X=前 +Y=右 +Z=上
# 侧面抓: 盒宽 <= 8cm, grip_mode 为 left_side/right_side
SIDE_GRASP_OFFSET_M = (-0.13, 0.02, 0.018)
# 上面抓: 盒宽 > 8cm, grip_mode 为 top_center。先沿用侧抓值, 实测后单独固化。
TOP_GRASP_OFFSET_M = (-0.12, 0.015, 0.035)

# 上面抓取前把第六轴转到手心向下的实测角度
TOP_J6_DEG = 8.3

# 抓取执行参数: 对齐位 -> 沿手指方向前进抓取 -> 后退。
APPROACH_LIFT_M = 0.00    # 对齐位额外抬高量, 当前不抬高
GRASP_ADVANCE_M = 0.04    # 抓取时沿手指方向前进 4cm
LOW_Z_GRASP_ADVANCE_M = 0.035  # 第4层低位空间更紧, 只探入 3.5cm
GRASP_APPROACH_BACKOFF_M = float(os.getenv("GRASP_APPROACH_BACKOFF_M", "0.15"))  # 抓取前沿手指方向后退对齐
TOP_APPROACH_BACKOFF_M = 0.15  # 上面抓时先在抓取轴线上退 15cm 再直线前进
LOW_Z_APPROACH_SPEED = 60
LOW_Z_GRASP_SPEED = 20
MOTION_SETTLE_S = float(os.getenv("MOTION_SETTLE_S", "0.2"))  # 运动完成后的短缓冲
GRASP_SETTLE_S = float(os.getenv("GRASP_SETTLE_S", "0.4"))    # 抓取前进/后退短缓冲
HAND_CLOSE_WAIT_S = float(os.getenv("HAND_CLOSE_WAIT_S", "3.0"))
PLACE_TRANSITION_CP = os.getenv("PLACE_TRANSITION_CP", "60")       # 过渡点到放置点的平滑比例
PLACE_SMOOTH_TRANSITION = os.getenv("PLACE_SMOOTH_TRANSITION", "0").lower() in ("1", "true", "yes", "on")
PLACE_RETURN_CLEARANCE_MM = float(os.getenv("PLACE_RETURN_CLEARANCE_MM", "140"))  # 放置后回层的安全抬高量
PLACE_RETURN_MAX_Z_MM = float(os.getenv("PLACE_RETURN_MAX_Z_MM", "820"))  # 回层抬高上限, 避免高位不可达

# 调试开关: 屏蔽手, 只测试手眼标定 + 机械臂走位
SKIP_HAND = False
# YOLO 显示/输出阈值。抓取现场默认放宽到 0.7，避免框不上。
DETECT_CONF_THRESHOLD = float(os.getenv("DETECT_CONF_THRESHOLD", "0.7"))
# 透明矿泉水/塑料瓶在复杂光照下置信度会明显偏低，只对饮料类单独放宽。
WATER_CONF_THRESHOLD = float(os.getenv("WATER_CONF_THRESHOLD", "0.52"))
DRINK_CLASS_IDS = set(range(70, 78)) | set(range(80, 87))
# 当前识别目标以瓶子为主, 默认使用瓶子抓取手势；原侧抓/上抓手势仍保留。
HAND_GRASP_MODE = os.getenv("HAND_GRASP_MODE", "bottle").lower()
DETECTION_STABILITY_IOU = float(os.getenv("DETECTION_STABILITY_IOU", "0.35"))
TARGET_PRODUCT_QUERY = os.getenv("TARGET_PRODUCT_QUERY", os.getenv("TARGET_NAME", "")).strip()
SHELF_GRASP_AUTO_ONCE = os.getenv("SHELF_GRASP_AUTO_ONCE", "0").lower() in ("1", "true", "yes", "on")
TARGET_DEPTH_REFINE_RADIUS_PX = int(os.getenv("TARGET_DEPTH_REFINE_RADIUS_PX", "10"))
TARGET_DEPTH_REFINE_MAX_DELTA_M = float(os.getenv("TARGET_DEPTH_REFINE_MAX_DELTA_M", "0.08"))
DEPTH_GUARD_ENABLE = os.getenv("DEPTH_GUARD_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_STRIDE = int(os.getenv("DEPTH_GUARD_STRIDE", "8"))
DEPTH_GUARD_CLEARANCE_M = float(os.getenv("DEPTH_GUARD_CLEARANCE_M", "0.005"))
DEPTH_GUARD_HAND_RADIUS_M = float(os.getenv("DEPTH_GUARD_HAND_RADIUS_M", str(HAND_W_Y * 0.5 + DEPTH_GUARD_CLEARANCE_M)))
DEPTH_GUARD_APPROACH_LENGTH_M = float(os.getenv("DEPTH_GUARD_APPROACH_LENGTH_M", "0.08"))
DEPTH_GUARD_PATH_SAMPLES = int(os.getenv("DEPTH_GUARD_PATH_SAMPLES", "8"))
DEPTH_GUARD_MAX_DEPTH_M = float(os.getenv("DEPTH_GUARD_MAX_DEPTH_M", "1.20"))
DEPTH_GUARD_EXCLUDE_MARGIN_PX = int(os.getenv("DEPTH_GUARD_EXCLUDE_MARGIN_PX", "32"))
DEPTH_GUARD_SELF_MASK_ENABLE = os.getenv("DEPTH_GUARD_SELF_MASK_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_SELF_MASK_RADIUS_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_RADIUS_M", str(DEPTH_GUARD_HAND_RADIUS_M + 0.02)))
DEPTH_GUARD_SELF_MASK_LENGTH_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_LENGTH_M", str(HAND_LENGTH)))
DEPTH_GUARD_SELF_MASK_BACKOFF_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_BACKOFF_M", "0.03"))
DEPTH_GUARD_DEPTH_ALIGN_ENABLE = os.getenv("DEPTH_GUARD_DEPTH_ALIGN_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M = float(os.getenv("DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M", "0.035"))
DEPTH_GUARD_AXIS_WINDOW_ENABLE = os.getenv("DEPTH_GUARD_AXIS_WINDOW_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M = float(os.getenv("DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M", "0.02"))
DEPTH_GUARD_FLANGE_BACK_MARGIN_M = float(os.getenv("DEPTH_GUARD_FLANGE_BACK_MARGIN_M", "0.03"))
DEPTH_GUARD_BYPASS_ENABLE = os.getenv("DEPTH_GUARD_BYPASS_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_BYPASS_LIFTS_MM = _parse_float_list_env("DEPTH_GUARD_BYPASS_LIFTS_MM", (60, 100, 140))
DEPTH_GUARD_BYPASS_SIDE_MM = _parse_float_list_env("DEPTH_GUARD_BYPASS_SIDE_MM", (60, -60))

# 货架层位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
SHELF_LAYERS_FILE = os.path.join(os.path.dirname(__file__), "shelf_layers.json")
STARTUP_LAYER_IDX = 2   # 程序启动固定回第2层, 避免从上次低层状态恢复导致碰撞
_current_layer_idx = STARTUP_LAYER_IDX  # 当前选中的层编号
_shelf_layer_poses = {}  # {1: ([x,y,z,rx,ry,rz], name), 2: ...}
LOW_Z_LAYER_IDXS = {4}  # 第4层在抬高基座下方, 允许低/负 Z

# 放置点位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
PLACE_POINT_FILE = os.path.join(os.path.dirname(__file__), "place_point.json")
_place_point_pose = None  # ([x,y,z,rx,ry,rz], name)
_place_point_joints = None  # [j1,j2,j3,j4,j5,j6]，可选
PLACE_TRANSITION_FILE = os.path.join(os.path.dirname(__file__), "place_transition.json")
_place_transition_pose = None  # ([x,y,z,rx,ry,rz], name)
_place_transition_joints = None  # [j1,j2,j3,j4,j5,j6]，可选

# 收起姿态: 调用底盘前和退出程序前使用, 防止机械臂/灵巧手与环境碰撞
STOW_POSE_FILE = os.path.join(os.path.dirname(__file__), "stow_pose.json")
_stow_pose = None  # ([x,y,z,rx,ry,rz], name)

# 放置前是否先让底盘去固定点。默认关闭, 现场确认 marker 后可用环境变量或主菜单 b 开启。
USE_CHASSIS_PLACE = os.getenv("USE_CHASSIS_PLACE", "0").lower() in ("1", "true", "yes", "on")
MOVE_ROBOT_SCRIPT = os.path.join(os.path.dirname(__file__), "move_robot.py")


def _layer_allows_low_z(layer_idx: Optional[int]) -> bool:
    return layer_idx in LOW_Z_LAYER_IDXS


def _current_layer_allows_low_z() -> bool:
    return _layer_allows_low_z(_current_layer_idx)


def _depth_guard_path_ok(
    label,
    obstacle_points_m,
    start_xyz_mm,
    target_xyz_mm,
    hand_dir_m,
    hand_length_m=HAND_LENGTH,
    camera_depth_axis_m=None,
    axis_min_m=None,
    axis_max_m=None,
    quiet=False,
) -> bool:
    if not DEPTH_GUARD_ENABLE:
        return True
    if obstacle_points_m is None or len(obstacle_points_m) == 0:
        if not quiet:
            print(f"  [深度护栏] {label}: 无有效障碍点, 放行")
        return True

    result = check_hand_path(
        obstacle_points_m,
        np.array(start_xyz_mm, dtype=np.float32) / 1000.0,
        np.array(target_xyz_mm, dtype=np.float32) / 1000.0,
        hand_dir_m,
        hand_length_m,
        DEPTH_GUARD_HAND_RADIUS_M,
        samples=DEPTH_GUARD_PATH_SAMPLES,
        camera_depth_axis_m=camera_depth_axis_m if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        camera_depth_tolerance_m=DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        axis_min_m=axis_min_m,
        axis_max_m=axis_max_m,
    )
    min_dist_mm = result.min_distance_m * 1000.0 if np.isfinite(result.min_distance_m) else float("inf")
    if not quiet:
        print(
            f"  [深度护栏] {label}: 最近 {min_dist_mm:.0f}mm, "
            f"命中 {result.hit_count}/{result.checked_points}, "
            f"半径 {DEPTH_GUARD_HAND_RADIUS_M*1000:.0f}mm, "
            f"长度 {hand_length_m*1000:.0f}mm"
            + (
                f", 深度对齐±{DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M*1000:.0f}mm"
                if DEPTH_GUARD_DEPTH_ALIGN_ENABLE and camera_depth_axis_m is not None
                else ""
            )
            + (
                f", 轴向{axis_min_m*1000:.0f}..{axis_max_m*1000:.0f}mm"
                if axis_min_m is not None and axis_max_m is not None
                else ""
            )
        )
    if not result.ok:
        if not quiet:
            print(f"  [深度护栏][中止] {label} 可能碰货架/障碍")
    return result.ok


def _depth_guard_check(label, obstacle_points_m, arm: "DobotArm", target_xyz_mm, hand_dir_m, hand_length_m=HAND_LENGTH) -> bool:
    """Check whether the planned hand sweep is clear before sending motion."""
    if not DEPTH_GUARD_ENABLE:
        return True
    if obstacle_points_m is None or len(obstacle_points_m) == 0:
        print(f"  [深度护栏] {label}: 无有效障碍点, 放行")
        return True
    try:
        cur_pose = arm.get_pose()
    except Exception as e:
        print(f"  [深度护栏][中止] {label}: 无法读取当前位姿: {e}")
        return False
    return _depth_guard_path_ok(label, obstacle_points_m, cur_pose[:3], target_xyz_mm, hand_dir_m, hand_length_m=hand_length_m)


def _path_is_depth_clear(
    label,
    obstacle_points_m,
    waypoints_mm,
    hand_dir_m,
    hand_length_m=HAND_LENGTH,
    camera_depth_axis_m=None,
    axis_min_m=None,
    axis_max_m=None,
) -> bool:
    for idx in range(len(waypoints_mm) - 1):
        if not _depth_guard_path_ok(
            f"{label} 绕行段{idx + 1}",
            obstacle_points_m,
            waypoints_mm[idx],
            waypoints_mm[idx + 1],
            hand_dir_m,
            hand_length_m=hand_length_m,
            camera_depth_axis_m=camera_depth_axis_m,
            axis_min_m=axis_min_m,
            axis_max_m=axis_max_m,
            quiet=True,
        ):
            return False
    return True


def _robot_points_to_pixels(points_m, transformer, flange_pose_mm, image_shape):
    if points_m is None or len(points_m) == 0 or flange_pose_mm is None:
        return np.zeros((0, 2), dtype=np.int32), np.zeros((0,), dtype=bool)
    pose = tuple(float(x) for x in flange_pose_mm)
    T_base_flange = np.eye(4, dtype=np.float32)
    T_base_flange[:3, :3] = transformer._rpy_to_rot(pose[3], pose[4], pose[5]).astype(np.float32)
    T_base_flange[:3, 3] = np.array(pose[:3], dtype=np.float32) / 1000.0
    T_base_cam = T_base_flange @ transformer.T_flange_to_cam.astype(np.float32)
    T_cam_base = np.linalg.inv(T_base_cam)

    pts = np.asarray(points_m, dtype=np.float32)
    pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1), dtype=np.float32)], axis=1)
    cam = (T_cam_base @ pts_h.T).T[:, :3]
    z = cam[:, 2]
    u = cam[:, 0] * float(transformer.fx) / np.maximum(z, 1e-6) + float(transformer.cx)
    v = cam[:, 1] * float(transformer.fy) / np.maximum(z, 1e-6) + float(transformer.cy)
    h, w = image_shape[:2]
    valid = (z > 0.02) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    pixels = np.stack([u, v], axis=1).astype(np.int32)
    return pixels, valid


def _camera_depth_axis_m(transformer, flange_pose_mm):
    if transformer is None or flange_pose_mm is None:
        return None
    pose = tuple(float(x) for x in flange_pose_mm)
    T_base_flange = np.eye(4, dtype=np.float32)
    T_base_flange[:3, :3] = transformer._rpy_to_rot(pose[3], pose[4], pose[5]).astype(np.float32)
    T_base_cam = T_base_flange @ transformer.T_flange_to_cam.astype(np.float32)
    axis = T_base_cam[:3, 2]
    return axis / max(float(np.linalg.norm(axis)), 1e-9)


def _save_depth_guard_viz(
    label,
    image,
    obstacle_points_m,
    obstacle_pixels,
    target_bbox,
    exclude_bboxes,
    start_xyz_mm,
    target_xyz_mm,
    hand_dir_m,
    hand_length_m,
    transformer,
    observation_flange_pose,
    camera_depth_axis_m=None,
    axis_min_m=None,
    axis_max_m=None,
):
    if image is None or obstacle_points_m is None or obstacle_pixels is None:
        return None
    viz = image.copy()
    pixels = np.asarray(obstacle_pixels, dtype=np.int32)

    if len(pixels):
        for u, v in pixels:
            if 0 <= u < viz.shape[1] and 0 <= v < viz.shape[0]:
                cv2.circle(viz, (int(u), int(v)), 1, (150, 150, 150), -1)

    hit_mask, min_dist = hand_path_hit_mask(
        obstacle_points_m,
        np.array(start_xyz_mm, dtype=np.float32) / 1000.0,
        np.array(target_xyz_mm, dtype=np.float32) / 1000.0,
        hand_dir_m,
        hand_length_m,
        DEPTH_GUARD_HAND_RADIUS_M,
        samples=DEPTH_GUARD_PATH_SAMPLES,
        camera_depth_axis_m=camera_depth_axis_m if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        camera_depth_tolerance_m=DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        axis_min_m=axis_min_m,
        axis_max_m=axis_max_m,
    )
    hit_pixels = pixels[hit_mask] if len(pixels) == len(hit_mask) else np.zeros((0, 2), dtype=np.int32)
    for u, v in hit_pixels:
        if 0 <= u < viz.shape[1] and 0 <= v < viz.shape[0]:
            cv2.circle(viz, (int(u), int(v)), 3, (0, 0, 255), -1)

    if exclude_bboxes:
        for bbox in exclude_bboxes:
            if bbox is None:
                continue
            x1, y1, x2, y2 = [int(v) for v in bbox]
            cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 180, 0), 1)

    if target_bbox is not None:
        x1, y1, x2, y2 = [int(v) for v in target_bbox]
        cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 2)

    start = np.array(start_xyz_mm, dtype=np.float32) / 1000.0
    end = np.array(target_xyz_mm, dtype=np.float32) / 1000.0
    hand_dir = np.asarray(hand_dir_m, dtype=np.float32)
    hand_dir = hand_dir / max(float(np.linalg.norm(hand_dir)), 1e-9)
    path_points = []
    for alpha in np.linspace(0.0, 1.0, max(2, DEPTH_GUARD_PATH_SAMPLES)):
        root = (1.0 - alpha) * start + alpha * end
        tip = root + float(hand_length_m) * hand_dir
        path_points.extend([root, tip])
    path_pixels, path_valid = _robot_points_to_pixels(
        np.asarray(path_points, dtype=np.float32),
        transformer,
        observation_flange_pose,
        viz.shape,
    )
    for i in range(0, len(path_pixels), 2):
        if i + 1 < len(path_pixels) and path_valid[i] and path_valid[i + 1]:
            p0 = tuple(int(x) for x in path_pixels[i])
            p1 = tuple(int(x) for x in path_pixels[i + 1])
            cv2.line(viz, p0, p1, (255, 255, 0), 1)
            cv2.circle(viz, p0, 2, (255, 255, 0), -1)

    finite_min = min_dist[np.isfinite(min_dist)] if len(min_dist) else np.zeros((0,), dtype=np.float32)
    min_mm = float(np.min(finite_min) * 1000.0) if len(finite_min) else float("inf")
    cv2.putText(
        viz,
        f"{label} hit={int(np.count_nonzero(hit_mask))} min={min_mm:.0f}mm r={DEPTH_GUARD_HAND_RADIUS_M*1000:.0f} len={hand_length_m*1000:.0f}",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 255) if np.any(hit_mask) else (0, 255, 0),
        2,
    )
    safe_label = "".join(ch if ch.isalnum() else "_" for ch in str(label))
    path = f"/tmp/depth_guard_{safe_label}.jpg"
    cv2.imwrite(path, viz)
    print(f"  [深度护栏] 可视化已保存: {path} (灰=障碍点, 红=命中点, 绿=排除框/目标框, 青=手部检查线)")
    return path


def _movel_with_depth_guard(
    arm: "DobotArm",
    label,
    obstacle_points_m,
    target_xyz_mm,
    hand_dir_m,
    speed=100,
    allow_low_z=False,
    allow_bypass=True,
    hand_length_m=HAND_LENGTH,
    viz_image=None,
    obstacle_pixels=None,
    target_bbox=None,
    exclude_bboxes=None,
    transformer=None,
    observation_flange_pose=None,
    camera_depth_axis_m=None,
    axis_min_m=None,
    axis_max_m=None,
) -> bool:
    target = np.array(target_xyz_mm, dtype=np.float32)
    try:
        cur_pose = arm.get_pose()
    except Exception as e:
        print(f"  [深度护栏][中止] {label}: 无法读取当前位姿: {e}")
        return False
    start = np.array(cur_pose[:3], dtype=np.float32)

    direct_ok = _depth_guard_path_ok(
        label,
        obstacle_points_m,
        start,
        target,
        hand_dir_m,
        hand_length_m=hand_length_m,
        camera_depth_axis_m=camera_depth_axis_m,
        axis_min_m=axis_min_m,
        axis_max_m=axis_max_m,
    )
    if direct_ok:
        if arm.movel(float(target[0]), float(target[1]), float(target[2]), speed=speed, allow_low_z=allow_low_z):
            return True
        print(f"  [深度绕行] {label}: 直达 MovL 失败, 尝试中间点绕行")
    else:
        _save_depth_guard_viz(
            label,
            viz_image,
            obstacle_points_m,
            obstacle_pixels,
            target_bbox,
            exclude_bboxes,
            start,
            target,
            hand_dir_m,
            hand_length_m,
            transformer,
            observation_flange_pose,
            camera_depth_axis_m,
            axis_min_m,
            axis_max_m,
        )

    if not (DEPTH_GUARD_BYPASS_ENABLE and allow_bypass):
        print(f"  [深度护栏][中止] {label} 无可用绕行, 不发动作")
        return False

    candidates = []
    for lift_mm in DEPTH_GUARD_BYPASS_LIFTS_MM:
        safe_z = max(float(start[2]), float(target[2])) + float(lift_mm)
        candidates.append((
            f"上抬{lift_mm:.0f}mm",
            [
                np.array([start[0], start[1], safe_z], dtype=np.float32),
                np.array([target[0], target[1], safe_z], dtype=np.float32),
                target,
            ],
        ))
    base_lift = float(DEPTH_GUARD_BYPASS_LIFTS_MM[0]) if DEPTH_GUARD_BYPASS_LIFTS_MM else 60.0
    for side_mm in DEPTH_GUARD_BYPASS_SIDE_MM:
        mid = (start + target) * 0.5
        mid[1] += float(side_mm)
        mid[2] = max(float(start[2]), float(target[2])) + base_lift
        candidates.append((f"侧绕Y{side_mm:+.0f}mm", [mid, target]))

    for desc, path in candidates:
        waypoints = [start] + path
        if not _path_is_depth_clear(
            label,
            obstacle_points_m,
            waypoints,
            hand_dir_m,
            hand_length_m=hand_length_m,
            camera_depth_axis_m=camera_depth_axis_m,
            axis_min_m=axis_min_m,
            axis_max_m=axis_max_m,
        ):
            continue
        print(f"  [深度绕行] {label}: 采用 {desc}, {len(path)} 个中间/目标点")
        for point in path:
            if not arm.movel(float(point[0]), float(point[1]), float(point[2]), speed=speed, allow_low_z=allow_low_z):
                print(f"  [深度绕行][停止] {label}: 绕行点运动失败")
                return False
            time.sleep(MOTION_SETTLE_S)
        return True

    print(f"  [深度绕行][中止] {label}: 试过上抬/侧绕, 仍没有无碰路径")
    return False


def load_shelf_layers():
    """从 JSON 加载货架层位姿，启动时固定默认第2层。"""
    global _shelf_layer_poses, _current_layer_idx
    if os.path.exists(SHELF_LAYERS_FILE):
        with open(SHELF_LAYERS_FILE) as f:
            data = json.load(f)
        for k, v in data.get("layers", {}).items():
            _shelf_layer_poses[int(k)] = (v["pose"], v.get("name", f"第{k}层"))
        saved_current = data.get("current")
        if STARTUP_LAYER_IDX in _shelf_layer_poses:
            if saved_current != STARTUP_LAYER_IDX:
                print(f"[货架层] 启动默认固定为第{STARTUP_LAYER_IDX}层 (忽略上次 current={saved_current})")
            _current_layer_idx = STARTUP_LAYER_IDX
        else:
            _current_layer_idx = saved_current
            print(f"[货架层][警告] 启动默认第{STARTUP_LAYER_IDX}层未记录, 暂用 current={_current_layer_idx}")
    return _shelf_layer_poses, _current_layer_idx


def load_place_point():
    """从 JSON 加载放置点位姿，返回 (pose, name) 或 None"""
    global _place_point_pose, _place_point_joints
    if os.path.exists(PLACE_POINT_FILE):
        with open(PLACE_POINT_FILE) as f:
            data = json.load(f)
        if "pose" in data:
            _place_point_pose = (data["pose"], data.get("name", "放置点"))
            _place_point_joints = data.get("joints")
    return _place_point_pose


def load_place_transition():
    """从 JSON 加载放置过渡点位姿，返回 (pose, name) 或 None"""
    global _place_transition_pose, _place_transition_joints
    if os.path.exists(PLACE_TRANSITION_FILE):
        with open(PLACE_TRANSITION_FILE) as f:
            data = json.load(f)
        if "pose" in data:
            _place_transition_pose = (data["pose"], data.get("name", "放置过渡点"))
            _place_transition_joints = data.get("joints")
    return _place_transition_pose


def load_stow_pose():
    """从 JSON 加载收起姿态，返回 (pose, name) 或 None"""
    global _stow_pose
    if os.path.exists(STOW_POSE_FILE):
        with open(STOW_POSE_FILE) as f:
            data = json.load(f)
        if "pose" in data:
            _stow_pose = (data["pose"], data.get("name", "收起姿态"))
    return _stow_pose


def save_shelf_layers():
    """保存货架层位姿到 JSON"""
    data = {
        
        "layers": {str(k): {"pose": p, "name": n} for k, (p, n) in _shelf_layer_poses.items()},
        "current": _current_layer_idx,
    }
    with open(SHELF_LAYERS_FILE, "w") as f:
        json.dump(data, f, indent=2)


def save_place_point():
    """保存放置点位姿到 JSON"""
    if _place_point_pose is None:
        if os.path.exists(PLACE_POINT_FILE):
            os.remove(PLACE_POINT_FILE)
        return
    pose, name = _place_point_pose
    data = {"pose": pose, "name": name}
    if _place_point_joints is not None:
        data["joints"] = _place_point_joints
    with open(PLACE_POINT_FILE, "w") as f:
        json.dump(data, f, indent=2)


def save_place_transition():
    """保存放置过渡点位姿到 JSON"""
    if _place_transition_pose is None:
        if os.path.exists(PLACE_TRANSITION_FILE):
            os.remove(PLACE_TRANSITION_FILE)
        return
    pose, name = _place_transition_pose
    data = {"pose": pose, "name": name}
    if _place_transition_joints is not None:
        data["joints"] = _place_transition_joints
    with open(PLACE_TRANSITION_FILE, "w") as f:
        json.dump(data, f, indent=2)


def save_stow_pose():
    """保存收起姿态到 JSON"""
    if _stow_pose is None:
        if os.path.exists(STOW_POSE_FILE):
            os.remove(STOW_POSE_FILE)
        return
    pose, name = _stow_pose
    data = {"pose": pose, "name": name}
    with open(STOW_POSE_FILE, "w") as f:
        json.dump(data, f, indent=2)


def go_to_layer(arm: "DobotArm", layer_idx: int):
    """移动机械臂到指定货架层的观察位姿 (MovL 笛卡尔)"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False
    pose, name = _shelf_layer_poses[layer_idx]
    print(f"\n[货架层] 移动到第{layer_idx}层 ({name})...")
    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < 80.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 80mm")
        target_z = 80.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5], speed=100,
                     allow_low_z=allow_low_z):
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def go_to_layer_smooth_from_place(arm: "DobotArm", layer_idx: int):
    """放置后返回层位姿；先抬高避碰，再平滑经过高位过渡点。"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False

    pose, name = _shelf_layer_poses[layer_idx]
    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < 80.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 80mm")
        target_z = 80.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")

    try:
        cur = arm.get_pose()
    except Exception as e:
        print(f"  [警告] 无法读取当前位姿, 回退为普通回层: {e}")
        return go_to_layer(arm, layer_idx)

    z_candidates = [cur[2], target_z]
    if _place_transition_pose is not None:
        z_candidates.append(_place_transition_pose[0][2])
    base_z = max(z_candidates)
    max_return_z = min(1100.0, max(base_z + 60.0, PLACE_RETURN_MAX_Z_MM))
    safe_z = min(max_return_z, base_z + PLACE_RETURN_CLEARANCE_MM)
    cp_pct = _clamp_cp(PLACE_TRANSITION_CP)
    descend_cp = min(cp_pct, 40)

    lift_targets = []
    for z in (
        safe_z,
        min(max_return_z, base_z + 100.0),
        min(max_return_z, base_z + 60.0),
    ):
        if z > cur[2] + 20.0 and all(abs(z - seen) > 10.0 for seen in lift_targets):
            lift_targets.append(z)
    if not lift_targets:
        lift_targets.append(max(cur[2], target_z))

    print(f"\n[安全回层] 先原地抬高, 再回第{layer_idx}层 ({name})...")
    lifted = False
    for idx, lift_z in enumerate(lift_targets, 1):
        print(f"  [安全回层] 尝试抬高到 Z={lift_z:.0f}mm ({idx}/{len(lift_targets)})")
        if arm.movel(cur[0], cur[1], lift_z, cur[3], cur[4], cur[5], speed=80, cp=0):
            safe_z = lift_z
            lifted = True
            break
        try:
            cur = arm.get_pose()
        except Exception:
            pass
    if not lifted:
        print("  [安全回层] 原地抬高失败")
        return False

    if _place_transition_pose is not None:
        t_pose, t_name = _place_transition_pose
        print(f"\n[放置过渡点] 回程关节运动到 {t_name} ...")
        ok = arm.move_pose_safe(t_pose, joints=_place_transition_joints, speed=80)
        if not ok:
            print("  [放置过渡点] 回程移动失败")
            return False

    print(f"\n[货架层] 高位移动到第{layer_idx}层上方 ...")
    if not arm.movel(pose[0], pose[1], safe_z, pose[3], pose[4], pose[5],
                     speed=100, allow_low_z=allow_low_z and safe_z < 50.0,
                     cp=descend_cp, wait=False):
        return False

    print(f"\n[货架层] 下落到第{layer_idx}层 ({name})...")
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5],
                     speed=100, allow_low_z=allow_low_z, cp=0):
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def go_to_layer_via_clearance(arm: "DobotArm", layer_idx: int, clearance_mm: float = 120.0):
    """先抬高再回到指定层, 用于避开放置点到货架层的直线路径"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False
    pose, name = _shelf_layer_poses[layer_idx]
    print(f"\n[货架层] 安全返回第{layer_idx}层 ({name})...")
    try:
        cur = arm.get_pose()
    except Exception as e:
        print(f"  [警告] 无法读取当前位姿, 回退为直接 MovL: {e}")
        return go_to_layer(arm, layer_idx)

    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < 80.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 80mm 作为可达观察位")
        target_z = 80.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")
    safe_z = max(cur[2], target_z) + clearance_mm
    if not arm.movel(cur[0], cur[1], safe_z, speed=120,
                     allow_low_z=allow_low_z and safe_z < 50.0):
        return False
    if not arm.movel(pose[0], pose[1], safe_z, pose[3], pose[4], pose[5], speed=120,
                     allow_low_z=allow_low_z and safe_z < 50.0):
        return False
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5], speed=100,
                     allow_low_z=allow_low_z):
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def go_to_place_point(arm: "DobotArm"):
    """移动机械臂到记录的放置点 (MovL 笛卡尔)"""
    if _place_point_pose is None:
        print("  [错误] 放置点未记录! 先用 p 进入放置点管理, 拖到目标位置后输入 s")
        return False
    pose, name = _place_point_pose
    if _place_transition_pose is not None:
        t_pose, t_name = _place_transition_pose
        print(f"\n[放置过渡点] 关节运动到 {t_name} ...")
        ok = arm.move_pose_safe(t_pose, joints=_place_transition_joints, speed=80)
        if not ok:
            print("  [放置过渡点] 移动失败")
            return False
    print(f"\n[放置点] 移动到 {name} ...")
    place_cp = _clamp_cp(PLACE_TRANSITION_CP) if PLACE_SMOOTH_TRANSITION else 0
    if not arm.movel(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], speed=100, cp=place_cp):
        return False
    time.sleep(MOTION_SETTLE_S)
    try:
        cur = arm.get_pose()
    except Exception:
        print("  [警告] 无法确认是否到达放置点, 本次不自动张手")
        return False
    err_mm = np.linalg.norm(np.array(cur[:3]) - np.array(pose[:3]))
    if err_mm > 30:
        print(f"  [警告] 当前距放置点 {err_mm:.0f}mm, 本次不自动张手")
        return False
    return True


def go_to_place_transition(arm: "DobotArm"):
    """如果记录了放置过渡点，则先移动到过渡点。未记录时直接跳过。"""
    if _place_transition_pose is None:
        return True
    pose, name = _place_transition_pose
    print(f"\n[放置过渡点] 移动到 {name} ...")
    ok = arm.move_pose_safe(pose, joints=_place_transition_joints, speed=80)
    if not ok:
        print("  [放置过渡点] 移动失败")
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def go_to_stow_pose(arm: "DobotArm"):
    """先回第2层观察位，再移动到记录的收起姿态。未记录时只提示并返回 False。"""
    if _stow_pose is None:
        print("  [收起姿态] 未记录, 跳过。可用 u 菜单拖动记录")
        return False
    if 2 in _shelf_layer_poses:
        print("\n[收起姿态] 先回第2层避碰 ...")
        if not go_to_layer_via_clearance(arm, 2):
            print("  [收起姿态] 回第2层失败, 不继续收起")
            return False
    else:
        print("  [收起姿态] 第2层未记录, 不继续收起")
        return False
    pose, name = _stow_pose
    print(f"\n[收起姿态] 移动到 {name} ...")
    if not arm.movel(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], speed=100):
        print("  [收起姿态] 移动失败")
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def move_chassis_to_place():
    """复用 move_robot.py, 让底盘去放置固定点。"""
    print("\n[底盘] 前往放置固定点 ...")
    try:
        result = subprocess.run(
            [sys.executable, MOVE_ROBOT_SCRIPT, "place"],
            cwd=os.path.dirname(__file__),
            text=True,
            capture_output=False,
            check=False,
        )
    except Exception as e:
        print(f"  [底盘] 调用 move_robot.py 失败: {e}")
        return False
    if result.returncode != 0:
        print(f"  [底盘] move_robot.py place 返回 {result.returncode}, 停止放置流程")
        return False
    return True


def finish_pick_cycle(arm: "DobotArm", hand: Optional["AeroGripper"], current_layer_idx: Optional[int]):
    """抓取后放置并回到观察层。低层上撤走指定中转层, 下降回低层时走安全路径。"""
    hand_ok = hand is not None and not SKIP_HAND

    if _place_point_pose is None:
        if hand_ok:
            hand.open()
        return True

    if current_layer_idx == 4:
        transfer_layers = [3, 1]
    elif current_layer_idx == 3:
        transfer_layers = [1]
    else:
        transfer_layers = []
    use_transfer_layer = bool(transfer_layers)

    if use_transfer_layer:
        print(f"\n[Transfer] 第{current_layer_idx}层抓取后先回当前层观察位 ...")
        if not go_to_layer(arm, current_layer_idx):
            print("  [停止] 回当前层失败, 停在当前位置, 不继续放置流程")
            return False
        print(f"\n[Transfer] 直接上撤: {' → '.join(f'第{i}层' for i in transfer_layers)}, 再前往放置点 ...")
        for layer_idx in transfer_layers:
            if layer_idx not in _shelf_layer_poses:
                print(f"  [停止] 第{layer_idx}层未记录, 停在当前位置, 不继续放置流程")
                return False
            if not go_to_layer(arm, layer_idx):
                print(f"  [停止] 回第{layer_idx}层中转失败, 停在当前位置, 不继续放置流程")
                return False

    if USE_CHASSIS_PLACE:
        if not go_to_stow_pose(arm):
            print("  [停止] 底盘移动前未能到达收起姿态, 不调用底盘")
            return False
        if not move_chassis_to_place():
            print("  [停止] 底盘未能前往放置固定点, 不继续机械臂放置")
            return False

    print(f"\n[Place] 前往放置点 ({_place_point_pose[1]}) ...")
    if not go_to_place_point(arm):
        print("  [停止] 前往放置点失败, 停在当前位置, 不继续回层")
        return False
    if hand_ok:
        hand.open()
    time.sleep(MOTION_SETTLE_S)

    if use_transfer_layer:
        home_transfer_idx = transfer_layers[-1]
        print(f"\n[Transfer] 放置后回第{home_transfer_idx}层中转 ...")
        if not go_to_layer_smooth_from_place(arm, home_transfer_idx):
            print(f"  [停止] 放置后回第{home_transfer_idx}层失败, 停在当前位置")
            return False

    if current_layer_idx in _shelf_layer_poses:
        print(f"\n[Home] 回到第{current_layer_idx}层 ({_shelf_layer_poses[current_layer_idx][1]}) ...")
        if use_transfer_layer:
            ok = go_to_layer_via_clearance(arm, current_layer_idx)
        else:
            ok = go_to_layer_smooth_from_place(arm, current_layer_idx)
        if not ok:
            print("  [停止] 回当前层失败, 停在当前位置")
            return False
    else:
        print("\n[Home] 回到 HOME ...")
        arm.go_home()
    return True


# ============================================================
# 主流程
# ============================================================
def _layer_management(arm: "DobotArm"):
    """货架层管理子菜单: 切换、记录、删除层"""
    global _current_layer_idx

    while True:
        print(f"\n── 货架层管理 ──")
        if _current_layer_idx:
            print(f"  当前层: 第{_current_layer_idx}层 ({_shelf_layer_poses.get(_current_layer_idx, (None, '?'))[1]})")
        else:
            print(f"  当前: HOME (未选层)")
        if _shelf_layer_poses:
            for idx in sorted(_shelf_layer_poses):
                p, name = _shelf_layer_poses[idx]
                marker = " ← 当前" if idx == _current_layer_idx else ""
                print(f"    第{idx}层: {name}  pose=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f},{p[3]:.0f},{p[4]:.0f},{p[5]:.0f}){marker}")
        else:
            print(f"  (无已记录层。拖动机械臂到目标层后, 输入 s 保存)")
        print(f"\n  操作:")
        print(f"    <数字>   ← 切换并移动到该层")
        print(f"    s       ← 记录当前法兰位姿为新层")
        print(f"    s <数字> ← 用当前位姿覆盖指定层")
        print(f"    d <数字> ← 删除指定层记录")
        print(f"    q       ← 返回主菜单")

        choice = input("  层管理 > ").strip()
        if choice.lower() == "q":
            return

        if choice.lower().startswith("s"):
            # 保存当前位姿
            try:
                pose = arm.get_pose()
            except Exception:
                print("  无法获取当前位姿")
                continue

            parts = choice.split()
            if len(parts) >= 2:
                try:
                    idx = int(parts[1])
                except ValueError:
                    print("  格式: s <数字>")
                    continue
            else:
                # 自动编号: 最大已有编号 + 1
                idx = max(_shelf_layer_poses.keys()) + 1 if _shelf_layer_poses else 1

            name = input('  层名称 (回车默认"第{}层"): '.format(idx)).strip()
            if not name:
                name = f"第{idx}层"

            _shelf_layer_poses[idx] = (list(pose[:6]), name)
            _current_layer_idx = idx
            save_shelf_layers()
            print(f"  ✓ 第{idx}层 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice.lower().startswith("d"):
            parts = choice.split()
            if len(parts) < 2:
                print("  格式: d <数字>")
                continue
            try:
                idx = int(parts[1])
            except ValueError:
                print("  格式: d <数字>")
                continue
            if idx not in _shelf_layer_poses:
                print(f"  第{idx}层不存在")
                continue
            del _shelf_layer_poses[idx]
            if _current_layer_idx == idx:
                _current_layer_idx = max(_shelf_layer_poses.keys()) if _shelf_layer_poses else None
            save_shelf_layers()
            print(f"  ✓ 第{idx}层已删除, 当前→{'HOME' if _current_layer_idx is None else f'第{_current_layer_idx}层'}")
            continue

        # 切换层
        try:
            idx = int(choice)
        except ValueError:
            print("  无效输入, 请输入数字 / s / d / q")
            continue

        if idx not in _shelf_layer_poses:
            print(f"  第{idx}层未记录! 先拖机械臂到对应位置, 然后 s {idx} 保存")
            continue

        _current_layer_idx = idx
        save_shelf_layers()
        go_to_layer_via_clearance(arm, idx)
        return  # 切换后返回主菜单


def _place_point_management(arm: "DobotArm"):
    """放置点管理子菜单: 记录、移动到、删除放置点"""
    global _place_point_pose, _place_transition_pose, _place_point_joints, _place_transition_joints

    while True:
        print(f"\n── 放置点管理 ──")
        if _place_point_pose:
            pose, name = _place_point_pose
            joint_state = "有joints" if _place_point_joints is not None else "无joints"
            print(f"  当前放置点: {name}  pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f}) [{joint_state}]")
        else:
            print("  当前: 未记录放置点")
        if _place_transition_pose:
            pose, name = _place_transition_pose
            joint_state = "有joints" if _place_transition_joints is not None else "无joints"
            print(f"  当前过渡点: {name}  pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f}) [{joint_state}]")
        else:
            print("  当前: 未记录放置过渡点")
        print("\n  操作:")
        print("    s       ← 记录当前法兰位姿为放置点")
        print("    g       ← 移动到放置点 (若有过渡点则先经过渡点)")
        print("    d       ← 删除放置点")
        print("    ts      ← 记录当前法兰位姿为放置过渡点")
        print("    tg      ← 移动到放置过渡点")
        print("    td      ← 删除放置过渡点")
        print("    q       ← 返回主菜单")

        choice = input("  放置点 > ").strip().lower()
        if choice == "q":
            return

        if choice == "s":
            try:
                pose = arm.get_pose()
                joints = arm.get_joints()
            except Exception:
                print("  无法获取当前位姿/关节角")
                continue
            name = input('  放置点名称 (回车默认"放置点"): ').strip()
            if not name:
                name = "放置点"
            _place_point_pose = (list(pose[:6]), name)
            _place_point_joints = list(joints[:6])
            save_place_point()
            print(f"  ✓ 放置点 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice == "d":
            _place_point_pose = None
            _place_point_joints = None
            save_place_point()
            print("  ✓ 放置点已删除")
            continue

        if choice == "ts":
            try:
                pose = arm.get_pose()
                joints = arm.get_joints()
            except Exception:
                print("  无法获取当前位姿/关节角")
                continue
            name = input('  过渡点名称 (回车默认"放置过渡点"): ').strip()
            if not name:
                name = "放置过渡点"
            _place_transition_pose = (list(pose[:6]), name)
            _place_transition_joints = list(joints[:6])
            save_place_transition()
            print(f"  ✓ 放置过渡点 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice == "td":
            _place_transition_pose = None
            _place_transition_joints = None
            save_place_transition()
            print("  ✓ 放置过渡点已删除")
            continue

        if choice == "tg":
            go_to_place_transition(arm)
            return

        if choice == "g":
            go_to_place_point(arm)
            return

        print("  无效输入, 请输入 s / g / d / ts / tg / td / q")


def _stow_pose_management(arm: "DobotArm"):
    """收起姿态管理子菜单: 记录、移动到、删除收起姿态"""
    global _stow_pose

    while True:
        print(f"\n── 收起姿态管理 ──")
        if _stow_pose:
            pose, name = _stow_pose
            print(f"  当前收起姿态: {name}  pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
        else:
            print("  当前: 未记录收起姿态")
        print("\n  操作:")
        print("    s       ← 记录当前法兰位姿为收起姿态")
        print("    g       ← 移动到收起姿态")
        print("    d       ← 删除收起姿态")
        print("    q       ← 返回主菜单")

        choice = input("  收起姿态 > ").strip().lower()
        if choice == "q":
            return

        if choice == "s":
            try:
                pose = arm.get_pose()
            except Exception:
                print("  无法获取当前位姿")
                continue
            name = input('  收起姿态名称 (回车默认"收起姿态"): ').strip()
            if not name:
                name = "收起姿态"
            _stow_pose = (list(pose[:6]), name)
            save_stow_pose()
            print(f"  ✓ 收起姿态 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice == "d":
            _stow_pose = None
            save_stow_pose()
            print("  ✓ 收起姿态已删除")
            continue

        if choice == "g":
            go_to_stow_pose(arm)
            return

        print("  无效输入, 请输入 s / g / d / q")


def _toggle_chassis_place():
    """切换放置前底盘固定点移动。"""
    global USE_CHASSIS_PLACE
    USE_CHASSIS_PLACE = not USE_CHASSIS_PLACE
    state = "开启" if USE_CHASSIS_PLACE else "关闭"
    print(f"\n[底盘放置] 已{state}")
    if USE_CHASSIS_PLACE:
        print("  流程: 先回第2层 → 到收起姿态 → 调用 move_robot.py place → 等底盘到达 → 机械臂放置")


def main():
    print("=" * 60)
    print("  货架边缘抓取系统")
    print(f"  灵巧手: {HAND_LENGTH*100:.0f}cm  {HAND_W_Y*100:.1f}cm×{HAND_W_X*100:.1f}cm  {HAND_WEIGHT}kg")
    print(f"  手势模式: {HAND_GRASP_MODE}")
    print(f"  目标商品: {TARGET_PRODUCT_QUERY or '未指定'}")
    if SHELF_GRASP_AUTO_ONCE:
        print("  前端自动模式: 单次抓取后自动退出")
    print("=" * 60)
    target_query = TARGET_PRODUCT_QUERY

    # ── 1. 初始化相机 + YOLO ──
    print("\n[1/4] 初始化相机 + YOLO ...")
    camera = RealSenseCamera(width=640, height=480, fps=30)
    locator = MedicineLocatorSystem(
        model_path="/home/nvidia/grasp/识别/best.pt",
        mapping_path="/home/nvidia/grasp/识别/shelf_mapping.yaml",
        conf_threshold=DETECT_CONF_THRESHOLD,
        class_conf_thresholds={cls_id: WATER_CONF_THRESHOLD for cls_id in DRINK_CLASS_IDS},
        use_ocr=False,
    )
    locator.init_camera = lambda: None
    locator.camera = camera

    # ── 2. 连接 DOBOT ──
    print("\n[2/4] 连接 DOBOT ...")
    arm = DobotArm()
    arm.connect()

    # 回初始观察位姿
    print("\n[Home] 回到预设初始位姿 ...")
    arm.go_home()
    time.sleep(MOTION_SETTLE_S)  # 短缓冲，MovJ 内部已等待运动完成

    # ── 3. 连接灵巧手 ──
    if SKIP_HAND:
        print("\n[3/4] 灵巧手 已屏蔽 (SKIP_HAND=True)")
        hand = None
    else:
        print("\n[3/4] 连接 Aero Hand 灵巧手 ...")
        hand = AeroGripper()
        hand.connect()

    # ── 货架层管理 ──
    load_shelf_layers()
    load_place_point()
    load_place_transition()
    load_stow_pose()
    if _current_layer_idx == STARTUP_LAYER_IDX:
        save_shelf_layers()
    if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
        if go_to_layer_via_clearance(arm, _current_layer_idx) and hand is not None:
            hand.open()
            time.sleep(MOTION_SETTLE_S)

    # ── 4. 手眼标定 ──
    print("\n[4/4] 手眼标定 ...")
    transformer = HandEyeTransformer(
        fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy
    )
    print(f"  内参: fx={FX:.1f} fy={FY:.1f}")

    print("\n" + "=" * 60)
    print("  操作说明:")
    print("  - 拖动机械臂到观察位姿 (能看到货架盒子)")
    print("  - 按 Enter ← 拍照识别并抓取最外侧盒子")
    print("  - 输入 l   ← 货架层管理 (切换/记录/删除)")
    print("  - 输入 p   ← 放置点管理 (记录/移动/删除)")
    print("  - 输入 u   ← 收起姿态管理 (记录/移动/删除)")
    print("  - 输入 b   ← 切换底盘放置流程")
    print("  - 输入 c   ← 设置目标商品/类别/id")
    print("  - 输入 ?   ← 查看全部模型类别")
    print("  - 输入 v   ← 实时画面预览 (YOLO 检测)")
    print("  - 输入 j   ← 手动点动 (wasd/e/r 微调法兰)")
    print("  - 也可直接输入目标名抓取；输入 all/clear 清空目标")
    print("  - 输入 q   ← 退出")
    print("=" * 60)

    try:
        auto_attempt_pending = SHELF_GRASP_AUTO_ONCE
        auto_attempt_succeeded = False
        while True:
            if SHELF_GRASP_AUTO_ONCE and not auto_attempt_pending:
                break
            chassis_state = "开" if USE_CHASSIS_PLACE else "关"
            target_state = target_query or "任意"
            if SHELF_GRASP_AUTO_ONCE:
                cmd = ""
                auto_attempt_pending = False
                print(f"\n[前端自动模式] 目标:{target_state}, 开始单次抓取")
            else:
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    _, layer_name = _shelf_layer_poses[_current_layer_idx]
                    prompt = f"\n[第{_current_layer_idx}层 {layer_name} | 目标:{target_state}] 按 Enter 抓取 | c 目标 | ? 类别 | l 层管理 | p 放置点 | u 收起 | b 底盘:{chassis_state} | v 预览 | j 点动 | q 退出 > "
                else:
                    prompt = f"\n[HOME | 目标:{target_state}] 按 Enter 抓取 | c 目标 | ? 类别 | l 层管理 | p 放置点 | u 收起 | b 底盘:{chassis_state} | v 预览 | j 点动 | q 退出 > "
                cmd = input(prompt).strip()
            if cmd.lower() == "q":
                break

            if cmd in ("?", "？") or cmd.lower() in ("classes", "class", "list"):
                _print_model_classes(locator)
                continue

            if cmd.lower() == "c" or cmd.lower().startswith("c "):
                raw = cmd[1:].strip() if len(cmd) > 1 else input("  目标商品/类别/id (空/all/clear=任意, ?=列类别): ").strip()
                if raw in ("?", "？") or raw.lower().startswith(("? ", "？ ")):
                    _print_model_classes(locator, raw[1:].strip())
                    continue
                target_query = "" if _is_clear_target_query(raw) else raw
                print(f"  [目标] 已设置为: {target_query or '任意'}")
                continue

            if cmd and _is_clear_target_query(cmd):
                target_query = ""
                print("  [目标] 已清空, 将按原策略抓取任意候选")
                continue

            if cmd.lower() == "j":
                step = 20  # mm
                j6_step = 5.0  # deg
                print(f"\n  点动模式 (位置步长 {step}mm, J6步长 {j6_step:.0f}°), 输入 q 退出")
                print(f"  wasd = XY, e/r = Z, z/x = J6 -/+")
                print(f"  =/+ 放大步长, - 缩小步长")
                while True:
                    jc = input("  点动 > ").strip().lower()
                    if jc == "q":
                        break

                    if jc in ("-", "=", "+"):
                        if jc == "-":
                            step = max(1, int(step / 1.5))
                            j6_step = max(1.0, j6_step / 1.5)
                        else:
                            step = min(100, int(step * 1.5))
                            j6_step = min(45.0, j6_step * 1.5)
                        print(f"  步长 → {step}mm / {j6_step:.0f}°")
                        continue

                    if jc in ("z", "x"):
                        try:
                            joints = arm.get_joints()
                        except Exception as e:
                            print(f"  无法获取关节角: {e}")
                            continue
                        dj6 = -j6_step if jc == "z" else j6_step
                        arm.move_j6(joints[5] + dj6)
                        time.sleep(MOTION_SETTLE_S)
                        continue

                    pose = arm.get_pose()
                    dx, dy, dz = 0, 0, 0
                    if jc == "a": dx = -step
                    elif jc == "d": dx = step
                    elif jc == "w": dy = step
                    elif jc == "s": dy = -step
                    elif jc == "e": dz = step
                    elif jc == "r": dz = -step
                    else:
                        print(f"  无效: {jc}")
                        continue
                    tx = pose[0] + dx
                    ty = pose[1] + dy
                    tz = pose[2] + dz
                    print(f"  MovL({tx:.0f},{ty:.0f},{tz:.0f}) ← ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}) + ({dx},{dy},{dz})")
                    resp = arm.movel(tx, ty, tz, speed=50, allow_low_z=_current_layer_allows_low_z())
                    time.sleep(MOTION_SETTLE_S)
                    continue

            if cmd.lower() == "l":
                # 货架层管理
                _layer_management(arm)
                continue

            if cmd.lower() == "p":
                _place_point_management(arm)
                continue

            if cmd.lower() == "u":
                _stow_pose_management(arm)
                continue

            if cmd.lower() == "b":
                _toggle_chassis_place()
                continue

            if cmd.lower() == "v":
                print("实时预览中… 按任意键退出预览")
                cv2.namedWindow("Live YOLO", cv2.WINDOW_NORMAL)
                cv2.destroyAllWindows()  # 清除旧窗口
                cv2.namedWindow("Live YOLO", cv2.WINDOW_NORMAL)
                while True:
                    color, depth = camera.read()
                    if color is None:
                        time.sleep(0.1)
                        continue
                    dets = locator.process_frame(color, depth)
                    viz = color.copy()
                    if dets:
                        for d in dets:
                            cv2.rectangle(viz, (int(d.bbox[0]), int(d.bbox[1])),
                                          (int(d.bbox[2]), int(d.bbox[3])), (0, 255, 0), 2)
                            cv2.putText(viz, f"{d.class_name} {d.depth_m or 0:.2f}m",
                                        (int(d.bbox[0]), int(d.bbox[1]) - 8),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow("Live YOLO", viz)
                    key = cv2.waitKey(30) & 0xFF
                    if key != 0xFF:
                        break
                cv2.destroyWindow("Live YOLO")
                print("预览结束")
                continue

            if cmd:
                target_query = cmd
                print(f"  [目标] 本次/后续按关键词抓取: {target_query}")

            # ── 获取当前法兰位姿 ──
            flange_pose = arm.get_pose()
            print(f"当前法兰: ({flange_pose[0]:.0f}, {flange_pose[1]:.0f}, {flange_pose[2]:.0f})mm "
                  f"rpy=({flange_pose[3]:.0f}, {flange_pose[4]:.0f}, {flange_pose[5]:.0f})")
            transformer.set_flange(flange_pose)

            # ── 多帧择优拍照：快速连拍 N 帧, 按光照评分选最佳 ──
            N_FRAMES = 10            # 采集帧数
            WARMUP = 2              # 前 N 帧丢弃 (等自动曝光稳定)
            best_img, best_depth = None, None
            best_score, best_grade = -1, ""
            best_idx = -1
            frame_detections = []
            captured_frames = []

            print(f"拍照中 ({N_FRAMES + WARMUP} 帧, 丢弃前 {WARMUP})...", end=" ", flush=True)
            for i in range(N_FRAMES + WARMUP):
                c, d = camera.read()
                if c is None:
                    continue
                if i < WARMUP:
                    continue
                score, grade, _, _, _, _ = _assess_lighting(c)
                if score > best_score:
                    best_score, best_grade = score, grade
                    best_img, best_depth = c, d
                    best_idx = i
                dets = locator.process_frame(c, d)
                frame_detections.append(dets)
                captured_frames.append((c, d))
                # 反光和过暗都有惩罚, 纯正常最高; 有反光也能选出相对最好的
            print(f"完成")
            print(f"  → 第 {best_idx} 帧 评分: {best_grade} ({best_score:.0f}分)")

            if best_img is None:
                print("相机未就绪")
                continue

            detections, tracks = _fuse_detection_tracks(frame_detections, DETECTION_STABILITY_IOU)
            if tracks:
                stable = sorted(
                    tracks,
                    key=lambda t: (len(t["items"]), max(item.confidence for _, item in t["items"])),
                    reverse=True,
                )[:6]
                stable_text = " | ".join(
                    f"{t['items'][0][1].class_name}:frames={len(t['items'])}/{N_FRAMES},"
                    f"max={max(item.confidence for _, item in t['items']):.2f}"
                    for t in stable
                )
                print(f"  → 多帧稳定候选: {stable_text}")
            cand_path = "/tmp/shelf_yolo_candidates.jpg"
            if locator.save_candidate_debug_image(best_img, cand_path):
                print(f"  → YOLO候选诊断图已保存: {cand_path} (绿=达阈值, 橙=低置信)")
            if not detections:
                print("未检测到盒子")
                # 保存原图排查
                cv2_path = "/tmp/shelf_no_detection.jpg"
                cv2.imwrite(cv2_path, best_img)
                print(f"  → 原图已保存: {cv2_path}")
                if locator.last_candidates:
                    top = sorted(locator.last_candidates, key=lambda item: item["confidence"], reverse=True)[:6]
                    cand_text = " | ".join(
                        f"{c['class_name']}:{c['confidence']:.2f}@{c['bbox']}" for c in top
                    )
                    print(f"  → 低置信候选Top: {cand_text}")
                continue

            # ── 选最靠边的盒子。先按当前层记录的世界 Z 过滤上下层，避免误抓上一层。 ──
            current_layer_z_m = None
            if _current_layer_idx in _shelf_layer_poses:
                current_layer_z_m = _shelf_layer_poses[_current_layer_idx][0][2] / 1000.0
            target_detections = _filter_detections_by_target_query(detections, target_query)
            _print_target_filter_result(target_query, detections, target_detections)
            if target_query and not target_detections:
                if SHELF_GRASP_AUTO_ONCE:
                    raise RuntimeError(f"前端自动模式未找到目标: {target_query}")
                continue
            target, edge = pick_target_box(
                target_detections,
                transformer=transformer,
                flange_pose=flange_pose,
                layer_z_m=current_layer_z_m,
            )
            if target is None:
                print("无法确定目标")
                if SHELF_GRASP_AUTO_ONCE:
                    raise RuntimeError("前端自动模式无法确定目标")
                continue
            target_tracks = [
                t for t in tracks
                if t["class_id"] == target.class_id and _bbox_iou(t["bbox"], target.bbox) >= DETECTION_STABILITY_IOU
            ]
            if target_tracks:
                best_track = max(
                    target_tracks,
                    key=lambda t: (len(t["items"]), max(item.confidence for _, item in t["items"])),
                )
                best_frame_idx, _ = max(best_track["items"], key=lambda item: item[1].confidence)
                if 0 <= best_frame_idx < len(captured_frames):
                    best_img, best_depth = captured_frames[best_frame_idx]
                    print(f"  → 使用目标最高置信帧作为抓取参考: frame={best_frame_idx}")
            target = refine_target_depth_on_frame(
                target,
                camera,
                best_depth,
                transformer,
                radius_px=TARGET_DEPTH_REFINE_RADIUS_PX,
                max_delta_m=TARGET_DEPTH_REFINE_MAX_DELTA_M,
            )
            print(f"\n=== 选中: {edge}侧盒子 bbox=({target.bbox[0]},{target.bbox[1]})-({target.bbox[2]},{target.bbox[3]}) "
                  f"depth={target.depth_m:.3f}m ===")

            # ── 计算抓取位姿 ──
            wx, wy, wz, grip_mode, p_cam, grip_pixel = plan_edge_grasp(
                target, edge, transformer, flange_pose, fx=camera.fx, camera=camera, depth_img=best_depth
            )
            print(f"抓取点 ({grip_mode}): ({wx:.3f}, {wy:.3f}, {wz:.3f})m")

            # 重新画正确的抓取点 (覆盖上面简单画的黄点)
            viz = best_img.copy()
            cv2.line(viz, (0, TARGET_ROI_Y_MIN), (viz.shape[1] - 1, TARGET_ROI_Y_MIN), (255, 0, 0), 1)
            cv2.line(viz, (0, TARGET_ROI_Y_MAX), (viz.shape[1] - 1, TARGET_ROI_Y_MAX), (255, 0, 0), 1)
            for d in detections:
                is_target_box = d.class_id == target.class_id and _bbox_iou(d.bbox, target.bbox) >= DETECTION_STABILITY_IOU
                col = (0, 0, 255) if is_target_box else (0, 255, 0)
                cv2.rectangle(viz, (int(d.bbox[0]), int(d.bbox[1])),
                              (int(d.bbox[2]), int(d.bbox[3])), col, 2)
                cv2.putText(viz, f"{d.class_name or '?'} {d.depth_m:.2f}m" if d.depth_m else f"{d.class_name or '?'}",
                            (int(d.bbox[0]), int(d.bbox[1]) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            # 目标抓取点: 使用规划模块返回的真实像素点，避免调试图仍显示旧的 bbox 中点。
            gx, gy = int(round(grip_pixel[0])), int(round(grip_pixel[1]))
            cv2.circle(viz, (gx, gy), 6, (0, 255, 255), -1)
            cv2.putText(viz, "GRIP", (gx + 8, gy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.imwrite("/tmp/shelf_detection.jpg", viz)
            print(f"  → 检测画面已保存: /tmp/shelf_detection.jpg  (蓝线=ROI, 绿框=检测, 红框=目标, 黄点=抓取点)")

            # ── 用法兰局部轴计算法兰/悬停/抓取位姿 (世界坐标, 米) ──
            R_f = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
            fz_dir = R_f[:, 2]   # 法兰+Z → 手指方向 (世界系)
            low_z_grasp = _current_layer_allows_low_z()
            grasp_advance_m = LOW_Z_GRASP_ADVANCE_M if low_z_grasp else GRASP_ADVANCE_M

            # ── 根据自动判定的抓取模式选择偏移参数 ──
            is_side_grasp = "side" in grip_mode
            grasp_offset_m = SIDE_GRASP_OFFSET_M if is_side_grasp else TOP_GRASP_OFFSET_M
            grip_label = "侧面抓" if is_side_grasp else "上面抓"
            fingertip_w = np.array([wx, wy, wz]) + np.array(grasp_offset_m)
            flange_pt   = fingertip_w - HAND_LENGTH * fz_dir   # 法兰在手背面

            # 抓取: 沿手指方向前进一小段探入
            grasp_pt = flange_pt + grasp_advance_m * fz_dir

            # 对齐: 固定退到抓取点后方, 避免从观察位保留过长探入距离。
            approach_pt = axis_aligned_approach_m(
                np.array(flange_pose[:3]) / 1000.0,
                grasp_pt,
                fz_dir,
                GRASP_APPROACH_BACKOFF_M,
            ) + np.array([0.0, 0.0, APPROACH_LIFT_M])

            # ── 转为 mm (python float) 供后续微调 ──
            fx, fy, fz  = float(flange_pt[0]*1000),  float(flange_pt[1]*1000),  float(flange_pt[2]*1000)
            ax, ay, az  = float(approach_pt[0]*1000), float(approach_pt[1]*1000), float(approach_pt[2]*1000)
            gx, gy, gz  = float(grasp_pt[0]*1000),    float(grasp_pt[1]*1000),    float(grasp_pt[2]*1000)

            depth_obstacles_m = np.zeros((0, 3), dtype=np.float32)
            depth_obstacles_px = np.zeros((0, 2), dtype=np.int32)
            depth_observation_flange_pose = tuple(flange_pose)
            depth_guard_camera_depth_axis = _camera_depth_axis_m(transformer, depth_observation_flange_pose)
            depth_guard_exclude_bboxes = _unique_bboxes(detections)
            if DEPTH_GUARD_ENABLE:
                depth_obstacles_m, depth_obstacles_px = depth_to_robot_obstacles(
                    best_depth,
                    camera,
                    transformer,
                    depth_observation_flange_pose,
                    stride=DEPTH_GUARD_STRIDE,
                    max_depth_m=DEPTH_GUARD_MAX_DEPTH_M,
                    exclude_bbox=None,
                    exclude_bboxes=depth_guard_exclude_bboxes,
                    exclude_margin_px=DEPTH_GUARD_EXCLUDE_MARGIN_PX,
                    exclude_self_hand=DEPTH_GUARD_SELF_MASK_ENABLE,
                    self_hand_dir_m=fz_dir,
                    self_hand_length_m=DEPTH_GUARD_SELF_MASK_LENGTH_M,
                    self_hand_radius_m=DEPTH_GUARD_SELF_MASK_RADIUS_M,
                    self_hand_backoff_m=DEPTH_GUARD_SELF_MASK_BACKOFF_M,
                    return_pixels=True,
                )
                print(
                    f"  [深度护栏] 障碍点={len(depth_obstacles_m)} "
                    f"(stride={DEPTH_GUARD_STRIDE}, max_depth={DEPTH_GUARD_MAX_DEPTH_M:.2f}m, "
                    f"排除框={len(depth_guard_exclude_bboxes)}, 外扩={DEPTH_GUARD_EXCLUDE_MARGIN_PX}px, "
                    f"自手部={'on' if DEPTH_GUARD_SELF_MASK_ENABLE else 'off'} "
                    f"r={DEPTH_GUARD_SELF_MASK_RADIUS_M*1000:.0f}mm, "
                    f"深度对齐={'on' if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else 'off'} "
                    f"±{DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M*1000:.0f}mm)"
                )

            # ========================================================
            # 调试交互: 微调 / 预览 / 执行  (offset 均为 mm)
            # ========================================================
            offset_x, offset_y, offset_z = 0.0, 0.0, 0.0
            jog_step_mm = 5.0

            while True:
                 tuned_offset_m = np.array(grasp_offset_m) + np.array([offset_x, offset_y, offset_z]) / 1000.0
                 print(f"\n── 目标位姿 (可微调) ──")
                 print(f"  模式: {grip_label} ({grip_mode})")
                 print(f"  当前参数: offset=({grasp_offset_m[0]*1000:.0f},{grasp_offset_m[1]*1000:.0f},{grasp_offset_m[2]*1000:.0f})mm")
                 print(f"  固化建议: {'SIDE' if is_side_grasp else 'TOP'}_GRASP_OFFSET_M=({tuned_offset_m[0]:+.3f}, {tuned_offset_m[1]:+.3f}, {tuned_offset_m[2]:+.3f})")
                 approach_to_grasp_mm = np.linalg.norm(
                     np.array([gx + offset_x - (ax + offset_x), gy + offset_y - (ay + offset_y), gz + offset_z - (az + offset_z)])
                 )
                 print(f"  对齐→抓取直线: {approach_to_grasp_mm:.0f}mm, 过抓前进: {grasp_advance_m*1000:.0f}mm" + (" (第4层低位)" if low_z_grasp else ""))
                 print(f"  指尖世界: ({wx*1000:.0f}, {wy*1000:.0f}, {wz*1000:.0f})mm")
                 print(f"  法兰(对齐): X={ax+offset_x:+.0f}({offset_x:+5.0f})  "
                       f"Y={ay+offset_y:+.0f}({offset_y:+5.0f})  "
                       f"Z={az+offset_z:+.0f}({offset_z:+5.0f}) mm")
                 print(f"  法兰(抓取): X={gx+offset_x:+.0f}  Y={gy+offset_y:+.0f}  Z={gz+offset_z:+.0f} mm")
                 print(f"\n  操作:")
                 print(f"    Enter ← 执行抓取")
                 print(f"    p     ← 预览对齐位 (不抓取)")
                 print(f"    t     ← 相对移动 (含标定位移)")
                 print(f"    m     ← 简化移动 (零标定: 沿法兰Z前移)")
                 print(f"    wasd  ← 微调 X/Y (±{jog_step_mm:.0f}mm)")
                 print(f"    e/r   ← 微调 Z (±{jog_step_mm:.0f}mm)")
                 print(f"    = / - ← 改变步长 (当前 {jog_step_mm:.0f}mm)")
                 print(f"    f     ← 手动输入偏移 (格式: x,y,z mm)")
                 print(f"    F5    ← 重新拍照 (复位偏移)")
                 print(f"    Q     ← 跳过/退出")
                 if SHELF_GRASP_AUTO_ONCE:
                     choice = ""
                     print("  [前端自动模式] 自动确认目标位姿")
                 else:
                     choice = input("  > ").strip()

                 if choice == "":
                     break  # 执行
                 elif choice == "Q":
                     return  # 退出主循环
                 elif choice.lower() == "q" and len(choice) > 1:
                     return  # 退出
                 elif choice == "Q" or choice == "q":
                     break  # 跳过当前, 回到外层
                 elif choice == "t":
                     # 相对移动: 用标定 T_flange→cam, 跳过基座变换
                     p_cam_h = np.append(p_cam, 1.0)
                     p_flange = transformer.T_flange_to_cam @ p_cam_h  # 盒在法兰系
                     delta = p_flange[:3] + np.array([0, 0, HAND_LENGTH + 0.04])  # +手长+悬停
                     print(f"\n  法兰系偏移: ({delta[0]*1000:.0f}, {delta[1]*1000:.0f}, {delta[2]*1000:.0f})mm")
                     R = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
                     delta_w = R @ delta
                     new_x = flange_pose[0] + delta_w[0] * 1000
                     new_y = flange_pose[1] + delta_w[1] * 1000
                     new_z = flange_pose[2] + delta_w[2] * 1000
                     print(f"  目标法兰: ({new_x:.0f}, {new_y:.0f}, {new_z:.0f})mm")
                     arm.movel(new_x, new_y, new_z, speed=100, allow_low_z=_current_layer_allows_low_z())
                     time.sleep(MOTION_SETTLE_S)
                     continue
                 elif choice == "m":
                     # 简化模式: 沿法兰Z前移
                     dist = np.linalg.norm(p_cam)
                     delta = np.array([0, 0, dist + HAND_LENGTH + 0.04])
                     print(f"\n  [简化模式] 盒距={dist:.3f}m → ΔZ={delta[2]*1000:.0f}mm")
                     R = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
                     delta_w = R @ delta
                     new_x = flange_pose[0] + delta_w[0] * 1000
                     new_y = flange_pose[1] + delta_w[1] * 1000
                     new_z = flange_pose[2] + delta_w[2] * 1000
                     print(f"  目标法兰: ({new_x:.0f}, {new_y:.0f}, {new_z:.0f})mm")
                     arm.movel(new_x, new_y, new_z, speed=60, allow_low_z=_current_layer_allows_low_z())
                     time.sleep(MOTION_SETTLE_S)
                     continue
                 elif choice == "p":
                     print(f"\n  预览对齐位: 移动到 ({ax+offset_x:.0f}, {ay+offset_y:.0f}, {az+offset_z:.0f})mm")
                     if not _movel_with_depth_guard(
                         arm,
                         "预览对齐路径",
                         depth_obstacles_m,
                         (ax + offset_x, ay + offset_y, az + offset_z),
                         fz_dir,
                         speed=100,
                         allow_low_z=_current_layer_allows_low_z(),
                         allow_bypass=True,
                         hand_length_m=DEPTH_GUARD_APPROACH_LENGTH_M,
                         viz_image=best_img,
                         obstacle_pixels=depth_obstacles_px,
                         target_bbox=target.bbox,
                         exclude_bboxes=depth_guard_exclude_bboxes,
                         transformer=transformer,
                         observation_flange_pose=depth_observation_flange_pose,
                         camera_depth_axis_m=depth_guard_camera_depth_axis,
                     ):
                         continue
                     time.sleep(MOTION_SETTLE_S)
                     continue
                 elif choice.lower() == "f5":
                     offset_x = offset_y = offset_z = 0.0
                     break
                 elif choice in ("w", "s", "a", "d"):
                     dx = {"a": -jog_step_mm, "d": +jog_step_mm}.get(choice, 0)
                     dy = {"w": -jog_step_mm, "s": +jog_step_mm}.get(choice, 0)
                     offset_x += dx
                     offset_y += dy
                 elif choice == "e":
                     offset_z += jog_step_mm
                 elif choice == "r":
                     offset_z -= jog_step_mm
                 elif choice in ("-", "=", "+"):
                     if choice == "-":
                         jog_step_mm = max(1, jog_step_mm / 1.5)
                     else:
                         jog_step_mm = min(50, jog_step_mm * 1.5)
                     print(f"  步长 → {jog_step_mm:.0f}mm")
                     continue
                 elif choice == "f":
                     val = input("  输入偏移 (x,y,z mm, 如 10,-5,0): ").strip()
                     try:
                         parts = [float(v.strip()) for v in val.split(",")]
                         if len(parts) == 3:
                             offset_x, offset_y, offset_z = parts
                         else:
                             print("  格式错误, 需要 x,y,z")
                     except ValueError:
                         print("  格式错误")
            if choice.lower() == "f5":
                continue
            if choice in ("Q", "q"):
                break

            # ========================================================
            # ── 执行 ──
            print("\n执行抓取动作...")

            # 1. 张开手
            if not SKIP_HAND:
                hand.open()

            # 上面抓取: 先把第六轴转到手心向下的实测角度
            if not is_side_grasp:
                print(f"  上面抓取 → J6 转到手心向下角度 {TOP_J6_DEG:.1f}°")
                arm.move_j6(TOP_J6_DEG)
                time.sleep(MOTION_SETTLE_S)
                flange_pose = arm.get_pose()
                R_f = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
                fz_dir = R_f[:, 2]
                flange_pt = fingertip_w - HAND_LENGTH * fz_dir
                grasp_pt = flange_pt + grasp_advance_m * fz_dir
                approach_pt = axis_aligned_approach_m(
                    np.array(flange_pose[:3]) / 1000.0,
                    grasp_pt,
                    fz_dir,
                    TOP_APPROACH_BACKOFF_M,
                ) + np.array([0.0, 0.0, APPROACH_LIFT_M])
                ax, ay, az = float(approach_pt[0]*1000), float(approach_pt[1]*1000), float(approach_pt[2]*1000)
                gx, gy, gz = float(grasp_pt[0]*1000), float(grasp_pt[1]*1000), float(grasp_pt[2]*1000)

            # 2. 对齐位: 先对齐上下左右, 不沿前后方向探入
            allow_low_z = _current_layer_allows_low_z()
            approach_speed = LOW_Z_APPROACH_SPEED if allow_low_z else 150
            grasp_speed = LOW_Z_GRASP_SPEED if allow_low_z else 50
            if allow_low_z:
                print(f"  第4层低位抓取: 探入 {grasp_advance_m*1000:.0f}mm, 速度 {grasp_speed}")
            if not _movel_with_depth_guard(
                arm,
                "对齐路径",
                depth_obstacles_m,
                (ax + offset_x, ay + offset_y, az + offset_z),
                fz_dir,
                speed=approach_speed,
                allow_low_z=allow_low_z,
                allow_bypass=True,
                hand_length_m=DEPTH_GUARD_APPROACH_LENGTH_M,
                viz_image=best_img,
                obstacle_pixels=depth_obstacles_px,
                target_bbox=target.bbox,
                exclude_bboxes=depth_guard_exclude_bboxes,
                transformer=transformer,
                observation_flange_pose=depth_observation_flange_pose,
                camera_depth_axis_m=depth_guard_camera_depth_axis,
            ):
                print("  [中止] 对齐位运动失败/报警, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                continue
            time.sleep(GRASP_SETTLE_S)

            # 3. 前进抓取 (沿手指方向探入 GRASP_ADVANCE_M)
            depth_axis_min_m = -DEPTH_GUARD_FLANGE_BACK_MARGIN_M if DEPTH_GUARD_AXIS_WINDOW_ENABLE else None
            depth_axis_max_m = HAND_LENGTH + DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M if DEPTH_GUARD_AXIS_WINDOW_ENABLE else None
            if not _movel_with_depth_guard(
                arm,
                "探入路径",
                depth_obstacles_m,
                (gx + offset_x, gy + offset_y, gz + offset_z),
                fz_dir,
                speed=grasp_speed,
                allow_low_z=allow_low_z,
                allow_bypass=False,
                hand_length_m=HAND_LENGTH,
                viz_image=best_img,
                obstacle_pixels=depth_obstacles_px,
                target_bbox=target.bbox,
                exclude_bboxes=depth_guard_exclude_bboxes,
                transformer=transformer,
                observation_flange_pose=depth_observation_flange_pose,
                camera_depth_axis_m=depth_guard_camera_depth_axis,
                axis_min_m=depth_axis_min_m,
                axis_max_m=depth_axis_max_m,
            ):
                print("  [中止] 深度护栏拒绝探入路径, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                continue
            time.sleep(GRASP_SETTLE_S)

            # 4. 抓取 (动作二) 并等待 5s 握紧
            if not SKIP_HAND:
                grasp_mode = "bottle" if HAND_GRASP_MODE == "bottle" and is_side_grasp else None
                if hand.close(top=not is_side_grasp, mode=grasp_mode):
                    print(f"  灵巧手 → 闭合, 等待 {HAND_CLOSE_WAIT_S:.1f}s ...")
                    time.sleep(HAND_CLOSE_WAIT_S)
                else:
                    print("  灵巧手不可用, 继续执行机械臂安全退回")

            # 5. 后退: 先沿手指方向退回对齐位, 再离开货架
            retreat_pt = np.array([ax+offset_x, ay+offset_y, az+offset_z])
            if not arm.movel(float(retreat_pt[0]), float(retreat_pt[1]), float(retreat_pt[2]), speed=100,
                             allow_low_z=allow_low_z):
                print("  [停止] 后退失败/报警, 停在当前位置, 不继续放置流程")
                continue
            time.sleep(GRASP_SETTLE_S)

            if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                if not go_to_layer(arm, _current_layer_idx):
                    print("  [停止] 回观察位失败/报警, 停在当前位置, 不继续放置流程")
                    continue
            else:
                arm.go_home()

            # 6. 放置并回到当前层观察位
            if not finish_pick_cycle(arm, hand, _current_layer_idx):
                print("  [停止] 放置/回层失败, 本次抓取未完成")
                continue
            time.sleep(MOTION_SETTLE_S)
            auto_attempt_succeeded = True

            print("抓取完成! 按 Enter 继续 / q 退出")
            if SHELF_GRASP_AUTO_ONCE:
                break

        if SHELF_GRASP_AUTO_ONCE and not auto_attempt_succeeded:
            raise RuntimeError("前端自动模式单次抓取未完成")

    except KeyboardInterrupt:
        pass
    finally:
        if SHELF_GRASP_AUTO_ONCE:
            print("[前端自动模式] 退出时不自动收起, 保持当前安全观察位/当前位置")
        else:
            try:
                go_to_stow_pose(arm)
            except Exception as e:
                print(f"[收起姿态] 退出前收起失败/跳过: {e}")
        if not SKIP_HAND:
            hand.disconnect()
        arm.close()
        camera.release()
        print("\n已退出")


if __name__ == "__main__":
    main()
