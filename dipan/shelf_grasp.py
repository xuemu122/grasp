#!/usr/bin/env python3
"""
货架边缘抓取系统
- 统一入口: 识别、抓取、放置、底盘和菜单流程编排
- 硬件控制/手眼变换/抓取规划拆到独立模块，便于现场调参
"""

import atexit
import sys
import os
import time
import json
import subprocess
import re
import cv2
import numpy as np
from collections import Counter
from typing import Optional

# ── 识别模块：按仓库相对路径加载，避免部署目录变化后入口失效 ──
VISION_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "识别"))
if VISION_DIR not in sys.path:
    sys.path.insert(0, VISION_DIR)
from orbbec_detect import MedicineLocatorSystem, RealSenseCamera
from orbbec_detect import _assess_lighting

from dobot_arm import DobotArm, _clamp_cp
from grasp_planner import (
    EDGE_MIDPOINT_GRASP_ENABLE,
    FORCE_SIDE_GRASP,
    SIDE_GRASP_MAX_HEIGHT_M,
    SIDE_GRASP_MIN_HEIGHT_M,
    SIDE_GRASP_LOWER_SHIFT_M,
    TARGET_ROI_Y_MIN,
    TARGET_ROI_Y_MAX,
    axis_aligned_approach_m,
    clamp_side_grasp_pixel_to_safe_clearance,
    detect_shelf_board_bounds,
    explain_target_filter,
    pick_target_box,
    plan_edge_grasp,
)
from hand_eye import FX, FY, HandEyeTransformer
from depth_collision_guard import depth_to_robot_obstacles, check_hand_path, hand_path_hit_mask
from tool_geometry import (
    HAND_BODY_DEPTH_X_M,
    HAND_LENGTH_Z_M,
    HAND_PALM_LENGTH_Z_M,
    HAND_PALM_WIDTH_Y_M,
    HAND_THUMB_Z_MAX_M,
    TOOL_HAND_MODEL,
    camera_box_bounds,
    camera_size_label_mm,
    hand_box_bounds,
)
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


def _shrink_box_bounds(boxes, shrink_m):
    """Inset depth-guard boxes without changing shared tool geometry."""
    shrink = max(0.0, float(shrink_m))
    if shrink <= 0.0:
        return tuple(boxes)
    result = []
    for box_min, box_max in boxes:
        lo = np.asarray(box_min, dtype=np.float32)
        hi = np.asarray(box_max, dtype=np.float32)
        center = (lo + hi) * 0.5
        half = np.maximum((hi - lo) * 0.5 - shrink, 0.001)
        result.append((tuple(center - half), tuple(center + half)))
    return tuple(result)


# ============================================================
# 参数
# ============================================================
HAND_LENGTH = HAND_LENGTH_Z_M      # 法兰 → 指尖 (米)
HAND_PALM_LENGTH_M = HAND_PALM_LENGTH_Z_M  # 法兰 → 掌心前端 (米)
HAND_FINGER_THICKNESS_M = float(os.getenv("HAND_FINGER_THICKNESS_M", "0.03"))  # 手指厚度 (米)
HAND_WEIGHT = float(os.getenv("HAND_WEIGHT_KG", "0.6"))
HAND_W_Y = HAND_PALM_WIDTH_Y_M     # 手 Y 方向宽度 (米)
HAND_W_X = HAND_BODY_DEPTH_X_M     # 手 X 方向宽度 (米)

# 抓取参数 (基座系, 米): -X=前 +Y=右 +Z=上
# 侧面抓: 盒宽 <= 8cm, grip_mode 为 left_side/right_side
SIDE_GRASP_OFFSET_M = (-0.12, 0.008, 0.008)
# 上面抓: 盒宽 > 8cm, grip_mode 为 top_center。先沿用侧抓值, 实测后单独固化。
TOP_GRASP_OFFSET_M = (-0.12, 0.015, 0.035)

# 上面抓取前把第六轴转到手心向下的实测角度
TOP_J6_DEG = 8.3

# 抓取执行参数: 对齐位 -> 沿手指方向前进抓取 -> 后退。
APPROACH_LIFT_M = 0.00    # 对齐位额外抬高量, 当前不抬高
GRASP_ADVANCE_M = 0.01    # 抓取时沿手指方向前进 4cm
LOW_Z_GRASP_ADVANCE_M = 0.035  # 第4层低位空间更紧, 只探入 3.5cm
PRE_CLOSE_LEFT_SHIFT_M = float(os.getenv("PRE_CLOSE_LEFT_SHIFT_M", "0.00"))
GRASP_APPROACH_BACKOFF_M = float(os.getenv("GRASP_APPROACH_BACKOFF_M", "0.15"))  # 抓取前沿手指方向后退对齐
TOP_APPROACH_BACKOFF_M = 0.15  # 上面抓时先在抓取轴线上退 15cm 再直线前进
LOW_Z_APPROACH_SPEED = 60
LOW_Z_GRASP_SPEED = 20
MOTION_SETTLE_S = float(os.getenv("MOTION_SETTLE_S", "0.2"))  # 运动完成后的短缓冲
GRASP_SETTLE_S = float(os.getenv("GRASP_SETTLE_S", "0.4"))    # 抓取前进/后退短缓冲
# LHandPro/Aero start their hold loop before returning from close; only keep a
# short confirmation window here so the arm does not pause for a full second.
HAND_CLOSE_WAIT_S = float(os.getenv("HAND_CLOSE_WAIT_S", "0.25"))
# The dexterous hand takes longer to physically release an item than the
# generic motion settle time; keep the arm at the place point while it opens.
HAND_OPEN_WAIT_S = float(os.getenv("HAND_OPEN_WAIT_S", "2.0"))
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
# 当前默认使用电流反馈柔性抓取；原侧抓/上抓固定手势仍可通过 HAND_GRASP_MODE=bottle/auto 保留。
HAND_GRASP_MODE = os.getenv("HAND_GRASP_MODE", "flex").lower()
HAND_BACKEND = os.getenv(
    "HAND_BACKEND",
    "aero" if TOOL_HAND_MODEL == "aero" else "lhandpro_dobot",
).strip().lower()
LHAND_DOBOT_BAUD = int(os.getenv("LHAND_DOBOT_BAUD", "115200"))
LHAND_RS485_NODE_ID = int(os.getenv("LHAND_RS485_NODE_ID", "1"))
LHAND_DOBOT_CONTROL = os.getenv("LHAND_DOBOT_CONTROL", "compact").strip().lower()
LHAND_FLEX_PROFILE = os.path.abspath(os.path.expanduser(os.getenv(
    "LHAND_FLEX_PROFILE",
    os.path.join(os.path.dirname(__file__), "runtime", "lhandpro_empty.csv"),
)))
LHAND_DEBUG = os.getenv("LHAND_DEBUG", "0").lower() in ("1", "true", "yes", "on")
LHAND_HEX_DEBUG = os.getenv("LHAND_HEX_DEBUG", "0").lower() in ("1", "true", "yes", "on")
DETECTION_STABILITY_IOU = float(os.getenv("DETECTION_STABILITY_IOU", "0.35"))
DETECTION_CAPTURE_FRAMES = max(1, int(os.getenv("DETECTION_CAPTURE_FRAMES", "10")))
DETECTION_WARMUP_FRAMES = max(0, int(os.getenv("DETECTION_WARMUP_FRAMES", "2")))
DETECTION_EARLY_STOP_FRAMES = max(0, int(os.getenv("DETECTION_EARLY_STOP_FRAMES", "6")))
DETECTION_EARLY_STOP_SUPPORT = max(1, int(os.getenv("DETECTION_EARLY_STOP_SUPPORT", "5")))
TARGET_PRODUCT_QUERY = os.getenv("TARGET_PRODUCT_QUERY", os.getenv("TARGET_NAME", "")).strip()
SHELF_GRASP_AUTO_ONCE = os.getenv("SHELF_GRASP_AUTO_ONCE", "0").lower() in ("1", "true", "yes", "on")
SHELF_GRASP_WORKER = os.getenv("SHELF_GRASP_WORKER", "0").lower() in ("1", "true", "yes", "on")
SHELF_GRASP_IDLE_STARTUP = os.getenv("SHELF_GRASP_IDLE_STARTUP", "0").lower() in ("1", "true", "yes", "on")
SAVE_DEBUG_IMAGES = os.getenv("SAVE_DEBUG_IMAGES", "0").lower() in ("1", "true", "yes", "on")
FAILURE_DIAGNOSTIC_PATH = os.getenv("SHELF_FAILURE_DIAGNOSTIC_PATH", "/tmp/shelf_failure_latest.jpg")
# 弱光增强前后对比图：抓取失败时一并输出，便于核对补光是否提升召回
ENHANCE_COMPARE_PATH = os.getenv(
    "SHELF_ENHANCE_COMPARE_PATH",
    os.path.join(os.path.dirname(FAILURE_DIAGNOSTIC_PATH) or ".", "shelf_enhance_compare_latest.jpg"),
)
LAYER_ENTRY_SHELF = "shelf"
LAYER_ENTRY_STOW = "stow"
LAYER_ENTRY_STARTUP = "startup"
LAYER_ENTRY_UNKNOWN = "unknown"
TARGET_DEPTH_REFINE_RADIUS_PX = int(os.getenv("TARGET_DEPTH_REFINE_RADIUS_PX", "10"))
TARGET_DEPTH_REFINE_MAX_DELTA_M = float(os.getenv("TARGET_DEPTH_REFINE_MAX_DELTA_M", "0.08"))
DEPTH_GUARD_ENABLE = os.getenv("DEPTH_GUARD_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_STRIDE = int(os.getenv("DEPTH_GUARD_STRIDE", "8"))
DEPTH_GUARD_CLEARANCE_M = float(os.getenv("DEPTH_GUARD_CLEARANCE_M", "0.002"))
DEPTH_GUARD_HAND_BOX_SHRINK_M = float(os.getenv("DEPTH_GUARD_HAND_BOX_SHRINK_M", "0.005"))
DEPTH_GUARD_CAMERA_ENABLE = os.getenv("DEPTH_GUARD_CAMERA_ENABLE", "0").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_CAMERA_CLEARANCE_M = float(os.getenv("DEPTH_GUARD_CAMERA_CLEARANCE_M", str(DEPTH_GUARD_CLEARANCE_M)))
DEPTH_GUARD_SELF_MASK_CAMERA_PADDING_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_CAMERA_PADDING_M", "0.01"))
DEPTH_GUARD_CAMERA_BOX_MIN_M, DEPTH_GUARD_CAMERA_BOX_MAX_M = camera_box_bounds(
    DEPTH_GUARD_CAMERA_CLEARANCE_M
)
DEPTH_GUARD_SELF_CAMERA_BOX_MIN_M, DEPTH_GUARD_SELF_CAMERA_BOX_MAX_M = camera_box_bounds(
    DEPTH_GUARD_SELF_MASK_CAMERA_PADDING_M
)
DEPTH_GUARD_HAND_RADIUS_M = float(os.getenv("DEPTH_GUARD_HAND_RADIUS_M", str(HAND_W_Y * 0.5 + DEPTH_GUARD_CLEARANCE_M)))
DEPTH_GUARD_FINGER_RADIUS_M = float(os.getenv("DEPTH_GUARD_FINGER_RADIUS_M", str(HAND_FINGER_THICKNESS_M * 0.5 + DEPTH_GUARD_CLEARANCE_M)))
DEPTH_GUARD_HAND_BOXES_M = _shrink_box_bounds(
    hand_box_bounds(DEPTH_GUARD_CLEARANCE_M),
    DEPTH_GUARD_HAND_BOX_SHRINK_M,
)
DEPTH_GUARD_SELF_HAND_BOXES_M = hand_box_bounds(DEPTH_GUARD_SELF_MASK_CAMERA_PADDING_M)
DEPTH_GUARD_APPROACH_LENGTH_M = float(os.getenv("DEPTH_GUARD_APPROACH_LENGTH_M", "0.08"))
DEPTH_GUARD_PATH_SAMPLES = int(os.getenv("DEPTH_GUARD_PATH_SAMPLES", "8"))
# Sparse shelf-edge/depth-boundary samples are common near the bottom of the
# image. Require a small point cluster before stopping the arm so one sampled
# edge pair does not cause a false alarm.
DEPTH_GUARD_MIN_HIT_POINTS = max(1, int(os.getenv("DEPTH_GUARD_MIN_HIT_POINTS", "4")))
DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS = max(1, int(os.getenv("DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS", "16")))
DEPTH_GUARD_CAMERA_MIN_HIT_POINTS = max(1, int(os.getenv("DEPTH_GUARD_CAMERA_MIN_HIT_POINTS", "12")))
DEPTH_GUARD_NOISE_FILTER_RADIUS_PX = max(0, int(os.getenv("DEPTH_GUARD_NOISE_FILTER_RADIUS_PX", "2")))
DEPTH_GUARD_NOISE_FILTER_MIN_SUPPORT = max(1, int(os.getenv("DEPTH_GUARD_NOISE_FILTER_MIN_SUPPORT", "4")))
DEPTH_GUARD_NOISE_FILTER_TOLERANCE_M = float(os.getenv("DEPTH_GUARD_NOISE_FILTER_TOLERANCE_M", "0.025"))
DEPTH_GUARD_MAX_DEPTH_M = float(os.getenv("DEPTH_GUARD_MAX_DEPTH_M", "1.20"))
DEPTH_GUARD_TARGET_DEPTH_MARGIN_M = float(os.getenv("DEPTH_GUARD_TARGET_DEPTH_MARGIN_M", "0.025"))
DEPTH_GUARD_SHELF_ROW_INSET_PX = max(0, int(os.getenv("DEPTH_GUARD_SHELF_ROW_INSET_PX", "8")))
DEPTH_GUARD_TARGET_ROW_TOP_MARGIN_PX = max(0, int(os.getenv("DEPTH_GUARD_TARGET_ROW_TOP_MARGIN_PX", "32")))
# In fallback mode the target box bottom is the last trustworthy product row.
# Extending it downward admits the shelf lip immediately below a bottle when
# board detection has no valid upper/lower pair. Dynamic board bounds still
# use their own inset below.
DEPTH_GUARD_TARGET_ROW_BOTTOM_MARGIN_PX = max(0, int(os.getenv("DEPTH_GUARD_TARGET_ROW_BOTTOM_MARGIN_PX", "0")))
DEPTH_GUARD_EXCLUDE_MARGIN_PX = int(os.getenv("DEPTH_GUARD_EXCLUDE_MARGIN_PX", "32"))
DEPTH_GUARD_SELF_MASK_ENABLE = os.getenv("DEPTH_GUARD_SELF_MASK_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_SELF_MASK_RADIUS_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_RADIUS_M", str(DEPTH_GUARD_HAND_RADIUS_M + 0.02)))
DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M = float(os.getenv(
    "DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M",
    str(max(DEPTH_GUARD_FINGER_RADIUS_M + 0.02, DEPTH_GUARD_SELF_MASK_RADIUS_M)),
))
DEPTH_GUARD_SELF_MASK_LENGTH_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_LENGTH_M", str(HAND_LENGTH)))
DEPTH_GUARD_SELF_MASK_BACKOFF_M = float(os.getenv("DEPTH_GUARD_SELF_MASK_BACKOFF_M", "0.03"))
DEPTH_GUARD_DEPTH_ALIGN_ENABLE = os.getenv("DEPTH_GUARD_DEPTH_ALIGN_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M = float(os.getenv("DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M", "0.025"))
DEPTH_GUARD_AXIS_WINDOW_ENABLE = os.getenv("DEPTH_GUARD_AXIS_WINDOW_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M = float(os.getenv("DEPTH_GUARD_FINGERTIP_FRONT_MARGIN_M", "0.005"))
DEPTH_GUARD_FLANGE_BACK_MARGIN_M = float(os.getenv("DEPTH_GUARD_FLANGE_BACK_MARGIN_M", "0.03"))
DEPTH_GUARD_BYPASS_ENABLE = os.getenv("DEPTH_GUARD_BYPASS_ENABLE", "1").lower() in ("1", "true", "yes", "on")
DEPTH_GUARD_BYPASS_LIFTS_MM = _parse_float_list_env("DEPTH_GUARD_BYPASS_LIFTS_MM", (60, 100, 140))
DEPTH_GUARD_BYPASS_SIDE_MM = _parse_float_list_env("DEPTH_GUARD_BYPASS_SIDE_MM", (60, -60))

# 货架层位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
SHELF_LAYERS_FILE = os.path.join(os.path.dirname(__file__), "shelf_layers.json")
STARTUP_LAYER_IDX = 2   # 程序启动固定回第2层, 避免从上次低层状态恢复导致碰撞
_current_layer_idx = STARTUP_LAYER_IDX  # 当前选中的层编号
_shelf_layer_poses = {}  # {1: ([x,y,z,rx,ry,rz], name), 2: ...}
LOW_Z_LAYER_IDXS = {4, 5}  # 第4/5层在抬高基座下方, 允许低/负 Z

# 放置点位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
PLACE_POINT_FILE = os.path.join(os.path.dirname(__file__), "place_point.json")
_place_point_pose = None  # ([x,y,z,rx,ry,rz], name)
_place_point_joints = None  # [j1,j2,j3,j4,j5,j6]，可选
PLACE_TRANSITION_FILE = os.path.join(os.path.dirname(__file__), "place_transition.json")
_place_transition_pose = None  # ([x,y,z,rx,ry,rz], name)
_place_transition_joints = None  # [j1,j2,j3,j4,j5,j6]，可选
PLACE_TRANSITION_LOW_FILE = os.path.join(os.path.dirname(__file__), "place_transition_low.json")
_place_transition_low_pose = None  # ([x,y,z,rx,ry,rz], name) 低位进层过渡点
_place_transition_low_joints = None  # [j1,j2,j3,j4,j5,j6]，可选

# 收起姿态: 调用底盘前和退出程序前使用, 防止机械臂/灵巧手与环境碰撞
STOW_POSE_FILE = os.path.join(os.path.dirname(__file__), "stow_pose.json")
_stow_pose = None  # ([x,y,z,rx,ry,rz], name)
SAFE_NAV_STARTUP = os.getenv("SHELF_GRASP_SAFE_NAV_STARTUP", "0").lower() in ("1", "true", "yes", "on")
STOW_VERIFY_POSITION_TOLERANCE_MM = float(os.getenv("STOW_VERIFY_POSITION_TOLERANCE_MM", "35"))
STOW_VERIFY_ANGLE_TOLERANCE_DEG = float(os.getenv("STOW_VERIFY_ANGLE_TOLERANCE_DEG", "8"))

# 放置前是否先让底盘去固定点。默认关闭, 现场确认 marker 后可用环境变量或主菜单 b 开启。
USE_CHASSIS_PLACE = os.getenv("USE_CHASSIS_PLACE", "0").lower() in ("1", "true", "yes", "on")
MOVE_ROBOT_SCRIPT = os.path.join(os.path.dirname(__file__), "move_robot.py")
CHASSIS_SETTINGS_FILE = os.getenv(
    "CHASSIS_SETTINGS_FILE",
    os.path.join(os.path.dirname(__file__), "..", "qianduan", "data", "chassis_settings.json"),
)
DEFAULT_CHASSIS_PLACE_MARKER = os.getenv("CHASSIS_PLACE_MARKER", os.getenv("MARKER_PLACE", "fangzhi"))


def _layer_allows_low_z(layer_idx: Optional[int]) -> bool:
    return layer_idx in LOW_Z_LAYER_IDXS


def _current_layer_allows_low_z() -> bool:
    return _layer_allows_low_z(_current_layer_idx)


def _depth_guard_hand_radius_profile(hand_length_m=HAND_LENGTH, self_mask=False):
    length = max(0.0, float(hand_length_m))
    palm_len = max(0.0, min(float(max(HAND_PALM_LENGTH_M, HAND_THUMB_Z_MAX_M)), length))
    palm_radius = DEPTH_GUARD_SELF_MASK_RADIUS_M if self_mask else DEPTH_GUARD_HAND_RADIUS_M
    finger_radius = DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M if self_mask else DEPTH_GUARD_FINGER_RADIUS_M
    profile = []
    if palm_len > 1e-6:
        profile.append((0.0, palm_len, palm_radius))
    if length > palm_len + 1e-6:
        profile.append((palm_len, length, finger_radius))
    return profile


def _depth_guard_radius_label():
    hand = (
        f"掌{DEPTH_GUARD_HAND_RADIUS_M*1000:.0f}mm/"
        f"指{DEPTH_GUARD_FINGER_RADIUS_M*1000:.0f}mm"
    )
    if DEPTH_GUARD_CAMERA_ENABLE:
        return f"{hand}/相机{camera_size_label_mm()}"
    return hand


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
    flange_rotation_m=None,
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
        hand_radius_profile_m=_depth_guard_hand_radius_profile(hand_length_m),
        samples=DEPTH_GUARD_PATH_SAMPLES,
        camera_depth_axis_m=camera_depth_axis_m if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        camera_depth_tolerance_m=DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        axis_min_m=axis_min_m,
        axis_max_m=axis_max_m,
        flange_rotation_m=flange_rotation_m,
        camera_box_min_local_m=DEPTH_GUARD_CAMERA_BOX_MIN_M if DEPTH_GUARD_CAMERA_ENABLE else None,
        camera_box_max_local_m=DEPTH_GUARD_CAMERA_BOX_MAX_M if DEPTH_GUARD_CAMERA_ENABLE else None,
        hand_boxes_local_m=DEPTH_GUARD_HAND_BOXES_M,
        min_hit_points=DEPTH_GUARD_MIN_HIT_POINTS,
        hand_box_min_hit_points=DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS,
        camera_min_hit_points=DEPTH_GUARD_CAMERA_MIN_HIT_POINTS,
    )
    min_dist_mm = result.min_distance_m * 1000.0 if np.isfinite(result.min_distance_m) else float("inf")
    axis_min_dist_mm = result.hand_axis_min_distance_m * 1000.0 if np.isfinite(result.hand_axis_min_distance_m) else float("inf")
    if not quiet:
        print(
            f"  [深度护栏] {label}: 最近 {min_dist_mm:.0f}mm, "
            f"独立命中 中心={result.hand_axis_hit_count}/{DEPTH_GUARD_MIN_HIT_POINTS} "
            f"偏置盒={result.hand_box_hit_count}/{DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS} "
            f"相机={result.camera_hit_count}/{DEPTH_GUARD_CAMERA_MIN_HIT_POINTS} "
            f"(障碍点 {result.checked_points}), "
            f"模型 {_depth_guard_radius_label()}, "
            f"长度 {hand_length_m*1000:.0f}mm, 中心最近 {axis_min_dist_mm:.0f}mm"
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


def _depth_guard_check(
    label,
    obstacle_points_m,
    arm: "DobotArm",
    target_xyz_mm,
    hand_dir_m,
    hand_length_m=HAND_LENGTH,
    flange_rotation_m=None,
) -> bool:
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
    return _depth_guard_path_ok(
        label,
        obstacle_points_m,
        cur_pose[:3],
        target_xyz_mm,
        hand_dir_m,
        hand_length_m=hand_length_m,
        flange_rotation_m=flange_rotation_m,
    )


def _path_is_depth_clear(
    label,
    obstacle_points_m,
    waypoints_mm,
    hand_dir_m,
    hand_length_m=HAND_LENGTH,
    camera_depth_axis_m=None,
    axis_min_m=None,
    axis_max_m=None,
    flange_rotation_m=None,
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
            flange_rotation_m=flange_rotation_m,
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


def _camera_box_world_corners(flange_m, flange_rotation_m):
    box_min = np.asarray(DEPTH_GUARD_CAMERA_BOX_MIN_M, dtype=np.float32)
    box_max = np.asarray(DEPTH_GUARD_CAMERA_BOX_MAX_M, dtype=np.float32)
    local = np.array(
        [
            [x, y, z]
            for x in (box_min[0], box_max[0])
            for y in (box_min[1], box_max[1])
            for z in (box_min[2], box_max[2])
        ],
        dtype=np.float32,
    )
    rotation = np.asarray(flange_rotation_m, dtype=np.float32).reshape(3, 3)
    return np.asarray(flange_m, dtype=np.float32)[None, :] + local @ rotation.T


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
    flange_rotation_m=None,
    failure_reason=None,
):
    if image is None or obstacle_points_m is None or obstacle_pixels is None:
        return None
    viz = image.copy()
    pixels = np.asarray(obstacle_pixels, dtype=np.int32)

    if len(pixels):
        for u, v in pixels:
            if 0 <= u < viz.shape[1] and 0 <= v < viz.shape[0]:
                cv2.circle(viz, (int(u), int(v)), 1, (150, 150, 150), -1)

    (
        hit_mask,
        min_dist,
        hand_hit_mask,
        camera_hit_mask,
        hand_axis_hit_mask,
        hand_box_hit_mask,
        hand_axis_min_dist,
    ) = hand_path_hit_mask(
        obstacle_points_m,
        np.array(start_xyz_mm, dtype=np.float32) / 1000.0,
        np.array(target_xyz_mm, dtype=np.float32) / 1000.0,
        hand_dir_m,
        hand_length_m,
        DEPTH_GUARD_HAND_RADIUS_M,
        hand_radius_profile_m=_depth_guard_hand_radius_profile(hand_length_m),
        samples=DEPTH_GUARD_PATH_SAMPLES,
        camera_depth_axis_m=camera_depth_axis_m if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        camera_depth_tolerance_m=DEPTH_GUARD_DEPTH_ALIGN_TOLERANCE_M if DEPTH_GUARD_DEPTH_ALIGN_ENABLE else None,
        axis_min_m=axis_min_m,
        axis_max_m=axis_max_m,
        flange_rotation_m=flange_rotation_m,
        camera_box_min_local_m=DEPTH_GUARD_CAMERA_BOX_MIN_M if DEPTH_GUARD_CAMERA_ENABLE else None,
        camera_box_max_local_m=DEPTH_GUARD_CAMERA_BOX_MAX_M if DEPTH_GUARD_CAMERA_ENABLE else None,
        hand_boxes_local_m=DEPTH_GUARD_HAND_BOXES_M,
        return_components=True,
        return_hand_components=True,
    )
    camera_hit_pixels = pixels[camera_hit_mask] if len(pixels) == len(camera_hit_mask) else np.zeros((0, 2), dtype=np.int32)
    for u, v in camera_hit_pixels:
        if 0 <= u < viz.shape[1] and 0 <= v < viz.shape[0]:
            cv2.circle(viz, (int(u), int(v)), 3, (255, 0, 255), -1)
    hand_box_hit_pixels = pixels[hand_box_hit_mask] if len(pixels) == len(hand_box_hit_mask) else np.zeros((0, 2), dtype=np.int32)
    for u, v in hand_box_hit_pixels:
        if 0 <= u < viz.shape[1] and 0 <= v < viz.shape[0]:
            cv2.circle(viz, (int(u), int(v)), 3, (0, 165, 255), -1)
    hand_axis_hit_pixels = pixels[hand_axis_hit_mask] if len(pixels) == len(hand_axis_hit_mask) else np.zeros((0, 2), dtype=np.int32)
    for u, v in hand_axis_hit_pixels:
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

    # The camera body surrounds the observation optical center and is partly
    # behind the image plane. Projecting that box through the same camera makes
    # near-zero-depth corners explode into the giant wireframe seen in debug
    # images. It is still included in the 3-D collision check above; omit only
    # its self-view wireframe so the path visualization remains meaningful.

    finite_axis_min = hand_axis_min_dist[np.isfinite(hand_axis_min_dist)] if len(hand_axis_min_dist) else np.zeros((0,), dtype=np.float32)
    axis_min_mm = float(np.min(finite_axis_min) * 1000.0) if len(finite_axis_min) else float("inf")
    axis_hits = int(np.count_nonzero(hand_axis_hit_mask))
    box_hits = int(np.count_nonzero(hand_box_hit_mask))
    camera_hits = int(np.count_nonzero(camera_hit_mask))
    blocked = (
        axis_hits >= DEPTH_GUARD_MIN_HIT_POINTS
        or box_hits >= DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS
        or camera_hits >= DEPTH_GUARD_CAMERA_MIN_HIT_POINTS
    )
    cv2.putText(
        viz,
        f"{label} axis={axis_hits}/{DEPTH_GUARD_MIN_HIT_POINTS} box={box_hits}/{DEPTH_GUARD_HAND_BOX_MIN_HIT_POINTS} camera={camera_hits}/{DEPTH_GUARD_CAMERA_MIN_HIT_POINTS} axis_min={axis_min_mm:.0f}mm model={_depth_guard_radius_label()} len={hand_length_m*1000:.0f}",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 255) if blocked else (0, 255, 0),
        2,
    )
    if failure_reason:
        cv2.putText(
            viz,
            str(failure_reason),
            (12, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 165, 255),
            2,
        )
    safe_label = "".join(ch if ch.isalnum() else "_" for ch in str(label))
    path = f"/tmp/depth_guard_{safe_label}.jpg"
    cv2.imwrite(path, viz)
    print(f"  [深度护栏] 可视化已保存: {path} (灰=障碍点, 红=中心命中, 橙=偏置盒命中, 紫=相机命中, 绿=排除框/目标框, 青=手部路径)")
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
    flange_rotation_m=None,
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
        flange_rotation_m=flange_rotation_m,
    )
    direct_motion_failed = False
    if direct_ok:
        if arm.movel(float(target[0]), float(target[1]), float(target[2]), speed=speed, allow_low_z=allow_low_z):
            return True
        direct_motion_failed = True
        print(f"  [深度绕行] {label}: 直达 MovL 失败, 尝试中间点绕行")
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
            flange_rotation_m,
            failure_reason="MovL rejected after depth check passed",
        )
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
            flange_rotation_m,
        )

    if not (DEPTH_GUARD_BYPASS_ENABLE and allow_bypass):
        if direct_motion_failed:
            print(f"  [运动拒绝][中止] {label}: 深度检查已通过, 但直达 MovL 失败且不允许绕行")
        else:
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
            flange_rotation_m=flange_rotation_m,
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


def load_place_transition_low():
    """从 JSON 加载低位进层过渡点位姿，返回 (pose, name) 或 None"""
    global _place_transition_low_pose, _place_transition_low_joints
    if os.path.exists(PLACE_TRANSITION_LOW_FILE):
        with open(PLACE_TRANSITION_LOW_FILE) as f:
            data = json.load(f)
        if "pose" in data:
            _place_transition_low_pose = (data["pose"], data.get("name", "低位过渡点"))
            _place_transition_low_joints = data.get("joints")
    return _place_transition_low_pose


def save_place_transition_low():
    """保存低位进层过渡点位姿到 JSON"""
    if _place_transition_low_pose is None:
        if os.path.exists(PLACE_TRANSITION_LOW_FILE):
            os.remove(PLACE_TRANSITION_LOW_FILE)
        return
    pose, name = _place_transition_low_pose
    data = {"pose": pose, "name": name}
    if _place_transition_low_joints is not None:
        data["joints"] = _place_transition_low_joints
    with open(PLACE_TRANSITION_LOW_FILE, "w") as f:
        json.dump(data, f, indent=2)


def go_to_place_transition_low(arm: "DobotArm"):
    """关节运动到低位进层过渡点"""
    if _place_transition_low_pose is None:
        print("  [低位过渡点] 未记录，先用 l t s 记录")
        return False
    pose, name = _place_transition_low_pose
    ok = arm.move_pose_safe(pose, joints=_place_transition_low_joints, speed=80)
    if ok:
        print(f"  ✓ 已到达低位过渡点 ({name})")
    return ok


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
    if target_z < -400.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 -400mm")
        target_z = -400.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5], speed=100,
                     allow_low_z=allow_low_z):
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def go_to_layer_from_non_shelf_pose(arm: "DobotArm", layer_idx: int, visit_place_transition: bool = True):
    """从放置/未知位姿安全进层；先抬高，再经高位过渡后垂直下落。"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False

    pose, name = _shelf_layer_poses[layer_idx]
    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < -400.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 -400mm")
        target_z = -400.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")

    try:
        cur = arm.get_pose()
    except Exception as e:
        print(f"  [安全进层][中止] 无法读取当前位姿, 不发送运动命令: {e}")
        return False

    cp_pct = _clamp_cp(PLACE_TRANSITION_CP)
    descend_cp = min(cp_pct, 40)

    # 低位层: 若已记录低位过渡点, 经低位过渡点进层, 避免高位横移不可达
    if target_z < 50.0 and _place_transition_low_pose is not None:
        lt_pose, lt_name = _place_transition_low_pose
        lt_z = lt_pose[2]
        print(f"\n[低位进层] 经低位过渡点 {lt_name} 进入第{layer_idx}层 ...")
        if not arm.move_pose_safe(lt_pose, joints=_place_transition_low_joints, speed=80):
            print("  [低位进层] 前往低位过渡点失败")
            return False
        print(f"\n[货架层] 低位横移到第{layer_idx}层上方 ...")
        if not arm.movel(pose[0], pose[1], lt_z, pose[3], pose[4], pose[5],
                         speed=100, allow_low_z=True, cp=descend_cp, wait=False):
            return False
        print(f"\n[货架层] 下落到第{layer_idx}层 ({name})...")
        if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5],
                         speed=100, allow_low_z=True, cp=0):
            return False
        time.sleep(MOTION_SETTLE_S)
        return True

    z_candidates = [cur[2], target_z]
    if _place_transition_pose is not None:
        z_candidates.append(_place_transition_pose[0][2])
    base_z = max(z_candidates)
    max_return_z = min(1100.0, max(base_z + 60.0, PLACE_RETURN_MAX_Z_MM))
    safe_z = min(max_return_z, base_z + PLACE_RETURN_CLEARANCE_MM)

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

    print(f"\n[安全进层] 先原地抬高, 再进入第{layer_idx}层 ({name})...")
    lifted = False
    for idx, lift_z in enumerate(lift_targets, 1):
        print(f"  [安全进层] 尝试抬高到 Z={lift_z:.0f}mm ({idx}/{len(lift_targets)})")
        if arm.movel(cur[0], cur[1], lift_z, cur[3], cur[4], cur[5], speed=80, cp=0):
            safe_z = lift_z
            lifted = True
            break
        try:
            cur = arm.get_pose()
        except Exception:
            pass
    if not lifted:
        print("  [安全进层] 原地抬高失败")
        return False

    if visit_place_transition and _place_transition_pose is not None:
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


def go_to_layer_from_stow(arm: "DobotArm", layer_idx: int):
    """从收起位直接关节运动到目标层，避免绕行放置过渡点。"""
    if not is_at_stow_pose(arm):
        print("  [收起位进层][中止] 当前位姿不再匹配收起位, 不发送运动命令")
        return False
    if layer_idx not in _shelf_layer_poses:
        print(f"  [收起位进层][中止] 第{layer_idx}层未记录, 不发送运动命令")
        return False

    pose, name = _shelf_layer_poses[layer_idx]
    print(f"\n[收起位进层] 直接安全关节运动到第{layer_idx}层 ({name}) ...")
    if not arm.move_pose_safe(pose, speed=80):
        print(f"  [收起位进层] 直接前往第{layer_idx}层失败")
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
        print(f"  [安全返回][中止] 无法读取当前位姿, 不发送运动命令: {e}")
        return False

    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < -400.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 -400mm 作为可达观察位")
        target_z = -400.0
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


def go_to_place_point(arm: "DobotArm", visit_transition: bool = True):
    """移动机械臂到记录的放置点 (MovL 笛卡尔)"""
    if _place_point_pose is None:
        print("  [错误] 放置点未记录! 先用 p 进入放置点管理, 拖到目标位置后输入 s")
        return False
    pose, name = _place_point_pose
    if visit_transition and _place_transition_pose is not None:
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


def _angle_distance_deg(a, b):
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def is_at_stow_pose(arm: "DobotArm", verbose=True):
    """只读核对当前法兰是否已在收起姿态，不产生任何运动。"""
    if _stow_pose is None:
        if verbose:
            print("  [收起姿态检查] 未记录收起姿态")
        return False
    try:
        current = arm.get_pose()
    except Exception as exc:
        if verbose:
            print(f"  [收起姿态检查] 无法读取当前位姿: {exc}")
        return False
    target, name = _stow_pose
    position_error = float(np.linalg.norm(np.asarray(current[:3]) - np.asarray(target[:3])))
    angle_error = max(_angle_distance_deg(current[index], target[index]) for index in range(3, 6))
    matched = (position_error <= STOW_VERIFY_POSITION_TOLERANCE_MM
               and angle_error <= STOW_VERIFY_ANGLE_TOLERANCE_DEG)
    if verbose:
        state = "已在" if matched else "不在"
        print(
            f"  [收起姿态检查] {state}{name}: 位置误差={position_error:.1f}mm, "
            f"角度误差={angle_error:.1f}°"
        )
    return matched


def go_to_stow_pose(
    arm: "DobotArm",
    from_shelf_pose: bool = False,
    skip_transition: bool = False,
):
    """移动到收起姿态；已知在货架层时直接关节收起，避免多余过渡点。"""
    if _stow_pose is None:
        print("  [收起姿态] 未记录, 跳过。可用 u 菜单拖动记录")
        return False
    if is_at_stow_pose(arm):
        print("  [收起姿态] 当前已经收起, 跳过机械臂运动")
        return True

    if skip_transition:
        print("\n[收起姿态] 冷启动直接尝试收起，跳过放置过渡点 ...")
    elif from_shelf_pose:
        print("\n[收起姿态] 当前在货架层，直接关节收起，跳过放置过渡点 ...")
    elif _place_transition_pose is not None:
        transition_pose, transition_name = _place_transition_pose
        print(f"\n[收起姿态] 经放置过渡点 {transition_name} 安全收起 ...")
        if not arm.move_pose_safe(
            transition_pose,
            joints=_place_transition_joints,
            speed=80,
        ):
            print("  [收起姿态] 前往放置过渡点失败, 不继续收起")
            return False
    elif 2 in _shelf_layer_poses:
        print("\n[收起姿态] 先回第2层避碰 ...")
        if not go_to_layer_via_clearance(arm, 2):
            print("  [收起姿态] 回第2层失败, 不继续收起")
            return False
    else:
        print("  [收起姿态] 第2层未记录, 不继续收起")
        return False
    pose, name = _stow_pose
    print(f"\n[收起姿态] 安全关节运动到 {name} ...")
    if not arm.move_pose_safe(pose, speed=80):
        print("  [收起姿态] 移动失败")
        return False
    time.sleep(MOTION_SETTLE_S)
    return True


def load_chassis_place_settings():
    """每次放置前读取管理页配置，允许常驻 worker 不重启即可切换点位。"""
    enabled = USE_CHASSIS_PLACE
    marker = DEFAULT_CHASSIS_PLACE_MARKER
    try:
        if os.path.exists(CHASSIS_SETTINGS_FILE):
            with open(CHASSIS_SETTINGS_FILE, "r", encoding="utf-8") as file:
                settings = json.load(file)
            enabled = bool(settings.get("navigation_enabled", enabled))
            marker = str(settings.get("place_marker") or marker).strip()
    except Exception as exc:
        print(f"  [底盘放置][警告] 配置读取失败，使用默认值: {exc}")
    return enabled, marker or "fangzhi"


def chassis_at_place(marker):
    """检查底盘是否已在放置点。True 表示可直接放置，False 表示需要导航。"""
    env = os.environ.copy()
    env["MARKER_PLACE"] = marker
    try:
        result = subprocess.run(
            [sys.executable, MOVE_ROBOT_SCRIPT, "place", "--check-only"],
            cwd=os.path.dirname(__file__),
            env=env,
            text=True,
            capture_output=False,
            check=False,
        )
    except Exception as exc:
        print(f"  [底盘] 检查放置点失败，将按需要导航处理: {exc}")
        return False
    return result.returncode == 0


def move_chassis_to_place(marker):
    """复用 move_robot.py, 让底盘去放置固定点。"""
    print(f"\n[底盘] 前往放置固定点 {marker} ...")
    env = os.environ.copy()
    env["MARKER_PLACE"] = marker
    try:
        result = subprocess.run(
            [sys.executable, MOVE_ROBOT_SCRIPT, "place"],
            cwd=os.path.dirname(__file__),
            env=env,
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


def finish_pick_cycle(arm: "DobotArm", hand: Optional[object], current_layer_idx: Optional[int]):
    """抓取后放置并停在放置点, 下一单按目标层/货架继续移动。"""
    hand_ok = hand is not None and not SKIP_HAND

    if hand_ok and not hand.is_holding():
        print("  [停止] 灵巧手保持线程未运行, 不继续运输/放置流程")
        return False

    if _place_point_pose is None:
        if hand_ok:
            hand.open()
        return True

    if current_layer_idx == 4:
        # 第4层位于低位空间，先逐层上撤到第3层，再由第2层安全转向放置区。
        transfer_layers = [3, 2]
    elif current_layer_idx == 3:
        transfer_layers = [2]
    else:
        transfer_layers = []
    use_transfer_layer = bool(transfer_layers)

    if use_transfer_layer:
        print(f"\n[Transfer] 抓取后直接安全上撤: {' → '.join(f'第{i}层' for i in transfer_layers)} ...")
        for layer_idx in transfer_layers:
            if layer_idx not in _shelf_layer_poses:
                print(f"  [停止] 第{layer_idx}层未记录, 停在当前位置, 不继续放置流程")
                return False
            if not go_to_layer_via_clearance(arm, layer_idx):
                print(f"  [停止] 回第{layer_idx}层中转失败, 停在当前位置, 不继续放置流程")
                return False

    use_chassis_place, place_marker = load_chassis_place_settings()
    held_at_transition = False
    if use_chassis_place:
        if chassis_at_place(place_marker):
            print(f"\n[底盘] 已在放置点 {place_marker}，直接执行机械臂放置")
        else:
            if not go_to_place_transition(arm):
                print("  [停止] 底盘移动前未能到达放置过渡点, 不调用底盘")
                return False
            held_at_transition = True
            if hand_ok and not hand.is_holding():
                print("  [停止] 到达放置过渡点后灵巧手保持失败, 不调用底盘")
                return False
            print("  [运输] 机械臂保持在放置过渡点，开始移动底盘")
            if not move_chassis_to_place(place_marker):
                print("  [停止] 底盘未能前往放置固定点, 不继续机械臂放置")
                return False

    if hand_ok and not hand.is_holding():
        print("  [停止] 运输期间灵巧手保持失败, 不继续机械臂放置")
        return False

    print(f"\n[Place] 前往放置点 ({_place_point_pose[1]}) ...")
    if not go_to_place_point(arm, visit_transition=not held_at_transition):
        print("  [停止] 前往放置点失败, 停在当前位置, 不继续后续订单")
        return False
    if hand_ok:
        hand.open()
        print(f"  灵巧手 → 张开，等待 {HAND_OPEN_WAIT_S:.1f}s ...")
        time.sleep(HAND_OPEN_WAIT_S)
    else:
        time.sleep(MOTION_SETTLE_S)
    print("[流程] 商品已放置，开始收起机械臂")
    if not go_to_stow_pose(arm):
        print("  [停止] 商品已放置，但机械臂收起失败")
        return False
    print("[流程] 商品已放置且机械臂已收起")
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
    global _place_transition_low_pose, _place_transition_low_joints

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
        if _place_transition_low_pose:
            pose, name = _place_transition_low_pose
            joint_state = "有joints" if _place_transition_low_joints is not None else "无joints"
            print(f"  低位过渡点: {name}  pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f}) [{joint_state}]")
        else:
            print("  当前: 未记录低位过渡点 (低位层进层用)")
        print("\n  操作:")
        print("    s       ← 记录当前法兰位姿为放置点")
        print("    g       ← 移动到放置点 (若有过渡点则先经过渡点)")
        print("    d       ← 删除放置点")
        print("    ts      ← 记录当前法兰位姿为放置过渡点")
        print("    tg      ← 移动到放置过渡点")
        print("    td      ← 删除放置过渡点")
        print("    ls      ← 记录当前法兰位姿为低位过渡点 (第4/5层低位进层用)")
        print("    lg      ← 移动到低位过渡点")
        print("    ld      ← 删除低位过渡点")
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

        if choice == "ls":
            try:
                pose = arm.get_pose()
                joints = arm.get_joints()
            except Exception:
                print("  无法获取当前位姿/关节角")
                continue
            name = input('  低位过渡点名称 (回车默认"低位过渡点"): ').strip()
            if not name:
                name = "低位过渡点"
            _place_transition_low_pose = (list(pose[:6]), name)
            _place_transition_low_joints = list(joints[:6])
            save_place_transition_low()
            print(f"  ✓ 低位过渡点 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice == "ld":
            _place_transition_low_pose = None
            _place_transition_low_joints = None
            save_place_transition_low()
            print("  ✓ 低位过渡点已删除")
            continue

        if choice == "lg":
            go_to_place_transition_low(arm)
            return

        if choice == "g":
            go_to_place_point(arm)
            return

        print("  无效输入, 请输入 s / g / d / ts / tg / td / ls / lg / ld / q")


def _stow_pose_management(arm: "DobotArm", from_shelf_pose: bool = False):
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
            go_to_stow_pose(arm, from_shelf_pose=from_shelf_pose)
            return

        print("  无效输入, 请输入 s / g / d / q")


def _toggle_chassis_place():
    """切换放置前底盘固定点移动。"""
    global USE_CHASSIS_PLACE
    USE_CHASSIS_PLACE = not USE_CHASSIS_PLACE
    state = "开启" if USE_CHASSIS_PLACE else "关闭"
    print(f"\n[底盘放置] 已{state}")
    if USE_CHASSIS_PLACE:
        print("  流程: 经安全过渡点收起 → 调用 move_robot.py place → 等底盘到达 → 机械臂放置")


def _save_filter_preview(camera, locator, transformer, flange_pose, layer_z_m, layer_idx=None):
    """Capture one diagnostic frame showing dynamic layer filtering only."""
    image_path = os.getenv("SHELF_FILTER_PREVIEW_IMAGE", "/tmp/shelf_filter_preview.jpg")
    report_path = os.getenv("SHELF_FILTER_PREVIEW_REPORT", f"{image_path}.json")
    color, depth = camera.read()
    if color is None or depth is None:
        print("[过滤预览][失败] 相机未返回 RGB-D 帧", flush=True)
        return None

    detections = locator.process_frame(color, depth) or []
    bounds = detect_shelf_board_bounds(
        depth,
        camera,
        transformer,
        flange_pose,
        reference_z_m=layer_z_m,
        exclude_bboxes=[d.bbox for d in detections],
    )
    report = explain_target_filter(
        detections,
        transformer=transformer,
        flange_pose=flange_pose,
        layer_z_m=layer_z_m,
        shelf_board_bounds=bounds,
    )
    report["image_path"] = image_path
    report["report_path"] = report_path
    report["current_layer_z_m"] = layer_z_m
    report["current_layer"] = layer_idx
    report["boards"] = bounds.get("boards", []) if bounds else []
    report["selected_boards"] = {
        key: bounds[key] for key in ("lower", "upper")
        if bounds and bounds.get(key) is not None
    }
    report["dynamic_bounds"] = {
        key: bounds[key] for key in ("row_min", "row_max", "z_min", "z_max")
        if bounds and bounds.get(key) is not None
    }

    viz = color.copy()
    row_min = report["row_min"]
    row_max = report["row_max"]
    cv2.line(viz, (0, int(round(row_min))), (viz.shape[1] - 1, int(round(row_min))), (255, 0, 0), 2)
    cv2.line(viz, (0, int(round(row_max))), (viz.shape[1] - 1, int(round(row_max))), (255, 0, 0), 2)
    cv2.putText(viz, f"{report['mode']} rows={row_min:.0f}..{row_max:.0f}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 1)
    for item in report["detections"]:
        x1, y1, x2, y2 = item["bbox"]
        accepted = item["accepted"]
        color_box = (0, 190, 0) if accepted else (0, 0, 255)
        label = "KEEP" if accepted else "DROP:" + ",".join(item["reasons"])
        depth_text = "" if item["depth_m"] is None else f" {item['depth_m']:.2f}m"
        cv2.rectangle(viz, (x1, y1), (x2, y2), color_box, 2)
        cv2.putText(viz, f"{item['class_name'] or '?'} {label}{depth_text}", (x1, max(38, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color_box, 1)
    os.makedirs(os.path.dirname(image_path) or ".", exist_ok=True)
    os.makedirs(os.path.dirname(report_path) or ".", exist_ok=True)
    if not cv2.imwrite(image_path, viz):
        print(f"[过滤预览][失败] 无法写入图片: {image_path}", flush=True)
        return None
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    accepted_count = sum(1 for item in report["detections"] if item["accepted"])
    print(
        f"[过滤预览][完成] image={image_path} report={report_path} "
        f"mode={report['mode']} boards={len(report['boards'])} "
        f"accepted={accepted_count}/{len(report['detections'])}",
        flush=True,
    )
    return report


def _scan_inventory_layer(camera, locator, transformer, flange_pose, layer_z_m, layer_idx=None):
    """Identify the dominant product on the current layer without grasping."""
    report_path = os.getenv("SHELF_INVENTORY_SCAN_REPORT", "/tmp/shelf_inventory_scan.json")
    best_img, best_depth = None, None
    best_score = -1
    frame_detections = []

    for index in range(DETECTION_CAPTURE_FRAMES + DETECTION_WARMUP_FRAMES):
        color, depth = camera.read()
        if color is None:
            continue
        if index < DETECTION_WARMUP_FRAMES:
            continue
        score, _, _, _, _, _ = _assess_lighting(color)
        if score > best_score:
            best_score = score
            best_img, best_depth = color, depth
        frame_detections.append(locator.process_frame(color, depth) or [])

    if best_img is None:
        print("[巡架识别][失败] 相机未返回有效画面", flush=True)
        return None

    detections, _ = _fuse_detection_tracks(frame_detections, DETECTION_STABILITY_IOU)
    bounds = detect_shelf_board_bounds(
        best_depth,
        camera,
        transformer,
        flange_pose,
        reference_z_m=layer_z_m,
        exclude_bboxes=[d.bbox for d in detections],
    )
    filter_report = explain_target_filter(
        detections,
        transformer=transformer,
        flange_pose=flange_pose,
        layer_z_m=layer_z_m,
        shelf_board_bounds=bounds,
    )
    accepted = [item for item in filter_report["detections"] if item["accepted"]]
    counts = Counter(item["class_name"] for item in accepted if item["class_name"])
    candidates = [
        {"name": name, "count": count}
        for name, count in sorted(counts.items(), key=lambda item: (-item[1], item[0]))
    ]
    img_h, img_w = best_img.shape[:2]
    img_cx, img_cy = img_w / 2.0, img_h / 2.0
    if accepted:
        def _center_distance(item):
            cx, cy = item["center"]
            return (cx - img_cx) ** 2 + (cy - img_cy) ** 2
        nearest = min(accepted, key=_center_distance)
        dominant = {
            "name": nearest["class_name"],
            "count": 1,
            "center": nearest["center"],
            "distance_px": round(_center_distance(nearest) ** 0.5, 1),
        }
        print(
            f"[巡架识别] 取最靠近相机中心物品 "
            f"center=({nearest['center'][0]:.0f},{nearest['center'][1]:.0f}) "
            f"dist={dominant['distance_px']:.1f}px name={dominant['name'] or '-'}",
            flush=True,
        )
    else:
        dominant = {"name": "", "count": 0}
    report = {
        "current_layer": layer_idx,
        "dominant_name": dominant["name"],
        "count": dominant["count"],
        "total": len(accepted),
        "candidates": candidates,
        "captured_frames": len(frame_detections),
        "filter_mode": filter_report["mode"],
    }
    os.makedirs(os.path.dirname(report_path) or ".", exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print(
        f"[巡架识别][完成] report={report_path} "
        f"dominant={dominant['name'] or '-'} count={dominant['count']} total={len(accepted)}",
        flush=True,
    )
    return report


def _handle_worker_command(cmd, arm, hand, camera, locator, transformer, layer_entry_origin):
    """Handle one stdin control command before the next worker grab.

    Returns ``(action, layer_entry_origin, target)`` where action is one of
    ``continue``, ``stow``, ``poweroff``, ``quit`` or ``target``.
    """
    global _current_layer_idx

    if cmd in ("__stow_pose__", "__safe_stow__"):
        print("[前端常驻模式] 收到底盘移动前安全收起指令", flush=True)
        stowed = go_to_stow_pose(
            arm,
            from_shelf_pose=layer_entry_origin == LAYER_ENTRY_SHELF,
            skip_transition=layer_entry_origin == LAYER_ENTRY_STARTUP,
        )
        if stowed:
            print("[前端常驻模式][完成] 已到达收起姿态", flush=True)
        else:
            print("[前端常驻模式][失败] 移动到收起姿态失败", flush=True)
        return "continue", LAYER_ENTRY_STOW if stowed else LAYER_ENTRY_UNKNOWN, None

    if cmd in ("**stow**", "__stow__", "__shutdown__"):
        print("[前端常驻模式] 收到收起指令")
        go_to_stow_pose(arm, from_shelf_pose=layer_entry_origin == LAYER_ENTRY_SHELF)
        return "stow", False, None

    if cmd in ("__poweroff__", "__shutdown_power__"):
        print("[前端常驻模式] 收到断电关闭指令, 保持当前位置并关闭硬件")
        return "poweroff", layer_entry_origin, None

    if cmd in ("__quit__", "__exit__"):
        print("[前端常驻模式] 收到退出指令, 不自动收起")
        return "quit", layer_entry_origin, None

    if cmd in ("__filter_preview__", "__inspect_filter__"):
        current_layer_z_m = None
        if _current_layer_idx in _shelf_layer_poses:
            current_layer_z_m = _shelf_layer_poses[_current_layer_idx][0][2] / 1000.0
        _save_filter_preview(
            camera,
            locator,
            transformer,
            arm.get_pose(),
            current_layer_z_m,
            _current_layer_idx,
        )
        return "continue", layer_entry_origin, None

    if cmd in ("__inventory_scan__", "__scan_layer__"):
        current_layer_z_m = None
        if _current_layer_idx in _shelf_layer_poses:
            current_layer_z_m = _shelf_layer_poses[_current_layer_idx][0][2] / 1000.0
        _scan_inventory_layer(
            camera,
            locator,
            transformer,
            arm.get_pose(),
            current_layer_z_m,
            _current_layer_idx,
        )
        return "continue", layer_entry_origin, None

    layer_match = re.match(r'^__layer_(\d+)__$', cmd)
    if layer_match:
        target_layer = int(layer_match.group(1))
        print(f"[前端常驻模式] 正在移动到第{target_layer}层...", flush=True)
        if layer_entry_origin == LAYER_ENTRY_SHELF:
            move_to_layer = go_to_layer
        elif layer_entry_origin == LAYER_ENTRY_STOW:
            move_to_layer = go_to_layer_from_stow
        else:
            move_to_layer = go_to_layer_from_non_shelf_pose
        if move_to_layer(arm, target_layer):
            _current_layer_idx = target_layer
            if hand is not None:
                hand.open()
                time.sleep(MOTION_SETTLE_S)
            print(f"[前端常驻模式][完成] 已到达第{target_layer}层")
            return "continue", LAYER_ENTRY_SHELF, None
        print(f"[前端常驻模式][失败] 移动到第{target_layer}层失败")
        # 失败后当前位置可能停在任意中间点，下一次禁止直接横移进层。
        return "continue", LAYER_ENTRY_UNKNOWN, None

    movel_match = re.match(
        r'^__movel_([\d.\-]+),([\d.\-]+),([\d.\-]+),'
        r'([\d.\-]+),([\d.\-]+),([\d.\-]+)__$',
        cmd,
    )
    if movel_match:
        vals = [float(value) for value in movel_match.groups()]
        print(f"[前端常驻模式][完成] 移动到指定位姿: {vals}")
        if arm.movel(*vals, speed=100):
            print("[前端常驻模式][完成] 已到达目标位姿")
            return "continue", LAYER_ENTRY_UNKNOWN, None
        print("[前端常驻模式][失败] 移动到指定位姿失败")
        return "continue", LAYER_ENTRY_UNKNOWN, None

    if not cmd:
        return "continue", layer_entry_origin, None
    return "target", layer_entry_origin, cmd


def _stable_target_seen(frame_detections, target_query):
    """Return true when enough frames support at least one requested target."""
    if not DETECTION_EARLY_STOP_FRAMES or len(frame_detections) < DETECTION_EARLY_STOP_FRAMES:
        return False
    fused, tracks = _fuse_detection_tracks(frame_detections, DETECTION_STABILITY_IOU)
    if target_query:
        target_ids = {
            detection.class_id
            for detection in _filter_detections_by_target_query(fused, target_query)
        }
        tracks = [track for track in tracks if track["class_id"] in target_ids]
    return any(len(track["items"]) >= DETECTION_EARLY_STOP_SUPPORT for track in tracks)


def _draw_failure_diagnostic(image, detections=(), target=None, grip_pixel=None):
    """Draw the latest boxes and planned grip point for post-failure inspection."""
    if image is None:
        return None
    viz = image.copy()
    for detection in detections or ():
        bbox = tuple(int(round(value)) for value in detection.bbox)
        is_target = target is not None and detection.class_id == target.class_id \
            and _bbox_iou(detection.bbox, target.bbox) >= DETECTION_STABILITY_IOU
        color = (0, 0, 255) if is_target else (0, 255, 0)
        cv2.rectangle(viz, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        confidence = float(getattr(detection, "confidence", 0.0) or 0.0)
        cv2.putText(
            viz,
            f"id={detection.class_id} {confidence:.2f}",
            (bbox[0], max(14, bbox[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )
    if grip_pixel is not None:
        gx, gy = int(round(grip_pixel[0])), int(round(grip_pixel[1]))
        cv2.circle(viz, (gx, gy), 6, (0, 255, 255), -1)
        cv2.putText(viz, "GRIP", (gx + 8, gy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    return viz


def _save_failure_diagnostic(image):
    if image is None:
        return None
    try:
        os.makedirs(os.path.dirname(FAILURE_DIAGNOSTIC_PATH) or ".", exist_ok=True)
        if cv2.imwrite(FAILURE_DIAGNOSTIC_PATH, image):
            return FAILURE_DIAGNOSTIC_PATH
    except Exception as exc:
        print(f"[失败诊断图][警告] 保存失败: {exc}")
    return None


def _save_enhance_compare(locator, path=None):
    """保存弱光增强前后对比图（左=增强前, 右=增强后）。

    仅当识别模块最近记录过一次真实的增强帧时才保存；否则返回 None。
    """
    if locator is None or not getattr(locator, "enhance_compare_enable", False):
        return None
    if getattr(locator, "last_enhance_before", None) is None:
        return None
    target = path or ENHANCE_COMPARE_PATH
    try:
        return locator.save_enhance_compare(path=target, save_archive=False)
    except Exception as exc:
        print(f"[增强对比图][警告] 保存失败: {exc}")
    return None


def _clear_failure_diagnostic():
    try:
        os.remove(FAILURE_DIAGNOSTIC_PATH)
    except FileNotFoundError:
        pass
    except OSError as exc:
        print(f"[失败诊断图][警告] 无法清理旧文件: {exc}")
    try:
        os.remove(ENHANCE_COMPARE_PATH)
    except FileNotFoundError:
        pass
    except OSError as exc:
        print(f"[增强对比图][警告] 无法清理旧文件: {exc}")


def _connect_and_open_hand(hand):
    """Initialize the gripper without requiring any arm motion."""
    if (
        HAND_GRASP_MODE in ("flex", "flexible", "current")
        and hasattr(hand, "profile_path")
        and not os.path.isfile(hand.profile_path)
    ):
        raise RuntimeError(
            f"柔性抓取缺少空载曲线: {hand.profile_path}。"
            "请先用 test_lhandpro_flexible_grasp.py record 录制。"
        )
    hand.connect()
    print("[启动] 机械臂保持当前位置，先张开灵巧手")
    if not hand.open():
        raise RuntimeError("灵巧手启动张开失败，拒绝进入机械臂抓取流程")
    return True


def _disable_arm_on_process_exit(arm):
    """Best-effort safety net for Ctrl+C during startup initialization."""
    try:
        arm.close()
    except Exception as exc:
        print(f"[退出][警告] 机械臂下使能失败: {exc}", flush=True)


def _create_hand(arm=None):
    backend_model = "aero" if HAND_BACKEND == "aero" else "dh116"
    if backend_model != TOOL_HAND_MODEL:
        raise ValueError(
            f"灵巧手配置不一致: TOOL_HAND_MODEL={TOOL_HAND_MODEL}, HAND_BACKEND={HAND_BACKEND}。"
            "请通过管理端切换，或同时设置这两个环境变量。"
        )
    if HAND_BACKEND == "aero":
        from aero_gripper import AeroGripper
        return AeroGripper()
    if HAND_BACKEND in ("lhand", "lhandpro", "rs485"):
        from lhandpro_gripper import LHandProGripper
        return LHandProGripper()
    if HAND_BACKEND in ("lhandpro_dobot", "dobot_modbus", "tool_rs485"):
        from dobot_tcp_gripper import DobotTcpGripper
        return DobotTcpGripper(
            arm=arm,
            robot_ip=getattr(arm, "ip", os.getenv("DOBOT_IP", "192.168.5.1")),
            baud_rate=LHAND_DOBOT_BAUD,
            node_id=LHAND_RS485_NODE_ID,
            profile_path=LHAND_FLEX_PROFILE,
            control_transport=LHAND_DOBOT_CONTROL,
            debug=LHAND_DEBUG,
            tcp_debug=LHAND_HEX_DEBUG,
        )
    raise ValueError(
        f"不支持的 HAND_BACKEND={HAND_BACKEND!r}, "
        "可选 aero/lhandpro/lhandpro_dobot"
    )


def main():
    global _current_layer_idx

    print("=" * 60)
    print("  货架边缘抓取系统")
    print(
        f"  灵巧手: 总长{HAND_LENGTH*100:.0f}cm, 掌心{HAND_PALM_LENGTH_M*100:.1f}cm, "
        f"手指厚{HAND_FINGER_THICKNESS_M*100:.1f}cm, 掌宽{HAND_W_Y*100:.1f}cm, {HAND_WEIGHT}kg"
    )
    if DEPTH_GUARD_CAMERA_ENABLE:
        print(
            f"  相机碰撞盒: {camera_size_label_mm()}, 法兰系 "
            f"X={DEPTH_GUARD_CAMERA_BOX_MIN_M[0]*1000:.0f}..{DEPTH_GUARD_CAMERA_BOX_MAX_M[0]*1000:.0f}mm, "
            f"Y={DEPTH_GUARD_CAMERA_BOX_MIN_M[1]*1000:.0f}..{DEPTH_GUARD_CAMERA_BOX_MAX_M[1]*1000:.0f}mm, "
            f"Z={DEPTH_GUARD_CAMERA_BOX_MIN_M[2]*1000:.0f}..{DEPTH_GUARD_CAMERA_BOX_MAX_M[2]*1000:.0f}mm"
        )
    else:
        print("  相机碰撞盒: 暂不建模（连接件长托板仍参与碰撞检查）")
    print(f"  深度护栏偏置盒内收: {DEPTH_GUARD_HAND_BOX_SHRINK_M*1000:.0f}mm")
    print(f"  灵巧手型号: {TOOL_HAND_MODEL}, 后端: {HAND_BACKEND}, 手势模式: {HAND_GRASP_MODE}")
    if HAND_BACKEND in ("lhandpro_dobot", "dobot_modbus", "tool_rs485"):
        print(
            f"  LHandPro: {LHAND_DOBOT_BAUD}bps, node={LHAND_RS485_NODE_ID}, "
            f"control={LHAND_DOBOT_CONTROL}, profile={LHAND_FLEX_PROFILE}"
        )
    print(f"  抓取策略: {'临时强制全部侧抓' if FORCE_SIDE_GRASP else '按物体尺寸自动选择'}")
    print(
        f"  侧抓高度: 离底最低{SIDE_GRASP_MIN_HEIGHT_M*100:.1f}cm, "
        f"最高{SIDE_GRASP_MAX_HEIGHT_M*100:.1f}cm, 下移{SIDE_GRASP_LOWER_SHIFT_M*100:.1f}cm"
    )
    print(f"  目标商品: {TARGET_PRODUCT_QUERY or '未指定'}")
    if SHELF_GRASP_WORKER:
        print("  前端常驻模式: 保持使能, 等待 stdin 商品名; __stow__ 后收起并退出")
    elif SHELF_GRASP_AUTO_ONCE:
        print("  前端自动模式: 单次抓取后自动退出")
    print("=" * 60)
    target_query = TARGET_PRODUCT_QUERY

    # ── 1. 连接 DOBOT 并立即上电 ──
    print("\n[1/4] 连接 DOBOT 并上电 ...")
    arm = DobotArm()
    atexit.register(_disable_arm_on_process_exit, arm)
    arm.connect()

    # ── 货架层/放置/收起姿态 ──
    load_shelf_layers()
    load_place_point()
    load_place_transition()
    load_place_transition_low()
    load_stow_pose()

    if SHELF_GRASP_IDLE_STARTUP:
        print("\n[启动待机] 已上电使能，保持当前位置，不发送 Home/MovJ/MovL")
    elif SAFE_NAV_STARTUP:
        print("\n[启动安全检查] 导航订单模式不执行 Home/观察层运动，只核对当前收起姿态 ...")
        if not is_at_stow_pose(arm):
            raise RuntimeError("导航订单启动时机械臂不在收起姿态，拒绝自动移动和接单")
        print("[启动安全检查][完成] 机械臂已在收起姿态，启动过程未产生机械臂运动")
    else:
        print("\n[Home] 回到预设初始位姿 ...")
        arm.go_home()
        time.sleep(MOTION_SETTLE_S)  # 短缓冲，MovJ 内部已等待运动完成

    # ── 2. 初始化相机 + YOLO ──
    print("\n[2/4] 初始化相机 + YOLO ...")
    camera = RealSenseCamera(width=640, height=480, fps=30)
    _repo_root = os.environ.get("GRASP_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _recognition_dir = os.environ.get("GRASP_RECOGNITION_DIR", os.path.join(_repo_root, "识别"))
    default_model_path = os.path.join(_recognition_dir, "best.engine")
    if not os.path.exists(default_model_path):
        default_model_path = os.path.join(_recognition_dir, "best.pt")
    model_path = os.getenv("YOLO_MODEL_PATH", default_model_path)
    print(f"  视觉模型: {model_path}")
    locator = MedicineLocatorSystem(
        model_path=model_path,
        mapping_path=os.environ.get("SHELF_MAPPING_PATH", os.path.join(_recognition_dir, "shelf_mapping.yaml")),
        conf_threshold=DETECT_CONF_THRESHOLD,
        class_conf_thresholds={cls_id: WATER_CONF_THRESHOLD for cls_id in DRINK_CLASS_IDS},
        use_ocr=False,
    )
    locator.init_camera = lambda: None
    locator.camera = camera

    # ── 3. 连接灵巧手 ──
    if SKIP_HAND:
        print("\n[3/4] 灵巧手 已屏蔽 (SKIP_HAND=True)")
        hand = None
    else:
        print(f"\n[3/4] 连接 {HAND_BACKEND} 灵巧手 ...")
        hand = _create_hand(arm)
        _connect_and_open_hand(hand)

    if _current_layer_idx == STARTUP_LAYER_IDX:
        save_shelf_layers()
    if not SHELF_GRASP_IDLE_STARTUP and not SAFE_NAV_STARTUP and _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
        go_to_layer_via_clearance(arm, _current_layer_idx)

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
    print("  - 输入 f   ← 保存当前层板过滤预览 (不移动/不抓取)")
    print("  - 输入 j   ← 手动点动 (wasd/e/r 微调法兰)")
    print("  - 也可直接输入目标名抓取；输入 all/clear 清空目标")
    print("  - 输入 q   ← 退出")
    print("=" * 60)

    try:
        auto_attempt_pending = SHELF_GRASP_AUTO_ONCE
        auto_attempt_succeeded = False
        worker_stow_requested = False
        worker_poweroff_requested = False
        latest_failure_image = None
        # 待机启动时不知道机械臂当前位于哪条已规划路径上，禁止直接横移进层。
        layer_entry_origin = LAYER_ENTRY_STARTUP if SHELF_GRASP_IDLE_STARTUP else LAYER_ENTRY_SHELF

        def worker_fail(message: str):
            diagnostic_path = _save_failure_diagnostic(latest_failure_image)
            if diagnostic_path:
                print(f"[失败诊断图] {diagnostic_path}", flush=True)
            compare_path = _save_enhance_compare(locator)
            if compare_path:
                print(f"[增强对比图] {compare_path} (左=增强前, 右=增强后)", flush=True)
            if SHELF_GRASP_WORKER:
                print(f"[前端常驻模式][失败] {message}", flush=True)

        while True:
            if SHELF_GRASP_AUTO_ONCE and not auto_attempt_pending:
                break
            chassis_state = "开" if USE_CHASSIS_PLACE else "关"
            target_state = target_query or "任意"
            if SHELF_GRASP_WORKER:
                print("\n[前端常驻模式] 等待商品名 (**stow**=收起并退出, __quit__=直接退出) ...", flush=True)
                cmd = sys.stdin.readline()
                if cmd == "":
                    print("[前端常驻模式] stdin 已关闭, 失去上级控制, 正在关闭硬件使能")
                    worker_poweroff_requested = True
                    break
                cmd = cmd.strip()
                action, layer_entry_origin, target = _handle_worker_command(
                    cmd, arm, hand, camera, locator, transformer, layer_entry_origin
                )
                if action == "continue":
                    continue
                if action in ("stow", "poweroff", "quit"):
                    worker_stow_requested = action == "stow"
                    worker_poweroff_requested = action == "poweroff"
                    break
                target_query = target
                print(f"[前端常驻模式] 目标:{target_query}, 开始抓取")
            elif SHELF_GRASP_AUTO_ONCE:
                cmd = ""
                auto_attempt_pending = False
                print(f"\n[前端自动模式] 目标:{target_state}, 开始单次抓取")
            else:
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    _, layer_name = _shelf_layer_poses[_current_layer_idx]
                    prompt = f"\n[第{_current_layer_idx}层 {layer_name} | 目标:{target_state}] 按 Enter 抓取 | c 目标 | ? 类别 | l 层管理 | p 放置点 | u 收起 | b 底盘:{chassis_state} | v 预览 | f 过滤预览 | j 点动 | q 退出 > "
                else:
                    prompt = f"\n[HOME | 目标:{target_state}] 按 Enter 抓取 | c 目标 | ? 类别 | l 层管理 | p 放置点 | u 收起 | b 底盘:{chassis_state} | v 预览 | f 过滤预览 | j 点动 | q 退出 > "
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
                _stow_pose_management(arm, from_shelf_pose=layer_entry_origin == LAYER_ENTRY_SHELF)
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

            if cmd.lower() == "f":
                current_layer_z_m = None
                if _current_layer_idx in _shelf_layer_poses:
                    current_layer_z_m = _shelf_layer_poses[_current_layer_idx][0][2] / 1000.0
                _save_filter_preview(camera, locator, transformer, arm.get_pose(), current_layer_z_m, _current_layer_idx)
                continue

            if cmd:
                target_query = cmd
                print(f"  [目标] 本次/后续按关键词抓取: {target_query}")

            # Never reuse a previous order's image when this attempt has no valid camera frame.
            latest_failure_image = None
            _clear_failure_diagnostic()

            # ── 获取当前法兰位姿 ──
            flange_pose = arm.get_pose()
            print(f"当前法兰: ({flange_pose[0]:.0f}, {flange_pose[1]:.0f}, {flange_pose[2]:.0f})mm "
                  f"rpy=({flange_pose[3]:.0f}, {flange_pose[4]:.0f}, {flange_pose[5]:.0f})")
            transformer.set_flange(flange_pose)

            # ── 多帧择优拍照：快速连拍 N 帧, 按光照评分选最佳 ──
            frame_limit = DETECTION_CAPTURE_FRAMES
            warmup_frames = DETECTION_WARMUP_FRAMES
            best_img, best_depth = None, None
            best_score, best_grade = -1, ""
            best_idx = -1
            frame_detections = []
            captured_frames = []

            print(f"拍照中 (最多 {frame_limit + warmup_frames} 帧, 丢弃前 {warmup_frames})...", end=" ", flush=True)
            capture_started = time.monotonic()
            for i in range(frame_limit + warmup_frames):
                c, d = camera.read()
                if c is None:
                    continue
                if i < warmup_frames:
                    continue
                score, grade, _, _, _, _ = _assess_lighting(c)
                if score > best_score:
                    best_score, best_grade = score, grade
                    best_img, best_depth = c, d
                    best_idx = i
                dets = locator.process_frame(c, d)
                frame_detections.append(dets)
                captured_frames.append((c, d))
                if _stable_target_seen(frame_detections, target_query):
                    print(f"稳定目标已确认({len(frame_detections)}帧), 提前结束; ", end="")
                    break
                # 反光和过暗都有惩罚, 纯正常最高; 有反光也能选出相对最好的
            capture_elapsed_s = time.monotonic() - capture_started
            print(f"完成 (YOLO={len(frame_detections)}帧, {capture_elapsed_s:.2f}s)")
            print(f"  → 第 {best_idx} 帧 评分: {best_grade} ({best_score:.0f}分)")

            if best_img is None:
                print("相机未就绪")
                worker_fail("相机未就绪")
                continue

            latest_failure_image = best_img.copy()

            detections, tracks = _fuse_detection_tracks(frame_detections, DETECTION_STABILITY_IOU)
            latest_failure_image = _draw_failure_diagnostic(best_img, detections)
            if tracks:
                stable = sorted(
                    tracks,
                    key=lambda t: (len(t["items"]), max(item.confidence for _, item in t["items"])),
                    reverse=True,
                )[:6]
                stable_text = " | ".join(
                    f"{t['items'][0][1].class_name}:frames={len(t['items'])}/{len(frame_detections)},"
                    f"max={max(item.confidence for _, item in t['items']):.2f}"
                    for t in stable
                )
                print(f"  → 多帧稳定候选: {stable_text}")
            if SAVE_DEBUG_IMAGES:
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
                worker_fail(f"未检测到目标: {target_query or '任意'}")
                continue

            # ── 从当前深度帧识别层板，按上下板边界过滤上下层，避免误抓相邻层。 ──
            current_layer_z_m = None
            if _current_layer_idx in _shelf_layer_poses:
                current_layer_z_m = _shelf_layer_poses[_current_layer_idx][0][2] / 1000.0
            shelf_board_bounds = detect_shelf_board_bounds(
                best_depth,
                camera,
                transformer,
                flange_pose,
                reference_z_m=current_layer_z_m,
                exclude_bboxes=[d.bbox for d in detections],
            )
            if shelf_board_bounds and shelf_board_bounds.get("lower") and shelf_board_bounds.get("upper"):
                lower = shelf_board_bounds["lower"]
                upper = shelf_board_bounds["upper"]
                print(
                    f"  层板识别: 下板 Z={lower['z']*1000:.0f}mm(row={lower['row']}), "
                    f"上板 Z={upper['z']*1000:.0f}mm(row={upper['row']})"
                )
            elif shelf_board_bounds:
                print(f"  层板识别: 找到 {len(shelf_board_bounds.get('boards', []))} 个候选板, 未形成当前层上下边界, 使用回退过滤")
            target_detections = _filter_detections_by_target_query(detections, target_query)
            _print_target_filter_result(target_query, detections, target_detections)
            if target_query and not target_detections:
                if SHELF_GRASP_WORKER:
                    worker_fail(f"未找到目标: {target_query}")
                    continue
                if SHELF_GRASP_AUTO_ONCE:
                    raise RuntimeError(f"前端自动模式未找到目标: {target_query}")
                continue
            target, edge = pick_target_box(
                target_detections,
                transformer=transformer,
                flange_pose=flange_pose,
                layer_z_m=current_layer_z_m,
                shelf_board_bounds=shelf_board_bounds,
            )
            if target is None:
                print("无法确定目标")
                if SHELF_GRASP_WORKER:
                    worker_fail("无法确定目标")
                    continue
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
            latest_failure_image = _draw_failure_diagnostic(best_img, detections, target, grip_pixel)

            if grip_mode.endswith("_side") and not EDGE_MIDPOINT_GRASP_ENABLE:
                corrected_pixel, original_clearance_m, bottom_clearance_m = (
                    clamp_side_grasp_pixel_to_safe_clearance(target, grip_pixel, camera.fy)
                )
                if corrected_pixel is None or bottom_clearance_m is None:
                    clearance_label = (
                        "无效"
                        if original_clearance_m is None
                        else f"{original_clearance_m*100:.1f}cm"
                    )
                    message = (
                        f"侧抓点距检测框底边 {clearance_label}, 且缺少可用深度/框高，无法安全纠偏"
                    )
                    print(f"  [抓点硬纠偏][中止] {message}; 不发送机械臂动作")
                    if SHELF_GRASP_WORKER:
                        worker_fail(message)
                        continue
                    if SHELF_GRASP_AUTO_ONCE:
                        raise RuntimeError(message)
                    continue
                if corrected_pixel != tuple(grip_pixel):
                    old_pixel = grip_pixel
                    grip_pixel = corrected_pixel
                    p_cam = transformer.pixel_to_camera(grip_pixel[0], grip_pixel[1], target.depth_m)
                    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=True)
                    wx, wy, wz = float(p_robot[0]), float(p_robot[1]), float(p_robot[2])
                    latest_failure_image = _draw_failure_diagnostic(
                        best_img, detections, target, grip_pixel
                    )
                    print(
                        f"  [抓点硬纠偏] 距检测框底边 {original_clearance_m*100:.1f}cm → "
                        f"{bottom_clearance_m*100:.1f}cm, 像素Y {old_pixel[1]:.1f} → {grip_pixel[1]:.1f}"
                    )
                    print(f"  [抓点硬纠偏] 更新抓取点: ({wx:.3f}, {wy:.3f}, {wz:.3f})m")
                else:
                    print(
                        f"  [抓点硬纠偏] 距检测框底边 {bottom_clearance_m*100:.1f}cm, "
                        f"已在 {SIDE_GRASP_MIN_HEIGHT_M*100:.1f}..{SIDE_GRASP_MAX_HEIGHT_M*100:.1f}cm 安全区间"
                    )

            if SAVE_DEBUG_IMAGES:
                # 重新画正确的抓取点 (覆盖上面简单画的黄点)
                viz = best_img.copy()
                if shelf_board_bounds and shelf_board_bounds.get("row_min") is not None:
                    filter_row_min = shelf_board_bounds["row_min"]
                    filter_row_max = shelf_board_bounds["row_max"]
                    filter_label = "动态层板范围"
                else:
                    filter_row_min = TARGET_ROI_Y_MIN
                    filter_row_max = TARGET_ROI_Y_MAX
                    filter_label = "回退ROI"
                cv2.line(viz, (0, filter_row_min), (viz.shape[1] - 1, filter_row_min), (255, 0, 0), 1)
                cv2.line(viz, (0, filter_row_max), (viz.shape[1] - 1, filter_row_max), (255, 0, 0), 1)
                cv2.putText(viz, filter_label, (8, max(18, int(filter_row_min) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)
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
                print(f"  → 检测画面已保存: /tmp/shelf_detection.jpg  (蓝线={filter_label}, 绿框=检测, 红框=目标, 黄点=抓取点)")

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

            # Camera-left direction is used for the separate centering nudge
            # after the product has entered the palm and before hand closure.
            camera_x_dir = R_f @ transformer.T_flange_to_cam[:3, 0]
            camera_left_dir = -camera_x_dir / max(float(np.linalg.norm(camera_x_dir)), 1e-9)

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
            if shelf_board_bounds and shelf_board_bounds.get("row_min") is not None:
                depth_guard_row_min = shelf_board_bounds["row_min"] + DEPTH_GUARD_SHELF_ROW_INSET_PX
                depth_guard_row_max = shelf_board_bounds["row_max"] - DEPTH_GUARD_SHELF_ROW_INSET_PX
                depth_guard_row_source = "layer"
            else:
                depth_guard_row_min = max(0, float(target.bbox[1]) - DEPTH_GUARD_TARGET_ROW_TOP_MARGIN_PX)
                depth_guard_row_max = min(
                    best_depth.shape[0] - 1,
                    float(target.bbox[3]) + DEPTH_GUARD_TARGET_ROW_BOTTOM_MARGIN_PX,
                )
                depth_guard_row_source = "target"
            if DEPTH_GUARD_ENABLE:
                depth_obstacles_m, depth_obstacles_px = depth_to_robot_obstacles(
                    best_depth,
                    camera,
                    transformer,
                    depth_observation_flange_pose,
                    stride=DEPTH_GUARD_STRIDE,
                    max_depth_m=DEPTH_GUARD_MAX_DEPTH_M,
                    target_depth_m=target.depth_m,
                    target_depth_margin_m=DEPTH_GUARD_TARGET_DEPTH_MARGIN_M,
                    include_row_min_px=depth_guard_row_min,
                    include_row_max_px=depth_guard_row_max,
                    exclude_bbox=None,
                    exclude_bboxes=depth_guard_exclude_bboxes,
                    exclude_margin_px=DEPTH_GUARD_EXCLUDE_MARGIN_PX,
                    exclude_self_hand=DEPTH_GUARD_SELF_MASK_ENABLE,
                    self_hand_dir_m=fz_dir,
                    self_hand_length_m=DEPTH_GUARD_SELF_MASK_LENGTH_M,
                    self_hand_radius_m=DEPTH_GUARD_SELF_MASK_RADIUS_M,
                    self_hand_radius_profile_m=_depth_guard_hand_radius_profile(
                        DEPTH_GUARD_SELF_MASK_LENGTH_M,
                        self_mask=True,
                    ),
                    self_hand_backoff_m=DEPTH_GUARD_SELF_MASK_BACKOFF_M,
                    self_camera_box_min_local_m=(
                        DEPTH_GUARD_SELF_CAMERA_BOX_MIN_M if DEPTH_GUARD_CAMERA_ENABLE else None
                    ),
                    self_camera_box_max_local_m=(
                        DEPTH_GUARD_SELF_CAMERA_BOX_MAX_M if DEPTH_GUARD_CAMERA_ENABLE else None
                    ),
                    self_hand_boxes_local_m=DEPTH_GUARD_SELF_HAND_BOXES_M,
                    noise_filter_radius_px=DEPTH_GUARD_NOISE_FILTER_RADIUS_PX,
                    noise_filter_min_support=DEPTH_GUARD_NOISE_FILTER_MIN_SUPPORT,
                    noise_filter_tolerance_m=DEPTH_GUARD_NOISE_FILTER_TOLERANCE_M,
                    return_pixels=True,
                )
                print(
                    f"  [深度护栏] 障碍点={len(depth_obstacles_m)} "
                    f"(stride={DEPTH_GUARD_STRIDE}, max_depth={DEPTH_GUARD_MAX_DEPTH_M:.2f}m, "
                    f"噪点滤波={DEPTH_GUARD_NOISE_FILTER_RADIUS_PX}px/{DEPTH_GUARD_NOISE_FILTER_MIN_SUPPORT}点±{DEPTH_GUARD_NOISE_FILTER_TOLERANCE_M*1000:.0f}mm, "
                    f"排除框={len(depth_guard_exclude_bboxes)}, 外扩={DEPTH_GUARD_EXCLUDE_MARGIN_PX}px, "
                    f"层内行={depth_guard_row_min:.0f}..{depth_guard_row_max:.0f}({depth_guard_row_source}), "
                    f"目标后方≤{DEPTH_GUARD_TARGET_DEPTH_MARGIN_M*1000:.0f}mm, "
                    f"自手部={'on' if DEPTH_GUARD_SELF_MASK_ENABLE else 'off'} "
                    f"r=掌{DEPTH_GUARD_SELF_MASK_RADIUS_M*1000:.0f}/指{DEPTH_GUARD_SELF_MASK_FINGER_RADIUS_M*1000:.0f}mm, "
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
                 print(
                     f"  对齐→抓取直线: {approach_to_grasp_mm:.0f}mm, "
                     f"过抓前进: {grasp_advance_m*1000:.0f}mm, "
                     f"入掌后左移: {PRE_CLOSE_LEFT_SHIFT_M*1000:.0f}mm"
                     + (" (第4层低位)" if low_z_grasp else "")
                 )
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
                 if SHELF_GRASP_AUTO_ONCE or SHELF_GRASP_WORKER:
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
                         flange_rotation_m=R_f,
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
                flange_rotation_m=R_f,
            ):
                print("  [中止] 对齐位运动失败/报警, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                worker_fail("对齐位运动失败/深度护栏拒绝")
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
                flange_rotation_m=R_f,
            ):
                print("  [中止] 探入路径运动失败或被深度护栏拒绝, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                worker_fail("探入路径运动失败/深度护栏拒绝")
                continue
            time.sleep(GRASP_SETTLE_S)

            # Product is now inside the palm. Nudge left in camera-image
            # coordinates to center it, then close the dexterous hand.
            pre_close_target = (
                np.array([gx + offset_x, gy + offset_y, gz + offset_z], dtype=np.float32)
                + camera_left_dir.astype(np.float32) * (PRE_CLOSE_LEFT_SHIFT_M * 1000.0)
            )
            print(
                f"  [抓前居中] 入掌后向画面左侧移动 "
                f"{PRE_CLOSE_LEFT_SHIFT_M*1000:.0f}mm"
            )
            if not arm.movel(
                float(pre_close_target[0]),
                float(pre_close_target[1]),
                float(pre_close_target[2]),
                speed=grasp_speed,
                allow_low_z=allow_low_z,
            ):
                print("  [中止] 抓前左移失败/报警, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                worker_fail("抓前左移失败")
                continue
            time.sleep(GRASP_SETTLE_S)

            # 4. 抓取 (动作二) 并等待握紧。反馈未确认时 LHandPro 会继续保持闭合；
            # 只有过流、通信等真正的执行失败才进入安全退回。
            grasp_succeeded = SKIP_HAND
            if not SKIP_HAND:
                if HAND_GRASP_MODE in ("flex", "flexible", "current"):
                    grasp_succeeded = hand.flexible_close()
                else:
                    grasp_mode = "bottle" if HAND_GRASP_MODE == "bottle" and is_side_grasp else None
                    grasp_succeeded = hand.close(top=not is_side_grasp, mode=grasp_mode)
                if grasp_succeeded:
                    print(f"  灵巧手 → 闭合, 等待 {HAND_CLOSE_WAIT_S:.1f}s ...")
                    time.sleep(HAND_CLOSE_WAIT_S)
                    if not hand.is_holding():
                        grasp_succeeded = False
                        print("  [抓取失败] 等待握紧期间灵巧手保持线程已停止")
                else:
                    print("  [抓取失败] 灵巧手闭合/保持失败，只执行机械臂安全退回，不调用底盘和放置流程")

            # 5. 后退: 先沿手指方向退回对齐位, 再离开货架
            retreat_pt = np.array([ax+offset_x, ay+offset_y, az+offset_z])
            if not arm.movel(float(retreat_pt[0]), float(retreat_pt[1]), float(retreat_pt[2]), speed=100,
                             allow_low_z=allow_low_z):
                print("  [停止] 后退失败/报警, 停在当前位置, 不继续放置流程")
                worker_fail("后退失败/报警")
                continue
            time.sleep(GRASP_SETTLE_S)

            if not grasp_succeeded:
                # Only failed grabs return to the original layer for a safe retry.
                # A successful grab goes straight to placement; the next order
                # will select its own layer or stow before changing shelves.
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    if not go_to_layer(arm, _current_layer_idx):
                        print("  [停止] 回观察位失败/报警, 停在当前位置, 不继续放置流程")
                        worker_fail("回观察位失败/报警")
                        continue
                else:
                    arm.go_home()
                # Keep the last finger command on a genuine close failure as
                # well. The next grasp attempt opens explicitly before entry.
                worker_fail("灵巧手闭合/保持失败")
                continue

            # 6. 先在过渡点保持夹持，底盘到位后放置，再收起等待下一单。
            if not finish_pick_cycle(arm, hand, _current_layer_idx):
                print("  [停止] 放置失败, 本次抓取未完成")
                worker_fail("放置失败")
                continue
            layer_entry_origin = LAYER_ENTRY_STOW
            time.sleep(MOTION_SETTLE_S)
            auto_attempt_succeeded = True

            print("抓取完成! 按 Enter 继续 / q 退出")
            if SHELF_GRASP_WORKER:
                print(f"[前端常驻模式][完成] {target_query}", flush=True)
                continue
            if SHELF_GRASP_AUTO_ONCE:
                break

        if SHELF_GRASP_AUTO_ONCE and not SHELF_GRASP_WORKER and not auto_attempt_succeeded:
            raise RuntimeError("前端自动模式单次抓取未完成")

    except KeyboardInterrupt:
        pass
    finally:
        if SHELF_GRASP_WORKER and worker_stow_requested:
            print("[前端常驻模式] 已按收起指令到位, 跳过退出前二次收起")
        elif SHELF_GRASP_WORKER and worker_poweroff_requested:
            print("[前端常驻模式] 正在停止保持、断开灵巧手并关闭 DOBOT 使能")
        elif SHELF_GRASP_WORKER:
            print("[前端常驻模式] worker 即将退出, 不执行移动, 直接关闭硬件使能")
        elif SHELF_GRASP_AUTO_ONCE:
            print("[前端自动模式] 退出时不自动收起, 保持当前安全观察位/当前位置")
        else:
            try:
                go_to_stow_pose(arm, from_shelf_pose=layer_entry_origin == LAYER_ENTRY_SHELF)
            except Exception as e:
                print(f"[收起姿态] 退出前收起失败/跳过: {e}")
        # Disable the arm first. The hand SDK disconnect may block while it
        # stops/flushes RS485, so it must not delay the robot safety command.
        try:
            arm.close()
        except Exception as exc:
            print(f"[退出][警告] 机械臂下使能失败: {exc}", flush=True)
        if not SKIP_HAND:
            try:
                hand.disconnect()
            except Exception as exc:
                print(f"[退出][警告] 灵巧手断开失败: {exc}", flush=True)
        camera.release()
        print("\n已退出")


if __name__ == "__main__":
    main()
