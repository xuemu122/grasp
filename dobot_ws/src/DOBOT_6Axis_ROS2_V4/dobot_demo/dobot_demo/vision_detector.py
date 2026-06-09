#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
药盒视觉检测模块
功能：利用摄像头检测药盒的四个角点位置 + 货架高度
输出：药盒中心点 3D 坐标、旋转角度、货架高度（相机坐标系下）

注意：此为检测逻辑框架，实际使用需替换为真实相机 + ArUco/轮廓检测等算法。
      当前使用模拟数据演示完整数据流。
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


# ============================================================
# 数据类：视觉检测结果
# ============================================================
@dataclass
class MedicineBox:
    """药盒检测结果"""
    corners: List[Tuple[float, float]]  # 四个角点像素坐标 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
    center_pixel: Tuple[float, float]   # 中心点像素坐标
    center_3d: Tuple[float, float, float]  # 中心点 3D 坐标（相机系，单位：mm）
    rotation_deg: float                 # 旋转角度（度）
    shelf_height_mm: float              # 货架高度（相机系 Z 坐标，单位：mm）
    box_size: Tuple[float, float]       # 药盒宽高 (mm)


# ============================================================
# 相机内参（示例 — 实际使用需标定）
# ============================================================
CAMERA_INTRINSICS = {
    "fx": 600.0,   # X 方向焦距（像素）
    "fy": 600.0,   # Y 方向焦距（像素）
    "cx": 320.0,   # 主点 X（像素）
    "cy": 240.0,   # 主点 Y（像素）
    "width": 640,  # 图像宽度
    "height": 480, # 图像高度
}


class MedicineBoxDetector:
    """
    药盒视觉检测器

    流程：
    1. 采集 RGB-D 图像
    2. 检测药盒四个角点（轮廓/AI模型/Aruco码）
    3. 用深度图 + 相机内参反投影到 3D
    4. 检测货架平面高度
    """

    def __init__(self, camera_matrix: dict = None):
        """
        Args:
            camera_matrix: 相机内参字典，默认使用 CAMERA_INTRINSICS
        """
        self.K = camera_matrix or CAMERA_INTRINSICS
        self.fx = self.K["fx"]
        self.fy = self.K["fy"]
        self.cx = self.K["cx"]
        self.cy = self.K["cy"]

        # 真实场景中，这里是相机驱动订阅
        # self.rgb_sub = self.create_subscription(Image, '/camera/rgb', ...)
        # self.depth_sub = self.create_subscription(Image, '/camera/depth', ...)

        print("[Detector] 药盒检测器已初始化")

    # ----------------------------------------------------------
    # Step 1: 检测药盒四个角点（像素坐标）
    # ----------------------------------------------------------
    def detect_box_corners(self, rgb_image=None) -> List[Tuple[float, float]]:
        """
        检测药盒四个角点的像素坐标。

        真实实现可选方案：
        - OpenCV findContours + approxPolyDP（矩形轮廓）
        - YOLO/目标检测 + 角点回归
        - ArUco 码（药盒上贴 marker）
        - 颜色分割 + 形态学

        当前返回模拟数据（假设一个 640x480 图像中的药盒）。
        """
        # TODO: 替换为真实检测逻辑
        corners = [
            (220.0, 160.0),  # 左上
            (420.0, 160.0),  # 右上
            (420.0, 260.0),  # 右下
            (220.0, 260.0),  # 左下
        ]
        print(f"[Detector] 检测到药盒角点: {corners}")
        return corners

    # ----------------------------------------------------------
    # Step 2: 像素 → 3D 反投影（需要深度图）
    # ----------------------------------------------------------
    def pixel_to_3d(self, u: float, v: float, depth_mm: float) -> Tuple[float, float, float]:
        """
        像素坐标 + 深度 → 相机坐标系 3D 坐标

        X_cam = (u - cx) * Z / fx
        Y_cam = (v - cy) * Z / fy
        Z_cam = depth

        Args:
            u, v: 像素坐标
            depth_mm: 该点深度值（mm）

        Returns:
            (X, Y, Z) in mm（相机坐标系）
        """
        Z = depth_mm
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return (X, Y, Z)

    # ----------------------------------------------------------
    # Step 3: 获取深度值（从深度图）
    # ----------------------------------------------------------
    def get_depth_at(self, u: float, v: float, depth_image=None) -> float:
        """
        获取某像素点的深度值。

        真实实现：从深度图读取 (u, v) 处的深度值（mm）。
        当前返回模拟深度。
        """
        # TODO: 从真实深度图读取
        # depth_mm = depth_image[int(v), int(u)]
        return 500.0  # 模拟：药盒距离相机约 500mm

    # ----------------------------------------------------------
    # Step 4: 检测货架高度
    # ----------------------------------------------------------
    def detect_shelf_height(self, depth_image=None) -> float:
        """
        检测货架平面在相机坐标系下的高度（Z 坐标）。

        方法：
        - 对深度图做平面拟合（RANSAC）
        - 或直接读取货架区域的深度均值
        - 或检测货架上的 ArUco 码

        Returns:
            货架平面在相机系下的 Z 坐标（mm）
        """
        # TODO: 真实平面检测
        return 450.0  # 模拟：货架距离相机 450mm（药盒在货架上高出 50mm）

    # ----------------------------------------------------------
    # Step 5: 完整检测流程
    # ----------------------------------------------------------
    def detect(self, rgb_image=None, depth_image=None) -> MedicineBox:
        """
        完整检测流程，返回药盒信息。

        Returns:
            MedicineBox 数据类，包含所有检测结果
        """
        print("[Detector] ====== 开始视觉检测 ======")

        # 5.1 检测角点
        corners = self.detect_box_corners(rgb_image)

        # 5.2 计算中心点像素
        cx_pix = sum(c[0] for c in corners) / 4.0
        cy_pix = sum(c[1] for c in corners) / 4.0

        # 5.3 获取中心点深度 → 反投影到 3D
        depth_center = self.get_depth_at(cx_pix, cy_pix, depth_image)
        center_3d = self.pixel_to_3d(cx_pix, cy_pix, depth_center)

        # 5.4 计算旋转角度（上边与水平线的夹角）
        top_left, top_right = corners[0], corners[1]
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        rotation_deg = np.degrees(np.arctan2(dy, dx))

        # 5.5 计算药盒尺寸（基于角点）
        width_pix = np.linalg.norm(np.array(corners[1]) - np.array(corners[0]))
        height_pix = np.linalg.norm(np.array(corners[3]) - np.array(corners[0]))
        scale = depth_center / self.fx  # 像素→mm 比例
        box_w_mm = width_pix * scale
        box_h_mm = height_pix * scale

        # 5.6 检测货架高度
        shelf_z = self.detect_shelf_height(depth_image)

        box = MedicineBox(
            corners=corners,
            center_pixel=(cx_pix, cy_pix),
            center_3d=center_3d,
            rotation_deg=rotation_deg,
            shelf_height_mm=shelf_z,
            box_size=(box_w_mm, box_h_mm),
        )

        print(f"[Detector] 药盒中心 3D (相机系):  {center_3d}")
        print(f"[Detector] 旋转角度:            {rotation_deg:.1f}°")
        print(f"[Detector] 药盒尺寸:            {box_w_mm:.1f} x {box_h_mm:.1f} mm")
        print(f"[Detector] 货架高度 Z:          {shelf_z:.1f} mm")
        print("[Detector] ====== 检测完成 ======")

        return box


# ============================================================
# 模拟检测（无真实相机时使用）
# ============================================================
def simulate_detection() -> MedicineBox:
    """无真实相机时的模拟数据"""
    return MedicineBox(
        corners=[(220, 160), (420, 160), (420, 260), (220, 260)],
        center_pixel=(320, 210),
        center_3d=(-50.0, -30.0, 500.0),   # 相机右前下方 50cm
        rotation_deg=0.0,
        shelf_height_mm=450.0,              # 货架在 450mm 处
        box_size=(100.0, 50.0),             # 药盒 100x50mm
    )
