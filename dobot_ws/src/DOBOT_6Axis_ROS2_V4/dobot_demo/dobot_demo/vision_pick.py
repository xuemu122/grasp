#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉引导抓取主控节点
完整流程：相机检测 → 坐标变换 → 运动到药盒 → 抓取 → 放置

=============================================================
架构概览
=============================================================

    ┌──────────────┐     ┌──────────────────┐     ┌──────────────┐
    │  相机/视觉    │────▶│  坐标变换模块      │────▶│  Dobot 运动  │
    │  (检测药盒)   │     │  相机系→机器人系   │     │  MovL/MovJ   │
    └──────────────┘     └──────────────────┘     └──────────────┘
                                                         │
                                                         ▼
                                                  ┌──────────────┐
                                                  │  夹爪控制     │
                                                  │  DO/ToolDO   │
                                                  └──────────────┘

=============================================================
坐标系说明
=============================================================

   相机坐标系 (cam):  Z 沿光轴前向, X 右, Y 下
   机器人基座系 (base): Z 上, X 前, Y 左
   工具坐标系 (tool):  TCP 点，默认法兰中心

   变换关系: P_base = T_cam2base * P_cam

=============================================================
运动策略
=============================================================

   1. [安全高度]  先移动到药盒正上方 50mm（预抓取位姿）
   2. [缓慢接近]  MovL 直线下移到抓取高度
   3. [抓取]       闭合夹爪
   4. [提升]       MovL 直线上升到安全高度
   5. [放置]       MovJ 移动到放置位置
   6. [释放]       打开夹爪

=============================================================
使用前准备（手眼标定）
=============================================================

   运行前需要标定 T_cam2base 矩阵，方法：
   - 眼在手外 (Eye-to-hand): 相机固定安装, 标定相机在机器人基座下的位姿
   - 眼在手上 (Eye-in-hand): 相机装在机械臂末端, 通过 MoveIt/手眼标定计算

   建议使用 ArUco 码贴在药盒上提高检测精度。
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import (
    EnableRobot, SpeedFactor,
    MovJ, MovL,
    DO,
    GetPose,
)


# ============================================================
# 手眼标定矩阵（相机 → 机器人基座）
# ============================================================
# 此矩阵需通过手眼标定获得。当前为示例矩阵，假设：
#   - 相机安装在机器人前方上方
#   - 看向工作台
# ============================================================
class HandEyeCalibration:
    """
    手眼标定：维护相机到机器人基座的变换矩阵。

    眼在手外 (Eye-to-hand) 模式下的示例：
    相机在机器人前方 800mm，高度 600mm，俯视 30°
    """

    def __init__(self):
        # ---------- 请根据实际标定结果替换此矩阵 ----------
        # 示例：相机在 (x=600, y=0, z=400) mm，俯视 45°
        #     绕 Y 轴旋转 -135° → 看向工作台
        self.T_cam2base = np.eye(4)

        # 平移部分（相机原点在机器人基座下的位置）
        tx, ty, tz = 600.0, 0.0, 400.0  # mm

        # 旋转部分（相机 Z→前, X→右, Y→下）
        # 机器人基座: Z→上, X→前, Y→左
        # 相机相对机器人旋转 (俯视角度)
        pitch = math.radians(-45)  # 俯视 -45° (绕新 Y)
        yaw = math.radians(0)       # 无偏航

        # 构建旋转矩阵（先绕 Z 再绕 Y）
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0,              0,             1],
        ])
        Ry = np.array([
            [math.cos(pitch),  0, math.sin(pitch)],
            [0,                1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ])
        R = Rz @ Ry  # 组合旋转

        self.T_cam2base[:3, :3] = R
        self.T_cam2base[:3, 3] = [tx, ty, tz]

        print("[Calib] 手眼标定矩阵已加载")
        print(f"[Calib] 平移: [{tx}, {ty}, {tz}] mm")
        print(f"[Calib] 俯仰角: {math.degrees(pitch):.1f}°")

    def cam_to_base(self, point_cam: tuple) -> np.ndarray:
        """
        将相机坐标系下的点转换到机器人基座坐标系

        Args:
            point_cam: (X_cam, Y_cam, Z_cam) in mm

        Returns:
            [X_base, Y_base, Z_base] in mm
        """
        P_cam = np.array([point_cam[0], point_cam[1], point_cam[2], 1.0])
        P_base = self.T_cam2base @ P_cam
        return P_base[:3]

    def rotation_cam_to_base_rz(self, cam_rz_deg: float) -> float:
        """
        将相机系下的 RZ 旋转角度转换到机器人基座系

        Args:
            cam_rz_deg: 相机系下的 RZ（度）

        Returns:
            机器人基座系下的 RZ（度），可直接给 MovL.d
        """
        # 简化处理：相机俯视时，相机 RZ ≈ 机器人 RZ 取反
        # 实际应根据标定矩阵精确计算
        return -cam_rz_deg


# ============================================================
# 视觉引导抓取控制器
# ============================================================
class VisionPickController(Node):
    """
    视觉引导抓取主控制器

    使用 MovL 笛卡尔模式直线运动到检测位置
    """

    def __init__(self, name="vision_pick_controller"):
        super().__init__(name)

        # ----- 手眼标定 -----
        self.calib = HandEyeCalibration()

        # ----- Service 客户端 -----
        self._wait_and_create_clients()

        # ----- 运动参数 -----
        self.safe_approach_z = 50.0      # 预抓取点距药盒上表面安全高度 (mm)
        self.grasp_offset_z = 3.0        # 抓取时 TCP 距药盒顶的额外偏移 (mm)
        self.speed_ratio = 30            # 默认速度 30%
        self.slow_speed = 10             # 接近物体时慢速 10%

        self.get_logger().info("视觉引导抓取控制器已就绪")

    def _wait_and_create_clients(self):
        """创建并等待所有 Service 客户端"""
        self.enable_cli = self.create_client(
            EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.speed_cli = self.create_client(
            SpeedFactor, '/dobot_bringup_ros2/srv/SpeedFactor')
        self.movj_cli = self.create_client(
            MovJ, '/dobot_bringup_ros2/srv/MovJ')
        self.movl_cli = self.create_client(
            MovL, '/dobot_bringup_ros2/srv/MovL')
        self.do_cli = self.create_client(
            DO, '/dobot_bringup_ros2/srv/DO')
        self.getpose_cli = self.create_client(
            GetPose, '/dobot_bringup_ros2/srv/GetPose')

        self.get_logger().info("等待 Dobot 服务...")
        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("  等待 EnableRobot...")
        self.get_logger().info("所有服务已连接！")

    # ============================================================
    # 底层控制 API
    # ============================================================

    def _call_async(self, cli, req, log_prefix=""):
        """统一的异步调用封装"""
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if hasattr(result, 'res') and result.res != 0:
            self.get_logger().warning(
                f"{log_prefix} 返回错误码: {result.res}")
        return result

    def enable_robot(self):
        self.get_logger().info("使能机器人...")
        self._call_async(self.enable_cli, EnableRobot.Request(), "使能")

    def set_speed(self, ratio: int):
        self.get_logger().info(f"设置速度: {ratio}%")
        req = SpeedFactor.Request()
        req.ratio = ratio
        self._call_async(self.speed_cli, req, "设置速度")

    def get_current_pose(self) -> tuple:
        """获取当前 TCP 位姿 (x,y,z,rx,ry,rz)"""
        req = GetPose.Request()
        req.user = "-1"
        req.tool = "-1"
        future = self.getpose_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        # 返回值格式: {x,y,z,rx,ry,rz}
        ret = result.robot_return.strip("{}")
        vals = [float(v) for v in ret.split(",")]
        return tuple(vals)

    def movj(self, j1, j2, j3, j4, j5, j6):
        """关节运动 (角度, mode=true)"""
        self.get_logger().info(
            f"MovJ → [{j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}, {j5:.1f}, {j6:.1f}]")
        req = MovJ.Request()
        req.mode = True
        req.a, req.b, req.c, req.d, req.e, req.f = (
            float(j1), float(j2), float(j3),
            float(j4), float(j5), float(j6),
        )
        self._call_async(self.movj_cli, req, "MovJ")

    def movl_pose(self, x, y, z, rx, ry, rz):
        """
        笛卡尔直线运动 (单位: mm, 度, mode=false)

        Args:
            x, y, z: 目标位置 (mm, 机器人基座系)
            rx, ry, rz: 目标姿态 RPY (度)
        """
        self.get_logger().info(
            f"MovL → pos=({x:.1f}, {y:.1f}, {z:.1f}) mm, "
            f"rpy=({rx:.1f}, {ry:.1f}, {rz:.1f})°"
        )
        req = MovL.Request()
        req.mode = False           # Cartesian/pose 模式
        req.a, req.b, req.c = float(x), float(y), float(z)
        req.d, req.e, req.f = float(rx), float(ry), float(rz)
        self._call_async(self.movl_cli, req, "MovL")

    def control_gripper(self, status: int, index: int = 1):
        """
        控制夹爪

        Args:
            status: 1=闭合, 0=打开
            index: DO 端口索引
        """
        act = "闭合" if status else "打开"
        self.get_logger().info(f"夹爪 {act} (DO{index})")
        req = DO.Request()
        req.index = index
        req.status = status
        self._call_async(self.do_cli, req, f"DO{index}")

    # ============================================================
    # 高层抓取流程
    # ============================================================

    def pick_medicine_box(self,
                          center_3d_cam: tuple,
                          rotation_cam_deg: float,
                          shelf_height_cam_mm: float,
                          box_size_mm: tuple):
        """
        完整的视觉引导抓取流程

        Args:
            center_3d_cam:     药盒中心在相机系下的 3D 坐标 (mm)
            rotation_cam_deg:  药盒在相机系下的旋转角度（度）
            shelf_height_cam_mm: 货架高度（相机系 Z）
            box_size_mm:       药盒尺寸 (width, height) mm
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("开始视觉引导抓取任务")
        self.get_logger().info("=" * 50)

        # --------------------------------------------------------
        # Step 1: 坐标变换（相机系 → 机器人基座系）
        # --------------------------------------------------------
        self.get_logger().info("[Step 1] 坐标变换: 相机系 → 机器人基座系")

        # 1.1 药盒中心
        target_base = self.calib.cam_to_base(center_3d_cam)
        tx, ty, tz = target_base[0], target_base[1], target_base[2]

        # 1.2 货架高度
        shelf_pos = self.calib.cam_to_base(
            (center_3d_cam[0], center_3d_cam[1], shelf_height_cam_mm))
        shelf_z = float(shelf_pos[2])

        # 1.3 旋转角度
        base_rz = self.calib.rotation_cam_to_base_rz(rotation_cam_deg)

        self.get_logger().info(
            f"  目标位置 (base): ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm")
        self.get_logger().info(f"  货架高度 (base Z): {shelf_z:.1f} mm")
        self.get_logger().info(f"  目标 RZ: {base_rz:.1f}°")

        # 抓取高度 = 货架高度 + 药盒高度/2 + 微小偏移
        grasp_z = shelf_z + box_size_mm[1] / 2.0 + self.grasp_offset_z
        # 预抓取高度 = 抓取高度 + 安全偏移
        pre_grasp_z = grasp_z + self.safe_approach_z

        self.get_logger().info(
            f"  预抓取 Z: {pre_grasp_z:.1f} mm, 抓取 Z: {grasp_z:.1f} mm")

        # --------------------------------------------------------
        # Step 2: 快速移动到预抓取位姿（药盒正上方）
        # --------------------------------------------------------
        self.get_logger().info("[Step 2] 移动到预抓取位姿（药盒正上方）")
        self.set_speed(self.speed_ratio)
        # 末端保持垂直（rx=180, ry=0 或 rx=0, ry=180 表示夹爪朝下）
        self.movl_pose(tx, ty, pre_grasp_z, rx=180.0, ry=0.0, rz=base_rz)

        # --------------------------------------------------------
        # Step 3: 慢速直线下移到抓取高度
        # --------------------------------------------------------
        self.get_logger().info("[Step 3] 缓慢接近 → 抓取高度")
        self.set_speed(self.slow_speed)
        self.movl_pose(tx, ty, grasp_z, rx=180.0, ry=0.0, rz=base_rz)

        # --------------------------------------------------------
        # Step 4: 闭合夹爪
        # --------------------------------------------------------
        self.get_logger().info("[Step 4] 闭合夹爪")
        self.control_gripper(1)  # 闭合
        time.sleep(0.5)          # 等待夹爪完成动作

        # --------------------------------------------------------
        # Step 5: 提升到安全高度
        # --------------------------------------------------------
        self.get_logger().info("[Step 5] 提升到安全高度")
        self.movl_pose(tx, ty, pre_grasp_z, rx=180.0, ry=0.0, rz=base_rz)

        # --------------------------------------------------------
        # Step 6: 移动到放置位置
        # --------------------------------------------------------
        self.get_logger().info("[Step 6] 移动到放置位置")
        self.set_speed(self.speed_ratio)
        # 放置位置可自定义（示例：向前移动 200mm）
        place_x, place_y, place_z = tx + 200.0, ty, pre_grasp_z
        self.movl_pose(place_x, place_y, place_z, rx=180.0, ry=0.0, rz=0.0)

        # --------------------------------------------------------
        # Step 7: 打开夹爪释放药盒
        # --------------------------------------------------------
        self.get_logger().info("[Step 7] 释放药盒")
        self.control_gripper(0)  # 打开

        # --------------------------------------------------------
        # 完成
        # --------------------------------------------------------
        self.get_logger().info("=" * 50)
        self.get_logger().info("视觉引导抓取任务完成！")
        self.get_logger().info("=" * 50)


# ============================================================
# 主函数
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    controller = VisionPickController()

    # 使能 + 设置速度
    controller.enable_robot()
    controller.set_speed(30)

    # ---------- 视觉检测 ----------
    from dobot_demo.vision_detector import (
        MedicineBoxDetector, simulate_detection,
    )

    # 方式 1: 使用真实检测器（需接入相机）
    # detector = MedicineBoxDetector()
    # box = detector.detect(rgb_image, depth_image)

    # 方式 2: 使用模拟数据（演示）
    print("\n⚠️  使用模拟视觉数据（无真实相机）\n")
    box = simulate_detection()

    # ---------- 执行抓取 ----------
    controller.pick_medicine_box(
        center_3d_cam=box.center_3d,
        rotation_cam_deg=box.rotation_deg,
        shelf_height_cam_mm=box.shelf_height_mm,
        box_size_mm=box.box_size,
    )

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
