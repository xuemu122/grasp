#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot Nova5 机械臂控制脚本：移动到预定位置 → 等待 5 秒 → 回到 home 位置

使用 Dobot 自定义 ROS2 服务（MovJ）进行关节空间运动。
home 位姿定义来自 nova5_moveit/config/nova5_robot.srdf（已转换为角度）。
"""

import sys
import time
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import EnableRobot, SpeedFactor, MovJ


class DobotController(Node):
    """Dobot Nova5 机械臂控制器"""

    def __init__(self, name="dobot_controller"):
        super().__init__(name)

        # ------ 创建 service 客户端 ------
        self.enable_robot_cli = self.create_client(
            EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')
        self.speed_factor_cli = self.create_client(
            SpeedFactor, '/dobot_bringup_ros2/srv/SpeedFactor')
        self.movj_cli = self.create_client(
            MovJ, '/dobot_bringup_ros2/srv/MovJ')

        self.get_logger().info("等待 EnableRobot 服务可用...")
        while not self.enable_robot_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("服务不可用，继续等待...")
        self.get_logger().info("服务已连接！")

        # ------ home 位姿（来自 SRDF，单位：度）------
        # SRDF 中弧度值：j2=0.5378, j3=-1.1869, j5=1.7785, 其余=0
        self.home_position = {
            "joint1": 0.0,
            "joint2": 30.82,    # 0.5378 rad
            "joint3": -68.00,   # -1.1869 rad
            "joint4": 0.0,
            "joint5": 101.90,   # 1.7785 rad
            "joint6": 0.0,
        }

        # ------ 预定目标位姿（可自定义，单位：度）------
        self.target_position = {
            "joint1": 30.0,
            "joint2": 15.0,
            "joint3": -40.0,
            "joint4": 0.0,
            "joint5": 60.0,
            "joint6": 0.0,
        }

    def enable_robot(self):
        """使能机器人"""
        self.get_logger().info("正在使能机器人...")
        req = EnableRobot.Request()
        future = self.enable_robot_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f"使能结果: {result}")

    def set_speed(self, ratio: int):
        """
        设置全局速度比率

        Args:
            ratio: 速度百分比，范围 1-100
        """
        self.get_logger().info(f"设置速度比率为: {ratio}%")
        req = SpeedFactor.Request()
        req.ratio = ratio
        future = self.speed_factor_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"速度设置结果: {future.result()}")

    def movj(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0):
        """
        关节空间运动（MovJ）

        Args:
            j1~j6: 各关节目标角度（单位：度）
        """
        self.get_logger().info(
            f"MovJ → j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, "
            f"j4={j4:.2f}, j5={j5:.2f}, j6={j6:.2f}"
        )
        req = MovJ.Request()
        req.mode = True          # True = 关节空间模式
        req.a = float(j1)        # joint1 角度（度）
        req.b = float(j2)        # joint2 角度（度）
        req.c = float(j3)        # joint3 角度（度）
        req.d = float(j4)        # joint4 角度（度）
        req.e = float(j5)        # joint5 角度（度）
        req.f = float(j6)        # joint6 角度（度）
        future = self.movj_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"MovJ 执行结果: {future.result()}")

    def move_to_position(self, pos: dict):
        """移动到指定关节位置"""
        self.movj(
            j1=pos["joint1"], j2=pos["joint2"], j3=pos["joint3"],
            j4=pos["joint4"], j5=pos["joint5"], j6=pos["joint6"],
        )


def main(args=None):
    rclpy.init(args=args)
    controller = DobotController("move_and_home_demo")

    # ---------- 初始化 ----------
    controller.enable_robot()
    controller.set_speed(20)          # 20% 速度，安全起见

    # ---------- Step 1: 移动到预定目标位置 ----------
    controller.get_logger().info(
        "==========================================")
    controller.get_logger().info(
        "Step 1: 移动到预定目标位置")
    controller.get_logger().info(
        "==========================================")
    controller.move_to_position(controller.target_position)

    # ---------- Step 2: 等待 5 秒 ----------
    controller.get_logger().info(
        "==========================================")
    controller.get_logger().info(
        "Step 2: 在目标位置等待 5 秒...")
    for i in range(5, 0, -1):
        controller.get_logger().info(f"  倒计时: {i} 秒")
        time.sleep(1)
    controller.get_logger().info(
        "==========================================")

    # ---------- Step 3: 回到 home 位置 ----------
    controller.get_logger().info(
        "Step 3: 回到 home 位置")
    controller.get_logger().info(
        "==========================================")
    controller.move_to_position(controller.home_position)

    # ---------- 完成 ----------
    controller.get_logger().info(
        "==========================================")
    controller.get_logger().info(
        "任务完成！机械臂已回到 home 位置。")

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
