#!/usr/bin/env python3
"""
Nova 5 J6 关节旋转 Demo
=======================
末端灵巧手: 长度 35cm / 重量 1.2kg / 重心距法兰 ~10cm
功能: 使能 -> 设置负载 -> J6 旋转 +90° -> J6 转回原位

使用 Dobot 原生 MovJ 服务，不依赖 control_msgs（ARM 上未安装）。
"""

import math
import time
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dobot_msgs_v4.srv import MovJ, SetPayload, EnableRobot, PowerOn


# ============================================================
# 末端工具参数
# ============================================================
TOOL_LENGTH = 0.35       # 灵巧手长度 (m)
TOOL_MASS = 1.2           # 灵巧手重量 (kg)
TOOL_COG = 0.10           # 重心距法兰面距离 (m)


class J6RotateDemo(Node):
    """Nova 5 关节 6 旋转演示节点"""

    def __init__(self):
        super().__init__("j6_rotate_demo")

        # 实际服务名格式: /dobot_bringup_ros2/srv/XXX
        self.srv_prefix = "/dobot_bringup_ros2/srv"

        self.current_joints = None

        # 订阅关节状态 (dobot bringup 发布在 joint_states_robot)
        for topic in ["joint_states_robot", "joint_states"]:
            self._joint_sub = self.create_subscription(
                JointState, topic, self._joint_callback, 10
            )
            self.get_logger().info(f"已订阅关节状态: {topic}")

        self._print_tool_info()

    def _print_tool_info(self):
        print("\n" + "=" * 50)
        print("  Nova 5 — 末端灵巧手参数")
        print(f"  长度: {TOOL_LENGTH * 100:.0f} cm")
        print(f"  重量: {TOOL_MASS} kg")
        print(f"  重心距法兰: {TOOL_COG * 100:.0f} cm")
        print("=" * 50 + "\n")

    def _joint_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return
        self.current_joints = list(msg.position[:6])

    def get_current_joints(self, timeout: float = 5.0) -> list:
        """阻塞等待拿到当前关节角度 (deg)"""
        self.get_logger().info("读取当前关节角度...")
        start = time.time()
        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error(
                    "无法获取关节状态! 请确认 dobot_bringup 已启动且机械臂已连接"
                )
                sys.exit(1)
        joints_deg = [math.degrees(j) for j in self.current_joints]
        self.get_logger().info(
            f"当前关节: J1={joints_deg[0]:.1f} J2={joints_deg[1]:.1f} "
            f"J3={joints_deg[2]:.1f} J4={joints_deg[3]:.1f} "
            f"J5={joints_deg[4]:.1f} J6={joints_deg[5]:.1f}"
        )
        return joints_deg

    def call_service(self, srv_name, srv_type, **kwargs):
        """通用服务调用"""
        full_name = f"{self.srv_prefix}/{srv_name}"
        client = self.create_client(srv_type, full_name)

        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"服务 {full_name} 不可用，跳过")
            return None

        req = srv_type.Request()
        for k, v in kwargs.items():
            setattr(req, k, v)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def enable_robot(self):
        """使能机械臂"""
        self.get_logger().info("正在使能机械臂...")
        rsp = self.call_service("EnableRobot", EnableRobot)
        if rsp is not None:
            self.get_logger().info(f"使能结果: res={rsp.res}")
        time.sleep(0.5)

    def set_payload(self):
        """设置末端负载参数"""
        self.get_logger().info(
            f"设置负载: {TOOL_MASS}kg, 重心 x={TOOL_COG}m y=0 z={TOOL_LENGTH}m"
        )
        rsp = self.call_service(
            "SetPayload", SetPayload,
            load=TOOL_MASS, x=TOOL_COG, y=0.0, z=TOOL_LENGTH
        )
        if rsp is not None:
            self.get_logger().info(f"负载设置结果: res={rsp.res}")
        time.sleep(0.3)

    def movj(self, joints_deg: list):
        """
        MovJ 关节空间运动
        joints_deg: 6 关节角度 (度)
        """
        rsp = self.call_service(
            "MovJ", MovJ,
            mode=True,
            a=float(joints_deg[0]),
            b=float(joints_deg[1]),
            c=float(joints_deg[2]),
            d=float(joints_deg[3]),
            e=float(joints_deg[4]),
            f=float(joints_deg[5]),
            param_value=[""]
        )
        if rsp is not None:
            self.get_logger().info(f"MovJ 结果: {rsp.robot_return}, res={rsp.res}")
        return rsp

    def run(self):
        """主流程"""
        # 1. 使能 + 设置负载
        self.enable_robot()
        self.set_payload()

        # 2. 读取当前关节 (角度制)
        joints_now = self.get_current_joints()
        j6_now = joints_now[5]

        # 3. J6 旋转 +90°
        joints_target = joints_now.copy()
        joints_target[5] = j6_now + 90.0

        print("\n>>> 运动 1/2: J6 旋转 +90° (目标: {:.1f}° -> {:.1f}°) ...".format(
            j6_now, joints_target[5]))
        self.movj(joints_target)
        time.sleep(2.0)

        # 4. J6 转回原位
        joints_back = joints_now.copy()
        joints_back[5] = j6_now

        print("\n>>> 运动 2/2: J6 转回原位 (目标: {:.1f}°) ...".format(j6_now))
        self.movj(joints_back)

        print("\n" + "=" * 50)
        print("  Demo 完成! J6 已转回初始位置")
        print("=" * 50 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = J6RotateDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
