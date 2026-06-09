#!/usr/bin/env python3
"""
轨迹播放器 + 灵巧手联动
=======================
手臂走轨迹序列，每个路点触发灵巧手对应预置动作。

用法:
  ros2 run servo_action play_trajectory --ros-args -p sequence:=[1,2,1,2] -p dwell:=3
  python3 play_trajectory.py --sequence 1212 --dwell 3

手动作映射:
  路点 3 → POS1（张开）
  路点 1 → POS2（闭合）
  路点 2 → 不动
"""

import math
import time
import os
import yaml
import sys

import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import MovJ
from dobot_msgs_v4.srv import EnableRobot
from dobot_msgs_v4.srv import ClearError
from dobot_msgs_v4.srv import SpeedFactor
from dobot_msgs_v4.srv import PowerOn
from dobot_msgs_v4.srv import SetTool
from dobot_msgs_v4.srv import SetPayload

SRV_PREFIX = "/dobot_bringup_ros2/srv"
TRAJ_DIR = os.path.join(os.path.dirname(__file__), "trajectories")

# ── 灵巧手预置位置（rad） ──────────────────────────────
POS1 = {  # 张开
    1: 3.8534, 2: 1.4711, 3: 0.0859, 4: 4.9517,
    5: 4.9962, 6: 1.3990, 7: 2.7428, 8: 4.6127,
    9: 3.9730, 10: 1.0999, 11: 2.9268, 12: 2.6845,
    13: 3.2076, 14: 2.0064, 15: 4.1970, 16: 4.7998,
    17: 0.3712,
}

POS2 = {  # 闭合
    1: 5.0207, 2: 2.6615, 3: 0.9833, 4: 5.5822,
    5: 4.1556, 6: 2.6952, 7: 3.6386, 8: 2.7796,
    9: 4.9317, 10: 2.2381, 11: 2.8962, 12: 3.6202,
    13: 3.9807, 14: 3.1631, 15: 4.1433, 16: 5.0499,
    17: 0.3313,
}

HAND_ACTIONS = {3: POS2, 1: POS1}


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__("trajectory_player")

        self.declare_parameter("sequence", [1, 2, 3])
        self.declare_parameter("dwell", 2.0)
        self.declare_parameter("file", "latest")
        self.declare_parameter("use_hand", True)

        self.sequence = self.get_parameter("sequence").value
        self.dwell = self.get_parameter("dwell").value
        self.file_arg = str(self.get_parameter("file").value)
        self.use_hand = self.get_parameter("use_hand").value

        self.hand = None
        if self.use_hand:
            self._init_hand()

    def _init_hand(self):
        """初始化灵巧手连接"""
        try:
            sys.path.insert(0, "/home/nvidia/orca_core1")
            from orca_core import OrcaHand
            self.hand = OrcaHand()
            success, msg = self.hand.connect()
            if success:
                self.hand.enable_torque()
                self.hand.set_control_mode("current_based_position")
                self.hand.set_max_current(self.hand.config.max_current)
                self.get_logger().info(f"灵巧手已连接: {msg}")
            else:
                self.get_logger().warn(f"灵巧手连接失败: {msg}，跳过手部动作")
                self.hand = None
        except Exception as e:
            self.get_logger().warn(f"灵巧手初始化失败: {e}，跳过手部动作")
            self.hand = None

    def hand_move(self, target: dict):
        """灵巧手移动到目标位置"""
        if self.hand is None:
            return
        motor_ids = list(target.keys())
        current = self.hand.get_motor_pos(as_dict=True)
        for step in range(1, 31):
            alpha = step / 30.0
            cmd = {mid: current[mid] + alpha * (target[mid] - current[mid])
                   for mid in motor_ids}
            self.hand._set_motor_pos(cmd)
            time.sleep(0.02)

    def load_waypoints(self):
        if self.file_arg == "latest":
            files = sorted(
                [f for f in os.listdir(TRAJ_DIR) if f.endswith(".yaml")],
                reverse=True,
            )
            if not files:
                self.get_logger().error(f"目录 {TRAJ_DIR} 中没有轨迹文件")
                sys.exit(1)
            filepath = os.path.join(TRAJ_DIR, files[0])
        else:
            filepath = self.file_arg
            if not os.path.isabs(filepath):
                filepath = os.path.join(TRAJ_DIR, filepath)

        with open(filepath, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        wp_map = {}
        for wp in data["waypoints"]:
            wp_map[wp["id"]] = wp["joints"]

        self.get_logger().info(f"已加载 {len(wp_map)} 个路点: {filepath}")
        return wp_map

    def call_service(self, srv_type, srv_name, req=None):
        """通用服务调用"""
        client = self.create_client(srv_type, f"{SRV_PREFIX}/{srv_name}")
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{srv_name} 服务不可用")
            return None
        if req is None:
            req = srv_type.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def init_robot(self):
        """初始化机械臂：使能、清错、设置速度、上电"""
        self.get_logger().info(">>> 机械臂初始化...")

        # 1. EnableRobot
        rsp = self.call_service(EnableRobot, "EnableRobot")
        if rsp is None:
            self.get_logger().error("使能失败")
            return False
        self.get_logger().info(f"EnableRobot res={rsp.res}")

        # 2. ClearError
        rsp = self.call_service(ClearError, "ClearError")
        if rsp is None:
            self.get_logger().error("清错失败")
            return False
        self.get_logger().info(f"ClearError res={rsp.res}")

        # 3. SetTool - 灵巧手工具坐标系 (z=34cm, 即340mm)
        req = SetTool.Request()
        req.index = 1
        req.value = "{0,0,340,0,0,0}"
        rsp = self.call_service(SetTool, "SetTool", req)
        if rsp is None:
            self.get_logger().error("设置工具坐标系失败")
            return False
        self.get_logger().info(f"SetTool res={rsp.res}")

        # 4. SetPayload - 灵巧手负载 (需确认实际重量kg)
        req = SetPayload.Request()
        req.load = 1.2
        req.x = 0.0
        req.y = 0.0
        req.z = 100.0  # 重心大约在工具一半位置
        rsp = self.call_service(SetPayload, "SetPayload", req)
        if rsp is None:
            self.get_logger().error("设置负载失败")
            return False
        self.get_logger().info(f"SetPayload res={rsp.res}")

        # 5. SpeedFactor
        req = SpeedFactor.Request()
        req.ratio = 30
        rsp = self.call_service(SpeedFactor, "SpeedFactor", req)
        if rsp is None:
            self.get_logger().error("设置速度失败")
            return False
        self.get_logger().info(f"SpeedFactor res={rsp.res}")

        # 6. PowerOn
        rsp = self.call_service(PowerOn, "PowerOn")
        if rsp is None:
            self.get_logger().error("上电失败")
            return False
        self.get_logger().info(f"PowerOn res={rsp.res}")

        self.get_logger().info(">>> 初始化完成")
        return True

    def movj(self, joints_deg: list):
        rsp = self.call_service(MovJ, "MovJ", self._make_movj_req(joints_deg))
        if rsp is not None:
            self.get_logger().info(f"MovJ res={rsp.res}")
            return True
        return False

    def _make_movj_req(self, joints_deg: list):
        req = MovJ.Request()
        req.mode = True
        req.a = float(joints_deg[0])
        req.b = float(joints_deg[1])
        req.c = float(joints_deg[2])
        req.d = float(joints_deg[3])
        req.e = float(joints_deg[4])
        req.f = float(joints_deg[5])
        req.param_value = [""]
        return req

    def run(self):
        if not self.init_robot():
            self.get_logger().error("机器人初始化失败，退出")
            return

        waypoints = self.load_waypoints()

        steps = [int(s) for s in self.sequence]
        if not steps:
            self.get_logger().error(f"序列为空: {self.sequence}")
            return

        for s in steps:
            if s not in waypoints:
                self.get_logger().error(f"路点 {s} 不存在! 可用: {list(waypoints.keys())}")
                return

        print("\n" + "=" * 50)
        print(f"  轨迹播放: {' → '.join(str(s) for s in steps)}")
        print(f"  每步驻留: {self.dwell}s")
        print(f"  手部联动: {'✓' if self.hand else '✗'}")
        print("=" * 50 + "\n")

        for i, wp_id in enumerate(steps):
            joints = waypoints[wp_id]
            self.get_logger().info(
                f"[{i+1}/{len(steps)}] 路点 {wp_id}: "
                f"J1={joints[0]} J2={joints[1]} J3={joints[2]} "
                f"J4={joints[3]} J5={joints[4]} J6={joints[5]}"
            )

            # 手臂运动
            self.movj(joints)

            # 手部动作（路点1 延迟1.5s张开，路点3 延迟3s闭合）
            hand_target = HAND_ACTIONS.get(wp_id)
            if hand_target:
                if wp_id == 1:
                    print("  >>> 等待手臂到位 1.5s ...")
                    time.sleep(1.5)
                elif wp_id == 3:
                    print("  >>> 等待手臂到位 3s ...")
                    time.sleep(5.0)
                label = "POS1(张开)" if hand_target is POS1 else "POS2(闭合)"
                print(f"  >>> 灵巧手 → {label}")
                self.hand_move(hand_target)

            if i < len(steps) - 1:
                time.sleep(self.dwell)

        print("\n[完成] 轨迹播放结束\n")

        if self.hand:
            self.hand.disconnect()


def main():
    rclpy.init()
    node = TrajectoryPlayer()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
