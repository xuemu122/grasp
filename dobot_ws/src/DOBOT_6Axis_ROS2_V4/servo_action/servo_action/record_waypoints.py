#!/usr/bin/env python3
"""
关节轨迹录制工具
=================
用法: 运行后在终端按 'r' 记录当前关节位姿，按 'q' 退出并保存为 YAML 文件。
"""

import math
import time
import os
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


OUTPUT_DIR = "/home/nvidia/dobot_ws/src/DOBOT_6Axis_ROS2_V4/servo_action/servo_action/trajectories"


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")
        self.current_joints = None
        self.waypoints = []

        # 订阅关节状态
        self._sub = self.create_subscription(
            JointState, "joint_states_robot", self._callback, 10
        )
        self.get_logger().info("已订阅 joint_states_robot")
        os.makedirs(OUTPUT_DIR, exist_ok=True)

    def _callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])

    def record(self):
        """记录当前关节位姿"""
        if self.current_joints is None:
            print("[错误] 尚未收到关节数据，请确认 bringup 已启动")
            return
        j = self.current_joints
        degs = [round(math.degrees(x), 1) for x in j]
        wp = {
            "timestamp": time.time(),
            "joints_rad": [round(x, 4) for x in j],
            "joints_deg": degs,
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints)
        print(f"  [记录] 路点 {idx}: J1={degs[0]} J2={degs[1]} J3={degs[2]} "
              f"J4={degs[3]} J5={degs[4]} J6={degs[5]}")

    def save(self):
        """保存路点到 YAML 文件"""
        if not self.waypoints:
            print("[取消] 没有记录任何路点")
            return

        filename = os.path.join(OUTPUT_DIR, f"trajectory_{int(time.time())}.yaml")
        with open(filename, "w", encoding="utf-8") as f:
            f.write(f"# 轨迹路点 — {len(self.waypoints)} 个点\n")
            f.write(f"# 录制时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 单位: joints_deg 为角度制, joints_rad 为弧度制\n\n")
            f.write("waypoints:\n")
            for i, wp in enumerate(self.waypoints):
                f.write(f"  - id: {i + 1}\n")
                f.write(f"    joints: {wp['joints_deg']}\n")

        print(f"\n[保存] 共 {len(self.waypoints)} 个路点 → {filename}\n")


def main():
    rclpy.init()
    node = WaypointRecorder()

    print("\n" + "=" * 50)
    print("  关节轨迹录制工具")
    print("  按 'r' 记录当前位姿  |  按 'q' 保存并退出")
    print("=" * 50 + "\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # 非阻塞读取按键（需要 termios）
            import termios, sys, tty, select
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                dr, _, _ = select.select([sys.stdin], [], [], 0.05)
                if dr:
                    ch = sys.stdin.read(1)
                    if ch.lower() == 'r':
                        node.record()
                    elif ch.lower() == 'q':
                        break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    except KeyboardInterrupt:
        pass

    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
