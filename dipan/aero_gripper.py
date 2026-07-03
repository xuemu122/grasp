#!/usr/bin/env python3
"""Aero Hand gripper wrapper."""

import sys


AERO_SDK_PATH = "/home/nvidia/grasp/aero-hand-open/sdk/src"
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand


class AeroGripper:
    OPEN = [80, 0, 0, 0, 0, 0, 0]
    GRASP = [80, 70, 52, 70, 58, 59, 53]
    GRASP_TOP = [100, 63, 7, 71, 68, 50, 62]
    GRASP_BOTTLE = [100, 92, 75, 76, 75, 67, 69]

    def __init__(self, port=None):
        self.hand = AeroHand(port=port)
        self.ok = True

    def connect(self):
        print("[Aero] 已连接")

    def disconnect(self):
        if not self.ok:
            print("[Aero] 已禁用, 跳过断开")
            return
        try:
            self.hand.close()
            print("[Aero] 已断开")
        except Exception as e:
            self.ok = False
            print(f"[Aero] 断开失败, 已禁用灵巧手: {e}")

    def open(self):
        if not self.ok:
            print("[Aero] 已禁用, 跳过张开")
            return False
        try:
            self.hand.set_joint_positions(self.OPEN)
            print("[Aero] 张开")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 张开失败, 已禁用灵巧手: {e}")
            return False

    def close(self, top=False, mode=None):
        if not self.ok:
            print("[Aero] 已禁用, 跳过抓取")
            return False
        try:
            if mode == "bottle":
                pose = self.GRASP_BOTTLE
                label = "瓶子抓取"
            elif top:
                pose = self.GRASP_TOP
                label = "上端抓取"
            else:
                pose = self.GRASP
                label = "抓取"
            self.hand.set_joint_positions(pose)
            print(f"[Aero] {label}")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 抓取失败, 已禁用灵巧手: {e}")
            return False
