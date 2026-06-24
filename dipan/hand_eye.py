#!/usr/bin/env python3
"""Hand-eye coordinate transforms for shelf grasping."""

import json
import os

import numpy as np


CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib_eye_in_hand.json")
FX, FY = 390.4, 390.0
CX, CY = 324.5, 241.2


class HandEyeTransformer:
    # 相机在法兰 -X 方向约 12cm, 光轴平行法兰+Z, 相机+X -> 法兰-Y.
    MANUAL_OFFSET_MM = [-120, 0, 0]
    # MANUAL_OFFSET_MM = None

    def __init__(self, fx=FX, fy=FY, cx=CX, cy=CY):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.flange_pose = None
        self.T_flange_to_cam = np.eye(4)
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE) as f:
                calib = json.load(f)
            self.T_flange_to_cam = np.array(calib["T_flange_to_camera"])
        if self.MANUAL_OFFSET_MM is not None:
            self.T_flange_to_cam[:3, 3] = np.array(self.MANUAL_OFFSET_MM) / 1000.0
            rz = np.deg2rad(-90)
            self.T_flange_to_cam[:3, :3] = np.array([
                [np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz),  np.cos(rz), 0],
                [0, 0, 1],
            ])
            print(
                f"[标定] 手动覆盖: 法兰→相机 平移=({self.MANUAL_OFFSET_MM[0]},"
                f"{self.MANUAL_OFFSET_MM[1]},{self.MANUAL_OFFSET_MM[2]})mm, "
                f"rz={np.rad2deg(rz):.0f}°"
            )
        t = self.T_flange_to_cam[:3, 3]
        print(f"[标定] 已加载手眼标定: t=({t[0]*1000:.1f},{t[1]*1000:.1f},{t[2]*1000:.1f})mm")

    def set_flange(self, pose):
        self.flange_pose = pose

    def pixel_to_camera(self, u, v, depth_m):
        """像素 -> 相机坐标系 (X右 Y下 Z前, 米)。"""
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        return np.array([x, y, depth_m], dtype=np.float64)

    def camera_to_robot(self, p_cam, flange_pose=None, debug=False):
        """相机坐标系 -> 机器人基座 (米)。"""
        pose = flange_pose or self.flange_pose

        T_base_flange = np.eye(4)
        T_base_flange[:3, :3] = self._rpy_to_rot(pose[3], pose[4], pose[5])
        T_base_flange[:3, 3] = [pose[0] / 1000, pose[1] / 1000, pose[2] / 1000]
        p_cam_h = np.append(p_cam, 1.0)
        p_w = T_base_flange @ self.T_flange_to_cam @ p_cam_h

        if debug:
            cam_w = T_base_flange @ self.T_flange_to_cam @ np.array([0, 0, 0, 1.0])
            print("\n  [DEBUG 手眼变换]")
            print(
                f"    法兰(基座): ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f})mm  "
                f"rpy=({pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})"
            )
            print(
                "    T_cam→flange (mm): "
                f"({self.T_flange_to_cam[0,3]*1000:.0f}, "
                f"{self.T_flange_to_cam[1,3]*1000:.0f}, "
                f"{self.T_flange_to_cam[2,3]*1000:.0f})"
            )
            print(f"    相机在世界: ({cam_w[0]:.3f},{cam_w[1]:.3f},{cam_w[2]:.3f})m")
            print(f"    盒子在相机: ({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f})m")
            print(f"    盒子在世界: ({p_w[0]:.3f},{p_w[1]:.3f},{p_w[2]:.3f})m")

        return p_w[:3]

    @staticmethod
    def _rpy_to_rot(rx_deg, ry_deg, rz_deg):
        """RPY (度, extrinsic ZYX) -> 3x3 旋转矩阵。"""
        rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz),  np.cos(rz), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                       [0, 1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx),  np.cos(rx)]])
        return Rz @ Ry @ Rx
