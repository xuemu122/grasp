#!/usr/bin/env python3
"""
手眼标定 (Eye-in-Hand)
======================
相机固定在机械臂法兰上 → 标定 法兰→相机 的 6-DOF 变换矩阵

流程:
  1. 打印 ArUco 标定板贴墙上 (脚本会自动生成)
  2. 机械臂对着标定板, 按 Enter 采集一组 (法兰位姿 + 相机看到 marker 的位姿)
  3. 换姿势重复 10-15 组 (绕 Z 倾斜、前后移动效果最好)
  4. 按 q 求解并保存 calib.yaml

原理 (AX=XB):
  A_i = T_base→flange_i          (机械臂正运动学)
  B_i = T_camera→marker_i        (相机识别 marker)
  X   = T_flange→camera          (标定目标)
  方程: A_i * X * B_i = 固定 → A_j⁻¹*A_i * X = X * B_j*B_i⁻¹

所需: opencv-contrib-python (ArUco)
"""

import sys
import os
import socket
import re
import time
import json

import cv2
import numpy as np

sys.path.insert(0, "/home/nvidia/grasp/识别")
from orbbec_detect import RealSenseCamera

DOBOT_IP = os.getenv("DOBOT_IP", "192.168.1.6")
DOBOT_PORT = 29999
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib_eye_in_hand.json")

# ArUco 参数
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_ID = 42
MARKER_SIZE_MM = 53  # 打印后量一下 marker 黑框实际边长 (mm)，改这里


# ============================================================
def print_aruco_marker():
    """生成 ArUco 标定板图片"""
    dict_obj = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    img = np.zeros((600, 600), dtype=np.uint8)
    img = cv2.aruco.generateImageMarker(dict_obj, MARKER_ID, 500, img, 1)
    img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    out = os.path.join(os.path.dirname(__file__), "aruco_marker.png")
    cv2.imwrite(out, img)
    # 也保存到大图方便打印
    a4_path = os.path.join(os.path.dirname(__file__), "aruco_marker_print.png")
    img_a4 = np.ones((1754, 1240, 3), dtype=np.uint8) * 255
    marker_resized = cv2.resize(img, (500, 500))
    img_a4[600:1100, 370:870] = cv2.cvtColor(marker_resized, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(a4_path, img_a4)
    print(f"\n[标定板] 已生成:")
    print(f"  原始: {out} (可直接打开查看)")
    print(f"  打印用: {a4_path} (放纸上打印)")
    print(f"  打印出来后请用尺子量一下 marker 黑框边长 (mm)，确认是 {MARKER_SIZE_MM}mm")
    print(f"  如果不是 {MARKER_SIZE_MM}mm，请改脚本里的 MARKER_SIZE_MM")
    return out


def rpy_to_rot(rx_deg, ry_deg, rz_deg):
    """RPY (度) → 3x3 旋转矩阵 (extrinsic ZYX)"""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz),  np.cos(rz), 0],
                    [0, 0, 1]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx),  np.cos(rx)]])
    return Rz @ Ry @ Rx


def flange_pose_to_4x4(pose):
    """法兰位姿 (x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg) → 4x4 齐次矩阵"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(pose[3], pose[4], pose[5])
    T[:3, 3] = [pose[0] / 1000, pose[1] / 1000, pose[2] / 1000]
    return T


class DobotClient:
    """带自动重连的 DOBOT TCP 客户端"""
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self._connect()

    def _connect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.sock.connect((self.ip, self.port))
        print(f"[DOBOT] 已连接 {self.ip}:{self.port}")

    def cmd(self, msg, timeout=3.0):
        """发送命令, 如果管道断开则自动重连一次"""
        if not msg.endswith("\n"):
            msg += "\n"
        for attempt in range(2):
            try:
                self.sock.sendall(msg.encode())
                self.sock.settimeout(timeout)
                resp = b""
                while True:
                    chunk = self.sock.recv(65536)
                    if not chunk:
                        break
                    resp += chunk
                    if b";" in chunk or b"}" in chunk:
                        break
                self.sock.settimeout(5)
                return resp.decode().strip()
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                if attempt == 0:
                    print(f"  [重连] {e}")
                    self._connect()
                else:
                    raise

    def get_pose(self):
        resp = self.cmd("GetPose()")
        nums = re.findall(r"[-+]?\d*\.?\d+", resp)
        if len(nums) >= 6:
            return tuple(map(float, nums[-6:]))
        raise RuntimeError(f"无法解析位姿: {resp}")

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


# ============================================================
def hand_eye_calib(A_list, B_list):
    """求解 AX=XB (OpenCV Tsai 方法)
    A_list: [4x4] T_base←flange (法兰位姿, gripper→base)
    B_list: [4x4] T_marker←camera 即 solvePnP 返回值 (target→cam)
    返回: X (4x4) T_flange←camera (相机在法兰坐标系下)
    """
    R_gripper2base = []   # flange → base
    t_gripper2base = []
    R_target2cam = []     # marker → camera
    t_target2cam = []

    for A, B in zip(A_list, B_list):
        # A = T_base←flange (gripper→base), 直接就是 OpenCV 要的 gripper2base
        R_gripper2base.append(A[:3, :3])
        t_gripper2base.append(A[:3, 3])
        # B = T_marker→camera (target→cam), 直接就是 OpenCV 要的 target2cam
        R_target2cam.append(B[:3, :3])
        t_target2cam.append(B[:3, 3])

    # OpenCV calibrateHandEye 返回 camera→flange (相机在法兰系下)
    # 我们要求的是 flange→camera = inv(camera→flange)
    R_cam2flange, t_cam2flange = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    T_cam2flange = np.eye(4)
    T_cam2flange[:3, :3] = R_cam2flange
    T_cam2flange[:3, 3] = t_cam2flange.flatten()
    T_flange2cam = np.linalg.inv(T_cam2flange)
    return T_flange2cam


# ============================================================
def main():
    print("=" * 60)
    print("  手眼标定 (Eye-in-Hand)")
    print("=" * 60)

    # 生成 marker
    marker_path = print_aruco_marker()

    print(f"\n准备工作:")
    print(f"  1. 打印 {marker_path} (边长约 {MARKER_SIZE_MM}mm)")
    print(f"  2. 贴在墙上固定")
    print(f"  3. 连接机械臂 + RealSense")

    # ── 连接 DOBOT ──
    robot = DobotClient(DOBOT_IP, DOBOT_PORT)
    robot.cmd("ClearError()")
    robot.cmd("RequestControl()")
    robot.cmd("PowerOn()")
    print(f"\n[DOBOT] 上电中 (等待 10 秒)...")
    time.sleep(10)
    robot.cmd("EnableRobot()")
    print(f"[DOBOT] 已使能")

    # ── 相机 ──
    camera = RealSenseCamera(width=640, height=480, fps=30)
    if camera is None:
        print("相机未就绪")
        return
    # 读内参
    fx, fy = camera.fx, camera.fy
    cx, cy = camera.cx, camera.cy
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # RealSense 内参已去畸变

    # ArUco 检测器
    dict_obj = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dict_obj, aruco_params)

    A_list, B_list = [], []

    print("\n" + "-" * 60)
    print("  采集流程:")
    print("  1. 拖动机械臂让 marker 在画面中央")
    print("  2. marker 要清晰 (别太远/太倾斜)")
    print("  3. 按 Enter 采集一组")
    print("  4. 换姿势重复 10-15 组")
    print("  5. 采集完按 q 求解")
    print("  tips: 绕 Z 旋转 + 改变深度 效果最好")
    print("-" * 60)

    try:
        while True:
            cmd = input(f"\n[{len(A_list)} 组] 按 Enter 采集 | q 求解 > ").strip()
            if cmd.lower() == "q":
                if len(A_list) < 4:
                    print("至少需要 4 组数据!")
                    continue
                break

            # 读法兰位姿
            pose = robot.get_pose()
            A = flange_pose_to_4x4(pose)
            print(f"  法兰: ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f})mm "
                  f"rpy=({pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")

            # 拍照
            for _ in range(5):
                camera.read()  # 暖机
            time.sleep(0.1)
            color, _ = camera.read()
            if color is None:
                print("  相机未就绪")
                continue

            # ArUco 检测
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            found = False
            if ids is not None and MARKER_ID in ids:
                idx = list(ids.flatten()).index(MARKER_ID)
                # ArUco 角点顺序: 左上→右上→右下→左下 (OpenCV 约定)
                s = MARKER_SIZE_MM / 1000.0 / 2
                obj_pts = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]], dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(obj_pts, corners[idx][0], camera_matrix, dist_coeffs)
                # solvePnP 返回 T_marker→camera (marker 在相机系下的位姿)
                if ok:
                    R, _ = cv2.Rodrigues(rvec)
                    B = np.eye(4)
                    B[:3, :3] = R
                    B[:3, 3] = tvec.flatten()
                    found = True

            if not found:
                print("  ⚠ 未检测到 ArUco marker, 跳过")
                cv2.imwrite(f"/tmp/calib_fail_{len(A_list)}.jpg", color)
                continue

            A_list.append(A)
            B_list.append(B)
            print(f"  ✓ 第 {len(A_list)} 组: 法兰→相机")
            print(f"    平移 t=({B[0,3]:.3f},{B[1,3]:.3f},{B[2,3]:.3f})m")

    finally:
        robot.cmd("DisableRobot()")
        robot.close()
        camera.release()

    if len(A_list) < 4:
        print("数据不足, 退出")
        return

    # ── 求解 ──
    print(f"\n求解 AX=XB ({len(A_list)} 组数据)...")
    X = hand_eye_calib(A_list, B_list)

    # 提取 RPY
    R = X[:3, :3]
    # 转 ZYX 欧拉角 (外旋)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0
    t = X[:3, 3]

    print(f"\n标定结果 T_flange→camera:")
    print(f"  平移 (m):  [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
    print(f"  平移 (mm): [{t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f}]")
    print(f"  RPY (deg): [{np.rad2deg(rx):.1f}, {np.rad2deg(ry):.1f}, {np.rad2deg(rz):.1f}]")
    print(f"\n  完整矩阵:")
    print(np.array2string(X, precision=4, suppress_small=True))

    # 保存
    result = {
        "method": "hand_eye_tsai",
        "T_flange_to_camera": X.tolist(),
        "translation_m": [float(t[0]), float(t[1]), float(t[2])],
        "rpy_deg": [float(np.rad2deg(rx)), float(np.rad2deg(ry)), float(np.rad2deg(rz))],
        "num_samples": len(A_list),
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\n✓ 已保存到: {CALIB_FILE}")
    print(f"  shelf_grasp.py 会自动加载此文件")


if __name__ == "__main__":
    main()
