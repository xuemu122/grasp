#!/usr/bin/env python3
"""
货架边缘抓取系统
- YOLO 检测盒子 → 选最外侧（深度最小）
- 边缘抓取（bbox 下边缘 = 盒子前缘，而不是中心）
- DOBOT Nova5 TCP 控制 + Aero Hand 灵巧手
"""

import sys
import os
import time
import socket
import re
import json
import cv2
import numpy as np
from typing import List, Tuple, Optional

# ── 识别模块 ──
sys.path.insert(0, "/home/nvidia/grasp/识别")
from orbbec_detect import MedicineLocatorSystem, RealSenseCamera
from orbbec_detect import _assess_lighting

# ── Aero Hand 灵巧手 ──
AERO_SDK_PATH = "/home/nvidia/grasp/aero-hand-open/sdk/src"
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand


# ============================================================
# 参数
# ============================================================
DOBOT_IP = os.getenv("DOBOT_IP", "192.168.1.6")
DOBOT_PORT = 29999

HAND_LENGTH = 0.35      # 法兰 → 指尖 (米)
HAND_WEIGHT = 0.6       # 末端重量 (kg), Aero Hand
HAND_COG_X = -0.15      # 重心 X 偏移 (米)
HAND_COG_Y = 0.14       # 重心 Y 偏移 (米)
HAND_COG_Z = 0.27       # 重心 Z 偏移 (米)
HAND_W_Y = 0.095        # 手 Y 方向宽度 (米)
HAND_W_X = 0.08         # 手 X 方向宽度 (米)

# D405 内参（自动读取，这里是 fallback）
FX, FY = 390.4, 390.0
CX, CY = 324.5, 241.2

# 调试开关: 屏蔽手, 只测试手眼标定 + 机械臂走位
SKIP_HAND = True


# ============================================================
# DOBOT TCP 控制
# ============================================================
class DobotArm:
    def __init__(self, ip=DOBOT_IP, port=DOBOT_PORT):
        self.ip = ip
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3)
        self.sock.connect((self.ip, self.port))
        print(f"[DOBOT] 已连接 {self.ip}:{self.port}")
        # V4 协议: ClearError → RequestControl → PowerOn → EnableRobot
        resp = self.send("ClearError()")
        print(f"[DOBOT] ClearError: {resp}")
        resp = self.send("RequestControl()")
        print(f"[DOBOT] RequestControl: {resp}")
        resp = self.send("PowerOn()")
        print(f"[DOBOT] PowerOn: {resp}  (等待 10 秒上电...)")
        time.sleep(10)
        resp = self.send(f"EnableRobot({HAND_WEIGHT}, {HAND_COG_X*1000:.0f}, {HAND_COG_Y*1000:.0f}, {HAND_COG_Z*1000:.0f})")
        print(f"[DOBOT] EnableRobot: {resp}")
        resp = self.send("RobotMode()")
        print(f"[DOBOT] RobotMode: {resp}")

    def send(self, cmd: str) -> str:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.sock.sendall(cmd.encode())
        resp = b""
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
                if b";" in chunk or b"}" in chunk:
                    break
            except socket.timeout:
                break
        return resp.decode().strip()

    def reconnect(self):
        """断开并重新使能"""
        print("[DOBOT] 重连中...")
        try:
            self.sock.close()
        except:
            pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3)
        self.sock.connect((self.ip, self.port))
        self.send("ClearError()")
        self.send("RequestControl()")
        self.send("PowerOn()")
        print("[DOBOT] 等待上电 10s...")
        time.sleep(10)
        self.send(f"EnableRobot({HAND_WEIGHT}, {HAND_COG_X*1000:.0f}, {HAND_COG_Y*1000:.0f}, {HAND_COG_Z*1000:.0f})")
        resp = self.send("RobotMode()")
        print(f"[DOBOT] 重连后 RobotMode: {resp}")

    def get_pose(self) -> Tuple[float, float, float, float, float, float]:
        """返回 (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)，失败时重试"""
        for attempt in range(3):
            resp = self.send("GetPose()")
            nums = re.findall(r"[-+]?\d*\.?\d+", resp)
            if len(nums) >= 6:
                pose = tuple(map(float, nums[-6:]))
                self._last_pose = pose
                return pose
            print(f"  [GetPose 重试 {attempt+1}]: {resp}")
            time.sleep(0.3)
        if self._last_pose is not None:
            print(f"  [GetPose] 用缓存位姿")
            return self._last_pose
        raise RuntimeError(f"无法解析位姿: {resp}")

    _last_pose = None

    def get_error_id(self) -> str:
        """查询当前报警码，返回错误码字符串"""
        resp = self.send("GetErrorID()")
        return resp

    def clear_alarm(self):
        """清除报警并重新使能"""
        print("[DOBOT] 清除报警并重新使能...")
        self.send("ClearError()")
        time.sleep(0.5)
        # 检查是否还有报警
        err = self.get_error_id()
        print(f"[DOBOT] GetErrorID: {err}")
        self.send("RequestControl()")
        self.send("PowerOn()")
        print("[DOBOT] 等待上电 10s...")
        time.sleep(10)
        self.send(f"EnableRobot({HAND_WEIGHT}, {HAND_COG_X*1000:.0f}, {HAND_COG_Y*1000:.0f}, {HAND_COG_Z*1000:.0f})")
        resp = self.send("RobotMode()")
        print(f"[DOBOT] RobotMode: {resp}")

    def movel(self, x_mm, y_mm, z_mm, rx=None, ry=None, rz=None, speed=100):
        """MovL V4: pose = {x, y, z, rx, ry, rz}，rx/ry/rz=None 则保持当前姿态"""
        try:
            if rx is None:
                pose = self.get_pose()
                rx, ry, rz = pose[3], pose[4], pose[5]
            # 安全兜底: Z 超出 Nova5 绝对工作范围时警告但不截断
            if z_mm > 1100 or z_mm < -300:
                print(f"  [警告] Z={z_mm:.0f}mm 可能超出工作范围, 请注意!")
            cmd = f"MovL(pose = {{{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}}})"
            print(f"  >>> {cmd}")
            resp = self.send(cmd)
            print(f"  <<< {resp}")
            if resp.startswith("-"):
                print(f"  [MovL 被拒绝] 错误码={resp.split(',')[0]}, 可能是位姿不可达")
                return  # -1/-2 等是立即拒绝, 不是报警, 不调 clear_alarm
            # 命令已被接受(0,...), 等待运动完成后检查是否途中报警
            time.sleep(0.5)
            err = self.get_error_id()
            if "[]" not in err:
                print(f"  [报警!] 运动中触发报警: {err}")
                print(f"  → 自动 clear_alarm() 恢复...")
                self.clear_alarm()
        except Exception as e:
            print(f"  [MovL 异常] {e}")

    def close(self):
        self.send("DisableRobot()")
        self.sock.close()
        print("[DOBOT] 已断开")


# ============================================================
# 手眼坐标转换
# ============================================================
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib_eye_in_hand.json")


class HandEyeTransformer:
    # 手动覆盖相机在法兰系的位姿 (mm)。设为 None 则使用标定文件。
    # [-130, 0, 0] = 相机在法兰 X=-130mm, 相机光轴∥法兰Z
    MANUAL_OFFSET_MM = [-130, 0, 0]

    def __init__(self, fx=FX, fy=FY, cx=CX, cy=CY):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.flange_pose = None
        # 加载标定结果
        self.T_flange_to_cam = np.eye(4)
        if os.path.exists(CALIB_FILE):
            with open(CALIB_FILE) as f:
                calib = json.load(f)
            self.T_flange_to_cam = np.array(calib["T_flange_to_camera"])
        # 手动覆盖位移 + 旋转 (相机光轴 ∥ 法兰Z)
        if self.MANUAL_OFFSET_MM is not None:
            self.T_flange_to_cam[:3, 3] = np.array(self.MANUAL_OFFSET_MM) / 1000.0
            self.T_flange_to_cam[:3, :3] = np.eye(3)  # 相机与法兰同轴
            print(f"[标定] 手动覆盖相机偏移: ({self.MANUAL_OFFSET_MM[0]},{self.MANUAL_OFFSET_MM[1]},{self.MANUAL_OFFSET_MM[2]})mm, 旋转=I")
        t = self.T_flange_to_cam[:3, 3]
        print(f"[标定] 已加载手眼标定: t=({t[0]*1000:.1f},{t[1]*1000:.1f},{t[2]*1000:.1f})mm")

    def set_flange(self, pose):
        self.flange_pose = pose

    def pixel_to_camera(self, u, v, depth_m):
        """像素 → 相机坐标系 (X右 Y下 Z前, 米)"""
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        return np.array([x, y, depth_m], dtype=np.float64)

    def camera_to_robot(self, p_cam, flange_pose=None, debug=False):
        """相机坐标系 → 机器人基座 (米)
        有标定: base←flange * T_flange→cam * p_cam
        无标定: 近似同轴
        """
        pose = flange_pose or self.flange_pose
        fx_m, fy_m, fz_m = pose[0]/1000, pose[1]/1000, pose[2]/1000

        # 有标定文件时用精确变换
        if not np.allclose(self.T_flange_to_cam, np.eye(4)):
            # 法兰→基座 (用完整 RPY pose 构造)
            T_base_flange = np.eye(4)
            T_base_flange[:3, :3] = self._rpy_to_rot(pose[3], pose[4], pose[5])
            T_base_flange[:3, 3] = [fx_m, fy_m, fz_m]
            # base←flange * flange←cam * cam←point
            p_cam_h = np.append(p_cam, 1.0)

            # 中间结果：相机在世界坐标系下的位置
            cam_w = T_base_flange @ self.T_flange_to_cam @ np.array([0, 0, 0, 1.0])

            p_w = T_base_flange @ self.T_flange_to_cam @ p_cam_h

            if debug:
                print(f"\n  [DEBUG 手眼变换]")
                print(f"    法兰(基座): ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f})mm  rpy=({pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
                print(f"    T_flange→cam (mm): ({self.T_flange_to_cam[0,3]*1000:.0f}, {self.T_flange_to_cam[1,3]*1000:.0f}, {self.T_flange_to_cam[2,3]*1000:.0f})")
                print(f"    相机在世界: ({cam_w[0]:.3f},{cam_w[1]:.3f},{cam_w[2]:.3f})m")
                print(f"    盒子在相机: ({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f})m")
                print(f"    盒子在世界: ({p_w[0]:.3f},{p_w[1]:.3f},{p_w[2]:.3f})m")

            return p_w[:3]
        else:
            # 无标定 fallback: 同轴近似
            rz = np.deg2rad(pose[5])
            cos_rz, sin_rz = np.cos(rz), np.sin(rz)
            x_w = fx_m + cos_rz * p_cam[2] - sin_rz * p_cam[0]
            y_w = fy_m + sin_rz * p_cam[2] + cos_rz * p_cam[0]
            z_w = fz_m - p_cam[1]
            if debug:
                print(f"\n  [DEBUG 无标定] 法兰:({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f}) 盒({x_w:.3f},{y_w:.3f},{z_w:.3f})m")
            return np.array([x_w, y_w, z_w])

    @staticmethod
    def _rpy_to_rot(rx_deg, ry_deg, rz_deg):
        """RPY (度, extrinsic ZYX) → 3×3 旋转矩阵"""
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


# ============================================================
# Aero Hand 灵巧手
# ============================================================
# 7 关节: thumb_abd, thumb_flex, thumb_mcp, index, middle, ring, pinky (度)
class AeroGripper:
    # 完全张开
    OPEN = [0, 0, 0, 0, 0, 42, 0]
    # 抓取 (四指弯曲包络)
    GRASP = [0, 18, 52, 70, 58, 59, 53]

    def __init__(self, port=None):
        self.hand = AeroHand(port=port)

    def connect(self):
        print(f"[Aero] 已连接")

    def disconnect(self):
        self.hand.close()
        print("[Aero] 已断开")

    def open(self):
        self.hand.set_joint_positions(self.OPEN)
        print("[Aero] 张开")

    def close(self):
        self.hand.set_joint_positions(self.GRASP)
        print("[Aero] 抓取")


# ============================================================
# 抓取规划
# ============================================================
def pick_target_box(detections):
    """
    选中货架排列中最靠边的盒子（在图像空间里 X 离中心最远的）
    货架紧凑，得从最外侧下手
    """
    valid = [d for d in detections if d.depth_m and d.depth_m > 0]
    if not valid:
        # fallback: 没有深度时选最大框
        valid = sorted(detections, key=lambda d: (d.bbox[2]-d.bbox[0])*(d.bbox[3]-d.bbox[1]), reverse=True)
        if valid:
            return valid[0], "side"  # 默认抓右边
        return None, None

    # 找 X 方向离画面中心最远的 → 最靠边的盒子
    img_center_x = 320  # 640/2
    target = max(valid, key=lambda d: abs((d.bbox[0] + d.bbox[2]) / 2 - img_center_x))
    # 判断在左侧还是右侧
    cx = (target.bbox[0] + target.bbox[2]) / 2
    edge = "left" if cx < img_center_x else "right"
    return target, edge


def plan_edge_grasp(det, edge, transformer, flange_pose, fx=390.4):
    """
    边缘抓取: 
      - 盒宽 ≤ 8cm: 侧边缘 (x1/x2) + 上边缘 (y1), 手从外侧进入
      - 盒宽 > 8cm: 上边缘 (y1) + 中心 X, 手从上方进入
    返回: (world_x, world_y, world_z, grasp_mode)
    """
    x1, y1, x2, y2 = det.bbox
    depth = det.depth_m or 0.5

    # 计算盒子实际宽度 (米)
    pixel_w = x2 - x1
    real_w = pixel_w * depth / fx  # 小孔模型: pixel = fx * real / depth
    print(f"  盒宽: {pixel_w}px → {real_w*100:.1f}cm")

    if real_w <= 0.08:
        # 窄盒 → 侧边抓: 侧边缘(u) + 垂直中点(v)
        if edge == "left":
            edge_u = x1
        else:
            edge_u = x2
        edge_v = (y1 + y2) / 2
        mode = f"{edge}_side"
        print(f"  → 窄盒 ({real_w*100:.1f}cm ≤ 8cm), 从{edge}侧边缘中点抓取")
    else:
        # 宽盒 → 上方抓: 中心 X + 上边缘
        edge_u = (x1 + x2) / 2
        edge_v = y1
        mode = "top_center"
        print(f"  → 宽盒 ({real_w*100:.1f}cm > 8cm), 从上边缘中心抓取")

    # 像素 → 相机 3D → 机器人基座
    p_cam = transformer.pixel_to_camera(edge_u, edge_v, depth)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=True)
    return p_robot[0], p_robot[1], p_robot[2], mode, p_cam


# ============================================================
# 主流程
# ============================================================
def main():
    print("=" * 60)
    print("  货架边缘抓取系统")
    print(f"  灵巧手: {HAND_LENGTH*100:.0f}cm  {HAND_W_Y*100:.1f}cm×{HAND_W_X*100:.1f}cm  {HAND_WEIGHT}kg")
    print("=" * 60)

    # ── 1. 初始化相机 + YOLO ──
    print("\n[1/4] 初始化相机 + YOLO ...")
    camera = RealSenseCamera(width=640, height=480, fps=30)
    locator = MedicineLocatorSystem(
        model_path="/home/nvidia/grasp/识别/best.pt",
        mapping_path="/home/nvidia/grasp/识别/shelf_mapping.yaml",
        conf_threshold=0.5,
        use_ocr=False,
    )
    locator.init_camera = lambda: None
    locator.camera = camera

    # ── 2. 连接 DOBOT ──
    print("\n[2/4] 连接 DOBOT ...")
    arm = DobotArm()
    arm.connect()

    # ── 3. 连接灵巧手 ──
    if SKIP_HAND:
        print("\n[3/4] 灵巧手 已屏蔽 (SKIP_HAND=True)")
        hand = None
    else:
        print("\n[3/4] 连接 Aero Hand 灵巧手 ...")
        hand = AeroGripper()
        hand.connect()

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
    print("  - 输入 v   ← 实时画面预览 (YOLO 检测)")
    print("  - 输入 j   ← 手动点动 (wasd/e/r 微调法兰)")
    print("  - 输入 q   ← 退出")
    print("=" * 60)

    try:
        while True:
            cmd = input("\n按 Enter 抓取 | v 预览 | j 点动 | q 退出 > ").strip()
            if cmd.lower() == "q":
                break

            if cmd.lower() == "j":
                step = 20  # mm
                print(f"\n  点动模式 (步长 {step}mm), 输入 q 退出")
                print(f"  wasd = XY, e/r = Z")
                while True:
                    jc = input("  点动 > ").strip().lower()
                    if jc == "q":
                        break
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
                    resp = arm.movel(tx, ty, tz, speed=50)
                    time.sleep(0.5)
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

            # ── 获取当前法兰位姿 ──
            flange_pose = arm.get_pose()
            print(f"当前法兰: ({flange_pose[0]:.0f}, {flange_pose[1]:.0f}, {flange_pose[2]:.0f})mm "
                  f"rpy=({flange_pose[3]:.0f}, {flange_pose[4]:.0f}, {flange_pose[5]:.0f})")
            transformer.set_flange(flange_pose)

            # ── 多帧择优拍照：快速连拍 N 帧, 按光照评分选最佳 ──
            N_FRAMES = 10            # 采集帧数
            WARMUP = 2              # 前 N 帧丢弃 (等自动曝光稳定)
            best_img, best_depth = None, None
            best_score, best_grade = -1, ""
            best_idx = -1

            print(f"拍照中 ({N_FRAMES + WARMUP} 帧, 丢弃前 {WARMUP})...", end=" ", flush=True)
            for i in range(N_FRAMES + WARMUP):
                c, d = camera.read()
                if c is None:
                    continue
                if i < WARMUP:
                    continue
                score, grade, _, _, _, _ = _assess_lighting(c)
                if score > best_score:
                    best_score, best_grade = score, grade
                    best_img, best_depth = c, d
                    best_idx = i
                # 反光和过暗都有惩罚, 纯正常最高; 有反光也能选出相对最好的
            print(f"完成")
            print(f"  → 第 {best_idx} 帧 评分: {best_grade} ({best_score:.0f}分)")

            if best_img is None:
                print("相机未就绪")
                continue

            detections = locator.process_frame(best_img, best_depth)
            if not detections:
                print("未检测到盒子")
                # 保存原图排查
                cv2_path = "/tmp/shelf_no_detection.jpg"
                cv2.imwrite(cv2_path, best_img)
                print(f"  → 原图已保存: {cv2_path}")
                continue

            # ── 选最靠边的盒子 ──
            target, edge = pick_target_box(detections)
            if target is None:
                print("无法确定目标")
                continue
            print(f"\n=== 选中: {edge}侧盒子 bbox=({target.bbox[0]},{target.bbox[1]})-({target.bbox[2]},{target.bbox[3]}) "
                  f"depth={target.depth_m:.3f}m ===")

            # ── 计算抓取位姿 ──
            wx, wy, wz, grip_mode, p_cam = plan_edge_grasp(target, edge, transformer, flange_pose, fx=camera.fx)
            print(f"抓取点 ({grip_mode}): ({wx:.3f}, {wy:.3f}, {wz:.3f})m")

            # 重新画正确的抓取点 (覆盖上面简单画的黄点)
            viz = best_img.copy()
            for d in detections:
                col = (0, 0, 255) if d is target else (0, 255, 0)
                cv2.rectangle(viz, (int(d.bbox[0]), int(d.bbox[1])),
                              (int(d.bbox[2]), int(d.bbox[3])), col, 2)
                cv2.putText(viz, f"{d.class_name or '?'} {d.depth_m:.2f}m",
                            (int(d.bbox[0]), int(d.bbox[1]) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            # 目标抓取点: 侧边抓取用边缘中点, 上方抓取用顶边中心
            x1, y1, x2, y2 = target.bbox
            if "side" in grip_mode:
                gx = int(x2 if "right" in grip_mode else x1)
                gy = int((y1 + y2) / 2)
            else:
                gx = int((x1 + x2) / 2)
                gy = int(y1)
            cv2.circle(viz, (gx, gy), 6, (0, 255, 255), -1)
            cv2.putText(viz, "GRIP", (gx + 8, gy - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.imwrite("/tmp/shelf_detection.jpg", viz)
            print(f"  → 检测画面已保存: /tmp/shelf_detection.jpg  (绿框=检测, 红框=目标, 黄点=抓取点)")

            # ── 用法兰局部轴计算法兰/悬停/抓取位姿 (世界坐标, 米) ──
            R_f = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
            fz_dir = R_f[:, 2]   # 法兰+Z → 手指方向 (世界系)

            fingertip_w = np.array([wx, wy, wz - 0.20])  # Z 手工补偿 (负值=下移)
            flange_pt   = fingertip_w - HAND_LENGTH * fz_dir   # 法兰在手背面

            # 悬停: 法兰比目标高 4cm (世界 Z↑, 不受法兰倾斜影响)
            approach_pt = flange_pt + np.array([0.0, 0.0, 0.04])

            # 抓取: 沿手指方向前进 2cm 探入盒面 (法兰+Z)
            grasp_pt = flange_pt + 0.02 * fz_dir

            # ── 转为 mm (python float) 供后续微调 ──
            fx, fy, fz  = float(flange_pt[0]*1000),  float(flange_pt[1]*1000),  float(flange_pt[2]*1000)
            ax, ay, az  = float(approach_pt[0]*1000), float(approach_pt[1]*1000), float(approach_pt[2]*1000)
            gx, gy, gz  = float(grasp_pt[0]*1000),    float(grasp_pt[1]*1000),    float(grasp_pt[2]*1000)

            # ========================================================
            # 调试交互: 微调 / 预览 / 执行  (offset 均为 mm)
            # ========================================================
            offset_x, offset_y, offset_z = 0.0, 0.0, 0.0
            jog_step_mm = 5.0

            while True:
                 print(f"\n── 目标位姿 (可微调) ──")
                 print(f"  指尖世界: ({wx*1000:.0f}, {wy*1000:.0f}, {wz*1000:.0f})mm")
                 print(f"  法兰(悬停): X={ax+offset_x:+.0f}({offset_x:+5.0f})  "
                       f"Y={ay+offset_y:+.0f}({offset_y:+5.0f})  "
                       f"Z={az+offset_z:+.0f}({offset_z:+5.0f}) mm")
                 print(f"  法兰(抓取): X={gx+offset_x:+.0f}  Y={gy+offset_y:+.0f}  Z={gz+offset_z:+.0f} mm")
                 print(f"\n  操作:")
                 print(f"    Enter ← 执行抓取")
                 print(f"    p     ← 预览悬停位 (不抓取)")
                 print(f"    t     ← 相对移动 (含标定位移)")
                 print(f"    m     ← 简化移动 (零标定: 沿法兰Z前移)")
                 print(f"    wasd  ← 微调 X/Y (±{jog_step_mm:.0f}mm)")
                 print(f"    e/r   ← 微调 Z (±{jog_step_mm:.0f}mm)")
                 print(f"    = / - ← 改变步长 (当前 {jog_step_mm:.0f}mm)")
                 print(f"    f     ← 手动输入偏移 (格式: x,y,z mm)")
                 print(f"    F5    ← 重新拍照 (复位偏移)")
                 print(f"    Q     ← 跳过/退出")
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
                     arm.movel(new_x, new_y, new_z, speed=100)
                     time.sleep(1)
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
                     arm.movel(new_x, new_y, new_z, speed=60)
                     time.sleep(1)
                     continue
                 elif choice == "p":
                     print(f"\n  预览: 移动到 ({ax+offset_x:.0f}, {ay+offset_y:.0f}, {az+offset_z:.0f})mm")
                     arm.movel(ax + offset_x, ay + offset_y, az + offset_z, speed=100)
                     time.sleep(1)
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

            # 2. 悬停位 (法兰退回 4cm)
            arm.movel(ax + offset_x, ay + offset_y, az + offset_z, speed=150)
            time.sleep(1.5)

            # 3. 下降抓取 (法兰前进，指尖探入盒面)
            arm.movel(gx + offset_x, gy + offset_y, gz + offset_z, speed=50)
            time.sleep(1.5)

            # 4. 抓取
            if not SKIP_HAND:
                hand.close()
                time.sleep(1.5)

            # 5. 后退 (沿法兰-Z 退 10cm, 离开货架)
            retreat_pt = np.array([gx+offset_x, gy+offset_y, gz+offset_z]) - 100.0 * fz_dir
            arm.movel(float(retreat_pt[0]), float(retreat_pt[1]), float(retreat_pt[2]), speed=100)
            time.sleep(1.5)

            print("抓取完成! 拖动到放置位按 Enter 继续 / q 退出")

    except KeyboardInterrupt:
        pass
    finally:
        if not SKIP_HAND:
            hand.disconnect()
        arm.close()
        camera.release()
        print("\n已退出")


if __name__ == "__main__":
    main()
