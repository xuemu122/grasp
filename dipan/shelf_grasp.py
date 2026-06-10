#!/usr/bin/env python3
"""
货架边缘抓取系统
- YOLO 检测盒子 → 选最外侧（深度最小）
- 边缘抓取（bbox 下边缘 = 盒子前缘，而不是中心）
- DOBOT Nova5 TCP 控制 + Orca 灵巧手
"""

import sys
import os
import time
import socket
import re
import json
import numpy as np
from typing import List, Tuple, Optional

# ── 识别模块 ──
sys.path.insert(0, "/home/nvidia/grasp/识别")
from orbbec_detect import MedicineLocatorSystem, RealSenseCamera
from orbbec_detect import _assess_lighting

# ── Orca 灵巧手 ──
ORCA_PATH = "/home/nvidia/grasp/orca_core1"
sys.path.insert(0, ORCA_PATH)
from orca_core import OrcaHand, OrcaJointPositions


# ============================================================
# 参数
# ============================================================
DOBOT_IP = os.getenv("DOBOT_IP", "192.168.1.6")
DOBOT_PORT = 29999

HAND_LENGTH = 0.35      # 法兰 → 指尖 (米)
HAND_WEIGHT = 1.2       # 末端重量 (kg)
HAND_COG_Z = 0.10       # 重心距法兰 (米)
HAND_W_Y = 0.095        # 手 Y 方向宽度 (米)
HAND_W_X = 0.08         # 手 X 方向宽度 (米)

# D405 内参（自动读取，这里是 fallback）
FX, FY = 390.4, 390.0
CX, CY = 324.5, 241.2


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
        self.sock.settimeout(10)
        self.sock.connect((self.ip, self.port))
        print(f"[DOBOT] 已连接 {self.ip}:{self.port}")
        # 设置负载
        cmds = [
            f"SetPayload(mass={HAND_WEIGHT}, centerX=0, centerY=0, centerZ={HAND_COG_Z})",
            "Tool(index=1)",
        ]
        for c in cmds:
            self.send(c)
        # 使能
        self.send("EnableRobot()")
        time.sleep(0.5)
        # 拖动示教（零重力）
        self.send("StartDrag()")
        print("[DOBOT] 已使能 + 进入拖动示教模式")

    def send(self, cmd: str) -> str:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.sock.sendall(cmd.encode())
        resp = b""
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            resp += chunk
            if b";" in chunk:
                break
        return resp.decode().strip()

    def get_pose(self) -> Tuple[float, float, float, float, float, float]:
        """返回 (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)"""
        resp = self.send("GetPose()")
        nums = re.findall(r"[-+]?\d*\.?\d+", resp)
        if len(nums) >= 6:
            return tuple(map(float, nums[-6:]))
        raise RuntimeError(f"无法解析位姿: {resp}")

    def movel(self, x_mm, y_mm, z_mm, rx=180.0, ry=0.0, rz=0.0, speed=100):
        cmd = f"MovL(x={x_mm:.1f}, y={y_mm:.1f}, z={z_mm:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}, speed={speed:.0f})"
        print(f"  >>> {cmd}")
        resp = self.send(cmd)
        print(f"  <<< {resp}")

    def close(self):
        self.send("StopDrag()")
        self.send("DisableRobot()")
        self.sock.close()
        print("[DOBOT] 已断开")


# ============================================================
# 手眼坐标转换
# ============================================================
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calib_eye_in_hand.json")


class HandEyeTransformer:
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
            t = self.T_flange_to_cam[:3, 3]
            print(f"[标定] 已加载手眼标定: t=({t[0]*1000:.1f},{t[1]*1000:.1f},{t[2]*1000:.1f})mm")

    def set_flange(self, pose):
        self.flange_pose = pose

    def pixel_to_camera(self, u, v, depth_m):
        """像素 → 相机坐标系 (X右 Y下 Z前, 米)"""
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        return np.array([x, y, depth_m], dtype=np.float64)

    def camera_to_robot(self, p_cam, flange_pose=None):
        """相机坐标系 → 机器人基座 (米)
        有标定: base←flange * T_flange→cam * p_cam
        无标定: 近似同轴
        """
        pose = flange_pose or self.flange_pose
        fx_m, fy_m, fz_m = pose[0]/1000, pose[1]/1000, pose[2]/1000
        rz = np.deg2rad(pose[5])
        cos_rz, sin_rz = np.cos(rz), np.sin(rz)

        # 有标定文件时用精确变换
        if not np.allclose(self.T_flange_to_cam, np.eye(4)):
            # 法兰→基座 (用 pose 构造)
            T_base_flange = np.eye(4)
            T_base_flange[:3, :3] = np.array([
                [cos_rz, -sin_rz, 0],
                [sin_rz,  cos_rz, 0],
                [0,       0,      1]
            ])
            T_base_flange[:3, 3] = [fx_m, fy_m, fz_m]
            # base←flange * flange←cam * cam←point
            p_cam_h = np.append(p_cam, 1.0)
            p_w = T_base_flange @ self.T_flange_to_cam @ p_cam_h
            return p_w[:3]
        else:
            # 无标定 fallback: 同轴近似
            x_w = fx_m + cos_rz * p_cam[2] - sin_rz * p_cam[0]
            y_w = fy_m + sin_rz * p_cam[2] + cos_rz * p_cam[0]
            z_w = fz_m - p_cam[1]
            return np.array([x_w, y_w, z_w])


# ============================================================
# Orca 灵巧手
# ============================================================
class OrcaGripper:
    OPEN = {
        "thumb_cmc": 0.70, "thumb_abd": 0.80, "thumb_mcp": 0.85, "thumb_dip": 0.75,
        "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
        "index_mcp": 0.15, "middle_mcp": 0.15, "ring_mcp": 0.15, "pinky_mcp": 0.15,
        "index_pip": 0.10, "middle_pip": 0.10, "ring_pip": 0.10, "pinky_pip": 0.10,
        "wrist": 0.30,
    }
    GRASP = {
        "thumb_cmc": 0.35, "thumb_abd": 0.55, "thumb_mcp": 0.20, "thumb_dip": 0.85,
        "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
        "index_mcp": 0.85, "middle_mcp": 0.85, "ring_mcp": 0.85, "pinky_mcp": 0.85,
        "index_pip": 0.90, "middle_pip": 0.90, "ring_pip": 0.90, "pinky_pip": 0.90,
        "wrist": 0.55,
    }

    def __init__(self):
        self.hand = OrcaHand()

    def connect(self):
        ok, msg = self.hand.connect()
        if not ok:
            raise RuntimeError(f"灵巧手连接失败: {msg}")
        print(f"[Orca] {msg}")
        self.hand.init_joints()

    def disconnect(self):
        self.hand.disable_torque()
        self.hand.disconnect()

    def _to_positions(self, fractions):
        p = dict(self.hand.config.neutral_position)
        for j, f in fractions.items():
            if j in self.hand.config.joint_roms_dict:
                lo, hi = self.hand.config.joint_roms_dict[j]
                p[j] = lo + f * (hi - lo)
        return OrcaJointPositions.from_dict(p)

    def open(self, steps=16):
        self.hand.set_joint_positions(self._to_positions(self.OPEN), num_steps=steps, step_size=0.02)
        print("[Orca] 张开")

    def close(self, steps=24):
        self.hand.set_joint_positions(self._to_positions(self.GRASP), num_steps=steps, step_size=0.02)
        print("[Orca] 握拳抓取")


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


def plan_edge_grasp(det, edge, transformer, flange_pose):
    """
    边缘抓取: 侧边缘 (x1 或 x2) × 上边缘 (y1)
    从盒子外侧边缘和上边缘抓取，手从外上方进入
    返回: (world_x, world_y, world_z, 是否需要旋转法兰)
    """
    x1, y1, x2, y2 = det.bbox
    depth = det.depth_m or 0.5

    if edge == "left":
        edge_u = x1           # 左边缘
    else:
        edge_u = x2           # 右边缘
    edge_v = y1               # 上边缘

    # 像素 → 相机 3D → 机器人基座
    p_cam = transformer.pixel_to_camera(edge_u, edge_v, depth)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose)
    return p_robot[0], p_robot[1], p_robot[2]


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
    print("\n[3/4] 连接 Orca 灵巧手 ...")
    hand = OrcaGripper()
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
    print("  - 输入 q ← 退出")
    print("=" * 60)

    try:
        while True:
            cmd = input("\n按 Enter 抓取 | q 退出 > ").strip()
            if cmd.lower() == "q":
                break

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
                continue

            # ── 选最靠边的盒子 ──
            target, edge = pick_target_box(detections)
            if target is None:
                print("无法确定目标")
                continue
            print(f"\n=== 选中: {edge}侧盒子 bbox=({target.bbox[0]},{target.bbox[1]})-({target.bbox[2]},{target.bbox[3]}) "
                  f"depth={target.depth_m:.3f}m ===")

            # ── 计算抓取位姿 (侧边缘 + 上边缘) ──
            wx, wy, wz = plan_edge_grasp(target, edge, transformer, flange_pose)
            print(f"\n=== 选中: {edge}侧盒子 bbox=({target.bbox[0]},{target.bbox[1]})-({target.bbox[2]},{target.bbox[3]}) "
                  f"depth={target.depth_m:.3f}m ===")
            print(f"抓取点 ({edge}上角): ({wx:.3f}, {wy:.3f}, {wz:.3f})m")

            # 法兰在指尖上方 HAND_LENGTH
            flange_z = wz + HAND_LENGTH
            flange_x, flange_y = wx, wy

            # 偏置: 手稍微偏出盒子外侧，避免撞到旁边的盒子
            side_offset = 0.03  # 3cm 向外偏
            if edge == "left":
                flange_x -= side_offset
            else:
                flange_x += side_offset

            approach_z = flange_z + 0.04    # 悬停 8cm 上方 (货架空间紧)
            grasp_z = flange_z - 0.02       # 指尖略深入盒面

            # ========================================================
            # 调试交互: 微调 / 预览 / 执行
            # ========================================================
            offset_x, offset_y, offset_z = 0.0, 0.0, 0.0
            jog_step_mm = 5.0  # 每次微调步长 (mm)
            preview_target = False  # 是否预览悬停位

            while True:
                 print(f"\n── 目标位姿 (可微调) ──")
                 print(f"  法兰: X={flange_x*1000+offset_x:+.0f}({offset_x:+5.0f})  "
                       f"Y={flange_y*1000+offset_y:+.0f}({offset_y:+5.0f})  "
                       f"Z={approach_z*1000+offset_z:+.0f}({offset_z:+5.0f}) mm")
                 print(f"  下降后Z: {(grasp_z+offset_z)*1000:.0f}mm")
                 print(f"\n  操作:")
                 print(f"    Enter ← 执行抓取")
                 print(f"    p     ← 预览悬停位 (不抓取)")
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
                 elif choice == "p":
                     # 预览: 只移动到悬停位, 不抓取
                     print(f"\n  预览: 移动到 ({flange_x*1000+offset_x:.0f}, "
                           f"{flange_y*1000+offset_y:.0f}, {approach_z*1000+offset_z:.0f})mm")
                     arm.movel(flange_x * 1000 + offset_x,
                               flange_y * 1000 + offset_y,
                               approach_z * 1000 + offset_z, speed=100)
                     time.sleep(1)
                     continue
                 elif choice.lower() == "f5":
                     # 重新拍照
                     offset_x = offset_y = offset_z = 0.0
                     break  # 跳出微调循环, 走外层重新读图和检测
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

            # 如果选了重新拍照, 跳过执行回到外层
            if choice.lower() == "f5":
                continue

            # 如果选了跳过, 跳出
            if choice in ("Q", "q"):
                break

            # ========================================================
            # ── 执行 ──
            print("\n执行抓取动作...")

            fx = flange_x * 1000 + offset_x
            fy = flange_y * 1000 + offset_y
            a_z = approach_z * 1000 + offset_z
            g_z = grasp_z * 1000 + offset_z

            # 1. 张开手
            hand.open()

            # 2. 移动到目标侧上方 (从外侧进入，避开旁边盒子)
            arm.movel(fx, fy, a_z, speed=150)
            time.sleep(1.5)

            # 3. 下降
            arm.movel(fx, fy, g_z, speed=50)
            time.sleep(1.5)

            # 4. 抓取
            hand.close()
            time.sleep(1.5)

            # 5. 抬升 (离开货架)
            lift_z = g_z + 100
            arm.movel(fx, fy, lift_z, speed=100)
            time.sleep(1.5)

            print("抓取完成! 拖动到放置位按 Enter 继续 / q 退出")

    except KeyboardInterrupt:
        pass
    finally:
        hand.disconnect()
        arm.close()
        camera.release()
        print("\n已退出")


if __name__ == "__main__":
    main()
