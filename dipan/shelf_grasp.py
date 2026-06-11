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
HAND_WEIGHT = 1.4       # 末端重量 (kg), 手 1.2 + 连接件 0.2
HAND_COG_Z = 0.10       # 重心距法兰 (米)
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
        print("[DOBOT] 已使能 (请手动进入拖动示教模式)")

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
        # 窄盒 → 侧边抓
        if edge == "left":
            edge_u = x1
        else:
            edge_u = x2
        edge_v = y1
        mode = f"{edge}_side"
        print(f"  → 窄盒 ({real_w*100:.1f}cm ≤ 8cm), 从{edge}侧边缘抓取")
    else:
        # 宽盒 → 上方抓
        edge_u = (x1 + x2) / 2  # 中心 X
        edge_v = y1
        mode = "top_center"
        print(f"  → 宽盒 ({real_w*100:.1f}cm > 8cm), 从上边缘中心抓取")

    # 像素 → 相机 3D → 机器人基座
    p_cam = transformer.pixel_to_camera(edge_u, edge_v, depth)
    p_robot = transformer.camera_to_robot(p_cam, flange_pose, debug=True)
    return p_robot[0], p_robot[1], p_robot[2], mode


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

            # ── 计算抓取位姿 ──
            wx, wy, wz, grip_mode = plan_edge_grasp(target, edge, transformer, flange_pose, fx=camera.fx)
            print(f"抓取点 ({grip_mode}): ({wx:.3f}, {wy:.3f}, {wz:.3f})m")

            # 法兰在指尖上方 HAND_LENGTH
            flange_z = wz + HAND_LENGTH
            flange_x, flange_y = wx, wy

            # 偏置: 侧边抓时向外偏, 上方抓时不偏
            if grip_mode.endswith("_side"):
                side_offset = 0.03  # 3cm 向外偏
                if "left" in grip_mode:
                    flange_x -= side_offset
                else:
                    flange_x += side_offset

            approach_z = flange_z + 0.04    # 悬停 4cm 上方 (货架空间紧)
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
            if not SKIP_HAND:
                hand.open()

            # 2. 移动到目标侧上方 (从外侧进入，避开旁边盒子)
            arm.movel(fx, fy, a_z, speed=150)
            time.sleep(1.5)

            # 3. 下降
            arm.movel(fx, fy, g_z, speed=50)
            time.sleep(1.5)

            # 4. 抓取
            if not SKIP_HAND:
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
        if not SKIP_HAND:
            hand.disconnect()
        arm.close()
        camera.release()
        print("\n已退出")


if __name__ == "__main__":
    main()
