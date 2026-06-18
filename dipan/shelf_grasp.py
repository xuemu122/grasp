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
DOBOT_IP = os.getenv("DOBOT_IP", "192.168.5.1")
DOBOT_PORT = 29999

HAND_LENGTH = 0.25      # 法兰 → 指尖 (米)
HAND_WEIGHT = 0.6       # 末端重量 (kg), Aero Hand
HAND_COG_Z = 0.12       # 重心 Z 偏移 (米), 12cm 法兰+Z 方向
HAND_W_Y = 0.095        # 手 Y 方向宽度 (米)
HAND_W_X = 0.08         # 手 X 方向宽度 (米)

# 抓取参数 (基座系, 米): -X=前 +Y=右 +Z=上
# 侧面抓: 盒宽 <= 8cm, grip_mode 为 left_side/right_side
SIDE_GRASP_OFFSET_M = (-0.12, 0.02, 0.015)
# 上面抓: 盒宽 > 8cm, grip_mode 为 top_center。先沿用侧抓值, 实测后单独固化。
TOP_GRASP_OFFSET_M = (-0.12, 0.015, 0.025)

# 上面抓取前把第六轴转到手心向下的实测角度
TOP_J6_DEG = 8.3

# 抓取前悬停/探入距离
APPROACH_LIFT_M = 0.00    # 悬停位比目标法兰点高 1cm
GRASP_ADVANCE_M = 0.04    # 抓取时沿手指方向前进 5cm
TOP_APPROACH_BACKOFF_M = 0.15  # 上面抓时先在抓取轴线上退 15cm 再直线前进

# D405 内参（自动读取，这里是 fallback）
FX, FY = 390.4, 390.0
CX, CY = 324.5, 241.2

# 调试开关: 屏蔽手, 只测试手眼标定 + 机械臂走位
SKIP_HAND = True

# 机械臂初始观察位姿 (角度制)
HOME_JOINTS_DEG = [-44.1, -20.2, 134.6, -112.7, -46.0, 90.6]

# 货架层位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
SHELF_LAYERS_FILE = os.path.join(os.path.dirname(__file__), "shelf_layers.json")
_current_layer_idx = 1  # 当前选中的层编号 (1=顶层)
_shelf_layer_poses = {}  # {1: ([x,y,z,rx,ry,rz], name), 2: ...}
LOW_Z_LAYER_IDXS = {4}  # 第4层在抬高基座下方, 允许低/负 Z

# 放置点位姿文件 (笛卡尔: x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg)
PLACE_POINT_FILE = os.path.join(os.path.dirname(__file__), "place_point.json")
_place_point_pose = None  # ([x,y,z,rx,ry,rz], name)


def _layer_allows_low_z(layer_idx: Optional[int]) -> bool:
    return layer_idx in LOW_Z_LAYER_IDXS


def _current_layer_allows_low_z() -> bool:
    return _layer_allows_low_z(_current_layer_idx)


def load_shelf_layers():
    """从 JSON 加载货架层位姿，返回 (layers_dict, current_idx)"""
    global _shelf_layer_poses, _current_layer_idx
    if os.path.exists(SHELF_LAYERS_FILE):
        with open(SHELF_LAYERS_FILE) as f:
            data = json.load(f)
        for k, v in data.get("layers", {}).items():
            _shelf_layer_poses[int(k)] = (v["pose"], v.get("name", f"第{k}层"))
        _current_layer_idx = data.get("current")
    return _shelf_layer_poses, _current_layer_idx


def load_place_point():
    """从 JSON 加载放置点位姿，返回 (pose, name) 或 None"""
    global _place_point_pose
    if os.path.exists(PLACE_POINT_FILE):
        with open(PLACE_POINT_FILE) as f:
            data = json.load(f)
        if "pose" in data:
            _place_point_pose = (data["pose"], data.get("name", "放置点"))
    return _place_point_pose


def save_shelf_layers():
    """保存货架层位姿到 JSON"""
    data = {
        
        "layers": {str(k): {"pose": p, "name": n} for k, (p, n) in _shelf_layer_poses.items()},
        "current": _current_layer_idx,
    }
    with open(SHELF_LAYERS_FILE, "w") as f:
        json.dump(data, f, indent=2)


def save_place_point():
    """保存放置点位姿到 JSON"""
    if _place_point_pose is None:
        if os.path.exists(PLACE_POINT_FILE):
            os.remove(PLACE_POINT_FILE)
        return
    pose, name = _place_point_pose
    data = {"pose": pose, "name": name}
    with open(PLACE_POINT_FILE, "w") as f:
        json.dump(data, f, indent=2)


def go_to_layer(arm: "DobotArm", layer_idx: int):
    """移动机械臂到指定货架层的观察位姿 (MovL 笛卡尔)"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False
    pose, name = _shelf_layer_poses[layer_idx]
    print(f"\n[货架层] 移动到第{layer_idx}层 ({name})...")
    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < 80.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 80mm")
        target_z = 80.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5], speed=100,
                     allow_low_z=allow_low_z):
        return False
    time.sleep(2)
    return True


def go_to_layer_via_clearance(arm: "DobotArm", layer_idx: int, clearance_mm: float = 120.0):
    """先抬高再回到指定层, 用于避开放置点到货架层的直线路径"""
    if layer_idx not in _shelf_layer_poses:
        print(f"  [错误] 第{layer_idx}层未记录! 先用 l s 记录当前位姿")
        return False
    pose, name = _shelf_layer_poses[layer_idx]
    print(f"\n[货架层] 安全返回第{layer_idx}层 ({name})...")
    try:
        cur = arm.get_pose()
    except Exception as e:
        print(f"  [警告] 无法读取当前位姿, 回退为直接 MovL: {e}")
        return go_to_layer(arm, layer_idx)

    target_z = pose[2]
    allow_low_z = _layer_allows_low_z(layer_idx)
    if target_z < 80.0 and not allow_low_z:
        print(f"  [警告] 第{layer_idx}层记录 Z={target_z:.1f}mm 过低, 自动抬到 80mm 作为可达观察位")
        target_z = 80.0
    if allow_low_z and target_z < 50.0:
        print(f"  [第{layer_idx}层] 允许低位观察 Z={target_z:.1f}mm")
    safe_z = max(cur[2], target_z) + clearance_mm
    if not arm.movel(cur[0], cur[1], safe_z, speed=120,
                     allow_low_z=allow_low_z and safe_z < 50.0):
        return False
    if not arm.movel(pose[0], pose[1], safe_z, pose[3], pose[4], pose[5], speed=120,
                     allow_low_z=allow_low_z and safe_z < 50.0):
        return False
    if not arm.movel(pose[0], pose[1], target_z, pose[3], pose[4], pose[5], speed=100,
                     allow_low_z=allow_low_z):
        return False
    time.sleep(2)
    return True


def go_to_place_point(arm: "DobotArm"):
    """移动机械臂到记录的放置点 (MovL 笛卡尔)"""
    if _place_point_pose is None:
        print("  [错误] 放置点未记录! 先用 p 进入放置点管理, 拖到目标位置后输入 s")
        return False
    pose, name = _place_point_pose
    print(f"\n[放置点] 移动到 {name} ...")
    arm.movel(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], speed=100)
    time.sleep(2)
    try:
        cur = arm.get_pose()
    except Exception:
        print("  [警告] 无法确认是否到达放置点, 本次不自动张手")
        return False
    err_mm = np.linalg.norm(np.array(cur[:3]) - np.array(pose[:3]))
    if err_mm > 30:
        print(f"  [警告] 当前距放置点 {err_mm:.0f}mm, 本次不自动张手")
        return False
    return True


def finish_pick_cycle(arm: "DobotArm", hand: Optional["AeroGripper"], current_layer_idx: Optional[int]):
    """抓取后放置并回到观察层。3/4层经第1层中转, 避免低层直达放置点。"""
    hand_ok = hand is not None and not SKIP_HAND

    if _place_point_pose is None:
        if hand_ok:
            hand.open()
        return

    transfer_layer_idx = 1
    use_transfer_layer = current_layer_idx in (3, 4) and transfer_layer_idx in _shelf_layer_poses

    if use_transfer_layer:
        print(f"\n[Transfer] 第{current_layer_idx}层抓取后先回当前层观察位 ...")
        if not go_to_layer_via_clearance(arm, current_layer_idx):
            print("  [停止] 回当前层失败, 停在当前位置, 不继续放置流程")
            return
        print(f"\n[Transfer] 经第{transfer_layer_idx}层中转前往放置点 ...")
        if not go_to_layer_via_clearance(arm, transfer_layer_idx):
            print(f"  [停止] 回第{transfer_layer_idx}层中转失败, 停在当前位置, 不继续放置流程")
            return

    print(f"\n[Place] 前往放置点 ({_place_point_pose[1]}) ...")
    if not go_to_place_point(arm):
        print("  [停止] 前往放置点失败, 停在当前位置, 不继续回层")
        return
    if hand_ok:
        hand.open()
    time.sleep(1)

    if use_transfer_layer:
        print(f"\n[Transfer] 放置后回第{transfer_layer_idx}层中转 ...")
        if not go_to_layer_via_clearance(arm, transfer_layer_idx):
            print(f"  [停止] 放置后回第{transfer_layer_idx}层失败, 停在当前位置")
            return

    if current_layer_idx in _shelf_layer_poses:
        print(f"\n[Home] 回到第{current_layer_idx}层 ({_shelf_layer_poses[current_layer_idx][1]}) ...")
        if not go_to_layer_via_clearance(arm, current_layer_idx):
            print("  [停止] 回当前层失败, 停在当前位置")
            return
    else:
        print("\n[Home] 回到 HOME ...")
        arm.go_home()


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
        # V4 协议: ClearError → RequestControl → PowerOn → SetPayload → EnableRobot
        resp = self.send("ClearError()")
        print(f"[DOBOT] ClearError: {resp}")
        resp = self.send("RequestControl()")
        print(f"[DOBOT] RequestControl: {resp}")
        resp = self.send("PowerOn()")
        print(f"[DOBOT] PowerOn: {resp}  (等待 10 秒上电...)")
        time.sleep(10)

        # 设置负载
        resp = self.send(f"SetPayload({HAND_WEIGHT}, 0, 0, {HAND_COG_Z*1000:.0f})")
        print(f"[DOBOT] SetPayload: {resp}")

        # 使能 (不带负载参数，负载已用 SetPayload 设置)
        resp = self.send("EnableRobot()")
        print(f"[DOBOT] EnableRobot: {resp}")
        time.sleep(0.5)

        resp = self.send("RobotMode()")
        print(f"[DOBOT] RobotMode: {resp}")
        if self._is_enabled_mode(resp):
            print("[DOBOT] 使能确认")
        else:
            mode = self._parse_robot_mode(resp)
            print(f"[DOBOT] 警告! RobotMode={mode if mode is not None else '?'} 未进入使能状态")
            self.do_enable()

    @staticmethod
    def _parse_robot_mode(resp: str) -> Optional[int]:
        """解析 RobotMode() 返回的模式值: ErrorID,{Value},RobotMode();"""
        match = re.search(r"\{(-?\d+)\}", resp)
        if not match:
            return None
        return int(match.group(1))

    @classmethod
    def _is_enabled_mode(cls, resp: str) -> bool:
        mode = cls._parse_robot_mode(resp)
        return mode in (5, 6, 7, 8)

    def do_enable(self):
        """重新使能"""
        for attempt in range(3):
            self.send("ClearError()")
            time.sleep(0.3)
            self.send("RequestControl()")
            time.sleep(0.3)
            resp = self.send("RobotMode()")
            print(f"[DOBOT] RobotMode: {resp}")
            if self._is_enabled_mode(resp):
                print("[DOBOT] 已处于使能状态, 跳过重复上电等待")
                return True
            self.send("PowerOn()")
            time.sleep(1)
            resp = self.send("RobotMode()")
            mode = self._parse_robot_mode(resp)
            if mode in (3, 4, None):
                print(f"[DOBOT] 等待上电 10s (重试 {attempt+1}/3)...")
                time.sleep(10)
            else:
                print(f"[DOBOT] RobotMode={mode}, 跳过完整上电等待")
            self.send(f"SetPayload({HAND_WEIGHT}, 0, 0, {HAND_COG_Z*1000:.0f})")
            resp = self.send("EnableRobot()")
            print(f"[DOBOT] EnableRobot: {resp}")
            time.sleep(0.5)
            resp = self.send("RobotMode()")
            print(f"[DOBOT] RobotMode: {resp}")
            if self._is_enabled_mode(resp):
                print("[DOBOT] 使能成功!")
                return True
        print("[DOBOT] 使能失败! 请手动检查机械臂状态")
        return False

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
        self.do_enable()

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

    def get_joints(self) -> Tuple[float, float, float, float, float, float]:
        """返回 (j1, j2, j3, j4, j5, j6)，单位: 度"""
        resp = self.send("GetAngle()")
        nums = re.findall(r"[-+]?\d*\.?\d+", resp)
        if len(nums) >= 6:
            return tuple(map(float, nums[-6:]))
        raise RuntimeError(f"无法解析关节角: {resp}")

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
        err = self.get_error_id()
        print(f"[DOBOT] GetErrorID: {err}")
        self.do_enable()

    def movel(self, x_mm, y_mm, z_mm, rx=None, ry=None, rz=None, speed=100, allow_low_z=False):
        """MovL V4: pose = {x, y, z, rx, ry, rz}，rx/ry/rz=None 则保持当前姿态"""
        try:
            if rx is None:
                pose = self.get_pose()
                rx, ry, rz = pose[3], pose[4], pose[5]
            # 安全: Z<50mm 拒绝执行, 防止撞击底座
            if z_mm < 50 and not allow_low_z:
                print(f"  [拒绝!] Z={z_mm:.0f}mm 太低, 可能碰撞, 跳过低抓取")
                return False
            if z_mm < 50 and allow_low_z:
                print(f"  [低位放行] Z={z_mm:.0f}mm (第4层/下方工作区)")
            if z_mm > 1100 or z_mm < -300:
                print(f"  [警告] Z={z_mm:.0f}mm 可能超出工作范围, 请注意!")
            cmd = f"MovL(pose = {{{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}}})"
            print(f"  >>> {cmd}")
            resp = self.send(cmd)
            print(f"  <<< {resp}")
            if resp.startswith("-"):
                print(f"  [MovL 被拒绝] 错误码={resp.split(',')[0]}, 可能是位姿不可达")
                return False  # -1/-2 等是立即拒绝, 不是报警, 不调 clear_alarm
            # 等待运动完成: RobotMode=5 表示使能空闲; 7/8 仍在运行/点动中
            for _ in range(60):
                time.sleep(0.5)
                err = self.get_error_id()
                if "[]" not in err:
                    print(f"  [报警!] 运动中触发报警: {err}")
                    print(f"  → 自动 clear_alarm() 恢复...")
                    self.clear_alarm()
                    return False
                rm = self.send("RobotMode()")
                mode = self._parse_robot_mode(rm)
                if mode == 5:
                    return True
                if mode in (4, 9, 11):
                    print(f"  [MovL 异常状态] RobotMode={mode}, 停止后续动作")
                    return False
            print("  [MovL 超时] 运动 30s 未完成")
            return False
        except Exception as e:
            print(f"  [MovL 异常] {e}")
            return False

    def go_home(self, speed=100):
        """MovJ 回到预设观察位姿"""
        self.movj(HOME_JOINTS_DEG)

    def movj(self, joints_deg):
        """MovJ 关节运动到目标关节角 (角度制)"""
        j = joints_deg
        cmd = f"MovJ(joint={{{j[0]:.1f},{j[1]:.1f},{j[2]:.1f},{j[3]:.1f},{j[4]:.1f},{j[5]:.1f}}})"
        print(f"  >>> {cmd}")
        resp = self.send(cmd)
        print(f"  <<< {resp}")
        if resp.startswith("-"):
            print(f"  [MovJ 被拒绝] 错误码={resp.split(',')[0]}")
            return False
        for _ in range(60):
            time.sleep(0.5)
            err = self.get_error_id()
            if "[]" not in err:
                print(f"  [报警!] MovJ 触发报警: {err}")
                self.clear_alarm()
                return False
            rm = self.send("RobotMode()")
            mode = self._parse_robot_mode(rm)
            if mode == 5:
                return True
            if mode in (4, 9, 11):
                print(f"  [MovJ 异常状态] RobotMode={mode}, 停止后续动作")
                return False
        print("  [MovJ 超时] 运动 30s 未完成")
        return False

    def move_j6(self, target_j6_deg: float):
        """只调整第六轴到目标角度，其他关节保持当前角度"""
        joints = list(self.get_joints())
        old_j6 = joints[5]
        joints[5] = target_j6_deg
        print(f"  MovJ J6 {old_j6:.1f}° → {target_j6_deg:.1f}°")
        return self.movj(joints)

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
    # 相机在法兰 -X 方向约 12cm, 光轴 ∥ 法兰+Z (手指方向), 相机+X→法兰-Y (rz≈-90°)
    MANUAL_OFFSET_MM = [-120, 0, 0]
    #MANUAL_OFFSET_MM = None

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
        # 手动覆盖: 相机光轴∥法兰+Z, 相机+X→法兰-Y (≈ rz=-90°)
        if self.MANUAL_OFFSET_MM is not None:
            self.T_flange_to_cam[:3, 3] = np.array(self.MANUAL_OFFSET_MM) / 1000.0
            rz = np.deg2rad(-90)  # 相机相对于法兰绕 Z 转了 -90°
            self.T_flange_to_cam[:3, :3] = np.array([
                [np.cos(rz), -np.sin(rz), 0],
                [np.sin(rz),  np.cos(rz), 0],
                [0, 0, 1]
            ])
            print(f"[标定] 手动覆盖: 法兰→相机 平移=({self.MANUAL_OFFSET_MM[0]},{self.MANUAL_OFFSET_MM[1]},{self.MANUAL_OFFSET_MM[2]})mm, rz={np.rad2deg(rz):.0f}°")
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
        法兰 RPY 完整变换 (相机与法兰同轴)
        """
        pose = flange_pose or self.flange_pose

        # 法兰→基座 (用完整 RPY pose 构造)
        T_base_flange = np.eye(4)
        T_base_flange[:3, :3] = self._rpy_to_rot(pose[3], pose[4], pose[5])
        T_base_flange[:3, 3] = [pose[0]/1000, pose[1]/1000, pose[2]/1000]
        # base←flange * flange←cam * cam←point
        p_cam_h = np.append(p_cam, 1.0)
        p_w = T_base_flange @ self.T_flange_to_cam @ p_cam_h

        if debug:
            cam_w = T_base_flange @ self.T_flange_to_cam @ np.array([0, 0, 0, 1.0])
            print(f"\n  [DEBUG 手眼变换]")
            print(f"    法兰(基座): ({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f})mm  rpy=({pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            print(f"    T_cam→flange (mm): ({self.T_flange_to_cam[0,3]*1000:.0f}, {self.T_flange_to_cam[1,3]*1000:.0f}, {self.T_flange_to_cam[2,3]*1000:.0f})")
            print(f"    相机在世界: ({cam_w[0]:.3f},{cam_w[1]:.3f},{cam_w[2]:.3f})m")
            print(f"    盒子在相机: ({p_cam[0]:.3f},{p_cam[1]:.3f},{p_cam[2]:.3f})m")
            print(f"    盒子在世界: ({p_w[0]:.3f},{p_w[1]:.3f},{p_w[2]:.3f})m")

        return p_w[:3]

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
    OPEN = [0, 0, 0, 0, 0, 0, 0]
    # 抓取 (四指弯曲包络)
    GRASP = [0, 70, 52, 70, 58, 59, 53]
    #上端抓取
    GRASP_TOP = [0, 86, 49, 71, 50, 70, 70]

    def __init__(self, port=None):
        self.hand = AeroHand(port=port)
        self.ok = True

    def connect(self):
        print(f"[Aero] 已连接")

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

    def close(self, top=False):
        if not self.ok:
            print("[Aero] 已禁用, 跳过抓取")
            return False
        try:
            pose = self.GRASP_TOP if top else self.GRASP
            self.hand.set_joint_positions(pose)
            print("[Aero] 上端抓取" if top else "[Aero] 抓取")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 抓取失败, 已禁用灵巧手: {e}")
            return False


# ============================================================
# 抓取规划
# ============================================================
def pick_target_box(detections):
    """
    货架分层抓取:
      1. 按深度分组 (同层盒子深度差 < 0.1m)
      2. 选最前层 (深度最小) 
      3. 在该层里选最靠边的盒子
    """
    valid = [d for d in detections if d.depth_m and d.depth_m > 0]
    if not valid:
        valid = sorted(detections, key=lambda d: (d.bbox[2]-d.bbox[0])*(d.bbox[3]-d.bbox[1]), reverse=True)
        if valid:
            return valid[0], "right"
        return None, None

    # 按深度分组 (容差 0.15m)
    valid.sort(key=lambda d: d.depth_m)
    layers = []
    current_layer = [valid[0]]
    for d in valid[1:]:
        if abs(d.depth_m - current_layer[0].depth_m) < 0.15:
            current_layer.append(d)
        else:
            layers.append(current_layer)
            current_layer = [d]
    layers.append(current_layer)

    # 选最前层 (深度最小的那层)
    front_layer = layers[0]
    print(f"  检测到 {len(layers)} 层货架, 选最前层 ({len(front_layer)} 个盒子, 深度~{front_layer[0].depth_m:.2f}m)")
    for i, layer in enumerate(layers):
        depths = [f"{d.depth_m:.2f}" for d in layer]
        print(f"    第{i+1}层: {len(layer)}个盒子 depth=[{', '.join(depths)}]m")

    # 在最前层选 X 方向离中心最远的 (始终从右侧抓取)
    img_center_x = 320
    target = max(front_layer, key=lambda d: abs((d.bbox[0] + d.bbox[2]) / 2 - img_center_x))
    return target, "right"


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
def _layer_management(arm: "DobotArm"):
    """货架层管理子菜单: 切换、记录、删除层"""
    global _current_layer_idx

    while True:
        print(f"\n── 货架层管理 ──")
        if _current_layer_idx:
            print(f"  当前层: 第{_current_layer_idx}层 ({_shelf_layer_poses.get(_current_layer_idx, (None, '?'))[1]})")
        else:
            print(f"  当前: HOME (未选层)")
        if _shelf_layer_poses:
            for idx in sorted(_shelf_layer_poses):
                p, name = _shelf_layer_poses[idx]
                marker = " ← 当前" if idx == _current_layer_idx else ""
                print(f"    第{idx}层: {name}  pose=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f},{p[3]:.0f},{p[4]:.0f},{p[5]:.0f}){marker}")
        else:
            print(f"  (无已记录层。拖动机械臂到目标层后, 输入 s 保存)")
        print(f"\n  操作:")
        print(f"    <数字>   ← 切换并移动到该层")
        print(f"    s       ← 记录当前法兰位姿为新层")
        print(f"    s <数字> ← 用当前位姿覆盖指定层")
        print(f"    d <数字> ← 删除指定层记录")
        print(f"    q       ← 返回主菜单")

        choice = input("  层管理 > ").strip()
        if choice.lower() == "q":
            return

        if choice.lower().startswith("s"):
            # 保存当前位姿
            try:
                pose = arm.get_pose()
            except Exception:
                print("  无法获取当前位姿")
                continue

            parts = choice.split()
            if len(parts) >= 2:
                try:
                    idx = int(parts[1])
                except ValueError:
                    print("  格式: s <数字>")
                    continue
            else:
                # 自动编号: 最大已有编号 + 1
                idx = max(_shelf_layer_poses.keys()) + 1 if _shelf_layer_poses else 1

            name = input('  层名称 (回车默认"第{}层"): '.format(idx)).strip()
            if not name:
                name = f"第{idx}层"

            _shelf_layer_poses[idx] = (list(pose[:6]), name)
            _current_layer_idx = idx
            save_shelf_layers()
            print(f"  ✓ 第{idx}层 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice.lower().startswith("d"):
            parts = choice.split()
            if len(parts) < 2:
                print("  格式: d <数字>")
                continue
            try:
                idx = int(parts[1])
            except ValueError:
                print("  格式: d <数字>")
                continue
            if idx not in _shelf_layer_poses:
                print(f"  第{idx}层不存在")
                continue
            del _shelf_layer_poses[idx]
            if _current_layer_idx == idx:
                _current_layer_idx = max(_shelf_layer_poses.keys()) if _shelf_layer_poses else None
            save_shelf_layers()
            print(f"  ✓ 第{idx}层已删除, 当前→{'HOME' if _current_layer_idx is None else f'第{_current_layer_idx}层'}")
            continue

        # 切换层
        try:
            idx = int(choice)
        except ValueError:
            print("  无效输入, 请输入数字 / s / d / q")
            continue

        if idx not in _shelf_layer_poses:
            print(f"  第{idx}层未记录! 先拖机械臂到对应位置, 然后 s {idx} 保存")
            continue

        _current_layer_idx = idx
        save_shelf_layers()
        go_to_layer_via_clearance(arm, idx)
        return  # 切换后返回主菜单


def _place_point_management(arm: "DobotArm"):
    """放置点管理子菜单: 记录、移动到、删除放置点"""
    global _place_point_pose

    while True:
        print(f"\n── 放置点管理 ──")
        if _place_point_pose:
            pose, name = _place_point_pose
            print(f"  当前放置点: {name}  pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
        else:
            print("  当前: 未记录放置点")
        print("\n  操作:")
        print("    s       ← 记录当前法兰位姿为放置点")
        print("    g       ← 移动到放置点")
        print("    d       ← 删除放置点")
        print("    q       ← 返回主菜单")

        choice = input("  放置点 > ").strip().lower()
        if choice == "q":
            return

        if choice == "s":
            try:
                pose = arm.get_pose()
            except Exception:
                print("  无法获取当前位姿")
                continue
            name = input('  放置点名称 (回车默认"放置点"): ').strip()
            if not name:
                name = "放置点"
            _place_point_pose = (list(pose[:6]), name)
            save_place_point()
            print(f"  ✓ 放置点 ({name}) 已记录: pose=({pose[0]:.0f},{pose[1]:.0f},{pose[2]:.0f},{pose[3]:.0f},{pose[4]:.0f},{pose[5]:.0f})")
            continue

        if choice == "d":
            _place_point_pose = None
            save_place_point()
            print("  ✓ 放置点已删除")
            continue

        if choice == "g":
            go_to_place_point(arm)
            return

        print("  无效输入, 请输入 s / g / d / q")


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

    # 回初始观察位姿
    print("\n[Home] 回到预设初始位姿 ...")
    arm.go_home()
    time.sleep(2)  # 等待运动完成

    # ── 货架层管理 ──
    load_shelf_layers()
    load_place_point()
    if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
        go_to_layer_via_clearance(arm, _current_layer_idx)

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
    print("  - 输入 l   ← 货架层管理 (切换/记录/删除)")
    print("  - 输入 p   ← 放置点管理 (记录/移动/删除)")
    print("  - 输入 v   ← 实时画面预览 (YOLO 检测)")
    print("  - 输入 j   ← 手动点动 (wasd/e/r 微调法兰)")
    print("  - 输入 q   ← 退出")
    print("=" * 60)

    try:
        while True:
            if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                _, layer_name = _shelf_layer_poses[_current_layer_idx]
                prompt = f"\n[第{_current_layer_idx}层 {layer_name}] 按 Enter 抓取 | l 层管理 | p 放置点 | v 预览 | j 点动 | q 退出 > "
            else:
                prompt = "\n[HOME] 按 Enter 抓取 | l 层管理 | p 放置点 | v 预览 | j 点动 | q 退出 > "
            cmd = input(prompt).strip()
            if cmd.lower() == "q":
                break

            if cmd.lower() == "j":
                step = 20  # mm
                j6_step = 5.0  # deg
                print(f"\n  点动模式 (位置步长 {step}mm, J6步长 {j6_step:.0f}°), 输入 q 退出")
                print(f"  wasd = XY, e/r = Z, z/x = J6 -/+")
                print(f"  =/+ 放大步长, - 缩小步长")
                while True:
                    jc = input("  点动 > ").strip().lower()
                    if jc == "q":
                        break

                    if jc in ("-", "=", "+"):
                        if jc == "-":
                            step = max(1, int(step / 1.5))
                            j6_step = max(1.0, j6_step / 1.5)
                        else:
                            step = min(100, int(step * 1.5))
                            j6_step = min(45.0, j6_step * 1.5)
                        print(f"  步长 → {step}mm / {j6_step:.0f}°")
                        continue

                    if jc in ("z", "x"):
                        try:
                            joints = arm.get_joints()
                        except Exception as e:
                            print(f"  无法获取关节角: {e}")
                            continue
                        dj6 = -j6_step if jc == "z" else j6_step
                        arm.move_j6(joints[5] + dj6)
                        time.sleep(0.5)
                        continue

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
                    resp = arm.movel(tx, ty, tz, speed=50, allow_low_z=_current_layer_allows_low_z())
                    time.sleep(0.5)
                continue

            if cmd.lower() == "l":
                # 货架层管理
                _layer_management(arm)
                continue

            if cmd.lower() == "p":
                _place_point_management(arm)
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
                cv2.putText(viz, f"{d.class_name or '?'} {d.depth_m:.2f}m" if d.depth_m else f"{d.class_name or '?'}",
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

            # ── 根据自动判定的抓取模式选择偏移参数 ──
            is_side_grasp = "side" in grip_mode
            grasp_offset_m = SIDE_GRASP_OFFSET_M if is_side_grasp else TOP_GRASP_OFFSET_M
            grip_label = "侧面抓" if is_side_grasp else "上面抓"
            fingertip_w = np.array([wx, wy, wz]) + np.array(grasp_offset_m)
            flange_pt   = fingertip_w - HAND_LENGTH * fz_dir   # 法兰在手背面

            # 悬停: 世界Z向上抬高一段安全距离
            approach_pt = flange_pt + np.array([0.0, 0.0, APPROACH_LIFT_M])

            # 抓取: 沿手指方向前进一小段探入
            grasp_pt = flange_pt + GRASP_ADVANCE_M * fz_dir

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
                 tuned_offset_m = np.array(grasp_offset_m) + np.array([offset_x, offset_y, offset_z]) / 1000.0
                 print(f"\n── 目标位姿 (可微调) ──")
                 print(f"  模式: {grip_label} ({grip_mode})")
                 print(f"  当前参数: offset=({grasp_offset_m[0]*1000:.0f},{grasp_offset_m[1]*1000:.0f},{grasp_offset_m[2]*1000:.0f})mm")
                 print(f"  固化建议: {'SIDE' if is_side_grasp else 'TOP'}_GRASP_OFFSET_M=({tuned_offset_m[0]:+.3f}, {tuned_offset_m[1]:+.3f}, {tuned_offset_m[2]:+.3f})")
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
                     arm.movel(new_x, new_y, new_z, speed=100, allow_low_z=_current_layer_allows_low_z())
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
                     arm.movel(new_x, new_y, new_z, speed=60, allow_low_z=_current_layer_allows_low_z())
                     time.sleep(1)
                     continue
                 elif choice == "p":
                     print(f"\n  预览: 移动到 ({ax+offset_x:.0f}, {ay+offset_y:.0f}, {az+offset_z:.0f})mm")
                     arm.movel(ax + offset_x, ay + offset_y, az + offset_z, speed=100,
                               allow_low_z=_current_layer_allows_low_z())
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

            # 上面抓取: 先把第六轴转到手心向下的实测角度
            if not is_side_grasp:
                print(f"  上面抓取 → J6 转到手心向下角度 {TOP_J6_DEG:.1f}°")
                arm.move_j6(TOP_J6_DEG)
                time.sleep(0.5)
                flange_pose = arm.get_pose()
                R_f = transformer._rpy_to_rot(flange_pose[3], flange_pose[4], flange_pose[5])
                fz_dir = R_f[:, 2]
                flange_pt = fingertip_w - HAND_LENGTH * fz_dir
                grasp_pt = flange_pt + GRASP_ADVANCE_M * fz_dir
                approach_pt = grasp_pt - TOP_APPROACH_BACKOFF_M * fz_dir
                ax, ay, az = float(approach_pt[0]*1000), float(approach_pt[1]*1000), float(approach_pt[2]*1000)
                gx, gy, gz = float(grasp_pt[0]*1000), float(grasp_pt[1]*1000), float(grasp_pt[2]*1000)

            # 2. 悬停/预备位 (侧抓: 世界Z抬高; 上抓: 抓取轴线上后退)
            allow_low_z = _current_layer_allows_low_z()
            if not arm.movel(ax + offset_x, ay + offset_y, az + offset_z, speed=150,
                             allow_low_z=allow_low_z):
                print("  [中止] 悬停位运动失败/报警, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                continue
            time.sleep(1.5)

            # 3. 前进抓取 (沿手指方向探入 GRASP_ADVANCE_M)
            if not arm.movel(gx + offset_x, gy + offset_y, gz + offset_z, speed=50,
                             allow_low_z=allow_low_z):
                print("  [中止] 抓取位运动失败/报警, 本次不闭手")
                if _current_layer_idx and _current_layer_idx in _shelf_layer_poses:
                    go_to_layer(arm, _current_layer_idx)
                else:
                    arm.go_home()
                continue
            time.sleep(1.5)

            # 4. 抓取 (动作二) 并等待 5s 握紧
            if not SKIP_HAND:
                if hand.close(top=not is_side_grasp):
                    print("  灵巧手 → 动作二 (闭合), 等待 3s ...")
                    time.sleep(3.0)
                else:
                    print("  灵巧手不可用, 继续执行机械臂安全退回")

            # 5. 后退 (沿法兰-Z 退 15cm, 离开货架)
            retreat_pt = np.array([gx+offset_x, gy+offset_y, gz+offset_z]) - 150.0 * fz_dir
            if not arm.movel(float(retreat_pt[0]), float(retreat_pt[1]), float(retreat_pt[2]), speed=100,
                             allow_low_z=allow_low_z):
                print("  [停止] 后退失败/报警, 停在当前位置, 不继续放置流程")
                continue
            time.sleep(1.5)

            # 6. 放置并回到当前层观察位
            finish_pick_cycle(arm, hand, _current_layer_idx)
            time.sleep(2)

            print("抓取完成! 按 Enter 继续 / q 退出")

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
