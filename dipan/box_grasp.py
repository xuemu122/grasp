#!/usr/bin/env python3
"""
轻量盒子抓取位姿计算 + Orca 灵巧手控制
输入: YOLO bbox 检测结果 + OCR 标签
输出: 最外侧盒子的抓取位姿 (机械臂末端坐标 + 旋转角)
执行: 机械臂 MovL + Orca 灵巧手开合
"""

import sys
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Orca 灵巧手 — 按需导入，仅 --real 模式需要
OrcaHand = None
OrcaJointPositions = None

def _init_orca():
    """按需导入 orca_core，自动加入 local path 避免安装"""
    global OrcaHand, OrcaJointPositions
    if OrcaHand is None:
        orca_path = "/home/nvidia/orca_core"
        if orca_path not in sys.path:
            sys.path.insert(0, orca_path)
        from orca_core import OrcaHand as _O, OrcaJointPositions as _J
        OrcaHand, OrcaJointPositions = _O, _J


@dataclass
class BoxDetection:
    """YOLO 检测到的单个盒子（也可从 orbbec_detect.DetectionResult 转换）"""
    x1: float          # bbox 左上角 x (像素)
    y1: float          # bbox 左上角 y (像素)
    x2: float          # bbox 右下角 x (像素)
    y2: float          # bbox 右下角 y (像素)
    depth: float = 0.0 # RealSense 深度 (米)
    label: str = ""    # OCR 识别的标签

    @classmethod
    def from_detection_result(cls, det) -> "BoxDetection":
        """从 orbbec_detect.DetectionResult 转换"""
        return cls(
            x1=det.bbox[0], y1=det.bbox[1],
            x2=det.bbox[2], y2=det.bbox[3],
            depth=det.depth_m or 0.0,
            label=det.ocr_text or "",
        )


@dataclass
class GraspPose:
    """抓取位姿"""
    x: float       # 机械臂坐标系 X (米)
    y: float       # 机械臂坐标系 Y (米)
    z: float       # 机械臂坐标系 Z (米，桌面高度 + 盒子半高 + 安全偏置)
    yaw: float     # 绕 Z 轴旋转角 (弧度)，夹爪开口方向
    label: str = ""


class OrcaHandGripper:
    """
    Orca 灵巧手包装 — 提供开手 / 握拳两个姿态
    17 关节灵巧手对盒子做 power_grasp 围抱式抓取
    """

    # 预设姿态（fraction 比例: 0=极限伸展, 1=极限弯曲）
    OPEN_HAND = {
        "thumb_cmc": 0.70, "thumb_abd": 0.80, "thumb_mcp": 0.85, "thumb_dip": 0.75,
        "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
        "index_mcp": 0.15, "middle_mcp": 0.15, "ring_mcp": 0.15, "pinky_mcp": 0.15,
        "index_pip": 0.10, "middle_pip": 0.10, "ring_pip": 0.10, "pinky_pip": 0.10,
        "wrist": 0.30,
    }

    POWER_GRASP = {
        "thumb_cmc": 0.35, "thumb_abd": 0.55, "thumb_mcp": 0.20, "thumb_dip": 0.85,
        "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
        "index_mcp": 0.85, "middle_mcp": 0.85, "ring_mcp": 0.85, "pinky_mcp": 0.85,
        "index_pip": 0.90, "middle_pip": 0.90, "ring_pip": 0.90, "pinky_pip": 0.90,
        "wrist": 0.55,
    }

    def __init__(self, config_path: str | None = None):
        _init_orca()
        self.hand = OrcaHand(config_path=config_path)

    def connect(self):
        success, msg = self.hand.connect()
        if not success:
            raise RuntimeError(f"灵巧手连接失败: {msg}")
        print(f"灵巧手已连接: {msg}")
        self.hand.init_joints()

    def disconnect(self):
        self.hand.disable_torque()
        self.hand.disconnect()
        print("灵巧手已断开")

    def _fractions_to_positions(self, fractions: dict) -> OrcaJointPositions:
        """将 fraction(0~1) 转为实际关节角度"""
        pose = dict(self.hand.config.neutral_position)
        for joint, frac in fractions.items():
            if joint not in self.hand.config.joint_roms_dict:
                continue
            lo, hi = self.hand.config.joint_roms_dict[joint]
            pose[joint] = lo + frac * (hi - lo)
        return OrcaJointPositions.from_dict(pose)

    def open(self, num_steps: int = 16):
        """张开手指"""
        target = self._fractions_to_positions(self.OPEN_HAND)
        self.hand.set_joint_positions(target, num_steps=num_steps, step_size=0.02)
        print("灵巧手 → 张开")

    def grasp(self, num_steps: int = 24):
        """握拳抓取 (power grasp)"""
        target = self._fractions_to_positions(self.POWER_GRASP)
        self.hand.set_joint_positions(target, num_steps=num_steps, step_size=0.02)
        print("灵巧手 → 握拳抓取")


class LightweightBoxGrasper:
    """
    轻量盒子抓取器 — 手腕根部相机 (eye-in-hand) 版
    - RealSense 安在机械臂法兰根部 (手腕处), 靠近法兰
    - 用法: 机械臂先移动到"观察位姿"拍照，
            算好目标点后再移动到"抓取位姿"
    - 深度 + 像素 → 相机坐标系 3D → flange → robot base 坐标
    """

    # ---- DOBOT Nova5 + Orca 灵巧手 物理参数 ----
    HAND_LENGTH = 0.35       # 末端执行器总长 (法兰 → 指尖, 米)
    HAND_WEIGHT = 1.2        # 末端重量 (kg)
    HAND_COG = (0, 0, 0.10)  # 重心相对法兰位置 (x,y,z 米)

    def __init__(self,
                 camera_fov_h_deg: float = 69.0,    # 相机水平 FOV (度), D435 默认 69
                 image_width: int = 640,
                 image_height: int = 480,
                 # ---- 手眼标定: 相机 → 法兰 (手腕根部, 几乎就是法兰) ----
                 cam_to_flange_trans: Tuple[float, float, float] = (0.0, 0.0, 0.02),
                 cam_to_flange_rot_deg: Tuple[float, float, float] = (0, 0, 0),
                 # 抓取偏置
                 grasp_z_offset: float = 0.05,       # 抓取时指尖距盒面额外偏置
                 ):
        self.img_w = image_width
        self.img_h = image_height
        self.grasp_z_offset = grasp_z_offset

        # ---- 等效焦距 (像素) ----
        self.fov_h = np.deg2rad(camera_fov_h_deg)
        self.focal_px = (self.img_w / 2.0) / np.tan(self.fov_h / 2.0)

        # ---- 手眼标定矩阵 T_camera→flange (4x4) ----
        tx, ty, tz = cam_to_flange_trans
        rx, ry, rz = np.deg2rad(cam_to_flange_rot_deg)
        self.T_cam_to_flange = self._make_transform(tx, ty, tz, rx, ry, rz)

        # ---- 机械臂当前位姿 (运行时更新) ----
        self.robot_flange_pose: Optional[Tuple[float,float,float,float,float,float]] = None

    def _make_transform(self, tx, ty, tz, rx, ry, rz):
        """构建 4x4 齐次变换矩阵 (XYZ 欧拉角, 外旋, 度 → 弧度)"""
        R = np.eye(3)
        # Rx
        cx, sx = np.cos(rx), np.sin(rx)
        R = R @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
        # Ry
        cy, sy = np.cos(ry), np.sin(ry)
        R = R @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
        # Rz
        cz, sz = np.cos(rz), np.sin(rz)
        R = R @ np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = [tx, ty, tz]
        return T

    def set_flange_pose(self, x, y, z, rx, ry, rz):
        """
        设置机械臂当前法兰位姿 (世界坐标系)
        x,y,z: 米, rx,ry,rz: 度 (机械臂 RPY)
        """
        self.robot_flange_pose = (x, y, z, rx, ry, rz)

    def pixel_depth_to_world(self, u: float, v: float, depth: float
                             ) -> Tuple[float, float, float]:
        """
        像素 + 深度 → 世界 (robot base) 3D 坐标
        depth: RealSense 沿光轴距离 (米)
        """
        if self.robot_flange_pose is None:
            raise RuntimeError("请先调用 set_flange_pose() 设置机械臂当前位姿")

        # 1. 像素 → 相机坐标系下的射线方向 (Z=光轴向前)
        dx = (u - self.img_w / 2.0) / self.focal_px
        dy = (v - self.img_h / 2.0) / self.focal_px
        ray = np.array([dx, dy, 1.0])
        ray /= np.linalg.norm(ray)
        point_cam = ray * depth  # 相机坐标系 3D

        # 2. 相机 → 法兰 (手眼标定)
        p_cam_h = np.append(point_cam, 1.0)
        p_flange = self.T_cam_to_flange @ p_cam_h

        # 3. 法兰 → 世界 (机械臂正运动学)
        fx, fy, fz, frx, fry, frz = self.robot_flange_pose
        T_flange_to_world = self._make_transform(fx/1000, fy/1000, fz/1000,
                                                  np.deg2rad(frx), np.deg2rad(fry), np.deg2rad(frz))
        p_world = T_flange_to_world @ p_flange
        return p_world[0], p_world[1], p_world[2]

    def find_outermost_box(self, detections: List[BoxDetection]) -> Optional[BoxDetection]:
        """找深度最近(最靠前)的盒子"""
        if not detections:
            return None
        with_depth = [d for d in detections if d.depth > 0]
        if with_depth:
            return min(with_depth, key=lambda d: d.depth)
        scored = [(d.y2 * 100 + (d.x2-d.x1)*(d.y2-d.y1), d) for d in detections]
        scored.sort(key=lambda x: x[0], reverse=True)
        return scored[0][1]

    def compute_grasp(self, det: BoxDetection) -> GraspPose:
        """从 bbox 计算抓取位姿 (eye-in-hand 模式)"""
        cx = (det.x1 + det.x2) / 2.0
        cy = (det.y1 + det.y2) / 2.0

        if det.depth > 0 and self.robot_flange_pose is not None:
            wx, wy, wz = self.pixel_depth_to_world(cx, cy, det.depth)
            # 沿光轴方向收一点安全偏置
            wz += self.grasp_z_offset
        else:
            raise RuntimeError("需要深度值和机械臂位姿")

        bw = det.x2 - det.x1
        bh = det.y2 - det.y1
        yaw = np.pi / 2 if bw > bh else 0.0

        return GraspPose(x=wx, y=wy, z=wz, yaw=yaw, label=det.label)

    def convert_to_movl(self, pose: GraspPose, speed: float = 100) -> str:
        """将位姿转成 DOBOT MovL 指令 (端口 29999)"""
        rx = 180.0
        ry = pose.yaw * 180 / np.pi
        rz = 0.0
        x = pose.x * 1000
        y = pose.y * 1000
        z = pose.z * 1000
        return f"MovL(x={x:.1f}, y={y:.1f}, z={z:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}, speed={speed})"

    def get_grasp_sequence(self, detections: List[BoxDetection],
                           approach_z: float = 0.15) -> List[str]:
        """
        生成 DOBOT 机械臂 + Orca 灵巧手 完整抓取动作序列:
        approach_z 默认 15cm —— 手长 35cm, 需要足够空间避免碰撞
        """
        det = self.find_outermost_box(detections)
        if det is None:
            return []

        grasp = self.compute_grasp(det)

        rx = 180.0
        ry = grasp.yaw * 180 / np.pi
        rz = 0.0
        x = grasp.x * 1000
        y = grasp.y * 1000
        z = grasp.z * 1000
        approach_z_mm = (grasp.z + approach_z) * 1000

        return [
            # 1. 灵巧手张开
            ("hand_open", None),
            # 2. 机械臂移动到盒子正上方
            ("robot",
             f"MovL(x={x:.1f}, y={y:.1f}, z={approach_z_mm:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}, speed=200)"),
            # 3. 下降到抓取点
            ("robot",
             f"MovL(x={x:.1f}, y={y:.1f}, z={z:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}, speed=50)"),
            # 4. 灵巧手握拳抓取
            ("hand_grasp", None),
            # 5. 机械臂抬升
            ("robot",
             f"MovL(x={x:.1f}, y={y:.1f}, z={approach_z_mm:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}, speed=100)"),
        ]


# ========== 执行引擎 ==========
def execute_grasp_sequence(gripper: OrcaHandGripper,
                           robot_socket,   # DOBOT socket (端口 29999)
                           sequence: List[tuple]):
    """按序执行: 灵巧手开合 + DOBOT MovL 指令"""
    for step_type, cmd in sequence:
        if step_type == "hand_open":
            gripper.open()
        elif step_type == "hand_grasp":
            gripper.grasp()
        elif step_type == "robot":
            print(f">>> DOBOT: {cmd}")
            robot_socket.sendall((cmd + "\n").encode())
            # 等待机械臂到位 (简化: 收返回后 sleep 确保完成)
            resp = robot_socket.recv(1024)
            print(f"<<< {resp.decode()}")
            time.sleep(2.0)  # 每段 MovL 等待到位


# ========== 集成 orbbec_detect 识别系统 ==========
def run_with_realsense():
    """
    完整集成: RealSense 相机 + YOLO + OCR + Orca 灵巧手 + DOBOT 机械臂
    从相机读一帧 → 识别最外侧盒子 → 计算抓取位姿 → 执行
    """
    import sys
    import socket
    import os

    # 把识别模块路径加入
    _repo_root = os.environ.get("GRASP_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    recognition_dir = os.environ.get("GRASP_RECOGNITION_DIR", os.path.join(_repo_root, "识别"))
    sys.path.insert(0, recognition_dir)
    from orbbec_detect import MedicineLocatorSystem

    print("=" * 50)
    print("  盒子抓取系统集成")
    print("  RealSense + YOLO + OCR + Orca 灵巧手 + DOBOT")
    print("=" * 50)

    # ---- 1. 初始化识别系统 ----
    default_model_path = os.path.join(recognition_dir, "best.engine")
    if not os.path.exists(default_model_path):
        default_model_path = os.path.join(recognition_dir, "best.pt")
    model_path = os.getenv("YOLO_MODEL_PATH", default_model_path)
    print(f"  视觉模型: {model_path}")
    locator = MedicineLocatorSystem(
        model_path=model_path,
        mapping_path=os.path.join(recognition_dir, "shelf_mapping.yaml"),
        conf_threshold=0.5,
        use_ocr=False,
    )
    locator.init_camera()

    # ---- 2. 抓取器 (手腕根部相机, Nova5 + Orca) ----
    grasper = LightweightBoxGrasper(
        cam_to_flange_trans=(0.0, 0.0, 0.02),      # 相机在法兰根部, 几乎同轴
        cam_to_flange_rot_deg=(0, 0, 0),             # 朝向和法兰一致
        grasp_z_offset=0.05,
    )

    # ---- 3. 连接硬件 ----
    # 灵巧手
    gripper = OrcaHandGripper()
    gripper.connect()

    # 机械臂 (DOBOT Nova5)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect(("192.168.1.6", 29999))

    # ---- 设置负载 & 工具坐标系 (Nova5 + Orca 35cm/1.2kg) ----
    setup_cmds = [
        f"SetPayload(mass=1.2, centerX=0, centerY=0, centerZ=0.10)",
        "Tool(index=1)",
    ]
    for scmd in setup_cmds:
        print(f">>> DOBOT: {scmd}")
        sock.sendall((scmd + "\n").encode())
        resp = sock.recv(1024)
        print(f"<<< {resp.decode()}")

    try:
        while True:
            cmd = input("\n按 Enter 拍照识别并抓取 (q 退出): ")
            if cmd.strip().lower() == "q":
                break

            # 拍一帧
            color_img, depth_img = locator.camera.read()
            if color_img is None:
                print("相机未就绪")
                continue

            # YOLO + OCR + 深度
            detections = locator.process_frame(color_img, depth_img)
            if not detections:
                print("未检测到盒子")
                continue

            # 转为抓取模块的 BoxDetection
            boxes = [BoxDetection.from_detection_result(d) for d in detections]

            # 打印识别结果
            print(f"\n检测到 {len(boxes)} 个盒子:")
            for i, b in enumerate(boxes):
                print(f"  [{i+1}] bbox=({b.x1},{b.y1},{b.x2},{b.y2}) "
                      f"depth={b.depth:.3f}m label={b.label}")

            outermost = grasper.find_outermost_box(boxes)
            if outermost is None:
                print("无法确定最外侧盒子")
                continue

            print(f"\n=== 选中最外侧盒子: depth={outermost.depth:.3f}m label={outermost.label} ===")
            # ---- 查询机械臂当前法兰位姿 (DOBOT GetPose) ----
            sock.sendall(b"GetPose()\n")
            resp = b""
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
                if b";" in chunk:
                    break
            # 解析 GetPose 返回: ErrorID,{"x":...,"y":...,...};
            resp_str = resp.decode()
            import re
            nums = re.findall(r"[-+]?\d*\.?\d+", resp_str)
            if len(nums) >= 6:
                grasper.set_flange_pose(
                    float(nums[-6]), float(nums[-5]), float(nums[-4]),
                    float(nums[-3]), float(nums[-2]), float(nums[-1]))
                print(f"当前法兰位姿: ({nums[-6]},{nums[-5]},{nums[-4]}) rpy=({nums[-3]},{nums[-2]},{nums[-1]})")
            else:
                print(f"警告: 无法解析法兰位姿, 返回: {resp_str}")
                continue

            # 计算抓取位姿
            pose = grasper.compute_grasp(outermost)
            print(f"抓取位姿: x={pose.x:.3f} y={pose.y:.3f} z={pose.z:.3f} yaw={pose.yaw:.3f}")

            # 生成并执行动作序列
            sequence = grasper.get_grasp_sequence(boxes)
            print("\n执行抓取动作...")
            execute_grasp_sequence(gripper, sock, sequence)
            print("抓取完成!")

    finally:
        gripper.disconnect()
        sock.close()
        locator.camera.release()


# ========== 示例: 接 YOLO 检测结果 ==========
if __name__ == "__main__":
    # 如果传 --real 参数则用真实硬件集成
    if len(sys.argv) > 1 and sys.argv[1] == "--real":
        run_with_realsense()
    else:
        # 模拟模式
        mock_detections = [
            BoxDetection(100, 200, 180, 280, 0.50, "A001"),
            BoxDetection(200, 195, 280, 275, 0.48, "A002"),
            BoxDetection(300, 190, 380, 270, 0.46, "A003"),
            BoxDetection(400, 185, 480, 265, 0.44, "A004"),
        ]

        grasper = LightweightBoxGrasper(
            cam_to_flange_trans=(0.0, 0.0, 0.02),   # 手腕根部
            cam_to_flange_rot_deg=(0, 0, 0),
            grasp_z_offset=0.05,
        )
        # 模拟机械臂当前法兰位姿
        grasper.set_flange_pose(0.4, 0.0, 0.3, 180, 0, 0)

        outermost = grasper.find_outermost_box(mock_detections)
        print(f"最外侧盒子: bbox=({outermost.x1},{outermost.y1},{outermost.x2},{outermost.y2}) "
              f"depth={outermost.depth:.3f}m label={outermost.label}")

        pose = grasper.compute_grasp(outermost)
        print(f"抓取位姿: x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}, yaw={pose.yaw:.3f}")

        sequence = grasper.get_grasp_sequence(mock_detections, approach_z=0.08)
        print("\n完整动作序列:")
        for step_type, cmd in sequence:
            print(f"  [{step_type}] {cmd}")

        print("\n要使用真实硬件，运行: python box_grasp.py --real")
