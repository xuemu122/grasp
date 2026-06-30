#!/usr/bin/env python3
"""Experimental Nova5 + DRP bridge test.

This is a compatibility harness, not a trained Nova5 policy.
The DRP checkpoint in this repo was trained for Franka Panda, so this script
pads Nova5's 6-DoF joints to 7 DoF for a first connection test.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np
import torch
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[1]
DRP_ROOT = REPO_ROOT / "drp"
POINTNET2_ROOT = DRP_ROOT / "pointnet2_ops"
ROB0FIN_ROOT = DRP_ROOT / "robofin"
VISION_ROOT = REPO_ROOT / "识别"
DIPAN_ROOT = REPO_ROOT / "dipan"

for path in (REPO_ROOT, POINTNET2_ROOT, DRP_ROOT, ROB0FIN_ROOT, VISION_ROOT, DIPAN_ROOT):
    sys.path.insert(0, str(path))

from drp.model.impact import IMPACT
from robofin.torch_urdf import TorchURDF
from dobot_arm import DobotArm
import hand_eye as hand_eye_mod
from hand_eye import HandEyeTransformer
from orbbec_detect import RealSenseCamera


NOVA5_URDF = (
    REPO_ROOT
    / "dobot_ws"
    / "src"
    / "DOBOT_6Axis_ROS2_V4"
    / "dobot_rviz"
    / "urdf"
    / "nova5_robot.urdf"
)
NOVA5_MESH_ROOT = (
    REPO_ROOT / "dobot_ws" / "src" / "DOBOT_6Axis_ROS2_V4" / "dobot_rviz" / "meshes"
)
NOVA5_HOME_RAD = np.array([0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0], dtype=np.float32)
NOVA5_HOME_DEG = np.rad2deg(NOVA5_HOME_RAD)
SHELF_LAYERS_JSON = DIPAN_ROOT / "shelf_layers.json"


def _ensure_float_array(values: Iterable[float], length: int) -> np.ndarray:
    arr = np.asarray(list(values), dtype=np.float32)
    if arr.shape != (length,):
        raise ValueError(f"expected {length} values, got {arr.shape}")
    return arr


def _pad_nova5_to_7(joints_rad: np.ndarray) -> np.ndarray:
    joints_rad = _ensure_float_array(joints_rad, 6)
    return np.concatenate([joints_rad, np.array([0.0], dtype=np.float32)])


def _clip_to_limits(values_deg: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.clip(values_deg, lower, upper)


def _pad_scene_points(scene_pcd: np.ndarray, target_n: int) -> np.ndarray:
    if scene_pcd.shape[0] >= target_n:
        return scene_pcd[:target_n]
    if scene_pcd.shape[0] == 0:
        return np.zeros((target_n, 3), dtype=np.float32)
    idx = np.random.choice(scene_pcd.shape[0], target_n - scene_pcd.shape[0], replace=True)
    return np.concatenate([scene_pcd, scene_pcd[idx]], axis=0)


def _depth_to_vis(depth_img: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_img)
    if depth.ndim != 2:
        raise ValueError(f"expected a single-channel depth image, got {depth.shape}")
    depth = depth.astype(np.float32, copy=False)
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)
    lo = np.percentile(depth[valid], 5)
    hi = np.percentile(depth[valid], 95)
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        hi = lo + 1.0
    norm = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    norm = (norm * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


def _make_pointcloud_view(
    robot_pcd: np.ndarray,
    scene_pcd: np.ndarray,
    size: tuple[int, int] = (720, 720),
) -> np.ndarray:
    h, w = size
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (18, 18, 18)

    # Fixed workspace bounds in meters. We show the X-Z plane because it makes
    # the shelf height and reach easy to inspect during X11 preview.
    x_min, x_max = -0.9, 0.9
    z_min, z_max = -0.1, 1.5
    margin = 36
    plot_w = w - 2 * margin
    plot_h = h - 2 * margin

    def _to_px(points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0, 2), dtype=np.int32)
        pts = np.asarray(points, dtype=np.float32)
        x = np.clip((pts[:, 0] - x_min) / max(x_max - x_min, 1e-6), 0.0, 1.0)
        z = np.clip((pts[:, 2] - z_min) / max(z_max - z_min, 1e-6), 0.0, 1.0)
        px = margin + (x * plot_w).astype(np.int32)
        py = h - margin - (z * plot_h).astype(np.int32)
        return np.stack([px, py], axis=1)

    # Frame and guide lines.
    cv2.rectangle(canvas, (margin, margin), (w - margin, h - margin), (80, 80, 80), 1)
    for frac in (0.25, 0.5, 0.75):
        x = margin + int(plot_w * frac)
        z = margin + int(plot_h * frac)
        cv2.line(canvas, (x, margin), (x, h - margin), (45, 45, 45), 1)
        cv2.line(canvas, (margin, z), (w - margin, z), (45, 45, 45), 1)

    cv2.putText(canvas, "X-Z point cloud", (margin, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
    cv2.putText(canvas, "X", (w - 22, h - margin + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
    cv2.putText(canvas, "Z", (10, margin + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

    scene_px = _to_px(scene_pcd)
    robot_px = _to_px(robot_pcd)

    for x, y in scene_px:
        if margin <= x < w - margin and margin <= y < h - margin:
            cv2.circle(canvas, (int(x), int(y)), 1, (60, 200, 70), -1)
    for x, y in robot_px:
        if margin <= x < w - margin and margin <= y < h - margin:
            cv2.circle(canvas, (int(x), int(y)), 1, (230, 120, 40), -1)

    cv2.putText(
        canvas,
        f"scene={scene_pcd.shape[0]}  robot={robot_pcd.shape[0]}",
        (margin, h - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (210, 210, 210),
        1,
    )
    return canvas


def _make_status_panel(
    current_deg: np.ndarray,
    goal_deg: np.ndarray,
    next_deg: np.ndarray,
    scene_n: int,
    execute: bool,
    camera_on: bool,
    model_id: str,
    size: tuple[int, int] = (720, 360),
) -> np.ndarray:
    h, w = size
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (24, 24, 24)

    def put(line: str, y: int, color=(230, 230, 230), scale=0.58):
        cv2.putText(canvas, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    put("Nova5 + DRP preview", 34, (255, 255, 255), 0.8)
    put(f"model: {model_id}", 64, (180, 180, 180), 0.52)
    put(f"camera: {'on' if camera_on else 'off'}   execute: {'on' if execute else 'off'}", 90, (180, 220, 255), 0.56)
    put(f"scene points: {scene_n}", 124)
    put(f"current deg: {np.round(current_deg, 2).tolist()}", 162)
    put(f"goal deg:    {np.round(goal_deg, 2).tolist()}", 198)
    put(f"next deg:    {np.round(next_deg, 2).tolist()}", 236, (120, 220, 120))
    put("window: X11 / MobaXterm", 292, (120, 180, 255), 0.56)
    return canvas


def _stack_preview(color_img: Optional[np.ndarray], depth_img: Optional[np.ndarray], pointcloud_img: np.ndarray, status_img: np.ndarray) -> np.ndarray:
    def _fit(img: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
        if img is None:
            return np.zeros((target_h, target_w, 3), dtype=np.uint8)
        out = img
        if out.ndim == 2:
            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        if out.shape[2] == 4:
            out = cv2.cvtColor(out, cv2.COLOR_BGRA2BGR)
        return cv2.resize(out, (target_w, target_h), interpolation=cv2.INTER_AREA)

    top_left = _fit(color_img, 360, 640)
    bottom_left = _fit(_depth_to_vis(depth_img) if depth_img is not None else None, 360, 640)
    right_top = _fit(pointcloud_img, 360, 720)
    right_bottom = _fit(status_img, 360, 720)
    left = np.vstack([top_left, bottom_left])
    right = np.vstack([right_top, right_bottom])
    return np.hstack([left, right])


def _rewrite_urdf_mesh_paths(src: Path) -> Path:
    text = src.read_text(encoding="utf-8")
    text = text.replace(
        "package://dobot_rviz/meshes/",
        str(NOVA5_MESH_ROOT).rstrip("/") + "/",
    )
    tmp_dir = Path(tempfile.gettempdir()) / "nova5_drp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    dst = tmp_dir / "nova5_robot_abs.urdf"
    dst.write_text(text, encoding="utf-8")
    return dst


def _matrix_from_xyz_rpy(xyz, rpy):
    x, y, z = xyz
    roll, pitch, yaw = rpy
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float32,
    )
    mat[:3, 3] = (x, y, z)
    return mat


def _pixel_to_robot_points(
    depth_img: np.ndarray,
    depth_scale: float,
    flange_pose_mm: Iterable[float],
    stride: int,
    max_points: int,
    hand_eye: HandEyeTransformer,
) -> np.ndarray:
    h, w = depth_img.shape[:2]
    ys = np.arange(0, h, stride, dtype=np.int32)
    xs = np.arange(0, w, stride, dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)
    sampled_depth = depth_img[grid_y, grid_x].astype(np.float32) * max(depth_scale, 1e-6)
    valid = (sampled_depth > 0.05) & (sampled_depth < 2.5)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    u = grid_x[valid].astype(np.float32)
    v = grid_y[valid].astype(np.float32)
    depth_m = sampled_depth[valid]

    depth_delta = np.maximum(depth_m - hand_eye_mod.PERSPECTIVE_COMP_Z_REF_M, 0.0)
    if hand_eye_mod.PERSPECTIVE_COMP_ENABLE:
        v_scale = 1.0 / (1.0 + hand_eye_mod.PERSPECTIVE_COMP_GAIN * depth_delta)
        v_scale = np.clip(v_scale, hand_eye_mod.PERSPECTIVE_COMP_MIN_SCALE, 1.0)
        v_adj = hand_eye_mod.CY + (v - hand_eye_mod.CY) * v_scale
    else:
        v_adj = v

    x_cam = (u - hand_eye_mod.CX) * depth_m / hand_eye_mod.FX
    y_cam = (v_adj - hand_eye_mod.CY) * depth_m / hand_eye_mod.FY
    cam_pts = np.stack([x_cam, y_cam, depth_m, np.ones_like(depth_m)], axis=1)

    pose = tuple(float(x) for x in flange_pose_mm)
    T_base_flange = _matrix_from_xyz_rpy(
        (pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0),
        tuple(np.deg2rad(pose[3:6])),
    )
    T = T_base_flange @ hand_eye.T_flange_to_cam
    base_pts = (T @ cam_pts.T).T[:, :3]

    if base_pts.shape[0] > max_points:
        idx = np.random.choice(base_pts.shape[0], max_points, replace=False)
        base_pts = base_pts[idx]
    return base_pts.astype(np.float32, copy=False)


@dataclass
class VisualEntry:
    geometry: object
    points: torch.Tensor


class Nova5Sampler:
    def __init__(self, device: torch.device, urdf_path: Path, mesh_root: Path, num_points: int = 1024):
        self.device = device
        self.num_points = num_points
        self.mesh_root = mesh_root
        self._tmp_urdf = _rewrite_urdf_mesh_paths(urdf_path)
        self.robot = TorchURDF.load(str(self._tmp_urdf), lazy_load_meshes=True, device=device)
        self.visual_entries = self._load_visual_entries()
        self.joint_limits = [
            (
                float(j.limit.lower) if j.limit is not None and j.limit.lower is not None else -np.pi,
                float(j.limit.upper) if j.limit is not None and j.limit.upper is not None else np.pi,
            )
            for j in self.robot.actuated_joints
        ]

    def close(self):
        try:
            self._tmp_urdf.unlink(missing_ok=True)
        except Exception:
            pass

    def _load_visual_entries(self) -> list[VisualEntry]:
        visual_meshes = []
        for link in self.robot.links:
            for visual in link.visuals:
                mesh = getattr(getattr(visual, "geometry", None), "mesh", None)
                if mesh is None or not getattr(mesh, "filename", None):
                    continue
                mesh_path = Path(mesh.filename)
                if not mesh_path.is_absolute():
                    mesh_path = self.mesh_root / mesh_path
                try:
                    tri = trimesh.load(mesh_path, force="mesh")
                except Exception:
                    continue
                if tri.is_empty or tri.area <= 0:
                    continue
                visual_meshes.append((visual.geometry, tri, float(tri.area)))

        if not visual_meshes:
            raise RuntimeError("Nova5 visual meshes could not be loaded")

        total_area = sum(area for _, _, area in visual_meshes)
        for idx, (geometry, tri, area) in enumerate(visual_meshes):
            count = max(16, int(round(self.num_points * area / total_area)))
            if idx == len(visual_meshes) - 1:
                count = max(16, self.num_points - sum(v.shape[0] for _, v in visual_meshes[:idx]))
            sampled = trimesh.sample.sample_surface(tri, count)[0].astype(np.float32, copy=False)
            visual_meshes[idx] = (geometry, sampled)

        return [
            VisualEntry(geometry=geometry, points=torch.as_tensor(points, device=self.device, dtype=torch.float32).unsqueeze(0))
            for geometry, points in visual_meshes
        ]

    def sample(self, joints_rad_6: np.ndarray, num_points: Optional[int] = None) -> torch.Tensor:
        q = torch.as_tensor(joints_rad_6, device=self.device, dtype=torch.float32)
        if q.ndim == 1:
            q = q.unsqueeze(0)
        fk = self.robot.visual_geometry_fk_batch(cfgs=q)
        pcs = []
        for entry in self.visual_entries:
            T = fk[entry.geometry]
            pts = entry.points.repeat(q.shape[0], 1, 1)
            rotation = T[:, :3, :3].transpose(1, 2)
            translation = T[:, :3, 3].unsqueeze(1)
            transformed = pts.matmul(rotation) + translation
            pcs.append(transformed)
        pc = torch.cat(pcs, dim=1)
        n = self.num_points if num_points is None else num_points
        if pc.shape[1] > n:
            idx = torch.randperm(pc.shape[1], device=pc.device)[:n]
            pc = pc[:, idx, :]
        return pc


class Nova5DRPBridge:
    def __init__(self, device: torch.device, model_id: str, compile_model: bool, robot_pcd_points: int = 1024):
        self.device = device
        self.sampler = Nova5Sampler(
            device=device,
            urdf_path=NOVA5_URDF,
            mesh_root=NOVA5_MESH_ROOT,
            num_points=robot_pcd_points,
        )
        self.model = IMPACT.from_pretrained(model_id)
        self.model = self.model.to(device).float().eval()
        if compile_model:
            self.model = torch.compile(self.model)

    @torch.inference_mode()
    def next_action(
        self,
        current_q_rad_7: np.ndarray,
        goal_q_rad_7: np.ndarray,
        scene_pcd: np.ndarray,
    ) -> np.ndarray:
        current_q = torch.as_tensor(current_q_rad_7, device=self.device, dtype=torch.float32).unsqueeze(0)
        goal_q = torch.as_tensor(goal_q_rad_7, device=self.device, dtype=torch.float32).unsqueeze(0)
        scene = torch.as_tensor(scene_pcd, device=self.device, dtype=torch.float32)
        if scene.ndim == 2:
            scene = scene.unsqueeze(0)
        if scene.shape[1] == 0:
            scene = torch.zeros((1, 2048, 3), device=self.device, dtype=torch.float32)
        robot_pcd = self.sampler.sample(current_q_rad_7[:6], num_points=1024)
        obs = {
            "current_angles": current_q,
            "goal_angles": goal_q,
            "scene_pcd": scene,
            "robot_pcd": robot_pcd,
        }
        action = self.model.get_action(obs)
        action = action.detach().cpu().numpy()
        if action.ndim == 3:
            return action[0, 0]
        if action.ndim == 2:
            return action[0]
        return action


class Nova5X11Visualizer:
    def __init__(self, window_name: str = "Nova5 DRP Preview"):
        self.window_name = window_name
        self.enabled = bool(os.environ.get("DISPLAY"))
        if not self.enabled:
            print("[VIS] DISPLAY is empty, X11 window will not open.")
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1440, 720)

    def show(
        self,
        color_img: Optional[np.ndarray],
        depth_img: Optional[np.ndarray],
        robot_pcd: np.ndarray,
        scene_pcd: np.ndarray,
        current_deg: np.ndarray,
        goal_deg: np.ndarray,
        next_deg: np.ndarray,
        scene_n: int,
        execute: bool,
        camera_on: bool,
        model_id: str,
    ):
        if not self.enabled:
            return
        pc_view = _make_pointcloud_view(robot_pcd, scene_pcd)
        status = _make_status_panel(
            current_deg=current_deg,
            goal_deg=goal_deg,
            next_deg=next_deg,
            scene_n=scene_n,
            execute=execute,
            camera_on=camera_on,
            model_id=model_id,
        )
        preview = _stack_preview(color_img, depth_img, pc_view, status)
        cv2.imshow(self.window_name, preview)
        cv2.waitKey(1)

    def wait(self):
        if not self.enabled:
            return
        cv2.waitKey(0)

    def close(self):
        if not self.enabled:
            return
        cv2.destroyWindow(self.window_name)


def _parse_floats(values: Optional[list[str]], expected: int) -> Optional[np.ndarray]:
    if values is None:
        return None
    if len(values) != expected:
        raise ValueError(f"expected {expected} values")
    return np.asarray([float(v) for v in values], dtype=np.float32)


def _parse_goal_pose(values: Optional[list[str]], current_pose: Iterable[float]) -> Optional[np.ndarray]:
    if values is None:
        return None
    if len(values) not in (3, 6):
        raise ValueError("expected 3 or 6 values for goal pose")
    pose = np.asarray([float(v) for v in values], dtype=np.float32)
    if len(values) == 3:
        cur = np.asarray(list(current_pose), dtype=np.float32)
        pose = np.concatenate([pose, cur[3:6]], axis=0)
    return pose


def _load_goal_layer_pose(layer_id: Optional[str]) -> Optional[np.ndarray]:
    if layer_id is None:
        return None
    data = json.loads(SHELF_LAYERS_JSON.read_text(encoding="utf-8"))
    layer = data.get("layers", {}).get(str(layer_id))
    if layer is None:
        raise ValueError(f"unknown shelf layer: {layer_id}")
    pose = np.asarray(layer["pose"], dtype=np.float32)
    if pose.shape != (6,):
        raise ValueError(f"invalid pose for shelf layer {layer_id}: {pose}")
    print(f"[DRP] goal layer: {layer_id} ({layer.get('name', '')})")
    return pose


def _prompt_goal_mode() -> str:
    print("\n[DRP] 目标切换:")
    print("  1 = 第1层")
    print("  2 = 第2层")
    print("  j = 自定义关节角")
    print("  p = 自定义空间点")
    print("  q = 退出")
    return input("  > ").strip().lower()


def _load_scene_pcd(
    depth_img: Optional[np.ndarray],
    depth_scale: float,
    flange_pose_mm: Iterable[float],
    hand_eye: HandEyeTransformer,
    stride: int,
    max_points: int,
) -> np.ndarray:
    if depth_img is None:
        return np.zeros((0, 3), dtype=np.float32)
    return _pixel_to_robot_points(
        depth_img=depth_img,
        depth_scale=depth_scale,
        flange_pose_mm=flange_pose_mm,
        stride=stride,
        max_points=max_points,
        hand_eye=hand_eye,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-id", default="jimyoung6709/DRP", help="HF model id or local model repo")
    parser.add_argument("--compile", action="store_true", help="enable torch.compile (slower warmup)")
    parser.add_argument("--no-camera", action="store_true", help="skip depth capture and use empty scene point cloud")
    parser.add_argument("--device-id", type=int, default=0, help="camera device id")
    parser.add_argument("--depth-stride", type=int, default=12, help="pixel sampling stride for depth point cloud")
    parser.add_argument("--scene-max-points", type=int, default=2048, help="max scene points passed to DRP")
    parser.add_argument("--vis", action="store_true", help="enable X11/OpenCV visualization")
    parser.add_argument("--execute", action="store_true", help="send one clipped MovJ step to Nova5")
    parser.add_argument("--max-step-deg", type=float, default=3.0, help="limit each joint step when executing")
    parser.add_argument("--goal-tol-deg", type=float, default=2.0, help="stop executing when goal error is below this threshold")
    parser.add_argument("--max-exec-iters", type=int, default=12, help="max replan/move iterations per selected goal")
    parser.add_argument("--goal-joints-deg", nargs=6, help="explicit 6 joint goal in degrees")
    parser.add_argument("--goal-pose-mm", nargs="+", help="explicit goal pose in mm: x y z [rx ry rz]")
    parser.add_argument("--goal-layer", choices=("1", "2"), help="use shelf layer 1 or 2 pose as goal")
    parser.add_argument("--once", action="store_true", help="run one inference and exit")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[DRP] device={device}")

    arm = DobotArm()
    camera = None
    bridge = None
    vis = None
    hand_eye = HandEyeTransformer()

    try:
        arm.connect()
        current_deg = np.asarray(arm.get_joints(), dtype=np.float32)
        current_rad_6 = np.deg2rad(current_deg)
        current_rad_7 = _pad_nova5_to_7(current_rad_6)
        color_img = None
        depth_img = None
        if not args.no_camera:
            try:
                camera = RealSenseCamera(width=640, height=480, fps=30, device_id=args.device_id)
                flange_pose = arm.get_pose()
                hand_eye.set_flange(flange_pose)
                color_img, depth_img = camera.read()
                depth_scale = getattr(camera, "_depth_scale", None) or 0.001
                scene_pcd = _load_scene_pcd(
                    depth_img=depth_img,
                    depth_scale=depth_scale,
                    flange_pose_mm=flange_pose,
                    hand_eye=hand_eye,
                    stride=args.depth_stride,
                    max_points=args.scene_max_points,
                )
            except Exception as e:
                print(f"[相机] 初始化失败，继续无相机模式: {e}")
                camera = None
                scene_pcd = np.zeros((0, 3), dtype=np.float32)
        else:
            scene_pcd = np.zeros((0, 3), dtype=np.float32)

        bridge = Nova5DRPBridge(
            device=device,
            model_id=args.model_id,
            compile_model=args.compile,
            robot_pcd_points=1024,
        )
        joint_lower_deg = np.rad2deg(np.array([lim[0] for lim in bridge.sampler.joint_limits], dtype=np.float32))
        joint_upper_deg = np.rad2deg(np.array([lim[1] for lim in bridge.sampler.joint_limits], dtype=np.float32))

        if args.vis:
            vis = Nova5X11Visualizer()
        if args.goal_layer and args.goal_pose_mm:
            raise ValueError("--goal-layer and --goal-pose-mm cannot be used together")
        if args.goal_layer and args.goal_joints_deg:
            raise ValueError("--goal-layer and --goal-joints-deg cannot be used together")

        while True:
            current_deg = np.asarray(arm.get_joints(), dtype=np.float32)
            current_rad_6 = np.deg2rad(current_deg)
            current_rad_7 = _pad_nova5_to_7(current_rad_6)
            current_pose = arm.get_pose()

            goal_deg_6 = _parse_floats(args.goal_joints_deg, 6)
            if args.goal_layer is not None:
                goal_pose_mm = _load_goal_layer_pose(args.goal_layer)
            elif args.goal_pose_mm:
                goal_pose_mm = _parse_goal_pose(args.goal_pose_mm, current_pose)
            else:
                choice = _prompt_goal_mode()
                if choice == "q":
                    break
                if choice in ("1", "2"):
                    goal_pose_mm = _load_goal_layer_pose(choice)
                elif choice == "j":
                    raw = input("  输入 6 个关节角(deg, 空格分隔): ").strip().split()
                    goal_deg_6 = _parse_floats(raw, 6)
                    goal_pose_mm = None
                elif choice == "p":
                    raw = input("  输入目标点 x y z [rx ry rz] (mm, 空格分隔): ").strip().split()
                    goal_pose_mm = _parse_goal_pose(raw, current_pose)
                else:
                    print("[DRP] 未识别输入")
                    continue

            if goal_deg_6 is None:
                if goal_pose_mm is None:
                    goal_deg_6 = NOVA5_HOME_DEG.copy()
                    goal_rad_6 = NOVA5_HOME_RAD.copy()
                    print("[DRP] using default home joints as goal")
                else:
                    ik_joints = arm.inverse_kin_near(goal_pose_mm.tolist())
                    if ik_joints is None:
                        print(f"[DRP] 无法为目标点求逆解: {goal_pose_mm.tolist()}")
                        continue
                    goal_deg_6 = np.asarray(ik_joints, dtype=np.float32)
                    goal_rad_6 = np.deg2rad(goal_deg_6)
                    print("[DRP] goal pose(mm):", np.round(goal_pose_mm, 3).tolist())
                    print("[DRP] goal joints(deg) from IK:", np.round(goal_deg_6, 3).tolist())
            else:
                goal_rad_6 = np.deg2rad(goal_deg_6)

            goal_rad_7 = _pad_nova5_to_7(goal_rad_6)
            scene_pcd_vis = _pad_scene_points(scene_pcd, 2048)
            print(f"[DRP] scene_pcd={scene_pcd.shape[0]} points")

            if not args.execute:
                robot_pcd = bridge.sampler.sample(current_rad_7[:6], num_points=1024)[0].detach().cpu().numpy()
                next_q_rad_7 = bridge.next_action(current_rad_7, goal_rad_7, scene_pcd_vis)
                next_q_deg_7 = np.rad2deg(next_q_rad_7[:6])
                next_q_deg_7 = _clip_to_limits(next_q_deg_7, joint_lower_deg, joint_upper_deg)
                print("\n[DRP] current(deg):", np.round(current_deg, 3).tolist())
                print("[DRP] goal(deg):   ", np.round(goal_deg_6, 3).tolist())
                print("[DRP] next(deg):   ", np.round(next_q_deg_7, 3).tolist())
                print("[DRP] dry-run only, no robot motion sent")
                if vis is not None:
                    vis.show(
                        color_img=color_img,
                        depth_img=depth_img,
                        robot_pcd=robot_pcd,
                        scene_pcd=scene_pcd_vis,
                        current_deg=current_deg,
                        goal_deg=goal_deg_6,
                        next_deg=next_q_deg_7,
                        scene_n=scene_pcd.shape[0],
                        execute=False,
                        camera_on=not args.no_camera,
                        model_id=args.model_id,
                    )
            else:
                for exec_idx in range(max(1, args.max_exec_iters)):
                    current_deg = np.asarray(arm.get_joints(), dtype=np.float32)
                    current_rad_7 = _pad_nova5_to_7(np.deg2rad(current_deg))
                    goal_err = goal_deg_6 - current_deg
                    if np.max(np.abs(goal_err)) <= args.goal_tol_deg:
                        print(f"[DRP] goal reached within {args.goal_tol_deg:.1f} deg")
                        break

                    robot_pcd = bridge.sampler.sample(current_rad_7[:6], num_points=1024)[0].detach().cpu().numpy()
                    next_q_rad_7 = bridge.next_action(current_rad_7, goal_rad_7, scene_pcd_vis)
                    next_q_deg_7 = np.rad2deg(next_q_rad_7[:6])
                    next_q_deg_7 = _clip_to_limits(next_q_deg_7, joint_lower_deg, joint_upper_deg)
                    step = np.clip(next_q_deg_7 - current_deg, -args.max_step_deg, args.max_step_deg)
                    target_deg = current_deg + step
                    print("\n[DRP] current(deg):", np.round(current_deg, 3).tolist())
                    print("[DRP] goal(deg):   ", np.round(goal_deg_6, 3).tolist())
                    print("[DRP] next(deg):   ", np.round(next_q_deg_7, 3).tolist())
                    print("[DRP] execute step(deg):", np.round(step, 3).tolist())
                    print("[DRP] target(deg):      ", np.round(target_deg, 3).tolist())
                    arm.movj(target_deg.tolist(), speed=20, cp=0)
                    if vis is not None:
                        vis.show(
                            color_img=color_img,
                            depth_img=depth_img,
                            robot_pcd=robot_pcd,
                            scene_pcd=scene_pcd_vis,
                            current_deg=current_deg,
                            goal_deg=goal_deg_6,
                            next_deg=next_q_deg_7,
                            scene_n=scene_pcd.shape[0],
                            execute=True,
                            camera_on=not args.no_camera,
                            model_id=args.model_id,
                        )
                    time.sleep(MOTION_SETTLE_S)

            if args.vis:
                cv2.waitKey(1)

    finally:
        try:
            if camera is not None:
                camera.release()
        except Exception:
            pass
        try:
            arm.close()
        except Exception:
            pass
        try:
            if bridge is not None:
                bridge.sampler.close()
        except Exception:
            pass
        try:
            if vis is not None:
                vis.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
