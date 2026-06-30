#!/usr/bin/env python3
"""Experimental Nova5 + DRP bridge test.

This is a compatibility harness, not a trained Nova5 policy.
The DRP checkpoint in this repo was trained for Franka Panda, so this script
pads Nova5's 6-DoF joints to 7 DoF for a first connection test.
"""

from __future__ import annotations

import argparse
import threading
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import viser
import torch
import trimesh
from viser.extras import ViserUrdf

REPO_ROOT = Path(__file__).resolve().parents[1]
DRP_ROOT = REPO_ROOT / "drp"
ROB0FIN_ROOT = DRP_ROOT / "robofin"
VISION_ROOT = REPO_ROOT / "识别"
DIPAN_ROOT = REPO_ROOT / "dipan"

for path in (REPO_ROOT, DRP_ROOT, ROB0FIN_ROOT, VISION_ROOT, DIPAN_ROOT):
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
        return action[0].detach().cpu().numpy()


class Nova5ViserBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._vis_urdf = _rewrite_urdf_mesh_paths(NOVA5_URDF)
        self.server = viser.ViserServer()
        self.server.scene.add_frame("/WorldAxes", show_axes=True, axes_length=0.15, axes_radius=0.01, visible=True)
        self.server.scene.add_frame("/robot", show_axes=False)
        self.server.scene.add_grid("/grid", width=8, height=8, position=(0.0, 0.0, 0.0), shadow_opacity=0.1)
        self._urdf_vis = ViserUrdf(
            self.server,
            urdf_or_path=self._vis_urdf,
            root_node_name="/robot",
            load_meshes=True,
            load_collision_meshes=False,
        )
        self.joint_names = self._urdf_vis.get_actuated_joint_names()
        self._name_to_idx = {name: i for i, name in enumerate(self.joint_names)}
        self._q = np.zeros(len(self.joint_names), dtype=float)
        self._robot_pcd_handle = self.server.scene.add_point_cloud(
            name="/robot_pcd",
            points=np.zeros((0, 3), dtype=np.float32),
            colors=(0, 80, 255),
            point_size=0.004,
            visible=True,
        )
        self._scene_pcd_handle = self.server.scene.add_point_cloud(
            name="/scene_pcd",
            points=np.zeros((0, 3), dtype=np.float32),
            colors=(60, 170, 80),
            point_size=0.005,
            visible=True,
        )
        self._set_nova5_neutral()

    def _set_nova5_neutral(self):
        q = np.deg2rad(NOVA5_HOME_DEG)
        self.set_joint_positions(q)

    def set_joint_positions(self, q_rad_6: np.ndarray):
        q = np.asarray(q_rad_6, dtype=float).reshape(6)
        cfg = self._q.copy()
        for i, value in enumerate(q, start=1):
            idx = self._name_to_idx.get(f"joint{i}")
            if idx is None:
                continue
            cfg[idx] = value
        self._q[:] = cfg
        self._urdf_vis.update_cfg(self._q)

    def update(self, current_q_deg: np.ndarray, robot_pcd: np.ndarray, scene_pcd: np.ndarray):
        with self._lock:
            self.set_joint_positions(np.deg2rad(current_q_deg[:6]))
            self._robot_pcd_handle.points = np.asarray(robot_pcd, dtype=np.float32)
            self._scene_pcd_handle.points = np.asarray(scene_pcd, dtype=np.float32)

    def close(self):
        try:
            self.server.stop()
        except Exception:
            pass


def _parse_floats(values: Optional[list[str]], expected: int) -> Optional[np.ndarray]:
    if values is None:
        return None
    if len(values) != expected:
        raise ValueError(f"expected {expected} values")
    return np.asarray([float(v) for v in values], dtype=np.float32)


def _load_scene_pcd(
    camera: Optional[RealSenseCamera],
    flange_pose_mm: Iterable[float],
    hand_eye: HandEyeTransformer,
    stride: int,
    max_points: int,
) -> np.ndarray:
    if camera is None:
        return np.zeros((0, 3), dtype=np.float32)
    color_img, depth_img = camera.read()
    if depth_img is None:
        return np.zeros((0, 3), dtype=np.float32)
    depth_scale = getattr(camera, "_depth_scale", None) or 0.001
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
    parser.add_argument("--vis", action="store_true", help="enable viser visualization")
    parser.add_argument("--execute", action="store_true", help="send one clipped MovJ step to Nova5")
    parser.add_argument("--max-step-deg", type=float, default=3.0, help="limit each joint step when executing")
    parser.add_argument("--goal-joints-deg", nargs=6, help="explicit 6 joint goal in degrees")
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
        goal_deg_6 = _parse_floats(args.goal_joints_deg, 6)
        if goal_deg_6 is None:
            goal_rad_6 = NOVA5_HOME_RAD.copy()
            goal_deg_6 = NOVA5_HOME_DEG.copy()
        else:
            goal_rad_6 = np.deg2rad(goal_deg_6)
        goal_rad_7 = _pad_nova5_to_7(goal_rad_6)

        if not args.no_camera:
            camera = RealSenseCamera(width=640, height=480, fps=30, device_id=args.device_id)
            flange_pose = arm.get_pose()
            hand_eye.set_flange(flange_pose)
            scene_pcd = _load_scene_pcd(
                camera=camera,
                flange_pose_mm=flange_pose,
                hand_eye=hand_eye,
                stride=args.depth_stride,
                max_points=args.scene_max_points,
            )
        else:
            scene_pcd = np.zeros((0, 3), dtype=np.float32)

        bridge = Nova5DRPBridge(
            device=device,
            model_id=args.model_id,
            compile_model=args.compile,
            robot_pcd_points=1024,
        )

        if args.vis:
            vis = Nova5ViserBridge()

        robot_pcd = bridge.sampler.sample(current_rad_7[:6], num_points=1024)[0].detach().cpu().numpy()
        scene_pcd_vis = _pad_scene_points(scene_pcd, 2048)
        next_q_rad_7 = bridge.next_action(current_rad_7, goal_rad_7, scene_pcd_vis)
        next_q_deg_7 = np.rad2deg(next_q_rad_7[:6])
        next_q_deg_7 = _clip_to_limits(
            next_q_deg_7,
            np.array([lim[0] for lim in bridge.sampler.joint_limits], dtype=np.float32),
            np.array([lim[1] for lim in bridge.sampler.joint_limits], dtype=np.float32),
        )

        print("\n[DRP] current(deg):", np.round(current_deg, 3).tolist())
        print("[DRP] goal(deg):   ", np.round(goal_deg_6, 3).tolist())
        print("[DRP] next(deg):   ", np.round(next_q_deg_7, 3).tolist())
        print(f"[DRP] scene_pcd={scene_pcd.shape[0]} points")

        if vis is not None:
            vis.update(current_deg, robot_pcd, scene_pcd_vis)

        if args.execute:
            step = np.clip(next_q_deg_7 - current_deg, -args.max_step_deg, args.max_step_deg)
            target_deg = current_deg + step
            print("[DRP] execute step(deg):", np.round(step, 3).tolist())
            print("[DRP] target(deg):      ", np.round(target_deg, 3).tolist())
            arm.movj(target_deg.tolist(), speed=20, cp=0)
        else:
            print("[DRP] dry-run only, no robot motion sent")

        if args.vis:
            input("[DRP] 已打开 viser，回车退出...")

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
