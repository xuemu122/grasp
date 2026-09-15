#!/usr/bin/env python3
"""Lightweight Nova5 self-collision guard.

This module intentionally avoids ROS/MoveIt runtime dependencies. It uses the
Nova5 URDF for forward kinematics, the MoveIt SRDF disabled-collision pairs,
and conservative capsule approximations for the robot links.
"""

from __future__ import annotations

import itertools
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from tool_geometry import hand_capsule_parts


_REPO_ROOT = os.environ.get("GRASP_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NOVA5_URDF = os.environ.get(
    "NOVA5_URDF",
    os.path.join(_REPO_ROOT, "dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/nova5_robot.urdf"),
)
NOVA5_SRDF = os.environ.get(
    "NOVA5_SRDF",
    os.path.join(_REPO_ROOT, "dobot_ws/src/DOBOT_6Axis_ROS2_V4/nova5_moveit/config/nova5_robot.srdf"),
)


@dataclass(frozen=True)
class CollisionIssue:
    link1: str
    link2: str
    distance_m: float
    clearance_m: float


@dataclass(frozen=True)
class SelfCollisionResult:
    ok: bool
    min_clearance_m: float
    checked_pairs: int
    issues: Tuple[CollisionIssue, ...] = ()


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    joint_type: str


@dataclass(frozen=True)
class CapsuleSpec:
    link: str
    name: str
    a_local: np.ndarray
    b_local: np.ndarray
    radius_m: float
    tool_group: Optional[str] = None


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1.0 - c
    return np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=np.float64,
    )


def _tf(xyz: Sequence[float], rot: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot
    out[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return out


def _transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    return T[:3, :3] @ p + T[:3, 3]


def _segment_segment_distance(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
    # From Real-Time Collision Detection, Christer Ericson.
    d1 = a1 - a0
    d2 = b1 - b0
    r = a0 - b0
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    eps = 1e-12

    if a <= eps and e <= eps:
        return float(np.linalg.norm(a0 - b0))
    if a <= eps:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = float(np.dot(d1, r))
        if e <= eps:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom != 0.0 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    closest_a = a0 + float(s) * d1
    closest_b = b0 + float(t) * d2
    return float(np.linalg.norm(closest_a - closest_b))


def _trim_segment(a: np.ndarray, b: np.ndarray, start_m: float, end_m: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    vec = b - a
    length = float(np.linalg.norm(vec))
    if length <= 1e-12:
        return a, b
    direction = vec / length
    start = min(max(0.0, start_m), length * 0.45)
    end = min(max(0.0, end_m), length * 0.45)
    return a + start * direction, b - end * direction


class Nova5SelfCollisionGuard:
    def __init__(
        self,
        urdf_path: str = NOVA5_URDF,
        srdf_path: str = NOVA5_SRDF,
        margin_m: float = 0.015,
        camera_margin_m: Optional[float] = None,
    ):
        self.urdf_path = urdf_path
        self.srdf_path = srdf_path
        self.margin_m = float(margin_m)
        self.camera_margin_m = float(
            os.getenv("SELF_COLLISION_CAMERA_MARGIN_M", "0.0")
            if camera_margin_m is None else camera_margin_m
        )
        self.joints = self._load_urdf_joints(urdf_path)
        self.disabled_pairs = self._load_disabled_pairs(srdf_path)
        self.disabled_pairs.add(tuple(sorted(("base_link", "Link2"))))
        self.chain = [f"joint{i}" for i in range(1, 7)]
        self.capsules = self._make_capsules()
        self._check_pairs = self._make_check_pairs()

    @staticmethod
    def _load_urdf_joints(path: str) -> Dict[str, JointSpec]:
        root = ET.parse(path).getroot()
        joints: Dict[str, JointSpec] = {}
        for joint in root.findall("joint"):
            name = joint.attrib.get("name", "")
            parent_el = joint.find("parent")
            child_el = joint.find("child")
            if not name or parent_el is None or child_el is None:
                continue
            origin = joint.find("origin")
            xyz = np.zeros(3, dtype=np.float64)
            rpy = np.zeros(3, dtype=np.float64)
            if origin is not None:
                xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ", dtype=np.float64)
                rpy = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ", dtype=np.float64)
            axis_el = joint.find("axis")
            axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            if axis_el is not None:
                axis = np.fromstring(axis_el.attrib.get("xyz", "0 0 1"), sep=" ", dtype=np.float64)
            joints[name] = JointSpec(
                name=name,
                parent=parent_el.attrib["link"],
                child=child_el.attrib["link"],
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                joint_type=joint.attrib.get("type", "fixed"),
            )
        return joints

    @staticmethod
    def _load_disabled_pairs(path: str) -> set:
        if not os.path.exists(path):
            return set()
        root = ET.parse(path).getroot()
        out = set()
        for item in root.findall("disable_collisions"):
            a = item.attrib.get("link1")
            b = item.attrib.get("link2")
            if a and b:
                out.add(tuple(sorted((a, b))))
        return out

    def _make_capsules(self) -> Tuple[CapsuleSpec, ...]:
        joints = self.joints
        j3 = joints["joint3"].xyz
        j4 = joints["joint4"].xyz
        j5 = joints["joint5"].xyz
        j6 = joints["joint6"].xyz
        l2_a, l2_b = _trim_segment(np.zeros(3), j3, 0.12, 0.03)
        l3_a, l3_b = _trim_segment(np.zeros(3), j4, 0.06, 0.03)
        robot = (
            CapsuleSpec("base_link", "base_link", np.array([0.0, 0.0, 0.02]), np.array([0.0, 0.0, 0.24]), 0.12),
            CapsuleSpec("Link1", "Link1", np.array([0.0, 0.0, -0.05]), np.array([0.0, 0.0, 0.06]), 0.11),
            CapsuleSpec("Link2", "Link2", l2_a, l2_b, 0.075),
            CapsuleSpec("Link3", "Link3", l3_a, l3_b, 0.065),
            CapsuleSpec("Link4", "Link4", np.zeros(3), j5, 0.055),
            CapsuleSpec("Link5", "Link5", np.zeros(3), j6, 0.055),
            CapsuleSpec("Link6", "Link6", np.zeros(3), np.array([0.0, 0.0, 0.05]), 0.055),
        )
        hand = tuple(
            CapsuleSpec(
                "Link6",
                group,
                np.asarray(start, dtype=np.float64),
                np.asarray(end, dtype=np.float64),
                float(radius),
                tool_group=group,
            )
            for group, capsules in hand_capsule_parts().items()
            for start, end, radius in capsules
        )
        return robot + hand

    def _make_check_pairs(self) -> Tuple[Tuple[int, int], ...]:
        pairs = []
        for i, j in itertools.combinations(range(len(self.capsules)), 2):
            a = self.capsules[i]
            b = self.capsules[j]
            if a.tool_group and b.tool_group:
                continue
            if a.tool_group or b.tool_group:
                robot_part = b if a.tool_group else a
                # The tool is rigidly mounted through Link6 next to Link5. Their
                # coarse envelopes overlap at the flange by construction.
                if robot_part.link in ("Link5", "Link6"):
                    continue
                pairs.append((i, j))
                continue
            if a.link == b.link:
                continue
            if tuple(sorted((a.link, b.link))) in self.disabled_pairs:
                continue
            pairs.append((i, j))
        return tuple(pairs)

    def fk(self, joints_deg: Sequence[float]) -> Dict[str, np.ndarray]:
        q = [math.radians(float(v)) for v in joints_deg]
        frames: Dict[str, np.ndarray] = {"dummy_link": np.eye(4, dtype=np.float64)}
        dummy = self.joints.get("dummy_joint")
        if dummy is not None:
            frames[dummy.child] = frames[dummy.parent] @ _tf(dummy.xyz, _rpy_to_rot(*dummy.rpy))
        else:
            frames["base_link"] = np.eye(4, dtype=np.float64)

        for idx, joint_name in enumerate(self.chain):
            joint = self.joints[joint_name]
            parent_T = frames[joint.parent]
            origin_T = _tf(joint.xyz, _rpy_to_rot(*joint.rpy))
            motion_T = np.eye(4, dtype=np.float64)
            if joint.joint_type in ("revolute", "continuous"):
                motion_T[:3, :3] = _axis_angle(joint.axis, q[idx])
            frames[joint.child] = parent_T @ origin_T @ motion_T
        return frames

    def check(self, joints_deg: Sequence[float]) -> SelfCollisionResult:
        if len(joints_deg) < 6:
            return SelfCollisionResult(False, float("-inf"), 0, (CollisionIssue("?", "?", 0.0, float("-inf")),))

        frames = self.fk(joints_deg)
        world_capsules = []
        for spec in self.capsules:
            T = frames[spec.link]
            world_capsules.append((
                spec,
                _transform_point(T, spec.a_local),
                _transform_point(T, spec.b_local),
            ))

        issues: List[CollisionIssue] = []
        min_clearance = float("inf")
        for i, j in self._check_pairs:
            spec_a, a0, a1 = world_capsules[i]
            spec_b, b0, b1 = world_capsules[j]
            dist = _segment_segment_distance(a0, a1, b0, b1)
            pair_margin = (
                self.camera_margin_m
                if "tool_camera" in (spec_a.name, spec_b.name)
                else self.margin_m
            )
            clearance = dist - spec_a.radius_m - spec_b.radius_m - pair_margin
            min_clearance = min(min_clearance, clearance)
            if clearance < 0.0:
                issues.append(CollisionIssue(spec_a.name, spec_b.name, dist, clearance))

        issues.sort(key=lambda item: item.clearance_m)
        return SelfCollisionResult(
            ok=not issues,
            min_clearance_m=min_clearance,
            checked_pairs=len(self._check_pairs),
            issues=tuple(issues),
        )

    def check_path(self, start_deg: Sequence[float], end_deg: Sequence[float], samples: int = 8) -> SelfCollisionResult:
        start = np.asarray(start_deg[:6], dtype=np.float64)
        end = np.asarray(end_deg[:6], dtype=np.float64)
        worst = SelfCollisionResult(True, float("inf"), 0, ())
        for alpha in np.linspace(0.0, 1.0, max(2, int(samples))):
            q = (1.0 - alpha) * start + alpha * end
            result = self.check(q)
            if result.min_clearance_m < worst.min_clearance_m:
                worst = result
            if not result.ok:
                return result
        return worst


def describe_self_collision(result: SelfCollisionResult, max_items: int = 3) -> str:
    if result.ok:
        clearance_mm = result.min_clearance_m * 1000.0 if np.isfinite(result.min_clearance_m) else float("inf")
        return f"最近余量 {clearance_mm:.0f}mm, 检查 {result.checked_pairs} 对"
    parts = []
    for issue in result.issues[:max_items]:
        parts.append(
            f"{issue.link1}-{issue.link2}: 距离 {issue.distance_m*1000:.0f}mm, "
            f"余量 {issue.clearance_m*1000:.0f}mm"
        )
    return "; ".join(parts)
