import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from depth_collision_guard import check_hand_path, depth_to_robot_obstacles, hand_path_hit_mask
from dobot_arm import DobotArm
from self_collision_guard import Nova5SelfCollisionGuard
from tool_geometry import camera_box_bounds, camera_capsule_strips, hand_capsule_parts


class ToolCollisionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.box_min, self.box_max = camera_box_bounds()

    def check_depth_path(self, points, rotation=None, include_camera=True):
        return check_hand_path(
            np.asarray(points, dtype=np.float32),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3) if rotation is None else rotation,
            camera_box_min_local_m=self.box_min if include_camera else None,
            camera_box_max_local_m=self.box_max if include_camera else None,
        )

    def test_camera_point_is_not_covered_by_hand_only(self):
        point = [[-0.10, 0.0, -0.0125]]
        self.assertTrue(self.check_depth_path(point, include_camera=False).ok)
        self.assertFalse(self.check_depth_path(point, include_camera=True).ok)

        hit_mask, _ = hand_path_hit_mask(
            np.asarray(point, dtype=np.float32),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3),
            camera_box_min_local_m=self.box_min,
            camera_box_max_local_m=self.box_max,
        )
        self.assertEqual(hit_mask.tolist(), [True])

    def test_depth_guard_counts_unique_points_across_path_samples(self):
        one_noise_point = np.asarray([[0.005, 0.0, 0.10]], dtype=np.float32)
        result = check_hand_path(
            one_noise_point,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.08],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=8,
            min_hit_points=2,
        )
        self.assertTrue(result.ok)
        self.assertEqual(result.hit_count, 1)

        two_obstacle_points = np.asarray(
            [[0.005, 0.0, 0.10], [-0.005, 0.0, 0.12]],
            dtype=np.float32,
        )
        result = check_hand_path(
            two_obstacle_points,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.08],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=8,
            min_hit_points=2,
        )
        self.assertFalse(result.ok)
        self.assertEqual(result.hit_count, 2)

    def test_depth_guard_uses_separate_camera_support_threshold(self):
        camera_points = np.asarray(
            [[-0.10, -0.02 + index * 0.008, -0.0125] for index in range(6)],
            dtype=np.float32,
        )
        mixed_points = np.concatenate(
            [camera_points, np.asarray([[0.005, 0.0, 0.10]], dtype=np.float32)],
            axis=0,
        )
        result = check_hand_path(
            mixed_points,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3),
            camera_box_min_local_m=self.box_min,
            camera_box_max_local_m=self.box_max,
            min_hit_points=2,
            camera_min_hit_points=12,
        )
        self.assertTrue(result.ok)
        self.assertEqual(result.hand_hit_count, 1)
        self.assertEqual(result.camera_hit_count, 6)

    def test_depth_obstacle_filter_rejects_isolated_speckle(self):
        depth = np.zeros((24, 24), dtype=np.uint16)
        depth[8, 8] = 500
        camera = SimpleNamespace(_depth_scale=0.001)
        transformer = SimpleNamespace(
            cx=0.0,
            cy=0.0,
            fx=100.0,
            fy=100.0,
            T_flange_to_cam=np.eye(4, dtype=np.float32),
            _rpy_to_rot=lambda *_args: np.eye(3, dtype=np.float32),
        )
        points = depth_to_robot_obstacles(
            depth,
            camera,
            transformer,
            [0, 0, 0, 0, 0, 0],
            stride=8,
            noise_filter_radius_px=2,
            noise_filter_min_support=4,
        )
        self.assertEqual(len(points), 0)

        depth[7:10, 7:10] = 500
        points = depth_to_robot_obstacles(
            depth,
            camera,
            transformer,
            [0, 0, 0, 0, 0, 0],
            stride=8,
            noise_filter_radius_px=2,
            noise_filter_min_support=4,
        )
        self.assertEqual(len(points), 1)

    def test_camera_box_is_centered_at_palm_root(self):
        self.assertAlmostEqual(self.box_min[0], -0.15)
        self.assertAlmostEqual(self.box_max[0], -0.09)
        self.assertAlmostEqual(self.box_min[1], -0.0725)
        self.assertAlmostEqual(self.box_max[1], 0.0725)
        self.assertAlmostEqual(self.box_min[2], -0.025)
        self.assertAlmostEqual(self.box_max[2], 0.0)
        self.assertFalse(self.check_depth_path([[-0.12, 0.06, -0.0125]]).ok)
        self.assertTrue(self.check_depth_path([[-0.03, 0.0, -0.0125]]).ok)

    def test_camera_box_follows_flange_rotation(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        local_point = np.array([-0.10, 0.0, -0.0125], dtype=np.float32)
        world_point = rotation @ local_point
        self.assertFalse(self.check_depth_path([world_point], rotation=rotation).ok)
        self.assertTrue(self.check_depth_path([[-0.10, 0.09, -0.0125]]).ok)

    def test_self_collision_camera_cover_and_safe_poses(self):
        strips = camera_capsule_strips()
        self.assertEqual(len(strips), 4)
        self.assertAlmostEqual(strips[0][0][0], -0.15)
        self.assertAlmostEqual(strips[-1][1][0], -0.09)

        guard = Nova5SelfCollisionGuard()
        camera_specs = [item for item in guard.capsules if item.name == "tool_camera"]
        self.assertEqual(len(camera_specs), 4)
        self.assertTrue(guard.check([-44.1, -20.2, 134.6, -112.7, -46.0, 90.6]).ok)
        self.assertTrue(guard.check([0.0] * 6).ok)
        self.assertTrue(guard.check([-106.5, -2.2, 98.1, -178.1, -88.2, 93.3]).ok)

    def test_self_collision_hand_envelope_and_tool_pairs(self):
        parts = hand_capsule_parts()
        self.assertEqual(len(parts["tool_hand_palm"]), 12)
        self.assertEqual(len(parts["tool_hand_fingers"]), 15)
        self.assertAlmostEqual(parts["tool_hand_palm"][0][0][2], 0.05)
        self.assertAlmostEqual(parts["tool_hand_fingers"][-1][1][2], 0.25)

        guard = Nova5SelfCollisionGuard()
        palm_specs = [item for item in guard.capsules if item.name == "tool_hand_palm"]
        finger_specs = [item for item in guard.capsules if item.name == "tool_hand_fingers"]
        self.assertEqual(len(palm_specs), 12)
        self.assertEqual(len(finger_specs), 15)

        checked_names = [
            (guard.capsules[i].name, guard.capsules[j].name)
            for i, j in guard._check_pairs
        ]
        self.assertTrue(any({a, b} == {"tool_hand_palm", "Link4"} for a, b in checked_names))
        self.assertTrue(any({a, b} == {"tool_hand_fingers", "Link3"} for a, b in checked_names))
        self.assertTrue(any({a, b} == {"tool_camera", "Link4"} for a, b in checked_names))
        self.assertFalse(any(
            a.startswith("tool_") and b in ("Link5", "Link6")
            or b.startswith("tool_") and a in ("Link5", "Link6")
            for a, b in checked_names
        ))

    def test_collision_gate_fails_closed_when_ik_or_check_fails(self):
        arm = DobotArm.__new__(DobotArm)
        arm.self_collision_guard = Mock()
        arm.get_joints = Mock(return_value=[0.0] * 6)
        arm.inverse_kin_near = Mock(return_value=None)
        with (
            patch("dobot_arm.SELF_COLLISION_ENABLE", True),
            patch("dobot_arm.SELF_COLLISION_REQUIRE_IK", True),
            patch("dobot_arm.SELF_COLLISION_FAIL_CLOSED", True),
        ):
            self.assertFalse(arm._check_self_collision_pose([0.0] * 6, "test-pose"))
            arm.self_collision_guard.check_path.side_effect = RuntimeError("guard failed")
            self.assertFalse(arm._check_self_collision_path([0.0] * 6, [1.0] * 6, "test-path"))

    def test_collision_gate_rejects_missing_required_model(self):
        arm = DobotArm.__new__(DobotArm)
        arm.self_collision_guard = None
        with (
            patch("dobot_arm.SELF_COLLISION_ENABLE", True),
            patch("dobot_arm.SELF_COLLISION_FAIL_CLOSED", True),
        ):
            self.assertFalse(arm._check_self_collision_path([0.0] * 6, [1.0] * 6, "test-path"))
            self.assertFalse(arm._check_self_collision_joints([1.0] * 6, "test-joints"))
            self.assertFalse(arm._check_self_collision_pose([0.0] * 6, "test-pose"))


if __name__ == "__main__":
    unittest.main()
