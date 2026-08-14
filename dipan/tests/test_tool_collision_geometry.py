import importlib
import os
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import tool_geometry
import shelf_grasp

from depth_collision_guard import check_hand_path, depth_to_robot_obstacles, hand_path_hit_mask
from dobot_arm import DobotArm
from self_collision_guard import Nova5SelfCollisionGuard
from tool_geometry import (
    ADAPTER_LENGTH_Z_M,
    adapter_camera_mount_capsules,
    camera_box_bounds,
    camera_capsule_strips,
    hand_box_bounds,
    hand_capsule_parts,
)


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

    def test_arm_close_disables_robot_and_is_idempotent(self):
        arm = DobotArm.__new__(DobotArm)
        arm.sock = Mock()
        arm.send = Mock(return_value="0,{},DisableRobot();")

        arm.close()
        arm.close()

        arm.send.assert_called_once_with("DisableRobot()")
        self.assertIsNone(arm.sock)

    def test_startup_exit_guard_disables_arm(self):
        arm = Mock()
        shelf_grasp._disable_arm_on_process_exit(arm)
        arm.close.assert_called_once_with()

    def test_movel_failure_saves_path_viz_after_depth_check_passes(self):
        arm = Mock()
        arm.get_pose.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        arm.movel.return_value = False
        obstacle_points = np.asarray([[0.2, 0.2, 0.2]], dtype=np.float32)
        obstacle_pixels = np.asarray([[10, 10]], dtype=np.int32)

        with (
            patch.object(shelf_grasp, "_depth_guard_path_ok", return_value=True),
            patch.object(shelf_grasp, "_save_depth_guard_viz") as save_viz,
        ):
            result = shelf_grasp._movel_with_depth_guard(
                arm,
                "探入路径",
                obstacle_points,
                (100.0, 0.0, 0.0),
                np.asarray([0.0, 0.0, 1.0], dtype=np.float32),
                allow_bypass=False,
                viz_image=np.zeros((24, 24, 3), dtype=np.uint8),
                obstacle_pixels=obstacle_pixels,
            )

        self.assertFalse(result)
        save_viz.assert_called_once()
        self.assertEqual(
            save_viz.call_args.kwargs["failure_reason"],
            "MovL rejected after depth check passed",
        )

    def test_camera_point_is_not_covered_by_hand_only(self):
        point = [[-0.10, 0.0, 0.0325]]
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

    def test_depth_guard_covers_offset_dh116_thumb_box(self):
        thumb_point = np.asarray([[0.10, 0.0, 0.08]], dtype=np.float32)
        without_boxes = check_hand_path(
            thumb_point,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.195,
            0.01,
            samples=2,
        )
        with_boxes = check_hand_path(
            thumb_point,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.195,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3),
            hand_boxes_local_m=hand_box_bounds(),
        )
        self.assertTrue(without_boxes.ok)
        self.assertFalse(with_boxes.ok)

    def test_offset_hand_box_requires_its_own_dense_support(self):
        # A few points inside the conservative thumb envelope should not stop
        # a path when the central hand axis is clearly clear.
        points = np.asarray(
            [[0.08, 0.0, 0.08 + index * 0.004] for index in range(6)],
            dtype=np.float32,
        )
        result = check_hand_path(
            points,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.08,
            0.01,
            samples=2,
            hand_boxes_local_m=hand_box_bounds(),
            min_hit_points=4,
            hand_box_min_hit_points=12,
        )
        self.assertTrue(result.ok)
        self.assertEqual(result.hand_axis_hit_count, 0)
        self.assertEqual(result.hand_box_hit_count, 6)

    def test_depth_guard_rotates_dh116_boxes_with_flange(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        rotated_thumb_point = rotation @ np.array([0.10, 0.0, 0.08], dtype=np.float32)
        result = check_hand_path(
            np.asarray([rotated_thumb_point]),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.195,
            0.01,
            samples=2,
            flange_rotation_m=rotation,
            hand_boxes_local_m=hand_box_bounds(),
        )
        self.assertFalse(result.ok)

    def test_depth_guard_uses_separate_camera_support_threshold(self):
        camera_points = np.asarray(
            [[-0.10, -0.02 + index * 0.008, 0.0325] for index in range(6)],
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

    def test_depth_obstacles_exclude_points_behind_target(self):
        depth = np.zeros((24, 24), dtype=np.uint16)
        depth[8, 8] = 400
        depth[8, 16] = 520
        camera = SimpleNamespace(_depth_scale=0.001)
        transformer = SimpleNamespace(
            cx=0.0,
            cy=0.0,
            fx=100.0,
            fy=100.0,
            T_flange_to_cam=np.eye(4, dtype=np.float32),
            _rpy_to_rot=lambda *_args: np.eye(3, dtype=np.float32),
        )
        points, pixels = depth_to_robot_obstacles(
            depth,
            camera,
            transformer,
            [0, 0, 0, 0, 0, 0],
            stride=8,
            target_depth_m=0.40,
            target_depth_margin_m=0.06,
            return_pixels=True,
        )
        self.assertEqual(len(points), 1)
        self.assertEqual(pixels.tolist(), [[8, 8]])

    def test_depth_obstacles_exclude_shelf_board_rows(self):
        depth = np.zeros((24, 24), dtype=np.uint16)
        depth[0, 8] = 400
        depth[8, 8] = 400
        depth[16, 8] = 400
        camera = SimpleNamespace(_depth_scale=0.001)
        transformer = SimpleNamespace(
            cx=0.0,
            cy=0.0,
            fx=100.0,
            fy=100.0,
            T_flange_to_cam=np.eye(4, dtype=np.float32),
            _rpy_to_rot=lambda *_args: np.eye(3, dtype=np.float32),
        )
        _, pixels = depth_to_robot_obstacles(
            depth,
            camera,
            transformer,
            [0, 0, 0, 0, 0, 0],
            stride=8,
            include_row_min_px=4,
            include_row_max_px=12,
            return_pixels=True,
        )
        self.assertEqual(pixels.tolist(), [[8, 8]])

    def test_depth_obstacles_use_transformer_pixel_projection(self):
        """Off-centre obstacle Y must follow the same calibrated projection as grasp points."""
        depth = np.zeros((32, 32), dtype=np.uint16)
        depth[8, 24] = 500

        def pixel_to_camera(u, v, z):
            # Deliberately exaggerate the calibrated vertical correction so a
            # pinhole-only implementation is distinguishable in this test.
            return np.array([(u - 16.0) * z / 100.0, (v - 16.0) * z / 200.0, z])

        camera = SimpleNamespace(_depth_scale=0.001)
        transformer = SimpleNamespace(
            cx=16.0,
            cy=16.0,
            fx=100.0,
            fy=100.0,
            pixel_to_camera=pixel_to_camera,
            T_flange_to_cam=np.eye(4, dtype=np.float32),
            _rpy_to_rot=lambda *_args: np.eye(3, dtype=np.float32),
        )
        points = depth_to_robot_obstacles(
            depth,
            camera,
            transformer,
            [0, 0, 0, 0, 0, 0],
            stride=8,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(float(points[0, 1]), -0.020, places=6)

    def test_camera_box_is_centered_at_palm_root(self):
        self.assertAlmostEqual(self.box_min[0], -0.15)
        self.assertAlmostEqual(self.box_max[0], -0.09)
        self.assertAlmostEqual(self.box_min[1], -0.0725)
        self.assertAlmostEqual(self.box_max[1], 0.0725)
        self.assertAlmostEqual(self.box_min[2], 0.020)
        self.assertAlmostEqual(self.box_max[2], 0.045)
        self.assertFalse(self.check_depth_path([[-0.12, 0.06, 0.0325]]).ok)
        self.assertTrue(self.check_depth_path([[-0.03, 0.0, 0.0325]]).ok)

    def test_camera_box_follows_flange_rotation(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        local_point = np.array([-0.10, 0.0, 0.0325], dtype=np.float32)
        world_point = rotation @ local_point
        self.assertFalse(self.check_depth_path([world_point], rotation=rotation).ok)
        self.assertTrue(self.check_depth_path([[-0.10, 0.09, 0.0325]]).ok)

    def test_camera_geometry_is_available_but_not_in_self_collision_model(self):
        strips = camera_capsule_strips()
        self.assertEqual(len(strips), 4)
        self.assertAlmostEqual(strips[0][0][0], -0.15)
        self.assertAlmostEqual(strips[-1][1][0], -0.09)

        guard = Nova5SelfCollisionGuard()
        camera_specs = [item for item in guard.capsules if item.name == "tool_camera"]
        self.assertEqual(len(camera_specs), 0)
        self.assertTrue(guard.check([-44.1, -20.2, 134.6, -112.7, -46.0, 90.6]).ok)
        self.assertTrue(guard.check([0.0] * 6).ok)
        self.assertTrue(guard.check([-106.5, -2.2, 98.1, -178.1, -88.2, 93.3]).ok)

    def test_self_collision_hand_envelope_and_tool_pairs(self):
        parts = hand_capsule_parts()
        self.assertEqual(len(parts["tool_adapter_flange"]), 36)
        self.assertEqual(len(parts["tool_adapter_camera_mount"]), 5)
        self.assertEqual(len(parts["tool_hand_palm"]), 24)
        self.assertEqual(len(parts["tool_hand_thumb"]), 12)
        self.assertEqual(len(parts["tool_hand_fingers"]), 15)
        self.assertAlmostEqual(parts["tool_hand_palm"][0][0][2], ADAPTER_LENGTH_Z_M)
        self.assertAlmostEqual(parts["tool_hand_fingers"][-1][1][2], 0.2405)

        mount = adapter_camera_mount_capsules()
        self.assertAlmostEqual(mount[0][0][0], -0.1505)
        self.assertAlmostEqual(mount[-1][1][0], -0.0335)

        palm_x = [capsule[0][0] for capsule in parts["tool_hand_palm"]]
        thumb_x = [capsule[0][0] for capsule in parts["tool_hand_thumb"]]
        self.assertLess(min(palm_x), 0.0)
        self.assertGreater(min(thumb_x), 0.0)
        self.assertGreater(max(thumb_x), max(palm_x))

        guard = Nova5SelfCollisionGuard()
        adapter_specs = [item for item in guard.capsules if item.name == "tool_adapter_flange"]
        mount_specs = [item for item in guard.capsules if item.name == "tool_adapter_camera_mount"]
        palm_specs = [item for item in guard.capsules if item.name == "tool_hand_palm"]
        thumb_specs = [item for item in guard.capsules if item.name == "tool_hand_thumb"]
        finger_specs = [item for item in guard.capsules if item.name == "tool_hand_fingers"]
        self.assertEqual(len(adapter_specs), 36)
        self.assertEqual(len(mount_specs), 5)
        self.assertEqual(len(palm_specs), 24)
        self.assertEqual(len(thumb_specs), 12)
        self.assertEqual(len(finger_specs), 15)

        checked_names = [
            (guard.capsules[i].name, guard.capsules[j].name)
            for i, j in guard._check_pairs
        ]
        self.assertTrue(any({a, b} == {"tool_hand_palm", "Link4"} for a, b in checked_names))
        self.assertTrue(any({a, b} == {"tool_hand_thumb", "Link4"} for a, b in checked_names))
        self.assertTrue(any({a, b} == {"tool_hand_fingers", "Link3"} for a, b in checked_names))
        self.assertFalse(any("tool_camera" in (a, b) for a, b in checked_names))
        self.assertFalse(any(
            a.startswith("tool_") and b in ("Link5", "Link6")
            or b.startswith("tool_") and a in ("Link5", "Link6")
            for a, b in checked_names
        ))

    def test_z_aero_model_restores_original_open_hand_envelope(self):
        try:
            with patch.dict(os.environ, {"TOOL_HAND_MODEL": "aero"}, clear=False):
                geometry = importlib.reload(tool_geometry)
                self.assertEqual(geometry.TOOL_HAND_MODEL, "aero")
                self.assertAlmostEqual(geometry.HAND_LENGTH_Z_M, 0.250)
                self.assertAlmostEqual(geometry.HAND_OPEN_WIDTH_Y_M, 0.150)
                self.assertEqual(len(geometry.hand_box_bounds()), 2)
                parts = geometry.hand_capsule_parts()
                self.assertEqual(set(parts), {"tool_hand_palm", "tool_hand_fingers"})
                self.assertEqual(len(parts["tool_hand_palm"]), 12)
                self.assertEqual(len(parts["tool_hand_fingers"]), 15)
        finally:
            with patch.dict(os.environ, {"TOOL_HAND_MODEL": "dh116"}, clear=False):
                importlib.reload(tool_geometry)

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
