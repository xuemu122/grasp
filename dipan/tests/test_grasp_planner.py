import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import grasp_planner


class FakeTransformer:
    fx = 100.0
    fy = 100.0
    cx = 49.5
    cy = 49.5
    T_flange_to_cam = np.eye(4)

    @staticmethod
    def _rpy_to_rot(_rx, _ry, _rz):
        return np.eye(3)

    def pixel_to_camera(self, u, v, depth_m):
        return np.array([(u - self.cx) * depth_m / self.fx, (v - self.cy) * depth_m / self.fy, depth_m])

    def camera_to_robot(self, point, _flange_pose=None, debug=False):
        return np.asarray(point, dtype=float)


class FakeCamera:
    _depth_scale = 0.001


class VerticalTransformer(FakeTransformer):
    def pixel_to_camera(self, u, v, depth_m):
        return np.array([u, v, depth_m], dtype=float)

    def camera_to_robot(self, point, _flange_pose=None, debug=False):
        return np.array([point[0], point[1], (100.0 - point[1]) * 0.002], dtype=float)


class ArbitraryWorldZTransformer(VerticalTransformer):
    def camera_to_robot(self, point, _flange_pose=None, debug=False):
        return np.array([point[0], point[1], -12.34], dtype=float)


def detection(name, row, depth):
    return SimpleNamespace(
        class_name=name,
        class_id=1,
        bbox=(35, row - 5, 65, row + 5),
        depth_m=depth,
    )


class ShelfBoardFilterTests(unittest.TestCase):
    def test_current_frame_boards_define_vertical_filter(self):
        depth = np.zeros((100, 100), dtype=np.uint16)
        depth[28:36, :] = 300
        depth[68:76, :] = 680
        transformer = FakeTransformer()
        previous_min_row_span = grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX
        previous_min_z_span = grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M
        previous_max_z_span = grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M
        grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX = 30
        grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M = 0.30
        grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M = 0.46
        try:
            bounds = grasp_planner.detect_shelf_board_bounds(
                depth,
                FakeCamera(),
                transformer,
                (0, 0, 0, 0, 0, 0),
                reference_z_m=0.3,
                exclude_bboxes=[(35, 45, 65, 55)],
            )
        finally:
            grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX = previous_min_row_span
            grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M = previous_min_z_span
            grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M = previous_max_z_span
        self.assertIsNotNone(bounds)
        self.assertAlmostEqual(bounds["lower"]["z"], 0.3, places=3)
        self.assertAlmostEqual(bounds["upper"]["z"], 0.68, places=3)

        previous_max_depth = grasp_planner.MAX_GRASP_DEPTH_M
        grasp_planner.MAX_GRASP_DEPTH_M = 1.0
        try:
            target, _ = grasp_planner.pick_target_box(
                [detection("upper", 15, 0.6), detection("target", 50, 0.4), detection("lower", 85, 0.2)],
                transformer=transformer,
                flange_pose=(0, 0, 0, 0, 0, 0),
                layer_z_m=0.3,
                shelf_board_bounds=bounds,
            )
        finally:
            grasp_planner.MAX_GRASP_DEPTH_M = previous_max_depth
        self.assertEqual(target.class_name, "target")

        report = grasp_planner.explain_target_filter(
            [detection("upper", 15, 0.6), detection("target", 50, 0.4), detection("lower", 85, 0.2)],
            transformer=transformer,
            flange_pose=(0, 0, 0, 0, 0, 0),
            layer_z_m=0.3,
            shelf_board_bounds=bounds,
        )
        self.assertEqual(report["mode"], "dynamic")
        self.assertEqual([item["accepted"] for item in report["detections"]], [False, True, False])

    def test_detection_box_does_not_exclude_visible_board_pixels_on_same_row(self):
        depth = np.zeros((100, 200), dtype=np.uint16)
        depth[20:28, :] = 300
        depth[68:76, :] = 680
        transformer = FakeTransformer()
        previous_min_row_span = grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX
        previous_min_z_span = grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M
        previous_max_z_span = grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M
        grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX = 30
        grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M = 0.30
        grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M = 0.46
        try:
            bounds = grasp_planner.detect_shelf_board_bounds(
                depth,
                FakeCamera(),
                transformer,
                (0, 0, 0, 0, 0, 0),
                reference_z_m=0.3,
                # The detection reaches into the lower board rows, but the
                # board remains visible to the left and right of the box.
                exclude_bboxes=[(70, 40, 130, 72)],
            )
        finally:
            grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX = previous_min_row_span
            grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M = previous_min_z_span
            grasp_planner.SHELF_BOARD_MAX_Z_SPAN_M = previous_max_z_span

        self.assertIsNotNone(bounds)
        self.assertIn("lower", bounds)
        self.assertIn("upper", bounds)
        self.assertAlmostEqual(bounds["lower"]["z"], 0.3, places=3)
        self.assertAlmostEqual(bounds["upper"]["z"], 0.68, places=3)

    def test_implausibly_small_board_spacing_is_not_used(self):
        depth = np.zeros((100, 100), dtype=np.uint16)
        depth[28:36, :] = 300
        depth[68:76, :] = 500
        with (
            patch.object(grasp_planner, "SHELF_BOARD_MIN_ROW_SPAN_PX", 30),
            patch.object(grasp_planner, "SHELF_BOARD_MIN_Z_SPAN_M", 0.30),
            patch.object(grasp_planner, "SHELF_BOARD_MAX_Z_SPAN_M", 0.46),
        ):
            bounds = grasp_planner.detect_shelf_board_bounds(
                depth,
                FakeCamera(),
                FakeTransformer(),
                (0, 0, 0, 0, 0, 0),
                reference_z_m=0.3,
                exclude_bboxes=[(35, 45, 65, 55)],
            )

        self.assertIsNotNone(bounds)
        self.assertNotIn("lower", bounds)
        self.assertNotIn("upper", bounds)

    def test_dynamic_z_rejection_falls_back_to_current_shelf_rows(self):
        bounds = {
            "row_min": 20,
            "row_max": 80,
            "z_min": 0.30,
            "z_max": 0.40,
        }
        target, _ = grasp_planner.pick_target_box(
            [detection("coke", 50, 0.45)],
            transformer=FakeTransformer(),
            flange_pose=(0, 0, 0, 0, 0, 0),
            layer_z_m=0.35,
            shelf_board_bounds=bounds,
        )

        self.assertEqual(target.class_name, "coke")

    def test_side_grasp_moves_original_selection_down_one_centimeter(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 100), depth_m=0.2)
        with (
            patch.object(grasp_planner, "EDGE_MIDPOINT_GRASP_ENABLE", False),
            patch.object(grasp_planner, "FORCE_SIDE_GRASP", True),
            patch.object(grasp_planner, "SIDE_GRASP_MIN_HEIGHT_M", 0.065),
            patch.object(grasp_planner, "SIDE_GRASP_MAX_HEIGHT_M", 0.11),
            patch.object(grasp_planner, "SIDE_GRASP_LOWER_SHIFT_M", 0.01),
        ):
            _, _, world_z, mode, _, grip_pixel = grasp_planner.plan_edge_grasp(
                det,
                "right",
                VerticalTransformer(),
                (0, 0, 0, 0, 0, 0),
                fx=100.0,
            )

        self.assertEqual(mode, "right_side")
        self.assertAlmostEqual(world_z, 0.09, places=5)
        self.assertAlmostEqual(grip_pixel[1], 55.0, places=5)
        clearance = grasp_planner.side_grasp_bottom_clearance_m(det, grip_pixel, 100.0)
        self.assertAlmostEqual(clearance, 0.09, places=5)

    def test_side_grasp_minimum_is_measured_from_detection_box_bottom(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 100), depth_m=0.14)
        with (
            patch.object(grasp_planner, "EDGE_MIDPOINT_GRASP_ENABLE", False),
            patch.object(grasp_planner, "FORCE_SIDE_GRASP", True),
            patch.object(grasp_planner, "SIDE_GRASP_MIN_HEIGHT_M", 0.065),
            patch.object(grasp_planner, "SIDE_GRASP_MAX_HEIGHT_M", 0.11),
            patch.object(grasp_planner, "SIDE_GRASP_LOWER_SHIFT_M", 0.01),
        ):
            _, _, world_z, mode, _, grip_pixel = grasp_planner.plan_edge_grasp(
                det,
                "right",
                ArbitraryWorldZTransformer(),
                (0, 0, 0, 0, 0, 0),
                fx=100.0,
            )

        self.assertEqual(mode, "right_side")
        self.assertEqual(world_z, -12.34)
        clearance = grasp_planner.side_grasp_bottom_clearance_m(det, grip_pixel, 100.0)
        self.assertAlmostEqual(clearance, 0.068, places=5)
        self.assertAlmostEqual(grip_pixel[1], 51.4285714, places=5)

    def test_side_grasp_maximum_is_measured_from_detection_box_bottom(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 200), depth_m=0.2)
        with (
            patch.object(grasp_planner, "EDGE_MIDPOINT_GRASP_ENABLE", False),
            patch.object(grasp_planner, "FORCE_SIDE_GRASP", True),
            patch.object(grasp_planner, "SIDE_GRASP_MIN_HEIGHT_M", 0.065),
            patch.object(grasp_planner, "SIDE_GRASP_MAX_HEIGHT_M", 0.11),
            patch.object(grasp_planner, "SIDE_GRASP_LOWER_SHIFT_M", 0.01),
        ):
            _, _, _, mode, _, grip_pixel = grasp_planner.plan_edge_grasp(
                det,
                "right",
                ArbitraryWorldZTransformer(),
                (0, 0, 0, 0, 0, 0),
                fx=100.0,
            )

        self.assertEqual(mode, "right_side")
        clearance = grasp_planner.side_grasp_bottom_clearance_m(det, grip_pixel, 100.0)
        self.assertAlmostEqual(clearance, 0.11, places=5)

    def test_near_miss_log_bbox_keeps_clearance_relative_to_box_bottom(self):
        det = SimpleNamespace(bbox=(457, 233, 551, 444), depth_m=0.318)
        transformer = ArbitraryWorldZTransformer()
        transformer.fy = 390.0
        with (
            patch.object(grasp_planner, "EDGE_MIDPOINT_GRASP_ENABLE", False),
            patch.object(grasp_planner, "FORCE_SIDE_GRASP", True),
            patch.object(grasp_planner, "SIDE_GRASP_MIN_HEIGHT_M", 0.065),
            patch.object(grasp_planner, "SIDE_GRASP_MAX_HEIGHT_M", 0.11),
            patch.object(grasp_planner, "SIDE_GRASP_LOWER_SHIFT_M", 0.01),
        ):
            _, _, _, mode, _, grip_pixel = grasp_planner.plan_edge_grasp(
                det,
                "right",
                transformer,
                (0, 0, 0, 0, 0, 0),
                fx=390.4,
            )

        self.assertEqual(mode, "right_side")
        clearance = grasp_planner.side_grasp_bottom_clearance_m(det, grip_pixel, transformer.fy)
        self.assertAlmostEqual(clearance, 0.0760192, places=5)

    def test_side_grasp_clearance_is_invalid_without_depth(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 100), depth_m=None)
        self.assertIsNone(grasp_planner.side_grasp_bottom_clearance_m(det, (30, 50), 390.0))

    def test_default_grasp_uses_selected_bbox_edge_midpoint(self):
        det = SimpleNamespace(bbox=(10, 20, 30, 100), depth_m=0.2)
        with patch.object(grasp_planner, "EDGE_MIDPOINT_GRASP_ENABLE", True):
            _, _, _, mode, _, grip_pixel = grasp_planner.plan_edge_grasp(
                det,
                "right",
                FakeTransformer(),
                (0, 0, 0, 0, 0, 0),
                fx=100.0,
            )
        self.assertEqual(mode, "right_side")
        self.assertEqual(grip_pixel, (30.0, 60.0))

    def test_low_side_grasp_pixel_is_moved_to_preferred_safe_height(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 100), depth_m=0.2)
        corrected, before_m, after_m = grasp_planner.clamp_side_grasp_pixel_to_safe_clearance(
            det, (30, 90), 100.0
        )
        self.assertAlmostEqual(before_m, 0.02, places=5)
        self.assertAlmostEqual(after_m, 0.070, places=5)
        self.assertAlmostEqual(corrected[1], 65.0, places=5)

    def test_high_side_grasp_pixel_is_moved_to_maximum_safe_height(self):
        det = SimpleNamespace(bbox=(10, 0, 30, 100), depth_m=0.2)
        corrected, before_m, after_m = grasp_planner.clamp_side_grasp_pixel_to_safe_clearance(
            det, (30, 20), 100.0
        )
        self.assertAlmostEqual(before_m, 0.16, places=5)
        self.assertAlmostEqual(after_m, 0.11, places=5)
        self.assertAlmostEqual(corrected[1], 45.0, places=5)


if __name__ == "__main__":
    unittest.main()
