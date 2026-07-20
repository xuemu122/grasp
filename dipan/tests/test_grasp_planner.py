import os
import sys
import unittest
from types import SimpleNamespace

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
        depth[68:76, :] = 500
        transformer = FakeTransformer()
        previous_min_row_span = grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX
        previous_min_z_span = grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M
        grasp_planner.SHELF_BOARD_MIN_ROW_SPAN_PX = 30
        grasp_planner.SHELF_BOARD_MIN_Z_SPAN_M = 0.1
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
        self.assertIsNotNone(bounds)
        self.assertAlmostEqual(bounds["lower"]["z"], 0.3, places=3)
        self.assertAlmostEqual(bounds["upper"]["z"], 0.5, places=3)

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


if __name__ == "__main__":
    unittest.main()
