import tempfile
import unittest
from pathlib import Path
from unittest.mock import ANY, Mock, call, patch

import grasp_planner
import shelf_grasp


class FakeHand:
    def __init__(self):
        self.open_calls = 0

    def is_holding(self):
        return True

    def open(self):
        self.open_calls += 1
        return True


class PickCycleRoutingTests(unittest.TestCase):
    def test_temporary_policy_forces_wide_products_to_side_grasp(self):
        detection = Mock(depth_m=0.5, bbox=(10, 20, 130, 180))
        transformer = Mock(fy=390.4)
        transformer.pixel_to_camera.side_effect = (
            lambda u, v, depth: shelf_grasp.np.array([u / 1000.0, v / 1000.0, depth])
        )
        transformer.camera_to_robot.side_effect = (
            lambda point, flange_pose, debug=False: shelf_grasp.np.array([point[0], point[1], -point[1]])
        )
        with patch.object(grasp_planner, "FORCE_SIDE_GRASP", True):
            result = grasp_planner.plan_edge_grasp(
                detection, "right", transformer, [0, 0, 0, 0, 0, 0]
            )

        self.assertEqual(result[3], "right_side")

    def test_failure_diagnostic_is_written_to_configured_tmp_path(self):
        image = shelf_grasp.np.zeros((48, 64, 3), dtype=shelf_grasp.np.uint8)
        detection = Mock(class_id=3, confidence=0.91, bbox=(8, 9, 40, 42))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failure.jpg"
            with patch.object(shelf_grasp, "FAILURE_DIAGNOSTIC_PATH", str(output)):
                diagnostic = shelf_grasp._draw_failure_diagnostic(
                    image, [detection], detection, (24, 26)
                )
                result = shelf_grasp._save_failure_diagnostic(diagnostic)

            self.assertEqual(result, str(output))
            self.assertTrue(output.is_file())
            self.assertGreater(output.stat().st_size, 0)
            with patch.object(shelf_grasp, "FAILURE_DIAGNOSTIC_PATH", str(output)):
                shelf_grasp._clear_failure_diagnostic()
            self.assertFalse(output.exists())

    def run_cycle(self, layer_idx):
        hand = FakeHand()
        transfer_calls = []
        with (
            patch.object(shelf_grasp, "SKIP_HAND", False),
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_place_point_pose", ([0] * 6, "test-place")),
            patch.object(
                shelf_grasp,
                "_shelf_layer_poses",
                {1: ([0] * 6, "layer-1"), 2: ([0] * 6, "layer-2"),
                 3: ([0] * 6, "layer-3"), 4: ([0] * 6, "layer-4")},
            ),
            patch.object(shelf_grasp, "load_chassis_place_settings", return_value=(False, "place")),
            patch.object(shelf_grasp, "go_to_place_point", return_value=True) as go_place,
            patch.object(shelf_grasp, "go_to_stow_pose", return_value=True) as go_stow,
            patch.object(
                shelf_grasp,
                "go_to_layer_via_clearance",
                side_effect=lambda arm, idx: transfer_calls.append(idx) or True,
            ),
            patch.object(shelf_grasp, "go_to_layer") as go_layer,
            patch.object(shelf_grasp, "go_to_layer_from_non_shelf_pose") as go_layer_from_place,
        ):
            result = shelf_grasp.finish_pick_cycle(object(), hand, layer_idx)

        return result, hand, transfer_calls, go_place, go_stow, go_layer, go_layer_from_place

    def test_normal_layer_stows_after_drop(self):
        result, hand, transfer_calls, go_place, go_stow, go_layer, go_layer_from_place = self.run_cycle(2)

        self.assertTrue(result)
        self.assertEqual(hand.open_calls, 1)
        self.assertEqual(transfer_calls, [])
        go_place.assert_called_once_with(ANY, visit_transition=True)
        go_stow.assert_called_once_with(ANY)
        go_layer.assert_not_called()
        go_layer_from_place.assert_not_called()

    def test_layer_three_transfers_through_layer_two(self):
        result, hand, transfer_calls, go_place, go_stow, go_layer, go_layer_from_place = self.run_cycle(3)

        self.assertTrue(result)
        self.assertEqual(hand.open_calls, 1)
        self.assertEqual(transfer_calls, [2])
        go_place.assert_called_once_with(ANY, visit_transition=True)
        go_stow.assert_called_once_with(ANY)
        go_layer.assert_not_called()
        go_layer_from_place.assert_not_called()

    def test_low_layer_uses_layer_three_then_layer_two(self):
        result, hand, transfer_calls, go_place, go_stow, go_layer, go_layer_from_place = self.run_cycle(4)

        self.assertTrue(result)
        self.assertEqual(hand.open_calls, 1)
        self.assertEqual(transfer_calls, [3, 2])
        go_place.assert_called_once_with(ANY, visit_transition=True)
        go_stow.assert_called_once_with(ANY)
        go_layer.assert_not_called()
        go_layer_from_place.assert_not_called()

    def test_stow_uses_place_transition_instead_of_layer_two(self):
        transition = ([1, 2, 3, 4, 5, 6], "place-transition")
        stow = ([7, 8, 9, 10, 11, 12], "stow")
        arm = Mock()
        arm.move_pose_safe.return_value = True

        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_stow_pose", stow),
            patch.object(shelf_grasp, "_place_transition_pose", transition),
            patch.object(shelf_grasp, "_place_transition_joints", None),
            patch.object(shelf_grasp, "is_at_stow_pose", return_value=False),
            patch.object(shelf_grasp, "go_to_layer_via_clearance") as go_to_layer,
        ):
            self.assertTrue(shelf_grasp.go_to_stow_pose(arm))

        self.assertEqual(
            arm.move_pose_safe.call_args_list,
            [
                call(transition[0], joints=None, speed=80),
                call(stow[0], speed=80),
            ],
        )
        go_to_layer.assert_not_called()

    def test_stow_from_shelf_skips_place_transition(self):
        transition = ([1, 2, 3, 4, 5, 6], "place-transition")
        stow = ([7, 8, 9, 10, 11, 12], "stow")
        arm = Mock()
        arm.move_pose_safe.return_value = True

        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_stow_pose", stow),
            patch.object(shelf_grasp, "_place_transition_pose", transition),
            patch.object(shelf_grasp, "is_at_stow_pose", return_value=False),
        ):
            self.assertTrue(shelf_grasp.go_to_stow_pose(arm, from_shelf_pose=True))

        arm.move_pose_safe.assert_called_once_with(stow[0], speed=80)

    def test_worker_product_name_passes_through(self):
        action, layer_entry_origin, target = shelf_grasp._handle_worker_command(
            "云端咖啡", Mock(), FakeHand(), None, None, None, shelf_grasp.LAYER_ENTRY_UNKNOWN
        )

        self.assertEqual(action, "target")
        self.assertEqual(layer_entry_origin, shelf_grasp.LAYER_ENTRY_UNKNOWN)
        self.assertEqual(target, "云端咖啡")

    def test_worker_poweroff_exits_without_stow_motion(self):
        arm = Mock()
        action, layer_entry_origin, target = shelf_grasp._handle_worker_command(
            "__poweroff__", arm, FakeHand(), None, None, None, shelf_grasp.LAYER_ENTRY_UNKNOWN
        )

        self.assertEqual(action, "poweroff")
        self.assertEqual(layer_entry_origin, shelf_grasp.LAYER_ENTRY_UNKNOWN)
        self.assertIsNone(target)
        arm.assert_not_called()

    def test_worker_stow_forces_reverse_transition_before_next_layer(self):
        arm = Mock()
        hand = FakeHand()
        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_current_layer_idx", 2),
            patch.object(shelf_grasp, "go_to_stow_pose", return_value=True) as stow,
            patch.object(shelf_grasp, "go_to_layer") as direct_move,
            patch.object(shelf_grasp, "go_to_layer_from_non_shelf_pose") as unknown_move,
            patch.object(shelf_grasp, "go_to_layer_from_stow", return_value=True) as stow_move,
        ):
            action, origin, target = shelf_grasp._handle_worker_command(
                "__stow_pose__", arm, hand, None, None, None, shelf_grasp.LAYER_ENTRY_SHELF
            )
            self.assertEqual((action, origin, target),
                             ("continue", shelf_grasp.LAYER_ENTRY_STOW, None))

            action, origin, target = shelf_grasp._handle_worker_command(
                "__layer_3__", arm, hand, None, None, None, origin
            )

        stow.assert_called_once_with(arm, from_shelf_pose=True, skip_transition=False)
        stow_move.assert_called_once_with(arm, 3)
        direct_move.assert_not_called()
        unknown_move.assert_not_called()
        self.assertEqual((action, origin, target),
                         ("continue", shelf_grasp.LAYER_ENTRY_SHELF, None))

    def test_startup_stow_skips_place_transition(self):
        arm = Mock()
        with patch.object(shelf_grasp, "go_to_stow_pose", return_value=True) as stow:
            action, origin, target = shelf_grasp._handle_worker_command(
                "__stow_pose__",
                arm,
                FakeHand(),
                None,
                None,
                None,
                shelf_grasp.LAYER_ENTRY_STARTUP,
            )

        stow.assert_called_once_with(arm, from_shelf_pose=False, skip_transition=True)
        self.assertEqual((action, origin, target), ("continue", shelf_grasp.LAYER_ENTRY_STOW, None))

    def test_chassis_move_holds_transition_then_places_and_stows(self):
        arm = object()
        hand = FakeHand()
        calls = []
        with (
            patch.object(shelf_grasp, "SKIP_HAND", False),
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_place_point_pose", ([0] * 6, "test-place")),
            patch.object(shelf_grasp, "load_chassis_place_settings", return_value=(True, "place")),
            patch.object(shelf_grasp, "chassis_at_place", return_value=False),
            patch.object(
                shelf_grasp,
                "go_to_place_transition",
                side_effect=lambda _arm: calls.append("transition") or True,
            ),
            patch.object(
                shelf_grasp,
                "move_chassis_to_place",
                side_effect=lambda _marker: calls.append("move") or True,
            ),
            patch.object(
                shelf_grasp,
                "go_to_place_point",
                side_effect=lambda _arm, visit_transition=True: calls.append(f"place:{visit_transition}") or True,
            ),
            patch.object(
                shelf_grasp,
                "go_to_stow_pose",
                side_effect=lambda _arm: calls.append("stow") or True,
            ),
        ):
            self.assertTrue(shelf_grasp.finish_pick_cycle(arm, hand, 2))

        self.assertEqual(calls, ["transition", "move", "place:False", "stow"])
        self.assertEqual(hand.open_calls, 1)

    def test_stow_entry_moves_directly_to_layer_without_place_transition(self):
        arm = Mock()
        transition = ([118, 535, 572, 109, 85, -167], "place-transition")
        target = ([-443, 28, 80, 145, 85, -39], "layer-3")
        arm.move_pose_safe.return_value = True
        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_place_transition_pose", transition),
            patch.object(shelf_grasp, "_place_transition_joints", [1, 2, 3, 4, 5, 6]),
            patch.object(shelf_grasp, "_shelf_layer_poses", {3: target}),
            patch.object(shelf_grasp, "is_at_stow_pose", return_value=True),
        ):
            self.assertTrue(shelf_grasp.go_to_layer_from_stow(arm, 3))

        arm.move_pose_safe.assert_called_once_with(target[0], speed=80)

    def test_safe_layer_entry_fails_closed_when_pose_is_unreadable(self):
        arm = Mock()
        arm.get_pose.side_effect = RuntimeError("pose unavailable")
        with patch.object(
            shelf_grasp,
            "_shelf_layer_poses",
            {3: ([-443, 28, 80, 145, 85, -39], "layer-3")},
        ):
            self.assertFalse(shelf_grasp.go_to_layer_from_non_shelf_pose(arm, 3))
            self.assertFalse(shelf_grasp.go_to_layer_via_clearance(arm, 3))

        arm.movel.assert_not_called()
        arm.move_pose_safe.assert_not_called()

    def test_worker_layer_command_uses_safe_path_from_place(self):
        arm = Mock()
        hand = FakeHand()
        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_current_layer_idx", 2),
            patch.object(shelf_grasp, "go_to_layer") as direct_move,
            patch.object(shelf_grasp, "go_to_layer_from_non_shelf_pose", return_value=True) as safe_move,
        ):
            action, layer_entry_origin, target = shelf_grasp._handle_worker_command(
                "__layer_3__", arm, hand, None, None, None, shelf_grasp.LAYER_ENTRY_UNKNOWN
            )
            self.assertEqual(shelf_grasp._current_layer_idx, 3)

        self.assertEqual(action, "continue")
        self.assertEqual(layer_entry_origin, shelf_grasp.LAYER_ENTRY_SHELF)
        self.assertIsNone(target)
        safe_move.assert_called_once_with(arm, 3)
        direct_move.assert_not_called()
        self.assertEqual(hand.open_calls, 1)

    def test_detection_capture_stops_after_stable_target_support(self):
        detection = Mock(class_id=7)
        frame_detections = [[detection] for _ in range(6)]
        track = {"class_id": 7, "items": [(index, detection) for index in range(5)]}

        with (
            patch.object(shelf_grasp, "DETECTION_EARLY_STOP_FRAMES", 6),
            patch.object(shelf_grasp, "DETECTION_EARLY_STOP_SUPPORT", 5),
            patch.object(shelf_grasp, "_fuse_detection_tracks", return_value=([detection], [track])),
            patch.object(shelf_grasp, "_filter_detections_by_target_query", return_value=[detection]),
        ):
            self.assertTrue(shelf_grasp._stable_target_seen(frame_detections, "target"))
            self.assertFalse(shelf_grasp._stable_target_seen(frame_detections[:5], "target"))


if __name__ == "__main__":
    unittest.main()
