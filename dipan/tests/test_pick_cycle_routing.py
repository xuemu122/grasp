import unittest
from unittest.mock import ANY, Mock, call, patch

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
            patch.object(
                shelf_grasp,
                "go_to_layer_via_clearance",
                side_effect=lambda arm, idx: transfer_calls.append(idx) or True,
            ),
            patch.object(shelf_grasp, "go_to_layer") as go_layer,
            patch.object(shelf_grasp, "go_to_layer_smooth_from_place") as go_layer_from_place,
        ):
            result = shelf_grasp.finish_pick_cycle(object(), hand, layer_idx)

        return result, hand, transfer_calls, go_place, go_layer, go_layer_from_place

    def test_normal_layer_stays_at_place_after_drop(self):
        result, hand, transfer_calls, go_place, go_layer, go_layer_from_place = self.run_cycle(2)

        self.assertTrue(result)
        self.assertEqual(hand.open_calls, 1)
        self.assertEqual(transfer_calls, [])
        go_place.assert_called_once_with(ANY)
        go_layer.assert_not_called()
        go_layer_from_place.assert_not_called()

    def test_low_layer_uses_only_required_outbound_transfers(self):
        result, hand, transfer_calls, go_place, go_layer, go_layer_from_place = self.run_cycle(4)

        self.assertTrue(result)
        self.assertEqual(hand.open_calls, 1)
        self.assertEqual(transfer_calls, [3, 1])
        go_place.assert_called_once_with(ANY)
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

    def test_worker_product_name_passes_through(self):
        action, waiting_at_place, target = shelf_grasp._handle_worker_command(
            "云端咖啡", Mock(), FakeHand(), None, None, None, True
        )

        self.assertEqual(action, "target")
        self.assertTrue(waiting_at_place)
        self.assertEqual(target, "云端咖啡")

    def test_worker_layer_command_uses_safe_path_from_place(self):
        arm = Mock()
        hand = FakeHand()
        with (
            patch.object(shelf_grasp, "MOTION_SETTLE_S", 0.0),
            patch.object(shelf_grasp, "_current_layer_idx", 2),
            patch.object(shelf_grasp, "go_to_layer") as direct_move,
            patch.object(shelf_grasp, "go_to_layer_smooth_from_place", return_value=True) as safe_move,
        ):
            action, waiting_at_place, target = shelf_grasp._handle_worker_command(
                "__layer_3__", arm, hand, None, None, None, True
            )
            self.assertEqual(shelf_grasp._current_layer_idx, 3)

        self.assertEqual(action, "continue")
        self.assertFalse(waiting_at_place)
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
