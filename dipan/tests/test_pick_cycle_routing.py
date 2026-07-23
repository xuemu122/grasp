import unittest
from unittest.mock import ANY, patch

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


if __name__ == "__main__":
    unittest.main()
