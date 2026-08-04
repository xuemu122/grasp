import os
import unittest
from unittest.mock import patch

from aero_gripper import AeroGripper


class AeroGripperHoldTests(unittest.TestCase):
    def test_temperature_trip_reports_hot_actuator(self):
        result = AeroGripper._temperature_trip([30, 31, 32, 33, 66, 34, 35], 65)
        self.assertEqual(result, (66.0, [(4, 66.0)]))

    def test_temperature_trip_ignores_unreadable_values(self):
        self.assertIsNone(AeroGripper._temperature_trip([30] * 6, 65))
        self.assertIsNone(AeroGripper._temperature_trip([30, 31, float("nan"), 33, 34, 35, 36], 65))

    def build_monitor(self, profile=None, max_cmd=80, joints=None, min_contacts=1):
        with patch.dict(os.environ, {
            "AERO_HOLD_TIGHTEN_ENABLE": "1",
            "AERO_HOLD_MIN_EXTRA_MA": "100",
            "AERO_FLEX_STEP": "3",
            "AERO_FLEX_SETTLE": "0.05",
            "AERO_FLEX_CONSECUTIVE": "2",
        }, clear=True):
            return AeroGripper._build_hold_monitor(
                joints=joints or {3},
                profile=profile,
                baseline=[0] * 7,
                profile_window_deg=6,
                contact_delta_ma=180,
                abs_ma=650,
                max_cmd=max_cmd,
                min_contacts=min_contacts,
            )

    def test_sustained_low_contact_tightens_at_normal_close_cadence(self):
        profile = [(40.0, [100] * 7)]
        monitor = self.build_monitor(profile=profile)
        pose = [0, 0, 0, 40, 0, 0, 0]
        low_currents = [0, 0, 0, 150, 0, 0, 0]

        pose, events, weak = AeroGripper._tighten_hold_pose(pose, low_currents, monitor)
        self.assertEqual(pose[3], 40)
        self.assertEqual(events, [])
        self.assertFalse(weak)

        pose, events, weak = AeroGripper._tighten_hold_pose(pose, low_currents, monitor)
        self.assertEqual(pose[3], 43)
        self.assertEqual(len(events), 1)
        self.assertFalse(weak)

        pose, events, weak = AeroGripper._tighten_hold_pose(pose, low_currents, monitor)
        self.assertEqual(pose[3], 46)
        self.assertEqual(len(events), 1)
        self.assertFalse(weak)

    def test_joint2_uses_larger_hold_tighten_step(self):
        profile = [(40.0, [100] * 7)]
        monitor = self.build_monitor(profile=profile, joints={2})
        pose = [0, 0, 40, 0, 0, 0, 0]
        low_currents = [0, 0, 150, 0, 0, 0, 0]

        AeroGripper._tighten_hold_pose(pose, low_currents, monitor)
        AeroGripper._tighten_hold_pose(pose, low_currents, monitor)
        self.assertEqual(pose[2], 45)

    def test_good_contact_resets_low_counter(self):
        profile = [(40.0, [100] * 7)]
        monitor = self.build_monitor(profile=profile)
        pose = [0, 0, 0, 40, 0, 0, 0]

        pose, _, _ = AeroGripper._tighten_hold_pose(pose, [0, 0, 0, 150, 0, 0, 0], monitor)
        pose, _, _ = AeroGripper._tighten_hold_pose(pose, [0, 0, 0, 250, 0, 0, 0], monitor)
        pose, _, _ = AeroGripper._tighten_hold_pose(pose, [0, 0, 0, 150, 0, 0, 0], monitor)
        self.assertEqual(pose[3], 40)
        self.assertEqual(monitor["low_counts"][3], 1)

    def test_low_contact_at_limit_marks_hold_unsafe(self):
        profile = [(80.0, [100] * 7)]
        monitor = self.build_monitor(profile=profile, max_cmd=80)
        pose = [0, 0, 0, 80, 0, 0, 0]
        currents = [0, 0, 0, 150, 0, 0, 0]

        all_events = []
        for _ in range(3):
            pose, events, weak = AeroGripper._tighten_hold_pose(pose, currents, monitor)
            all_events.extend(events)
        self.assertEqual(pose[3], 80)
        self.assertEqual(len(all_events), 1)
        self.assertTrue(weak)

    def test_one_weak_finger_is_allowed_when_minimum_contacts_remain(self):
        profile = [(80.0, [100] * 7)]
        monitor = self.build_monitor(
            profile=profile,
            max_cmd=80,
            joints={3, 4},
            min_contacts=1,
        )
        pose = [0, 0, 0, 80, 80, 0, 0]

        for _ in range(3):
            pose, _, unsafe = AeroGripper._tighten_hold_pose(
                pose,
                [0, 0, 0, 150, 250, 0, 0],
                monitor,
            )
        self.assertFalse(unsafe)

        for _ in range(3):
            pose, _, unsafe = AeroGripper._tighten_hold_pose(
                pose,
                [0, 0, 0, 150, 150, 0, 0],
                monitor,
            )
        self.assertTrue(unsafe)


if __name__ == "__main__":
    unittest.main()
