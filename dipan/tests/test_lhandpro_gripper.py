import csv
import os
import tempfile
import unittest
from unittest.mock import patch

from lhandpro_gripper import (
    LHandProGripper,
    load_empty_profile,
    profile_current_envelope,
)


class FakeSDK:
    def __init__(self):
        self.positions = [0] * 6
        self.stopped = False
        self.enabled = False
        self.homing = False
        self.homing_status_reads = 0
        self.events = []

    def set_clear_alarm(self, axis):
        self.events.append("clear_alarm")

    def set_enable(self, axis, enable):
        self.enabled = bool(enable)
        self.events.append("enable")

    def get_enable(self, axis):
        return self.enabled

    def home_motors(self, axis):
        self.homing = True
        self.homing_status_reads = 0
        self.events.append("home")

    def get_now_status(self, axis):
        if not self.homing:
            return 0
        self.homing_status_reads += 1
        if self.homing_status_reads <= 6:
            return 7
        self.homing = False
        return 0

    def get_now_alarm(self, axis):
        return 0

    def get_now_position(self, axis):
        return self.positions[axis - 1]

    def set_target_position(self, axis, position):
        self.positions[axis - 1] = position

    def set_position_velocity(self, axis, velocity):
        pass

    def set_max_current(self, axis, current):
        pass

    def move_motors(self, axis):
        pass

    def stop_motors(self, axis):
        self.stopped = True

    def get_now_current(self, axis):
        base = 100
        if axis in (3, 4) and self.positions[axis - 1] >= 1500:
            return base + 120
        return base


class FakeController:
    latest = None

    def __init__(self, **kwargs):
        self.sdk_handle = FakeSDK()
        self.is_connected = False
        self.connect_kwargs = None
        FakeController.latest = self

    def connect(self, **kwargs):
        self.connect_kwargs = kwargs
        self.is_connected = True
        return True

    def get_dof(self):
        return 11, 6

    def stop_motors(self):
        self.sdk_handle.stop_motors(0)

    def disconnect(self):
        self.is_connected = False


class LHandProfileTests(unittest.TestCase):
    def test_wrapper_homes_after_vendor_connect_configuration(self):
        hand = LHandProGripper(controller_factory=FakeController)
        with patch.dict(os.environ, {"LHAND_HOME_POLL_S": "0.1"}, clear=False):
            hand.connect(enable_motors=True, home_motors=True, home_timeout_s=5)
        controller = FakeController.latest
        self.assertFalse(controller.connect_kwargs["enable_motors"])
        self.assertFalse(controller.connect_kwargs["home_motors"])
        self.assertEqual(controller.sdk_handle.events[:3], ["enable", "clear_alarm", "enable"])
        self.assertIn("home", controller.sdk_handle.events)

    def test_profile_loader_and_window_envelope(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "empty.csv")
            with open(path, "w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=["command"] + [f"current_{axis}" for axis in range(1, 7)],
                )
                writer.writeheader()
                writer.writerow({"command": 0, **{f"current_{axis}": 100 + axis for axis in range(1, 7)}})
                writer.writerow({"command": 200, **{f"current_{axis}": 200 + axis for axis in range(1, 7)}})
            profile = load_empty_profile(path)
            self.assertEqual(len(profile), 2)
            self.assertEqual(profile_current_envelope(profile, 100, 3, 100), 203)

    def test_flexible_close_uses_profile_and_marks_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "empty.csv")
            with open(path, "w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=["command"] + [f"current_{axis}" for axis in range(1, 7)],
                )
                writer.writeheader()
                for command in range(0, 3001, 250):
                    writer.writerow({"command": command, **{f"current_{axis}": 100 for axis in range(1, 7)}})

            hand = LHandProGripper(profile_path=path, controller_factory=FakeController)
            hand.connect(enable_motors=False, home_motors=False)
            environment = {
                "LHAND_FLEX_REQUIRED_AXES": "3,4",
                "LHAND_FLEX_ACTIVE_AXES": "3,4",
                "LHAND_FLEX_MAX_POSITIONS": "3000",
                "LHAND_FLEX_STEP": "500",
                "LHAND_FLEX_SETTLE_S": "0.035",
                "LHAND_FLEX_SAMPLES": "1",
                "LHAND_FLEX_CONSECUTIVE": "1",
                "LHAND_FLEX_PROFILE_DELTA": "80",
                "LHAND_FLEX_MIN_CONTACTS": "2",
                "LHAND_FLEX_HARD_CURRENT": "700",
            }
            with patch.dict(os.environ, environment, clear=False), patch("time.sleep"):
                self.assertTrue(hand.flexible_close())
            self.assertTrue(hand.is_holding())

    def test_missing_profile_refuses_flexible_close(self):
        with tempfile.TemporaryDirectory() as directory:
            hand = LHandProGripper(
                profile_path=os.path.join(directory, "missing.csv"),
                controller_factory=FakeController,
            )
            hand.connect(enable_motors=False, home_motors=False)
            self.assertFalse(hand.flexible_close())
            self.assertFalse(hand.is_holding())


if __name__ == "__main__":
    unittest.main()
