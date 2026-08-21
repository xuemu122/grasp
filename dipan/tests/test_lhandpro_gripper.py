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
        self.position_commands = []
        self.position_read_count = 0
        self.velocity_commands = []
        self.rs485_baudrate = 500000

    def _get_scalar(self, suffix, c_type, operation, *args):
        del c_type, operation, args
        if suffix != "get_rs485_baudrate":
            raise AssertionError(suffix)
        return self.rs485_baudrate

    def _call_checked(self, suffix, operation, *args):
        del operation
        if suffix != "set_rs485_baudrate":
            raise AssertionError(suffix)
        self.rs485_baudrate = int(args[0])

    def set_clear_alarm(self, axis):
        self.events.append("clear_alarm")

    def set_control_mode(self, axis, mode):
        self.events.append(f"control_mode:{axis}:{mode}")

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
        self.position_read_count += 1
        return self.positions[axis - 1]

    def set_target_position(self, axis, position):
        self.positions[axis - 1] = position

    def set_position_velocity(self, axis, velocity):
        self.velocity_commands.append((axis, velocity))

    def set_max_current(self, axis, current):
        pass

    def move_motors(self, axis):
        self.position_commands.append(tuple(self.positions))

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


class StallingController(FakeController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        original_set_target = self.sdk_handle.set_target_position

        def set_target_position(axis, position):
            if axis in (3, 4):
                position = min(int(position), 1500)
            original_set_target(axis, position)

        self.sdk_handle.set_target_position = set_target_position
        self.sdk_handle.get_now_current = lambda _axis: 100


class StaleFeedbackController(FakeController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.sdk_handle.get_now_position = (
            lambda axis: 9000 if axis == 1 else 0
        )
        self.sdk_handle.get_now_current = lambda _axis: 100


class HardContactController(FakeController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        def get_now_current(axis):
            if axis in (4, 6) and self.sdk_handle.positions[axis - 1] >= 1000:
                return 800
            return 100

        self.sdk_handle.get_now_current = get_now_current


class TwoCycleStopEvent:
    def __init__(self):
        self.calls = 0

    def wait(self, _timeout):
        self.calls += 1
        return self.calls > 1

    def is_set(self):
        return self.calls > 1


class LHandProfileTests(unittest.TestCase):
    def test_default_open_pose_lowers_thumb_side_axis(self):
        hand = LHandProGripper(controller_factory=FakeController)
        hand.connect(enable_motors=False, home_motors=False)
        with patch.dict(os.environ, {}, clear=True), patch("time.sleep"):
            self.assertTrue(hand.open())
        self.assertEqual(FakeController.latest.sdk_handle.positions, [9000, 0, 0, 0, 0, 0])

    def test_open_pose_can_still_be_overridden(self):
        hand = LHandProGripper(controller_factory=FakeController)
        hand.connect(enable_motors=False, home_motors=False)
        with patch.dict(os.environ, {"LHAND_OPEN_POSITIONS": "100,0,0,0,0,0"}, clear=False), patch("time.sleep"):
            self.assertTrue(hand.open())
        self.assertEqual(FakeController.latest.sdk_handle.positions, [100, 0, 0, 0, 0, 0])

    def test_open_succeeds_when_latency_diagnostics_are_enabled(self):
        with patch.dict(os.environ, {"LHAND_LATENCY_DEBUG": "1"}, clear=False), patch("time.sleep"):
            hand = LHandProGripper(controller_factory=FakeController)
            hand.connect(enable_motors=False, home_motors=False)
            self.assertTrue(hand.open())
        self.assertEqual(FakeController.latest.sdk_handle.positions, [9000, 0, 0, 0, 0, 0])

    def test_default_open_uses_two_broadcasts_without_feedback_reads(self):
        hand = LHandProGripper(controller_factory=FakeController)
        hand.connect(enable_motors=False, home_motors=False)
        sdk = FakeController.latest.sdk_handle
        with patch.dict(os.environ, {"LHAND_OPEN_VERIFY": "0"}, clear=False), patch("time.sleep"):
            self.assertTrue(hand.open())
        self.assertEqual(sdk.position_read_count, 0)
        self.assertEqual(sdk.position_commands, [
            (9000, 0, 0, 0, 0, 0),
            (9000, 0, 0, 0, 0, 0),
        ])
        self.assertEqual(
            sdk.velocity_commands,
            [(axis, 12000) for _ in range(2) for axis in range(1, 7)],
        )

    def test_hold_does_not_rebroadcast_an_unchanged_target_before_refresh(self):
        hand = LHandProGripper(controller_factory=FakeController)
        hand.connect(enable_motors=False, home_motors=False)
        hand._hold_positions = [9000, 2500, 2500, 2500, 2500, 2500]
        hand._hold_axes = set()
        hand._hold_max_positions = [9000] * 6
        hand._hold_profile = None
        hand._hold_baseline = [100] * 6
        with patch.dict(os.environ, {
            "LHAND_HOLD_RESEND_INTERVAL_S": "60",
            "LHAND_HOLD_HZ": "100",
        }, clear=False):
            hand._hold_loop(TwoCycleStopEvent())
        self.assertEqual(FakeController.latest.sdk_handle.position_commands, [])

    def test_changes_hand_baud_without_motor_enable(self):
        hand = LHandProGripper(controller_factory=FakeController)
        hand.connect(enable_motors=False, home_motors=False)
        self.assertEqual(hand.get_rs485_baudrate(), 500000)
        self.assertEqual(hand.set_rs485_baudrate(115200), 115200)
        self.assertEqual(FakeController.latest.sdk_handle.rs485_baudrate, 115200)
        self.assertFalse(FakeController.latest.sdk_handle.enabled)
        hand.disconnect(stop_motors=False)
        self.assertFalse(FakeController.latest.sdk_handle.stopped)

    def test_wrapper_homes_after_vendor_connect_configuration(self):
        hand = LHandProGripper(controller_factory=FakeController)
        with patch.dict(os.environ, {"LHAND_HOME_POLL_S": "0.1"}, clear=False):
            hand.connect(enable_motors=True, home_motors=True, home_timeout_s=5)
        controller = FakeController.latest
        self.assertFalse(controller.connect_kwargs["enable_motors"])
        self.assertFalse(controller.connect_kwargs["home_motors"])
        self.assertEqual(
            controller.sdk_handle.events[:4],
            ["control_mode:0:0", "enable", "clear_alarm", "enable"],
        )
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

    def test_two_hard_current_axes_are_accepted_as_object_contact(self):
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

            hand = LHandProGripper(profile_path=path, controller_factory=HardContactController)
            environment = {
                "LHAND_FLEX_REQUIRED_AXES": "4,6",
                "LHAND_FLEX_ACTIVE_AXES": "4,6",
                "LHAND_FLEX_MAX_POSITIONS": "3000",
                "LHAND_FLEX_MIN_CONTACTS": "2",
                "LHAND_FLEX_HARD_CURRENT": "750",
            }
            with patch.dict(os.environ, environment, clear=False), patch("time.sleep"):
                hand.connect(enable_motors=False, home_motors=False)
                self.assertTrue(hand.flexible_close())
            self.assertTrue(hand.is_holding())
            self.assertFalse(HardContactController.latest.sdk_handle.stopped)

    def test_flexible_close_keeps_thumb_side_axis_open_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "empty.csv")
            with open(path, "w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=["command"] + [f"current_{axis}" for axis in range(1, 7)],
                )
                writer.writeheader()
                for command in range(0, 3001, 250):
                    writer.writerow({
                        "command": command,
                        **{f"current_{axis}": 100 for axis in range(1, 7)},
                    })

            hand = LHandProGripper(profile_path=path, controller_factory=FakeController)
            with patch.dict(os.environ, {}, clear=True), patch("time.sleep"):
                hand.connect(enable_motors=False, home_motors=False)
                self.assertTrue(hand.flexible_close())

            positions = FakeController.latest.sdk_handle.positions
            self.assertEqual(positions[0], 9000)
            self.assertEqual(positions[1], 3000)
            self.assertEqual(positions[4], 3000)
            self.assertEqual(positions[5], 3000)
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

    def test_empty_profile_keeps_axis_one_at_open_position(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "empty.csv")
            hand = LHandProGripper(
                profile_path=path,
                controller_factory=FakeController,
            )
            hand.connect(enable_motors=False, home_motors=False)
            with patch.dict(os.environ, {}, clear=True), patch("time.sleep"):
                hand.record_empty_profile(
                    output_path=path,
                    start=0,
                    maximum=500,
                    step=250,
                    samples=1,
                )
            commands = FakeController.latest.sdk_handle.position_commands
            self.assertTrue(commands)
            self.assertTrue(all(command[0] == 9000 for command in commands))
            rows = load_empty_profile(path)
            self.assertEqual(len(rows), 3)

    def test_flexible_close_starts_hold_and_open_stops_it(self):
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
            with patch.dict(os.environ, {
                "LHAND_FLEX_REQUIRED_AXES": "3,4",
                "LHAND_FLEX_ACTIVE_AXES": "3,4",
                "LHAND_FLEX_MAX_POSITIONS": "3000",
                "LHAND_FLEX_STEP": "500",
                "LHAND_FLEX_CONSECUTIVE": "1",
                "LHAND_FLEX_MIN_CONTACTS": "2",
            }, clear=False), patch("time.sleep"):
                hand.connect(enable_motors=False, home_motors=False)
                self.assertTrue(hand.flexible_close())
                self.assertIsNotNone(hand._hold_thread)
                self.assertTrue(hand._hold_thread.is_alive())
                hand.open()
            self.assertIsNone(hand._hold_thread)

    def test_stalled_fingers_are_accepted_as_a_firm_grip(self):
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

            hand = LHandProGripper(profile_path=path, controller_factory=StallingController)
            environment = {
                "LHAND_FLEX_REQUIRED_AXES": "3,4",
                "LHAND_FLEX_ACTIVE_AXES": "3,4",
                "LHAND_FLEX_MAX_POSITIONS": "3000",
                "LHAND_FLEX_MAX_STEPS": "6",
                "LHAND_FLEX_STALL_POLLS": "3",
                "LHAND_FLEX_MIN_CONTACTS": "2",
            }
            with patch.dict(os.environ, environment, clear=False), patch("time.sleep"):
                hand.connect(enable_motors=False, home_motors=False)
                self.assertTrue(hand.flexible_close())
                self.assertTrue(hand.is_holding())
                hand.open()

    def test_feedback_timeout_keeps_closing_and_enters_hold(self):
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

            hand = LHandProGripper(profile_path=path, controller_factory=StaleFeedbackController)
            environment = {
                "LHAND_FLEX_REQUIRED_AXES": "3,4",
                "LHAND_FLEX_ACTIVE_AXES": "3,4",
                "LHAND_FLEX_MAX_POSITIONS": "3000",
                "LHAND_FLEX_MAX_STEPS": "2",
                "LHAND_FLEX_MIN_CONTACTS": "2",
            }
            with patch.dict(os.environ, environment, clear=False), patch("time.sleep"):
                hand.connect(enable_motors=False, home_motors=False)
                self.assertTrue(hand.flexible_close())
                self.assertTrue(hand.is_holding())
                self.assertEqual(hand._hold_positions[2:4], [3000, 3000])
                hand.open()


if __name__ == "__main__":
    unittest.main()
