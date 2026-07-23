import threading
import time
import unittest

from aero_gripper import AeroGripper


class FakeHand:
    def __init__(self):
        self.commands = []
        self.lock = threading.Lock()

    def set_joint_positions(self, pose):
        with self.lock:
            self.commands.append(list(pose))


class FlakyHand(FakeHand):
    def __init__(self, failing_attempts):
        super().__init__()
        self.attempt = 0
        self.failing_attempts = set(failing_attempts)

    def set_joint_positions(self, pose):
        self.attempt += 1
        if self.attempt in self.failing_attempts:
            raise TimeoutError("temporary serial timeout")
        return super().set_joint_positions(pose)


class AeroGripperHoldTest(unittest.TestCase):
    def setUp(self):
        self.gripper = AeroGripper.__new__(AeroGripper)
        self.gripper.hand = FakeHand()
        self.gripper.ok = True
        self.gripper._hold_hz = 100.0
        self.gripper._hold_max_errors = 3
        self.gripper._hold_log_interval_s = 0.0
        self.gripper._hold_pose = None
        self.gripper._hold_lock = threading.Lock()
        self.gripper._hold_stop = threading.Event()
        self.gripper._hold_thread = None
        self.gripper._hold_sent = 0
        self.gripper._hold_failures = 0

    def tearDown(self):
        self.gripper._stop_hold()

    def test_repeats_grasp_pose_until_open(self):
        grasp = [80, 70, 52, 70, 58, 59, 53]
        self.gripper._start_hold(grasp)
        time.sleep(0.045)
        with self.gripper.hand.lock:
            held_commands = list(self.gripper.hand.commands)
        self.assertGreaterEqual(len(held_commands), 3)
        self.assertTrue(all(command == grasp for command in held_commands))

        self.assertTrue(self.gripper.open())
        with self.gripper.hand.lock:
            count_after_open = len(self.gripper.hand.commands)
            self.assertEqual(self.gripper.hand.commands[-1], AeroGripper.OPEN)
        time.sleep(0.03)
        with self.gripper.hand.lock:
            self.assertEqual(len(self.gripper.hand.commands), count_after_open)

    def test_recovers_from_transient_send_failures(self):
        self.gripper.hand = FlakyHand(failing_attempts={2, 3})
        self.gripper._start_hold(AeroGripper.GRASP)
        time.sleep(0.075)

        self.assertTrue(self.gripper.ok)
        self.assertTrue(self.gripper.is_holding())
        self.assertEqual(self.gripper._hold_failures, 2)
        self.assertGreaterEqual(self.gripper._hold_sent, 3)

    def test_stops_after_consecutive_send_failures(self):
        self.gripper.hand = FlakyHand(failing_attempts={2, 3, 4, 5})
        self.gripper._start_hold(AeroGripper.GRASP)
        time.sleep(0.055)

        self.assertFalse(self.gripper.ok)
        self.assertFalse(self.gripper.is_holding())
        self.assertEqual(self.gripper._hold_failures, 3)


if __name__ == "__main__":
    unittest.main()
