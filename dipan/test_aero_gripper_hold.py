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


class AeroGripperHoldTest(unittest.TestCase):
    def setUp(self):
        self.gripper = AeroGripper.__new__(AeroGripper)
        self.gripper.hand = FakeHand()
        self.gripper.ok = True
        self.gripper._hold_hz = 100.0
        self.gripper._hold_pose = None
        self.gripper._hold_lock = threading.Lock()
        self.gripper._hold_stop = threading.Event()
        self.gripper._hold_thread = None

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


if __name__ == "__main__":
    unittest.main()
