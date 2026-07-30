import unittest
from unittest.mock import patch

import move_robot


def robot_status(x, y, theta, *, move_status="running", running_status="moving", target="fangzhi"):
    return {
        "status": "OK",
        "results": {
            "current_pose": {"x": x, "y": y, "theta": theta},
            "current_floor": 1,
            "move_status": move_status,
            "running_status": running_status,
            "move_target": target,
        },
    }


class MoveRobotPollingTests(unittest.TestCase):
    target = {"x": 1.0, "y": 2.0, "theta": 0.5, "floor": 1}

    def test_running_task_finishes_after_two_stable_pose_polls(self):
        statuses = [
            robot_status(1.02, 2.01, 0.52),
            robot_status(1.01, 2.02, 0.51),
        ]
        with (
            patch.object(move_robot, "get_robot_status", side_effect=statuses) as get_status,
            patch.object(move_robot.time, "sleep"),
        ):
            result = move_robot.wait_move_done(
                object(), marker="fangzhi", timeout_s=10, target=self.target
            )

        self.assertTrue(result)
        self.assertEqual(get_status.call_count, 2)

    def test_transient_pose_match_does_not_finish_navigation(self):
        statuses = [
            robot_status(1.02, 2.01, 0.52),
            robot_status(1.5, 2.5, 0.52),
            robot_status(1.01, 2.02, 0.51, move_status="succeeded"),
        ]
        with (
            patch.object(move_robot, "get_robot_status", side_effect=statuses) as get_status,
            patch.object(move_robot.time, "sleep"),
        ):
            result = move_robot.wait_move_done(
                object(), marker="fangzhi", timeout_s=10, target=self.target
            )

        self.assertTrue(result)
        self.assertEqual(get_status.call_count, 3)

    def test_charging_waits_for_docking_even_when_pose_matches(self):
        statuses = [
            robot_status(1.01, 2.01, 0.51, target="charge"),
            robot_status(
                1.01,
                2.01,
                0.51,
                running_status="dock_to_charging_pile",
                target="charge",
            ),
        ]
        with (
            patch.object(move_robot, "get_robot_status", side_effect=statuses) as get_status,
            patch.object(move_robot.time, "sleep"),
        ):
            result = move_robot.wait_move_done(
                object(),
                marker="charge",
                timeout_s=10,
                arrive_on_docking=True,
                target=self.target,
            )

        self.assertTrue(result)
        self.assertEqual(get_status.call_count, 2)


if __name__ == "__main__":
    unittest.main()
