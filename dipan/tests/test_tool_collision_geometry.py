import unittest

import numpy as np

from depth_collision_guard import check_hand_path, hand_path_hit_mask
from self_collision_guard import Nova5SelfCollisionGuard
from tool_geometry import camera_box_bounds, camera_capsule_strips


class ToolCollisionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.box_min, self.box_max = camera_box_bounds()

    def check_depth_path(self, points, rotation=None, include_camera=True):
        return check_hand_path(
            np.asarray(points, dtype=np.float32),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3) if rotation is None else rotation,
            camera_box_min_local_m=self.box_min if include_camera else None,
            camera_box_max_local_m=self.box_max if include_camera else None,
        )

    def test_camera_point_is_not_covered_by_hand_only(self):
        point = [[-0.10, 0.0, 0.0125]]
        self.assertTrue(self.check_depth_path(point, include_camera=False).ok)
        self.assertFalse(self.check_depth_path(point, include_camera=True).ok)

        hit_mask, _ = hand_path_hit_mask(
            np.asarray(point, dtype=np.float32),
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.25,
            0.01,
            samples=2,
            flange_rotation_m=np.eye(3),
            camera_box_min_local_m=self.box_min,
            camera_box_max_local_m=self.box_max,
        )
        self.assertEqual(hit_mask.tolist(), [True])

    def test_camera_box_follows_flange_rotation(self):
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        local_point = np.array([-0.10, 0.0, 0.0125], dtype=np.float32)
        world_point = rotation @ local_point
        self.assertFalse(self.check_depth_path([world_point], rotation=rotation).ok)
        self.assertTrue(self.check_depth_path([[-0.10, 0.05, 0.0125]]).ok)

    def test_self_collision_camera_cover_and_safe_poses(self):
        strips = camera_capsule_strips()
        self.assertEqual(len(strips), 4)
        self.assertAlmostEqual(strips[0][0][0], -0.145)
        self.assertAlmostEqual(strips[-1][1][0], 0.0)

        guard = Nova5SelfCollisionGuard()
        camera_specs = [item for item in guard.capsules if item.name == "tool_camera"]
        self.assertEqual(len(camera_specs), 4)
        self.assertTrue(guard.check([-44.1, -20.2, 134.6, -112.7, -46.0, 90.6]).ok)
        self.assertTrue(guard.check([0.0] * 6).ok)


if __name__ == "__main__":
    unittest.main()
