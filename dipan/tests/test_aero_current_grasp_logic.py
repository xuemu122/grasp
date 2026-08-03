import unittest

from test_aero_current_grasp import update_per_finger_locks


class PerFingerLockTests(unittest.TestCase):
    def test_transient_contact_is_cancelled_while_tightening(self):
        hits = {4: 1}
        pending = {}
        losses = {4: 0}
        confirms = {4: 0}
        locked = {}

        values, events = update_per_finger_locks(
            {4: 25.0}, [4], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, ["contact:4@25.0->target:33.0"])
        self.assertEqual(values[4], 26.0)

        values, events = update_per_finger_locks(
            {4: 26.0}, [], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, [])
        self.assertEqual(locked, {})
        values, events = update_per_finger_locks(
            {4: 27.0}, [], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, ["contact_lost:4@27.0/target:33.0"])
        self.assertEqual(values[4], 28.0)
        self.assertEqual(pending, {})

    def test_finger_locks_only_after_tighter_target_was_sent(self):
        hits = {4: 1}
        pending = {}
        losses = {4: 0}
        confirms = {4: 0}
        locked = {}

        values, events = update_per_finger_locks(
            {4: 10.0}, [4], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, ["contact:4@10.0->target:18.0"])
        self.assertEqual(values[4], 11.0)
        self.assertEqual(pending, {4: 18.0})
        self.assertEqual(locked, {})

        for sent in range(11, 18):
            values, events = update_per_finger_locks(
                {4: float(sent)}, [4], {4}, hits, pending, losses, confirms, locked,
                consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
                max_cmd=80.0, step=1.0,
            )
            self.assertEqual(events, [])
            self.assertEqual(locked, {})

        values, events = update_per_finger_locks(
            {4: 18.0}, [4], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, [])
        self.assertEqual(locked, {})
        values, events = update_per_finger_locks(
            {4: 18.0}, [4], {4}, hits, pending, losses, confirms, locked,
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0,
        )
        self.assertEqual(events, ["lock:4@18.0"])
        self.assertEqual(values[4], 18.0)
        self.assertEqual(pending, {})
        self.assertEqual(locked, {4: 18.0})

    def test_thumb_follow_stops_at_its_own_limit(self):
        next_values, events = update_per_finger_locks(
            {1: 60.0, 2: 59.0, 3: 55.0}, [], {3}, {3: 0}, {}, {3: 0}, {3: 0}, {},
            consecutive=2, contact_loss_consecutive=2, extra_close=8.0,
            max_cmd=80.0, step=1.0, follow_limits={1: 60.0, 2: 60.0},
        )
        self.assertEqual(events, [])
        self.assertEqual(next_values, {1: 60.0, 2: 60.0, 3: 56.0})


if __name__ == "__main__":
    unittest.main()
