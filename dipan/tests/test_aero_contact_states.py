import unittest

from demo_aero_contact_states import ContactStateClassifier, StateThresholds, profile_envelope


SENSE = {3, 4, 5, 6}
EMPTY = [100.0] * 7


def currents(extra):
    values = EMPTY[:]
    for joint, value in extra.items():
        values[joint] += value
    return values


class ContactStateClassifierTests(unittest.TestCase):
    def make_classifier(self, **kwargs):
        thresholds = StateThresholds(**kwargs)
        return ContactStateClassifier(SENSE, thresholds)

    def test_empty_and_shallow_contact_are_debounced(self):
        classifier = self.make_classifier()
        self.assertEqual(classifier.update(0, currents({}), EMPTY).raw_state, "empty")
        self.assertEqual(
            classifier.update(20, currents({3: 250, 4: 250}), EMPTY).raw_state,
            "empty",
        )
        result = classifier.update(21, currents({3: 250, 4: 250}), EMPTY)
        self.assertEqual(result.raw_state, "shallow_contact")
        self.assertEqual(result.contact_joints, [3, 4])

    def test_sustained_contact_and_saturated_pile(self):
        classifier = self.make_classifier()
        classifier.update(20, currents({3: 300, 4: 300, 5: 300}), EMPTY)
        classifier.update(21, currents({3: 300, 4: 300, 5: 300}), EMPTY)
        result = classifier.update(28, currents({3: 300, 4: 300, 5: 300}), EMPTY)
        self.assertEqual(result.raw_state, "contact")
        result = classifier.update(34, currents({3: 600, 4: 600, 5: 600}), EMPTY)
        self.assertEqual(result.raw_state, "saturated_pile")

    def test_overflow_has_priority_and_is_immediate(self):
        classifier = self.make_classifier()
        result = classifier.update(10, currents({3: 1500}), EMPTY)
        self.assertEqual(result.raw_state, "squeeze_overflow")
        self.assertEqual(result.overflow_joints, [3])

    def test_hard_current_protects_an_unsensed_actuator(self):
        classifier = self.make_classifier()
        result = classifier.update(10, currents({1: 6000}), EMPTY)
        self.assertEqual(result.raw_state, "squeeze_overflow")
        self.assertEqual(result.contact_joints, [])

    def test_profile_envelope_uses_maximum_in_angle_window(self):
        profile = [
            (0.0, [100.0] * 7),
            (10.0, [200.0, 150.0, 300.0, 100.0, 100.0, 100.0, 100.0]),
            (12.0, [180.0, 250.0, 250.0, 120.0, 100.0, 100.0, 100.0]),
        ]
        command, envelope = profile_envelope(profile, 11.0, 2.0)
        self.assertEqual(command, 10.0)
        self.assertEqual(envelope[:3], [200.0, 250.0, 300.0])


if __name__ == "__main__":
    unittest.main()
