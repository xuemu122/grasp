import struct
import threading
import unittest

from aero_open_sdk.aero_hand import AeroHand, GET_CURR


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.buffer = bytearray()
        self.write_count = 0

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, data):
        response = self.responses[self.write_count]
        self.write_count += 1
        self.buffer.extend(response)
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data


def make_hand(serial, retries=1):
    hand = AeroHand.__new__(AeroHand)
    hand.ser = serial
    hand._serial_lock = threading.RLock()
    hand._response_timeout_s = 0.01
    hand._response_retries = retries
    return hand


class SerialResponseTests(unittest.TestCase):
    def test_skips_boot_log_before_current_frame(self):
        frame = struct.pack("<2B7h", GET_CURR, 0, 1, -2, 3, -4, 5, -6, 7)
        serial = FakeSerial([b"[BOOT] Non-power reset\r\n" + frame])
        hand = make_hand(serial)

        self.assertEqual(
            hand.get_actuator_currents(),
            [6.5, -13.0, 19.5, -26.0, 32.5, -39.0, 45.5],
        )

    def test_retries_after_missing_response(self):
        frame = struct.pack("<2B7h", GET_CURR, 0, 1, 2, 3, 4, 5, 6, 7)
        serial = FakeSerial([b"", frame])
        hand = make_hand(serial, retries=2)

        self.assertIsNotNone(hand.get_actuator_currents())
        self.assertEqual(serial.write_count, 2)


if __name__ == "__main__":
    unittest.main()
