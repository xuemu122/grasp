import unittest
from unittest.mock import patch

from dobot_tcp_gripper import (
    DobotTcpGripper,
    _crc16_modbus,
    _dashboard_call,
    _extract_rs485_frames,
    _send_tcp_rs485,
)


class FakeDashboard:
    def __init__(self):
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        return f"0,{{}},{command};"


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendall(self, data):
        self.sent.append(bytes(data))


class DobotTcpGripperTests(unittest.TestCase):
    def test_splits_long_frame_at_nova_forwarding_boundary(self):
        sock = FakeSocket()
        payload = bytes(range(68))
        with patch("dobot_tcp_gripper.time.sleep") as sleep:
            count = _send_tcp_rs485(sock, payload, 60, 0.001)
        self.assertEqual(count, 2)
        self.assertEqual([len(chunk) for chunk in sock.sent], [60, 8])
        self.assertEqual(b"".join(sock.sent), payload)
        sleep.assert_called_once_with(0.001)

    def test_does_not_split_short_frame(self):
        sock = FakeSocket()
        payload = bytes(range(12))
        with patch("dobot_tcp_gripper.time.sleep") as sleep:
            count = _send_tcp_rs485(sock, payload, 60, 0.001)
        self.assertEqual(count, 1)
        self.assertEqual(sock.sent, [payload])
        sleep.assert_not_called()

    def test_extracts_fragmented_control_response_and_discards_noise(self):
        body = bytes([0x48, 1, 0x81, 1, 1])
        crc = _crc16_modbus(body)
        frame = body + bytes([crc & 0xFF, crc >> 8])
        frames, remainder = _extract_rs485_frames(b"\x00\xff" + frame[:3])
        self.assertEqual(frames, [])
        frames, remainder = _extract_rs485_frames(remainder + frame[3:])
        self.assertEqual(frames, [frame])
        self.assertEqual(remainder, bytearray())

    def test_dashboard_call_rejects_nonzero_error(self):
        dashboard = FakeDashboard()
        self.assertEqual(_dashboard_call(dashboard, "SetToolPower(1)"), [])

    def test_configures_raw_transparent_port_without_modbus_create(self):
        dashboard = FakeDashboard()
        hand = DobotTcpGripper(arm=dashboard, baud_rate=500000, node_id=1)
        with patch("dobot_tcp_gripper.time.sleep"):
            hand._configure_robot_port()
        self.assertEqual(dashboard.commands, [
            "SetToolPower(1)",
            "SetToolMode(1,0,1)",
            'SetTool485(500000,"N",1,1)',
        ])
        self.assertFalse(any("ModbusCreate" in command for command in dashboard.commands))


if __name__ == "__main__":
    unittest.main()
