import unittest
from unittest.mock import patch

from dobot_tcp_gripper import (
    DobotTcpGripper,
    _crc16_modbus,
    _dashboard_call,
    _extract_rs485_frames,
    _send_tcp_rs485,
    _CompactMotorAdapter,
    _SdoMotorAdapter,
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


class FakeSDK:
    def __init__(self):
        self.calls = []

    def _call_checked(self, suffix, operation, *args):
        self.calls.append((suffix, operation, args))


class FakeFrameSender:
    def __init__(self):
        self.frames = []

    def __call__(self, frame):
        self.frames.append(bytes(frame))
        return True


class DobotTcpGripperTests(unittest.TestCase):
    def test_compact_adapter_matches_vendor_enable_and_home_examples(self):
        sender = FakeFrameSender()
        adapter = _CompactMotorAdapter(FakeSDK(), sender, 1, 6, direction_bits=0x20)
        adapter.set_enable(0, True)
        adapter.home_motors(0)
        self.assertEqual(
            sender.frames[0],
            bytes.fromhex(
                "50 01 81 06 20 01 20 01 20 01 20 01 20 01 20 01 e8 4a"
            ),
        )
        self.assertEqual(
            sender.frames[1],
            bytes.fromhex(
                "50 01 81 06 25 04 25 04 25 04 25 04 25 04 25 04 50 3e"
            ),
        )

    def test_compact_adapter_sends_current_speed_then_position(self):
        sender = FakeFrameSender()
        adapter = _CompactMotorAdapter(FakeSDK(), sender, 1, 6, direction_bits=0x20)
        for axis in range(1, 7):
            adapter.set_max_current(axis, 1000)
            adapter.set_position_velocity(axis, 2000)
            adapter.set_target_position(axis, 5000)
        adapter.move_motors(0)
        self.assertEqual([frame[2] for frame in sender.frames], [0x84, 0x83, 0x82])
        self.assertEqual(
            sender.frames[0],
            bytes.fromhex(
                "50 01 84 06 e8 03 e8 03 e8 03 e8 03 e8 03 e8 03 97 c0"
            ),
        )
        self.assertEqual(
            sender.frames[1],
            bytes.fromhex(
                "50 01 83 06 d0 07 d0 07 d0 07 d0 07 d0 07 d0 07 7a d5"
            ),
        )
        self.assertEqual(
            sender.frames[2],
            bytes.fromhex(
                "50 01 82 06 88 13 88 13 88 13 88 13 88 13 88 13 62 7b"
            ),
        )

        sender.frames.clear()
        for axis in range(1, 7):
            adapter.set_target_position(axis, 5500)
        adapter.move_motors(0)
        self.assertEqual([frame[2] for frame in sender.frames], [0x82])

        sender.frames.clear()
        adapter.force_velocity_broadcast()
        adapter.move_motors(0)
        self.assertEqual([frame[2] for frame in sender.frames], [0x83, 0x82])

    def test_sdo_adapter_maps_six_axis_motion_objects(self):
        sdk = FakeSDK()
        sender = FakeFrameSender()
        adapter = _SdoMotorAdapter(sdk, sender, 1, 6, direction_bits=0x20)
        adapter.set_enable(1, True)
        adapter.set_target_position(1, 1234)
        adapter.set_position_velocity(1, 5000)
        adapter.set_max_current(1, 600)
        adapter.move_motors(1)
        self.assertEqual([frame[2] for frame in sender.frames], [0x23] * 5)
        self.assertEqual([frame[3:6] for frame in sender.frames], [
            b"\x00\xf1\x01", b"\x60\xf1\x01", b"\x40\xf1\x01",
            b"\x20\xf1\x01", b"\x00\xf1\x01",
        ])
        self.assertEqual(sender.frames[0][6:10], bytes.fromhex("20 01 00 00"))
        self.assertEqual(sender.frames[1][6:10], (1234).to_bytes(4, "little"))
        self.assertTrue(all(_crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little")
                            for frame in sender.frames))

    def test_sdo_adapter_homes_all_axes_with_direction_bits(self):
        sdk = FakeSDK()
        sender = FakeFrameSender()
        adapter = _SdoMotorAdapter(sdk, sender, 1, 2, direction_bits=0x20)
        adapter.home_motors(0)
        self.assertEqual([frame[5] for frame in sender.frames], [1, 2])
        self.assertEqual([frame[6:10] for frame in sender.frames], [
            bytes.fromhex("25 04 00 00"), bytes.fromhex("25 04 00 00")
        ])

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
            "SetToolMode(1,0,1)",
            'SetTool485(500000,"N",1,1)',
            "SetToolPower(1)",
        ])
        self.assertFalse(any("ModbusCreate" in command for command in dashboard.commands))

    def test_can_power_cycle_tool_before_reconfiguring_rs485(self):
        dashboard = FakeDashboard()
        hand = DobotTcpGripper(arm=dashboard, baud_rate=500000, node_id=1)
        with patch.dict("dobot_tcp_gripper.os.environ", {
            "LHAND_DOBOT_POWER_CYCLE": "1",
            "LHAND_DOBOT_POWER_ON_WAIT_S": "0.1",
        }), patch("dobot_tcp_gripper.time.sleep"):
            hand._configure_robot_port()
        self.assertEqual(dashboard.commands, [
            "SetToolPower(0)",
            "SetToolMode(1,0,1)",
            'SetTool485(500000,"N",1,1)',
            "SetToolPower(1)",
        ])


if __name__ == "__main__":
    unittest.main()
