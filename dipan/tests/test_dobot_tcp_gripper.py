import unittest
from unittest.mock import patch

from dobot_tcp_gripper import DobotTcpGripper, _dashboard_call


class FakeDashboard:
    def __init__(self):
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        return f"0,{{}},{command};"


class DobotTcpGripperTests(unittest.TestCase):
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
