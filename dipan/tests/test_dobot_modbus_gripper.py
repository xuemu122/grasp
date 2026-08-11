import unittest

from dobot_modbus_gripper import (
    DashboardError,
    REG_MODE_CONTROL,
    REG_POSITION_COMMAND,
    _ModbusBridge,
    _ModbusSDKAdapter,
    _parse_dashboard_response,
)


class FakeDashboard:
    def __init__(self):
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        if command.startswith("ModbusRTUCreate"):
            return "0,{3},ModbusRTUCreate();"
        if command.startswith("GetHoldRegs"):
            count = int(command.split(",")[2])
            return f"0,{{{','.join(str(100 + i) for i in range(count))}}},GetHoldRegs();"
        return "0,{},ok();"


class MemoryBridge:
    def __init__(self):
        self.registers = {}
        self.writes = []

    def read(self, address, count=1):
        return [self.registers.get(address + offset, 0) for offset in range(count)]

    def write(self, address, value):
        self.registers[address] = value
        self.writes.append((address, value))


class DobotModbusGripperTests(unittest.TestCase):
    def test_parse_dashboard_response_supports_decimal_and_hex(self):
        self.assertEqual(_parse_dashboard_response("0,{1,0x10,65535},x();"), (0, [1, 16, 65535]))
        with self.assertRaises(DashboardError):
            _parse_dashboard_response("unexpected")

    def test_bridge_configures_tool_rs485_and_chunks_reads(self):
        dashboard = FakeDashboard()
        bridge = _ModbusBridge(dashboard, node_id=1, baud_rate=500000)
        self.assertEqual(bridge.open(), 3)
        self.assertEqual(bridge.read(0xF800, 6), [100, 101, 102, 103, 100, 101])
        self.assertEqual(dashboard.commands[:4], [
            "SetToolPower(1)",
            "SetToolMode(1,0,1)",
            'SetTool485(500000,"N",1,1)',
            'ModbusRTUCreate(1,500000,"N",8,1)',
        ])
        self.assertIn("GetHoldRegs(3,63488,4,U16)", dashboard.commands)
        self.assertIn("GetHoldRegs(3,63492,2,U16)", dashboard.commands)

    def test_adapter_writes_big_endian_mode_control_and_position(self):
        bridge = MemoryBridge()
        adapter = _ModbusSDKAdapter(bridge)
        adapter.set_enable(1, True)
        adapter.set_target_position(1, 1234)
        adapter.move_motors(1)
        self.assertEqual(bridge.writes, [
            (REG_MODE_CONTROL, 0x0001),
            (REG_POSITION_COMMAND, 1234),
            (REG_MODE_CONTROL, 0x0005),
        ])

    def test_info_reads_do_not_write_control_registers(self):
        bridge = MemoryBridge()
        bridge.registers.update({0xF880: 1, 0xF8A0: 0, 0xF800: 456, 0xF840: 20})
        adapter = _ModbusSDKAdapter(bridge)
        self.assertTrue(adapter.get_enable(1))
        self.assertEqual(adapter.get_now_alarm(1), 0)
        self.assertEqual(adapter.get_now_position(1), 456)
        self.assertEqual(adapter.get_now_current(1), 20)
        self.assertEqual(bridge.writes, [])


if __name__ == "__main__":
    unittest.main()
