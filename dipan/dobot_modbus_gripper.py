#!/usr/bin/env python3
"""LHandPro backend through a Nova robot tool RS485 Modbus bridge.

The hand SDK cannot open the robot-owned end-tool RS485 device.  Nova exposes
that bus through Dashboard ModbusRTUCreate/GetHoldRegs/SetHoldRegs commands,
so this module adapts the same high-level gripper API to the hand's Modbus
register dictionary.
"""

import re
import socket
import time
import os

from lhandpro_gripper import LHandProGripper


DEFAULT_DASHBOARD_PORT = 29999
DOF = 6

# LHandPro protocol address dictionary, represented as decimal Modbus offsets.
REG_MODE_CONTROL = 0xF100
REG_CURRENT_LIMIT = 0xF120
REG_POSITION_COMMAND = 0xF160
REG_POSITION_FEEDBACK = 0xF800
REG_CURRENT_FEEDBACK = 0xF840
REG_STATUS = 0xF880
REG_ALARM = 0xF8A0


class DashboardError(RuntimeError):
    pass


def _parse_dashboard_response(response):
    """Return (error_id, values) from Nova's ``0,{...},...;`` response."""
    match = re.match(r"\s*(-?\d+)\s*,\s*\{([^}]*)\}", response or "")
    if not match:
        raise DashboardError(f"无法解析机械臂响应: {response!r}")
    error_id = int(match.group(1))
    raw_values = match.group(2).strip()
    values = []
    for item in raw_values.split(",") if raw_values else ():
        text = item.strip()
        try:
            values.append(int(text, 0))
        except ValueError:
            values.append(int(float(text)))
    return error_id, values


class _DashboardClient:
    """Minimal Dashboard socket client; it never powers or moves the arm."""

    def __init__(self, ip, port=DEFAULT_DASHBOARD_PORT):
        self.ip = ip
        self.port = int(port)
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.ip, self.port), timeout=3.0)
        self.sock.settimeout(3.0)

    def send(self, command):
        if self.sock is None:
            raise DashboardError("Dashboard 未连接")
        self.sock.sendall((command.rstrip("\n") + "\n").encode())
        response = b""
        while True:
            chunk = self.sock.recv(4096)
            if not chunk:
                break
            response += chunk
            if b";" in chunk or b"}" in chunk:
                break
        return response.decode(errors="replace").strip()

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None


class _ModbusBridge:
    def __init__(self, dashboard, node_id, baud_rate):
        self.dashboard = dashboard
        self.node_id = int(node_id)
        self.baud_rate = int(baud_rate)
        self.index = None

    def _call(self, command):
        response = self.dashboard.send(command)
        error_id, values = _parse_dashboard_response(response)
        if error_id != 0:
            raise DashboardError(f"{command} 返回 ErrorID={error_id}: {response}")
        return values

    def open(self):
        # Configure the robot end port for the hand.  The final identify
        # argument enables the RS485 transceiver on Nova firmware versions that
        # expose it; older firmware accepts the same documented four-argument
        # form.
        self._call("SetToolPower(1)")
        self._call("SetToolMode(1,0,1)")
        self._call(f'SetTool485({self.baud_rate},"N",1,1)')
        try:
            values = self._call(
                f'ModbusRTUCreate({self.node_id},{self.baud_rate},"N",8,1)'
            )
        except DashboardError as exc:
            if "ErrorID=-40002" in str(exc):
                raise DashboardError(
                    f"Nova 拒绝波特率 {self.baud_rate}（-40002：第二个必选参数范围错误）。"
                    "当前 V4.6.2 通常使用 115200；请让灵巧手波特率与机械臂一致，"
                    "然后重试 --baud 115200。"
                ) from exc
            raise
        if not values:
            raise DashboardError("ModbusRTUCreate 未返回句柄")
        self.index = values[0]
        return self.index

    def close(self):
        if self.index is not None:
            try:
                self._call(f"ModbusClose({self.index})")
            finally:
                self.index = None

    def read(self, address, count=1):
        if count < 1:
            return []
        values = []
        # Nova documents 1..4 registers per call for U16 tables.
        for offset in range(0, int(count), 4):
            chunk = min(4, int(count) - offset)
            values.extend(self._call(f"GetHoldRegs({self.index},{int(address) + offset},{chunk},U16)"))
        if len(values) != count:
            raise DashboardError(f"GetHoldRegs 返回数量错误: expected={count}, got={values}")
        return values

    def write(self, address, value):
        value = int(value) & 0xFFFF
        self._call(f"SetHoldRegs({self.index},{int(address)},1,{{{value}}},U16)")


class _ModbusSDKAdapter:
    """Subset of the vendor SDK methods used by LHandProGripper."""

    def __init__(self, bridge):
        self.bridge = bridge

    def _reg(self, base, axis):
        if not 1 <= int(axis) <= DOF:
            raise ValueError(f"轴编号必须在 1..{DOF}: {axis}")
        return base + int(axis) - 1

    def _mode_control(self, axis, mode, control):
        # Each F100 register contains the protocol's mode byte followed by its
        # control byte (big-endian U16 on Modbus).
        self.bridge.write(self._reg(REG_MODE_CONTROL, axis), (int(mode) << 8) | int(control))

    def _axes(self, axis):
        return range(1, DOF + 1) if int(axis) == 0 else (int(axis),)

    def set_enable(self, axis, enabled):
        for item in self._axes(axis):
            self._mode_control(item, 0, 1 if enabled else 2)

    def set_clear_alarm(self, axis):
        for item in self._axes(axis):
            self._mode_control(item, 0, 3)

    def get_enable(self, axis):
        return bool(self.get_now_status(axis) & 0x01)

    def get_now_status(self, axis):
        return self.bridge.read(self._reg(REG_STATUS, axis), 1)[0]

    def get_now_alarm(self, axis):
        return self.bridge.read(self._reg(REG_ALARM, axis), 1)[0]

    def get_now_position(self, axis):
        return self.bridge.read(self._reg(REG_POSITION_FEEDBACK, axis), 1)[0]

    def get_now_current(self, axis):
        value = self.bridge.read(self._reg(REG_CURRENT_FEEDBACK, axis), 1)[0]
        return value - 65536 if value > 32767 else value

    def set_target_position(self, axis, position):
        self.bridge.write(self._reg(REG_POSITION_COMMAND, axis), position)

    def set_position_velocity(self, axis, velocity):
        # Position-speed is exposed as a single-joint command pair F204/F205.
        self.bridge.write(0xF204, axis)
        self.bridge.write(0xF205, velocity)

    def set_max_current(self, axis, current):
        self.bridge.write(self._reg(REG_CURRENT_LIMIT, axis), current)

    def move_motors(self, axis):
        for item in self._axes(axis):
            self._mode_control(item, 0, 5)  # absolute-position command

    def home_motors(self, axis):
        for item in self._axes(axis):
            self._mode_control(item, 5, 4)  # homing mode + trigger

    def stop_motors(self, axis):
        for item in self._axes(axis):
            # Control=0 is the protocol's idle/stop command.  Do not latch the
            # emergency-stop code here: flexible_close may immediately issue
            # an open command after a high-current safety stop.
            self._mode_control(item, 0, 0)

    def set_home_current(self, current):
        del current  # No separate home-current register in the exposed map.

    def log_clear(self):
        return None

    def log_on(self, *args):
        del args

    def log_save(self, path):
        del path


class _ControllerShim:
    def __init__(self, adapter):
        self.sdk_handle = adapter
        self.is_connected = True


class DobotModbusGripper(LHandProGripper):
    """Drop-in gripper backend using the robot's end-tool RS485 bridge."""

    def __init__(self, arm=None, robot_ip="192.168.5.1", robot_port=DEFAULT_DASHBOARD_PORT,
                 baud_rate=None, node_id=None, profile_path=None, debug=False):
        if baud_rate is None:
            baud_rate = int(os.getenv("LHAND_DOBOT_BAUD", "115200"))
        super().__init__(port=None, baud_rate=baud_rate, node_id=node_id,
                         profile_path=profile_path, debug=debug)
        self.arm = arm
        self.robot_ip = robot_ip
        self.robot_port = int(robot_port)
        self._dashboard = None
        self._bridge = None
        self._motors_enabled_by_us = False

    def connect(self, enable_motors=None, home_motors=None, home_timeout_s=None):
        if self.controller is not None:
            return True
        dashboard = self.arm if self.arm is not None else _DashboardClient(self.robot_ip, self.robot_port)
        owns_dashboard = self.arm is None
        try:
            if owns_dashboard:
                dashboard.connect()
            bridge = _ModbusBridge(dashboard, self.node_id, self.baud_rate)
            self._dashboard, self._bridge = dashboard, bridge
            bridge.open()
            self.controller = _ControllerShim(_ModbusSDKAdapter(bridge))
            self.dof_active = DOF
            print(f"[LHandPro/机械臂485] 已连接: node={self.node_id} baud={self.baud_rate} dof={DOF}")
            enable = True if enable_motors is None else bool(enable_motors)
            home = True if home_motors is None else bool(home_motors)
            if enable:
                self._enable_motors()
                self._motors_enabled_by_us = True
            if home:
                self.home(timeout_s=home_timeout_s)
            return True
        except Exception:
            self.controller = None
            if self._bridge is not None:
                try:
                    self._bridge.close()
                except Exception:
                    pass
            self._bridge = None
            if owns_dashboard and self._dashboard is not None:
                self._dashboard.close()
            self._dashboard = None
            raise

    def home(self, timeout_s=None):
        timeout_s = max(3.0, float(timeout_s or 45.0))
        sdk = self._sdk
        sdk.set_clear_alarm(0)
        print(f"[LHandPro/机械臂485] 开始六轴回零，超时 {timeout_s:.0f}s")
        sdk.home_motors(0)
        deadline = time.monotonic() + timeout_s
        stable = 0
        previous = None
        while time.monotonic() < deadline:
            time.sleep(max(0.1, float(os.getenv('LHAND_HOME_POLL_S', '0.25'))))
            snapshot = self._motor_snapshot()
            if any(item['alarm'] for item in snapshot):
                raise RuntimeError('回零期间检测到报警: ' + self._snapshot_label(snapshot))
            positions = tuple(item['position'] for item in snapshot)
            homing = any(item['status'] & 0x10 for item in snapshot)
            stable = stable + 1 if previous == positions and not homing else 0
            previous = positions
            if self.debug:
                print('[LHandPro/机械臂485][回零] ' + self._snapshot_label(snapshot))
            if stable >= 4 and not homing:
                print('[LHandPro/机械臂485] 回零流程已停止')
                return True
        raise TimeoutError('机械臂转发模式六轴回零超时: ' + self._snapshot_label(self._motor_snapshot()))

    def disconnect(self):
        self._holding = False
        controller = self.controller
        self.controller = None
        if controller is not None and self._motors_enabled_by_us:
            try:
                controller.sdk_handle.set_enable(0, False)
            except Exception as exc:
                print(f'[LHandPro/机械臂485][警告] 禁用电机失败: {exc}')
        if self._bridge is not None:
            try:
                self._bridge.close()
            except Exception as exc:
                print(f'[LHandPro/机械臂485][警告] ModbusClose 失败: {exc}')
        self._bridge = None
        self._motors_enabled_by_us = False
        if self.arm is None and self._dashboard is not None:
            self._dashboard.close()
        self._dashboard = None
        print('[LHandPro/机械臂485] 已断开')
