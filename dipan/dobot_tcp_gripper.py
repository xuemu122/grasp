#!/usr/bin/env python3
"""LHandPro SDK backend over Nova's TCP end-RS485 transparent port."""

import os
import socket
import sys
import threading
import time

from dobot_modbus_gripper import (
    DEFAULT_DASHBOARD_PORT,
    DashboardError,
    _DashboardClient,
    _parse_dashboard_response,
)
from lhandpro_gripper import DEFAULT_SDK_ROOT, LHandProGripper


DEFAULT_TRANSPARENT_PORT = 60000
DEFAULT_TX_CHUNK_BYTES = 4096
DEFAULT_TX_CHUNK_DELAY_S = 0.0

SDO_MODE_CONTROL = 0xF100
SDO_CURRENT_LIMIT = 0xF120
SDO_VELOCITY = 0xF140
SDO_POSITION = 0xF160


def _crc16_modbus(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return value


def _send_tcp_rs485(sock, payload, chunk_bytes, chunk_delay_s):
    """Send one complete RS485 frame, optionally chunked for diagnostics."""
    chunk_bytes = max(1, int(chunk_bytes))
    chunks = [payload[offset:offset + chunk_bytes]
              for offset in range(0, len(payload), chunk_bytes)]
    for index, chunk in enumerate(chunks):
        sock.sendall(chunk)
        if index + 1 < len(chunks) and chunk_delay_s > 0:
            time.sleep(float(chunk_delay_s))
    return len(chunks)


def _extract_rs485_frames(buffer):
    """Extract complete SDK response frames from an arbitrary TCP byte stream."""
    data = bytearray(buffer)
    frames = []
    while data:
        start = next((index for index, byte in enumerate(data) if byte in (0x48, 0x58)), None)
        if start is None:
            return frames, bytearray()
        if start:
            del data[:start]
        if data[0] == 0x58:
            frame_length = 12
        else:
            if len(data) < 4:
                break
            frame_length = 6 + data[3]
        if len(data) < frame_length:
            break
        frame = bytes(data[:frame_length])
        expected_crc = _crc16_modbus(frame[:-2])
        received_crc = frame[-2] | (frame[-1] << 8)
        if expected_crc != received_crc:
            del data[0]
            continue
        frames.append(frame)
        del data[:frame_length]
    return frames, data


def _dashboard_call(dashboard, command):
    response = dashboard.send(command)
    error_id, values = _parse_dashboard_response(response)
    if error_id != 0:
        raise DashboardError(f"{command} 返回 ErrorID={error_id}: {response}")
    return values


def _build_tcp_controller(host, port, debug=False):
    """Build a vendor controller whose physical RS485 link is a TCP socket."""
    from lhandprolib_python_sdk.controller_rs485 import RS485Controller
    from lhandprolib_python_sdk.lhandprolib_wrapper import LCN_RS485

    class TcpRS485Controller(RS485Controller):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.tcp_host = host
            self.tcp_port = int(port)
            self.tcp_socket = None
            self._tcp_stop = threading.Event()
            self._tcp_reader = None
            # One physical half-duplex Nova TCP-to-RS485 channel. Use an RLock
            # so a high-level motion reservation can keep feedback requests
            # out while its own SDK callback sends a position frame.
            self._tcp_send_lock = threading.RLock()
            self._tcp_debug = bool(debug)
            self._tcp_latency_debug = self._tcp_debug or os.getenv(
                "LHAND_LATENCY_DEBUG", "0"
            ).lower() in ("1", "true", "yes", "on")
            self._tcp_rx_buffer = bytearray()
            self._tcp_response = threading.Event()
            self._tcp_expected = None
            self._tcp_protocol_error = None
            self._tcp_rx_frame_count = 0
            self._tcp_last_rx_at = None
            self._tcp_response_timeout = max(
                0.05, float(os.getenv("LHAND_TCP_RESPONSE_TIMEOUT_S", "0.6"))
            )
            self._tcp_tx_chunk_bytes = max(
                1,
                int(os.getenv("LHAND_TCP_TX_CHUNK_BYTES", str(DEFAULT_TX_CHUNK_BYTES))),
            )
            self._tcp_tx_chunk_delay_s = max(
                0.0,
                float(os.getenv(
                    "LHAND_TCP_TX_CHUNK_DELAY_S", str(DEFAULT_TX_CHUNK_DELAY_S)
                )),
            )

        def _rs485_send_callback(self, data):
            sock = self.tcp_socket
            if sock is None or not self.is_connected:
                return False
            try:
                payload = bytes(data)
                started_at = time.perf_counter()
                response_at = None
                with self._tcp_send_lock:
                    self._tcp_response.clear()
                    self._tcp_last_rx_at = None
                    self._tcp_protocol_error = None
                    self._tcp_expected = (
                        (0x58, None) if payload[0] == 0x60
                        else (0x48, payload[2] if len(payload) > 2 else None)
                    )
                    chunk_count = _send_tcp_rs485(
                        sock,
                        payload,
                        self._tcp_tx_chunk_bytes,
                        self._tcp_tx_chunk_delay_s,
                    )
                    tx_done_at = time.perf_counter()
                    # Log only after sendall has acquired the physical
                    # transport lock. A pre-lock log made queued feedback
                    # requests look as though they had interleaved with an
                    # exclusive release frame.
                    if self._tcp_debug:
                        print(f"[LHandPro/TCP485][TX] {payload.hex(' ')}")
                    if self._tcp_debug and chunk_count > 1:
                        print(
                            f"[LHandPro/TCP485] 长帧分 {chunk_count} 段发送: "
                            f"bytes={len(payload)} chunk={self._tcp_tx_chunk_bytes} "
                            f"gap={self._tcp_tx_chunk_delay_s:.4f}s"
                        )
                    got_response = self._tcp_response.wait(self._tcp_response_timeout)
                    response_at = self._tcp_last_rx_at
                    if not got_response and payload[0] == 0x60:
                        # Nova sometimes releases TCP data in 32-byte blocks.
                        # An SDO reply can be left with its final CRC bytes in
                        # the gateway buffer. A read-only status request flushes
                        # that remainder so the normal CRC parser can validate it.
                        flush_body = bytes((0x50, payload[1], 0xD0, 0x06))
                        flush_crc = _crc16_modbus(flush_body)
                        flush_frame = flush_body + bytes((flush_crc & 0xFF, flush_crc >> 8))
                        if self._tcp_debug:
                            print("[LHandPro/TCP485] SDO回包未完整，发送只读状态请求刷新透传缓冲")
                        _send_tcp_rs485(
                            sock,
                            flush_frame,
                            self._tcp_tx_chunk_bytes,
                            self._tcp_tx_chunk_delay_s,
                        )
                        got_response = self._tcp_response.wait(
                            max(0.3, self._tcp_response_timeout)
                        )
                        response_at = self._tcp_last_rx_at
                    if not got_response:
                        if self._tcp_debug:
                            print("[LHandPro/TCP485] 等待对应响应超时")
                    protocol_error = self._tcp_protocol_error
                    self._tcp_expected = None
                if self._tcp_latency_debug:
                    rx_wait_ms = (
                        (response_at - tx_done_at) * 1000.0
                        if response_at is not None else -1.0
                    )
                    print(
                        f"[LHandPro/TCP485][时延] type=0x{payload[2]:02x} "
                        f"bytes={len(payload)} chunks={chunk_count} "
                        f"tx={(tx_done_at-started_at)*1000:.2f}ms "
                        f"rx_wait={rx_wait_ms:.2f}ms "
                        f"total={(time.perf_counter()-started_at)*1000:.2f}ms "
                        f"timeout={self._tcp_response_timeout*1000:.0f}ms "
                        f"ok={int(bool(got_response))}"
                    )
                if protocol_error:
                    if self._tcp_debug:
                        print(f"[LHandPro/TCP485] 发送被手端拒绝: {protocol_error}")
                    return False
                if payload[0] == 0x60 and not got_response:
                    return False
                return True
            except OSError as exc:
                print(f"[LHandPro/TCP485] 发送失败: {exc}")
                return False

        def send_rs485_frame(self, data):
            """Send a protocol-specific frame outside the SDK command builder."""
            return self._rs485_send_callback(bytes(data))

        def begin_motion_exclusive(self, label="motion"):
            """Block new feedback frames until ``end_motion_exclusive``.

            Acquiring the transport lock first also drains an already active
            feedback transaction, avoiding a late 0xD0 response ahead of a
            release-position acknowledgement.
            """
            self._tcp_send_lock.acquire()
            if self._tcp_debug:
                print(f"[LHandPro/TCP485] 总线独占开始: {label}")

        def end_motion_exclusive(self, label="motion"):
            if self._tcp_debug:
                print(f"[LHandPro/TCP485] 总线独占结束: {label}")
            self._tcp_send_lock.release()

        def _receive_loop(self):
            sock = self.tcp_socket
            while not self._tcp_stop.is_set():
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._tcp_stop.is_set():
                        print(f"[LHandPro/TCP485] 接收失败: {exc}")
                    break
                if not data:
                    break
                if self._tcp_debug:
                    print(f"[LHandPro/TCP485][RX] {data.hex(' ')}")
                self._tcp_rx_buffer.extend(data)
                frames, self._tcp_rx_buffer = _extract_rs485_frames(self._tcp_rx_buffer)
                for frame in frames:
                    self._tcp_rx_frame_count += 1
                    self._tcp_last_rx_at = time.perf_counter()
                    expected = self._tcp_expected
                    if frame[0] == 0x48 and frame[3] == 1 and frame[4] == 2:
                        self._tcp_protocol_error = (
                            f"type=0x{frame[2]:02x} CRC校验错误"
                        )
                        print(
                            f"[LHandPro/TCP485][协议错误] 手端报告CRC错误: "
                            f"type=0x{frame[2]:02x}"
                        )
                    if expected and frame[0] == expected[0] and (
                        expected[1] is None or frame[2] == expected[1]
                    ):
                        # Signal before SDK decode so a synchronous SDK command
                        # cannot deadlock with its receive-side decoder.
                        self._tcp_response.set()
                    if not self.sdk_handle or not self.is_connected:
                        continue
                    try:
                        self.sdk_handle.set_rs485_data_decode(frame)
                    except Exception as exc:
                        print(f"[LHandPro/TCP485] SDK 解码失败: {exc}")

        def _connect_transport(
            self,
            enable_motors,
            home_motors,
            home_wait_time,
            device_index,
            slave_id,
            auto_select,
            canfd_nom_baudrate,
            canfd_dat_baudrate,
            rs485_port_name,
            canfd_driver=None,
        ):
            del device_index, slave_id, auto_select, canfd_nom_baudrate
            del canfd_dat_baudrate, rs485_port_name, canfd_driver
            print(f"Using Nova TCP RS485 transparent port {self.tcp_host}:{self.tcp_port}")
            self.tcp_socket = socket.create_connection(
                (self.tcp_host, self.tcp_port), timeout=3.0
            )
            self.tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.tcp_socket.settimeout(0.25)
            self.is_connected = True
            self.sdk_handle.set_send_rs485_callback(self._rs485_send_callback)
            self._tcp_stop.clear()
            self._tcp_reader = threading.Thread(
                target=self._receive_loop, name="lhandpro-tcp485-rx", daemon=True
            )
            self._tcp_reader.start()
            self.sdk_handle.initial_ex(LCN_RS485, self.rs485_node_id)
            return self._common_initialization(
                enable_motors, home_motors, home_wait_time
            )

        def _cleanup_communication_resources(self):
            self._tcp_stop.set()
            sock = self.tcp_socket
            self.tcp_socket = None
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
            reader = self._tcp_reader
            self._tcp_reader = None
            if reader is not None and reader is not threading.current_thread():
                reader.join(timeout=1.0)

    return TcpRS485Controller


class _SdoMotorAdapter:
    """Send motor commands through short SDO frames; keep SDK feedback APIs."""

    def __init__(self, sdk, send_frame, node_id, dof, direction_bits=0x20):
        self._sdk = sdk
        self._send_frame = send_frame
        self._node_id = int(node_id) & 0xFF
        self._dof = int(dof)
        self._direction_bits = int(direction_bits) & 0x60
        self._modes = {axis: self._direction_bits for axis in range(1, self._dof + 1)}

    def __getattr__(self, name):
        return getattr(self._sdk, name)

    def _axes(self, axis):
        axis = int(axis)
        if axis == 0:
            return range(1, self._dof + 1)
        if not 1 <= axis <= self._dof:
            raise ValueError(f"轴编号必须在 0..{self._dof}: {axis}")
        return (axis,)

    def _write(self, index, axis, value, label):
        value = int(value) & 0xFFFFFFFF
        for item in self._axes(axis):
            # The supplied RS485 protocol table defines 0x23 as the write
            # command (four data bytes). The SDK emits 0x2B for small values,
            # which this hand firmware silently ignores.
            body = bytes((
                0x60,
                self._node_id,
                0x23,
                int(index) & 0xFF,
                (int(index) >> 8) & 0xFF,
                int(item) & 0xFF,
            )) + value.to_bytes(4, "little")
            crc = _crc16_modbus(body)
            if not self._send_frame(body + bytes((crc & 0xFF, crc >> 8))):
                raise RuntimeError(
                    f"SDO {label} F{index & 0xFFF:03X}:{item:02X} 发送失败"
                )

    def _control(self, axis, control, mode=None):
        for item in self._axes(axis):
            selected_mode = self._modes[item] if mode is None else int(mode)
            # SDO serializes a U16 low byte first. Pack it so its wire bytes
            # remain [mode, control], matching the protocol's PDO definition.
            value = ((int(control) & 0xFF) << 8) | (selected_mode & 0xFF)
            self._write(SDO_MODE_CONTROL, item, value, "模式/控制")

    def set_control_mode(self, axis, mode):
        for item in self._axes(axis):
            self._modes[item] = self._direction_bits | (int(mode) & 0x07)

    def set_enable(self, axis, enabled):
        self._control(axis, 1 if enabled else 2)

    def set_clear_alarm(self, axis):
        self._control(axis, 3)

    def set_target_position(self, axis, position):
        self._write(SDO_POSITION, axis, position, "位置")

    def set_position_velocity(self, axis, velocity):
        self._write(SDO_VELOCITY, axis, velocity, "速度")

    def set_max_current(self, axis, current):
        self._write(SDO_CURRENT_LIMIT, axis, current, "电流限制")

    def move_motors(self, axis):
        self._control(axis, 5)

    def home_motors(self, axis):
        self._control(axis, 4, self._direction_bits | 5)

    def stop_motors(self, axis):
        self._control(axis, 0)

    def set_home_current(self, current):
        # No corresponding runtime object is defined in the supplied table.
        del current


class _CompactMotorAdapter:
    """Motor commands using the firmware's compact six-axis custom frames."""

    def __init__(self, sdk, send_frame, node_id, dof, direction_bits=0x20):
        self._sdk = sdk
        self._send_frame = send_frame
        self._node_id = int(node_id) & 0xFF
        self._dof = int(dof)
        self._direction_bits = int(direction_bits) & 0x60
        self._modes = [self._direction_bits] * self._dof
        self._controls = [0] * self._dof
        self._currents = [600] * self._dof
        self._velocities = [12000] * self._dof
        self._positions = [0] * self._dof
        self._sent_currents = None
        self._sent_velocities = None

    def __getattr__(self, name):
        return getattr(self._sdk, name)

    def _axes(self, axis):
        axis = int(axis)
        if axis == 0:
            return range(self._dof)
        if not 1 <= axis <= self._dof:
            raise ValueError(f"轴编号必须在 0..{self._dof}: {axis}")
        return (axis - 1,)

    @staticmethod
    def _u16(value):
        return (int(value) & 0xFFFF).to_bytes(2, "little")

    def _send(self, frame_type, payload, label):
        body = bytes((0x50, self._node_id, int(frame_type), self._dof)) + bytes(payload)
        crc = _crc16_modbus(body)
        frame = body + bytes((crc & 0xFF, crc >> 8))
        if not self._send_frame(frame):
            raise RuntimeError(f"短帧{label}发送失败: type=0x{frame_type:02X}")

    def _send_control(self):
        payload = bytearray()
        for mode, control in zip(self._modes, self._controls):
            payload.extend((mode & 0xFF, control & 0xFF))
        self._send(0x81, payload, "模式/控制")

    def set_control_mode(self, axis, mode):
        for index in self._axes(axis):
            self._modes[index] = self._direction_bits | (int(mode) & 0x07)

    def set_enable(self, axis, enabled):
        for index in self._axes(axis):
            self._controls[index] = 1 if enabled else 2
        self._send_control()

    def set_clear_alarm(self, axis):
        for index in self._axes(axis):
            self._controls[index] = 3
        self._send_control()

    def set_target_position(self, axis, position):
        for index in self._axes(axis):
            self._positions[index] = int(position)

    def set_position_velocity(self, axis, velocity):
        for index in self._axes(axis):
            self._velocities[index] = int(velocity)

    def set_max_current(self, axis, current):
        for index in self._axes(axis):
            self._currents[index] = int(current)

    def force_velocity_broadcast(self):
        """Send the current six-axis velocity vector on the next motion."""
        self._sent_velocities = None

    def move_motors(self, axis):
        del axis
        currents = tuple(self._currents)
        velocities = tuple(self._velocities)
        if currents != self._sent_currents:
            self._send(0x84, b"".join(self._u16(v) for v in currents), "电流")
            self._sent_currents = currents
        if velocities != self._sent_velocities:
            self._send(0x83, b"".join(self._u16(v) for v in velocities), "速度")
            self._sent_velocities = velocities
        self._send(0x82, b"".join(self._u16(v) for v in self._positions), "位置")

    def home_motors(self, axis):
        for index in self._axes(axis):
            self._modes[index] = self._direction_bits | 5
            self._controls[index] = 4
        self._send_control()

    def stop_motors(self, axis):
        for index in self._axes(axis):
            self._controls[index] = 0
        self._send_control()

    def set_home_current(self, current):
        del current


class DobotTcpGripper(LHandProGripper):
    """Use the vendor grasp API while Nova transports its raw RS485 frames."""

    def __init__(
        self,
        arm=None,
        robot_ip="192.168.5.1",
        robot_port=DEFAULT_DASHBOARD_PORT,
        transparent_port=DEFAULT_TRANSPARENT_PORT,
        baud_rate=None,
        node_id=None,
        profile_path=None,
        debug=False,
        tcp_debug=False,
        control_transport=None,
        controller_factory=None,
    ):
        baud_rate = int(baud_rate or os.getenv("LHAND_DOBOT_BAUD", "115200"))
        self.arm = arm
        self.robot_ip = robot_ip
        self.robot_port = int(robot_port)
        self.transparent_port = int(transparent_port)
        self.tcp_debug = bool(tcp_debug)
        self.control_transport = (
            control_transport or os.getenv("LHAND_DOBOT_CONTROL", "compact")
        ).strip().lower()
        self._sdo_adapter = None
        self._custom_controller_factory = controller_factory
        super().__init__(
            port=f"tcp://{robot_ip}:{self.transparent_port}",
            baud_rate=baud_rate,
            node_id=node_id,
            profile_path=profile_path,
            controller_factory=None,
            debug=debug,
        )

    def _load_controller_factory(self):
        if self._custom_controller_factory is not None:
            return self._custom_controller_factory
        sdk_root = os.path.abspath(os.path.expanduser(
            os.getenv("LHAND_SDK_ROOT", DEFAULT_SDK_ROOT)
        ))
        python_dir = os.path.join(
            sdk_root, "share", "LHandProLib", "examples", "RS485_python"
        )
        library_path = os.path.join(sdk_root, "lib", "libLHandProLib.so")
        if not os.path.isfile(library_path):
            raise FileNotFoundError(f"找不到 LHandPro 动态库: {library_path}")
        if python_dir not in sys.path:
            sys.path.insert(0, python_dir)
        return _build_tcp_controller(
            self.robot_ip, self.transparent_port, debug=self.tcp_debug
        )

    def _configure_robot_port(self):
        dashboard = self.arm
        owns_dashboard = dashboard is None
        if owns_dashboard:
            dashboard = _DashboardClient(self.robot_ip, self.robot_port)
            dashboard.connect()
        try:
            power_cycle = os.getenv("LHAND_DOBOT_POWER_CYCLE", "0").lower() in (
                "1", "true", "yes", "on"
            )
            if power_cycle:
                print("[LHandPro/TCP485] 重置末端链路：工具电源断电后重新上电")
                _dashboard_call(dashboard, "SetToolPower(0)")
                time.sleep(max(0.2, float(os.getenv("LHAND_DOBOT_POWER_OFF_WAIT_S", "0.5"))))
            _dashboard_call(dashboard, "SetToolMode(1,0,1)")
            _dashboard_call(dashboard, f'SetTool485({self.baud_rate},"N",1,1)')
            _dashboard_call(dashboard, "SetToolPower(1)")
            time.sleep(max(0.1, float(os.getenv("LHAND_DOBOT_POWER_ON_WAIT_S", "1.0"))))
        finally:
            if owns_dashboard:
                dashboard.close()

    @property
    def _sdk(self):
        if self._sdo_adapter is not None:
            return self._sdo_adapter
        return super()._sdk

    def connect(self, enable_motors=None, home_motors=None, home_timeout_s=None):
        if self.controller is not None:
            return True
        self._configure_robot_port()
        enable = (
            os.getenv("LHAND_ENABLE_ON_CONNECT", "1").lower() in ("1", "true", "yes", "on")
            if enable_motors is None else bool(enable_motors)
        )
        home = (
            os.getenv("LHAND_HOME_ON_CONNECT", "1").lower() in ("1", "true", "yes", "on")
            if home_motors is None else bool(home_motors)
        )
        try:
            super().connect(enable_motors=False, home_motors=False)
            if getattr(self.controller, "_tcp_rx_frame_count", 0) <= 0:
                raise RuntimeError(
                    "Nova 60000 连接已建立，但未收到任何有效手端RS485帧。"
                    f"请确认手端波特率与当前{self.baud_rate}一致，检查末端供电、A/B/通信地和"
                    "机械臂60000透传服务；不允许使能或回零。"
                )
            control_transport = self.control_transport
            if control_transport not in ("compact", "sdo", "pdo"):
                raise ValueError("LHAND_DOBOT_CONTROL 只能是 compact、sdo 或 pdo")
            direction_bits = int(os.getenv("LHAND_SDO_DIRECTION_BITS", "0x20"), 0)
            if control_transport == "compact":
                self._sdo_adapter = _CompactMotorAdapter(
                    self.controller.sdk_handle,
                    self.controller.send_rs485_frame,
                    self.node_id,
                    self.dof_active,
                    direction_bits,
                )
                print("[LHandPro/TCP485] 电机控制使用厂家短帧协议")
            if control_transport == "sdo":
                self._sdo_adapter = _SdoMotorAdapter(
                    self.controller.sdk_handle,
                    self.controller.send_rs485_frame,
                    self.node_id,
                    self.dof_active,
                    direction_bits,
                )
                print("[LHandPro/TCP485] 电机控制使用短 SDO 帧（状态读取仍使用 SDK）")
            if enable:
                if control_transport == "compact":
                    self._compact_enable(after_home=False)
                else:
                    self._enable_motors()
            if home:
                self.home(
                    timeout_s=home_timeout_s,
                    clear_alarm=(control_transport != "compact"),
                )
                if control_transport == "compact":
                    self._compact_enable(after_home=True)
            return True
        except Exception:
            self._sdo_adapter = None
            if self.controller is not None:
                super().disconnect(stop_motors=False)
            raise

    def _compact_enable(self, after_home):
        sdk = self._sdk
        sdk.set_control_mode(0, 0)
        sdk.set_enable(0, True)
        timeout_s = max(2.0, float(os.getenv("LHAND_ENABLE_TIMEOUT_S", "10")))
        deadline = time.monotonic() + timeout_s
        last_snapshot = []
        while time.monotonic() < deadline:
            time.sleep(0.15)
            last_snapshot = self._motor_snapshot()
            if last_snapshot and all(item["enable"] for item in last_snapshot):
                label = "回零后六轴重新使能完成" if after_home else "六轴使能完成"
                print(f"[LHandPro] {label}")
                return
        raise TimeoutError(
            ("回零后等待六轴重新使能超时: " if after_home else "等待六轴使能超时: ")
            + self._snapshot_label(last_snapshot)
        )

    def disconnect(self, stop_motors=True):
        if stop_motors and self._sdo_adapter is not None and self.controller is not None:
            try:
                self._sdo_adapter.stop_motors(0)
            except Exception as exc:
                print(f"[LHandPro/TCP485][警告] SDO 停止失败: {exc}")
        self._sdo_adapter = None
        return super().disconnect(stop_motors=False)
