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
            self._tcp_send_lock = threading.Lock()
            self._tcp_debug = bool(debug)

        def _rs485_send_callback(self, data):
            sock = self.tcp_socket
            if sock is None or not self.is_connected:
                return False
            try:
                payload = bytes(data)
                if self._tcp_debug:
                    print(f"[LHandPro/TCP485][TX] {payload.hex(' ')}")
                with self._tcp_send_lock:
                    sock.sendall(payload)
                return True
            except OSError as exc:
                print(f"[LHandPro/TCP485] 发送失败: {exc}")
                return False

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
                if self.sdk_handle and self.is_connected:
                    try:
                        self.sdk_handle.set_rs485_data_decode(data)
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
        controller_factory=None,
    ):
        baud_rate = int(baud_rate or os.getenv("LHAND_DOBOT_BAUD", "500000"))
        self.arm = arm
        self.robot_ip = robot_ip
        self.robot_port = int(robot_port)
        self.transparent_port = int(transparent_port)
        self.tcp_debug = bool(tcp_debug)
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
            _dashboard_call(dashboard, "SetToolPower(1)")
            _dashboard_call(dashboard, "SetToolMode(1,0,1)")
            _dashboard_call(dashboard, f'SetTool485({self.baud_rate},"N",1,1)')
            time.sleep(0.1)
        finally:
            if owns_dashboard:
                dashboard.close()

    def connect(self, enable_motors=None, home_motors=None, home_timeout_s=None):
        if self.controller is not None:
            return True
        self._configure_robot_port()
        return super().connect(
            enable_motors=enable_motors,
            home_motors=home_motors,
            home_timeout_s=home_timeout_s,
        )
