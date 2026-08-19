#!/usr/bin/env python3
"""LHandPro RS485 gripper backend with current-profile flexible grasping."""

import csv
import os
import sys
import threading
import time
from ctypes import c_int


DIPAN_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SDK_ROOT = os.path.abspath(os.path.join(DIPAN_DIR, "..", "LHandProLib-API-Linux"))
DEFAULT_PROFILE_PATH = os.path.join(DIPAN_DIR, "runtime", "lhandpro_empty.csv")


def _parse_int_set(raw, default, min_value, max_value):
    text = str(raw if raw not in (None, "") else default)
    values = {int(item.strip()) for item in text.split(",") if item.strip()}
    invalid = sorted(value for value in values if value < min_value or value > max_value)
    if invalid:
        raise ValueError(f"轴编号必须在 {min_value}..{max_value}, 当前无效: {invalid}")
    return values


def _parse_int_vector(raw, count, default):
    text = str(raw or "").strip()
    if not text:
        return [int(default)] * count
    values = [int(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) == 1:
        return values * count
    if len(values) != count:
        raise ValueError(f"期望 {count} 个逗号分隔位置值, 实际 {len(values)}")
    return values


def load_empty_profile(path, dof=6):
    """Load (command, absolute-current-vector) rows from a calibration CSV."""
    rows = []
    with open(path, newline="", encoding="utf-8") as profile_file:
        reader = csv.DictReader(profile_file)
        for row in reader:
            try:
                command = int(float(row["command"]))
                currents = [abs(float(row[f"current_{axis}"])) for axis in range(1, dof + 1)]
            except (KeyError, TypeError, ValueError):
                continue
            rows.append((command, currents))
    rows.sort(key=lambda item: item[0])
    if not rows:
        raise ValueError(f"空载曲线没有有效数据: {path}")
    return rows


def profile_current_envelope(profile, command, axis, window):
    """Return the conservative empty current near one axis command."""
    if not profile:
        raise ValueError("empty profile is required")
    axis_index = int(axis) - 1
    nearby = [
        currents[axis_index]
        for row_command, currents in profile
        if abs(float(row_command) - float(command)) <= abs(float(window))
    ]
    if nearby:
        return max(nearby)
    _, currents = min(profile, key=lambda item: abs(float(item[0]) - float(command)))
    return currents[axis_index]


class LHandProGripper:
    """Expose the shelf workflow's gripper API on the LHandPro RS485 SDK."""

    def __init__(
        self,
        port=None,
        baud_rate=None,
        node_id=None,
        profile_path=None,
        controller_factory=None,
        debug=False,
    ):
        self.port = port or os.getenv("LHAND_RS485_PORT") or None
        self.baud_rate = int(baud_rate or os.getenv("LHAND_RS485_BAUD", "500000"))
        self.node_id = int(node_id or os.getenv("LHAND_RS485_NODE_ID", "1"))
        self.profile_path = os.path.abspath(os.path.expanduser(
            profile_path or os.getenv("LHAND_FLEX_PROFILE", DEFAULT_PROFILE_PATH)
        ))
        self._controller_factory = controller_factory
        self.debug = bool(debug)
        self.controller = None
        self.dof_active = 0
        self.ok = True
        self._holding = False
        # Position broadcasts and feedback requests share one half-duplex
        # TCP-to-RS485 bus. An RLock lets open() reserve the bus for its whole
        # release window while _command_positions keeps its normal locking.
        self._command_lock = threading.RLock()
        self._hold_lock = threading.Lock()
        self._feedback_paused = threading.Event()
        self._hold_stop = threading.Event()
        self._hold_thread = None
        self._hold_positions = None
        self._hold_axes = set()
        self._hold_max_positions = None
        self._hold_profile = None
        self._hold_baseline = None
        self._last_command_positions = None
        self._latency_debug = os.getenv("LHAND_LATENCY_DEBUG", "0").lower() in (
            "1", "true", "yes", "on"
        )

    def _load_controller_factory(self):
        if self._controller_factory is not None:
            return self._controller_factory
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
        from lhandprolib_python_sdk.controller import LHandProController
        return LHandProController

    def connect(self, enable_motors=None, home_motors=None, home_timeout_s=None):
        if self.controller is not None:
            return True
        enable = (
            os.getenv("LHAND_ENABLE_ON_CONNECT", "1").lower() in ("1", "true", "yes", "on")
            if enable_motors is None else bool(enable_motors)
        )
        home = (
            os.getenv("LHAND_HOME_ON_CONNECT", "1").lower() in ("1", "true", "yes", "on")
            if home_motors is None else bool(home_motors)
        )
        try:
            factory = self._load_controller_factory()
            controller = factory(
                communication_mode="RS485",
                rs485_baud_rate=self.baud_rate,
                rs485_node_id=self.node_id,
            )
            connected = controller.connect(
                # The vendor high-level controller applies hand_type after its
                # built-in homing step. Configure first, then home below so the
                # SDK's homed state is not reset before the first move.
                enable_motors=False,
                home_motors=False,
                home_wait_time=0.0,
                rs485_port_name=self.port,
                rs485_baud_rate=self.baud_rate,
                rs485_node_id=self.node_id,
            )
            if not connected:
                raise RuntimeError("SDK connect() 返回失败")
            self.controller = controller
            _, self.dof_active = controller.get_dof()
            if self.dof_active <= 0:
                raise RuntimeError(f"SDK 返回无效主动轴数: {self.dof_active}")
            print(
                f"[LHandPro] 已连接: port={self.port or 'auto'} "
                f"baud={self.baud_rate} node={self.node_id} dof={self.dof_active}"
            )
            if self.debug:
                self._sdk.log_clear()
                self._sdk.log_on(True, 20000)
            if enable:
                self._enable_motors()
            if home:
                self.home(timeout_s=home_timeout_s)
            return True
        except Exception as exc:
            self._save_debug_log()
            self.ok = False
            if self.controller is not None:
                try:
                    self.controller.disconnect()
                except Exception:
                    pass
            self.controller = None
            print(f"[LHandPro] 连接失败: {exc}")
            raise

    def _save_debug_log(self):
        if not self.debug or self.controller is None or not self.controller.sdk_handle:
            return None
        path = os.path.abspath(os.path.expanduser(
            os.getenv("LHAND_DEBUG_LOG", "/tmp/lhandpro_rs485_debug.log")
        ))
        try:
            self.controller.sdk_handle.log_save(path)
            print(f"[LHandPro][调试] SDK 通信日志已保存: {path}")
            return path
        except Exception as exc:
            print(f"[LHandPro][调试] SDK 日志保存失败: {exc}")
            return None

    @property
    def _sdk(self):
        if not self.ok or self.controller is None or not self.controller.sdk_handle:
            raise RuntimeError("LHandPro 尚未连接")
        return self.controller.sdk_handle

    def _motor_snapshot(self):
        started_at = time.perf_counter()
        sdk = self._sdk
        snapshot = []
        for axis in range(1, self.dof_active + 1):
            snapshot.append({
                "axis": axis,
                "enable": bool(sdk.get_enable(axis)),
                "status": int(sdk.get_now_status(axis)),
                "alarm": int(sdk.get_now_alarm(axis)),
                "position": int(sdk.get_now_position(axis)),
                "current": int(sdk.get_now_current(axis)),
            })
        return snapshot

    @staticmethod
    def _snapshot_label(snapshot):
        return " | ".join(
            f"A{item['axis']}:en={int(item['enable'])} st={item['status']} "
            f"alm={item['alarm']} pos={item['position']} I={item['current']}‰"
            for item in snapshot
        )

    def _enable_motors(self):
        sdk = self._sdk
        # The vendor initialization example explicitly selects position mode
        # before enabling.  Some firmware acknowledges RS485 traffic but
        # ignores the enable edge while the control mode is still unset.
        if hasattr(sdk, "set_control_mode"):
            sdk.set_control_mode(0, 0)  # LCM_POSITION
        # The protocol only acts on a changed control command. Force a disable
        # edge first so repeated test runs cannot leave an old enable command
        # latched in the device/SDK state machine.
        sdk.set_enable(0, False)
        time.sleep(0.25)
        sdk.set_clear_alarm(0)
        sdk.set_enable(0, True)
        timeout_s = max(2.0, float(os.getenv("LHAND_ENABLE_TIMEOUT_S", "10")))
        started_at = time.monotonic()
        deadline = time.monotonic() + timeout_s
        last_snapshot = []
        retried_per_axis = False
        last_log_at = 0.0
        while time.monotonic() < deadline:
            time.sleep(0.15)
            last_snapshot = self._motor_snapshot()
            alarms = [item for item in last_snapshot if item["alarm"]]
            if alarms:
                raise RuntimeError(
                    "使能后检测到报警: " + self._snapshot_label(last_snapshot)
                )
            if last_snapshot and all(item["enable"] for item in last_snapshot):
                print("[LHandPro] 六轴使能完成")
                return
            elapsed = time.monotonic() - started_at
            if self.debug and (not last_log_at or elapsed - last_log_at >= 1.0):
                print(f"[LHandPro][使能 {elapsed:.1f}s] {self._snapshot_label(last_snapshot)}")
                last_log_at = elapsed
            if elapsed >= 2.0 and not retried_per_axis:
                disabled_axes = [item["axis"] for item in last_snapshot if not item["enable"]]
                print(f"[LHandPro][使能恢复] 广播未生效，逐轴重发: {disabled_axes}")
                for axis in disabled_axes:
                    sdk.set_enable(axis, False)
                    time.sleep(0.05)
                    sdk.set_enable(axis, True)
                retried_per_axis = True
        if str(self.port or "").startswith("tcp://"):
            power_hint = (
                "当前是机械臂末端透传：通信已正常，但请重点核对手端 VDC/GND 和电机电源。"
                "Nova 末端 IO 额定 24V、总最大约 2A（单通道约 0.5A），DH116 类灵巧手"
                "规格约需 12~48V、2.5A；若电压在使能瞬间下陷，需要按手册增加独立电源，"
                "不能把两路 24V 直接并联。"
            )
        else:
            power_hint = (
                "请检查转接盒 DC 主电源、手端 VDC/GND、急停/使能开关和电源电流能力；"
                "USB-RS485 本身通常只保证通信。"
            )
        raise TimeoutError(
            f"等待六轴使能超时 {timeout_s:.1f}s: {self._snapshot_label(last_snapshot)}。"
            "通信配置可读但驱动未使能。" + power_hint
        )

    def home(self, timeout_s=None, clear_alarm=True):
        """Home all axes after SDK hand type/configuration has been applied."""
        timeout_s = max(
            3.0,
            float(timeout_s or os.getenv("LHAND_HOME_TIMEOUT_S", "45")),
        )
        poll_s = max(0.1, float(os.getenv("LHAND_HOME_POLL_S", "0.25")))
        sdk = self._sdk
        home_current = os.getenv("LHAND_HOME_CURRENT")
        if home_current not in (None, ""):
            sdk.set_home_current(int(home_current))
        if not all(item["enable"] for item in self._motor_snapshot()):
            self._enable_motors()
        if clear_alarm:
            sdk.set_clear_alarm(0)
        print(f"[LHandPro] 开始六轴回零，超时 {timeout_s:.0f}s")
        sdk.home_motors(0)

        started_at = time.monotonic()
        deadline = started_at + timeout_s
        saw_homing = False
        stable_count = 0
        previous_positions = None
        last_log_at = 0.0
        last_snapshot = []
        while time.monotonic() < deadline:
            time.sleep(poll_s)
            last_snapshot = self._motor_snapshot()
            alarms = [item for item in last_snapshot if item["alarm"]]
            if alarms:
                raise RuntimeError(
                    "回零期间检测到报警: " + self._snapshot_label(last_snapshot)
                )
            statuses = [item["status"] for item in last_snapshot]
            positions = tuple(item["position"] for item in last_snapshot)
            if any(status == 7 for status in statuses):
                saw_homing = True
                stable_count = 0
            elif previous_positions == positions:
                stable_count += 1
            else:
                stable_count = 0
            previous_positions = positions

            elapsed = time.monotonic() - started_at
            if self.debug and (not last_log_at or elapsed - last_log_at >= 1.0):
                print(f"[LHandPro][回零 {elapsed:.1f}s] {self._snapshot_label(last_snapshot)}")
                last_log_at = elapsed

            # Some firmware exposes LST_HOMING=7, while some only updates the
            # final stopped state. Accept stable, alarm-free axes after a
            # minimum settling interval in either case; move_motors performs
            # the SDK's definitive homed-state check afterward.
            minimum_elapsed = (
                0.8
                if saw_homing
                else max(3.0, float(os.getenv("LHAND_HOME_MIN_WAIT_S", "8")))
            )
            if elapsed >= minimum_elapsed and stable_count >= 3 and all(status != 7 for status in statuses):
                print(
                    f"[LHandPro] 回零流程已停止: elapsed={elapsed:.1f}s "
                    f"saw_homing={int(saw_homing)} positions={list(positions)}"
                )
                return True

        raise TimeoutError(
            f"六轴回零超时 {timeout_s:.1f}s: {self._snapshot_label(last_snapshot)}"
        )

    def get_rs485_baudrate(self):
        """Read the baud rate stored in the hand through the vendor SDK."""
        sdk = self._sdk
        if not hasattr(sdk, "_get_scalar"):
            raise RuntimeError("当前 Python SDK 封装不支持读取 RS485 波特率")
        return int(sdk._get_scalar(
            "get_rs485_baudrate", c_int, "读取RS485波特率"
        ))

    def set_rs485_baudrate(self, baud_rate):
        """Persist a new hand-side baud rate without enabling any motor."""
        baud_rate = int(baud_rate)
        if baud_rate <= 0:
            raise ValueError("RS485 波特率必须大于 0")
        sdk = self._sdk
        if not hasattr(sdk, "_call_checked"):
            raise RuntimeError("当前 Python SDK 封装不支持设置 RS485 波特率")
        sdk._call_checked("set_rs485_baudrate", "设置RS485波特率", baud_rate)
        library = getattr(sdk, "_lib", None)
        if library is not None and hasattr(library, "lhandprolib_save_sdo_drive_param"):
            sdk._call_checked("save_sdo_drive_param", "保存RS485波特率")
        self.baud_rate = baud_rate
        return baud_rate

    def disconnect(self, stop_motors=True):
        self._stop_hold()
        self._holding = False
        controller = self.controller
        self.controller = None
        if controller is None:
            return
        try:
            if stop_motors:
                controller.stop_motors()
        except Exception as exc:
            print(f"[LHandPro][警告] 停止电机失败: {exc}")
        finally:
            controller.disconnect()
        print("[LHandPro] 已断开")

    def _open_positions(self):
        raw = os.getenv("LHAND_OPEN_POSITIONS")
        if raw not in (None, ""):
            return _parse_int_vector(raw, self.dof_active, 0)
        # DH116 axis 1 is thumb abduction/side-swing. Its tested 9000 endpoint
        # is the lowest open pose; axes 2..6 remain fully open at zero.
        default = [9000] + [0] * max(0, self.dof_active - 1)
        return default[:self.dof_active]

    def _max_positions(self):
        return _parse_int_vector(
            os.getenv("LHAND_FLEX_MAX_POSITIONS"), self.dof_active, 9000
        )

    def _command_positions(
        self,
        positions,
        wait_s=0.0,
        velocity=None,
        max_current=None,
        velocities=None,
    ):
        if len(positions) != self.dof_active:
            raise ValueError(f"位置数量应为 {self.dof_active}, 实际 {len(positions)}")
        if velocities is not None and len(velocities) != self.dof_active:
            raise ValueError(f"速度数量应为 {self.dof_active}, 实际 {len(velocities)}")
        velocity = int(velocity or os.getenv("LHAND_FLEX_VELOCITY", "12000"))
        max_current = int(max_current or os.getenv("LHAND_FLEX_MAX_CURRENT", "600"))
        sdk = self._sdk
        started_at = time.perf_counter()
        with self._command_lock:
            for axis, position in enumerate(positions, start=1):
                sdk.set_target_position(axis, int(position))
                axis_velocity = velocity if velocities is None else int(velocities[axis - 1])
                sdk.set_position_velocity(axis, axis_velocity)
                sdk.set_max_current(axis, max_current)
            sdk.move_motors(0)
            self._last_command_positions = [int(position) for position in positions]
        if self._latency_debug:
            print(
                f"[LHandPro][时延] 广播 positions={list(positions)} "
                f"sdk_call={(time.perf_counter()-started_at)*1000:.2f}ms "
                f"wait={float(wait_s)*1000:.2f}ms"
            )
        if wait_s > 0:
            time.sleep(float(wait_s))

    def _command_open_positions(self, positions, wait_s=0.0):
        """Release all axes with the same continuous position command."""
        # Match continuous grasp: all axes receive one common speed and one
        # shared position frame. Per-axis velocity matching looked synchronous
        # on paper, but real motor load/acceleration differences made fingers
        # visibly start at different times.
        velocity = max(1, int(os.getenv(
            "LHAND_OPEN_VELOCITY", os.getenv("LHAND_FLEX_VELOCITY", "12000")
        )))
        previous = list(self._last_command_positions or [0] * self.dof_active)
        if len(previous) != self.dof_active:
            previous = [0] * self.dof_active
        longest_travel_s = max(
            (abs(int(target) - int(current)) / float(velocity)
             for current, target in zip(previous, positions)),
            default=0.0,
        )
        minimum_wait_s = max(
            1.0,
            longest_travel_s + max(0.0, float(os.getenv("LHAND_OPEN_SETTLE_MARGIN_S", "0.20"))),
        )
        effective_wait_s = max(float(wait_s), minimum_wait_s)
        print(
            f"[LHandPro] 同步张开: 目标={list(positions)} 统一速度={velocity} "
            f"等待={effective_wait_s:.2f}s (单次全轴连续广播)"
        )
        self._command_positions(positions, wait_s=effective_wait_s, velocity=velocity)

    def _read_currents(self, samples=1, delay_s=0.0):
        rows = []
        for sample_index in range(max(1, int(samples))):
            # Never interleave a long status response with a position frame.
            with self._command_lock:
                rows.append([
                    float(self._sdk.get_now_current(axis))
                    for axis in range(1, self.dof_active + 1)
                ])
            if sample_index + 1 < max(1, int(samples)) and delay_s > 0:
                time.sleep(float(delay_s))
        return [sum(row[index] for row in rows) / len(rows) for index in range(self.dof_active)]

    def open(self):
        if not self.ok:
            print("[LHandPro] 已禁用, 跳过张开")
            return False
        try:
            # Stop new hold feedback before reserving the bus for release.
            self._feedback_paused.set()
            self._stop_hold()
            self._holding = False
            positions = self._open_positions()
            settle_s = max(0.0, float(os.getenv("LHAND_OPEN_WAIT_S", "1.0")))
            verify = os.getenv("LHAND_OPEN_VERIFY", "0").lower() in (
                "1", "true", "yes", "on"
            )
            if not verify:
                transport = self.controller
                reserve = getattr(transport, "begin_motion_exclusive", None)
                release = getattr(transport, "end_motion_exclusive", None)
                if callable(reserve):
                    reserve("张开")
                try:
                    with self._command_lock:
                        self._command_open_positions(positions, wait_s=settle_s)
                finally:
                    if callable(release):
                        release("张开")
                print("[LHandPro] 张开 (单次全轴连续广播)")
                return True

            self._feedback_paused.clear()

            timeout_s = max(
                settle_s,
                float(os.getenv("LHAND_OPEN_TIMEOUT_S", "4.0")),
            )
            poll_s = max(0.02, float(os.getenv("LHAND_OPEN_POLL_S", "0.05")))
            retries = max(1, int(os.getenv("LHAND_OPEN_RETRIES", "2")))
            tolerance = max(0, int(os.getenv("LHAND_OPEN_POSITION_TOLERANCE", "300")))
            for attempt in range(retries):
                try:
                    # One broadcast starts all axes; do not wait a fixed short
                    # interval and mistake different motor travel distances for
                    # a release failure.
                    self._command_open_positions(positions, wait_s=0.0)
                except Exception as exc:
                    if getattr(exc, "error_code", None) != 11:
                        raise
                    print("[LHandPro][恢复] SDK 仍报告未回零，重新执行回零后再试一次")
                    self.home()
                    self._command_open_positions(positions, wait_s=0.0)

                deadline = time.monotonic() + timeout_s
                last_actual = None
                while time.monotonic() < deadline:
                    try:
                        last_actual = [
                            int(self._sdk.get_now_position(axis))
                            for axis in range(1, self.dof_active + 1)
                        ]
                    except Exception:
                        last_actual = None
                    if last_actual is not None and all(
                        abs(last_actual[index] - int(target)) <= tolerance
                        for index, target in enumerate(positions)
                    ):
                        print("[LHandPro] 张开")
                        return True
                    time.sleep(poll_s)

                if attempt + 1 < retries:
                    print(
                        f"[LHandPro][张开] 仍在运动，重新广播张开 "
                        f"({attempt + 1}/{retries}) 当前={last_actual}"
                    )
            print(f"[LHandPro][张开失败] 超过 {timeout_s:.1f}s 仍未全部到位, 当前={last_actual}")
            return False
        except Exception as exc:
            self.ok = False
            print(f"[LHandPro] 张开失败, 已禁用灵巧手: {exc}")
            return False

    def close(self, top=False, mode=None):
        del top, mode
        if not self.ok:
            return False
        try:
            self._feedback_paused.clear()
            positions = _parse_int_vector(
                os.getenv("LHAND_GRASP_POSITIONS"), self.dof_active, 7500
            )
            self._command_positions(
                positions,
                wait_s=float(os.getenv("LHAND_CLOSE_WAIT_S", "1.0")),
            )
            self._holding = True
            self._start_hold(positions, set(range(1, self.dof_active + 1)), positions, None, None)
            print(f"[LHandPro] 固定位置抓取: {positions}")
            return True
        except Exception as exc:
            self.ok = False
            print(f"[LHandPro] 抓取失败, 已禁用灵巧手: {exc}")
            return False

    def is_holding(self):
        return bool(
            self.ok
            and self._holding
            and self.controller is not None
            and self.controller.is_connected
        )

    def _stop_hold(self):
        stop_event = self._hold_stop
        stop_event.set()
        thread = self._hold_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            # A late tightening broadcast can overwrite the first release
            # command. Wait until the hold loop has fully exited.
            thread.join()
        self._hold_thread = None
        self._hold_positions = None
        self._hold_profile = None
        self._hold_baseline = None
        self._hold_axes = set()
        self._hold_max_positions = None
        self._hold_stop = threading.Event()

    def _start_hold(self, positions, axes, max_positions, profile, baseline):
        self._stop_hold()
        with self._hold_lock:
            self._hold_positions = list(positions)
            self._hold_axes = set(axes)
            self._hold_max_positions = list(max_positions)
            self._hold_profile = profile
            self._hold_baseline = list(baseline) if baseline is not None else None
        stop_event = self._hold_stop
        self._hold_thread = threading.Thread(
            target=self._hold_loop,
            args=(stop_event,),
            name="lhandpro-grasp-hold",
            daemon=True,
        )
        self._hold_thread.start()
        hz = max(1.0, float(os.getenv("LHAND_HOLD_HZ", "5")))
        resend_interval = max(0.0, float(os.getenv("LHAND_HOLD_RESEND_INTERVAL_S", "1.0")))
        print(
            f"[LHandPro] 持续保持抓取姿态 (检测 {hz:.0f}Hz, "
            f"低负载自动补紧, 静态目标重发 {resend_interval:.1f}s)"
        )

    def _hold_loop(self, stop_event):
        # The motor controller holds its last position command continuously.
        # Re-broadcasting an unchanged six-axis target at 20Hz competes with
        # feedback frames on Nova's TCP-to-RS485 bridge and can delay release.
        period = 1.0 / max(1.0, float(os.getenv("LHAND_HOLD_HZ", "5")))
        tighten_interval = max(0.05, float(os.getenv("LHAND_HOLD_TIGHTEN_INTERVAL_S", "0.20")))
        resend_interval = max(0.0, float(os.getenv("LHAND_HOLD_RESEND_INTERVAL_S", "1.0")))
        tighten_step = max(1, int(os.getenv("LHAND_HOLD_TIGHTEN_STEP", "100")))
        min_delta = float(os.getenv("LHAND_HOLD_MIN_DELTA", "100"))
        hard_current = float(os.getenv("LHAND_FLEX_HARD_CURRENT", "750"))
        next_tighten = time.monotonic() + tighten_interval
        last_command_at = time.monotonic()
        while not stop_event.wait(period):
            try:
                if self._feedback_paused.is_set():
                    continue
                with self._hold_lock:
                    positions = list(self._hold_positions or [])
                    axes = set(self._hold_axes)
                    max_positions = list(self._hold_max_positions or [])
                    profile = self._hold_profile
                    baseline = list(self._hold_baseline or [])
                if not positions or self.controller is None:
                    return

                currents = [abs(value) for value in self._read_currents(1, 0.0)]
                changed = False
                if time.monotonic() >= next_tighten:
                    for axis in axes:
                        index = axis - 1
                        command = positions[index]
                        if command >= max_positions[index] or currents[index] >= hard_current:
                            continue
                        if profile:
                            empty = profile_current_envelope(profile, command, axis, 300)
                        else:
                            empty = baseline[index] if index < len(baseline) else 0.0
                        if currents[index] - empty < min_delta:
                            positions[index] = min(max_positions[index], command + tighten_step)
                            changed = True
                    next_tighten = time.monotonic() + tighten_interval

                    if changed:
                        with self._hold_lock:
                            if self._hold_positions is not None:
                                self._hold_positions = positions

                if stop_event.is_set():
                    return
                now = time.monotonic()
                # A changed target must reach the hand immediately. A sparse
                # refresh remains available for firmware that expects a
                # heartbeat, without burying release behind stale holds.
                if changed or (
                    resend_interval > 0 and now - last_command_at >= resend_interval
                ):
                    self._command_positions(positions, wait_s=0.0)
                    last_command_at = time.monotonic()
            except Exception as exc:
                self._holding = False
                print(f"[LHandPro][警告] 抓取保持失败: {exc}")
                return

    def flexible_close(self):
        """Close each active axis until its current rises above the empty curve."""
        if not self.ok:
            print("[LHandPro] 已禁用, 跳过柔性抓取")
            return False
        try:
            self._feedback_paused.clear()
            self._stop_hold()
            require_profile = os.getenv("LHAND_FLEX_REQUIRE_PROFILE", "1").lower() in (
                "1", "true", "yes", "on"
            )
            profile = None
            if os.path.isfile(self.profile_path):
                profile = load_empty_profile(self.profile_path, self.dof_active)
                print(f"[LHandPro] 柔性抓取使用空载曲线: {self.profile_path}")
            elif require_profile:
                print(f"[LHandPro] 柔性抓取拒绝启动: 尚未录制空载曲线 {self.profile_path}")
                return False

            open_positions = self._open_positions()
            max_positions = self._max_positions()
            if profile:
                profile_max = max(command for command, _ in profile)
                if any(position > profile_max for position in max_positions):
                    print(
                        f"[LHandPro] 柔性闭合上限按空载曲线截断到 {profile_max}, "
                        f"原配置={max_positions}"
                    )
                    max_positions = [min(position, profile_max) for position in max_positions]
            active_axes = _parse_int_set(
                os.getenv("LHAND_FLEX_ACTIVE_AXES"),
                ",".join(str(axis) for axis in range(2, self.dof_active + 1)),
                1,
                self.dof_active,
            )
            required_axes = _parse_int_set(
                os.getenv("LHAND_FLEX_REQUIRED_AXES"),
                ",".join(str(axis) for axis in range(2, self.dof_active + 1)),
                1,
                self.dof_active,
            )
            if not required_axes or not required_axes.issubset(active_axes):
                raise ValueError("required axes 必须是 active axes 的非空子集")

            # Use a short, even command cadence. Current feedback and the hard
            # stop remain active at every step, while a larger default ramp
            # avoids dozens of visible start/stop cycles over RS485.
            step = max(1, int(os.getenv("LHAND_FLEX_STEP", "500")))
            settle = max(0.010, float(os.getenv("LHAND_FLEX_SETTLE_S", "0.015")))
            samples = max(1, int(os.getenv("LHAND_FLEX_SAMPLES", "1")))
            sample_delay = max(0.0, float(os.getenv("LHAND_FLEX_SAMPLE_DELAY_S", "0")))
            threshold = float(os.getenv("LHAND_FLEX_PROFILE_DELTA", "80"))
            fallback_delta = float(os.getenv("LHAND_FLEX_BASELINE_DELTA", "120"))
            profile_window = max(0, int(os.getenv("LHAND_FLEX_PROFILE_WINDOW", "300")))
            consecutive = max(1, int(os.getenv("LHAND_FLEX_CONSECUTIVE", "1")))
            extra_close = max(0, int(os.getenv("LHAND_FLEX_EXTRA_CLOSE", "300")))
            min_position = int(os.getenv("LHAND_FLEX_MIN_POSITION", "1000"))
            min_contacts = max(1, min(
                len(required_axes), int(os.getenv("LHAND_FLEX_MIN_CONTACTS", "2"))
            ))
            hard_current = float(os.getenv("LHAND_FLEX_HARD_CURRENT", "750"))
            hard_consecutive = max(1, int(os.getenv("LHAND_FLEX_HARD_CURRENT_CONSECUTIVE", "3")))
            max_steps = max(1, int(os.getenv("LHAND_FLEX_MAX_STEPS", "300")))
            continuous = os.getenv("LHAND_FLEX_CONTINUOUS", "1").lower() in (
                "1", "true", "yes", "on"
            )

            values = list(open_positions)
            self._holding = False
            self._command_positions(values, wait_s=float(os.getenv("LHAND_FLEX_START_WAIT_S", "0.03")))
            baseline = [abs(value) for value in self._read_currents(samples, sample_delay)]
            hits = {axis: 0 for axis in required_axes}
            locked = {}
            contacts = set()
            at_max = set()
            stall_counts = {axis: 0 for axis in required_axes}
            hard_counts = {axis: 0 for axis in active_axes}
            previous_actual = None
            print(
                f"[LHandPro] 柔性抓取开始: active={sorted(active_axes)} "
                f"required={sorted(required_axes)} "
                f"mode={'continuous' if continuous else 'step'} "
                f"step={step} settle={settle:.3f}s "
                f"samples={samples} threshold={threshold:.0f}"
            )

            def accept_firm_grip(actual, currents, reason):
                confirm_delta = float(os.getenv("LHAND_FLEX_TIMEOUT_ACCEPT_DELTA", "50"))
                stall_polls = max(1, int(os.getenv("LHAND_FLEX_STALL_POLLS", "5")))
                firm_axes = set(contacts)
                for axis in required_axes:
                    index = axis - 1
                    command = actual[index]
                    if command < min_position:
                        continue
                    empty_current = (
                        profile_current_envelope(profile, command, axis, profile_window)
                        if profile
                        else baseline[index]
                    )
                    if currents[index] - empty_current >= confirm_delta:
                        firm_axes.add(axis)
                    if stall_counts.get(axis, 0) >= stall_polls:
                        firm_axes.add(axis)
                if len(firm_axes) < min_contacts:
                    return False

                hold_values = list(actual)
                for axis in firm_axes:
                    index = axis - 1
                    hold_values[index] = min(
                        max_positions[index],
                        hold_values[index] + extra_close,
                    )
                self._command_positions(hold_values, wait_s=settle)
                self._holding = True
                # Once the minimum number of contact axes is firm, keep every
                # active axis under the hold loop. Unconfirmed axes continue
                # closing in the background instead of blocking placement.
                self._start_hold(
                    hold_values,
                    active_axes,
                    max_positions,
                    profile,
                    baseline,
                )
                print(
                    f"[LHandPro] 柔性抓取确认: {reason}, "
                    f"有效接触轴={sorted(firm_axes)}, 后台保持轴={sorted(active_axes)}, "
                    f"保持位置={hold_values}"
                )
                return True

            def keep_closing_unverified(reason):
                # Current/position feedback can remain stale even though the
                # hand has physically closed. Keep every unconfirmed axis on
                # its close target and let the hold loop continue tightening;
                # do not turn a feedback timeout into an automatic release.
                hold_values = list(values)
                for axis in active_axes:
                    if axis not in locked:
                        hold_values[axis - 1] = max_positions[axis - 1]
                self._command_positions(hold_values, wait_s=settle)
                self._holding = True
                self._start_hold(
                    hold_values,
                    active_axes,
                    max_positions,
                    profile,
                    baseline,
                )
                print(
                    f"[LHandPro][未确认但继续] {reason}; "
                    f"保持闭合并继续放置, 保持位置={hold_values}"
                )
                return True

            # In continuous mode every active motor receives its final travel
            # target once and moves without intermediate stops. The loop only
            # monitors actual position/current and locks a contacted axis.
            for axis in active_axes:
                index = axis - 1
                values[index] = (
                    max_positions[index]
                    if continuous
                    else min(max_positions[index], values[index] + step)
                )

            if continuous:
                self._command_positions(values, wait_s=0.0)

            for step_index in range(max_steps):
                if continuous:
                    time.sleep(settle)
                else:
                    self._command_positions(values, wait_s=settle)
                currents = [abs(value) for value in self._read_currents(samples, sample_delay)]
                actual_positions = [
                    int(self._sdk.get_now_position(axis))
                    for axis in range(1, self.dof_active + 1)
                ]
                stall_tolerance = max(0, int(os.getenv("LHAND_FLEX_STALL_TOLERANCE", "20")))
                stall_max_margin = max(1, int(os.getenv("LHAND_FLEX_STALL_MAX_MARGIN", "200")))
                if previous_actual is not None:
                    for axis in required_axes:
                        index = axis - 1
                        stalled = (
                            actual_positions[index] >= min_position
                            and actual_positions[index] < max_positions[index] - stall_max_margin
                            and abs(actual_positions[index] - previous_actual[index]) <= stall_tolerance
                        )
                        stall_counts[axis] = stall_counts[axis] + 1 if stalled else 0
                previous_actual = actual_positions
                for axis in active_axes:
                    hard_counts[axis] = (
                        hard_counts[axis] + 1
                        if currents[axis - 1] >= hard_current
                        else 0
                    )
                hot_axes = {
                    axis for axis in active_axes
                    if hard_counts[axis] >= hard_consecutive
                }
                if hot_axes:
                    unsafe_hot_axes = {
                        axis for axis in hot_axes
                        if actual_positions[axis - 1] < min_position
                    }
                    if unsafe_hot_axes:
                        self._sdk.stop_motors(0)
                        print(
                            f"[LHandPro] 柔性抓取安全停止: 轴 {sorted(unsafe_hot_axes)} "
                            f"连续 {hard_consecutive} 次达到 {hard_current:.0f}‰, "
                            f"位置={actual_positions} 电流={[round(value) for value in currents]}"
                        )
                        return False

                    # A motor that reaches the hard current after it has
                    # entered the closing range is a contact, not necessarily
                    # a fault. Lock only that motor and let the other fingers
                    # continue toward their targets.
                    hot_contact_axes = hot_axes & required_axes
                    hot_target_changed = False
                    for axis in hot_axes:
                        index = axis - 1
                        contacts.add(axis) if axis in required_axes else None
                        locked[axis] = actual_positions[index]
                        values[index] = actual_positions[index]
                        hot_target_changed = True
                    print(
                        f"[LHandPro] 柔性锁定: 硬电流轴={sorted(hot_axes)}, "
                        f"有效接触轴={sorted(hot_contact_axes)}, 连续={hard_consecutive}"
                    )
                    if len(contacts) >= min_contacts and accept_firm_grip(
                        actual_positions, currents,
                        f"硬电流接触轴={sorted(hot_contact_axes)}",
                    ):
                        return True
                    if hot_target_changed:
                        self._command_positions(values, wait_s=0.0)

                events = []
                target_changed = False
                for axis in required_axes:
                    if axis in locked:
                        continue
                    command = actual_positions[axis - 1] if continuous else values[axis - 1]
                    if profile:
                        empty_current = profile_current_envelope(
                            profile, command, axis, profile_window
                        )
                        excess = currents[axis - 1] - empty_current
                        over = excess >= threshold
                    else:
                        empty_current = baseline[axis - 1]
                        excess = currents[axis - 1] - empty_current
                        over = excess >= fallback_delta
                    over = over and command >= min_position
                    hits[axis] = hits[axis] + 1 if over else 0
                    if hits[axis] >= consecutive:
                        lock_position = min(max_positions[axis - 1], command + extra_close)
                        values[axis - 1] = lock_position
                        target_changed = True
                        locked[axis] = lock_position
                        contacts.add(axis)
                        events.append(
                            f"axis{axis} contact@{command} lock@{lock_position} "
                            f"current={currents[axis - 1]:.0f}‰ "
                            f"empty={empty_current:.0f}‰ extra={excess:.0f}‰"
                        )

                for axis in active_axes:
                    if axis in locked:
                        continue
                    index = axis - 1
                    if not continuous:
                        values[index] = min(max_positions[index], values[index] + step)
                    reached_max = (
                        actual_positions[index] >= max_positions[index]
                        if continuous
                        else values[index] >= max_positions[index]
                    )
                    if axis in required_axes and reached_max:
                        locked[axis] = max_positions[index]
                        at_max.add(axis)
                        events.append(f"axis{axis}@max(no contact)")

                if events:
                    print(f"[LHandPro] 柔性锁定: {', '.join(events)}")
                if continuous and target_changed:
                    self._command_positions(values, wait_s=0.0)
                if len(contacts) >= min_contacts:
                    if accept_firm_grip(actual_positions, currents, "达到最少接触轴"):
                        return True
                if all(axis in locked for axis in required_axes):
                    self._command_positions(values, wait_s=settle)
                    if len(contacts) < min_contacts:
                        if accept_firm_grip(actual_positions, currents, "到位后二次电流确认"):
                            return True
                        print(
                            f"[LHandPro] 柔性抓取未确认: 接触轴={sorted(contacts)}, "
                            f"到上限未接触轴={sorted(at_max)}, 要求至少 {min_contacts}"
                        )
                        return keep_closing_unverified("到达闭合上限但接触检测不足")
                    self._holding = True
                    self._start_hold(values, required_axes, max_positions, profile, baseline)
                    print(
                        f"[LHandPro] 柔性抓取完成: 接触轴={sorted(contacts)} "
                        f"保持位置={values}"
                    )
                    return True

            if accept_firm_grip(actual_positions, currents, "检测超时后二次电流确认"):
                return True
            print(
                f"[LHandPro] 柔性抓取未确认: 达到最大步数 {max_steps}, "
                f"位置={actual_positions}, 电流={[round(value) for value in currents]}, "
                f"停滞计数={stall_counts}"
            )
            return keep_closing_unverified("达到最大步数且接触检测不足")
        except Exception as exc:
            self.ok = False
            self._holding = False
            print(f"[LHandPro] 柔性抓取失败, 已禁用灵巧手: {exc}")
            return False

    def record_empty_profile(
        self,
        output_path=None,
        start=0,
        maximum=9000,
        step=250,
        settle_s=0.12,
        samples=3,
        sample_delay_s=0.01,
        safety_current=750,
    ):
        """Sweep unloaded flex axes while keeping the thumb-side axis open."""
        if maximum < start or step <= 0:
            raise ValueError("录制范围或步长无效")
        path = os.path.abspath(os.path.expanduser(output_path or self.profile_path))
        commands = list(range(int(start), int(maximum) + 1, int(step)))
        if not commands or commands[-1] != int(maximum):
            commands.append(int(maximum))
        rows = []
        try:
            self._holding = False
            fixed_positions = self._open_positions()
            sweep_axes = _parse_int_set(
                os.getenv("LHAND_PROFILE_SWEEP_AXES"),
                ",".join(str(axis) for axis in range(2, self.dof_active + 1)),
                1,
                self.dof_active,
            )
            print(
                f"[LHandPro][空载] 扫描轴={sorted(sweep_axes)}, "
                f"固定张开位置={fixed_positions}"
            )
            for command in commands:
                positions = list(fixed_positions)
                for axis in sweep_axes:
                    positions[axis - 1] = command
                self._command_positions(positions, wait_s=settle_s)
                currents = [abs(value) for value in self._read_currents(samples, sample_delay_s)]
                if any(value >= float(safety_current) for value in currents):
                    raise RuntimeError(
                        f"空载电流达到安全阈值 {safety_current}‰: "
                        f"command={command}, currents={[round(v) for v in currents]}"
                    )
                rows.append((command, currents))
                print(
                    f"[LHandPro][空载] command={command:5d} "
                    + " ".join(f"I{axis}={currents[axis - 1]:6.1f}‰" for axis in range(1, self.dof_active + 1))
                )
        finally:
            try:
                self._command_open_positions(self._open_positions(), wait_s=0.8)
            except Exception as exc:
                print(f"[LHandPro][警告] 录制结束后张开失败: {exc}")

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        temporary_path = f"{path}.tmp"
        fieldnames = ["command"] + [f"current_{axis}" for axis in range(1, self.dof_active + 1)]
        fieldnames += ["baud", "node_id", "samples"]
        with open(temporary_path, "w", newline="", encoding="utf-8") as profile_file:
            writer = csv.DictWriter(profile_file, fieldnames=fieldnames)
            writer.writeheader()
            for command, currents in rows:
                row = {"command": command, "baud": self.baud_rate, "node_id": self.node_id, "samples": samples}
                row.update({f"current_{axis}": f"{currents[axis - 1]:.3f}" for axis in range(1, self.dof_active + 1)})
                writer.writerow(row)
        os.replace(temporary_path, path)
        print(f"[LHandPro] 空载曲线已保存: {path} ({len(rows)} 个位置)")
        return path
