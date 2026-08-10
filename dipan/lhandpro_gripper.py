#!/usr/bin/env python3
"""LHandPro RS485 gripper backend with current-profile flexible grasping."""

import csv
import os
import sys
import time


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
    ):
        self.port = port or os.getenv("LHAND_RS485_PORT") or None
        self.baud_rate = int(baud_rate or os.getenv("LHAND_RS485_BAUD", "500000"))
        self.node_id = int(node_id or os.getenv("LHAND_RS485_NODE_ID", "1"))
        self.profile_path = os.path.abspath(os.path.expanduser(
            profile_path or os.getenv("LHAND_FLEX_PROFILE", DEFAULT_PROFILE_PATH)
        ))
        self._controller_factory = controller_factory
        self.controller = None
        self.dof_active = 0
        self.ok = True
        self._holding = False

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

    def connect(self, enable_motors=None, home_motors=None):
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
                enable_motors=enable,
                home_motors=home,
                home_wait_time=float(os.getenv("LHAND_HOME_WAIT_S", "6")),
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
            return True
        except Exception as exc:
            self.ok = False
            if self.controller is not None:
                try:
                    self.controller.disconnect()
                except Exception:
                    pass
            self.controller = None
            print(f"[LHandPro] 连接失败: {exc}")
            raise

    @property
    def _sdk(self):
        if not self.ok or self.controller is None or not self.controller.sdk_handle:
            raise RuntimeError("LHandPro 尚未连接")
        return self.controller.sdk_handle

    def disconnect(self):
        self._holding = False
        controller = self.controller
        self.controller = None
        if controller is None:
            return
        try:
            controller.stop_motors()
        except Exception as exc:
            print(f"[LHandPro][警告] 停止电机失败: {exc}")
        finally:
            controller.disconnect()
        print("[LHandPro] 已断开")

    def _open_positions(self):
        return _parse_int_vector(
            os.getenv("LHAND_OPEN_POSITIONS"), self.dof_active, 0
        )

    def _max_positions(self):
        return _parse_int_vector(
            os.getenv("LHAND_FLEX_MAX_POSITIONS"), self.dof_active, 9000
        )

    def _command_positions(self, positions, wait_s=0.0, velocity=None, max_current=None):
        if len(positions) != self.dof_active:
            raise ValueError(f"位置数量应为 {self.dof_active}, 实际 {len(positions)}")
        velocity = int(velocity or os.getenv("LHAND_FLEX_VELOCITY", "12000"))
        max_current = int(max_current or os.getenv("LHAND_FLEX_MAX_CURRENT", "600"))
        sdk = self._sdk
        for axis, position in enumerate(positions, start=1):
            sdk.set_target_position(axis, int(position))
            sdk.set_position_velocity(axis, velocity)
            sdk.set_max_current(axis, max_current)
        sdk.move_motors(0)
        if wait_s > 0:
            time.sleep(float(wait_s))

    def _read_currents(self, samples=1, delay_s=0.0):
        rows = []
        for sample_index in range(max(1, int(samples))):
            rows.append([float(self._sdk.get_now_current(axis)) for axis in range(1, self.dof_active + 1)])
            if sample_index + 1 < max(1, int(samples)) and delay_s > 0:
                time.sleep(float(delay_s))
        return [sum(row[index] for row in rows) / len(rows) for index in range(self.dof_active)]

    def open(self):
        if not self.ok:
            print("[LHandPro] 已禁用, 跳过张开")
            return False
        try:
            self._holding = False
            self._command_positions(
                self._open_positions(),
                wait_s=float(os.getenv("LHAND_OPEN_WAIT_S", "0.8")),
            )
            print("[LHandPro] 张开")
            return True
        except Exception as exc:
            self.ok = False
            print(f"[LHandPro] 张开失败, 已禁用灵巧手: {exc}")
            return False

    def close(self, top=False, mode=None):
        del top, mode
        if not self.ok:
            return False
        try:
            positions = _parse_int_vector(
                os.getenv("LHAND_GRASP_POSITIONS"), self.dof_active, 7500
            )
            self._command_positions(
                positions,
                wait_s=float(os.getenv("LHAND_CLOSE_WAIT_S", "1.0")),
            )
            self._holding = True
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

    def flexible_close(self):
        """Close each active axis until its current rises above the empty curve."""
        if not self.ok:
            print("[LHandPro] 已禁用, 跳过柔性抓取")
            return False
        try:
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
                ",".join(str(axis) for axis in range(1, self.dof_active + 1)),
                1,
                self.dof_active,
            )
            required_axes = _parse_int_set(
                os.getenv("LHAND_FLEX_REQUIRED_AXES"),
                ",".join(str(axis) for axis in range(3, self.dof_active + 1)),
                1,
                self.dof_active,
            )
            if not required_axes or not required_axes.issubset(active_axes):
                raise ValueError("required axes 必须是 active axes 的非空子集")

            step = max(1, int(os.getenv("LHAND_FLEX_STEP", "250")))
            settle = max(0.035, float(os.getenv("LHAND_FLEX_SETTLE_S", "0.08")))
            samples = max(1, int(os.getenv("LHAND_FLEX_SAMPLES", "2")))
            sample_delay = max(0.0, float(os.getenv("LHAND_FLEX_SAMPLE_DELAY_S", "0.01")))
            threshold = float(os.getenv("LHAND_FLEX_PROFILE_DELTA", "80"))
            fallback_delta = float(os.getenv("LHAND_FLEX_BASELINE_DELTA", "120"))
            profile_window = max(0, int(os.getenv("LHAND_FLEX_PROFILE_WINDOW", "300")))
            consecutive = max(1, int(os.getenv("LHAND_FLEX_CONSECUTIVE", "2")))
            extra_close = max(0, int(os.getenv("LHAND_FLEX_EXTRA_CLOSE", "150")))
            min_position = int(os.getenv("LHAND_FLEX_MIN_POSITION", "1000"))
            min_contacts = max(1, min(
                len(required_axes), int(os.getenv("LHAND_FLEX_MIN_CONTACTS", "2"))
            ))
            hard_current = float(os.getenv("LHAND_FLEX_HARD_CURRENT", "750"))
            max_steps = max(1, int(os.getenv("LHAND_FLEX_MAX_STEPS", "120")))

            values = list(open_positions)
            self._holding = False
            self._command_positions(values, wait_s=0.25)
            baseline = [abs(value) for value in self._read_currents(samples, sample_delay)]
            hits = {axis: 0 for axis in required_axes}
            locked = {}
            contacts = set()
            at_max = set()
            print(
                f"[LHandPro] 柔性抓取开始: active={sorted(active_axes)} "
                f"required={sorted(required_axes)} step={step} threshold={threshold:.0f}"
            )

            for step_index in range(max_steps):
                self._command_positions(values, wait_s=settle)
                currents = [abs(value) for value in self._read_currents(samples, sample_delay)]
                if any(current >= hard_current for current in currents):
                    hot_axes = [index + 1 for index, current in enumerate(currents) if current >= hard_current]
                    self._sdk.stop_motors(0)
                    print(
                        f"[LHandPro] 柔性抓取安全停止: 轴 {hot_axes} 电流达到 "
                        f"{hard_current:.0f}‰"
                    )
                    return False

                events = []
                for axis in required_axes:
                    if axis in locked:
                        continue
                    command = values[axis - 1]
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
                    values[index] = min(max_positions[index], values[index] + step)
                    if axis in required_axes and values[index] >= max_positions[index]:
                        locked[axis] = values[index]
                        at_max.add(axis)
                        events.append(f"axis{axis}@max(no contact)")

                if events:
                    print(f"[LHandPro] 柔性锁定: {', '.join(events)}")
                if all(axis in locked for axis in required_axes):
                    self._command_positions(values, wait_s=settle)
                    if len(contacts) < min_contacts:
                        print(
                            f"[LHandPro] 柔性抓取失败: 接触轴={sorted(contacts)}, "
                            f"到上限未接触轴={sorted(at_max)}, 要求至少 {min_contacts}"
                        )
                        return False
                    self._holding = True
                    print(
                        f"[LHandPro] 柔性抓取完成: 接触轴={sorted(contacts)} "
                        f"保持位置={values}"
                    )
                    return True

            print(f"[LHandPro] 柔性抓取失败: 达到最大步数 {max_steps}")
            return False
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
        """Sweep an unloaded hand and atomically write its current envelope CSV."""
        if maximum < start or step <= 0:
            raise ValueError("录制范围或步长无效")
        path = os.path.abspath(os.path.expanduser(output_path or self.profile_path))
        commands = list(range(int(start), int(maximum) + 1, int(step)))
        if not commands or commands[-1] != int(maximum):
            commands.append(int(maximum))
        rows = []
        try:
            self._holding = False
            for command in commands:
                positions = [command] * self.dof_active
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
                self._command_positions(self._open_positions(), wait_s=0.8)
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
