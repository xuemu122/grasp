#!/usr/bin/env python3
"""Aero Hand gripper wrapper.

手型开关：AERO_HAND_MODE=normal | selflock（默认 normal）

normal（普通 Aero）：
- 抓取后以 100Hz 持续发送 CTRL_POS 保持姿态；
- 张开时直接下发一次 OPEN。

selflock（自锁手，断电也能保持、张开需反驱）：
- 抓取后停止持续驱动，依赖机械自锁；
- 张开时主动反驱（AERO_OPEN_REVERSE_S / AERO_OPEN_REVERSE_HZ 可调）。

兼容旧开关：显式设置 AERO_SELF_LOCK_HOLD / AERO_OPEN_REVERSE_DRIVE 会覆盖手型开关。
"""

import csv
import os
import sys
import threading
import time


_REPO_ROOT = os.environ.get("GRASP_ROOT") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AERO_SDK_PATH = os.environ.get("AERO_SDK_PATH", os.path.join(_REPO_ROOT, "aero-hand-open/sdk/src"))
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand


COMPACT_NAMES = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_tendon",
    "index",
    "middle",
    "ring",
    "pinky",
]

DIPAN_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_AERO_FLEX_PROFILE = os.path.join(DIPAN_DIR, "runtime", "aero_empty.csv")


def _parse_index_set(raw, default, min_value=1, max_value=6):
    text = str(raw if raw not in (None, "") else default)
    values = {
        int(item.strip())
        for item in text.split(",")
        if item.strip()
    }
    bad = [item for item in values if item < min_value or item > max_value]
    if bad:
        raise ValueError(f"关节编号只能填 {min_value}..{max_value}, 当前无效: {bad}")
    return values


def _parse_float_list(raw, expected_len=7):
    values = [float(item) for item in str(raw or "").split() if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"期望 {expected_len} 个数, 实际 {len(values)}")
    return values


def _load_empty_profile(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                cmd = float(row["cmd_1_6"])
                abs_ma = _parse_float_list(row["abs_ma"])
            except Exception:
                continue
            rows.append((cmd, abs_ma))
    rows.sort(key=lambda item: item[0])
    return rows


def _nearest_profile_abs(profile, command_value):
    return min(profile, key=lambda item: abs(item[0] - command_value))


def _profile_abs_envelope(profile, command_value, window_deg):
    nearest_cmd, nearest_abs = _nearest_profile_abs(profile, command_value)
    window = abs(float(window_deg))
    if window <= 1e-6:
        return nearest_cmd, nearest_abs
    rows = [abs_ma for cmd, abs_ma in profile if abs(cmd - float(command_value)) <= window]
    if not rows:
        return nearest_cmd, nearest_abs
    envelope = [max(row[i] for row in rows) for i in range(7)]
    return nearest_cmd, envelope


def _current_summary(currents, baseline):
    abs_curr = [abs(v) for v in currents]
    delta = [max(0.0, abs(currents[i]) - abs(baseline[i])) for i in range(7)]
    return abs_curr, delta


class AeroGripper:
    OPEN = [80, 0, 0, 0, 0, 0, 0]
    GRASP = [80, 70, 52, 70, 58, 59, 53]
    GRASP_TOP = [100, 63, 7, 71, 68, 50, 62]
    GRASP_BOTTLE = [100, 92, 75, 76, 75, 67, 69]

    def __init__(self, port=None):
        self.hand = AeroHand(port=port)
        self.ok = True
        # Match the official power-grasp example's 10ms CTRL_POS loop.
        self._hold_hz = max(1.0, float(os.getenv("AERO_HOLD_HZ", "100")))
        self._hold_max_errors = max(1, int(os.getenv("AERO_HOLD_MAX_ERRORS", "5")))
        self._hold_log_interval_s = max(0.0, float(os.getenv("AERO_HOLD_LOG_INTERVAL_S", "10")))
        self._hold_pose = None
        self._hold_monitor = None
        self._hold_grip_ok = True
        self._hold_lock = threading.Lock()
        self._hold_stop = threading.Event()
        self._hold_thread = None
        self._hold_sent = 0
        self._hold_failures = 0
        # 手型开关：AERO_HAND_MODE=normal（普通 Aero，默认）或 selflock（自锁手）。
        # 兼容旧的单选项 AERO_SELF_LOCK_HOLD / AERO_OPEN_REVERSE_DRIVE（显式设置时覆盖）。
        hand_mode = os.getenv("AERO_HAND_MODE", "normal").strip().lower()
        self._self_lock_hold = hand_mode in (
            "selflock", "self-lock", "self_lock", "lock", "locking", "reverse")
        self._reverse_open = self._self_lock_hold
        lock_override = os.getenv("AERO_SELF_LOCK_HOLD")
        if lock_override is not None and lock_override.strip():
            self._self_lock_hold = lock_override.strip().lower() in ("1", "true", "yes", "on")
        reverse_override = os.getenv("AERO_OPEN_REVERSE_DRIVE")
        if reverse_override is not None and reverse_override.strip():
            self._reverse_open = reverse_override.strip().lower() in ("1", "true", "yes", "on")
        self._grasp_locked = False
        self._reverse_open_s = max(0.0, float(os.getenv("AERO_OPEN_REVERSE_S", "0.5")))
        self._reverse_open_hz = max(1.0, float(os.getenv("AERO_OPEN_REVERSE_HZ", "20")))

    @staticmethod
    def _temperature_trip(temperatures, max_temperature_c):
        """Return (max temperature, hot (index, temperature) pairs), or None."""
        if temperatures is None or len(temperatures) != 7:
            return None
        finite = [float(value) for value in temperatures]
        if not all(value == value and abs(value) != float("inf") for value in finite):
            return None
        hot = [
            (idx, value)
            for idx, value in enumerate(finite)
            if value >= float(max_temperature_c)
        ]
        return max(finite), hot

    def connect(self):
        print("[Aero] 已连接")

    def disconnect(self):
        self._stop_hold()
        if not self.ok:
            print("[Aero] 已禁用, 跳过断开")
            return
        try:
            self.hand.close()
            print("[Aero] 已断开")
        except Exception as e:
            self.ok = False
            print(f"[Aero] 断开失败, 已禁用灵巧手: {e}")

    def _hold_loop(self, stop_event=None):
        stop_event = stop_event or self._hold_stop
        period = 1.0 / self._hold_hz
        next_tick = time.monotonic() + period
        next_check = time.monotonic()
        temperature_interval_s = max(0.2, float(os.getenv("AERO_HOLD_TEMPERATURE_INTERVAL_S", "1")))
        temperature_log_interval_s = max(
            0.0,
            float(os.getenv("AERO_HOLD_TEMPERATURE_LOG_INTERVAL_S", "10")),
        )
        max_temperature_c = float(os.getenv("AERO_HOLD_MAX_TEMPERATURE_C", "65"))
        next_temperature_check = time.monotonic()
        last_temperature_log = 0.0
        consecutive_errors = 0
        last_log = time.monotonic()
        while True:
            if stop_event.wait(max(0.0, next_tick - time.monotonic())):
                return
            next_tick += period
            with self._hold_lock:
                pose = self._hold_pose[:] if self._hold_pose is not None else None
                monitor = self._hold_monitor
            if pose is None or not self.ok:
                continue
            try:
                result = self.hand.set_joint_positions(pose)
                if result is False:
                    raise RuntimeError("CTRL_POS 发送超时")
                self._hold_sent += 1
                if consecutive_errors:
                    print(f"[Aero] 抓取姿态保持已恢复 (连续失败 {consecutive_errors} 次)")
                    consecutive_errors = 0
                if self._hold_log_interval_s and time.monotonic() - last_log >= self._hold_log_interval_s:
                    print(
                        f"[Aero] 保持心跳: CTRL_POS={self._hold_sent}, "
                        f"失败={self._hold_failures}, 频率={self._hold_hz:.0f}Hz"
                    )
                    last_log = time.monotonic()
            except Exception as e:
                consecutive_errors += 1
                self._hold_failures += 1
                print(
                    f"[Aero][警告] 抓取姿态保持发送失败 "
                    f"({consecutive_errors}/{self._hold_max_errors}): {e}"
                )
                if consecutive_errors >= self._hold_max_errors:
                    self.ok = False
                    stop_event.set()
                    print("[Aero] 抓取姿态保持连续失败, 已禁用灵巧手")
                    return

            now = time.monotonic()
            if now >= next_temperature_check:
                next_temperature_check = now + temperature_interval_s
                try:
                    temperatures = self.hand.get_actuator_temperatures()
                    temperature_trip = self._temperature_trip(
                        temperatures,
                        max_temperature_c,
                    )
                    if temperature_trip is not None:
                        max_temperature, hot_actuators = temperature_trip
                        should_log_temperature = (
                            temperature_log_interval_s > 0.0
                            and now - last_temperature_log >= temperature_log_interval_s
                        )
                        if should_log_temperature:
                            temperature_label = " ".join(
                                f"{idx}:{COMPACT_NAMES[idx]}={float(temperature):.0f}C"
                                for idx, temperature in enumerate(temperatures)
                            )
                            print(f"[Aero] 保持温度: {temperature_label}")
                            last_temperature_log = now
                        if hot_actuators:
                            hot_label = ", ".join(
                                f"{idx}:{COMPACT_NAMES[idx]}={temperature:.1f}C"
                                for idx, temperature in hot_actuators
                            )
                            if should_log_temperature or temperature_log_interval_s <= 0.0:
                                print(
                                    f"[Aero][温度提示] {hot_label} >= {max_temperature_c:.1f}C "
                                    "，仅后台记录，继续保持抓取，不改变手部动作"
                                )
                except Exception as e:
                    print(f"[Aero][警告] 保持温度读取失败: {e}")
            if monitor is not None and now >= next_check and not stop_event.is_set():
                next_check = now + monitor["interval_s"]
                try:
                    currents = self._read_currents(
                        samples=monitor["samples"],
                        delay_s=monitor["sample_delay_s"],
                    )
                    if currents is None:
                        monitor["read_failures"] += 1
                        if monitor["read_failures"] == 1 or monitor["read_failures"] % 10 == 0:
                            print(
                                f"[Aero][警告] 保持电流读取失败 "
                                f"({monitor['read_failures']} 次)"
                            )
                    else:
                        monitor["read_failures"] = 0
                        with self._hold_lock:
                            if self._hold_monitor is monitor and self._hold_pose is not None:
                                updated, events, weak_at_max = self._tighten_hold_pose(
                                    self._hold_pose[:],
                                    currents,
                                    monitor,
                                )
                                self._hold_pose = updated
                                self._hold_grip_ok = not weak_at_max
                            else:
                                events = []
                        for event in events:
                            print(event)
                except Exception as e:
                    print(f"[Aero][警告] 保持电流检测失败: {e}")
            if time.monotonic() - next_tick > period * 4:
                # Do not spin after a long serial/OS stall; resume from now.
                next_tick = time.monotonic() + period

    @staticmethod
    def _tighten_hold_pose(pose, currents, monitor):
        """Return a slightly tighter pose when sustained contact load drops."""
        abs_curr = [abs(float(value)) for value in currents]
        baseline_abs = [abs(float(value)) for value in monitor["baseline"]]
        events = []
        for idx in monitor["joints"]:
            command = float(pose[idx])
            if monitor["profile"]:
                _, empty_abs = _profile_abs_envelope(
                    monitor["profile"],
                    command,
                    monitor["profile_window_deg"],
                )
                load_ma = max(0.0, abs_curr[idx] - float(empty_abs[idx]))
                firm = load_ma >= monitor["min_extra_ma"]
                load_label = f"余量={load_ma:.0f}mA"
            else:
                delta_ma = max(0.0, abs_curr[idx] - baseline_abs[idx])
                firm = delta_ma >= monitor["min_extra_ma"] or abs_curr[idx] >= monitor["abs_ma"]
                load_label = f"增量={delta_ma:.0f}mA abs={abs_curr[idx]:.0f}mA"

            if firm:
                monitor["low_counts"][idx] = 0
                monitor["weak_at_max"].discard(idx)
                monitor["warned_at_max"].discard(idx)
                continue

            monitor["low_counts"][idx] += 1
            if monitor["low_counts"][idx] < monitor["consecutive"]:
                continue
            if command < monitor["max_cmd"] - 1e-6:
                step = monitor["steps"].get(idx, monitor["step"])
                tightened = min(monitor["max_cmd"], command + step)
                pose[idx] = tightened
                # Contact loss is already confirmed. Keep tightening at the
                # normal close cadence until load returns instead of waiting
                # for a fresh debounce window after every small step.
                monitor["low_counts"][idx] = max(0, monitor["consecutive"] - 1)
                events.append(
                    f"[Aero] 保持补紧: {idx}:{COMPACT_NAMES[idx]} "
                    f"{command:.0f}°→{tightened:.0f}° ({load_label})"
                )
            else:
                monitor["low_counts"][idx] = 0
                monitor["weak_at_max"].add(idx)
                if idx not in monitor["warned_at_max"]:
                    monitor["warned_at_max"].add(idx)
                    events.append(
                        f"[Aero][警告] {idx}:{COMPACT_NAMES[idx]} 已到 "
                        f"{command:.0f}° 上限仍未检测到抓紧 ({load_label})"
                    )
        reliable_contacts = len(monitor["joints"]) - len(monitor["weak_at_max"])
        grip_unsafe = reliable_contacts < monitor["min_contacts"]
        return pose, events, grip_unsafe

    @staticmethod
    def _build_hold_monitor(
        joints,
        profile,
        baseline,
        profile_window_deg,
        contact_delta_ma,
        abs_ma,
        max_cmd,
        min_contacts,
    ):
        joints = tuple(sorted(int(idx) for idx in joints))
        if not joints or os.getenv("AERO_HOLD_TIGHTEN_ENABLE", "1").lower() not in ("1", "true", "yes", "on"):
            return None
        min_extra_ma = float(os.getenv("AERO_HOLD_MIN_EXTRA_MA", str(max(60.0, contact_delta_ma * 0.55))))
        normal_step = float(os.getenv("AERO_HOLD_TIGHTEN_STEP", os.getenv("AERO_FLEX_STEP", "3")))
        joint1_step = max(
            0.1,
            float(os.getenv("AERO_HOLD_JOINT1_STEP", os.getenv("AERO_FLEX_JOINT1_STEP", "2"))),
        )
        joint2_step = max(
            0.1,
            float(os.getenv("AERO_HOLD_JOINT2_STEP", os.getenv("AERO_FLEX_JOINT2_STEP", "5"))),
        )
        normal_interval_s = float(os.getenv("AERO_FLEX_SETTLE", "0.05"))
        normal_consecutive = int(os.getenv("AERO_FLEX_CONSECUTIVE", "2"))
        return {
            "joints": joints,
            "profile": profile,
            "baseline": list(baseline),
            "profile_window_deg": float(profile_window_deg),
            "min_extra_ma": min_extra_ma,
            "abs_ma": float(os.getenv("AERO_HOLD_ABS_MA", str(abs_ma))),
            "max_cmd": float(os.getenv("AERO_HOLD_TIGHTEN_MAX", str(max_cmd))),
            "min_contacts": max(1, min(len(joints), int(min_contacts))),
            "step": max(0.1, float(os.getenv("AERO_HOLD_TIGHTEN_STEP", str(normal_step)))),
            "steps": {
                idx: (
                    joint2_step
                    if idx == 2
                    else joint1_step
                    if idx == 1
                    else max(0.1, normal_step)
                )
                for idx in joints
            },
            "consecutive": max(1, int(os.getenv("AERO_HOLD_TIGHTEN_CONSECUTIVE", str(normal_consecutive)))),
            "interval_s": max(0.01, float(os.getenv("AERO_HOLD_CHECK_INTERVAL_S", str(normal_interval_s)))),
            "samples": max(1, int(os.getenv("AERO_HOLD_CURRENT_SAMPLES", "1"))),
            "sample_delay_s": max(0.0, float(os.getenv("AERO_HOLD_CURRENT_SAMPLE_DELAY", "0"))),
            "low_counts": {idx: 0 for idx in joints},
            "weak_at_max": set(),
            "warned_at_max": set(),
            "read_failures": 0,
        }

    def _start_hold(self, pose, monitor=None):
        target = [float(value) for value in pose]
        result = self.hand.set_joint_positions(target)
        if result is False:
            raise RuntimeError("CTRL_POS 初始发送超时")
        if self._self_lock_hold:
            # 自锁手：目标下发一次即可，机械结构保持；不再启动 100Hz 驱动。
            with self._hold_lock:
                self._hold_pose = target
                self._hold_monitor = monitor
                self._hold_grip_ok = True
            self._grasp_locked = True
            print("[Aero] 自锁保持: 已停止持续驱动（断电保持抓取）")
            return
        with self._hold_lock:
            self._hold_pose = target
            self._hold_monitor = monitor
            self._hold_grip_ok = True
        stop_event = threading.Event()
        self._hold_stop = stop_event
        self._hold_sent = 0
        self._hold_failures = 0
        if self._hold_thread is None or not self._hold_thread.is_alive():
            self._hold_thread = threading.Thread(
                target=self._hold_loop,
                args=(stop_event,),
                name="aero-grasp-hold",
                daemon=True,
            )
            self._hold_thread.start()
        monitor_label = "，带电流补紧" if monitor is not None else ""
        print(f"[Aero] 持续保持抓取姿态 ({self._hold_hz:.0f}Hz{monitor_label})")

    def _stop_hold(self):
        stop_event = self._hold_stop
        stop_event.set()
        thread = self._hold_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._hold_thread = None
        with self._hold_lock:
            self._hold_pose = None
            self._hold_monitor = None
            self._hold_grip_ok = True
        self._grasp_locked = False

    def is_holding(self):
        """Return whether the hand is still holding the grasp.

        自锁手：抓取后不再持续发送 CTRL_POS，但只要机械锁存就视为 holding。
        """
        if not self.ok:
            return False
        if self._grasp_locked:
            return True
        thread = self._hold_thread
        return bool(
            thread is not None
            and thread.is_alive()
            and self._hold_pose is not None
            and self._hold_grip_ok
            and not self._hold_stop.is_set()
        )

    def open(self):
        if not self.ok:
            print("[Aero] 已禁用, 跳过张开")
            return False
        try:
            self._stop_hold()
            if self._reverse_open:
                # 自锁手必须由舵机主动反驱才能张开，不能只松力/断电。
                deadline = time.monotonic() + self._reverse_open_s
                sent = 0
                while True:
                    result = self.hand.set_joint_positions(self.OPEN)
                    if result is False:
                        raise RuntimeError("CTRL_POS 张开发送超时")
                    sent += 1
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(max(0.0, 1.0 / self._reverse_open_hz))
                print(f"[Aero] 张开(反驱 {sent} 次, {self._reverse_open_s:.2f}s)")
            else:
                result = self.hand.set_joint_positions(self.OPEN)
                if result is False:
                    raise RuntimeError("CTRL_POS 张开发送超时")
                print("[Aero] 张开")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 张开失败, 已禁用灵巧手: {e}")
            return False

    def _read_currents(self, samples=1, delay_s=0.0):
        values = []
        for _ in range(max(1, int(samples))):
            current = self.hand.get_actuator_currents()
            if current is not None and len(current) == 7:
                values.append([float(v) for v in current])
            time.sleep(max(0.0, float(delay_s)))
        if not values:
            return None
        return [sum(row[i] for row in values) / len(values) for i in range(7)]

    def _read_temperatures(self):
        try:
            temperatures = self.hand.get_actuator_temperatures()
        except Exception as exc:
            print(f"[Aero][警告] 温度读取失败: {exc}")
            return None
        return temperatures if temperatures is not None and len(temperatures) == 7 else None

    def _pose_from_values(self, thumb0, values_by_joint):
        pose = self.OPEN[:]
        pose[0] = float(thumb0)
        for idx, value in values_by_joint.items():
            pose[int(idx)] = float(value)
        return pose

    def flexible_close(self):
        """Close fingers independently and stop after required fingers are tight."""
        if not self.ok:
            print("[Aero] 已禁用, 跳过柔性抓取")
            return False
        try:
            self._stop_hold()
            thumb0 = float(os.getenv("AERO_FLEX_THUMB0", "80"))
            start = float(os.getenv("AERO_FLEX_START", "0"))
            max_cmd = float(os.getenv("AERO_FLEX_MAX", "80"))
            step = float(os.getenv("AERO_FLEX_STEP", "3"))
            joint1_step = max(0.1, float(os.getenv("AERO_FLEX_JOINT1_STEP", "2")))
            joint2_step = max(0.1, float(os.getenv("AERO_FLEX_JOINT2_STEP", "5")))
            settle = float(os.getenv("AERO_FLEX_SETTLE", "0.05"))
            samples = int(os.getenv("AERO_FLEX_SAMPLES", "1"))
            sample_delay = float(os.getenv("AERO_FLEX_SAMPLE_DELAY", "0"))
            min_cmd = float(os.getenv("AERO_FLEX_MIN_CMD", "15"))
            consecutive = max(1, int(os.getenv("AERO_FLEX_CONSECUTIVE", "2")))
            extra_close = float(os.getenv("AERO_FLEX_EXTRA_CLOSE", "8"))
            profile_delta_ma = float(os.getenv("AERO_FLEX_PROFILE_DELTA_MA", "180"))
            profile_window_deg = float(os.getenv("AERO_FLEX_PROFILE_WINDOW_DEG", "6"))
            delta_ma = float(os.getenv("AERO_FLEX_DELTA_MA", "180"))
            abs_ma = float(os.getenv("AERO_FLEX_ABS_MA", "650"))
            max_steps = int(os.getenv("AERO_FLEX_MAX_STEPS", "300"))
            temperature_every = max(1, int(os.getenv("AERO_FLEX_TEMPERATURE_EVERY", "10")))
            max_temperature_c = float(os.getenv("AERO_FLEX_MAX_TEMPERATURE_C", "65"))
            profile_value = os.getenv("AERO_FLEX_PROFILE", DEFAULT_AERO_FLEX_PROFILE).strip()
            profile_path = (
                os.path.abspath(os.path.expanduser(profile_value))
                if profile_value
                else ""
            )
            active_joints = _parse_index_set(os.getenv("AERO_FLEX_ACTIVE_JOINTS"), "1,2,3,4,5,6")
            required_raw = os.getenv("AERO_FLEX_REQUIRED_FINGERS", "")
            required_fingers = (
                _parse_index_set(required_raw, "", min_value=3, max_value=6)
                if required_raw.strip()
                else {idx for idx in active_joints if 3 <= idx <= 6}
            )
            if not required_fingers:
                required_fingers = {3, 4}
            min_contacts = max(
                1,
                min(
                    len(required_fingers),
                    int(os.getenv("AERO_FLEX_MIN_CONTACTS", "2")),
                ),
            )
            missing_required = sorted(required_fingers - active_joints)
            if missing_required:
                raise ValueError(f"required fingers 未激活: {missing_required}")

            empty_profile = None
            profile_error = None
            if profile_path and os.path.isfile(profile_path):
                try:
                    empty_profile = _load_empty_profile(profile_path)
                except (OSError, ValueError) as exc:
                    profile_error = str(exc)
            if profile_path and empty_profile:
                print(f"[Aero] 柔性抓取: 使用空载曲线 {profile_path}")
            elif profile_path and os.path.exists(profile_path):
                print(
                    f"[Aero][警告] 空载曲线内容无效: {profile_path}"
                    f"{f' ({profile_error})' if profile_error else ''}, "
                    "使用起始基线电流判断"
                )
            elif profile_path:
                print(
                    f"[Aero][警告] 空载曲线文件不存在: {profile_path}, "
                    "使用起始基线电流判断"
                )
            else:
                print("[Aero] 未配置空载曲线, 使用起始基线电流判断")

            values = {idx: start for idx in active_joints}
            for idx in required_fingers:
                values.setdefault(idx, start)
            self.hand.set_joint_positions(self._pose_from_values(thumb0, values))
            time.sleep(0.2)
            baseline = self._read_currents(samples=samples, delay_s=sample_delay)
            if baseline is None:
                print("[Aero] 柔性抓取失败: 无法读取基线电流")
                return False

            hit_counts = {idx: 0 for idx in required_fingers}
            locked = {}
            contact_locked = set()
            print(
                "[Aero] 柔性抓取开始: "
                f"active={sorted(active_joints)} required={sorted(required_fingers)} "
                f"max={max_cmd:.0f} step={step:.1f} "
                f"joint1_step={joint1_step:.1f} joint2_step={joint2_step:.1f} "
                f"extra={extra_close:.1f} "
                f"profile_window=±{profile_window_deg:.1f}°"
            )

            step_by_joint = {
                idx: (
                    joint2_step
                    if idx == 2
                    else joint1_step
                    if idx == 1
                    else step
                )
                for idx in active_joints
            }

            for step_idx in range(max_steps):
                pose = self._pose_from_values(thumb0, values)
                self.hand.set_joint_positions(pose)
                time.sleep(settle)
                currents = self._read_currents(samples=samples, delay_s=sample_delay)
                if currents is None:
                    print(f"[Aero][警告] 柔性抓取 step={step_idx}: 电流读取失败")
                    continue
                if step_idx % temperature_every == 0:
                    temperature_trip = self._temperature_trip(
                        self._read_temperatures(),
                        max_temperature_c,
                    )
                    if temperature_trip is not None:
                        max_temperature, hot_actuators = temperature_trip
                        if hot_actuators:
                            hot_label = ", ".join(
                                f"{idx}:{COMPACT_NAMES[idx]}={temperature:.1f}C"
                                for idx, temperature in hot_actuators
                            )
                            print(
                                f"[Aero][温度提示] 柔性闭合检测到 {hot_label} "
                                f"温度 >= {max_temperature_c:.1f}C，仅后台记录，继续闭合"
                            )

                abs_curr, delta = _current_summary(currents, baseline)
                over = []
                profile_extra = [0.0] * 7
                if empty_profile:
                    for idx in range(1, 7):
                        _, profile_abs = _profile_abs_envelope(
                            empty_profile,
                            values.get(idx, start),
                            profile_window_deg,
                        )
                        profile_extra[idx] = max(0.0, abs_curr[idx] - profile_abs[idx])
                        if profile_extra[idx] >= profile_delta_ma:
                            over.append(idx)
                else:
                    over = [
                        idx for idx in range(1, 7)
                        if delta[idx] >= delta_ma or abs_curr[idx] >= abs_ma
                    ]

                active_over = [
                    idx for idx in over
                    if idx in required_fingers and values.get(idx, start) >= min_cmd
                ]
                events = []
                for idx in required_fingers:
                    if idx in locked:
                        continue
                    hit_counts[idx] = hit_counts[idx] + 1 if idx in active_over else 0
                    if hit_counts[idx] >= consecutive:
                        trigger_cmd = values.get(idx, start)
                        lock_value = min(max_cmd, values.get(idx, start) + extra_close)
                        values[idx] = lock_value
                        locked[idx] = lock_value
                        contact_locked.add(idx)
                        if empty_profile:
                            _, profile_abs = _profile_abs_envelope(empty_profile, trigger_cmd, profile_window_deg)
                            reason = (
                                f"extra={profile_extra[idx]:.0f}mA "
                                f"abs={abs_curr[idx]:.0f}mA empty_env={profile_abs[idx]:.0f}mA "
                                f"hit={hit_counts[idx]}/{consecutive}"
                            )
                        else:
                            reason = (
                                f"delta={delta[idx]:.0f}mA abs={abs_curr[idx]:.0f}mA "
                                f"hit={hit_counts[idx]}/{consecutive}"
                            )
                        events.append(
                            f"{idx}:{COMPACT_NAMES[idx]} trigger@{trigger_cmd:.0f} "
                            f"lock@{lock_value:.0f} ({reason})"
                        )

                for idx in active_joints:
                    if idx not in locked:
                        values[idx] = min(max_cmd, values.get(idx, start) + step_by_joint.get(idx, step))

                for idx in required_fingers:
                    if idx not in locked and values.get(idx, start) >= max_cmd:
                        locked[idx] = values.get(idx, max_cmd)
                        events.append(f"{idx}:{COMPACT_NAMES[idx]}@max")

                if events:
                    print(f"[Aero] 柔性锁定: {', '.join(events)}")
                if all(idx in locked for idx in required_fingers):
                    if len(contact_locked) < min_contacts:
                        print(
                            "[Aero] 柔性抓取失败: "
                            f"有效接触手指={sorted(contact_locked)}, "
                            f"少于要求 {min_contacts}, 判定为空抓"
                        )
                        return False
                    final_pose = self._pose_from_values(thumb0, values)
                    hold_monitor = self._build_hold_monitor(
                        contact_locked,
                        empty_profile,
                        baseline,
                        profile_window_deg,
                        profile_delta_ma,
                        abs_ma,
                        max_cmd,
                        min_contacts,
                    )
                    self._start_hold(final_pose, monitor=hold_monitor)
                    print(
                        "[Aero] 柔性抓取完成: "
                        + ", ".join(f"{idx}:{COMPACT_NAMES[idx]}={locked[idx]:.0f}" for idx in sorted(locked))
                    )
                    return True

            pending = sorted(idx for idx in required_fingers if idx not in locked)
            print(f"[Aero][警告] 柔性抓取达到最大步数, 未完成: {pending}")
            if len(contact_locked) >= min_contacts:
                hold_monitor = self._build_hold_monitor(
                    contact_locked,
                    empty_profile,
                    baseline,
                    profile_window_deg,
                    profile_delta_ma,
                    abs_ma,
                    max_cmd,
                    min_contacts,
                )
                self._start_hold(self._pose_from_values(thumb0, values), monitor=hold_monitor)
                return True
            print(
                "[Aero] 柔性抓取失败: "
                f"有效接触手指={sorted(contact_locked)}, 少于要求 {min_contacts}"
            )
            return False
        except Exception as e:
            self.ok = False
            print(f"[Aero] 柔性抓取失败, 已禁用灵巧手: {e}")
            return False

    def close(self, top=False, mode=None):
        if not self.ok:
            print("[Aero] 已禁用, 跳过抓取")
            return False
        try:
            if mode == "bottle":
                pose = self.GRASP_BOTTLE
                label = "瓶子抓取"
            elif top:
                pose = self.GRASP_TOP
                label = "上端抓取"
            else:
                pose = self.GRASP
                label = "抓取"
            self._stop_hold()
            self._start_hold(pose)
            print(f"[Aero] {label}")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 抓取失败, 已禁用灵巧手: {e}")
            return False
