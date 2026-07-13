#!/usr/bin/env python3
"""Aero Hand gripper wrapper."""

import csv
import os
import sys
import time


AERO_SDK_PATH = "/home/nvidia/grasp/aero-hand-open/sdk/src"
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

    def connect(self):
        print("[Aero] 已连接")

    def disconnect(self):
        if not self.ok:
            print("[Aero] 已禁用, 跳过断开")
            return
        try:
            self.hand.close()
            print("[Aero] 已断开")
        except Exception as e:
            self.ok = False
            print(f"[Aero] 断开失败, 已禁用灵巧手: {e}")

    def open(self):
        if not self.ok:
            print("[Aero] 已禁用, 跳过张开")
            return False
        try:
            self.hand.set_joint_positions(self.OPEN)
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
            thumb0 = float(os.getenv("AERO_FLEX_THUMB0", "80"))
            start = float(os.getenv("AERO_FLEX_START", "0"))
            max_cmd = float(os.getenv("AERO_FLEX_MAX", "80"))
            step = float(os.getenv("AERO_FLEX_STEP", "2"))
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
            profile_path = os.getenv("AERO_FLEX_PROFILE", DEFAULT_AERO_FLEX_PROFILE)
            active_joints = _parse_index_set(os.getenv("AERO_FLEX_ACTIVE_JOINTS"), "1,2,3,4,5,6")
            required_raw = os.getenv("AERO_FLEX_REQUIRED_FINGERS", "")
            required_fingers = (
                _parse_index_set(required_raw, "", min_value=3, max_value=6)
                if required_raw.strip()
                else {idx for idx in active_joints if 3 <= idx <= 6}
            )
            if not required_fingers:
                required_fingers = {3, 4}
            missing_required = sorted(required_fingers - active_joints)
            if missing_required:
                raise ValueError(f"required fingers 未激活: {missing_required}")

            empty_profile = None
            if profile_path and os.path.exists(profile_path):
                empty_profile = _load_empty_profile(profile_path)
            if profile_path and empty_profile:
                print(f"[Aero] 柔性抓取: 使用空载曲线 {profile_path}")
            else:
                print("[Aero][警告] 柔性抓取未找到空载曲线, 使用基线电流判断")

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
            print(
                "[Aero] 柔性抓取开始: "
                f"active={sorted(active_joints)} required={sorted(required_fingers)} "
                f"max={max_cmd:.0f} step={step:.1f} extra={extra_close:.1f} "
                f"profile_window=±{profile_window_deg:.1f}°"
            )

            for step_idx in range(max_steps):
                pose = self._pose_from_values(thumb0, values)
                self.hand.set_joint_positions(pose)
                time.sleep(settle)
                currents = self._read_currents(samples=samples, delay_s=sample_delay)
                if currents is None:
                    print(f"[Aero][警告] 柔性抓取 step={step_idx}: 电流读取失败")
                    continue

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
                        values[idx] = min(max_cmd, values.get(idx, start) + step)

                for idx in required_fingers:
                    if idx not in locked and values.get(idx, start) >= max_cmd:
                        locked[idx] = values.get(idx, max_cmd)
                        events.append(f"{idx}:{COMPACT_NAMES[idx]}@max")

                if events:
                    print(f"[Aero] 柔性锁定: {', '.join(events)}")
                if all(idx in locked for idx in required_fingers):
                    print(
                        "[Aero] 柔性抓取完成: "
                        + ", ".join(f"{idx}:{COMPACT_NAMES[idx]}={locked[idx]:.0f}" for idx in sorted(locked))
                    )
                    return True

            pending = sorted(idx for idx in required_fingers if idx not in locked)
            print(f"[Aero][警告] 柔性抓取达到最大步数, 未完成: {pending}")
            return bool(locked)
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
            self.hand.set_joint_positions(pose)
            print(f"[Aero] {label}")
            return True
        except Exception as e:
            self.ok = False
            print(f"[Aero] 抓取失败, 已禁用灵巧手: {e}")
            return False
