#!/usr/bin/env python3
"""Aero Hand contact-state recognition demo.

The demo closes selected compact joints in small increments and classifies the
load into empty, shallow contact, contact, saturated pile, or squeeze overflow.
Normal unloaded tendon current is removed with an angle-indexed empty profile.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import Counter, deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence


AERO_SDK_PATH = "/home/nvidia/grasp/aero-hand-open/sdk/src"
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand  # noqa: E402


DIPAN_DIR = Path(__file__).resolve().parent
DEFAULT_EMPTY_PROFILE = DIPAN_DIR / "runtime" / "aero_empty.csv"
OPEN_POSE = [80.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
COMPACT_NAMES = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_tendon",
    "index",
    "middle",
    "ring",
    "pinky",
]
STATE_LABELS = {
    "empty": "空载",
    "shallow_contact": "浅层接触",
    "contact": "持续接触",
    "saturated_pile": "饱和堆积",
    "squeeze_overflow": "挤压溢出",
}
STATE_PRIORITY = {
    "empty": 0,
    "shallow_contact": 1,
    "contact": 2,
    "saturated_pile": 3,
    "squeeze_overflow": 4,
}


@dataclass(frozen=True)
class StateThresholds:
    contact_extra_ma: float = 180.0
    high_extra_ma: float = 450.0
    overflow_extra_ma: float = 1200.0
    hard_abs_ma: float = 5500.0
    shallow_depth_deg: float = 6.0
    saturation_depth_deg: float = 12.0
    overflow_depth_deg: float = 20.0
    saturation_fingers: int = 3
    overflow_fingers: int = 2
    saturation_lag_deg: float = 5.0
    overflow_lag_deg: float = 15.0
    plateau_window: int = 4
    plateau_range_ma: float = 160.0
    contact_consecutive: int = 2


@dataclass
class ContactFeatures:
    raw_state: str
    confidence: float
    contact_joints: list[int]
    high_joints: list[int]
    overflow_joints: list[int]
    extra_ma: list[float]
    lag_deg: list[float]
    max_abs_ma: float
    max_extra_ma: float
    mean_extra_ma: float
    max_contact_depth_deg: float
    max_lag_deg: float
    plateau: bool
    reason: str


def parse_index_set(raw: str, label: str) -> set[int]:
    try:
        values = {int(item.strip()) for item in str(raw).split(",") if item.strip()}
    except ValueError as error:
        raise ValueError(f"{label} 格式错误: {raw!r}") from error
    invalid = sorted(value for value in values if value < 1 or value > 6)
    if invalid:
        raise ValueError(f"{label} 只能包含 1..6, 无效值: {invalid}")
    if not values:
        raise ValueError(f"{label} 至少需要一个关节")
    return values


def parse_float_list(raw: str, expected_len: int = 7) -> list[float]:
    values = [float(item) for item in str(raw or "").split() if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"期望 {expected_len} 个数, 实际 {len(values)}")
    return values


def load_empty_profile(path: Path) -> list[tuple[float, list[float]]]:
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append((float(row["cmd_1_6"]), parse_float_list(row["abs_ma"])))
            except (KeyError, TypeError, ValueError):
                continue
    rows.sort(key=lambda item: item[0])
    if not rows:
        raise ValueError(f"空载曲线没有有效数据: {path}")
    return rows


def profile_envelope(
    profile: Sequence[tuple[float, list[float]]],
    command: float,
    window_deg: float,
) -> tuple[float, list[float]]:
    nearest_command, nearest_abs = min(profile, key=lambda item: abs(item[0] - command))
    window = max(0.0, float(window_deg))
    nearby = [values for cmd, values in profile if abs(cmd - command) <= window]
    if not nearby:
        return nearest_command, list(nearest_abs)
    return nearest_command, [max(row[index] for row in nearby) for index in range(7)]


def compact_pose(thumb0: float, command: float, drive_joints: set[int]) -> list[float]:
    pose = OPEN_POSE[:]
    pose[0] = float(thumb0)
    for joint in drive_joints:
        pose[joint] = float(command)
    return pose


def average_currents(hand: AeroHand, samples: int, delay_s: float) -> Optional[list[float]]:
    rows = []
    for sample_index in range(max(1, int(samples))):
        values = hand.get_actuator_currents()
        if values is not None and len(values) == 7:
            rows.append([float(value) for value in values])
        if sample_index + 1 < samples:
            time.sleep(max(0.0, float(delay_s)))
    if not rows:
        return None
    return [sum(row[index] for row in rows) / len(rows) for index in range(7)]


class ContactStateClassifier:
    def __init__(self, sense_joints: set[int], thresholds: StateThresholds):
        self.sense_joints = tuple(sorted(sense_joints))
        self.thresholds = thresholds
        self.contact_counts = {joint: 0 for joint in self.sense_joints}
        self.release_counts = {joint: 0 for joint in self.sense_joints}
        self.contact_start_cmd: dict[int, float] = {}
        self.load_history = deque(maxlen=max(2, thresholds.plateau_window))

    def update(
        self,
        command: float,
        currents_ma: Sequence[float],
        empty_abs_ma: Sequence[float],
        actual_positions_deg: Optional[Sequence[float]] = None,
    ) -> ContactFeatures:
        t = self.thresholds
        abs_curr = [abs(float(value)) for value in currents_ma]
        extra = [max(0.0, abs_curr[index] - abs(float(empty_abs_ma[index]))) for index in range(7)]

        raw_contacts = {joint for joint in self.sense_joints if extra[joint] >= t.contact_extra_ma}
        for joint in self.sense_joints:
            if joint in raw_contacts:
                self.contact_counts[joint] += 1
                self.release_counts[joint] = 0
            else:
                self.contact_counts[joint] = 0
                self.release_counts[joint] += 1
                if self.release_counts[joint] >= t.contact_consecutive:
                    self.contact_start_cmd.pop(joint, None)

        contacts = [
            joint for joint in self.sense_joints
            if self.contact_counts[joint] >= t.contact_consecutive
        ]
        for joint in contacts:
            self.contact_start_cmd.setdefault(joint, float(command))

        high = [joint for joint in contacts if extra[joint] >= t.high_extra_ma]
        overflow = [joint for joint in self.sense_joints if extra[joint] >= t.overflow_extra_ma]
        contact_depths = [
            max(0.0, float(command) - self.contact_start_cmd[joint])
            for joint in contacts
        ]
        max_contact_depth = max(contact_depths, default=0.0)

        lag = [math.nan] * 7
        if actual_positions_deg is not None and len(actual_positions_deg) == 7:
            for joint in self.sense_joints:
                value = float(actual_positions_deg[joint])
                if math.isfinite(value):
                    lag[joint] = max(0.0, float(command) - value)
        finite_lag = [lag[joint] for joint in self.sense_joints if math.isfinite(lag[joint])]
        max_lag = max(finite_lag, default=0.0)

        sensed_extra = [extra[joint] for joint in self.sense_joints]
        max_extra = max(sensed_extra, default=0.0)
        mean_extra = sum(sensed_extra) / max(1, len(sensed_extra))
        # Hard-current protection covers every actuator, including joints that
        # are driven but intentionally excluded from contact classification.
        max_abs = max(abs_curr, default=0.0)
        self.load_history.append((float(command), mean_extra))
        plateau = False
        if len(self.load_history) == self.load_history.maxlen:
            commands = [item[0] for item in self.load_history]
            loads = [item[1] for item in self.load_history]
            plateau = (
                commands[-1] > commands[0]
                and max(loads) - min(loads) <= t.plateau_range_ma
                and sum(loads) / len(loads) >= t.high_extra_ma
            )

        hard_current = max_abs >= t.hard_abs_ma
        overflow_by_depth = (
            len(contacts) >= t.overflow_fingers
            and max_contact_depth >= t.overflow_depth_deg
            and (mean_extra >= t.high_extra_ma or max_lag >= t.overflow_lag_deg)
        )
        if hard_current or overflow or overflow_by_depth:
            state = "squeeze_overflow"
            reason = (
                f"hard_abs={hard_current}, overflow={overflow}, depth={max_contact_depth:.1f}deg, "
                f"lag={max_lag:.1f}deg"
            )
            confidence = min(1.0, 0.75 + max(max_extra / max(t.overflow_extra_ma, 1.0) - 1.0, 0.0) * 0.2)
        else:
            saturated = (
                len(contacts) >= t.saturation_fingers
                and max_contact_depth >= t.saturation_depth_deg
                and (len(high) >= 2 or plateau or max_lag >= t.saturation_lag_deg)
            )
            if saturated:
                state = "saturated_pile"
                reason = (
                    f"contacts={contacts}, high={high}, depth={max_contact_depth:.1f}deg, "
                    f"plateau={plateau}, lag={max_lag:.1f}deg"
                )
                confidence = min(1.0, 0.65 + 0.08 * len(contacts) + (0.08 if plateau else 0.0))
            elif contacts:
                if max_contact_depth <= t.shallow_depth_deg:
                    state = "shallow_contact"
                    reason = f"contacts={contacts}, depth={max_contact_depth:.1f}deg"
                    confidence = min(0.9, 0.55 + max_extra / max(t.high_extra_ma, 1.0) * 0.25)
                else:
                    state = "contact"
                    reason = f"contacts={contacts}, depth={max_contact_depth:.1f}deg"
                    confidence = min(0.95, 0.60 + 0.06 * len(contacts))
            else:
                state = "empty"
                reason = f"max_extra={max_extra:.1f}mA"
                confidence = max(0.5, 1.0 - max_extra / max(t.contact_extra_ma, 1.0) * 0.5)

        return ContactFeatures(
            raw_state=state,
            confidence=float(confidence),
            contact_joints=contacts,
            high_joints=high,
            overflow_joints=overflow,
            extra_ma=extra,
            lag_deg=lag,
            max_abs_ma=max_abs,
            max_extra_ma=max_extra,
            mean_extra_ma=mean_extra,
            max_contact_depth_deg=max_contact_depth,
            max_lag_deg=max_lag,
            plateau=plateau,
            reason=reason,
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Aero Hand 空载/接触/堆积/溢出状态识别 demo")
    parser.add_argument("--port", default=os.getenv("AERO_PORT"), help="串口; 默认自动识别")
    parser.add_argument(
        "--profile",
        default=str(DEFAULT_EMPTY_PROFILE),
        help="空载曲线 CSV; 传空字符串则只用起始基线",
    )
    parser.add_argument("--profile-window-deg", type=float, default=3.0, help="空载曲线角度包络窗口")
    parser.add_argument("--drive-joints", default="1,2,3,4,5,6", help="随命令闭合的 compact 关节")
    parser.add_argument("--sense-joints", default="3,4,5,6", help="参与状态识别的 compact 关节")
    parser.add_argument("--thumb0", type=float, default=80.0)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--max", type=float, default=90.0, help="最大闭合角度; 默认 90")
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=0.08)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--sample-delay", type=float, default=0.02)
    parser.add_argument(
        "--torque-limit",
        type=int,
        default=0,
        help="0 表示不修改; 默认保持当前设置（须与空载曲线采集条件一致）",
    )
    parser.add_argument("--temperature-every", type=int, default=10)
    parser.add_argument("--max-temperature-c", type=float, default=65.0)
    parser.add_argument(
        "--state-consecutive",
        type=int,
        default=2,
        help="普通状态连续多少步才确认; 溢出立即确认",
    )
    parser.add_argument("--stop-on", choices=("overflow", "saturation"), default="overflow")
    parser.add_argument("--hold-seconds", type=float, default=0.2)
    parser.add_argument("--release-after", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log", default="", help="CSV 路径; 默认写 /tmp")
    parser.add_argument("--summary", default="", help="JSON 摘要路径; 默认与 CSV 同名")

    parser.add_argument("--contact-extra-ma", type=float, default=180.0)
    parser.add_argument("--high-extra-ma", type=float, default=450.0)
    parser.add_argument("--overflow-extra-ma", type=float, default=1200.0)
    parser.add_argument("--hard-abs-ma", type=float, default=5500.0)
    parser.add_argument("--shallow-depth-deg", type=float, default=6.0)
    parser.add_argument("--saturation-depth-deg", type=float, default=12.0)
    parser.add_argument("--overflow-depth-deg", type=float, default=20.0)
    parser.add_argument("--saturation-fingers", type=int, default=3)
    parser.add_argument("--overflow-fingers", type=int, default=2)
    parser.add_argument("--saturation-lag-deg", type=float, default=5.0)
    parser.add_argument("--overflow-lag-deg", type=float, default=15.0)
    parser.add_argument("--plateau-window", type=int, default=4)
    parser.add_argument("--plateau-range-ma", type=float, default=160.0)
    parser.add_argument("--contact-consecutive", type=int, default=2)
    return parser.parse_args()


def thresholds_from_args(args) -> StateThresholds:
    return StateThresholds(
        contact_extra_ma=args.contact_extra_ma,
        high_extra_ma=args.high_extra_ma,
        overflow_extra_ma=args.overflow_extra_ma,
        hard_abs_ma=args.hard_abs_ma,
        shallow_depth_deg=args.shallow_depth_deg,
        saturation_depth_deg=args.saturation_depth_deg,
        overflow_depth_deg=args.overflow_depth_deg,
        saturation_fingers=max(1, args.saturation_fingers),
        overflow_fingers=max(1, args.overflow_fingers),
        saturation_lag_deg=args.saturation_lag_deg,
        overflow_lag_deg=args.overflow_lag_deg,
        plateau_window=max(2, args.plateau_window),
        plateau_range_ma=args.plateau_range_ma,
        contact_consecutive=max(1, args.contact_consecutive),
    )


def format_vector(values: Sequence[float]) -> str:
    return " ".join("nan" if not math.isfinite(float(value)) else f"{float(value):.1f}" for value in values)


def main() -> int:
    args = parse_args()
    if args.step <= 0 or args.max < args.start:
        raise ValueError("需要 --step > 0 且 --max >= --start")
    drive_joints = parse_index_set(args.drive_joints, "--drive-joints")
    sense_joints = parse_index_set(args.sense_joints, "--sense-joints")
    if not sense_joints.issubset(drive_joints):
        raise ValueError("--sense-joints 必须包含在 --drive-joints 中")

    profile_path = Path(args.profile).expanduser() if str(args.profile).strip() else None
    profile = load_empty_profile(profile_path) if profile_path else None
    thresholds = thresholds_from_args(args)
    if not (
        0.0 <= thresholds.contact_extra_ma
        <= thresholds.high_extra_ma
        <= thresholds.overflow_extra_ma
    ):
        raise ValueError("电流残差阈值必须满足 0 <= contact <= high <= overflow")
    if thresholds.hard_abs_ma <= 0 or args.max_temperature_c <= 0:
        raise ValueError("硬电流上限和温度上限必须 > 0")
    classifier = ContactStateClassifier(sense_joints, thresholds)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = Path(args.log or f"/tmp/aero_contact_states_{timestamp}.csv")
    summary_path = Path(args.summary or log_path.with_suffix(".summary.json"))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    print("[接触状态] compact映射: " + " | ".join(f"{i}:{name}" for i, name in enumerate(COMPACT_NAMES)))
    print(
        f"[接触状态] drive={sorted(drive_joints)} sense={sorted(sense_joints)} "
        f"cmd={args.start:.1f}..{args.max:.1f} step={args.step:.1f} torque={args.torque_limit or '保持'}"
    )
    print(f"[接触状态] 空载曲线: {profile_path if profile else '未使用，仅起始基线'}")
    print(f"[接触状态] CSV={log_path}")

    hand = None
    rows = 0
    stable_state = "empty"
    candidate_state = "empty"
    candidate_count = 0
    transitions = []
    state_counts = Counter()
    worst_state = "empty"
    worst_raw_state = "empty"
    stop_reason = "到达最大命令"
    max_seen_abs = 0.0
    max_seen_extra = 0.0
    baseline = None
    latest_temperatures = [math.nan] * 7
    started_at = time.time()
    error_message = ""

    fields = [
        "time", "step", "command_deg", "raw_state", "stable_state", "state_zh", "confidence",
        "currents_ma", "empty_abs_ma", "extra_ma", "actual_positions_deg", "lag_deg",
        "contact_joints", "high_joints", "overflow_joints", "max_abs_ma", "max_extra_ma",
        "mean_extra_ma", "contact_depth_deg", "max_lag_deg", "plateau", "temperatures_c", "reason",
    ]

    try:
        hand = AeroHand(port=args.port)
        print("[接触状态] 灵巧手已连接")
        if args.torque_limit > 0:
            for actuator_id in range(7):
                hand.set_torque(actuator_id, min(1000, max(1, args.torque_limit)))
            print(f"[接触状态] 0..6 力矩上限已设为 {args.torque_limit}")

        start_pose = compact_pose(args.thumb0, args.start, drive_joints)
        hand.set_joint_positions(start_pose)
        time.sleep(0.5)
        baseline = average_currents(hand, args.samples, args.sample_delay)
        if baseline is None:
            raise RuntimeError("无法读取起始基线电流")
        print("[接触状态] 起始基线(mA): " + format_vector(baseline))

        with log_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            step_index = 0
            command = float(args.start)
            while command <= args.max + 1e-6:
                hand.set_joint_positions(compact_pose(args.thumb0, command, drive_joints))
                time.sleep(max(0.0, args.settle))
                currents = average_currents(hand, args.samples, args.sample_delay)
                if currents is None:
                    print(f"[接触状态][警告] step={step_index} 电流读取失败")
                    command += args.step
                    step_index += 1
                    continue

                if profile:
                    profile_command, empty_abs = profile_envelope(profile, command, args.profile_window_deg)
                else:
                    profile_command, empty_abs = args.start, [abs(value) for value in baseline]

                actual_positions = hand.get_joint_positions_compact()
                if actual_positions is not None:
                    actual_positions = [float(value) for value in actual_positions]
                if args.temperature_every > 0 and step_index % args.temperature_every == 0:
                    temperatures = hand.get_actuator_temperatures()
                    if temperatures is not None and len(temperatures) == 7:
                        latest_temperatures = [float(value) for value in temperatures]

                features = classifier.update(command, currents, empty_abs, actual_positions)
                max_seen_abs = max(max_seen_abs, features.max_abs_ma)
                max_seen_extra = max(max_seen_extra, features.max_extra_ma)
                worst_raw_state = max(
                    worst_raw_state,
                    features.raw_state,
                    key=lambda state: STATE_PRIORITY[state],
                )

                if features.raw_state == candidate_state:
                    candidate_count += 1
                else:
                    candidate_state = features.raw_state
                    candidate_count = 1
                required = 1 if candidate_state == "squeeze_overflow" else max(1, args.state_consecutive)
                if candidate_count >= required and candidate_state != stable_state:
                    transitions.append({
                        "time": time.time(),
                        "step": step_index,
                        "command_deg": command,
                        "from": stable_state,
                        "to": candidate_state,
                    })
                    stable_state = candidate_state
                    print(
                        f"[接触状态][切换] {STATE_LABELS[transitions[-1]['from']]} -> "
                        f"{STATE_LABELS[stable_state]} @ {command:.1f}deg"
                    )
                state_counts[stable_state] += 1
                worst_state = max(worst_state, stable_state, key=lambda state: STATE_PRIORITY[state])

                finite_temperatures = [value for value in latest_temperatures if math.isfinite(value)]
                over_temperature = bool(finite_temperatures) and max(finite_temperatures) >= args.max_temperature_c
                row = {
                    "time": f"{time.time():.3f}",
                    "step": step_index,
                    "command_deg": f"{command:.2f}",
                    "raw_state": features.raw_state,
                    "stable_state": stable_state,
                    "state_zh": STATE_LABELS[stable_state],
                    "confidence": f"{features.confidence:.3f}",
                    "currents_ma": format_vector(currents),
                    "empty_abs_ma": format_vector(empty_abs),
                    "extra_ma": format_vector(features.extra_ma),
                    "actual_positions_deg": format_vector(actual_positions or [math.nan] * 7),
                    "lag_deg": format_vector(features.lag_deg),
                    "contact_joints": ",".join(map(str, features.contact_joints)),
                    "high_joints": ",".join(map(str, features.high_joints)),
                    "overflow_joints": ",".join(map(str, features.overflow_joints)),
                    "max_abs_ma": f"{features.max_abs_ma:.1f}",
                    "max_extra_ma": f"{features.max_extra_ma:.1f}",
                    "mean_extra_ma": f"{features.mean_extra_ma:.1f}",
                    "contact_depth_deg": f"{features.max_contact_depth_deg:.1f}",
                    "max_lag_deg": f"{features.max_lag_deg:.1f}",
                    "plateau": int(features.plateau),
                    "temperatures_c": format_vector(latest_temperatures),
                    "reason": f"profile_cmd={profile_command:.1f}; {features.reason}",
                }
                writer.writerow(row)
                handle.flush()
                rows += 1

                print(
                    f"[接触状态] step={step_index:03d} cmd={command:5.1f} "
                    f"state={STATE_LABELS[stable_state]:<6} conf={features.confidence:.2f} "
                    f"contact={features.contact_joints or '-'} extra={features.max_extra_ma:6.0f}mA "
                    f"depth={features.max_contact_depth_deg:4.1f}deg lag={features.max_lag_deg:4.1f}deg "
                    f"plateau={'Y' if features.plateau else 'N'}"
                )

                should_stop = (
                    features.raw_state == "squeeze_overflow"
                    or (
                        args.stop_on == "saturation"
                        and stable_state == "saturated_pile"
                    )
                )
                if over_temperature:
                    print(
                        f"[接触状态][温度提示] 温度达到 {max(finite_temperatures):.1f}C "
                        f">= {args.max_temperature_c:.1f}C，仅后台记录，不改变手部动作"
                    )
                if should_stop:
                    stop_reason = f"检测到{STATE_LABELS[features.raw_state]}"
                    break
                command += args.step
                step_index += 1

        time.sleep(max(0.0, args.hold_seconds))
    except KeyboardInterrupt:
        stop_reason = "用户中断"
        print("\n[接触状态] 用户中断")
    except Exception as error:
        error_message = str(error)
        stop_reason = f"异常: {error}"
        print(f"[接触状态][错误] {error}")
    finally:
        if hand is not None:
            try:
                if args.release_after:
                    print("[接触状态] 安全张开")
                    hand.set_joint_positions(OPEN_POSE)
                    time.sleep(0.4)
            except Exception as release_error:
                print(f"[接触状态][警告] 安全张开失败: {release_error}")
            finally:
                try:
                    hand.close()
                    print("[接触状态] 已断开")
                except Exception as close_error:
                    print(f"[接触状态][警告] 断开失败: {close_error}")

        summary = {
            "started_at": started_at,
            "finished_at": time.time(),
            "profile": str(profile_path) if profile_path else None,
            "drive_joints": sorted(drive_joints),
            "sense_joints": sorted(sense_joints),
            "thresholds": asdict(thresholds),
            "final_state": stable_state,
            "final_state_zh": STATE_LABELS[stable_state],
            "worst_state": worst_state,
            "worst_state_zh": STATE_LABELS[worst_state],
            "worst_raw_state": worst_raw_state,
            "worst_raw_state_zh": STATE_LABELS[worst_raw_state],
            "state_counts": dict(state_counts),
            "transitions": transitions,
            "max_abs_ma": max_seen_abs,
            "max_extra_ma": max_seen_extra,
            "rows": rows,
            "stop_reason": stop_reason,
            "error": error_message or None,
            "csv": str(log_path),
        }
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[接触状态] 停止原因: {stop_reason}")
        print(f"[接触状态] 最严重状态: {STATE_LABELS[worst_state]}")
        if worst_raw_state != worst_state:
            print(f"[接触状态] 未确认瞬时状态: {STATE_LABELS[worst_raw_state]}")
        print(f"[接触状态] CSV: {log_path} ({rows}条)")
        print(f"[接触状态] 摘要: {summary_path}")

    return 1 if error_message else 0


if __name__ == "__main__":
    raise SystemExit(main())
