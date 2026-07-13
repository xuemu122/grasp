#!/usr/bin/env python3
"""Test Aero Hand current-based grasp stop.

Compact 7-joint command:
  joint 0 stays fixed at 80
  joints 1..6 slowly increase from 0 until actuator current rises
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime


AERO_SDK_PATH = "/home/nvidia/grasp/aero-hand-open/sdk/src"
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand  # noqa: E402


OPEN_POSE = [80, 0, 0, 0, 0, 0, 0]
COMPACT_NAMES = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_tendon",
    "index",
    "middle",
    "ring",
    "pinky",
]


def parse_args():
    parser = argparse.ArgumentParser(description="Aero Hand 电流夹紧停止测试")
    parser.add_argument("--port", default=os.getenv("AERO_PORT"), help="Aero Hand 串口; 默认自动识别")
    parser.add_argument("--thumb0", type=float, default=float(os.getenv("AERO_TEST_THUMB0", "80")), help="0号固定角度")
    parser.add_argument("--start", type=float, default=float(os.getenv("AERO_TEST_START", "0")), help="1-6号起始角度")
    parser.add_argument("--max", type=float, default=float(os.getenv("AERO_TEST_MAX", "90")), help="1-6号最大角度")
    parser.add_argument("--step", type=float, default=float(os.getenv("AERO_TEST_STEP", "2")), help="每步增加角度")
    parser.add_argument("--settle", type=float, default=float(os.getenv("AERO_TEST_SETTLE", "0.12")), help="每步动作后等待秒数")
    parser.add_argument("--samples", type=int, default=int(os.getenv("AERO_TEST_SAMPLES", "3")), help="每步电流采样次数")
    parser.add_argument("--sample-delay", type=float, default=float(os.getenv("AERO_TEST_SAMPLE_DELAY", "0.03")), help="采样间隔秒")
    parser.add_argument("--delta-ma", type=float, default=float(os.getenv("AERO_TEST_DELTA_MA", "180")), help="相对基线电流增量阈值 mA")
    parser.add_argument("--abs-ma", type=float, default=float(os.getenv("AERO_TEST_ABS_MA", "650")), help="绝对电流阈值 mA")
    parser.add_argument("--trigger-count", type=int, default=int(os.getenv("AERO_TEST_TRIGGER_COUNT", "1")), help="至少多少路电机超过阈值才停止")
    parser.add_argument("--consecutive", type=int, default=int(os.getenv("AERO_TEST_CONSECUTIVE", "2")), help="连续多少步触发才停止")
    parser.add_argument("--min-cmd", type=float, default=float(os.getenv("AERO_TEST_MIN_CMD", "5")), help="1-6号低于该角度时只记录不触发")
    parser.add_argument("--ignore-actuators", default=os.getenv("AERO_TEST_IGNORE_ACTUATORS", ""), help="不参与触发的电机号, 如 5 或 2,5")
    parser.add_argument("--active-joints", default=os.getenv("AERO_TEST_ACTIVE_JOINTS", "1,2,3,4,5,6"), help="随step变化的compact关节号, 如 3 或 3,4,5,6")
    parser.add_argument("--disable-trigger", action="store_true", help="只记录电流曲线, 不因电流停止")
    parser.add_argument("--profile", default=os.getenv("AERO_TEST_PROFILE", ""), help="空载CSV曲线; 有值时按同角度空载电流差值触发")
    parser.add_argument("--profile-delta-ma", type=float, default=float(os.getenv("AERO_TEST_PROFILE_DELTA_MA", "180")), help="相对空载曲线额外增加多少mA才触发")
    parser.add_argument("--torque-limit", type=int, default=int(os.getenv("AERO_TEST_TORQUE_LIMIT", "0")), help="可选: 先把0-6号位置控制力矩限制到0..1000")
    parser.add_argument("--hold-seconds", type=float, default=float(os.getenv("AERO_TEST_HOLD_SECONDS", "5")), help="触发后保持秒数")
    parser.add_argument("--release-after", action="store_true", help="测试结束后张开")
    parser.add_argument("--log", default=os.getenv("AERO_TEST_LOG", ""), help="CSV日志路径; 默认写 /tmp")
    return parser.parse_args()


def read_currents(hand, samples, delay_s):
    values = []
    for _ in range(max(1, samples)):
        current = hand.get_actuator_currents()
        if current is not None and len(current) == 7:
            values.append([float(v) for v in current])
        time.sleep(max(0.0, delay_s))
    if not values:
        return None
    return [sum(row[i] for row in values) / len(values) for i in range(7)]


def parse_float_list(raw, expected_len=7):
    values = [float(item) for item in str(raw or "").split() if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"期望 {expected_len} 个数, 实际 {len(values)}: {raw!r}")
    return values


def load_empty_profile(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                cmd = float(row["cmd_1_6"])
                abs_ma = parse_float_list(row["abs_ma"])
            except Exception:
                continue
            rows.append((cmd, abs_ma))
    if not rows:
        raise ValueError(f"空载曲线无有效数据: {path}")
    rows.sort(key=lambda item: item[0])
    return rows


def nearest_profile_abs(profile, command_value):
    cmd, abs_ma = min(profile, key=lambda item: abs(item[0] - command_value))
    return cmd, abs_ma


def parse_index_set(raw, label, min_value=0, max_value=6):
    values = {
        int(item.strip())
        for item in str(raw or "").split(",")
        if item.strip()
    }
    bad = [item for item in values if item < min_value or item > max_value]
    if bad:
        raise ValueError(f"{label} 只能填 {min_value}..{max_value}, 当前无效: {bad}")
    return values


def compact_pose(thumb0, value, active_joints):
    pose = OPEN_POSE[:]
    pose[0] = thumb0
    for idx in active_joints:
        if idx == 0:
            pose[0] = value
        else:
            pose[idx] = value
    return pose


def current_summary(currents, baseline):
    abs_curr = [abs(v) for v in currents]
    delta = [max(0.0, abs(currents[i]) - abs(baseline[i])) for i in range(7)]
    return abs_curr, delta


def write_log_row(writer, step_idx, command_value, currents, baseline, reason=""):
    abs_curr, delta = current_summary(currents, baseline)
    writer.writerow({
        "time": f"{time.time():.3f}",
        "step": step_idx,
        "cmd_1_6": f"{command_value:.2f}",
        "currents_ma": " ".join(f"{v:.1f}" for v in currents),
        "abs_ma": " ".join(f"{v:.1f}" for v in abs_curr),
        "delta_ma": " ".join(f"{v:.1f}" for v in delta),
        "max_abs_ma": f"{max(abs_curr[1:]):.1f}",
        "max_delta_ma": f"{max(delta[1:]):.1f}",
        "reason": reason,
    })


def main():
    args = parse_args()
    if args.step <= 0:
        raise ValueError("--step 必须 > 0")
    if args.max < args.start:
        raise ValueError("--max 必须 >= --start")
    ignored = parse_index_set(args.ignore_actuators, "--ignore-actuators")
    active_joints = parse_index_set(args.active_joints, "--active-joints", min_value=1, max_value=6)
    if not active_joints:
        raise ValueError("--active-joints 至少需要一个 1..6 的关节")

    log_path = args.log or f"/tmp/aero_current_grasp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    empty_profile = load_empty_profile(args.profile) if args.profile else None
    hand = AeroHand(port=args.port)
    print("[Aero测试] 已连接")
    print(
        f"[Aero测试] 0号固定 {args.thumb0:.1f}; active={','.join(str(i) for i in sorted(active_joints))} "
        f"{args.start:.1f}→{args.max:.1f}, "
        f"step={args.step:.1f}; delta阈值={args.delta_ma:.0f}mA, abs阈值={args.abs_ma:.0f}mA, "
        f"min_cmd={args.min_cmd:.1f}, consecutive={args.consecutive}"
    )
    print("[Aero测试] compact映射: " + " | ".join(f"{i}:{name}" for i, name in enumerate(COMPACT_NAMES)))
    if ignored:
        print("[Aero测试] 忽略触发电机: " + ",".join(str(i) for i in sorted(ignored)))
    if args.disable_trigger:
        print("[Aero测试] disable-trigger=on: 只记录曲线, 不会因电流停止")
    if empty_profile:
        print(f"[Aero测试] 使用空载曲线: {args.profile}, profile_delta={args.profile_delta_ma:.0f}mA")
    print(f"[Aero测试] CSV日志: {log_path}")

    triggered = False
    final_pose = OPEN_POSE[:]
    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "time",
                "step",
                "cmd_1_6",
                "currents_ma",
                "abs_ma",
                "delta_ma",
                "max_abs_ma",
                "max_delta_ma",
                "reason",
            ],
        )
        writer.writeheader()

        try:
            if args.torque_limit > 0:
                print(f"[Aero测试] 设置0-6号力矩上限: {args.torque_limit}")
                for actuator_id in range(7):
                    try:
                        hand.set_torque(actuator_id, args.torque_limit)
                    except Exception as e:
                        print(f"[Aero测试][警告] 设置 {actuator_id} 号力矩失败: {e}")

            base_pose = compact_pose(args.thumb0, args.start, active_joints)
            hand.set_joint_positions(base_pose)
            time.sleep(0.5)
            baseline = read_currents(hand, args.samples, args.sample_delay)
            if baseline is None:
                raise RuntimeError("无法读取基线电流")
            print("[Aero测试] 基线电流(mA): " + " ".join(f"{v:.1f}" for v in baseline))

            value = args.start
            step_idx = 0
            consecutive_hits = 0
            while value <= args.max + 1e-6:
                pose = compact_pose(args.thumb0, value, active_joints)
                final_pose = pose
                hand.set_joint_positions(pose)
                time.sleep(args.settle)
                currents = read_currents(hand, args.samples, args.sample_delay)
                if currents is None:
                    print(f"[Aero测试][警告] step={step_idx} cmd={value:.1f}: 电流读取失败, 继续")
                    value += args.step
                    step_idx += 1
                    continue

                abs_curr, delta = current_summary(currents, baseline)
                profile_extra = None
                profile_cmd = None
                if empty_profile:
                    profile_cmd, profile_abs = nearest_profile_abs(empty_profile, value)
                    profile_extra = [max(0.0, abs_curr[i] - profile_abs[i]) for i in range(7)]
                    over = [
                        i for i in range(1, 7)
                        if i not in ignored and profile_extra[i] >= args.profile_delta_ma
                    ]
                else:
                    over = [
                        i for i in range(1, 7)
                        if i not in ignored and (delta[i] >= args.delta_ma or abs_curr[i] >= args.abs_ma)
                    ]
                reason = ""
                active_detection = value >= args.min_cmd
                if args.disable_trigger:
                    consecutive_hits = 0
                elif active_detection and len(over) >= args.trigger_count:
                    consecutive_hits += 1
                else:
                    consecutive_hits = 0
                if consecutive_hits >= max(1, args.consecutive):
                    reason = "trigger:" + ",".join(str(i) for i in over)
                    triggered = True
                elif over and not active_detection:
                    reason = "below_min_cmd:" + ",".join(str(i) for i in over)
                elif over:
                    reason = f"candidate({consecutive_hits}/{max(1, args.consecutive)}):" + ",".join(str(i) for i in over)

                write_log_row(writer, step_idx, value, currents, baseline, reason)
                f.flush()
                print(
                    f"[Aero测试] cmd={value:5.1f} "
                    f"max_delta={max(delta[1:]):6.1f}mA max_abs={max(abs_curr[1:]):6.1f}mA "
                    + (
                        f"profile_cmd={profile_cmd:.1f} max_extra={max(profile_extra[1:]):6.1f}mA "
                        if profile_extra is not None
                        else ""
                    )
                    + f"over={over or '-'} {reason}"
                )

                if triggered:
                    names = ", ".join(f"{i}:{COMPACT_NAMES[i]}" for i in over)
                    print(f"[Aero测试][停止] 检测到夹紧/接触: {reason} ({names}), 保持当前姿态 {args.hold_seconds:.1f}s")
                    time.sleep(max(0.0, args.hold_seconds))
                    break

                value += args.step
                step_idx += 1

            if not triggered:
                print(f"[Aero测试][停止] 到达最大角度 {args.max:.1f}, 未触发电流阈值")

        except KeyboardInterrupt:
            print("\n[Aero测试] 用户中断")
        finally:
            if args.release_after:
                print("[Aero测试] 张开")
                hand.set_joint_positions(OPEN_POSE)
                time.sleep(0.3)
            else:
                print("[Aero测试] 保持最后姿态: " + " ".join(f"{v:.1f}" for v in final_pose))
            hand.close()
            print("[Aero测试] 已断开")


if __name__ == "__main__":
    main()
