#!/usr/bin/env python3
"""Aero 手小尺寸物体柔性抓握测试.

初始位 (GUI 滑块归一化 0..1 → 关节角度 = lower + (upper-lower)*slider):
  0 thumb_abduction  0.801 -> 80.1  固定
  1 thumb_flex       0.501 -> 50.1  固定
  2 thumb_tendon     0.000 ->  0.0  跟随递增(柔性)
  3 index_finger     0.152 -> 15.2  柔性抓取(电流停止)
  4 middle_finger    0.139 -> 13.9  柔性抓取(电流停止)
  5 ring_finger      0.000 ->  0.0  固定(已改为张开, 避免跟中指顶死)
  6 pinky_finger     0.000 ->  0.0  固定(已改为张开, 避免跟中指顶死)

0/1/5/6 全程不动; 3/4 按空载曲线或电流增量逐步闭合, 碰到小物体即锁住;
2 号拇指腱跟随递增到 --thumb-max。

先空手采同一姿态的空载曲线(否则旧曲线会误触发):
  python3 test_aero_small_grasp.py --record-profile runtime/aero_small_empty.csv \
      --max 60 --step 2 --fast --restore-after
再抓:
  python3 test_aero_small_grasp.py --fast --profile runtime/aero_small_empty.csv \
      --profile-delta-ma 150 --min-cmd 20 --extra-close 10 --thumb-max 25 \
      --max 55 --hold-seconds 5 --release-after
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime


AERO_SDK_PATH = os.environ.get(
    "AERO_SDK_PATH",
    os.path.join(
        os.environ.get("GRASP_ROOT")
        or os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "aero-hand-open/sdk/src",
    ),
)
sys.path.insert(0, AERO_SDK_PATH)
from aero_open_sdk.aero_hand import AeroHand  # noqa: E402


COMPACT_NAMES = [
    "thumb_abduction",
    "thumb_flex",
    "thumb_tendon",
    "index_finger",
    "middle_finger",
    "ring_finger",
    "pinky_finger",
]

# 现场初始位(GUI 滑块读数 * 100), 单位: 度; 5/6 号已改为 0(张开), 否则中指闭合时会顶到无名指/小指
INITIAL_POSE = [80.1, 50.1, 0.0, 15.2, 13.9, 0.0, 0.0]
OPEN_POSE = [80.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
THUMB_TENDON_IDX = 2
DEFAULT_FLEX_FINGERS = (3, 4)


def parse_args():
    parser = argparse.ArgumentParser(description="Aero Hand 小尺寸物体柔性抓握测试")
    parser.add_argument("--port", default=os.getenv("AERO_PORT"), help="Aero Hand 串口; 默认自动识别")
    parser.add_argument(
        "--initial",
        default=os.getenv("AERO_SMALL_INITIAL", ""),
        help="7 个初始关节角(度), 空格分隔; 默认用 GUI 记录的初始位",
    )
    parser.add_argument(
        "--active-joints",
        default=os.getenv("AERO_SMALL_ACTIVE_JOINTS", "2,3,4"),
        help="允许运动的关节(1..6); 默认 2,3,4",
    )
    parser.add_argument(
        "--flex-fingers",
        default=os.getenv("AERO_SMALL_FLEX_FINGERS", "3,4"),
        help="需要按电流柔性闭合并锁住的手指(3..6); 默认 3,4",
    )
    parser.add_argument("--max", type=float, default=float(os.getenv("AERO_SMALL_MAX", "60")), help="闭合上限角度")
    parser.add_argument("--step", type=float, default=float(os.getenv("AERO_SMALL_STEP", "2")), help="每步增加角度")
    parser.add_argument(
        "--thumb-max",
        type=float,
        default=float(os.getenv("AERO_SMALL_THUMB_MAX", "30")),
        help="2 号拇指腱跟随上限; 默认 30 度",
    )
    parser.add_argument("--settle", type=float, default=float(os.getenv("AERO_SMALL_SETTLE", "0.12")), help="每步后等待秒数")
    parser.add_argument("--samples", type=int, default=int(os.getenv("AERO_SMALL_SAMPLES", "3")), help="每步电流采样次数")
    parser.add_argument("--sample-delay", type=float, default=float(os.getenv("AERO_SMALL_SAMPLE_DELAY", "0.03")), help="采样间隔秒")
    parser.add_argument("--fast", action="store_true", help="快速测试: settle=0.05, samples=1, sample-delay=0")
    parser.add_argument("--delta-ma", type=float, default=float(os.getenv("AERO_SMALL_DELTA_MA", "180")), help="相对基线电流增量阈值 mA")
    parser.add_argument("--abs-ma", type=float, default=float(os.getenv("AERO_SMALL_ABS_MA", "650")), help="绝对电流阈值 mA")
    parser.add_argument("--consecutive", type=int, default=int(os.getenv("AERO_SMALL_CONSECUTIVE", "2")), help="连续多少步触发才锁住")
    parser.add_argument("--min-cmd", type=float, default=float(os.getenv("AERO_SMALL_MIN_CMD", "20")), help="低于该角度只记录不触发")
    parser.add_argument(
        "--profile",
        default=os.getenv("AERO_SMALL_PROFILE", ""),
        help="空载 CSV 曲线; 有值时按相对空载电流差值触发",
    )
    parser.add_argument(
        "--profile-delta-ma",
        type=float,
        default=float(os.getenv("AERO_SMALL_PROFILE_DELTA_MA", "150")),
        help="相对空载曲线额外增加多少 mA 才触发",
    )
    parser.add_argument(
        "--profile-window-deg",
        type=float,
        default=float(os.getenv("AERO_SMALL_PROFILE_WINDOW_DEG", "6")),
        help="空载曲线角度包络窗口; 默认 ±6 度",
    )
    parser.add_argument("--extra-close", type=float, default=float(os.getenv("AERO_SMALL_EXTRA_CLOSE", "10")), help="触发后继续闭合多少度再锁住")
    parser.add_argument(
        "--contact-loss-consecutive",
        type=int,
        default=int(os.getenv("AERO_SMALL_CONTACT_LOSS_CONSECUTIVE", "2")),
        help="收紧途中连续多少次失去接触才取消触发",
    )
    parser.add_argument("--hold-seconds", type=float, default=float(os.getenv("AERO_SMALL_HOLD_SECONDS", "5")), help="锁住后保持秒数")
    parser.add_argument(
        "--record-profile",
        default=os.getenv("AERO_SMALL_RECORD_PROFILE", ""),
        help="空载采曲线模式: 手空着, 2/3/4 从初始位匀速闭合到 --max, 写出该姿态自己的空载 CSV 后退出",
    )
    parser.add_argument(
        "--record-stop-ma",
        type=float,
        default=float(os.getenv("AERO_SMALL_RECORD_STOP_MA", "1200")),
        help="采曲线时单个电机超过该电流视为卡住, 停止继续采集; 默认 1200mA",
    )
    parser.add_argument(
        "--record-stop-consecutive",
        type=int,
        default=int(os.getenv("AERO_SMALL_RECORD_STOP_CONSECUTIVE", "2")),
        help="采曲线时连续多少步超限才停止; 默认 2",
    )
    parser.add_argument(
        "--release-pose",
        default=os.getenv("AERO_SMALL_RELEASE_POSE", ""),
        help="--release-after 时张开的 7 个角度; 默认标准张开位",
    )
    parser.add_argument("--release-after", action="store_true", help="测试结束后张开")
    parser.add_argument("--restore-after", action="store_true", help="测试结束后回到初始位")
    parser.add_argument("--max-steps", type=int, default=int(os.getenv("AERO_SMALL_MAX_STEPS", "200")), help="最大循环步数")
    parser.add_argument("--temperature-every", type=int, default=int(os.getenv("AERO_SMALL_TEMPERATURE_EVERY", "2")), help="每多少步读一次温度")
    parser.add_argument("--max-temperature-c", type=float, default=float(os.getenv("AERO_SMALL_MAX_TEMPERATURE_C", "65")), help="温度提示阈值")
    parser.add_argument("--log", default=os.getenv("AERO_SMALL_LOG", ""), help="CSV 日志路径; 默认 /tmp")
    return parser.parse_args()


def parse_float_list(raw, expected_len=7):
    values = [float(item) for item in str(raw or "").replace(",", " ").split() if item.strip()]
    if len(values) != expected_len:
        raise ValueError(f"期望 {expected_len} 个数, 实际 {len(values)}: {raw!r}")
    return values


def parse_index_set(raw, label, min_value=0, max_value=6):
    values = {int(item.strip()) for item in str(raw or "").split(",") if item.strip()}
    bad = [item for item in values if item < min_value or item > max_value]
    if bad:
        raise ValueError(f"{label} 只能填 {min_value}..{max_value}, 当前无效: {bad}")
    return values


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


def read_temperatures(hand):
    try:
        temperatures = hand.get_actuator_temperatures()
    except Exception as exc:
        print(f"[小物体抓握][警告] 温度读取失败: {exc}")
        return None
    if temperatures is None or len(temperatures) != 7:
        return None
    return [float(value) for value in temperatures]


def load_empty_profile(path):
    """读取空载曲线. 每行存 (7个关节的指令角度, 7个绝对电流).

    优先用 commands_0_6 做逐关节查表(本脚本采的曲线), 兼容旧的 cmd_1_6 单值曲线.
    """
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                abs_ma = parse_float_list(row["abs_ma"])
                raw_cmds = str(row.get("commands_0_6") or "").replace(",", " ").split()
                if len(raw_cmds) == 7:
                    cmds = [float(item) for item in raw_cmds]
                else:
                    cmds = [float(row["cmd_1_6"])] * 7
            except Exception:
                continue
            rows.append((cmds, abs_ma))
    if not rows:
        raise ValueError(f"空载曲线无有效数据: {path}")
    return rows


def profile_abs_for_joint(profile, joint_idx, command_value, window_deg):
    """取该关节在相近指令角度下的空载电流包络最大值(mA)."""
    window = abs(float(window_deg))
    nearest = min(profile, key=lambda row: abs(row[0][joint_idx] - float(command_value)))
    if window <= 1e-6:
        return nearest[1][joint_idx]
    values = [
        row[1][joint_idx]
        for row in profile
        if abs(row[0][joint_idx] - float(command_value)) <= window
    ]
    if not values:
        return nearest[1][joint_idx]
    return max(values)


def contact_extra(args, profile, sent_values, abs_curr):
    """返回每根柔性手指相对空载的电流余量(mA)."""
    extra = {}
    for idx in args.flex_fingers:
        if profile:
            profile_abs = profile_abs_for_joint(profile, idx, sent_values[idx], args.profile_window_deg)
            extra[idx] = max(0.0, abs_curr[idx] - profile_abs)
        else:
            extra[idx] = abs_curr[idx]
    return extra


def compact_pose(initial, values):
    pose = list(initial)
    for idx, value in values.items():
        pose[idx] = float(value)
    return pose


def record_empty_profile(args, hand, initial, active_joints):
    """空手采曲线: 2/3/4 从初始位按 --step 走到 --max, 其余关节保持初始位.

    遇到堵转(单个电机电流超 --record-stop-ma 连续若干步)会提前停止, 避免中途卡住.
    """
    path = args.record_profile
    values = {idx: float(initial[idx]) for idx in active_joints}
    rows = []
    step_idx = 0
    jam_counts = 0
    while step_idx < args.max_steps:
        pose = compact_pose(initial, values)
        hand.set_joint_positions(pose)
        time.sleep(max(args.settle, 0.15))
        currents = read_currents(hand, max(args.samples, 3), max(args.sample_delay, 0.02))
        if currents is None:
            print(f"[采曲线][警告] step={step_idx}: 电流读取失败, 停止采集")
            break
        abs_ma = [abs(v) for v in currents]
        cmd = max(values.values())
        rows.append((cmd, abs_ma, pose, currents))
        print(
            f"[采曲线] step={step_idx:03d} cmd={cmd:5.1f} "
            + " ".join(f"{idx}:{values[idx]:.1f}" for idx in sorted(values))
            + " abs=" + " ".join(f"{v:.0f}" for v in abs_ma)
        )

        cap_map = {
            idx: (args.thumb_max if idx == THUMB_TENDON_IDX else args.max)
            for idx in values
        }
        # 已经到自身上限的关节不再继续转, 它的电流升高只是憋着, 不算整轮卡住;
        # 只有还在继续闭合的关节超限才视为真堵转.
        over_limit = [
            idx for idx in sorted(active_joints)
            if values[idx] < cap_map[idx] - 1e-6 and abs_ma[idx] >= args.record_stop_ma
        ]
        jam_counts = jam_counts + 1 if over_limit else 0
        if jam_counts >= max(1, args.record_stop_consecutive):
            print(
                "[采曲线][停止] 检测到卡住/堵转 ("
                + ",".join(f"{idx}:{abs_ma[idx]:.0f}mA" for idx in over_limit)
                + f"), 已采到 cmd={cmd:.1f}, 后面的角度跳过"
            )
            break

        moved = False
        for idx in values:
            new_value = min(cap_map[idx], values[idx] + args.step)
            if new_value > values[idx] + 1e-6:
                moved = True
            values[idx] = new_value
        if not moved:
            print(f"[采曲线] 所有关节都到达上限, 采集完成 (cmd={cmd:.1f})")
            break
        step_idx += 1

    if not rows:
        print("[采曲线][失败] 没有采到数据")
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "step", "cmd_1_6", "commands_0_6", "currents_ma", "abs_ma"])
        for idx, (cmd, abs_ma, pose, currents) in enumerate(rows):
            writer.writerow([
                f"{time.time():.3f}",
                idx,
                f"{cmd:.2f}",
                " ".join(f"{v:.1f}" for v in pose),
                " ".join(f"{v:.1f}" for v in currents),
                " ".join(f"{v:.1f}" for v in abs_ma),
            ])
    print(f"[采曲线] 已保存 {path} ({len(rows)} 行), 下次抓取用 --profile {path}")


def main():
    args = parse_args()
    if args.fast:
        args.settle = min(args.settle, 0.05)
        args.samples = 1
        args.sample_delay = 0.0
    if args.step <= 0:
        raise ValueError("--step 必须 > 0")
    if args.consecutive <= 0:
        raise ValueError("--consecutive 必须 > 0")
    if args.contact_loss_consecutive <= 0:
        raise ValueError("--contact-loss-consecutive 必须 > 0")

    active_joints = parse_index_set(args.active_joints, "--active-joints", min_value=1, max_value=6)
    flex_fingers = parse_index_set(args.flex_fingers, "--flex-fingers", min_value=3, max_value=6)
    missing = sorted(flex_fingers - active_joints)
    if missing:
        raise ValueError(f"--flex-fingers 必须包含在 --active-joints 里, 未激活: {missing}")
    if not flex_fingers:
        raise ValueError("--flex-fingers 至少需要一个 3..6 的手指")
    args.flex_fingers = flex_fingers
    args.active_joints = active_joints

    initial = parse_float_list(args.initial) if str(args.initial or "").strip() else list(INITIAL_POSE)
    release_pose = (
        parse_float_list(args.release_pose) if str(args.release_pose or "").strip() else list(OPEN_POSE)
    )

    profile = load_empty_profile(args.profile) if args.profile else None
    log_path = args.log or f"/tmp/aero_small_grasp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    hand = AeroHand(port=args.port)
    print("[小物体抓握] 已连接")
    print("[小物体抓握] 初始位: " + " ".join(f"{i}:{initial[i]:.1f}" for i in range(7)))
    print(
        "[小物体抓握] 固定关节: "
        + ",".join(str(i) for i in range(7) if i not in active_joints)
        + " | 柔性手指: "
        + ",".join(str(i) for i in sorted(flex_fingers))
    )
    print(
        f"[小物体抓握] 上限={args.max:.1f} step={args.step:.1f} extra_close={args.extra_close:.1f} "
        f"min_cmd={args.min_cmd:.1f} consecutive={args.consecutive}"
        + (f" 空载曲线={args.profile} delta={args.profile_delta_ma:.0f}mA" if profile else
           f" delta阈值={args.delta_ma:.0f}mA abs阈值={args.abs_ma:.0f}mA")
    )
    print(f"[小物体抓握] CSV日志: {log_path}")

    values = {idx: float(initial[idx]) for idx in active_joints}
    locked = {}
    hit_counts = {idx: 0 for idx in flex_fingers}
    pending_targets = {}
    pending_loss_counts = {idx: 0 for idx in flex_fingers}
    target_confirm_counts = {idx: 0 for idx in flex_fingers}
    final_pose = list(initial)
    log_rows = 0

    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "time",
                "step",
                "commands_0_6",
                "currents_ma",
                "abs_ma",
                "profile_extra_ma",
                "temperatures_c",
                "reason",
            ],
        )
        writer.writeheader()
        try:
            hand.set_joint_positions(initial)
            time.sleep(0.5)
            baseline = read_currents(hand, args.samples, args.sample_delay)
            if baseline is None:
                raise RuntimeError("无法读取基线电流")
            print("[小物体抓握] 基线电流(mA): " + " ".join(f"{v:.1f}" for v in baseline))
            temperatures = read_temperatures(hand)
            if temperatures is not None:
                print(f"[小物体抓握] 初始温度: max={max(temperatures):.1f}C motor4={temperatures[4]:.1f}C")

            if str(args.record_profile or "").strip():
                record_empty_profile(args, hand, initial, active_joints)
                return

            step_idx = 0
            while any(idx not in locked for idx in flex_fingers):
                if step_idx >= args.max_steps:
                    print(f"[小物体抓握][停止] 达到最大步数 {args.max_steps}, 未锁住: "
                          + ",".join(str(i) for i in sorted(flex_fingers - set(locked))))
                    break

                sent_values = dict(values)
                pose = compact_pose(initial, sent_values)
                final_pose = pose
                hand.set_joint_positions(pose)
                time.sleep(args.settle)
                currents = read_currents(hand, args.samples, args.sample_delay)
                if currents is None:
                    print(f"[小物体抓握][警告] step={step_idx}: 电流读取失败, 继续")
                    step_idx += 1
                    continue

                if step_idx % max(1, args.temperature_every) == 0:
                    temperatures = read_temperatures(hand)

                abs_curr = [abs(v) for v in currents]
                extra = contact_extra(args, profile, sent_values, abs_curr)
                if profile:
                    over = [idx for idx in sorted(flex_fingers) if extra[idx] >= args.profile_delta_ma]
                else:
                    over = [
                        idx for idx in sorted(flex_fingers)
                        if (abs_curr[idx] - abs(baseline[idx])) >= args.delta_ma or abs_curr[idx] >= args.abs_ma
                    ]

                events = []
                for idx in sorted(flex_fingers):
                    if idx in locked:
                        continue
                    if idx in pending_targets:
                        if idx in over:
                            pending_loss_counts[idx] = 0
                        else:
                            pending_loss_counts[idx] = pending_loss_counts.get(idx, 0) + 1
                            target_confirm_counts[idx] = 0
                            if pending_loss_counts[idx] >= max(1, args.contact_loss_consecutive):
                                target = pending_targets.pop(idx)
                                pending_loss_counts[idx] = 0
                                hit_counts[idx] = 0
                                events.append(f"contact_lost:{idx}@{sent_values[idx]:.1f}/target:{target:.1f}")
                            continue
                        if sent_values[idx] >= pending_targets[idx] - 1e-6:
                            target_confirm_counts[idx] = target_confirm_counts.get(idx, 0) + 1
                            if target_confirm_counts[idx] >= max(1, args.consecutive):
                                locked[idx] = float(sent_values[idx])
                                pending_targets.pop(idx, None)
                                pending_loss_counts[idx] = 0
                                target_confirm_counts[idx] = 0
                                events.append(f"lock:{idx}@{locked[idx]:.1f}")
                        continue

                    if sent_values[idx] < args.min_cmd:
                        hit_counts[idx] = 0
                        continue
                    hit_counts[idx] = hit_counts[idx] + 1 if idx in over else 0
                    if hit_counts[idx] >= max(1, args.consecutive):
                        target = min(args.max, sent_values[idx] + args.extra_close)
                        pending_targets[idx] = float(target)
                        pending_loss_counts[idx] = 0
                        target_confirm_counts[idx] = 0
                        events.append(f"contact:{idx}@{sent_values[idx]:.1f}->target:{target:.1f}")
                    elif sent_values[idx] >= args.max - 1e-6:
                        locked[idx] = float(sent_values[idx])
                        events.append(f"max:{idx}@{locked[idx]:.1f}")

                next_values = dict(sent_values)
                for idx in next_values:
                    if idx in locked:
                        continue
                    cap = args.thumb_max if idx == THUMB_TENDON_IDX else args.max
                    limit = min(cap, pending_targets.get(idx, cap))
                    next_values[idx] = min(float(limit), float(sent_values[idx]) + args.step)

                reason = ";".join(events)
                writer.writerow({
                    "time": f"{time.time():.3f}",
                    "step": step_idx,
                    "commands_0_6": " ".join(f"{cent:.1f}" for cent in pose),
                    "currents_ma": " ".join(f"{v:.1f}" for v in currents),
                    "abs_ma": " ".join(f"{v:.1f}" for v in abs_curr),
                    "profile_extra_ma": " ".join(f"{extra.get(i, 0.0):.1f}" for i in range(7)) if profile else "",
                    "temperatures_c": " ".join(f"{v:.1f}" for v in temperatures) if temperatures is not None else "",
                    "reason": reason,
                })
                log_rows += 1
                f.flush()

                sent_text = " ".join(f"{idx}:{sent_values[idx]:.1f}" for idx in sorted(active_joints))
                next_text = " ".join(f"{idx}:{next_values[idx]:.1f}" for idx in sorted(active_joints))
                temp_text = f" temp_max={max(temperatures):.1f}C" if temperatures is not None else ""
                print(
                    f"[小物体抓握] step={step_idx:03d} sent={sent_text} next={next_text} "
                    f"over={over or '-'} locked={sorted(locked)}"
                    + (f" extra={{{','.join(f'{i}:{extra[i]:.0f}' for i in sorted(flex_fingers))}}}mA" if profile else
                       f" max_delta={max(abs_curr[i] - abs(baseline[i]) for i in sorted(flex_fingers)):.0f}mA")
                    + (f" {reason}" if reason else "")
                    + temp_text
                )
                if temperatures is not None and max(temperatures) >= args.max_temperature_c:
                    print("[小物体抓握][温度提示] 超过阈值, 仅记录, 不改变动作")

                values = next_values
                step_idx += 1

            if all(idx in locked for idx in flex_fingers):
                print("[小物体抓握][完成] 已锁住: "
                      + ", ".join(f"{idx}:{COMPACT_NAMES[idx]}={locked[idx]:.1f}" for idx in sorted(locked)))
                hold_seconds = max(0.0, args.hold_seconds)
                print(f"[小物体抓握][保持] 保持 {hold_seconds:.1f}s ...")
                end_time = time.time() + hold_seconds
                while time.time() < end_time:
                    time.sleep(min(1.0, max(0.0, end_time - time.time())))
                    hold_currents = read_currents(hand, 1, 0.0)
                    if hold_currents is not None:
                        print("[小物体抓握][保持] 电流(mA): " + " ".join(f"{v:.0f}" for v in hold_currents))
                print("[小物体抓握][保持完成]")
        except KeyboardInterrupt:
            print("\n[小物体抓握] 用户中断")
        finally:
            if args.release_after:
                print("[小物体抓握] 张开: " + " ".join(f"{v:.1f}" for v in release_pose))
                hand.set_joint_positions(release_pose)
                time.sleep(0.3)
            elif args.restore_after:
                print("[小物体抓握] 回到初始位")
                hand.set_joint_positions(initial)
                time.sleep(0.3)
            else:
                print("[小物体抓握] 保持最后姿态: " + " ".join(f"{v:.1f}" for v in final_pose))
            print(f"[小物体抓握] 已写CSV: {log_path} ({log_rows}条数据 + 表头)")
            hand.close()
            print("[小物体抓握] 已断开")


if __name__ == "__main__":
    main()
