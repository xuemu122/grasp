"""
演示程序：终端命令控制机械手抓取。
输入 1 抓取，输入 2 结束。

用法：
    python mine/demo.py
    python mine/demo.py --steps 40 --delay 0.03
"""

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand

# ── 预设电机位置（rad），基于实测数据 ─────────────────────
POS_OPEN = {  # 张开位置（准备抓取）
    1: 3.8534, 2: 1.4711, 3: 0.0859, 4: 4.9517,
    5: 4.9962, 6: 1.3990, 7: 2.7428, 8: 4.6127,
    9: 3.9730, 10: 1.0999, 11: 2.9268, 12: 2.6845,
    13: 3.2076, 14: 2.0064, 15: 4.1970, 16: 4.7998,
    17: 0.3712,
}

POS_GRAB = {  # 抓取位置
    1: 5.0207, 2: 2.6615, 3: 0.9833, 4: 5.5822,
    5: 4.1556, 6: 2.6952, 7: 3.6386, 8: 2.7796,
    9: 4.9317, 10: 2.2381, 11: 2.8962, 12: 3.6202,
    13: 3.9807, 14: 3.1631, 15: 4.1433, 16: 5.0499,
    17: 0.3313,
}


def move_motors_smooth(
    hand: OrcaHand,
    target: dict[int, float],
    motor_ids: list[int],
    num_steps: int = 30,
    step_delay: float = 0.02,
):
    """在电机空间中，将指定电机从当前位置平滑插值到目标位置。"""
    current = hand.get_motor_pos(as_dict=True)
    starts = {mid: current[mid] for mid in motor_ids}
    deltas = {mid: target[mid] - starts[mid] for mid in motor_ids}

    for step in range(1, num_steps + 1):
        alpha = step / num_steps
        cmd = {mid: starts[mid] + alpha * deltas[mid] for mid in motor_ids}
        hand._set_motor_pos(cmd)
        time.sleep(step_delay)


def main():
    parser = argparse.ArgumentParser(description="演示：终端命令控制抓取")
    parser.add_argument("--config", type=str, default=None, help="config.yaml 路径")
    parser.add_argument("--steps", type=int, default=40, help="插值步数")
    parser.add_argument("--delay", type=float, default=0.03, help="每步间隔(秒)")
    parser.add_argument("--hold", type=float, default=2.0, help="抓取后保持时间(秒)")
    args = parser.parse_args()

    hand = OrcaHand(config_path=args.config)
    success, msg = hand.connect()
    print(f"连接: success={success}, message={msg}")
    if not success:
        return 1

    try:
        hand.enable_torque()
        hand.set_control_mode("current_based_position")
        hand.set_max_current(hand.config.max_current)

        all_motors = hand.config.motor_ids

        # 先移动到张开位置
        print("初始化: 移动到张开位置...")
        move_motors_smooth(hand, POS_OPEN, all_motors,
                           num_steps=args.steps, step_delay=args.delay)
        time.sleep(0.5)

        print("\n=== 就绪 ===")
        print("输入 1 - 抓取")
        print("输入 2 - 结束")
        print("========================\n")

        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == "1":
                print("正在抓取...")
                move_motors_smooth(hand, POS_GRAB, all_motors,
                                   num_steps=args.steps, step_delay=args.delay)
                print(f"保持 {args.hold}s...")
                time.sleep(args.hold)
                print("正在张开...")
                move_motors_smooth(hand, POS_OPEN, all_motors,
                                   num_steps=args.steps, step_delay=args.delay)
                print("完成，可再次抓取\n")

            elif cmd == "2":
                print("结束程序。")
                break

            elif cmd:
                print("无效输入，请输入 1 或 2\n")

    finally:
        success, msg = hand.disconnect()
        print(f"断开: success={success}, message={msg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
