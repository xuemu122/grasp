"""
预设动作5：从初始位置平稳过渡到结束位置。
直接使用电机位置（rad），基于实测数据。

用法：
    python mine/preset_action5.py
    python mine/preset_action5.py --steps 50 --delay 0.03
    python mine/preset_action5.py --reverse
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
POS1 = {  # 初始位置
    1: 3.8534, 2: 1.4711, 3: 0.0859, 4: 4.9517,
    5: 4.9962, 6: 1.3990, 7: 2.7428, 8: 4.6127,
    9: 3.9730, 10: 1.0999, 11: 2.9268, 12: 2.6845,
    13: 3.2076, 14: 2.0064, 15: 4.1970, 16: 4.7998,
    17: 0.3712,
}

POS2 = {  # 结束位置
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
    parser = argparse.ArgumentParser(description="预设动作5：基于实测数据的平滑过渡")
    parser.add_argument("--config", type=str, default=None, help="config.yaml 路径")
    parser.add_argument("--steps", type=int, default=50, help="插值步数")
    parser.add_argument("--delay", type=float, default=0.03, help="每步间隔(秒)")
    parser.add_argument("--hold", type=float, default=10.0, help="到达目标后保持时间(秒)")
    parser.add_argument("--reverse", action="store_true", help="反向：从结束位置回到初始位置")
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

        src, dst = (POS2, POS1) if args.reverse else (POS1, POS2)

        # 先移到起始位置
        all_motors = hand.config.motor_ids
        print(f"移动到起始位置...")
        move_motors_smooth(hand, src, all_motors,
                           num_steps=args.steps, step_delay=args.delay)
        time.sleep(0.5)

        # 所有电机同时移动到目标位置
        print("移动所有手指...")
        move_motors_smooth(hand, dst, all_motors,
                           num_steps=args.steps, step_delay=args.delay)

        # 保持目标位置一段时间，确保电机物理到位
        print(f"保持目标位置 {args.hold}s...")
        time.sleep(args.hold)

        print("动作完成。")

    finally:
        success, msg = hand.disconnect()
        print(f"断开: success={success}, message={msg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
