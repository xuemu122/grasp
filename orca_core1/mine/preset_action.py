"""
预设动作：从位置1平滑过渡到位置2，拇指后于其他四指变换。
直接使用电机位置（rad），无需校准数据。

用法：
    python mine/preset_action.py
    python mine/preset_action.py --steps 50 --delay 0.03
    python mine/preset_action.py --config orca_core/models/v2/orcahand_touch_left/config.yaml
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand

# ── 预设电机位置（rad）──────────────────────────────────
POS1 = {
    1: 3.7322, 2: 0.2930, 3: 6.6682, 4: 5.0238,
    5: 4.6848, 6: 0.1749, 7: 2.3669, 8: 1.6720,
    9: 2.8885, 10: 0.2378, 11: 6.4366, 12: 2.4329,
    13: 2.7596, 14: 1.8224, 15: 4.1157, 16: 3.9024,
    17: 0.4602,
}

POS2 = {
    1: 4.2246, 2: 2.0601, 3: 6.0868, 4: 4.8827,
    5: 3.5680, 6: 2.1322, 7: 3.7552, 8: 1.9098,
    9: 4.6096, 10: 2.0954, 11: 6.5010, 12: 3.2873,
    13: 3.6033, 14: 2.5295, 15: 4.1571, 16: 4.7584,
    17: 0.3927,
}

# 拇指对应的电机ID
THUMB_MOTORS = {14, 13, 15, 16}  # thumb_cmc, thumb_abd, thumb_mcp, thumb_dip


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
    parser = argparse.ArgumentParser(description="预设动作：拇指后动（电机空间直接插值）")
    parser.add_argument("--config", type=str, default=None, help="config.yaml 路径")
    parser.add_argument("--steps", type=int, default=50, help="插值步数")
    parser.add_argument("--delay", type=float, default=0.03, help="每步间隔(秒)")
    parser.add_argument("--grip-current", type=int, default=120, help="握紧保持电流(mA)，0=不握紧")
    parser.add_argument("--grip-time", type=float, default=3.0, help="握紧保持时间(秒)")
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

        # 先把所有电机移到位置1（确保起点一致）
        print("移动到位置1...")
        all_motors = hand.config.motor_ids
        move_motors_smooth(hand, POS1, all_motors,
                           num_steps=args.steps, step_delay=args.delay)
        time.sleep(0.5)

        # 分组电机ID
        non_thumb = [mid for mid in all_motors if mid not in THUMB_MOTORS]
        thumb = [mid for mid in all_motors if mid in THUMB_MOTORS]

        # 第一步：移动非拇指电机到位置2
        print("移动四指到位置2...")
        move_motors_smooth(hand, POS2, non_thumb,
                           num_steps=args.steps, step_delay=args.delay)

        # 第二步：移动拇指到位置2
        print("移动拇指到位置2...")
        move_motors_smooth(hand, POS2, thumb,
                           num_steps=args.steps, step_delay=args.delay)

        # 握紧保持阶段：切到纯电流模式，用低电流持续施力
        if args.grip_current > 0:
            print(f"握紧保持中... 电流={args.grip_current}mA, 时间={args.grip_time}s")
            wrist_motor = hand.config.joint_to_motor_map.get("wrist")
            grip_motors = [mid for mid in all_motors if mid != wrist_motor]
            hand.set_control_mode("current")
            hand._motor_client.write_desired_current(grip_motors, args.grip_current * np.ones(len(grip_motors)))
            try:
                time.sleep(args.grip_time)
            except KeyboardInterrupt:
                print("\n提前释放。")
            hand.set_control_mode("current_based_position")
            hand.set_max_current(hand.config.max_current)

        print("动作完成。")

    finally:
        success, msg = hand.disconnect()
        print(f"断开: success={success}, message={msg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
