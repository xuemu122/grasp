"""
读取并输出当前所有舵机的状态（位置、电流、温度）。

用法：
    python mine/read_motor_status.py                                    # 默认右手配置
    python mine/read_motor_status.py --config orca_core/models/v2/orcahand_touch_left/config.yaml
"""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand


def main():
    parser = argparse.ArgumentParser(description="读取并输出所有舵机的当前状态")
    parser.add_argument(
        "--config", type=str, default=None,
        help="config.yaml 路径，默认使用右手配置",
    )
    args = parser.parse_args()

    hand = OrcaHand(config_path=args.config)

    success, msg = hand.connect()
    print(f"连接: success={success}, message={msg}")
    if not success:
        return 1

    try:
        positions = hand.get_motor_pos(as_dict=True)
        currents = hand.get_motor_current(as_dict=True)
        temps = hand.get_motor_temp(as_dict=True)

        motor_to_joint = hand.config.motor_to_joint_dict

        print(f"\n{'电机ID':>6}  {'关节名':>14}  {'位置(rad)':>12}  {'电流(mA)':>10}  {'温度(°C)':>8}")
        print("-" * 60)

        for motor_id in hand.config.motor_ids:
            joint_name = motor_to_joint.get(motor_id, "unknown")
            pos = positions.get(motor_id, float("nan"))
            cur = currents.get(motor_id, float("nan"))
            tmp = temps.get(motor_id, float("nan"))
            print(f"{motor_id:>6}  {joint_name:>14}  {pos:>12.4f}  {cur:>10.1f}  {tmp:>8.1f}")

        print()

    finally:
        success, msg = hand.disconnect()
        print(f"断开: success={success}, message={msg}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
