#!/usr/bin/env python3
"""Aero 手“抓握维持”独立测试（不移动机械臂）。

用法：
  python3 test_aero_hold.py --mode fixed --hold 10          # 闭合，保持10s，观察位置漂移
  python3 test_aero_hold.py --mode fixed --hold 10 --open   # 保持后反驱张开
  python3 test_aero_hold.py --mode flex  --hold 10          # 用柔性闭合（需空载曲线）
  python3 test_aero_hold.py --open-only                     # 只做反驱张开

默认是普通 Aero（AERO_SELF_LOCK_HOLD=0，100Hz 持续保持）。若用自锁版本，
启动时加 AERO_SELF_LOCK_HOLD=1 AERO_OPEN_REVERSE_DRIVE=1：闭合后只下发一次目标，
之后不再发送 CTRL_POS；保持期间脚本只读取位置/电流，不驱动。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aero_gripper import AeroGripper


def fmt(values):
    if not isinstance(values, (list, tuple)):
        return str(values)
    return "[" + ", ".join(f"{float(value):.1f}" for value in values) + "]"


def read_state(gripper):
    try:
        positions = gripper.hand.get_joint_positions_compact()
    except Exception as exc:
        positions = f"ERR {exc}"
    try:
        currents = gripper.hand.get_actuator_currents()
    except Exception as exc:
        currents = f"ERR {exc}"
    return positions, currents


def max_drift(base, current):
    if not isinstance(base, (list, tuple)) or not isinstance(current, (list, tuple)):
        return None
    if len(base) != len(current):
        return None
    return max(abs(float(a) - float(b)) for a, b in zip(base, current))


def main():
    parser = argparse.ArgumentParser(description="Aero hand grasp-hold test")
    parser.add_argument("--mode", choices=("fixed", "flex"), default="fixed",
                        help="fixed=固定 GRASP 姿态；flex=电流柔性闭合")
    parser.add_argument("--hold", type=float, default=10.0, help="保持观察时间（秒）")
    parser.add_argument("--sample", type=float, default=0.5, help="采样间隔（秒）")
    parser.add_argument("--open", action="store_true", dest="do_open",
                        help="保持结束后反驱张开")
    parser.add_argument("--open-only", action="store_true",
                        help="只执行反驱张开，不闭合")
    parser.add_argument("--port", default=None, help="Aero 串口（默认自动检测）")
    args = parser.parse_args()

    gripper = AeroGripper(port=args.port)
    gripper.connect()

    if args.open_only:
        ok = gripper.open()
        print("open", ok)
        gripper.disconnect()
        return 0 if ok else 1

    if args.mode == "flex":
        ok = gripper.flexible_close()
    else:
        ok = gripper.close()
    print("grasp", ok)
    if not ok:
        gripper.disconnect()
        return 1

    print("is_holding", gripper.is_holding())
    base_positions, base_currents = read_state(gripper)
    print("t=0.0 pos=" + fmt(base_positions) + " I=" + fmt(base_currents))

    started = time.monotonic()
    while time.monotonic() - started < args.hold:
        time.sleep(min(max(0.05, args.sample), max(0.0, args.hold - (time.monotonic() - started))))
        positions, currents = read_state(gripper)
        drift = max_drift(base_positions, positions)
        drift_text = "" if drift is None else f" drift={drift:.1f}deg"
        print(f"t={time.monotonic() - started:.1f} pos={fmt(positions)} I={fmt(currents)}{drift_text}",
              flush=True)

    if args.do_open:
        ok = gripper.open()
        print("open", ok)
        positions, _ = read_state(gripper)
        print("after open pos=" + fmt(positions))
    else:
        print("保持中：未发送驱动命令（自锁保持）。可手动拉拽或断开手电源验证。")

    gripper.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
