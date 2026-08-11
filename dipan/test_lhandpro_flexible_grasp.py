#!/usr/bin/env python3
"""Record and test LHandPro current-based flexible grasping without arm motion."""

import argparse
import os

from lhandpro_gripper import DEFAULT_PROFILE_PATH, LHandProGripper, load_empty_profile


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--transport", choices=("serial", "dobot"),
        default=os.getenv("LHAND_TRANSPORT", "serial"),
        help="serial=USB-RS485; dobot=机械臂末端RS485",
    )
    parser.add_argument("--port", default=os.getenv("LHAND_RS485_PORT"), help="例如 /dev/ttyUSB0")
    parser.add_argument("--robot-ip", default=os.getenv("DOBOT_IP", "192.168.5.1"))
    parser.add_argument("--robot-port", type=int, default=int(os.getenv("DOBOT_PORT", "29999")))
    parser.add_argument(
        "--baud", type=int, default=None,
        help="串口波特率；serial 默认500000，dobot 默认115200",
    )
    parser.add_argument("--node", type=int, default=int(os.getenv("LHAND_RS485_NODE_ID", "1")))
    parser.add_argument("--profile", default=os.getenv("LHAND_FLEX_PROFILE", DEFAULT_PROFILE_PATH))
    parser.add_argument("--home-timeout", type=float, default=float(os.getenv("LHAND_HOME_TIMEOUT_S", "45")))
    parser.add_argument("--debug", action="store_true", help="打印回零期间六轴状态")
    subparsers = parser.add_subparsers(dest="action", required=True)

    subparsers.add_parser("info", help="只连接并读取轴状态，不使能、不回零")
    subparsers.add_parser("home", help="清报警、使能并执行六轴回零，不发送位置命令")
    subparsers.add_parser("open", help="使能、回零并张开")

    record = subparsers.add_parser("record", help="空手扫位并录制空载电流曲线")
    record.add_argument("--start", type=int, default=0)
    record.add_argument("--max", dest="maximum", type=int, default=9000)
    record.add_argument("--step", type=int, default=250)
    record.add_argument("--settle", type=float, default=0.12)
    record.add_argument("--samples", type=int, default=3)
    record.add_argument("--safety-current", type=float, default=750)
    record.add_argument("--yes", action="store_true", help="跳过空手确认提示")

    flex = subparsers.add_parser("flex", help="执行一次柔性抓取，不移动机械臂")
    flex.add_argument("--yes", action="store_true", help="跳过放置物体确认提示")

    check = subparsers.add_parser("check-profile", help="只校验曲线文件，不连接硬件")
    check.add_argument("--dof", type=int, default=6)
    return parser


def require_port(parser, args):
    if not args.port:
        parser.error("该操作必须用 --port 明确指定串口，例如 --port /dev/ttyUSB0")


def confirm(prompt, expected, skip):
    if skip:
        return
    answer = input(prompt).strip()
    if answer != expected:
        raise RuntimeError("用户取消")


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.action == "check-profile":
        if not os.path.isfile(args.profile):
            print(f"曲线不存在: {args.profile}\n请先执行 record 录制空载曲线。")
            return 1
        rows = load_empty_profile(args.profile, args.dof)
        print(f"曲线有效: {args.profile}, rows={len(rows)}, range={rows[0][0]}..{rows[-1][0]}")
        return 0

    if args.transport == "serial":
        require_port(parser, args)
        baud = args.baud if args.baud is not None else int(os.getenv("LHAND_RS485_BAUD", "500000"))
        hand = LHandProGripper(
            port=args.port,
            baud_rate=baud,
            node_id=args.node,
            profile_path=args.profile,
            debug=args.debug,
        )
    else:
        from dobot_modbus_gripper import DobotModbusGripper
        baud = args.baud if args.baud is not None else int(os.getenv("LHAND_DOBOT_BAUD", "115200"))
        hand = DobotModbusGripper(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            baud_rate=baud,
            node_id=args.node,
            profile_path=args.profile,
            debug=args.debug,
        )
    try:
        if args.action == "info":
            hand.connect(enable_motors=False, home_motors=False)
            sdk = hand.controller.sdk_handle
            print(f"主动轴数: {hand.dof_active}")
            for axis in range(1, hand.dof_active + 1):
                print(
                    f"axis{axis}: enable={int(sdk.get_enable(axis))} "
                    f"status={sdk.get_now_status(axis)} alarm={sdk.get_now_alarm(axis)} "
                    f"position={sdk.get_now_position(axis)} current={sdk.get_now_current(axis)}‰"
                )
            return 0

        if args.action == "record":
            confirm(
                "确认手指范围内没有任何物体，且末端有足够空间。输入 RECORD 继续: ",
                "RECORD",
                args.yes,
            )
            hand.connect(
                enable_motors=True,
                home_motors=True,
                home_timeout_s=args.home_timeout,
            )
            if not hand.open():
                return 1
            hand.record_empty_profile(
                output_path=args.profile,
                start=args.start,
                maximum=args.maximum,
                step=args.step,
                settle_s=args.settle,
                samples=args.samples,
                safety_current=args.safety_current,
            )
            return 0

        hand.connect(
            enable_motors=True,
            home_motors=True,
            home_timeout_s=args.home_timeout,
        )
        if args.action == "home":
            return 0
        if args.action == "open":
            return 0 if hand.open() else 1

        if not hand.open():
            return 1
        confirm(
            "把软质测试物放到掌心，手和机械臂保持静止。输入 GRASP 继续: ",
            "GRASP",
            args.yes,
        )
        success = hand.flexible_close()
        print(f"柔性抓取结果: {'成功' if success else '失败'}")
        if success and not args.yes:
            input("观察保持状态，按 Enter 张开并退出: ")
        hand.open()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n已中断")
        return 130
    except Exception as exc:
        print(f"测试失败: {exc}")
        return 1
    finally:
        hand.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
