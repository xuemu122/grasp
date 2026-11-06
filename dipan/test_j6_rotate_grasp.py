#!/usr/bin/env python3
"""抓饮料 -> 末端 J6 转到放置角(约90°) -> 等 5 秒 -> 转回 -> 松手.

使用流程:
  1. 手动把机械臂摆到抓取位(手对准饮料);
  2. 运行本脚本;
  3. 脚本先在当前位置合手, 再把 J6 相对抓取角旋转 --j6-delta(默认 +90°),
     保持 --hold-seconds(默认 5s), 再转回抓取角, 最后张手.

只做这个循环, 不移动其它关节, 不改点位.
"""

import argparse
import os
import sys
import time
import math
from pathlib import Path


DIPAN_DIR = os.environ.get("GRASP_DIPAN_DIR", "/home/nvidia/grasp/dipan")
sys.path.insert(0, DIPAN_DIR)




def parse_args():
    parser = argparse.ArgumentParser(description="饮料抓取 + J6 放置角旋转测试")
    parser.add_argument("--ip", default=os.getenv("DOBOT_IP", "192.168.5.1"), help="DOBOT IP")
    parser.add_argument("--j6-delta", type=float, default=90.0, help="J6 相对抓取角的旋转角度; 默认 +90°")
    parser.add_argument("--hold-seconds", type=float, default=5.0, help="转到放置角后保持秒数; 默认 5s")
    parser.add_argument("--grasp-wait", type=float, default=1.0, help="合手后等待秒数; 默认 1s")
    parser.add_argument("--speed", type=float, default=10.0, help="J6 旋转速度百分比; 默认 10")
    parser.add_argument("--skip-hand", action="store_true", help="不接灵巧手, 只测 J6 旋转")
    parser.add_argument("--hand-port", default=os.getenv("AERO_PORT", ""), help="Aero 手串口; 默认自动识别")
    parser.add_argument(
        "--mode",
        default="",
        choices=("", "bottle", "top"),
        help="手抓取模式: 空=普通抓取, bottle=瓶子, top=上端",
    )
    parser.add_argument("--yes", action="store_true", help="不询问确认, 直接开始")
    parser.add_argument('--method', choices=('original','direct','linear','smooth'), default='original')
    parser.add_argument('--tau', type=float, default=1.0)
    parser.add_argument('--extra-close-deg', type=float, default=3.0)
    parser.add_argument('--middle-extra-close-deg', type=float, default=9.0)
    parser.add_argument('--max-command-deg', type=float, default=80.0)
    parser.add_argument('--preview', action='store_true', help='Show parameters without hardware imports')
    parser.add_argument('--log-current', action='store_true', help='Log seven actuator currents at 10 Hz during the comparison')
    parser.add_argument('--log-dir', type=Path, default=Path(__file__).resolve().parent/'transition_logs')
    return parser.parse_args()


def describe(joints):
    return "{" + ",".join(f"{v:.1f}" for v in joints) + "}"


def main():
    args = parse_args()
    values=(args.tau,args.extra_close_deg,args.middle_extra_close_deg,args.max_command_deg,
            args.speed,args.grasp_wait,args.hold_seconds,args.j6_delta)
    if not all(math.isfinite(v) for v in values) or args.tau<=0 or min(values[1:4])<0 or not 0<args.speed<=100 or args.grasp_wait<0 or args.hold_seconds<0:
        raise ValueError('Invalid command/timing setting')
    print('Method:', args.method, 'Extra:',args.extra_close_deg,'Middle:',args.middle_extra_close_deg,
          'tau:',args.tau,'Rotation delay:',args.grasp_wait,'J6 relative delta:',args.j6_delta)
    print('Original bottle grasp and arm interface retained. Not physical K/D impedance control.')
    if args.log_current and (args.method == 'original' or args.skip_hand or args.mode != 'bottle'):
        raise ValueError('--log-current requires --mode bottle and direct, linear or smooth')
    if args.log_current:
        print('[电流] 计划采样 10 Hz，记录七路电机电流 (mA)。')
    if args.preview:
        print('PREVIEW ONLY: no hardware connection or commands.');return
    if args.method!='original':
        if args.skip_hand or args.mode!='bottle':raise ValueError('Transition comparison requires --mode bottle and hand connection')
        if os.getenv('AERO_KD_SHADOW_CONFIG'):raise ValueError('Clear old AERO_KD_SHADOW_CONFIG before this position-transition test')
    from dobot_arm import DobotArm
    from pose_transition import PoseTransition
    transition=None

    if not args.yes:
        print("请确认: 已知抓取位及相对+90度往返路径可行，手对准饮料、线缆清空，返回自动张开处有可靠托盘。")
        input("回车开始, Ctrl-C 取消 ... ")

    arm = DobotArm(ip=args.ip)
    hand = None
    released = False
    try:
        print(f"[J6测试] 连接 DOBOT {args.ip} ...")
        arm.connect()

        if not args.skip_hand:
            from aero_gripper import AeroGripper

            hand = AeroGripper(port=args.hand_port or None)
            hand.open()
        else:
            print("[J6测试] skip-hand: 不连接灵巧手")

        joints = list(arm.get_joints())
        j6_grasp = joints[5]
        print(f"[J6测试] 抓取位关节: {describe(joints)}")

        if hand is not None:
            mode = args.mode or None
            if mode == "top":
                ok = hand.close(top=True)
            elif mode == "bottle":
                ok = hand.close(mode="bottle")
            else:
                ok = hand.close()
            if not ok:
                raise RuntimeError("灵巧手抓取失败")
            print(f"[J6测试] 已合手 ({args.mode or 'normal'}), 等待 {args.grasp_wait:.1f}s")
            if args.method!='original':
                log_file=args.log_dir/('position_'+args.method+'_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl')
                transition=PoseTransition(hand,args.method,args.tau,args.extra_close_deg,
                                          args.middle_extra_close_deg,args.max_command_deg,log_file,
                                          current_hz=10 if args.log_current else 0)
                print('[调度] 实际追加量:',[b-a for a,b in zip(transition.base,transition.target)])
                print('[调度] 日志:',log_file)
                transition.start()
                transition.wait(args.grasp_wait)
            else:
                time.sleep(max(0.0,args.grasp_wait))

        target = list(joints)
        target[5] = j6_grasp + args.j6_delta
        print(f"[J6测试] J6 {j6_grasp:.1f}° → {target[5]:.1f}° (delta={args.j6_delta:+.1f}°)")
        if transition:
            transition.check();transition.write(dict(event='rotation_request',j6_target=target[5]))
        if not arm.movj(target, speed=args.speed):
            raise RuntimeError("J6 转到放置角失败")

        print(f"[J6测试] 保持在放置角 {args.hold_seconds:.1f}s ...")
        end_time = time.time() + max(0.0, args.hold_seconds)
        while time.time() < end_time:
            if transition:transition.check()
            time.sleep(min(0.05, max(0.0, end_time - time.time())))

        back = list(joints)
        print(f"[J6测试] J6 转回抓取角 {back[5]:.1f}° ...")
        if transition:transition.check();transition.write(dict(event='return_request',j6_target=back[5]))
        if not arm.movj(back, speed=args.speed):
            raise RuntimeError("J6 转回抓取角失败")

        if transition:
            transition.check();transition.close();transition=None
        if hand is not None:
            if hand.open() is not True:raise RuntimeError('Release not acknowledged')
            released = True
            print("[J6测试] 已松手")

        print("[J6测试] 完成")
    except KeyboardInterrupt:
        print("\n[J6测试] 用户中断")
    except Exception as exc:
        print(f"[J6测试] 失败: {exc}")
        raise
    finally:
        try:
            if transition:
                transition.close()
        finally:
            try:
                if hand is not None:
                    if not released:
                        print("[J6测试][注意] 未松手, 手保持当前姿态")
                    hand.disconnect()
            finally:
                arm.close()


if __name__ == "__main__":
    main()
