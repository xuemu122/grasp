#!/usr/bin/env python3
"""
Aero Hand 灵巧手调试与路点录制工具
====================================
- 终端交互控制每根手指的每个关节
- 记录关节位姿 → 保存 YAML
- 5 种预设姿态快速切换
- 轨迹录制 & 播放
- 可调步长 (1~50%)

用法:
  cd /home/nvidia/grasp/aero-hand-open/sdk
  python examples/aero_tuner.py
  python examples/aero_tuner.py --play recorded_poses/trajectory_xxx.yaml
"""

import os
import sys
import time
import argparse
from pathlib import Path

import yaml
from aero_open_sdk.aero_hand import AeroHand

# ── 预设姿态 ──
PRESETS = {
    "1": {"desc": "全张开", "joints": [0]*16},
    "2": {"desc": "握拳", "joints": [
        0, 55, 90, 90,
        90, 90, 90, 90, 90, 90,
        90, 90, 90, 90, 90, 90,
    ]},
    "3": {"desc": "拇指-食指捏合", "joints": [
        0, 45, 70, 50,
        70, 60, 40, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
    ]},
    "4": {"desc": "食指指向 (其余握拳)", "joints": [
        0, 55, 90, 90,
        0, 0, 0,
        90, 90, 90, 90, 90, 90,
        90, 90, 90,
    ]},
    "5": {"desc": "包裹抓取 (圆柱)", "joints": [
        0, 35, 60, 60,
        60, 70, 70, 60, 70, 70,
        60, 70, 70, 60, 70, 70,
    ]},
}

OUTPUT_DIR = Path(__file__).parent / "recorded_poses"
JOINT_NAMES = [
    "thumb_abd", "thumb_flex", "thumb_mcp", "thumb_ip",
    "index_mcp", "index_pip", "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp", "ring_pip", "ring_dip",
    "pinky_mcp", "pinky_pip", "pinky_dip",
]
FINGER_IDXS = {"t": [0,1,2,3], "i": [4,5,6], "m": [7,8,9], "r": [10,11,12], "p": [13,14,15]}
FINGER_NAMES = {"t": "拇指", "i": "食指", "m": "中指", "r": "无名指", "p": "小指"}


def show_status(hand):
    """显示当前 16 关节状态"""
    pos = hand.get_joint_positions()
    print(f"\n  拇指: abd={pos[0]:5.0f}% flex={pos[1]:5.0f}% mcp={pos[2]:5.0f}% ip={pos[3]:5.0f}%")
    print(f"  食指: mcp={pos[4]:5.0f}% pip={pos[5]:5.0f}% dip={pos[6]:5.0f}%")
    print(f"  中指: mcp={pos[7]:5.0f}% pip={pos[8]:5.0f}% dip={pos[9]:5.0f}%")
    print(f"  无名: mcp={pos[10]:5.0f}% pip={pos[11]:5.0f}% dip={pos[12]:5.0f}%")
    print(f"  小指: mcp={pos[13]:5.0f}% pip={pos[14]:5.0f}% dip={pos[15]:5.0f}%")
    return pos


def save_pose(label, joints):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = OUTPUT_DIR / f"pose_{ts}_{label}.yaml"
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# Aero Hand 路点\n")
        f.write(f"# 标签: {label}\n")
        f.write(f"# 时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write(f"label: \"{label}\"\njoints:\n")
        for name, val in zip(JOINT_NAMES, joints):
            f.write(f"  {name}: {val:.1f}\n")
    print(f"  [保存] {path}")
    return path


def save_trajectory(name, waypoints):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = OUTPUT_DIR / f"traj_{name}_{ts}.yaml"
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# Aero Hand 轨迹: {name}\n")
        f.write(f"# 路点数: {len(waypoints)}\n\nwaypoints:\n")
        for wp in waypoints:
            f.write(f"  - label: \"{wp['label']}\"\n")
            f.write(f"    duration: {wp['duration']}\n")
            f.write(f"    joints: {[round(v,1) for v in wp['joints']]}\n")
    print(f"  [保存] 轨迹 ({len(waypoints)}点) → {path}")
    return path


def load_trajectory_yaml(filepath):
    with open(filepath) as f:
        data = yaml.safe_load(f)
    return data.get("waypoints", [])


def play_trajectory(hand, waypoints):
    traj = [(wp["joints"], wp["duration"]) for wp in waypoints]
    print(f"\n播放轨迹 ({len(traj)} 点)...")
    hand.run_trajectory(traj)


# ============================================================
def interactive_tune(hand):
    waypoints = []
    step = 10.0

    print("=" * 60)
    print("  Aero Hand 灵巧手调试 & 路点录制")
    print("  手指: t/i/m/r/p  预设: 1-5  h=帮助  q=退出")
    print("=" * 60)

    # 初始化张开
    hand.set_joint_positions([0]*16)
    time.sleep(0.3)

    while True:
        show_status(hand)
        print(f"\n  路点: {len(waypoints)}个  步长: ±{step:.0f}%")
        print("  [t/i/m/r/p]粗调  [s]记录  [e]出轨迹  [l]播放  [1-5]预设  [L/P]载入  [q]退出")
        raw = input("  > ").strip()
        choice = raw.lower()

        # ── 手指粗调 ──
        if choice in FINGER_IDXS:
            fgr = choice
            fname = FINGER_NAMES[fgr]
            while True:
                joints = hand.get_joint_positions()
                idxs = FINGER_IDXS[fgr]
                labels = [JOINT_NAMES[i] for i in idxs]
                print(f"\n  [{fname}] {' | '.join(f'{l}={joints[i]:.0f}%' for i,l in zip(idxs, labels))}")
                print(f"  选关节 (1-{len(idxs)})  +/-加减  回车返回  [=/-]改步长({step:.0f})")
                fcmd = input("  > ").strip().lower()
                if fcmd == "":
                    break
                if fcmd in FINGER_IDXS:
                    fgr, fname = fcmd, FINGER_NAMES[fcmd]
                    continue
                if fcmd in ("=", "+"):
                    step = min(50, step * 2)
                    continue
                if fcmd == "-":
                    step = max(1, step / 2)
                    continue
                try:
                    jid = int(fcmd)
                    if jid < 1 or jid > len(idxs):
                        continue
                    idx = idxs[jid - 1]
                    delta = input(f"    {JOINT_NAMES[idx]} +/- ? ").strip()
                    if delta in ("+", "="):
                        joints[idx] = min(100, joints[idx] + step)
                    elif delta == "-":
                        joints[idx] = max(0, joints[idx] - step)
                    else:
                        continue
                    hand.set_joint_positions(joints)
                    time.sleep(0.15)
                except ValueError:
                    pass

        # ── 预设 ──
        elif choice in PRESETS:
            p = PRESETS[choice]
            hand.set_joint_positions(p["joints"])
            print(f"  [预设] {p['desc']}")
            time.sleep(0.3)

        # ── 快速开合 ──
        elif choice == "o":
            hand.set_joint_positions([0]*16)
            time.sleep(0.3)
        elif choice == "f":
            hand.set_joint_positions(PRESETS["2"]["joints"])
            time.sleep(0.3)

        # ── 步长 ──
        elif choice == "=" or choice == "+":
            step = min(50, step * 2)
            print(f"  步长 → {step:.0f}")
        elif choice == "-":
            step = max(1, step / 2)
            print(f"  步长 → {step:.0f}")

        # ── 记录路点 ──
        elif choice == "s":
            joints = hand.get_joint_positions()
            label = input("  标签 (回车=自动): ").strip() or f"wp{len(waypoints)+1}"
            dur = input("  持续秒数 (回车=1.0): ").strip() or "1.0"
            waypoints.append({"label": label, "joints": list(joints), "duration": float(dur)})
            save_pose(label, joints)
            print(f"  [记录] #{len(waypoints)} → {label}")

        # ── 保存训练 ──
        elif choice == "e":
            if not waypoints:
                print("  没有路点!")
                continue
            name = input("  轨迹名称: ").strip() or "custom"
            save_trajectory(name, waypoints)

        # ── 播放轨迹 ──
        elif choice == "l":
            if not waypoints:
                print("  没有路点!")
                continue
            play_trajectory(hand, waypoints)

        # ── 载入文件 (区分大小写: L=载入, P=播放文件, l=播放路点) ──
        elif raw == "L":
            path = input("  YAML 文件路径: ").strip()
            if not os.path.exists(path):
                print("  文件不存在!")
                continue
            wps = load_trajectory_yaml(path)
            if not wps:
                print("  轨迹为空!")
                continue
            waypoints = wps
            print(f"  [载入] {len(waypoints)} 个路点")

        elif raw == "P":
            path = input("  YAML 文件路径: ").strip()
            if not os.path.exists(path):
                print("  文件不存在!")
                continue
            wps = load_trajectory_yaml(path)
            if not wps:
                print("  轨迹为空!")
                continue
            play_trajectory(hand, wps)

        # ── 帮助 ──
        elif choice == "h":
            print("""
  16 关节 (0~100% 行程):
    拇指: abd flex mcp ip
    食指: mcp pip dip
    中指: mcp pip dip
    无名: mcp pip dip
    小指: mcp pip dip

  操作:
    t/i/m/r/p → 选择手指, 进入关节微调
    1/2/3/4   → 选定关节, 再按 +/- 加减
    回车      → 返回主菜单
    s         → 记录当前位姿为路点
    e         → 将所有路点导出为轨迹 YAML
    l         → 播放当前路点序列
    L         → 从 YAML 文件加载路点
    P         → 从 YAML 文件加载并播放
    1-5       → 预设姿态
    +/-       → 改步长
    q         → 退出
""")

        elif choice == "q":
            hand.set_joint_positions([0]*16)
            time.sleep(0.3)
            break


def main():
    parser = argparse.ArgumentParser(description="Aero Hand 灵巧手调试 & 路点录制")
    parser.add_argument("--port", help="串口路径")
    parser.add_argument("--play", help="播放 YAML 轨迹文件 (不进入交互)")
    args = parser.parse_args()

    hand = AeroHand(port=args.port)
    print(f"[Aero Hand] 已连接, {len(JOINT_NAMES)} 关节就绪\n")

    if args.play:
        wps = load_trajectory_yaml(args.play)
        play_trajectory(hand, wps)
    else:
        try:
            interactive_tune(hand)
        except KeyboardInterrupt:
            hand.set_joint_positions([0]*16)
            time.sleep(0.3)

    hand.close()
    print("\n已退出")


if __name__ == "__main__":
    main()
