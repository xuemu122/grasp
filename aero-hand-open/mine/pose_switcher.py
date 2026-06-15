"""
手指动作控制 — 按键直连模式

每个动作绑定独立按键，按下即执行。

按键说明:
  1  完全张开
  2  动作一
  3  动作二
  Q  退出
"""

import time
from aero_open_sdk.aero_hand import AeroHand

PORT = "ttyACM0"

# ============================================================
# 动作一: [0.997,0.266,0.162,0.256,0.294,0.246,0.267]
# thumb_abd: 0.997×100=99.7  thumb_flex: 0.266×55=14.6
# thumb_mcp: 0.162×90=14.6   index: 0.256×90=23.0
# middle: 0.294×90=26.5      ring: 0.246×90=22.1  pinky: 0.267×90=24.0
#
# 动作二: [0.963,0.329,0.583,0.554,0.642,0.660,0.591]
# thumb_abd: 0.963×100=96.3  thumb_flex: 0.329×55=18.1
# thumb_mcp: 0.583×90=52.5   index: 0.554×90=49.9
# middle: 0.642×90=57.8      ring: 0.660×90=59.4  pinky: 0.591×90=53.2
# ============================================================
ACTION_MAP = {
    "1": ("完全张开", [  0,  0,  0,  0,  0,  42, 0]),
    "2": ("动作一",   [0, 8, 22, 24, 26, 64, 0.0]),
    "3": ("动作二",   [ 0, 30, 52, 70, 58, 59, 53]),
}


def print_menu():
    print("\n" + "=" * 35)
    print("  灵巧手抓握控制")
    print("=" * 35)
    for key, (name, _) in ACTION_MAP.items():
        print(f"  {key} - {name}")
    print("  Q - 退出")
    print("=" * 35)


if __name__ == "__main__":
    hand = AeroHand(port=PORT)
    print(f"连接 {PORT} 成功")
    print_menu()

    try:
        import msvcrt

        while True:
            print("\n按键: ", end="", flush=True)
            ch = msvcrt.getch().decode("utf-8", errors="ignore").lower()

            if ch in ("q", "\x1b"):
                break

            elif ch in ACTION_MAP:
                name, joints = ACTION_MAP[ch]
                hand.set_joint_positions(joints)
                print(f"→ {name}")

            else:
                print(f"[!] 无效按键 '{ch}'，请按 1/2/3/Q")

    except ImportError:
        # 非 Windows
        while True:
            ch = input("按键 (1/2/3=动作, Q=退出): ").strip().lower()
            if ch == "q":
                break
            elif ch in ACTION_MAP:
                name, joints = ACTION_MAP[ch]
                hand.set_joint_positions(joints)
                print(f"→ {name}")
            else:
                print("[!] 无效按键")

    finally:
        hand.close()
        print("串口已关闭，程序退出")
