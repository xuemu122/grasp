#!/usr/bin/env python3
"""
机器人底盘 TCP 控制脚本
通过 TCP socket 发送移动指令到机器人底盘
"""

import socket
import json
import sys

HOST = "192.168.124.90"
PORT = 31001

# 目标点位名称
MARKER_YAOJIA = "yaojia1"
MARKER_CHARGE = "charge_point_1F_35744"


def send_command(sock, cmd):
    """发送指令并接收返回"""
    sock.sendall((cmd + "\n").encode())
    resp = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        resp += chunk
        try:
            # 尝试解析 JSON，成功则表示接收完整
            json.loads(resp.decode())
            break
        except json.JSONDecodeError:
            continue
    return json.loads(resp.decode())


def move_to(sock, marker):
    """发送移动到目标点位的指令"""
    cmd = f"/api/move?marker={marker}"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 返回: {json.dumps(result, ensure_ascii=False, indent=2)}")
    if result.get("status") == "OK":
        print(f"任务已下发，task_id: {result.get('task_id', 'N/A')}")
    else:
        print(f"失败: {result.get('error_message', '未知错误')}")
    return result


def cancel_move(sock):
    """取消当前移动任务"""
    cmd = "/api/move/cancel"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 返回: {json.dumps(result, ensure_ascii=False, indent=2)}")


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else None

    if target and target not in ("yaojia1", "charge", "cancel"):
        print("用法:")
        print("  python3 move_robot.py yaojia1   # 药架1")
        print("  python3 move_robot.py charge    # 去充电桩")
        print("  python3 move_robot.py cancel    # 取消当前任务")
        print("  python3 move_robot.py           # 交互模式")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)

    try:
        sock.connect((HOST, PORT))
        print(f"已连接到 {HOST}:{PORT}")

        if target == "cancel":
            cancel_move(sock)
        elif target == "yaojia1":
            move_to(sock, MARKER_YAOJIA)
        elif target == "charge":
            if not MARKER_CHARGE:
                print("错误: 充电桩点位名称未设置，请先编辑脚本填写 MARKER_CHARGE")
                sys.exit(1)
            move_to(sock, MARKER_CHARGE)
        else:
            # 交互模式
            print("交互模式 (输入 q 退出):")
            print(f"  1 - 去 {MARKER_YAOJIA}")
            print(f"  2 - 去充电桩 ({MARKER_CHARGE or '未设置'})")
            print(f"  c - 取消任务")
            while True:
                choice = input("\n请选择: ").strip()
                if choice == "q":
                    break
                elif choice == "1":
                    move_to(sock, MARKER_YAOJIA)
                elif choice == "2":
                    if not MARKER_CHARGE:
                        print("错误: 充电桩点位名称未设置")
                        continue
                    move_to(sock, MARKER_CHARGE)
                elif choice == "c":
                    cancel_move(sock)

    except socket.timeout:
        print("连接超时")
    except ConnectionRefusedError:
        print(f"连接被拒绝，请检查 {HOST}:{PORT} 是否可达")
    finally:
        sock.close()
        print("连接已关闭")


if __name__ == "__main__":
    main()
