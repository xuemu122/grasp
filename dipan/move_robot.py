#!/usr/bin/env python3
"""
机器人底盘 TCP 控制脚本
通过 TCP socket 发送移动指令到机器人底盘
"""

import socket
import json
import os
import sys
import time

HOST = "192.168.124.90"
PORT = 31001

# 目标点位名称
MARKER_YAOJIA = "yaojia1"
MARKER_CHARGE = "charge_point_1F_35744"
MARKER_PLACE = os.getenv("MARKER_PLACE", "place_point")
DEFAULT_WAIT_TIMEOUT_S = int(os.getenv("MOVE_WAIT_TIMEOUT_S", "180"))


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


def get_power_status(sock):
    """查询底盘电源状态"""
    cmd = "/api/get_power_status"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 电源: {json.dumps(result, ensure_ascii=False, indent=2)}")
    return result


def print_power_summary(power_result):
    """打印简短电量信息"""
    if not power_result or power_result.get("status") != "OK":
        msg = (power_result or {}).get("error_message", "未知错误")
        print(f"电量查询失败: {msg}")
        return
    info = power_result.get("results") or {}
    capacity = info.get("battery_capacity")
    voltage = info.get("battery_voltage")
    current = info.get("battery_current")
    charging = info.get("charger_connected_notice")
    charging_text = "充电中" if charging else "未充电"
    print(f"电量: {capacity}%  电压: {voltage}V  电流: {current}A  {charging_text}")


def attach_power_status(sock, result):
    """把电源状态合并到原命令返回值中"""
    try:
        power_result = get_power_status(sock)
    except Exception as e:
        power_result = {"status": "ERROR", "error_message": str(e)}
        print(f"电量查询失败: {e}")
    else:
        print_power_summary(power_result)
    result["power_status"] = power_result
    return result


def get_robot_status(sock):
    """查询底盘全局状态"""
    cmd = "/api/robot_status"
    result = send_command(sock, cmd)
    return result


def wait_move_done(sock, marker=None, timeout_s=DEFAULT_WAIT_TIMEOUT_S):
    """等待底盘移动任务完成。成功返回 True, 失败/取消/超时返回 False。"""
    deadline = time.time() + timeout_s
    last_status = None
    while time.time() < deadline:
        try:
            result = get_robot_status(sock)
        except Exception as e:
            print(f"[底盘] 查询 robot_status 失败: {e}")
            time.sleep(1)
            continue

        if result.get("status") != "OK":
            print(f"[底盘] robot_status 异常: {result.get('error_message', '未知错误')}")
            time.sleep(1)
            continue

        info = result.get("results") or {}
        move_status = info.get("move_status")
        running_status = info.get("running_status")
        move_target = info.get("move_target")
        if (move_status, running_status, move_target) != last_status:
            print(f"[底盘] move_status={move_status} running_status={running_status} target={move_target}")
            last_status = (move_status, running_status, move_target)

        if move_status == "succeeded":
            if marker and move_target and move_target != marker:
                print(f"[底盘] 移动完成, 但目标为 {move_target}, 不是期望 {marker}")
            print("[底盘] 已到达目标点")
            return True
        if move_status in ("failed", "canceled"):
            print(f"[底盘] 移动任务结束但未成功: {move_status}")
            return False
        time.sleep(1)

    print(f"[底盘] 等待移动完成超时 ({timeout_s}s)")
    return False


def move_to(sock, marker, wait=False):
    """发送移动到目标点位的指令"""
    cmd = f"/api/move?marker={marker}"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 返回: {json.dumps(result, ensure_ascii=False, indent=2)}")
    if result.get("status") == "OK":
        print(f"任务已下发，task_id: {result.get('task_id', 'N/A')}")
    else:
        print(f"失败: {result.get('error_message', '未知错误')}")
    result = attach_power_status(sock, result)
    if wait and result.get("status") == "OK":
        result["move_done"] = wait_move_done(sock, marker=marker)
    return result


def cancel_move(sock):
    """取消当前移动任务"""
    cmd = "/api/move/cancel"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 返回: {json.dumps(result, ensure_ascii=False, indent=2)}")
    return attach_power_status(sock, result)


def main():
    args = sys.argv[1:]
    wait = "--no-wait" not in args
    args = [arg for arg in args if arg != "--no-wait"]
    target = args[0] if args else None

    if target and target not in ("yaojia1", "charge", "place", "cancel"):
        print("用法:")
        print("  python3 move_robot.py yaojia1   # 药架1, 默认等待到达")
        print("  python3 move_robot.py charge    # 去充电桩, 默认等待到达")
        print("  python3 move_robot.py place     # 去放置固定点, 默认等待到达")
        print("  python3 move_robot.py place --no-wait")
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
            result = move_to(sock, MARKER_YAOJIA, wait=wait)
            if wait and result.get("move_done") is False:
                sys.exit(2)
        elif target == "charge":
            if not MARKER_CHARGE:
                print("错误: 充电桩点位名称未设置，请先编辑脚本填写 MARKER_CHARGE")
                sys.exit(1)
            result = move_to(sock, MARKER_CHARGE, wait=wait)
            if wait and result.get("move_done") is False:
                sys.exit(2)
        elif target == "place":
            if not MARKER_PLACE:
                print("错误: 放置固定点名称未设置，请设置 MARKER_PLACE 或编辑脚本")
                sys.exit(1)
            result = move_to(sock, MARKER_PLACE, wait=wait)
            if wait and result.get("move_done") is False:
                sys.exit(2)
        else:
            # 交互模式
            print("交互模式 (输入 q 退出):")
            print(f"  1 - 去 {MARKER_YAOJIA}")
            print(f"  2 - 去充电桩 ({MARKER_CHARGE or '未设置'})")
            print(f"  3 - 去放置固定点 ({MARKER_PLACE or '未设置'})")
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
                elif choice == "3":
                    if not MARKER_PLACE:
                        print("错误: 放置固定点名称未设置")
                        continue
                    move_to(sock, MARKER_PLACE)
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
