#!/usr/bin/env python3
"""
机器人底盘 TCP 控制脚本
通过 TCP socket 发送移动指令到机器人底盘
"""

import socket
import json
import math
import os
import sys
import time

HOST = os.getenv("CHASSIS_HOST", os.getenv("MOVE_ROBOT_HOST", "192.168.10.10"))
PORT = int(os.getenv("CHASSIS_PORT", os.getenv("MOVE_ROBOT_PORT", "31001")))

# 目标点位名称
MARKER_SHELF = os.getenv("MARKER_SHELF", "huojia1")
MARKER_CHARGE = "charge_point_1F_40953"
MARKER_PLACE = os.getenv("MARKER_PLACE", "fangzhi")
DEFAULT_WAIT_TIMEOUT_S = int(os.getenv("MOVE_WAIT_TIMEOUT_S", "180"))
MARKER_DISTANCE_TOLERANCE_M = float(os.getenv("CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M", "0.15"))
MARKER_THETA_TOLERANCE_RAD = float(os.getenv("CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD", "0.25"))
CHARGE_RETURN_ON_DOCKING = os.getenv("CHARGE_RETURN_ON_DOCKING", "1").lower() in ("1", "true", "yes", "on")
MARKER_ARRIVAL_STABLE_POLLS = max(1, int(os.getenv("CHASSIS_MARKER_ARRIVAL_STABLE_POLLS", "2")))
MOVE_POLL_INTERVAL_S = max(0.1, float(os.getenv("CHASSIS_NAVIGATION_POLL_MS", "1000")) / 1000.0)
STATUS_POLL_TIMEOUT_S = max(0.2, float(os.getenv("CHASSIS_STATUS_POLL_TIMEOUT_MS", "2000")) / 1000.0)


def response_results(result):
    """Return the nested API results object when present."""
    if isinstance(result, dict) and isinstance(result.get("results"), dict):
        return result["results"]
    return {}


def response_field(result, key, default=None):
    """Read a field from either the top level or results.* for API version compatibility."""
    if isinstance(result, dict) and key in result:
        return result.get(key, default)
    return response_results(result).get(key, default)


def response_error_message(result):
    """Build a useful error string from inconsistent chassis API responses."""
    if not isinstance(result, dict):
        return "无返回"
    status = result.get("status") or "UNKNOWN"
    message = result.get("error_message") or response_results(result).get("error_message") or ""
    return f"status={status}" + (f" error={message}" if message else "")


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
        msg = response_error_message(power_result)
        print(f"电量查询失败: {msg}")
        return
    info = response_results(power_result)
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


def get_robot_status_fresh():
    """使用独立短连接查询状态，避免移动期间的长连接阻塞后续轮询。"""
    with socket.create_connection((HOST, PORT), timeout=STATUS_POLL_TIMEOUT_S) as status_sock:
        status_sock.settimeout(STATUS_POLL_TIMEOUT_S)
        return get_robot_status(status_sock)


def normalize_angle(angle):
    value = float(angle or 0.0)
    while value > 3.141592653589793:
        value -= 6.283185307179586
    while value < -3.141592653589793:
        value += 6.283185307179586
    return value


def marker_theta(orientation):
    orientation = orientation or {}
    z = float(orientation.get("z") or 0.0)
    w = float(orientation.get("w") or 1.0)
    return normalize_angle(2.0 * math.atan2(z, w))


def marker_target(sock, marker_name):
    """读取 marker 的目标位姿，供导航前检查和轮询复用。"""
    markers_result = send_command(sock, "/api/markers/query_list")
    if markers_result.get("status") != "OK":
        return None
    markers = response_results(markers_result)
    marker = markers.get(marker_name)
    if marker is None:
        for candidate in markers.values():
            if isinstance(candidate, dict) and candidate.get("marker_name") == marker_name:
                marker = candidate
                break
    position = (marker or {}).get("pose", {}).get("position", {})
    if not marker or not all(key in position for key in ("x", "y")):
        return None
    orientation = (marker or {}).get("pose", {}).get("orientation", {})
    return {
        "x": float(position["x"]),
        "y": float(position["y"]),
        "theta": marker_theta(orientation),
        "floor": int((marker or {}).get("floor") or 0),
    }


def status_at_marker(status, target):
    """按实时位姿判断底盘是否进入 marker 容差范围。"""
    if not target:
        return False
    info = response_results(status) if isinstance(status, dict) and "results" in status else status
    pose = (info or {}).get("current_pose") or {}
    if not all(key in pose for key in ("x", "y")):
        return False
    marker_floor = int(target.get("floor") or 0)
    current_floor = int((info or {}).get("current_floor") or 0)
    if marker_floor and current_floor and marker_floor != current_floor:
        return False
    distance = math.hypot(float(pose["x"]) - target["x"], float(pose["y"]) - target["y"])
    theta_distance = abs(normalize_angle(float(pose.get("theta") or 0.0) - target["theta"]))
    return distance <= MARKER_DISTANCE_TOLERANCE_M and theta_distance <= MARKER_THETA_TOLERANCE_RAD


def marker_info(sock, marker_name):
    """读取 marker 和当前位姿，用于放置前判断是否已经在目标点。"""
    target = marker_target(sock, marker_name)
    if target is None:
        return None
    status_result = get_robot_status_fresh()
    if status_result.get("status") != "OK":
        return None
    return status_at_marker(status_result, target)


def wait_move_done(sock, marker=None, timeout_s=DEFAULT_WAIT_TIMEOUT_S, arrive_on_docking=False,
                   target=None):
    """等待底盘移动任务完成。成功返回 True, 失败/取消/超时返回 False。"""
    deadline = time.time() + timeout_s
    last_status = None
    last_query_error = None
    seen_current_task = False
    seen_stale_status = None
    stable_pose_polls = 0
    while time.time() < deadline:
        try:
            result = get_robot_status_fresh()
        except Exception as e:
            err = str(e)
            if err != last_query_error:
                print(f"[底盘] robot_status 暂不可用: {err}")
                last_query_error = err
            time.sleep(MOVE_POLL_INTERVAL_S)
            continue

        if result.get("status") != "OK":
            err = response_error_message(result)
            if err != last_query_error:
                print(f"[底盘] robot_status 暂不可用: {err}")
                last_query_error = err
            time.sleep(MOVE_POLL_INTERVAL_S)
            continue

        last_query_error = None
        info = response_results(result)
        move_status = info.get("move_status")
        running_status = info.get("running_status")
        move_target = info.get("move_target")
        if (move_status, running_status, move_target) != last_status:
            print(f"[底盘] move_status={move_status} running_status={running_status} target={move_target}")
            last_status = (move_status, running_status, move_target)

        target_matches = (not marker) or (move_target == marker)
        if target_matches and move_status in ("running", "succeeded"):
            seen_current_task = True

        if target_matches and arrive_on_docking and running_status == "dock_to_charging_pile":
            print("[底盘] 已到充电桩附近, 开始自动对接充电桩")
            return True

        if not arrive_on_docking and status_at_marker(info, target):
            stable_pose_polls += 1
            if stable_pose_polls >= MARKER_ARRIVAL_STABLE_POLLS:
                print(f"[底盘] 实时位姿连续 {stable_pose_polls} 次进入目标容差，确认已到达 {marker}")
                return True
        else:
            stable_pose_polls = 0

        if move_status == "succeeded":
            if marker and not target_matches and not seen_current_task:
                stale = (move_status, move_target)
                if stale != seen_stale_status:
                    print(f"[底盘] 忽略上一轮完成状态: target={move_target}, 当前等待 {marker}")
                    seen_stale_status = stale
                time.sleep(MOVE_POLL_INTERVAL_S)
                continue
            if marker and not target_matches:
                print(f"[底盘] 移动完成, 但目标为 {move_target}, 不是期望 {marker}")
                return False
            print("[底盘] 已到达目标点")
            return True
        if move_status in ("failed", "canceled"):
            if marker and not target_matches and not seen_current_task:
                stale = (move_status, move_target)
                if stale != seen_stale_status:
                    print(f"[底盘] 忽略上一轮结束状态: status={move_status} target={move_target}, 当前等待 {marker}")
                    seen_stale_status = stale
                time.sleep(MOVE_POLL_INTERVAL_S)
                continue
            print(f"[底盘] 移动任务结束但未成功: {move_status}")
            return False
        time.sleep(MOVE_POLL_INTERVAL_S)

    print(f"[底盘] 等待移动完成超时 ({timeout_s}s)")
    return False


def move_to(sock, marker, wait=False, arrive_on_docking=False):
    """发送移动到目标点位的指令"""
    target = None
    try:
        if wait:
            target = marker_target(sock, marker)
            status_result = get_robot_status_fresh() if target is not None else None
            if status_result and status_result.get("status") == "OK" and status_at_marker(status_result, target):
                print(f"[底盘] 已在目标点 {marker}，跳过导航")
                return {"status": "OK", "move_done": True, "skipped": True, "marker": marker}
    except Exception as exc:
        print(f"[底盘] 无法判断当前位置，将继续导航: {exc}")
    cmd = f"/api/move?marker={marker}"
    print(f">>> 发送: {cmd}")
    result = send_command(sock, cmd)
    print(f"<<< 返回: {json.dumps(result, ensure_ascii=False, indent=2)}")
    if result.get("status") == "OK":
        task_id = response_field(result, "task_id", "N/A")
        print(f"任务已下发，task_id: {task_id}")
    else:
        print(f"失败: {response_error_message(result)}")
    result = attach_power_status(sock, result)
    if wait and result.get("status") == "OK":
        result["move_done"] = wait_move_done(
            sock,
            marker=marker,
            arrive_on_docking=arrive_on_docking,
            target=target,
        )
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
    check_only = "--check-only" in args
    args = [arg for arg in args if arg != "--no-wait"]
    args = [arg for arg in args if arg != "--check-only"]
    target = args[0] if args else None

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)

    try:
        sock.connect((HOST, PORT))
        print(f"已连接到 {HOST}:{PORT}")

        if target == "cancel":
            cancel_move(sock)
        elif target == "charge":
            if not MARKER_CHARGE:
                print("错误: 充电桩点位名称未设置，请先编辑脚本填写 MARKER_CHARGE")
                sys.exit(1)
            result = move_to(sock, MARKER_CHARGE, wait=wait, arrive_on_docking=CHARGE_RETURN_ON_DOCKING)
            if wait and result.get("move_done") is False:
                sys.exit(2)
        elif target == "place":
            if not MARKER_PLACE:
                print("错误: 放置固定点名称未设置，请设置 MARKER_PLACE 或编辑脚本")
                sys.exit(1)
            if check_only:
                at_marker = marker_info(sock, MARKER_PLACE)
                print(f"[底盘] 放置点 {MARKER_PLACE}: {'已到达' if at_marker is True else '未到达或无法确认'}")
                if at_marker is not True:
                    sys.exit(3)
            else:
                result = move_to(sock, MARKER_PLACE, wait=wait)
                if wait and result.get("move_done") is False:
                    sys.exit(2)
        elif target:
            result = move_to(sock, target, wait=wait)
            if wait and result.get("move_done") is False:
                sys.exit(2)
        else:
            # 交互模式
            print("交互模式 (输入 q 退出):")
            print(f"  1 - 去 {MARKER_SHELF}")
            print(f"  2 - 去充电桩 ({MARKER_CHARGE or '未设置'})")
            print(f"  3 - 去放置固定点 ({MARKER_PLACE or '未设置'})")
            print(f"  c - 取消任务")
            while True:
                choice = input("\n请选择: ").strip()
                if choice == "q":
                    break
                elif choice == "1":
                    move_to(sock, MARKER_SHELF)
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
