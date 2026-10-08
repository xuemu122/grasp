#!/usr/bin/env python3
"""DOBOT Nova5 TCP control helpers."""

import os
import re
import socket
import time
from typing import Optional, Tuple

from self_collision_guard import Nova5SelfCollisionGuard, describe_self_collision
from tool_geometry import tool_size_label_mm


DOBOT_IP = os.getenv("DOBOT_IP", "192.168.5.1")
DOBOT_PORT = 29999
HAND_WEIGHT = float(os.getenv("HAND_WEIGHT_KG", "0.6"))
HAND_COG_X = float(os.getenv("HAND_COG_X_M", "0.0"))
HAND_COG_Y = float(os.getenv("HAND_COG_Y_M", "0.0"))
HAND_COG_Z = float(os.getenv("HAND_COG_Z_M", "0.12"))
HOME_JOINTS_DEG = [-44.1, -20.2, 134.6, -112.7, -46.0, 90.6]
ROBOT_POLL_S = float(os.getenv("ROBOT_POLL_S", "0.2"))
SELF_COLLISION_ENABLE = os.getenv("SELF_COLLISION_ENABLE", "1").lower() in ("1", "true", "yes", "on")
SELF_COLLISION_MARGIN_M = float(os.getenv("SELF_COLLISION_MARGIN_M", "0.015"))
SELF_COLLISION_PATH_SAMPLES = int(os.getenv("SELF_COLLISION_PATH_SAMPLES", "8"))
SELF_COLLISION_REQUIRE_IK = os.getenv("SELF_COLLISION_REQUIRE_IK", "1").lower() in ("1", "true", "yes", "on")
SELF_COLLISION_FAIL_CLOSED = os.getenv("SELF_COLLISION_FAIL_CLOSED", "1").lower() in ("1", "true", "yes", "on")
SELF_COLLISION_VERBOSE = os.getenv("SELF_COLLISION_VERBOSE", "0").lower() in ("1", "true", "yes", "on")
DOBOT_IK_VERBOSE = os.getenv("DOBOT_IK_VERBOSE", "0").lower() in ("1", "true", "yes", "on")


def _clamp_speed_pct(speed) -> int:
    try:
        return max(1, min(100, int(round(float(speed)))))
    except (TypeError, ValueError):
        return 100


def _clamp_cp(cp) -> int:
    try:
        return max(0, min(100, int(round(float(cp)))))
    except (TypeError, ValueError):
        return 0


def _motion_poll_count(timeout_s: float = 30.0) -> int:
    interval = max(0.05, ROBOT_POLL_S)
    return max(1, int(round(timeout_s / interval)))


class DobotArm:
    def __init__(self, ip=DOBOT_IP, port=DOBOT_PORT):
        self.ip = ip
        self.port = port
        self.sock = None
        self._last_pose = None
        self.self_collision_guard = None
        if SELF_COLLISION_ENABLE:
            try:
                self.self_collision_guard = Nova5SelfCollisionGuard(margin_m=SELF_COLLISION_MARGIN_M)
                print(
                    f"[自碰撞] Nova5 URDF 护栏已启用 "
                    f"(margin={SELF_COLLISION_MARGIN_M*1000:.0f}mm, "
                    f"camera_margin={self.self_collision_guard.camera_margin_m*1000:.0f}mm, "
                    f"samples={SELF_COLLISION_PATH_SAMPLES}, tool={tool_size_label_mm()})"
                )
            except Exception as e:
                message = f"[自碰撞][错误] 完整机械臂/灵巧手/相机模型初始化失败: {e}"
                if SELF_COLLISION_FAIL_CLOSED:
                    raise RuntimeError(message) from e
                print(message + ", 已显式配置为放行")

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3)
        self.sock.connect((self.ip, self.port))
        print(f"[DOBOT] 已连接 {self.ip}:{self.port}")
        resp = self.send("ClearError()")
        print(f"[DOBOT] ClearError: {resp}")
        resp = self.send("RequestControl()")
        print(f"[DOBOT] RequestControl: {resp}")
        resp = self.send("PowerOn()")
        print(f"[DOBOT] PowerOn: {resp}  (等待 10 秒上电...)")
        time.sleep(10)

        resp = self.send(
            f"SetPayload({HAND_WEIGHT}, {HAND_COG_X*1000:.0f}, "
            f"{HAND_COG_Y*1000:.0f}, {HAND_COG_Z*1000:.0f})"
        )
        print(f"[DOBOT] SetPayload: {resp}")

        resp = self.send("EnableRobot()")
        print(f"[DOBOT] EnableRobot: {resp}")
        time.sleep(0.5)

        resp = self.send("RobotMode()")
        print(f"[DOBOT] RobotMode: {resp}")
        if self._is_enabled_mode(resp):
            print("[DOBOT] 使能确认")
        else:
            mode = self._parse_robot_mode(resp)
            print(f"[DOBOT] 警告! RobotMode={mode if mode is not None else '?'} 未进入使能状态")
            self.do_enable()

    @staticmethod
    def _parse_robot_mode(resp: str) -> Optional[int]:
        match = re.search(r"\{(-?\d+)\}", resp)
        if not match:
            return None
        return int(match.group(1))

    @classmethod
    def _is_enabled_mode(cls, resp: str) -> bool:
        mode = cls._parse_robot_mode(resp)
        return mode in (5, 6, 7, 8)

    def do_enable(self):
        for attempt in range(3):
            self.send("ClearError()")
            time.sleep(0.3)
            self.send("RequestControl()")
            time.sleep(0.3)
            resp = self.send("RobotMode()")
            print(f"[DOBOT] RobotMode: {resp}")
            if self._is_enabled_mode(resp):
                print("[DOBOT] 已处于使能状态, 跳过重复上电等待")
                return True
            self.send("PowerOn()")
            time.sleep(0.5)
            resp = self.send("RobotMode()")
            mode = self._parse_robot_mode(resp)
            if mode in (3, 4, None):
                print(f"[DOBOT] 等待上电 10s (重试 {attempt+1}/3)...")
                time.sleep(10)
            else:
                print(f"[DOBOT] RobotMode={mode}, 跳过完整上电等待")
            self.send(
                f"SetPayload({HAND_WEIGHT}, {HAND_COG_X*1000:.0f}, "
                f"{HAND_COG_Y*1000:.0f}, {HAND_COG_Z*1000:.0f})"
            )
            resp = self.send("EnableRobot()")
            print(f"[DOBOT] EnableRobot: {resp}")
            time.sleep(0.5)
            resp = self.send("RobotMode()")
            print(f"[DOBOT] RobotMode: {resp}")
            if self._is_enabled_mode(resp):
                print("[DOBOT] 使能成功!")
                return True
        print("[DOBOT] 使能失败! 请手动检查机械臂状态")
        return False

    def send(self, cmd: str) -> str:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.sock.sendall(cmd.encode())
        resp = b""
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
                if b";" in chunk or b"}" in chunk:
                    break
            except socket.timeout:
                break
        return resp.decode().strip()

    def reconnect(self):
        print("[DOBOT] 重连中...")
        try:
            self.sock.close()
        except Exception:
            pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(3)
        self.sock.connect((self.ip, self.port))
        self.do_enable()

    def get_pose(self) -> Tuple[float, float, float, float, float, float]:
        for attempt in range(3):
            resp = self.send("GetPose()")
            nums = re.findall(r"[-+]?\d*\.?\d+", resp)
            if len(nums) >= 6:
                pose = tuple(map(float, nums[-6:]))
                self._last_pose = pose
                return pose
            print(f"  [GetPose 重试 {attempt+1}]: {resp}")
            time.sleep(0.3)
        if self._last_pose is not None:
            print("  [GetPose] 用缓存位姿")
            return self._last_pose
        raise RuntimeError(f"无法解析位姿: {resp}")

    def get_joints(self) -> Tuple[float, float, float, float, float, float]:
        resp = self.send("GetAngle()")
        nums = re.findall(r"[-+]?\d*\.?\d+", resp)
        if len(nums) >= 6:
            return tuple(map(float, nums[-6:]))
        raise RuntimeError(f"无法解析关节角: {resp}")

    def get_error_id(self) -> str:
        return self.send("GetErrorID()")

    def _check_self_collision_path(self, current_joints, target_joints, label: str = "目标关节") -> bool:
        if self.self_collision_guard is None:
            if SELF_COLLISION_ENABLE and SELF_COLLISION_FAIL_CLOSED:
                print(f"  [自碰撞][中止] {label}: 碰撞模型不可用")
                return False
            return True
        try:
            result = self.self_collision_guard.check_path(current_joints, target_joints, samples=SELF_COLLISION_PATH_SAMPLES)
        except Exception as e:
            action = "中止" if SELF_COLLISION_FAIL_CLOSED else "放行"
            print(f"  [自碰撞][{action}] {label}: 检查失败: {e}")
            return not SELF_COLLISION_FAIL_CLOSED
        if result.ok:
            if SELF_COLLISION_VERBOSE:
                print(f"  [自碰撞] {label}: {describe_self_collision(result)}")
            return True
        print(f"  [自碰撞][中止] {label}: {describe_self_collision(result)}")
        return False

    def _check_self_collision_joints(self, target_joints, label: str = "目标关节") -> bool:
        if self.self_collision_guard is None:
            if SELF_COLLISION_ENABLE and SELF_COLLISION_FAIL_CLOSED:
                print(f"  [自碰撞][中止] {label}: 碰撞模型不可用")
                return False
            return True
        try:
            current = self.get_joints()
        except Exception as e:
            action = "中止" if SELF_COLLISION_FAIL_CLOSED else "放行"
            print(f"  [自碰撞][{action}] {label}: 无法读取当前关节角: {e}")
            return not SELF_COLLISION_FAIL_CLOSED
        return self._check_self_collision_path(current, target_joints, label=label)

    def _check_self_collision_pose(self, pose, label: str = "目标位姿") -> bool:
        if self.self_collision_guard is None:
            if SELF_COLLISION_ENABLE and SELF_COLLISION_FAIL_CLOSED:
                print(f"  [自碰撞][中止] {label}: 碰撞模型不可用")
                return False
            return True
        try:
            current = self.get_joints()
        except Exception as e:
            action = "中止" if SELF_COLLISION_FAIL_CLOSED else "放行"
            print(f"  [自碰撞][{action}] {label}: 无法读取当前关节角: {e}")
            return not SELF_COLLISION_FAIL_CLOSED
        target_joints = self.inverse_kin_near(pose, near=current)
        if target_joints is None:
            msg = f"  [自碰撞][警告] {label}: 未能求近邻逆解, 无法做自碰撞检查"
            if SELF_COLLISION_REQUIRE_IK:
                print(msg + ", 中止")
                return False
            print(msg + ", 放行")
            return True
        return self._check_self_collision_path(current, target_joints, label=label)

    def clear_alarm(self):
        print("[DOBOT] 清除报警并重新使能...")
        self.send("ClearError()")
        time.sleep(0.5)
        err = self.get_error_id()
        print(f"[DOBOT] GetErrorID: {err}")
        self.do_enable()

    def movel(self, x_mm, y_mm, z_mm, rx=None, ry=None, rz=None, speed=100, allow_low_z=False,
              cp=0, wait=True):
        try:
            speed_pct = _clamp_speed_pct(speed)
            cp_pct = _clamp_cp(cp)
            if rx is None:
                pose = self.get_pose()
                rx, ry, rz = pose[3], pose[4], pose[5]
            if z_mm < -400:
                print(f"  [拒绝!] Z={z_mm:.0f}mm 太低, 可能碰撞, 跳过低抓取")
                return False
            if z_mm < 50:
                print(f"  [低位放行] Z={z_mm:.0f}mm (第4层/下方工作区)")
            if z_mm > 1100 or z_mm < -400:
                print(f"  [警告] Z={z_mm:.0f}mm 可能超出工作范围, 请注意!")
            target_pose = (float(x_mm), float(y_mm), float(z_mm), float(rx), float(ry), float(rz))
            if not self._check_self_collision_pose(target_pose, label="MovL"):
                return False
            cmd = f"MovL(pose={{{x_mm:.1f},{y_mm:.1f},{z_mm:.1f},{rx:.1f},{ry:.1f},{rz:.1f}}},v={speed_pct},cp={cp_pct})"
            print(f"  >>> {cmd}")
            resp = self.send(cmd)
            print(f"  <<< {resp}")
            if resp.startswith("-"):
                print(f"  [MovL 被拒绝] 错误码={resp.split(',')[0]}, 可能是位姿不可达")
                return False
            if not wait:
                return True
            return self._wait_motion_done("MovL", clear_alarm=True)
        except Exception as e:
            print(f"  [MovL 异常] {e}")
            return False

    def go_home(self, speed=100):
        return self.movj(HOME_JOINTS_DEG, speed=speed)

    def move_pose_safe(self, pose, joints=None, speed=80, cp=0, wait=True):
        if joints is not None:
            print("  [安全过渡] 使用记录关节角 MovJ")
            return self.movj(joints, speed=speed, cp=cp, wait=wait)
        near_joints = self.inverse_kin_near(pose)
        if near_joints is not None:
            print("  [安全过渡] 使用当前关节附近逆解 MovJ")
            return self.movj(near_joints, speed=speed, cp=cp, wait=wait)
        print("  [安全过渡] 未能求近邻逆解, 回退 pose MovJ")
        return self.movj_pose(pose, speed=speed, cp=cp, wait=wait)

    def inverse_kin_near(self, pose, near=None, verbose=None):
        try:
            if near is None:
                near = self.get_joints()
            if verbose is None:
                verbose = DOBOT_IK_VERBOSE
            p = pose
            cmd = (
                f"InverseKin({p[0]:.1f},{p[1]:.1f},{p[2]:.1f},{p[3]:.1f},{p[4]:.1f},{p[5]:.1f},"
                f"useJointNear=1,jointNear={{{near[0]:.1f},{near[1]:.1f},{near[2]:.1f},{near[3]:.1f},{near[4]:.1f},{near[5]:.1f}}})"
            )
            if verbose:
                print(f"  >>> {cmd}")
            resp = self.send(cmd)
            if verbose:
                print(f"  <<< {resp}")
            if resp.startswith("-"):
                return None
            match = re.match(r"^-?\d+,\{([^{}]+)\},InverseKin", resp)
            if not match:
                return None
            nums = re.findall(r"[-+]?\d*\.?\d+", match.group(1))
            if len(nums) >= 6:
                return list(map(float, nums[:6]))
        except Exception as e:
            print(f"  [InverseKin 异常] {e}")
        return None

    def movj_pose(self, pose, speed=80, cp=0, wait=True):
        p = pose
        speed_pct = _clamp_speed_pct(speed)
        cp_pct = _clamp_cp(cp)
        if not self._check_self_collision_pose(p, label="MovJ(pose)"):
            return False
        cmd = f"MovJ(pose={{{p[0]:.1f},{p[1]:.1f},{p[2]:.1f},{p[3]:.1f},{p[4]:.1f},{p[5]:.1f}}},v={speed_pct},cp={cp_pct})"
        print(f"  >>> {cmd}")
        resp = self.send(cmd)
        print(f"  <<< {resp}")
        if resp.startswith("-"):
            print(f"  [MovJ(pose) 被拒绝] 错误码={resp.split(',')[0]}")
            return False
        if not wait:
            return True
        return self._wait_motion_done("MovJ(pose)")

    def movj(self, joints_deg, speed=100, cp=0, wait=True):
        j = joints_deg
        speed_pct = _clamp_speed_pct(speed)
        cp_pct = _clamp_cp(cp)
        if not self._check_self_collision_joints(j, label="MovJ"):
            return False
        cmd = f"MovJ(joint={{{j[0]:.1f},{j[1]:.1f},{j[2]:.1f},{j[3]:.1f},{j[4]:.1f},{j[5]:.1f}}},v={speed_pct},cp={cp_pct})"
        print(f"  >>> {cmd}")
        resp = self.send(cmd)
        print(f"  <<< {resp}")
        if resp.startswith("-"):
            print(f"  [MovJ 被拒绝] 错误码={resp.split(',')[0]}")
            return False
        if not wait:
            return True
        return self._wait_motion_done("MovJ")

    def _wait_motion_done(self, label: str, clear_alarm: bool = False) -> bool:
        poll_s = max(0.05, ROBOT_POLL_S)
        seen_moving = False
        for _ in range(_motion_poll_count()):
            time.sleep(poll_s)
            err = self.get_error_id()
            if "[]" not in err:
                print(f"  [报警!] {label} 触发报警: {err}")
                if clear_alarm:
                    print("  → 自动 clear_alarm() 恢复...")
                self.clear_alarm()
                return False
            rm = self.send("RobotMode()")
            mode = self._parse_robot_mode(rm)
            if mode in (7, 8):
                seen_moving = True
            waited_s = (_ + 1) * poll_s
            if mode == 5 and (seen_moving or waited_s >= 0.5):
                return True
            if mode in (4, 9, 11):
                print(f"  [{label} 异常状态] RobotMode={mode}, 停止后续动作")
                return False
        print(f"  [{label} 超时] 运动 30s 未完成")
        return False

    def move_j6(self, target_j6_deg: float):
        joints = list(self.get_joints())
        old_j6 = joints[5]
        joints[5] = target_j6_deg
        print(f"  MovJ J6 {old_j6:.1f}° → {target_j6_deg:.1f}°")
        return self.movj(joints, speed=80)

    def close(self):
        sock = self.sock
        if sock is None:
            return
        try:
            response = self.send("DisableRobot()")
            print(f"[DOBOT] DisableRobot: {response}")
        finally:
            try:
                sock.close()
            finally:
                self.sock = None
                print("[DOBOT] 已断开")
