#!/usr/bin/env python3
"""第 7 关节：串口总线舵机（FTcontrol 协议）封装。

协议（来自 FTcontrol/duo6u0.py、duo11.py）：
- 串口总线舵机，默认 /dev/servo（udev 软链），1,000,000 bps，ID=1。
- 位置 0..4095 对应 0..360°（180°=2048）。
- 写角度顺序：先写加速度 0x29 → 目标速度 0x2E → 目标位置 0x2A（duo11.py 的平滑做法）。
- 寄存器：使能 0x28、加速度 0x29、目标位置 0x2A、目标速度 0x2E、当前位置 0x38、
  当前负载 0x3C、Moving 0x42（0=停/到位，1=在动）。

环境变量：
- SERVO_ENABLE      0 关闭（默认开）
- SERVO_PORT        默认 /dev/servo
- SERVO_BAUD        默认 1000000
- SERVO_ID          默认 1
- SERVO_SPEED       默认速度，默认 800
- SERVO_ACC         默认加速度 0..254，越小越柔和，默认 20
- SERVO_WAIT_TIMEOUT 到位等待超时（秒），默认 1.5
- SERVO_POS_TOL_DEG 位置容差（度，退化判据），默认 3

连接失败不抛异常：`ok=False`，所有操作安全空转，抓取主程序照常运行。
"""

import os
import threading
import time

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

PORT = os.getenv("SERVO_PORT", "/dev/servo")
BAUD = int(os.getenv("SERVO_BAUD", "1000000"))
SERVO_ID = int(os.getenv("SERVO_ID", "1"))
DEFAULT_SPEED = int(os.getenv("SERVO_SPEED", "800"))
DEFAULT_ACC = int(os.getenv("SERVO_ACC", "20"))
WAIT_TIMEOUT = float(os.getenv("SERVO_WAIT_TIMEOUT", "1.5"))
POS_TOL_DEG = float(os.getenv("SERVO_POS_TOL_DEG", "3"))
ENABLE = os.getenv("SERVO_ENABLE", "").strip().lower() not in ("0", "false", "no", "off")

ADDR_TORQUE_ENABLE = 0x28
ADDR_ACC = 0x29
ADDR_GOAL_POSITION_L = 0x2A
ADDR_GOAL_SPEED_L = 0x2E
ADDR_PRESENT_POSITION_L = 0x38
ADDR_PRESENT_LOAD_L = 0x3C
ADDR_MOVING = 0x42

POS_MAX = 4095
DEG_MAX = 360.0


def _checksum(buf):
    return (~sum(buf[2:])) & 0xFF


def _build_write(servo_id, addr, value):
    cmd = [0xFF, 0xFF, servo_id, 0x05, 0x03, addr, value & 0xFF, (value >> 8) & 0xFF]
    return bytes(cmd + [_checksum(cmd)])


def _build_write_1byte(servo_id, addr, value):
    cmd = [0xFF, 0xFF, servo_id, 0x04, 0x03, addr, value & 0xFF]
    return bytes(cmd + [_checksum(cmd)])


def _build_read(servo_id, addr, length):
    cmd = [0xFF, 0xFF, servo_id, 0x04, 0x02, addr, length]
    return bytes(cmd + [_checksum(cmd)])


def _angle_diff(a, b):
    diff = abs(float(a) - float(b)) % DEG_MAX
    return min(diff, DEG_MAX - diff)


def angle_to_pos(angle):
    return max(0, min(POS_MAX, int(round(float(angle) / DEG_MAX * (POS_MAX + 1)))))


def pos_to_angle(pos):
    return float(pos) / (POS_MAX + 1) * DEG_MAX


class ServoJoint:
    def __init__(self, port=None, baud=None, servo_id=None, speed=None, acc=None):
        self.port = port or PORT
        self.baud = int(baud or BAUD)
        self.servo_id = int(servo_id or SERVO_ID)
        self.speed = int(speed if speed is not None else DEFAULT_SPEED)
        self.acc = int(acc if acc is not None else DEFAULT_ACC)
        self.ser = None
        self.ok = False
        self._lock = threading.Lock()

    def connect(self):
        if not ENABLE:
            print("[舵机] SERVO_ENABLE=0，已跳过")
            return False
        if serial is None:
            print("[舵机][警告] 未安装 pyserial，已跳过")
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(0.2)
            self._tx_rx(_build_write_1byte(self.servo_id, ADDR_TORQUE_ENABLE, 1), 16)
            self.ok = True
            print(f"[舵机] 已连接 {self.port} @ {self.baud}, id={self.servo_id}, "
                  f"speed={self.speed}, acc={self.acc}")
            return True
        except Exception as exc:
            self.ser = None
            self.ok = False
            print(f"[舵机][警告] 连接失败，已跳过: {exc}")
            return False

    def _tx_rx(self, cmd, read_len):
        if self.ser is None:
            return b""
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                time.sleep(0.01)
                return self.ser.read(read_len)
            except Exception as exc:
                print(f"[舵机][警告] 通信失败: {exc}")
                return b""

    def set_angle(self, angle, speed=None):
        """转到指定角度（0..360，180=中点）：先写加速度→速度→位置。"""
        if not self.ok or self.ser is None:
            return False
        pos = angle_to_pos(angle)
        spd = int(self.speed if speed is None else speed) & 0xFFFF
        acc = int(self.acc) & 0xFF
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(_build_write_1byte(self.servo_id, ADDR_ACC, acc))
                time.sleep(0.005)
                self.ser.write(_build_write(self.servo_id, ADDR_GOAL_SPEED_L, spd))
                time.sleep(0.005)
                self.ser.write(_build_write(self.servo_id, ADDR_GOAL_POSITION_L, pos))
                time.sleep(0.01)
                self.ser.read(32)
                return True
            except Exception as exc:
                print(f"[舵机][警告] 设置角度失败: {exc}")
                return False

    def get_angle(self):
        if not self.ok or self.ser is None:
            return None
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(_build_read(self.servo_id, ADDR_PRESENT_POSITION_L, 2))
                time.sleep(0.01)
                resp = self.ser.read(8)
            except Exception:
                return None
        if len(resp) >= 7 and resp[0] == 0xFF and resp[1] == 0xFF:
            return pos_to_angle(resp[5] | (resp[6] << 8))
        return None

    def is_moving(self):
        """读 Moving(0x42)：True=在动、False=停/到位；读不到返回 None。"""
        if not self.ok or self.ser is None:
            return None
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(_build_read(self.servo_id, ADDR_MOVING, 1))
                time.sleep(0.01)
                resp = self.ser.read(8)
            except Exception:
                return None
        if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
            return bool(resp[5])
        return None

    def wait_until_reached(self, angle, timeout=None):
        """等待转到目标角度：优先 Moving(0x42)==0，退化用位置(0x38)容差。"""
        if not self.ok or self.ser is None:
            return False
        timeout = float(WAIT_TIMEOUT if timeout is None else timeout)
        time.sleep(0.05)  # 启动保护，避免刚下发就读到“尚未开始动”
        deadline = time.time() + timeout
        stop_count = 0
        while time.time() < deadline:
            moving = self.is_moving()
            if moving is not None:
                stop_count = stop_count + 1 if not moving else 0
            else:
                pos = self.get_angle()
                if pos is not None and _angle_diff(pos, angle) <= POS_TOL_DEG:
                    stop_count += 1
                else:
                    stop_count = 0
            if stop_count >= 2:
                return True
            time.sleep(0.02)
        return False

    def close(self):
        if self.ser is not None:
            try:
                self._tx_rx(_build_write_1byte(self.servo_id, ADDR_TORQUE_ENABLE, 0), 16)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            print("[舵机] 已断开")
        self.ok = False
        self.ser = None
