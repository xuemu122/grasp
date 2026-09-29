#!/usr/bin/env python3
"""电池控制板 485 监控进程。

协议（厂家说明）：
- RS485 / 5V TTL，115200 8N1，JSON 文本，单包不超过 250 字节。
- 板子周期上报，例如：
  {"feng1":"0","feng2":"0","jdq1":"0","jdq2":"0","ut2":"0",
   "temperature":"45","temperature1":"528.430","temperature2":"999","battery":"87.429"}
- `battery` 为电池电压，单位 V。

本进程只读不写：自动扫描串口找到电池板，持续读取电压并换算百分比，
原子写入 runtime/battery.json 供后端读取。

常用环境变量：
- BATTERY_SERIAL        指定串口（如 /dev/ttyTHS1）；不填则自动扫描
- BATTERY_BAUD          波特率，默认 115200
- BATTERY_SKIP_PORTS    扫描时跳过的串口（逗号分隔，避免和灵巧手抢串口）
- BATTERY_FULL_V        满电电压，默认 52.0
- BATTERY_EMPTY_V       空电电压，默认 42.0
- BATTERY_STATE_FILE    状态文件，默认 dipan/runtime/battery.json
- BATTERY_SCAN_USB      设为 1 时才扫描 /dev/ttyUSB*、/dev/ttyACM*（默认不扫，避免抢灵巧手串口）
- BATTERY_SCAN_TTYS     设为 1 时才扫描 /dev/ttyS*（默认不扫，避免干扰控制台串口）

没装电池板的机器：扫描不到就会一直静默重试，`/api/device/info` 里
`power.battery.online=false`，不会报错；也可用 `BATTERY_MONITOR_ENABLE=0` 直接不启动。
"""

import glob
import json
import os
import re
import time

import serial
from serial.tools import list_ports

DIPAN_DIR = os.path.dirname(os.path.abspath(__file__))
RUNTIME_DIR = os.path.join(DIPAN_DIR, "runtime")

BAUD = int(os.getenv("BATTERY_BAUD", "115200"))
STATE_FILE = os.path.abspath(os.path.expanduser(
    os.getenv("BATTERY_STATE_FILE", os.path.join(RUNTIME_DIR, "battery.json"))))
EXPLICIT_PORT = os.getenv("BATTERY_SERIAL", "").strip()
SKIP_PORTS = {p.strip() for p in os.getenv("BATTERY_SKIP_PORTS", "").split(",") if p.strip()}
FULL_V = float(os.getenv("BATTERY_FULL_V", "52.0"))
EMPTY_V = float(os.getenv("BATTERY_EMPTY_V", "42.0"))
SCAN_LISTEN_S = float(os.getenv("BATTERY_SCAN_LISTEN_S", "2.5"))
RESCAN_DELAY_S = float(os.getenv("BATTERY_RESCAN_DELAY_S", "5"))
SCAN_USB = os.getenv("BATTERY_SCAN_USB", "").lower() in ("1", "true", "yes", "on")
SCAN_TTYS = os.getenv("BATTERY_SCAN_TTYS", "").lower() in ("1", "true", "yes", "on")

_JSON_RE = re.compile(rb"\{[^{}]*\}")
_BOARD_KEYS = {
    "battery", "feng1", "feng2", "jdq1", "jdq2", "ut2",
    "temperature", "temperature1", "temperature2",
}


def candidate_ports():
    if EXPLICIT_PORT:
        return [EXPLICIT_PORT]
    ports = []
    # 默认只扫盒子的板载 485（ttyTHS*），不动 USB，避免抢灵巧手串口
    patterns = ["/dev/ttyTHS*"]
    if SCAN_USB:
        patterns += ["/dev/ttyUSB*", "/dev/ttyACM*"]
    if SCAN_TTYS:
        patterns.append("/dev/ttyS*")
    for pattern in patterns:
        for device in sorted(glob.glob(pattern)):
            if device not in ports:
                ports.append(device)
    if SCAN_USB:
        for info in list_ports.comports():
            if info.device and info.device not in ports:
                ports.append(info.device)
    return [p for p in ports if p not in SKIP_PORTS]


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_board_packet(text):
    for match in _JSON_RE.finditer(text.encode("utf-8", "ignore")):
        try:
            obj = json.loads(match.group(0).decode("utf-8"))
        except Exception:
            continue
        if isinstance(obj, dict) and _BOARD_KEYS & set(obj.keys()):
            return obj
    return None


def battery_percent(voltage):
    if voltage is None or FULL_V <= EMPTY_V:
        return None
    ratio = (voltage - EMPTY_V) / (FULL_V - EMPTY_V)
    return max(0, min(100, int(round(ratio * 100))))


def write_state(payload):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    payload = dict(payload)
    payload["updated_at"] = time.time()
    payload["full_v"] = FULL_V
    payload["empty_v"] = EMPTY_V
    temp = f"{STATE_FILE}.{os.getpid()}.tmp"
    with open(temp, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False)
    os.replace(temp, STATE_FILE)


def handle_packet(port, packet):
    voltage = to_float(packet.get("battery"))
    percent = battery_percent(voltage)
    write_state({
        "online": True,
        "port": port,
        "voltage": voltage,
        "percent": percent,
        "temperature": to_float(packet.get("temperature")),
        "temperature1": to_float(packet.get("temperature1")),
        "temperature2": to_float(packet.get("temperature2")),
        "raw": packet,
    })
    print(f"[电池] {port} 电压={voltage}V 电量={percent}% "
          f"温度={packet.get('temperature')}C", flush=True)


def try_port(port):
    """打开串口并在 SCAN_LISTEN_S 内尝试解析一帧电池数据；成功返回已打开的串口。"""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception:
        return None
    deadline = time.time() + SCAN_LISTEN_S
    buffer = b""
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            buffer += data
            packet = parse_board_packet(buffer.decode("utf-8", "ignore"))
            if packet:
                handle_packet(port, packet)
                return ser
        else:
            time.sleep(0.02)
    try:
        ser.close()
    except Exception:
        pass
    return None


def stream(ser, port):
    """持续读取已识别到的电池板。"""
    buffer = b""
    while True:
        data = ser.read(256)
        if not data:
            time.sleep(0.02)
            continue
        buffer += data
        if len(buffer) > 4096:
            buffer = buffer[-2048:]
        packet = parse_board_packet(buffer.decode("utf-8", "ignore"))
        if packet:
            handle_packet(port, packet)
            buffer = b""


def main():
    print(f"[电池] 启动: baud={BAUD} full={FULL_V}V empty={EMPTY_V}V state={STATE_FILE}", flush=True)
    last_notice = None

    def notice(message, error):
        nonlocal last_notice
        write_state({"online": False, "error": error})
        if message != last_notice:
            print(f"{message}，{RESCAN_DELAY_S:.0f}s 后重试", flush=True)
            last_notice = message

    while True:
        ports = candidate_ports()
        if not ports:
            notice("[电池] 未找到可用串口", "未找到可用串口")
            time.sleep(RESCAN_DELAY_S)
            continue
        identified = False
        for port in ports:
            ser = try_port(port)
            if ser is None:
                continue
            identified = True
            last_notice = None
            print(f"[电池] 已连接 {port}", flush=True)
            try:
                stream(ser, port)
            except Exception as exc:
                print(f"[电池] {port} 读取中断: {exc}", flush=True)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            break
        if not identified:
            notice("[电池] 未识别到电池板", "未识别到电池板")
            time.sleep(RESCAN_DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[电池] 退出", flush=True)
