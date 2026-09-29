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
- BATTERY_CHEMISTRY     电池类型，默认 lifepo4（磷酸铁锂，按单体查表）；设 linear 用满/空电压线性换算
- BATTERY_CELLS         串联节数，默认 15（48V 铁锂 = 15S）
- BATTERY_FULL_V        线性模式满电电压，默认 54.75
- BATTERY_EMPTY_V       线性模式空电电压，默认 37.5
- BATTERY_SOC_TABLE     自定义 SOC 表，如 "37.5:0,46:20,48.9:50,51:100"（整包电压:百分比），优先于内置表
- BATTERY_STATE_FILE    状态文件，默认 dipan/runtime/battery.json
- BATTERY_SCAN_USB      设为 1 时才扫描 /dev/ttyUSB*、/dev/ttyACM*（默认不扫，避免抢灵巧手串口）
- BATTERY_SCAN_TTYS     设为 1 时才扫描 /dev/ttyS*（默认不扫，避免干扰控制台串口）

没装电池板的机器：扫描不到就会一直静默重试，`/api/device/info` 里
`power.battery.online=false`，不会报错；也可用 `BATTERY_MONITOR_ENABLE=0` 直接不启动。

离线测试用法：
- `python3 battery_monitor.py --probe`  列出/逐个扫描所有串口，打印收到的原始数据，
  用来确认电池板接在哪个口（不受默认“只扫 ttyTHS”限制）。
- `python3 battery_monitor.py --once`   正常扫描，收到一帧就打印并退出。
"""

import glob
import json
import os
import re
import sys
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
FULL_V = float(os.getenv("BATTERY_FULL_V", "54.75"))
EMPTY_V = float(os.getenv("BATTERY_EMPTY_V", "37.5"))
CHEMISTRY = os.getenv("BATTERY_CHEMISTRY", "lifepo4").strip().lower()
CELLS = max(1, int(os.getenv("BATTERY_CELLS", "15")))
# 磷酸铁锂单体静置电压 -> SOC（%），曲线中段很平，必须查表
LIFEPO4_CELL_SOC = [
    (2.50, 0), (3.00, 10), (3.20, 20), (3.22, 30), (3.25, 40),
    (3.26, 50), (3.27, 60), (3.30, 70), (3.32, 80), (3.35, 90), (3.40, 100),
]


def _parse_soc_table(raw):
    """解析自定义 SOC 表，如 '37.5:0,45:10,48:50,51:100'（整包电压:百分比）。"""
    table = []
    for part in str(raw or "").split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        voltage, percent = part.split(":", 1)
        try:
            table.append((float(voltage), float(percent)))
        except ValueError:
            continue
    table.sort(key=lambda item: item[0])
    return table


CUSTOM_SOC_TABLE = _parse_soc_table(os.getenv("BATTERY_SOC_TABLE", ""))
SCAN_LISTEN_S = float(os.getenv("BATTERY_SCAN_LISTEN_S", "2.5"))
RESCAN_DELAY_S = float(os.getenv("BATTERY_RESCAN_DELAY_S", "5"))
PROBE_LISTEN_S = float(os.getenv("BATTERY_PROBE_LISTEN_S", "3"))
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
    if voltage is None:
        return None
    if CUSTOM_SOC_TABLE:
        table = CUSTOM_SOC_TABLE
    elif CHEMISTRY == "lifepo4":
        table = [(v * CELLS, s) for v, s in LIFEPO4_CELL_SOC]
    else:
        if FULL_V <= EMPTY_V:
            return None
        table = [(EMPTY_V, 0.0), (FULL_V, 100.0)]
    if voltage <= table[0][0]:
        return 0
    if voltage >= table[-1][0]:
        return 100
    for (v0, s0), (v1, s1) in zip(table, table[1:]):
        if v0 <= voltage <= v1:
            if v1 == v0:
                return int(round(s1))
            return int(round(s0 + (voltage - v0) / (v1 - v0) * (s1 - s0)))
    return None


def write_state(payload):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    payload = dict(payload)
    payload["updated_at"] = time.time()
    payload["chemistry"] = CHEMISTRY
    payload["cells"] = CELLS
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


def stream(ser, port, once=False):
    """持续读取已识别到的电池板；once=True 时读到一帧就返回。"""
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
            if once:
                return


def probe_ports():
    """列出并逐个扫描所有候选串口，打印收到的原始数据，用于现场找电池板。"""
    devices = []
    for pattern in ("/dev/ttyTHS*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS*"):
        for device in sorted(glob.glob(pattern)):
            if device not in devices:
                devices.append(device)
    for info in list_ports.comports():
        if info.device and info.device not in devices:
            devices.append(info.device)
    devices = [d for d in devices if d not in SKIP_PORTS]
    print(f"[探测] 候选串口: {devices}", flush=True)
    for device in devices:
        try:
            ser = serial.Serial(device, BAUD, timeout=0.3)
        except Exception as exc:
            print(f"[探测] {device} 打不开: {exc}", flush=True)
            continue
        print(f"[探测] {device} 打开成功，读取 {PROBE_LISTEN_S:.1f}s ...", flush=True)
        deadline = time.time() + PROBE_LISTEN_S
        buffer = b""
        try:
            while time.time() < deadline:
                data = ser.read(256)
                if data:
                    buffer += data
                else:
                    time.sleep(0.02)
        finally:
            try:
                ser.close()
            except Exception:
                pass
        if not buffer:
            print(f"[探测] {device} 无数据", flush=True)
            continue
        text = buffer.decode("utf-8", "replace")
        print(f"[探测] {device} 收到 {len(buffer)} 字节: {text[:200]!r}", flush=True)
        packet = parse_board_packet(text)
        if packet:
            print(f"[探测] {device} ← 识别为电池板: {packet}", flush=True)


def main():
    args = [a.strip() for a in sys.argv[1:]]
    if "--probe" in args:
        probe_ports()
        return
    once = "--once" in args
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
                stream(ser, port, once=once)
            except Exception as exc:
                print(f"[电池] {port} 读取中断: {exc}", flush=True)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            if once:
                return
            break
        if not identified:
            notice("[电池] 未识别到电池板", "未识别到电池板")
            time.sleep(RESCAN_DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[电池] 退出", flush=True)
