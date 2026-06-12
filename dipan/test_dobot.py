#!/usr/bin/env python3
"""DOBOT V4 MovL 诊断"""
import socket
import os

IP = os.getenv("DOBOT_IP", "192.168.5.1")
PORT = 29999

def cmd(sock, msg):
    if not msg.endswith("\n"):
        msg += "\n"
    sock.sendall(msg.encode())
    resp = b""
    sock.settimeout(3)
    while True:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            resp += chunk
            if b";" in chunk or b"}" in chunk:
                break
        except socket.timeout:
            break
    return resp.decode().strip()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(3)
sock.connect((IP, PORT))
print(f"连上 {IP}:{PORT}")

print("\n1. 清错")
print(cmd(sock, "ClearError()"))

print("\n2. RequestControl")
print(cmd(sock, "RequestControl()"))

print("\n3. PowerOn")
print(cmd(sock, "PowerOn()"))

import time
time.sleep(10)

print("\n4. 使能")
print(cmd(sock, "EnableRobot()"))

print("\n5. 状态")
print("RobotMode:", cmd(sock, "RobotMode()"))

# 读当前位姿
r = cmd(sock, "GetPose()")
print(f"\n6. 当前: {r}")

import re
nums = re.findall(r"[-+]?\d*\.?\d+", r)
if len(nums) >= 6:
    x, y, z, rx, ry, rz = map(float, nums[-6:])
    print(f"  解析: ({x:.1f},{y:.1f},{z:.1f}) rpy=({rx:.1f},{ry:.1f},{rz:.1f})")

    # 7. MovL V4 格式: pose = {x, y, z, rx, ry, rz}
    print(f"\n7. MovL V4 (pose=语法)")
    resp = cmd(sock, f"MovL(pose = {{{x:.1f}, {y:.1f}, {z:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}}})")
    print(f"  {resp}")

else:
    print("  解析失败")

sock.close()
print("\n完成")
