#!/usr/bin/env python3
import re
import sys


print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
for raw in sys.stdin:
    command = raw.strip()
    if command in ("**stow**", "__stow__"):
        print("[前端常驻模式] 已按收起指令到位", flush=True)
        raise SystemExit(0)
    if not command:
        continue
    print(f"[fake-worker-command] {command}", flush=True)
    layer_match = re.fullmatch(r"__layer_(\d+)__", command)
    if layer_match:
        layer = layer_match.group(1)
        print(f"[前端常驻模式][完成] 正在移动到第{layer}层...", flush=True)
        print(f"[前端常驻模式][完成] 已到达第{layer}层", flush=True)
        print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
        continue
    print("[深度护栏] 探入路径", flush=True)
    print(f"[前端常驻模式][完成] {command}", flush=True)
    print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
