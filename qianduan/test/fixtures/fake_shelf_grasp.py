#!/usr/bin/env python3
import re
import os
import sys
import json


print("[fake-worker-started]", flush=True)
print(f"[fake-worker-idle-startup] {os.getenv('SHELF_GRASP_IDLE_STARTUP', '')}", flush=True)
print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
for raw in sys.stdin:
    command = raw.strip()
    if command in ("**stow**", "__stow__"):
        print("[前端常驻模式] 已按收起指令到位", flush=True)
        raise SystemExit(0)
    if not command:
        continue
    print(f"[fake-worker-command] {command}", flush=True)
    if command == "__poweroff__":
        print("[前端常驻模式] 正在停止保持、断开灵巧手并关闭 DOBOT 使能", flush=True)
        raise SystemExit(0)
    if command == "__stow_pose__":
        print("[前端常驻模式] 收到底盘移动前安全收起指令", flush=True)
        print("[前端常驻模式][完成] 已到达收起姿态", flush=True)
        print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
        continue
    if command == "__inventory_scan__":
        report_path = os.getenv("SHELF_INVENTORY_SCAN_REPORT", "/tmp/shelf_inventory_scan.json")
        with open(report_path, "w", encoding="utf-8") as handle:
            json.dump({
                "dominant_name": "测试商品",
                "count": 3,
                "total": 4,
                "candidates": [{"name": "测试商品", "count": 3}]
            }, handle, ensure_ascii=False)
        print(f"[巡架识别][完成] report={report_path} dominant=测试商品 count=3 total=4", flush=True)
        print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
        continue
    layer_match = re.fullmatch(r"__layer_(\d+)__", command)
    if layer_match:
        layer = layer_match.group(1)
        print(f"[前端常驻模式] 正在移动到第{layer}层...", flush=True)
        print(f"[前端常驻模式][完成] 已到达第{layer}层", flush=True)
        print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
        continue
    if command == "灵巧手失败商品":
        print("[前端常驻模式][失败] 灵巧手闭合/保持失败", flush=True)
        print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
        continue
    print("[深度护栏] 探入路径", flush=True)
    print(f"[前端常驻模式][完成] {command}", flush=True)
    print("[前端常驻模式] 等待商品名 (**stow**=收起并退出) ...", flush=True)
