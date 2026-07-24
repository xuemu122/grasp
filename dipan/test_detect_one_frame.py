#!/usr/bin/env python3
"""拍一帧 -> YOLO 识别 -> 显示结果并保存图片（历史调试工具）。"""
import os
import sys

import cv2

VISION_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "识别"))
sys.path.insert(0, VISION_DIR)
from orbbec_detect import MedicineLocatorSystem, RealSenseCamera

MODEL   = os.path.join(VISION_DIR, "best.pt")
MAPPING = os.path.join(VISION_DIR, "shelf_mapping.yaml")
OUT     = "/tmp/test_detection.jpg"

print("[1] 初始化相机 ...")
cam = RealSenseCamera(width=640, height=480, fps=30)

print("[2] 加载模型 ...")
locator = MedicineLocatorSystem(model_path=MODEL, mapping_path=MAPPING,
                                conf_threshold=0.4, use_ocr=False)
locator.init_camera = lambda: None
locator.camera = cam

print("[3] 拍一帧 ...")
# warmup: 丢弃前 5 帧等自动曝光稳定
for _ in range(5):
    cam.read()
color, depth = cam.read()
if color is None:
    print("相机未就绪")
    sys.exit(1)

print("[4] 推理 ...")
dets = locator.process_frame(color, depth)

print(f"\n检测到 {len(dets)} 个目标:\n")
for i, d in enumerate(dets):
    x1, y1, x2, y2 = d.bbox
    print(f"  [{i}] {d.class_name:12s}  bbox=({x1:3d},{y1:3d})-({x2:3d},{y2:3d})  "
          f"conf={d.confidence:.2f}  depth={d.depth_m or 'N/A'}")

# 画框保存
for d in dets:
    x1, y1, x2, y2 = d.bbox
    cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
    label = f"{d.class_name} {d.depth_m:.2f}m" if d.depth_m else d.class_name
    cv2.putText(color, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

cv2.imwrite(OUT, color)
print(f"\n结果已保存: {OUT}")
cam.release()
