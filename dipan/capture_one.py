#!/usr/bin/env python3
"""拍一帧: D405 彩色 + 深度, 存到 dipan/"""

import cv2
import numpy as np
import os

try:
    import pyrealsense2 as rs
except ImportError:
    print("请先安装: pip3 install pyrealsense2")
    exit(1)

OUT_DIR = os.path.expanduser("~/dipan")
os.makedirs(OUT_DIR, exist_ok=True)

pipe = rs.pipeline()
cfg = rs.config()

# D405 depth 640x480, color 1280x720/640x480
# 先试 640x480 depth + color 对齐
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("启动 D405 ...")
profile = pipe.start(cfg)

# 扔前几帧暖机
for i in range(30):
    pipe.wait_for_frames()

print("拍照 ...")
frames = pipe.wait_for_frames()

depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
depth = np.asanyarray(depth_frame.get_data())
color = np.asanyarray(color_frame.get_data())

pipe.stop()

color_path = os.path.join(OUT_DIR, "color.png")
depth_raw_path = os.path.join(OUT_DIR, "depth_raw.png")
depth_vis_path = os.path.join(OUT_DIR, "depth_colormap.png")

cv2.imwrite(color_path, color)
cv2.imwrite(depth_raw_path, depth.astype(np.uint16))

depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
cv2.imwrite(depth_vis_path, depth_vis)

valid = depth[depth > 0]
print(f"已保存:")
print(f"  彩色:     {color_path}   {color.shape[1]}x{color.shape[0]}")
print(f"  深度原始:  {depth_raw_path}")
print(f"  深度伪彩:  {depth_vis_path}")
print(f"  深度范围:  {valid.min() if len(valid) else 'N/A'}")
print(f"            ~{valid.max() if len(valid) else 'N/A'} mm")
