#!/usr/bin/env python3
"""Capture color frames from camera for dataset collection."""

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


class Camera:
    def __init__(self, width: int, height: int, fps: int, device_id: int):
        self.width = width
        self.height = height
        self.fps = fps
        self.device_id = device_id
        self.pipeline = None
        self.cap = None
        self.use_realsense = False
        self._connect()

    def _connect(self):
        try:
            import pyrealsense2 as rs

            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.pipeline.start(config)
            self.use_realsense = True
            print(f"[相机] RealSense 已连接 ({self.width}x{self.height}@{self.fps}fps)")

            for _ in range(30):
                self.pipeline.wait_for_frames()
            return
        except ImportError:
            print("[相机] pyrealsense2 未安装, 使用 OpenCV 回退")
        except Exception as e:
            print(f"[相机] RealSense 连接失败: {e}, 使用 OpenCV 回退")

        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 device_id={self.device_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        print(f"[相机] OpenCV 已连接 device_id={self.device_id}")

    def read(self):
        if self.use_realsense:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())

        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.cap is not None:
            self.cap.release()


def timestamp_ms() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


def main():
    parser = argparse.ArgumentParser(description="Capture camera frames for dataset collection.")
    parser.add_argument("-n", "--count", type=int, default=10, help="number of images to capture")
    parser.add_argument("-o", "--output", default="dataset_captures", help="output directory")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between captures")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--device", type=int, default=0, help="OpenCV camera device id")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    camera = Camera(args.width, args.height, args.fps, args.device)
    saved = 0
    try:
        print(f"[采集] 保存目录: {out_dir.resolve()}")
        while saved < args.count:
            frame = camera.read()
            if frame is None:
                print("[采集] 读取失败, 重试...")
                time.sleep(0.1)
                continue

            path = out_dir / f"{timestamp_ms()}.jpg"
            if cv2.imwrite(str(path), frame):
                saved += 1
                print(f"[采集] {saved}/{args.count}: {path}")
            else:
                print(f"[采集] 保存失败: {path}")
            time.sleep(args.interval)
    finally:
        camera.release()

    print(f"[完成] 已保存 {saved} 张图片到 {out_dir.resolve()}")


if __name__ == "__main__":
    main()
