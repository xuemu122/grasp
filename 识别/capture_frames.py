#!/usr/bin/env python3
"""Capture RGB / depth / mask frames from camera for dataset collection."""

import argparse
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

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
        self._depth_scale = None
        self._align = None
        self._connect()

    def _connect(self):
        try:
            import pyrealsense2 as rs

            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self.pipeline.start(config)
            self._align = rs.align(rs.stream.color)
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
            if self._align is not None:
                frames = self._align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame:
                return None, None
            color = np.asanyarray(color_frame.get_data())
            depth = None
            if depth_frame:
                depth = np.asanyarray(depth_frame.get_data())
                if self._depth_scale is None:
                    self._depth_scale = depth_frame.get_units()
                    print(f"[相机] depth_scale={self._depth_scale:.6f} (raw→m)")
            return color, depth

        ok, frame = self.cap.read()
        return (frame, None) if ok else (None, None)

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


def depth_to_vis(depth_raw: np.ndarray) -> np.ndarray:
    valid = depth_raw[depth_raw > 0]
    if len(valid) == 0:
        return np.zeros((*depth_raw.shape, 3), dtype=np.uint8)
    depth_vis = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)


def depth_to_mask(depth_raw: np.ndarray, depth_scale: float, quantile: float, margin_mm: float) -> np.ndarray:
    depth_m = depth_raw.astype(np.float32) * max(depth_scale, 1e-6)
    valid = depth_m[depth_m > 0]
    if len(valid) == 0:
        return np.zeros_like(depth_raw, dtype=np.uint8)

    near = float(np.quantile(valid, quantile))
    cutoff = near + margin_mm / 1000.0
    mask = ((depth_m > 0) & (depth_m <= cutoff)).astype(np.uint8) * 255

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask

    best_idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    filtered = np.zeros_like(mask)
    filtered[labels == best_idx] = 255
    return filtered


def load_segmentation_model(model_path: str):
    if not model_path:
        return None
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("未安装 ultralytics，无法加载分割模型。请先安装: pip install ultralytics") from exc
    model = YOLO(model_path)
    print(f"[分割] 已加载模型: {model_path}")
    return model


def segment_to_mask(model, image: np.ndarray, conf: float) -> Optional[np.ndarray]:
    if model is None:
        return None

    results = model(image, conf=conf, verbose=False)
    if not results:
        return None

    result = results[0]
    if result.masks is None or result.masks.data is None:
        return None

    masks = result.masks.data
    if hasattr(masks, "cpu"):
        masks = masks.cpu().numpy()
    masks = np.asarray(masks)
    if masks.size == 0:
        return None

    combined = np.any(masks > 0.5, axis=0).astype(np.uint8) * 255
    h, w = image.shape[:2]
    if combined.shape[:2] != (h, w):
        combined = cv2.resize(combined, (w, h), interpolation=cv2.INTER_NEAREST)
    return combined


def main():
    parser = argparse.ArgumentParser(description="Capture camera frames for dataset collection.")
    parser.add_argument("-n", "--count", type=int, default=10, help="number of images to capture")
    parser.add_argument("-o", "--output", default="dataset_captures", help="output directory")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between captures")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--device", type=int, default=0, help="OpenCV camera device id")
    parser.add_argument("--mask-quantile", type=float, default=0.18, help="foreground mask uses the nearest depth quantile")
    parser.add_argument("--mask-margin-mm", type=float, default=120.0, help="extra depth margin for mask in mm")
    parser.add_argument("--seg-model", default="", help="optional YOLO segmentation model path, e.g. best-seg.pt")
    parser.add_argument("--seg-conf", type=float, default=0.25, help="segmentation confidence threshold")
    parser.add_argument(
        "--mask-source",
        choices=("depth", "seg", "seg-or-depth"),
        default="seg-or-depth",
        help="mask source: segmentation model, depth, or segmentation with depth fallback",
    )
    parser.set_defaults(save_depth=True, save_mask=True)
    parser.add_argument("--save-depth", dest="save_depth", action="store_true", help="save aligned raw depth and visualization")
    parser.add_argument("--no-save-depth", dest="save_depth", action="store_false", help="do not save depth")
    parser.add_argument("--save-mask", dest="save_mask", action="store_true", help="save depth-derived binary mask")
    parser.add_argument("--no-save-mask", dest="save_mask", action="store_false", help="do not save mask")
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    images_dir = out_dir / "images"
    depth_dir = out_dir / "depth_raw"
    depth_vis_dir = out_dir / "depth_vis"
    mask_dir = out_dir / "masks"
    images_dir.mkdir(parents=True, exist_ok=True)
    if args.save_depth:
        depth_dir.mkdir(parents=True, exist_ok=True)
        depth_vis_dir.mkdir(parents=True, exist_ok=True)
    if args.save_mask:
        mask_dir.mkdir(parents=True, exist_ok=True)

    seg_model = load_segmentation_model(args.seg_model)
    camera = Camera(args.width, args.height, args.fps, args.device)
    saved = 0
    try:
        print(f"[采集] 保存目录: {out_dir.resolve()}")
        while saved < args.count:
            color, depth = camera.read()
            if color is None:
                print("[采集] 读取失败, 重试...")
                time.sleep(0.1)
                continue

            stem = timestamp_ms()
            color_path = images_dir / f"{stem}.jpg"
            ok_color = cv2.imwrite(str(color_path), color)

            if ok_color:
                saved += 1
                print(f"[采集] {saved}/{args.count}: {color_path}")
            else:
                print(f"[采集] 保存失败: {color_path}")

            if args.save_depth and depth is not None and camera._depth_scale is not None:
                depth_path = depth_dir / f"{stem}.png"
                depth_vis_path = depth_vis_dir / f"{stem}.jpg"
                cv2.imwrite(str(depth_path), depth.astype(np.uint16))
                cv2.imwrite(str(depth_vis_path), depth_to_vis(depth))

            if args.save_mask:
                mask = None
                if args.mask_source in ("seg", "seg-or-depth"):
                    mask = segment_to_mask(seg_model, color, args.seg_conf)
                if mask is None and args.mask_source in ("depth", "seg-or-depth"):
                    if depth is not None and camera._depth_scale is not None:
                        mask = depth_to_mask(depth, camera._depth_scale, args.mask_quantile, args.mask_margin_mm)
                if mask is not None:
                    cv2.imwrite(str(mask_dir / f"{stem}.png"), mask)

            time.sleep(args.interval)
    finally:
        camera.release()

    print(f"[完成] 已保存 {saved} 张图片到 {out_dir.resolve()}")


if __name__ == "__main__":
    main()
