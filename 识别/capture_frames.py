#!/usr/bin/env python3
"""Capture RGB / depth / mask frames from camera for dataset collection."""

import argparse
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


CAMERA_AUTO_EXPOSURE = os.getenv("CAMERA_AUTO_EXPOSURE", "0").strip().lower() in ("1", "true", "yes", "on")
CAMERA_EXPOSURE_MODE = os.getenv(
    "CAMERA_EXPOSURE_MODE", "auto" if CAMERA_AUTO_EXPOSURE else "adaptive").strip().lower()
CAMERA_EXPOSURE_US = float(os.getenv("CAMERA_EXPOSURE_US", "4000"))
CAMERA_AUTO_WB = os.getenv("CAMERA_AUTO_WB", "1").strip().lower() in ("1", "true", "yes", "on")
CAMERA_GAIN = float(os.getenv("CAMERA_GAIN", "50"))
# 动态曝光参数
CAMERA_ADAPT_TARGET = float(os.getenv("CAMERA_ADAPT_TARGET", "110"))
CAMERA_ADAPT_DEADBAND = float(os.getenv("CAMERA_ADAPT_DEADBAND", "15"))
CAMERA_ADAPT_INTERVAL_S = float(os.getenv("CAMERA_ADAPT_INTERVAL_S", "0.5"))
CAMERA_EXPOSURE_MIN = float(os.getenv("CAMERA_EXPOSURE_MIN", "100"))
CAMERA_EXPOSURE_MAX = float(os.getenv("CAMERA_EXPOSURE_MAX", "15000"))
CAMERA_ADAPT_STEP = float(os.getenv("CAMERA_ADAPT_STEP", "1.5"))


def apply_color_sensor_options(profile):
    """设置彩色相机曝光/白平衡/增益，返回彩色 sensor（动态曝光用）。"""
    import pyrealsense2 as rs

    try:
        sensors = profile.get_device().query_sensors()
    except Exception as exc:
        print(f"[相机][警告] 读取传感器失败: {exc}")
        return None

    def _looks_color(sensor):
        try:
            name = str(sensor.get_info(rs.camera_info.name)).lower()
        except Exception:
            name = ""
        try:
            has_wb = (sensor.supports(rs.option.white_balance)
                      or sensor.supports(rs.option.enable_auto_white_balance))
        except Exception:
            has_wb = False
        return has_wb or "rgb" in name or "color" in name

    candidates = []
    for sensor in sensors:
        try:
            if sensor.supports(rs.option.exposure):
                candidates.append(sensor)
        except Exception:
            continue
    candidates.sort(key=lambda s: 0 if _looks_color(s) else 1)

    for sensor in candidates:
        try:
            sensor_name = sensor.get_info(rs.camera_info.name)
        except Exception:
            sensor_name = "?"
        if CAMERA_EXPOSURE_MODE == "auto":
            sensor.set_option(rs.option.enable_auto_exposure, 1)
            exposure_text = "自动"
        else:
            sensor.set_option(rs.option.enable_auto_exposure, 0)
            sensor.set_option(rs.option.exposure, CAMERA_EXPOSURE_US)
            exposure_text = "手动 %.0fus" % CAMERA_EXPOSURE_US
            if CAMERA_EXPOSURE_MODE == "adaptive":
                exposure_text += "(动态)"
        try:
            sensor.set_option(rs.option.enable_auto_white_balance, 1 if CAMERA_AUTO_WB else 0)
        except Exception:
            pass
        try:
            sensor.set_option(rs.option.gain, CAMERA_GAIN)
        except Exception:
            pass
        print(
            f"[相机] 传感器={sensor_name} 曝光: {exposure_text}, "
            f"白平衡: {'自动' if CAMERA_AUTO_WB else '手动'}, gain={CAMERA_GAIN:.0f}"
        )
        return sensor
    return None


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

            self._color_sensor = apply_color_sensor_options(self.pipeline.get_active_profile())
            if self._color_sensor is not None and CAMERA_EXPOSURE_MODE != "auto":
                try:
                    self._exposure_us = float(self._color_sensor.get_option(rs.option.exposure))
                except Exception:
                    pass

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

    def _brightness_stats(self, color):
        v = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)[:, :, 2]
        return float(v.mean()), float((v >= 245).mean())

    def _adapt_exposure(self, color):
        if CAMERA_EXPOSURE_MODE != "adaptive" or self._color_sensor is None:
            return
        now = time.monotonic()
        if now - self._last_adapt_at < CAMERA_ADAPT_INTERVAL_S:
            return
        self._last_adapt_at = now
        try:
            mean_v, sat = self._brightness_stats(color)
            if sat >= 0.10:
                factor = 0.45
            elif sat >= 0.03:
                factor = 0.70
            elif sat >= 0.01:
                factor = 0.85
            elif sat <= 0.005 and mean_v < 60:
                factor = 1.15
            elif abs(mean_v - CAMERA_ADAPT_TARGET) <= CAMERA_ADAPT_DEADBAND:
                return
            else:
                factor = (CAMERA_ADAPT_TARGET / max(mean_v, 1.0)) ** 0.7
                factor = max(0.85, min(1.25, factor))
            factor = max(1.0 / CAMERA_ADAPT_STEP, min(CAMERA_ADAPT_STEP, factor))
            old = self._exposure_us
            new = max(CAMERA_EXPOSURE_MIN, min(CAMERA_EXPOSURE_MAX, old * factor))
            if abs(new - old) < max(5.0, old * 0.02):
                return
            import pyrealsense2 as rs
            self._color_sensor.set_option(rs.option.exposure, float(new))
            self._exposure_us = float(new)
            print("[相机][自适应] mean_v=%.0f sat=%.1f%% 曝光 %.0f→%.0fus (x%.2f)" % (
                mean_v, sat * 100.0, old, new, new / max(old, 1e-6)), flush=True)
        except Exception as exc:
            print(f"[相机][自适应][警告] 调整曝光失败: {exc}", flush=True)

    def tune_exposure(self, iters=3):
        if CAMERA_EXPOSURE_MODE != "adaptive" or self._color_sensor is None:
            return
        for _ in range(max(1, iters)):
            self._last_adapt_at = 0.0
            color, _ = self.read()
            if color is None:
                break
            time.sleep(0.1)

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
