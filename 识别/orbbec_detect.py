"""
RealSense D405 深度相机 + YOLO 零售商品检测与货架定位系统 + EasyOCR 文字识别
"""

import cv2
import numpy as np
import yaml
import time
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple

from ultralytics import YOLO

# EasyOCR（ARM 平台 PaddlePaddle 会 segfault，EasyOCR 基于 PyTorch 更稳定）
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False
    easyocr = None
    print("[警告] EasyOCR 未安装，文字识别功能不可用")
    print("       安装命令: pip install easyocr")


# ============================================================
# RealSense D405 相机支持
# ============================================================

class RealSenseCamera:
    """Intel RealSense 深度相机封装（OpenCV 回退）"""

    # D405 默认内参（640x480, 约69° HFOV）
    DEFAULT_FX = 462.0
    DEFAULT_FY = 462.0
    DEFAULT_CX = 320.0
    DEFAULT_CY = 240.0

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 device_id: int = 0):
        self.width = width
        self.height = height
        self.fps = fps
        self.device_id = device_id
        self._pipeline = None
        self._config = None
        self._align = None
        self.cap_color = None
        self.use_sdk = False
        self._connected = False
        self._depth_scale = None  # RealSense depth_units (m/raw)
        # 相机内参（像素单位）
        self.fx = self.DEFAULT_FX
        self.fy = self.DEFAULT_FY
        self.cx = self.DEFAULT_CX
        self.cy = self.DEFAULT_CY
        self._connect()

    def _connect(self):
        """RealSense 优先 → OpenCV 回退"""
        try:
            import pyrealsense2 as rs

            self._pipeline = rs.pipeline()
            self._config = rs.config()
            self._config.enable_stream(rs.stream.color,
                                       self.width, self.height,
                                       rs.format.bgr8, self.fps)
            self._config.enable_stream(rs.stream.depth,
                                       self.width, self.height,
                                       rs.format.z16, self.fps)

            self._pipeline.start(self._config)
            self._align = rs.align(rs.stream.color)
            self.use_sdk = True
            self._connected = True

            # 启用自动曝光 + 暖机（让曝光稳定）
            try:
                sensor = self._pipeline.get_active_profile().get_device().query_sensors()[1]  # color sensor
                sensor.set_option(rs.option.enable_auto_exposure, True)
            except:
                pass
            for _ in range(30):
                frames = self._pipeline.wait_for_frames()
                self._align.process(frames)

            # 读取实际内参
            try:
                profile = self._pipeline.get_active_profile()
                color_stream = profile.get_stream(rs.stream.color)
                intr = color_stream.as_video_stream_profile().get_intrinsics()
                self.fx, self.fy = intr.fx, intr.fy
                self.cx, self.cy = intr.ppx, intr.ppy
                print(f"[相机] 内参: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")
            except Exception:
                pass

            print(f"[相机] RealSense 已连接 ({self.width}x{self.height}@{self.fps}fps), 深度已对齐到彩色图")
        except ImportError:
            print("[相机] pyrealsense2 未安装，使用 OpenCV 回退...")
            print("       安装: pip install pyrealsense2")
            self._connect_opencv()
        except Exception as e:
            print(f"[相机] RealSense 连接失败: {e}，使用 OpenCV 回退...")
            self._connect_opencv()

    def _connect_opencv(self):
        """OpenCV 回退（Linux 用 V4L2）"""
        import platform
        backend = cv2.CAP_V4L2 if platform.system() == 'Linux' else cv2.CAP_DSHOW
        for idx in [self.device_id, 0, 1, 2]:
            cap = cv2.VideoCapture(idx, backend)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.cap_color = cap
                    self.cap_color.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.cap_color.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    self._connected = True
                    print(f"[相机] OpenCV 模式 - 设备{idx}")
                    return
                cap.release()
        raise RuntimeError("无法打开任何摄像头")

    @property
    def has_depth(self) -> bool:
        return self.use_sdk

    def read(self):
        """读取帧，返回 (彩色图, 深度图)"""
        if not self._connected:
            return None, None

        if self.use_sdk:
            return self._read_realsense()
        else:
            if self.cap_color is None:
                return None, None
            ret, frame = self.cap_color.read()
            return (frame, None) if ret else (None, None)

    def _read_realsense(self):
        """RealSense 读取彩色+深度帧"""
        try:
            import pyrealsense2 as rs
            frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            if self._align is not None:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                return None, None

            # RealSense bgr8 格式直接得到 numpy (BGR)，无需转换
            color_img = np.asanyarray(color_frame.get_data())

            depth_img = None
            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())
                # 首次获取 depth_scale（单位: 米/raw_unit）
                if self._depth_scale is None:
                    self._depth_scale = depth_frame.get_units()
                    print(f"[相机] depth_scale={self._depth_scale:.6f} (raw→m)")

            return color_img, depth_img

        except Exception as e:
            print(f"[相机] 读取帧失败: {e}")
            return None, None

    def get_depth_at(self, depth_img, cx, cy, box=None, radius=15):
        """获取指定像素附近的中位数深度值（米），无效时返回 None。"""
        if depth_img is None:
            return None
        cx = int(round(cx))
        cy = int(round(cy))
        h, w = depth_img.shape[:2]
        y_start = max(cy - radius, 0)
        y_end = min(cy + radius, h)
        x_start = max(cx - radius, 0)
        x_end = min(cx + radius, w)
        roi = depth_img[y_start:y_end, x_start:x_end]
        valid = roi[roi > 0]
        if len(valid) == 0:
            return None
        # 用实际 depth_scale 转换（fallback: /1000 即 1mm/unit）
        scale = self._depth_scale or 0.001
        depth_m = float(np.median(valid)) * scale
        # D405 有效范围 ~0.05-3m，超范围视为无效
        if depth_m < 0.05 or depth_m > 3.0:
            return None
        return depth_m

    def pixel_to_camera_3d(self, u: float, v: float, depth_m: float) -> Tuple[float, float, float]:
        """像素坐标 + 深度 → 相机坐标系 3D (X右 Y下 Z前, 单位米)"""
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        z = depth_m
        return (x, y, z)

    def release(self):
        self._connected = False
        if self._pipeline:
            try:
                self._pipeline.stop()
            except:
                pass
            self._pipeline = None
            self._align = None
        if self.cap_color:
            self.cap_color.release()


# ============================================================
# 数据结构
# ============================================================

@dataclass
class ShelfPosition:
    shelf_id: str
    layer: int
    layer_height_m: float
    horizontal_offset: float
    description: str


@dataclass
class DetectionResult:
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center_pixel: Tuple[int, int]
    depth_m: Optional[float]
    camera_xyz: Optional[Tuple[float, float, float]] = None  # 相机坐标系3D (米)
    shelf_pos: Optional[ShelfPosition] = None
    product_id: Optional[str] = None       # 商品库ID / SKU ID
    product_name: Optional[str] = None     # OCR 匹配到的商品标准名
    category: Optional[str] = None         # 商品类别，如 drink / snack / medicine
    package_type: Optional[str] = None     # 包装形态，如 box / bottle / can / bag
    medicine_name: Optional[str] = None    # 兼容旧药品抓取链路，等同 product_name
    mapping_source: str = "none"           # ocr / yolo_fallback / none
    match_confidence: float = 0.0          # 预留给OCR/检索/条码融合
    match_sources: Tuple[str, ...] = ()    # 预留: ocr / barcode / visual_retrieval
    ocr_text: Optional[str] = None  # OCR 识别的文字


# ============================================================
# 映射表管理器
# ============================================================

class ProductMappingManager:
    def __init__(self, config_path: str = "shelf_mapping.yaml",
                 dataset_config_path: Optional[str] = None):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        # product_catalog: product_id -> aliases + shelf position + category/package metadata.
        # 兼容旧配置：medicine_catalog 的 key 是药品标准名，这里自动转为商品条目。
        self.product_catalog = self.config.get("product_catalog")
        if self.product_catalog is None:
            self.product_catalog = {
                key: {"name": key, **value}
                for key, value in self.config.get("medicine_catalog", {}).items()
            }
        self.medicine_catalog = self.product_catalog
        self.yolo_fallback_mapping = self.config.get(
            "yolo_fallback_mapping",
            self.config.get("medicine_mapping", {}),
        )
        self.category_fallback_mapping = self.config.get("category_fallback_mapping", {})
        matching_cfg = self.config.get("matching", {})
        self.min_ocr_alias_chars = int(matching_cfg.get("min_ocr_alias_chars", 3))
        self.prefer_ocr_over_yolo = bool(matching_cfg.get("prefer_ocr_over_yolo", False))
        self.medicine_mapping = self.yolo_fallback_mapping
        self.dataset_classes = self._load_dataset_classes(dataset_config_path)
        self.shelves = self.config["shelves"]
        print(
            f"[映射表] 已加载 {len(self.product_catalog)} 种OCR商品映射，"
            f"{len(self.yolo_fallback_mapping)} 个YOLO兜底映射，"
            f"{len(self.category_fallback_mapping)} 个品类兜底映射，"
            f"{len(self.dataset_classes)} 个数据集类别"
        )

    @staticmethod
    def _load_dataset_classes(dataset_config_path: Optional[str]) -> Dict[int, dict]:
        if dataset_config_path:
            path = Path(dataset_config_path)
        else:
            base_dir = Path(__file__).resolve().parent
            candidates = [
                base_dir.parent / "dataset" / "yolo_rpc_test" / "dataset.yaml",
                base_dir.parent / "yolo_rpc_test" / "dataset.yaml",
                base_dir / "madata.yaml",
            ]
            path = next((p for p in candidates if p.exists()), None)
            if path is None:
                return {}
        if not path.exists():
            return {}
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        names = cfg.get("names", {})
        if isinstance(names, list):
            items = enumerate(names)
        elif isinstance(names, dict):
            items = ((int(k), v) for k, v in names.items())
        else:
            return {}

        classes = {}
        for class_id, raw_name in items:
            category = str(raw_name).strip()
            classes[class_id] = {
                "product_id": f"rpc_{class_id:03d}",
                "product_name": f"{category} #{class_id + 1}",
                "category": category,
                "package_type": None,
            }
        return classes

    @staticmethod
    def _normalize_text(text: str) -> str:
        """统一 OCR 文本格式，便于药名/别名做宽松匹配。"""
        if not text:
            return ""
        drop_chars = " \t\r\n|,，.。:：;；()（）[]【】{}<>《》\"'`"
        normalized = text.lower()
        for ch in drop_chars:
            normalized = normalized.replace(ch, "")
        return normalized

    @staticmethod
    def _to_position(info: dict) -> ShelfPosition:
        if info is None:
            return None
        return ShelfPosition(
            shelf_id=info["shelf_id"],
            layer=info["layer"],
            layer_height_m=info["layer_height_m"],
            horizontal_offset=info["horizontal_offset"],
            description=info.get("description", ""),
        )

    def _get_catalog_position(self, product_id: str) -> Optional[ShelfPosition]:
        entry = self.product_catalog.get(product_id)
        if not entry:
            return None
        return self._to_position(entry)

    def match_product_from_ocr(self, ocr_text: str) -> Optional[Tuple[str, dict, float]]:
        """用 OCR 文本匹配标准商品名，支持别名和包含关系。"""
        norm_ocr = self._normalize_text(ocr_text)
        if not norm_ocr:
            return None

        best_product_id = None
        best_entry = None
        best_len = 0
        for product_id, entry in self.product_catalog.items():
            product_name = entry.get("name", product_id)
            aliases = [product_name, product_id] + list(entry.get("aliases", []))
            for alias in aliases:
                norm_alias = self._normalize_text(alias)
                if not norm_alias:
                    continue
                # 避免用 "500ml"、"vc" 这类短/弱特征误命中大量商品。
                if len(norm_alias) < self.min_ocr_alias_chars and norm_alias != self._normalize_text(product_name):
                    continue
                # OCR 往往会多字/漏字，先用包含关系做轻量匹配。
                if norm_alias in norm_ocr or norm_ocr in norm_alias:
                    if len(norm_alias) > best_len:
                        best_product_id = product_id
                        best_entry = entry
                        best_len = len(norm_alias)
        if best_product_id is None:
            return None
        confidence = min(1.0, best_len / max(len(norm_ocr), 1))
        return best_product_id, best_entry, confidence

    def match_medicine_from_ocr(self, ocr_text: str) -> Optional[str]:
        """兼容旧接口：返回 OCR 命中的标准商品名。"""
        match = self.match_product_from_ocr(ocr_text)
        if not match:
            return None
        _, entry, _ = match
        return entry.get("name")

    def resolve_position(self, ocr_text: Optional[str], class_name: str,
                         class_id: Optional[int] = None):
        """融合 OCR 与 YOLO。

        对 RPC 这类 200 类 SKU 数据集，YOLO class_id 是主要商品身份；
        OCR 默认只作为辅助，除非配置 prefer_ocr_over_yolo=true。
        """
        dataset_entry = self.dataset_classes.get(class_id) if class_id is not None else None
        match = self.match_product_from_ocr(ocr_text or "")
        if match and (self.prefer_ocr_over_yolo or dataset_entry is None):
            product_id, entry, confidence = match
            product_name = entry.get("name", product_id)
            return {
                "product_id": product_id,
                "product_name": product_name,
                "medicine_name": product_name,
                "category": entry.get("category"),
                "package_type": entry.get("package_type"),
                "shelf_pos": self._get_catalog_position(product_id),
                "mapping_source": "ocr",
                "match_confidence": confidence,
                "match_sources": ("ocr",),
            }

        fallback = self.get_position(class_name)
        if dataset_entry:
            catalog_entry = self.product_catalog.get(dataset_entry["product_id"], {})
            product_name = catalog_entry.get("name", dataset_entry["product_name"])
            category = catalog_entry.get("category", dataset_entry["category"])
            category_fallback = self.get_category_position(dataset_entry["category"])
            return {
                "product_id": dataset_entry["product_id"],
                "product_name": product_name,
                "medicine_name": product_name,
                "category": category,
                "package_type": catalog_entry.get("package_type", dataset_entry["package_type"]),
                "shelf_pos": self._get_catalog_position(dataset_entry["product_id"]) or fallback or category_fallback,
                "mapping_source": "yolo_class",
                "match_confidence": 1.0,
                "match_sources": ("yolo",),
            }

        if fallback:
            return {
                "product_id": None,
                "product_name": None,
                "medicine_name": None,
                "category": None,
                "package_type": class_name,
                "shelf_pos": fallback,
                "mapping_source": "yolo_fallback",
                "match_confidence": 0.0,
                "match_sources": (),
            }

        return {
            "product_id": None,
            "product_name": None,
            "medicine_name": None,
            "category": None,
            "package_type": class_name,
            "shelf_pos": None,
            "mapping_source": "none",
            "match_confidence": 0.0,
            "match_sources": (),
        }

    def get_position(self, class_name: str) -> Optional[ShelfPosition]:
        """按 YOLO 类别名取兜底货架位置，保留旧接口兼容。"""
        info = self.yolo_fallback_mapping.get(class_name)
        return self._to_position(info) if info else None

    def get_category_position(self, category: Optional[str]) -> Optional[ShelfPosition]:
        """按 RPC 数据集大类取货架兜底位置。"""
        if not category:
            return None
        info = self.category_fallback_mapping.get(category)
        return self._to_position(info) if info else None


# 兼容旧名称，避免其他脚本导入 ShelfMappingManager 时失效。
ShelfMappingManager = ProductMappingManager


# ============================================================
# 可视化
# ============================================================

def _assess_lighting(color_img):
    """评估图像光照质量，返回 (score浮点, grade字符串, color_bgr元组, 判定列表)"""
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2].astype(np.float32)
    h, w = v.shape
    mean_v = float(np.mean(v))
    # 中心区域分析
    v_center = v[h//4 : 3*h//4, w//4 : 3*w//4]
    std_v = float(np.std(v_center))

    # 反光检测：饱和像素占比（V>240）和低饱和度区域
    glare_ratio = float(np.sum(v_center > 240)) / v_center.size
    s_center = hsv[h//4 : 3*h//4, w//4 : 3*w//4, 1].astype(np.float32)
    low_sat_ratio = float(np.sum((v_center > 200) & (s_center < 30))) / v_center.size

    # 评分
    brightness_score = max(0, min(100, (mean_v - 40) * 100 / 140))
    contrast_bonus = min(30, std_v)
    score = min(100, brightness_score * 0.7 + contrast_bonus * 0.3 / 30 * 100)
    # 反光惩罚：每1%反光扣5分
    glare_penalty = min(40, (glare_ratio + low_sat_ratio) * 100 * 5)
    score = max(0, score - glare_penalty)

    # 分级
    grades = []
    color_bgr = (0, 255, 120)
    if mean_v < 50:
        grades.append("过暗"); color_bgr = (0, 0, 255)
    elif mean_v < 85:
        grades.append("偏暗"); color_bgr = (0, 165, 255)
    if mean_v > 230:
        grades.append("过曝"); color_bgr = (0, 0, 255)
    if std_v < 10:
        grades.append("低对比度"); color_bgr = (0, 200, 255)
    if glare_ratio > 0.03 or low_sat_ratio > 0.03:
        grades.append("反光"); color_bgr = (0, 165, 255)
    if not grades:
        grades.append("正常")
    grade = "+".join(grades) if len(grades) == 1 else "/".join(grades)
    extra = {"glare_ratio": glare_ratio, "low_sat_ratio": low_sat_ratio}
    return score, grade, color_bgr, mean_v, std_v, extra


def draw_detection(frame, result, is_selected=False):
    """绘制单个检测结果"""
    x1, y1, x2, y2 = result.bbox
    cx, cy = result.center_pixel

    if is_selected:
        color = (0, 0, 255)          # 选中：红色框
        thickness = 4
    elif result.confidence > 0.7:
        color = (0, 255, 0)
        thickness = 2
    else:
        color = (0, 165, 255)
        thickness = 2

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # 标签
    lines = [f"{result.class_name} ({result.confidence:.2f})"]
    if is_selected:
        lines.insert(0, ">>> 已选中 <<<")
    if result.shelf_pos:
        sp = result.shelf_pos
        if result.product_name:
            lines.append(f"商品:{result.product_name}")
        lines.extend([
            f"货架:{sp.shelf_id} 第{sp.layer}层",
            f"高度:{sp.layer_height_m}m 偏移:{sp.horizontal_offset}m",
        ])
    if result.camera_xyz:
        x3d, y3d, z3d = result.camera_xyz
        lines.append(f"3D:({x3d:.2f},{y3d:.2f},{z3d:.2f})m")
    elif result.depth_m:
        lines.append(f"深度:{result.depth_m:.2f}m")
    if result.ocr_text:
        lines.append(f"文字:{result.ocr_text[:20]}")

    # 绘制标签（选中时背景为红色）
    font = cv2.FONT_HERSHEY_SIMPLEX
    y0 = max(y1 - 10, 20)
    for i, line in enumerate(lines):
        y = y0 + i * 20
        bg_color = (0, 0, 180) if is_selected else (0, 0, 0)
        text_color = (255, 255, 255) if is_selected else color
        cv2.putText(frame, line, (x1, y), font, 0.5, text_color, 1)

    return frame


def draw_info_panel(frame, detections, fps, camera_mode, light_info=None):
    """绘制信息面板
    light_info: (score, grade, color_bgr, mean_v, std_v, extra_dict) or None
    """
    panel_w = 300
    h, w = frame.shape[:2]
    panel = np.full((h, panel_w, 3), 30, dtype=np.uint8)

    cv2.putText(panel, "零售商品定位系统 + OCR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
    cv2.putText(panel, f"模式: {camera_mode}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)
    cv2.putText(panel, f"FPS: {fps:.1f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)
    cv2.putText(panel, f"数量: {len(detections)}", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)

    # 光照评分
    y = 130
    if light_info:
        score, grade, lcolor, mean_v, std_v, extra = light_info
        cv2.putText(panel, f"光照: {grade} ({score:.0f}分)", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, lcolor, 1)
        y += 20
        cv2.putText(panel, f"  亮度均值:{mean_v:.0f} 对比度:{std_v:.0f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        y += 20
        if extra["glare_ratio"] > 0.01:
            cv2.putText(panel, f"  反光比例:{extra['glare_ratio']*100:.1f}%", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 165, 255), 1)
            y += 20
        y += 2
    else:
        y = 130

    for i, det in enumerate(detections[:5]):  # 最多显示5个
        sp = det.shelf_pos
        if sp:
            name = det.product_name or det.class_name
            text = f"[{i+1}] {name} -> {sp.shelf_id}-L{sp.layer}({sp.layer_height_m}m)"
            color = (0, 255, 120)
        else:
            text = f"[{i+1}] {det.class_name} 未映射"
            color = (0, 165, 255)
        cv2.putText(panel, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        y += 25
        # YOLO 框位置
        x1, y1, x2, y2 = det.bbox
        cx, cy = det.center_pixel
        box_text = f"    框:({x1},{y1})-({x2},{y2}) 中心:({cx},{cy})"
        cv2.putText(panel, box_text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        y += 22
        # 三维坐标
        if det.camera_xyz:
            x3d, y3d, z3d = det.camera_xyz
            xyz_text = f"    3D:({x3d:.3f},{y3d:.3f},{z3d:.3f})m"
            cv2.putText(panel, xyz_text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
            y += 22
        # OCR 文字
        if det.ocr_text:
            ocr_text_short = det.ocr_text[:30] + ("..." if len(det.ocr_text) > 30 else "")
            ocr_text = f"    文字:{ocr_text_short}"
            cv2.putText(panel, ocr_text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 255, 255), 1)
            y += 22

    return np.hstack([frame, panel])


# ============================================================
# 主系统
# ============================================================

class RetailProductLocatorSystem:
    def __init__(self,
                 model_path: str = "runs/detect/retail_product/weights/best.pt",
                 mapping_path: str = "shelf_mapping.yaml",
                 dataset_config_path: Optional[str] = None,
                 conf_threshold: float = 0.15,
                 raw_conf_threshold: float = 0.01,
                 class_conf_thresholds: Optional[Dict[int, float]] = None,
                 use_ocr: bool = True,
                 ocr_interval: int = 90):
        
        print(f"[模型] 加载 YOLO 模型: {model_path}")
        if not Path(model_path).exists():
            print(f"[模型][警告] 权重文件不存在: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.raw_conf_threshold = min(raw_conf_threshold, conf_threshold)
        self.class_conf_thresholds = class_conf_thresholds or {}
        self.use_ocr = use_ocr and EASYOCR_AVAILABLE
        names = getattr(self.model, "names", {}) or {}
        print(f"[模型] 显示阈值 conf={self.conf_threshold:.2f}, 原始候选阈值 raw_conf={self.raw_conf_threshold:.2f}")
        if self.class_conf_thresholds:
            preview = ", ".join(
                f"{cls}:{thr:.2f}"
                for cls, thr in sorted(self.class_conf_thresholds.items())[:20]
            )
            print(f"[模型] 类别专用阈值: {preview}")
        print(f"[模型] 类别数: {len(names)}")
        if names:
            preview = ", ".join(f"{k}:{v}" for k, v in list(names.items())[:10])
            print(f"[模型] 类别预览: {preview}")
        if dataset_config_path:
            print(f"[模型] dataset_config: {dataset_config_path} ({'存在' if Path(dataset_config_path).exists() else '不存在'})")

        # 目标选择
        self.selected_index = None     # 鼠标点击选中的框索引
        self.detections = []           # 当前帧检测结果（供鼠标回调使用）
        self.raw_box_count = 0
        self.filtered_box_count = 0
        self.low_conf_box_count = 0
        self.last_candidates = []

        self.mapping_mgr = ProductMappingManager(mapping_path, dataset_config_path=dataset_config_path)
        self.camera = None

        # OCR — 主进程序初始化 EasyOCR 实例（复用，避免重复加载模型）
        self.ocr_interval = ocr_interval
        self.frame_counter = 0
        self.ocr_working = True
        self.ocr_model = None             # EasyOCR Reader 实例
        self.ocr_cache = []               # 跨帧 OCR 缓存，减少非 OCR 帧结果闪烁
        self.ocr_cache_ttl_frames = max(ocr_interval * 4, 120)

        self.prev_time = time.time()
        self.fps = 0.0
        self.frame_count = 0

    def _threshold_for_class(self, class_id: int) -> float:
        return self.class_conf_thresholds.get(class_id, self.conf_threshold)

    def _init_ocr(self):
        """在主进程中初始化 EasyOCR Reader（只加载一次模型）"""
        if not self.use_ocr or self.ocr_model is not None:
            return
        try:
            print("[OCR] 正在初始化 EasyOCR（首次加载模型，约需 20-40 秒）...")
            self.ocr_model = easyocr.Reader(['ch_sim', 'en'], gpu=True)
            print("[OCR] EasyOCR 初始化完成")
        except Exception as e:
            print(f"[OCR] 初始化失败: {e}，文字识别功能已关闭")
            self.ocr_working = False

    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标点击选择目标商品"""
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        # 坐标从 2x 显示画面缩回 640x480
        x = x // 2
        y = y // 2
        # 检查点击是否落在某个检测框内
        for i, det in enumerate(self.detections):
            x1, y1, x2, y2 = det.bbox
            if x1 <= x <= x2 and y1 <= y <= y2:
                if self.selected_index == i:
                    self.selected_index = None  # 再次点击取消选中
                    print("[选择] 已取消选中")
                else:
                    self.selected_index = i
                    print(f"[选择] 已选中 [{i+1}] {det.class_name} "
                          f"中心:({det.center_pixel[0]},{det.center_pixel[1]}) "
                          f"置信度:{det.confidence:.2f}")
                return
        # 点到空白区域取消选中
        if self.selected_index is not None:
            self.selected_index = None
            print("[选择] 已取消选中")

    def init_camera(self, device_id: int = 0):
        self.camera = RealSenseCamera(width=640, height=480, fps=30, device_id=device_id)

    @staticmethod
    def _bbox_iou(a, b) -> float:
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = iw * ih
        area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
        area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    def _lookup_cached_ocr(self, class_id: int, bbox) -> Optional[str]:
        best = None
        best_iou = 0.0
        min_frame = self.frame_counter - self.ocr_cache_ttl_frames
        self.ocr_cache = [item for item in self.ocr_cache if item["frame"] >= min_frame]
        for item in self.ocr_cache:
            if item["class_id"] != class_id:
                continue
            iou = self._bbox_iou(item["bbox"], bbox)
            if iou > best_iou:
                best = item
                best_iou = iou
        return best["ocr_text"] if best and best_iou >= 0.5 else None

    def _remember_ocr(self, class_id: int, bbox, ocr_text: str):
        if not ocr_text:
            return
        self.ocr_cache.append({
            "class_id": class_id,
            "bbox": bbox,
            "ocr_text": ocr_text,
            "frame": self.frame_counter,
        })

    def save_candidate_debug_image(self, color_img, output_path: str, max_candidates: int = 20):
        """保存 YOLO 原始候选诊断图，包含低置信候选和光照评分。"""
        if color_img is None:
            return False
        viz = color_img.copy()
        candidates = sorted(
            self.last_candidates,
            key=lambda item: item["confidence"],
            reverse=True,
        )[:max_candidates]

        for item in candidates:
            x1, y1, x2, y2 = item["bbox"]
            conf = item["confidence"]
            ok = conf >= self._threshold_for_class(item["class_id"])
            color = (0, 220, 0) if ok else (0, 165, 255)
            cv2.rectangle(viz, (x1, y1), (x2, y2), color, 2)
            thr = self._threshold_for_class(item["class_id"])
            label = f"{item['class_name']} {conf:.2f}/{thr:.2f}"
            cv2.putText(
                viz,
                label,
                (x1, max(15, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        score, grade, _, mean_v, std_v, extra = _assess_lighting(color_img)
        lines = [
            f"raw={self.raw_box_count} shown={self.filtered_box_count} low={self.low_conf_box_count}",
            f"raw_conf={self.raw_conf_threshold:.2f} conf={self.conf_threshold:.2f}",
            f"light={grade} score={score:.0f} mean={mean_v:.0f} std={std_v:.0f} glare={extra['glare_ratio']*100:.1f}%",
        ]
        for i, line in enumerate(lines):
            y = 22 + i * 22
            cv2.putText(viz, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3)
            cv2.putText(viz, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)

        if not candidates:
            cv2.putText(
                viz,
                "NO YOLO RAW CANDIDATES",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
            )

        return cv2.imwrite(output_path, viz)

    def process_frame(self, color_img, depth_img=None):
        """处理单帧"""
        results_list = []

        # YOLO 预测
        results = self.model.predict(
            source=color_img,
            conf=self.raw_conf_threshold,
            verbose=False,
        )
        result = results[0]
        raw_boxes = list(result.boxes)
        self.raw_box_count = len(raw_boxes)
        self.filtered_box_count = 0
        self.low_conf_box_count = 0
        self.last_candidates = []

        self.frame_counter += 1
        do_ocr_this_frame = (self.frame_counter % self.ocr_interval == 0)

        for box in raw_boxes:
            cls_id = int(box.cls[0])
            conf_score = float(box.conf[0])
            class_name = result.names[cls_id]
            conf_threshold = self._threshold_for_class(cls_id)

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            self.last_candidates.append({
                "class_id": cls_id,
                "class_name": class_name,
                "confidence": conf_score,
                "threshold": conf_threshold,
                "bbox": (x1, y1, x2, y2),
            })
            if conf_score < conf_threshold:
                self.low_conf_box_count += 1
                continue

            self.filtered_box_count += 1
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # 深度
            depth_m = None
            if self.camera and depth_img is not None:
                depth_m = self.camera.get_depth_at(depth_img, cx, cy, (x1, y1, x2, y2))

            # 三维坐标（相机坐标系，米）
            camera_xyz = None
            if self.camera and depth_m is not None:
                camera_xyz = self.camera.pixel_to_camera_3d(cx, cy, depth_m)

            # OCR：逐框裁剪 + CLAHE + 2x 放大 + EasyOCR 识别
            ocr_text = None
            bbox = (x1, y1, x2, y2)
            if self.use_ocr and self.ocr_working and do_ocr_this_frame and self.ocr_model is not None:
                padding = 10
                x1c = max(x1 - padding, 0); y1c = max(y1 - padding, 0)
                x2c = min(x2 + padding, color_img.shape[1]); y2c = min(y2 + padding, color_img.shape[0])
                crop = color_img[y1c:y2c, x1c:x2c]
                if crop.size > 0 and crop.shape[0] >= 15 and crop.shape[1] >= 15:
                    try:
                        # 逐框 CLAHE：tile 大小按裁剪区自适应
                        lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
                        l, a, b = cv2.split(lab)
                        tile = max(2, min(l.shape) // 10)
                        clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(tile, tile))
                        l = clahe.apply(l)
                        crop_eq = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
                        # 始终 2x 放大，中文字形需要足够像素
                        h_c, w_c = crop_eq.shape[:2]
                        crop_up = cv2.resize(crop_eq, (w_c * 2, h_c * 2), interpolation=cv2.INTER_CUBIC)
                        results = self.ocr_model.readtext(
                            crop_up,
                            text_threshold=0.3,
                            low_text=0.2,
                            contrast_ths=0.05,
                            adjust_contrast=0.3,
                        )
                        texts = [r[1].strip() for r in results if r[1].strip()]
                        ocr_text = "|".join(texts) if texts else None
                        if ocr_text:
                            self._remember_ocr(cls_id, bbox, ocr_text)
                    except Exception:
                        pass
            if ocr_text is None:
                ocr_text = self._lookup_cached_ocr(cls_id, bbox)

            # 映射：优先 OCR 商品名匹配；OCR 无结果/未命中时用 YOLO 类别兜底
            product_match = self.mapping_mgr.resolve_position(
                ocr_text=ocr_text,
                class_name=class_name,
                class_id=cls_id,
            )

            results_list.append(DetectionResult(
                class_id=cls_id,
                class_name=class_name,
                confidence=conf_score,
                bbox=(x1, y1, x2, y2),
                center_pixel=(cx, cy),
                depth_m=depth_m,
                camera_xyz=camera_xyz,
                shelf_pos=product_match["shelf_pos"],
                product_id=product_match["product_id"],
                product_name=product_match["product_name"],
                category=product_match["category"],
                package_type=product_match["package_type"],
                medicine_name=product_match["medicine_name"],
                mapping_source=product_match["mapping_source"],
                match_confidence=product_match["match_confidence"],
                match_sources=product_match["match_sources"],
                ocr_text=ocr_text,
            ))

        return results_list

    def run(self):
        """主循环"""
        if self.camera is None:
            self.init_camera()

        # 初始化 OCR（主线程同步）
        self._init_ocr()

        # 创建结果目录
        results_dir = Path("results")
        results_dir.mkdir(exist_ok=True)

        print("\n" + "=" * 50)
        print("  零售商品识别与货架定位系统")
        print("  按 'q' 退出  |  按 's' 截图  |  点击画面选目标")
        print("=" * 50 + "\n")

        cv2.namedWindow("Retail Product Locator")
        cv2.setMouseCallback("Retail Product Locator", self._mouse_callback)

        while True:
            start_t = time.time()

            color_img, depth_img = self.camera.read()
            if color_img is None:
                time.sleep(0.1)
                continue

            # 检测
            self.detections = self.process_frame(color_img, depth_img)
            detections = self.detections

            # 清除野指针：如果选中索引超出当前检测数量
            if self.selected_index is not None and self.selected_index >= len(detections):
                self.selected_index = None

            # 终端打印（每 30 帧一次）
            if self.frame_counter % 30 == 0:
                # 光照评估
                lscore, lgrade, _, lv, lstd, lextra = _assess_lighting(color_img)
                light_tag = f"  [光照] {lgrade} ({lscore:.0f}分) 均值:{lv:.0f} std:{lstd:.0f}"
                if lextra["glare_ratio"] > 0.03:
                    light_tag += f" 反光:{lextra['glare_ratio']*100:.1f}%"
                bad = ("过暗", "偏暗", "过曝", "反光")
                if any(g in lgrade for g in bad):
                    light_tag += " ⚠ 可能影响识别精度！"
                print(f"\n--- 检测到 {len(detections)} 个商品 ---")
                print(f"  [YOLO] raw={self.raw_box_count} 显示={self.filtered_box_count} "
                      f"低于阈值={self.low_conf_box_count} "
                      f"raw_conf={self.raw_conf_threshold:.2f} conf={self.conf_threshold:.2f}")
                if self.last_candidates:
                    top_candidates = sorted(self.last_candidates, key=lambda item: item["confidence"], reverse=True)[:8]
                    cand_text = " | ".join(
                        f"{c['class_name']}:{c['confidence']:.2f}@{c['bbox']}"
                        for c in top_candidates
                    )
                    print(f"  [YOLO候选Top] {cand_text}")
                else:
                    print("  [YOLO候选Top] 无候选框；请检查模型/画面/类别训练是否匹配")
                print(light_tag)
                for i, det in enumerate(detections):
                    sel = ">>> 选中 <<< " if i == self.selected_index else ""
                    x1, y1, x2, y2 = det.bbox
                    cx, cy = det.center_pixel
                    pos_info = ""
                    if det.shelf_pos:
                        sp = det.shelf_pos
                        name_info = f"商品:{det.product_name} " if det.product_name else ""
                        pos_info = (
                            f"{name_info}货架:{sp.shelf_id} 第{sp.layer}层 "
                            f"高度:{sp.layer_height_m}m 偏移:{sp.horizontal_offset}m "
                            f"映射:{det.mapping_source} 匹配:{det.match_confidence:.2f}"
                        )
                    else:
                        pos_info = "未映射到货架"
                    # 三维坐标
                    if det.camera_xyz:
                        x3d, y3d, z3d = det.camera_xyz
                        xyz_info = f"3D:({x3d:.3f},{y3d:.3f},{z3d:.3f})m"
                    else:
                        xyz_info = "3D:无深度"
                    ocr_info = f"文字:{det.ocr_text}" if det.ocr_text else "文字:识别中..."
                    print(f"  {sel}[{i+1}] {det.class_name} | "
                          f"YOLO位置:({x1},{y1})-({x2},{y2}) 中心:({cx},{cy}) | "
                          f"置信度:{det.confidence:.2f} | {xyz_info} | {pos_info} | {ocr_info}")
                print("-" * 50)

            # 绘制
            display = color_img.copy()
            for i, det in enumerate(detections):
                display = draw_detection(display, det, is_selected=(i == self.selected_index))

            # 面板（含光照评分）
            cam_mode = "RealSense(深度)" if self.camera.has_depth else "OpenCV(无深度)"
            light_info = _assess_lighting(color_img)
            display = draw_info_panel(display, detections, self.fps, cam_mode, light_info)

            # 深度可视化
            if depth_img is not None:
                depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                depth_vis = cv2.resize(depth_vis, (160, 120))
                dh, dw = depth_vis.shape[:2]
                display[10:10+dh, 10:10+dw] = depth_vis

            # 2x 放大显示（640x480 在高分屏上太小）
            display_show = cv2.resize(display, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Retail Product Locator", display_show)

            # FPS
            self.frame_count += 1
            now = time.time()
            if now - self.prev_time >= 1.0:
                self.fps = self.frame_count / (now - self.prev_time)
                self.frame_count = 0
                self.prev_time = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # 截图 + 保存 JSON 结果
                timestamp = int(time.time())
                save_img = f"results/capture_{timestamp}.jpg"
                cv2.imwrite(save_img, display)

                save_json = f"results/result_{timestamp}.json"
                with open(save_json, "w", encoding="utf-8") as f:
                    json.dump([{
                        "class": d.class_name,
                        "class_id": d.class_id,
                        "confidence": round(d.confidence, 3),
                        "bbox": d.bbox,
                        "center_pixel": d.center_pixel,
                        "product_id": d.product_id,
                        "product_name": d.product_name,
                        "category": d.category,
                        "package_type": d.package_type,
                        "medicine_name": d.medicine_name,
                        "mapping_source": d.mapping_source,
                        "match_confidence": round(d.match_confidence, 3),
                        "match_sources": list(d.match_sources),
                        "shelf": {
                            "shelf_id": d.shelf_pos.shelf_id,
                            "layer": d.shelf_pos.layer,
                            "layer_height_m": d.shelf_pos.layer_height_m,
                            "horizontal_offset": d.shelf_pos.horizontal_offset,
                        } if d.shelf_pos else None,
                        "ocr_text": d.ocr_text,
                        "depth_m": round(d.depth_m, 4) if d.depth_m is not None else None,
                        "camera_xyz": [round(v, 4) for v in d.camera_xyz] if d.camera_xyz else None,
                    } for d in detections], f, ensure_ascii=False, indent=2)

                print(f"[保存] 截图: {save_img}")
                print(f"[保存] 结果: {save_json}")

        # 退出时清理
        self.camera.release()
        cv2.destroyAllWindows()
        print("\n[系统] 已退出")


# 兼容旧名称，避免 dipan 或旧脚本导入 MedicineLocatorSystem 时失效。
MedicineLocatorSystem = RetailProductLocatorSystem


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    import os
    parser = argparse.ArgumentParser()
    parser.add_argument("--conf", type=float, default=0.15, help="显示/输出的置信度阈值")
    parser.add_argument("--raw-conf", type=float, default=0.01, help="YOLO 原始候选阈值")
    parser.add_argument("--ocr-interval", type=int, default=30, help="OCR 扫描间隔帧数")
    parser.add_argument("--no-ocr", action="store_true", help="关闭 OCR")
    args = parser.parse_args()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    system = RetailProductLocatorSystem(
            model_path=os.path.join(base_dir, "best.pt"),
            mapping_path=os.path.join(base_dir, "shelf_mapping.yaml"),
            dataset_config_path=os.path.abspath(os.path.join(base_dir, "..", "dataset", "yolo_rpc_test", "dataset.yaml")),
            conf_threshold=args.conf,
            raw_conf_threshold=args.raw_conf,
            use_ocr=not args.no_ocr,
            ocr_interval=args.ocr_interval,
        )
    system.run()
