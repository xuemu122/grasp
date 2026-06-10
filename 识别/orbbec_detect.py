"""
RealSense D405 深度相机 + YOLO 药品识别与货架定位系统 + EasyOCR 文字识别
"""

import cv2
import numpy as np
import yaml
import time
import json
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
        self.cap_color = None
        self.use_sdk = False
        self._connected = False
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
            self.use_sdk = True
            self._connected = True

            # 启用自动曝光 + 暖机（让曝光稳定）
            try:
                sensor = self._pipeline.get_active_profile().get_device().query_sensors()[1]  # color sensor
                sensor.set_option(rs.option.enable_auto_exposure, True)
            except:
                pass
            for _ in range(30):
                self._pipeline.wait_for_frames()

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

            print(f"[相机] RealSense 已连接 ({self.width}x{self.height}@{self.fps}fps)")
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

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                return None, None

            # RealSense bgr8 格式直接得到 numpy (BGR)，无需转换
            color_img = np.asanyarray(color_frame.get_data())

            depth_img = None
            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())

            return color_img, depth_img

        except Exception as e:
            print(f"[相机] 读取帧失败: {e}")
            return None, None

    def get_depth_at(self, depth_img, cx, cy, box, radius=15):
        """获取目标区域的中位数深度值（米）"""
        if depth_img is None:
            return None
        x1, y1, x2, y2 = box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        h, w = depth_img.shape[:2]
        y_start = max(cy - radius, 0)
        y_end = min(cy + radius, h)
        x_start = max(cx - radius, 0)
        x_end = min(cx + radius, w)
        roi = depth_img[y_start:y_end, x_start:x_end]
        valid = roi[roi > 0]
        if len(valid) == 0:
            return None
        # RealSense 深度单位：毫米 → 米
        return float(np.median(valid)) / 1000.0

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
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center_pixel: Tuple[int, int]
    depth_m: Optional[float]
    camera_xyz: Optional[Tuple[float, float, float]] = None  # 相机坐标系3D (米)
    shelf_pos: Optional[ShelfPosition] = None
    ocr_text: Optional[str] = None  # OCR 识别的文字


# ============================================================
# 映射表管理器
# ============================================================

class ShelfMappingManager:
    def __init__(self, config_path: str = "shelf_mapping.yaml"):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        self.medicine_mapping = self.config["medicine_mapping"]
        self.shelves = self.config["shelves"]
        print(f"[映射表] 已加载 {len(self.medicine_mapping)} 种药品映射")

    def get_position(self, medicine_name: str) -> Optional[ShelfPosition]:
        info = self.medicine_mapping.get(medicine_name)
        if info is None:
            return None
        return ShelfPosition(
            shelf_id=info["shelf_id"],
            layer=info["layer"],
            layer_height_m=info["layer_height_m"],
            horizontal_offset=info["horizontal_offset"],
            description=info.get("description", ""),
        )


# ============================================================
# 可视化
# ============================================================

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


def draw_info_panel(frame, detections, fps, camera_mode):
    """绘制信息面板"""
    panel_w = 300
    h, w = frame.shape[:2]
    panel = np.full((h, panel_w, 3), 30, dtype=np.uint8)

    cv2.putText(panel, "药品定位系统 + OCR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
    cv2.putText(panel, f"模式: {camera_mode}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)
    cv2.putText(panel, f"FPS: {fps:.1f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)
    cv2.putText(panel, f"数量: {len(detections)}", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 200), 1)

    y = 140
    for i, det in enumerate(detections[:5]):  # 最多显示5个
        sp = det.shelf_pos
        if sp:
            text = f"[{i+1}] {det.class_name} -> {sp.shelf_id}-L{sp.layer}({sp.layer_height_m}m)"
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

class MedicineLocatorSystem:
    def __init__(self,
                 model_path: str = "runs/detect/medicine_final/weights/best.pt",
                 mapping_path: str = "shelf_mapping.yaml",
                 conf_threshold: float = 0.4,
                 use_ocr: bool = True,
                 ocr_interval: int = 90):
        
        print(f"[模型] 加载 YOLO 模型: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.use_ocr = use_ocr and EASYOCR_AVAILABLE

        # 目标选择
        self.selected_index = None     # 鼠标点击选中的框索引
        self.detections = []           # 当前帧检测结果（供鼠标回调使用）

        self.mapping_mgr = ShelfMappingManager(mapping_path)
        self.camera = None

        # OCR — 主进程序初始化 EasyOCR 实例（复用，避免重复加载模型）
        self.ocr_interval = ocr_interval
        self.frame_counter = 0
        self.ocr_cache: Dict[str, str] = {}
        self.ocr_working = True
        self.ocr_model = None             # EasyOCR Reader 实例

        self.prev_time = time.time()
        self.fps = 0.0
        self.frame_count = 0

    def _init_ocr(self):
        """在主进程中初始化 EasyOCR Reader（只加载一次模型）"""
        if not self.use_ocr or self.ocr_model is not None:
            return
        try:
            print("[OCR] 正在初始化 EasyOCR（首次加载模型，约需 20-40 秒）...")
            self.ocr_model = easyocr.Reader(['ch_sim', 'en'], gpu=False)
            print("[OCR] EasyOCR 初始化完成")
        except Exception as e:
            print(f"[OCR] 初始化失败: {e}，文字识别功能已关闭")
            self.ocr_working = False

    def _ocr_one_box(self, cache_key: str, crop_img: np.ndarray) -> Optional[str]:
        """主进程直接调用 EasyOCR — 复用已加载的模型实例"""
        if not self.use_ocr or not self.ocr_working or self.ocr_model is None:
            return None

        # 放大到至少 128px 高（EasyOCR 对分辨率要求低一些）
        h, w = crop_img.shape[:2]
        if h < 128:
            scale = 128.0 / h
            crop_img = cv2.resize(crop_img, (int(w * scale), 128), interpolation=cv2.INTER_CUBIC)

        try:
            results = self.ocr_model.readtext(crop_img)
            texts = [item[1].strip() for item in results if item[1].strip()]
            return "|".join(texts) if texts else "[无文字]"
        except Exception as e:
            print(f"[OCR] 识别异常: {e}")
            return None

    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标点击选择目标药盒"""
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

    def process_frame(self, color_img, depth_img=None):
        """处理单帧"""
        results_list = []

        # YOLO 预测
        results = self.model.predict(
            source=color_img,
            conf=self.conf_threshold,
            verbose=False
        )
        result = results[0]

        self.frame_counter += 1
        do_ocr_this_frame = (self.frame_counter % self.ocr_interval == 0)

        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf_score = float(box.conf[0])
            class_name = result.names[cls_id]

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # 深度
            depth_m = None
            if self.camera and depth_img is not None:
                depth_m = self.camera.get_depth_at(depth_img, cx, cy, (x1, y1, x2, y2))

            # 三维坐标（相机坐标系，米）
            camera_xyz = None
            if self.camera and depth_m is not None:
                camera_xyz = self.camera.pixel_to_camera_3d(cx, cy, depth_m)

            # 映射
            shelf_pos = self.mapping_mgr.get_position(class_name)

            # OCR 文字识别（2x 上采样裁剪区，提高识别率）
            ocr_text = None
            if self.use_ocr and self.ocr_working and do_ocr_this_frame:
                cache_key = f"{class_name}_{x1}_{y1}_{x2}_{y2}"
                if cache_key in self.ocr_cache:
                    ocr_text = self.ocr_cache[cache_key]
                else:
                    padding = 5
                    x1p = max(x1 - padding, 0)
                    y1p = max(y1 - padding, 0)
                    x2p = min(x2 + padding, color_img.shape[1])
                    y2p = min(y2 + padding, color_img.shape[0])
                    crop = color_img[y1p:y2p, x1p:x2p]
                    if crop.size > 0:
                        crop = cv2.resize(crop, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                        ocr_text = self._ocr_one_box(cache_key, crop)
                        if ocr_text is not None:
                            self.ocr_cache[cache_key] = ocr_text

            results_list.append(DetectionResult(
                class_name=class_name,
                confidence=conf_score,
                bbox=(x1, y1, x2, y2),
                center_pixel=(cx, cy),
                depth_m=depth_m,
                camera_xyz=camera_xyz,
                shelf_pos=shelf_pos,
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
        print("  药品识别与货架定位系统")
        print("  按 'q' 退出  |  按 's' 截图  |  点击画面选目标")
        print("=" * 50 + "\n")

        cv2.namedWindow("Medicine Locator")
        cv2.setMouseCallback("Medicine Locator", self._mouse_callback)

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
                print(f"\n--- 检测到 {len(detections)} 个药品 ---")
                for i, det in enumerate(detections):
                    sel = ">>> 选中 <<< " if i == self.selected_index else ""
                    x1, y1, x2, y2 = det.bbox
                    cx, cy = det.center_pixel
                    pos_info = ""
                    if det.shelf_pos:
                        sp = det.shelf_pos
                        pos_info = f"货架:{sp.shelf_id} 第{sp.layer}层 高度:{sp.layer_height_m}m 偏移:{sp.horizontal_offset}m"
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

            # 面板
            cam_mode = "RealSense(深度)" if self.camera.has_depth else "OpenCV(无深度)"
            display = draw_info_panel(display, detections, self.fps, cam_mode)

            # 深度可视化
            if depth_img is not None:
                depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                depth_vis = cv2.resize(depth_vis, (160, 120))
                dh, dw = depth_vis.shape[:2]
                display[10:10+dh, 10:10+dw] = depth_vis

            # 2x 放大显示（640x480 在高分屏上太小）
            display_show = cv2.resize(display, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Medicine Locator", display_show)

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
                        "confidence": round(d.confidence, 3),
                        "bbox": d.bbox,
                        "center_pixel": d.center_pixel,
                        "shelf": {
                            "shelf_id": d.shelf_pos.shelf_id,
                            "layer": d.shelf_pos.layer,
                            "layer_height_m": d.shelf_pos.layer_height_m,
                            "horizontal_offset": d.shelf_pos.horizontal_offset,
                        } if d.shelf_pos else None,
                        "ocr_text": d.ocr_text,
                    } for d in detections], f, ensure_ascii=False, indent=2)

                print(f"[保存] 截图: {save_img}")
                print(f"[保存] 结果: {save_json}")

        # 退出时清理（同步模式无需 stop worker）
        if self.ocr_cache:
            timestamp = int(time.time())
            save_json = f"results/ocr_cache_{timestamp}.json"
            with open(save_json, "w", encoding="utf-8") as f:
                json.dump(self.ocr_cache, f, ensure_ascii=False, indent=2)
            print(f"[保存] OCR缓存: {save_json}")

        self.camera.release()
        cv2.destroyAllWindows()
        print("\n[系统] 已退出")


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    import os
    base_dir = os.path.dirname(os.path.abspath(__file__))
    system = MedicineLocatorSystem(
        model_path=os.path.join(base_dir, "best.pt"),
        mapping_path=os.path.join(base_dir, "shelf_mapping.yaml"),
        conf_threshold=0.45,
        use_ocr=True,
    )
    system.run()