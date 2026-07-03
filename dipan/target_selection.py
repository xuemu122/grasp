#!/usr/bin/env python3
"""Detection fusion and target query helpers for shelf grasping."""

from __future__ import annotations

import re
from dataclasses import replace
from typing import Iterable, Sequence

import numpy as np


TARGET_ALIAS_GROUPS = (
    ("褪黑素", "melatonin", "ths"),
    ("雪碧", "sprite", "xuebi", "xue bi"),
)


def normalize_target_text(value) -> str:
    if value is None:
        return ""
    return re.sub(r"\s+", "", str(value).strip().lower())


def target_query_tokens(query: str):
    parts = re.split(r"[,，;；]+", query or "")
    tokens = set()
    for part in parts:
        token = normalize_target_text(part)
        if token:
            tokens.add(token)
    for group in TARGET_ALIAS_GROUPS:
        normalized_group = {normalize_target_text(item) for item in group}
        if tokens & normalized_group:
            tokens |= normalized_group
    return tokens


def unique_bboxes(detections: Iterable):
    seen = set()
    boxes = []
    for det in detections:
        bbox = getattr(det, "bbox", None)
        if bbox is None:
            continue
        key = tuple(int(v) for v in bbox)
        if key in seen:
            continue
        seen.add(key)
        boxes.append(bbox)
    return boxes


def refine_target_depth_on_frame(
    det,
    camera,
    depth_img,
    transformer,
    *,
    radius_px: int = 10,
    max_delta_m: float = 0.08,
):
    if det is None or depth_img is None:
        return det
    x1, y1, x2, y2 = det.bbox
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2
    sampled_depth = camera.get_depth_at(depth_img, u, v, radius=radius_px)
    if not sampled_depth or sampled_depth <= 0:
        return det
    old_depth = det.depth_m
    if old_depth and abs(sampled_depth - old_depth) > max_delta_m:
        print(
            f"  [深度校正][跳过] 参考帧中心深度 {sampled_depth:.3f}m 与融合深度 "
            f"{old_depth:.3f}m 差异过大"
        )
        return det
    if old_depth and abs(sampled_depth - old_depth) <= 0.003:
        return det
    p_cam = transformer.pixel_to_camera(u, v, sampled_depth)
    if old_depth:
        print(f"  [深度校正] 目标中心深度: 融合 {old_depth:.3f}m → 参考帧 {sampled_depth:.3f}m")
    else:
        print(f"  [深度校正] 目标中心深度: 参考帧 {sampled_depth:.3f}m")
    return replace(
        det,
        depth_m=sampled_depth,
        camera_xyz=tuple(float(x) for x in p_cam),
        center_pixel=(int(round(u)), int(round(v))),
    )


def detection_identity_values(det):
    values = [
        getattr(det, "product_name", None),
        getattr(det, "medicine_name", None),
        getattr(det, "class_name", None),
        getattr(det, "category", None),
        getattr(det, "package_type", None),
        getattr(det, "product_id", None),
        getattr(det, "ocr_text", None),
        getattr(det, "class_id", None),
    ]
    class_id = getattr(det, "class_id", None)
    if class_id is not None:
        values.extend((f"id:{class_id}", f"class:{class_id}", f"class_id:{class_id}"))
    return [v for v in values if v not in (None, "")]


def detection_label(det):
    name = (
        getattr(det, "product_name", None)
        or getattr(det, "medicine_name", None)
        or getattr(det, "class_name", None)
        or "?"
    )
    class_id = getattr(det, "class_id", "?")
    conf = getattr(det, "confidence", 0.0) or 0.0
    return f"{name}(id={class_id}, conf={conf:.2f})"


def filter_detections_by_target_query(detections: Sequence, query: str):
    tokens = target_query_tokens(query)
    if not tokens:
        return list(detections)

    matched = []
    for det in detections:
        haystack = [normalize_target_text(v) for v in detection_identity_values(det)]
        if any(token in value or value in token for token in tokens for value in haystack if value):
            matched.append(det)
    return matched


def print_target_filter_result(query: str, detections, matched):
    if not query:
        return
    if matched:
        details = " | ".join(detection_label(det) for det in matched[:8])
        print(f"  [目标过滤] {query!r}: 命中 {len(matched)}/{len(detections)} 个候选: {details}")
        return
    candidates = " | ".join(detection_label(det) for det in detections[:8])
    print(f"  [目标过滤][停止] {query!r}: 没有匹配候选")
    if candidates:
        print(f"  [目标过滤] 当前候选: {candidates}")


def is_clear_target_query(value: str) -> bool:
    return normalize_target_text(value) in ("", "*", "all", "any", "none", "clear", "全部", "所有", "无", "清空")


def print_model_classes(locator, filter_text: str = ""):
    names = getattr(getattr(locator, "model", None), "names", {}) or {}
    if not names:
        print("  [类别] 当前模型没有暴露类别列表")
        return
    tokens = target_query_tokens(filter_text)
    items = sorted(names.items(), key=lambda item: int(item[0]) if str(item[0]).isdigit() else str(item[0]))
    if tokens:
        items = [
            (class_id, name) for class_id, name in items
            if any(token in normalize_target_text(name) or token in normalize_target_text(class_id) for token in tokens)
        ]
    if not items:
        print(f"  [类别] 没有匹配 {filter_text!r} 的模型类别")
        return
    print(f"  [类别] 可选模型类别 {len(items)} 个" + (f" (过滤: {filter_text})" if filter_text else ""))
    line = []
    for class_id, name in items:
        line.append(f"{class_id}:{name}")
        if len(line) >= 4:
            print("    " + " | ".join(line))
            line = []
    if line:
        print("    " + " | ".join(line))


def bbox_iou(a, b) -> float:
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


def fuse_detection_tracks(frame_detections, iou_threshold=0.35):
    tracks = []
    for frame_idx, detections in enumerate(frame_detections):
        for det in detections:
            matched = None
            best_iou = 0.0
            for track in tracks:
                if track["class_id"] != det.class_id:
                    continue
                iou = bbox_iou(track["bbox"], det.bbox)
                if iou > best_iou:
                    best_iou = iou
                    matched = track
            if matched is None or best_iou < iou_threshold:
                tracks.append({
                    "class_id": det.class_id,
                    "items": [(frame_idx, det)],
                    "bbox": det.bbox,
                })
                continue
            matched["items"].append((frame_idx, det))
            boxes = np.array([item.bbox for _, item in matched["items"]], dtype=float)
            matched["bbox"] = tuple(np.median(boxes, axis=0).astype(int).tolist())

    fused_with_support = []
    for track in tracks:
        items = [item for _, item in track["items"]]
        boxes = np.array([d.bbox for d in items], dtype=float)
        bbox = tuple(np.median(boxes, axis=0).astype(int).tolist())
        conf = float(np.mean([d.confidence for d in items]))
        depths = [d.depth_m for d in items if d.depth_m is not None]
        depth_m = float(np.median(depths)) if depths else None
        best = max(items, key=lambda d: d.confidence)
        cx = int(round((bbox[0] + bbox[2]) / 2))
        cy = int(round((bbox[1] + bbox[3]) / 2))
        fused_det = replace(
            best,
            confidence=conf,
            bbox=bbox,
            center_pixel=(cx, cy),
            depth_m=depth_m,
        )
        fused_with_support.append((len(items), conf, fused_det))

    fused_with_support.sort(key=lambda item: (item[0], item[1]), reverse=True)
    return [item[2] for item in fused_with_support], tracks
