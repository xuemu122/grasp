#!/usr/bin/env python3
"""Object-aware light/style augmentation for YOLO datasets.

This is a lightweight OA-DG inspired preprocessing tool for the current
Ultralytics YOLO pipeline. It keeps YOLO labels unchanged and generates images
where foreground objects and background receive different photometric changes.
"""

import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def _resolve_dataset_path(data_yaml: Path, item: str) -> Path:
    raw = Path(item)
    if raw.is_absolute():
        return raw
    with data_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    root = Path(data.get("path", "."))
    if not root.is_absolute():
        root = (data_yaml.parent / root).resolve()
    return (root / raw).resolve()


def _load_yaml(data_yaml: Path):
    with data_yaml.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _label_path_for(image_path: Path, images_dir: Path) -> Path:
    rel = image_path.relative_to(images_dir)
    parts = list(rel.parts)
    if "images" in parts:
        parts[parts.index("images")] = "labels"
        return images_dir.parents[len(parts) - parts.index("labels") - 1] / Path(*parts)
    return images_dir.parent / "labels" / rel.with_suffix(".txt")


def _parse_classes(spec: str):
    if not spec:
        return None
    result = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            result.update(range(int(a), int(b) + 1))
        else:
            result.add(int(part))
    return result


def _read_yolo_labels(label_path: Path):
    labels = []
    if not label_path.exists():
        return labels
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        cls = int(float(parts[0]))
        vals = list(map(float, parts[1:5]))
        labels.append((cls, *vals, line.strip()))
    return labels


def _bbox_to_pixels(label, width: int, height: int):
    _, cx, cy, bw, bh, _ = label
    x1 = int(round((cx - bw / 2) * width))
    y1 = int(round((cy - bh / 2) * height))
    x2 = int(round((cx + bw / 2) * width))
    y2 = int(round((cy + bh / 2) * height))
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height, y2))
    return x1, y1, x2, y2


def _make_object_mask(shape, labels, classes):
    h, w = shape[:2]
    mask = np.zeros((h, w), dtype=np.float32)
    selected = 0
    for label in labels:
        cls = label[0]
        if classes is not None and cls not in classes:
            continue
        x1, y1, x2, y2 = _bbox_to_pixels(label, w, h)
        if x2 <= x1 or y2 <= y1:
            continue
        pad_x = max(2, int((x2 - x1) * 0.04))
        pad_y = max(2, int((y2 - y1) * 0.04))
        x1 = max(0, x1 - pad_x)
        x2 = min(w, x2 + pad_x)
        y1 = max(0, y1 - pad_y)
        y2 = min(h, y2 + pad_y)
        mask[y1:y2, x1:x2] = 1.0
        selected += 1
    if selected == 0:
        return None
    mask = cv2.GaussianBlur(mask, (21, 21), 0)
    return mask[:, :, None]


def _gamma(img, gamma):
    inv = 1.0 / max(gamma, 1e-3)
    table = np.array([(i / 255.0) ** inv * 255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(img, table)


def _adjust_hsv(img, sat_scale, val_scale, hue_shift):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 0] = (hsv[:, :, 0] + hue_shift) % 180
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat_scale, 0, 255)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * val_scale, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def _add_shadow(img, rng):
    h, w = img.shape[:2]
    overlay = img.copy().astype(np.float32)
    x0 = rng.randint(-w // 2, w)
    x1 = x0 + rng.randint(w // 3, w)
    poly = np.array([[x0, 0], [x1, 0], [x1 - rng.randint(0, w // 3), h], [x0 - rng.randint(0, w // 3), h]])
    shadow = np.zeros((h, w), dtype=np.float32)
    cv2.fillPoly(shadow, [poly], rng.uniform(0.25, 0.55))
    shadow = cv2.GaussianBlur(shadow, (81, 81), 0)[:, :, None]
    overlay *= 1.0 - shadow
    return np.clip(overlay, 0, 255).astype(np.uint8)


def _add_plastic_glare(img, labels, classes, rng):
    out = img.copy().astype(np.float32)
    h, w = img.shape[:2]
    for label in labels:
        cls = label[0]
        if classes is not None and cls not in classes:
            continue
        x1, y1, x2, y2 = _bbox_to_pixels(label, w, h)
        if x2 <= x1 or y2 <= y1:
            continue
        glare = np.zeros((h, w), dtype=np.float32)
        for _ in range(rng.randint(1, 3)):
            gx = rng.randint(x1, max(x1 + 1, x2))
            gw = max(2, int((x2 - x1) * rng.uniform(0.04, 0.12)))
            cv2.ellipse(
                glare,
                (gx, (y1 + y2) // 2),
                (gw, max(4, (y2 - y1) // 2)),
                rng.uniform(-12, 12),
                0,
                360,
                rng.uniform(0.25, 0.65),
                -1,
            )
        glare = cv2.GaussianBlur(glare, (31, 31), 0)[:, :, None]
        out = out * (1.0 - glare) + 255.0 * glare
    return np.clip(out, 0, 255).astype(np.uint8)


def _augment_object_aware(img, labels, classes, rng):
    mask = _make_object_mask(img.shape, labels, classes)
    if mask is None:
        return None

    bg = _adjust_hsv(
        img,
        sat_scale=rng.uniform(0.65, 1.25),
        val_scale=rng.uniform(0.55, 1.35),
        hue_shift=rng.uniform(-8, 8),
    )
    if rng.random() < 0.65:
        bg = _add_shadow(bg, rng)
    bg = _gamma(bg, rng.uniform(0.65, 1.55))

    fg = _adjust_hsv(
        img,
        sat_scale=rng.uniform(0.55, 1.5),
        val_scale=rng.uniform(0.65, 1.45),
        hue_shift=rng.uniform(-10, 10),
    )
    fg = _gamma(fg, rng.uniform(0.7, 1.45))
    if rng.random() < 0.85:
        fg = _add_plastic_glare(fg, labels, classes, rng)

    mixed = fg.astype(np.float32) * mask + bg.astype(np.float32) * (1.0 - mask)
    noise_sigma = rng.uniform(0, 4.0)
    if noise_sigma > 0:
        mixed += rng.normal(0, noise_sigma, mixed.shape)
    return np.clip(mixed, 0, 255).astype(np.uint8)


def _iter_images(images_dir: Path):
    return sorted(p for p in images_dir.rglob("*") if p.suffix.lower() in IMG_EXTS)


def _write_yaml(src_yaml: Path, out_root: Path, names, nc):
    data = {
        "path": str(out_root.resolve()),
        "train": "images",
        "val": "images",
        "test": "images",
        "nc": nc,
        "names": names,
    }
    out_yaml = out_root / "dataset.yaml"
    with out_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return out_yaml


def main():
    parser = argparse.ArgumentParser(description="Generate OA-Mix style YOLO augmentations.")
    parser.add_argument("--data", default="识别/madata.yaml", help="YOLO dataset yaml.")
    parser.add_argument("--out", default="", help="Output dataset root. Default: <src>_oa_mix")
    parser.add_argument("--copies", type=int, default=3, help="Augmented copies per source image.")
    parser.add_argument("--classes", default="", help="Class ids to object-augment, e.g. 70-77,80-86. Empty means all.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--include-original", action="store_true", help="Copy original images/labels too.")
    args = parser.parse_args()

    data_yaml = Path(args.data).resolve()
    data = _load_yaml(data_yaml)
    images_dir = _resolve_dataset_path(data_yaml, data["train"])
    out_root = Path(args.out).resolve() if args.out else images_dir.parent.with_name(images_dir.parent.name + "_oa_mix")
    out_images = out_root / "images"
    out_labels = out_root / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    classes = _parse_classes(args.classes)
    rng = random.Random(args.seed)
    images = _iter_images(images_dir)
    made = 0
    skipped = 0

    for image_path in images:
        label_path = _label_path_for(image_path, images_dir)
        labels = _read_yolo_labels(label_path)
        img = cv2.imread(str(image_path))
        if img is None or not labels:
            skipped += 1
            continue

        if args.include_original:
            shutil.copy2(image_path, out_images / image_path.name)
            if label_path.exists():
                shutil.copy2(label_path, out_labels / label_path.name)

        for idx in range(args.copies):
            aug = _augment_object_aware(img, labels, classes, rng)
            if aug is None:
                skipped += 1
                continue
            stem = f"{image_path.stem}_oa{idx+1}"
            cv2.imwrite(str(out_images / f"{stem}.jpg"), aug)
            if label_path.exists():
                shutil.copy2(label_path, out_labels / f"{stem}.txt")
            made += 1

    out_yaml = _write_yaml(data_yaml, out_root, data.get("names", {}), data.get("nc", len(data.get("names", {}))))
    print(f"[OA-Mix] source images: {len(images)}")
    print(f"[OA-Mix] generated: {made}, skipped: {skipped}")
    print(f"[OA-Mix] dataset yaml: {out_yaml}")


if __name__ == "__main__":
    main()
