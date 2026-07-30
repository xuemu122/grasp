#!/usr/bin/env python3
"""Object-aware light/style augmentation for YOLO datasets.

This is a lightweight OA-DG inspired preprocessing tool for the current
Ultralytics YOLO pipeline. It keeps YOLO labels unchanged and generates images
where foreground objects and background receive different photometric changes.
"""

import argparse
import random
import shutil
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
SPLIT_NAMES = ("train", "val", "test")


def _resolve_dataset_root(data_yaml: Path, data) -> Path:
    root = Path(data.get("path", ".")).expanduser()
    if not root.is_absolute():
        root = (data_yaml.parent / root).resolve()
    return root.resolve()


def _load_yaml(data_yaml: Path):
    with data_yaml.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _configured_image_dirs(data_yaml: Path, data):
    dataset_root = _resolve_dataset_root(data_yaml, data)
    image_dirs = []
    for split in SPLIT_NAMES:
        value = data.get(split)
        if not value:
            continue
        values = value if isinstance(value, list) else [value]
        for item in values:
            path = Path(str(item)).expanduser()
            if not path.is_absolute():
                path = dataset_root / path
            path = path.resolve()
            if path not in image_dirs:
                image_dirs.append(path)
    if not image_dirs:
        raise ValueError(f"{data_yaml} 未配置 train/val/test 图片目录")
    return dataset_root, image_dirs


def _label_path_for(image_path: Path, images_dir: Path) -> Path:
    rel = image_path.relative_to(images_dir)
    parts = list(image_path.parts)
    image_indexes = [index for index, part in enumerate(parts) if part == "images"]
    if image_indexes:
        parts[image_indexes[-1]] = "labels"
        return Path(*parts).with_suffix(".txt")
    return (images_dir.parent / "labels" / rel).with_suffix(".txt")


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


def _parse_split_ratios(spec: str):
    try:
        values = tuple(float(item.strip()) for item in spec.split(","))
    except ValueError as exc:
        raise ValueError("--split 必须是三个逗号分隔的数字，如 0.8,0.1,0.1") from exc
    if len(values) != 3 or any(value < 0 for value in values) or sum(values) <= 0:
        raise ValueError("--split 必须提供三个非负比例，如 0.8,0.1,0.1")
    total = sum(values)
    return tuple(value / total for value in values)


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


def _add_vignette(img, strength, rng):
    h, w = img.shape[:2]
    y, x = np.ogrid[:h, :w]
    cy = h / 2.0 + rng.uniform(-0.05, 0.05) * h
    cx = w / 2.0 + rng.uniform(-0.05, 0.05) * w
    ry = max(h * rng.uniform(0.72, 0.92), 1.0)
    rx = max(w * rng.uniform(0.72, 0.92), 1.0)
    dist = ((x - cx) / rx) ** 2 + ((y - cy) / ry) ** 2
    vignette = np.clip(1.0 - strength * np.maximum(dist - 0.15, 0.0), 0.35, 1.0).astype(np.float32)
    return np.clip(img.astype(np.float32) * vignette[:, :, None], 0, 255).astype(np.uint8)


def _add_sensor_noise(img, np_rng, sigma_range):
    sigma = np_rng.uniform(*sigma_range)
    if sigma <= 0:
        return img
    noisy = img.astype(np.float32) + np_rng.normal(0, sigma, img.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def _make_daylight_variant(img, mask, labels, classes, rng, np_rng):
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
    return _add_sensor_noise(np.clip(mixed, 0, 255).astype(np.uint8), np_rng, (0.0, 4.0))


def _make_low_light_variant(img, mask, labels, classes, rng, np_rng):
    bg = _adjust_hsv(
        img,
        sat_scale=rng.uniform(0.25, 0.9),
        val_scale=rng.uniform(0.12, 0.55),
        hue_shift=rng.uniform(-6, 6),
    )
    bg = _gamma(bg, rng.uniform(0.35, 0.9))
    if rng.random() < 0.8:
        bg = _add_shadow(bg, rng)
    if rng.random() < 0.9:
        bg = _add_vignette(bg, rng.uniform(0.18, 0.6), rng)

    fg = _adjust_hsv(
        img,
        sat_scale=rng.uniform(0.35, 1.1),
        val_scale=rng.uniform(0.2, 0.85),
        hue_shift=rng.uniform(-8, 8),
    )
    fg = _gamma(fg, rng.uniform(0.4, 0.95))
    if rng.random() < 0.35:
        fg = _add_plastic_glare(fg, labels, classes, rng)
    if rng.random() < 0.6:
        fg = _add_vignette(fg, rng.uniform(0.08, 0.35), rng)

    mixed = fg.astype(np.float32) * mask + bg.astype(np.float32) * (1.0 - mask)
    return _add_sensor_noise(np.clip(mixed, 0, 255).astype(np.uint8), np_rng, (4.0, 12.0))


def _augment_object_aware(img, labels, classes, rng, np_rng):
    mask = _make_object_mask(img.shape, labels, classes)
    if mask is None:
        return None

    if rng.random() < 0.5:
        return _make_daylight_variant(img, mask, labels, classes, rng, np_rng)
    return _make_low_light_variant(img, mask, labels, classes, rng, np_rng)


def _iter_images(images_dir: Path):
    return sorted(p for p in images_dir.rglob("*") if p.suffix.lower() in IMG_EXTS)


def _source_output_relative(image_path: Path, images_dir: Path, dataset_root: Path):
    rel = image_path.relative_to(images_dir)
    try:
        prefix = images_dir.relative_to(dataset_root).parent
    except ValueError:
        prefix = Path()
    return rel if str(prefix) == "." else prefix / rel


def _collect_records(data_yaml: Path, data):
    dataset_root, image_dirs = _configured_image_dirs(data_yaml, data)
    records = []
    seen_images = set()
    missing_labels = 0
    empty_labels = 0
    for images_dir in image_dirs:
        if not images_dir.is_dir():
            raise FileNotFoundError(f"图片目录不存在: {images_dir}")
        for image_path in _iter_images(images_dir):
            resolved = image_path.resolve()
            if resolved in seen_images:
                continue
            seen_images.add(resolved)
            label_path = _label_path_for(image_path, images_dir)
            if not label_path.is_file():
                missing_labels += 1
                continue
            labels = _read_yolo_labels(label_path)
            if not labels:
                empty_labels += 1
                continue
            records.append({
                "image": image_path,
                "label": label_path,
                "labels": labels,
                "relative": _source_output_relative(image_path, images_dir, dataset_root),
            })
    return dataset_root, records, len(seen_images), missing_labels, empty_labels


def _split_counts(size, ratios):
    raw = [size * ratio for ratio in ratios]
    counts = [int(value) for value in raw]
    remainder = size - sum(counts)
    order = sorted(range(3), key=lambda index: (raw[index] - counts[index], -index), reverse=True)
    for index in order[:remainder]:
        counts[index] += 1

    active = [index for index, ratio in enumerate(ratios) if ratio > 0]
    if size >= len(active):
        for index in active:
            if counts[index] > 0:
                continue
            donor = max(active, key=lambda candidate: counts[candidate])
            if counts[donor] > 1:
                counts[donor] -= 1
                counts[index] += 1
    return counts


def _split_records(records, ratios, seed):
    class_frequency = Counter()
    for record in records:
        class_frequency.update({label[0] for label in record["labels"]})

    groups = defaultdict(list)
    for record in records:
        classes = {label[0] for label in record["labels"]}
        rarest_class = min(classes, key=lambda class_id: (class_frequency[class_id], class_id))
        groups[rarest_class].append(record)

    rng = random.Random(seed)
    result = {name: [] for name in SPLIT_NAMES}
    for class_id in sorted(groups, key=lambda item: (class_frequency[item], item)):
        group = groups[class_id]
        rng.shuffle(group)
        counts = _split_counts(len(group), ratios)
        offset = 0
        for split_name, count in zip(SPLIT_NAMES, counts):
            result[split_name].extend(group[offset:offset + count])
            offset += count
    for split_name in SPLIT_NAMES:
        rng.shuffle(result[split_name])
    return result


def _write_yaml(build_root: Path, output_root: Path, names, nc):
    data = {
        "path": str(output_root.resolve()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "nc": nc,
        "names": names,
    }
    out_yaml = build_root / "dataset.yaml"
    with out_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return out_yaml


def _write_record(record, split_name, build_root, classes, copies, include_original, rng, np_rng):
    image_path = record["image"]
    label_path = record["label"]
    labels = record["labels"]
    relative = record["relative"]
    image_output = build_root / split_name / "images" / relative
    label_output = (build_root / split_name / "labels" / relative).with_suffix(".txt")
    image_output.parent.mkdir(parents=True, exist_ok=True)
    label_output.parent.mkdir(parents=True, exist_ok=True)

    img = cv2.imread(str(image_path))
    if img is None:
        return 0, 0, 1

    copied = 0
    generated = 0
    skipped = 0
    if include_original:
        shutil.copy2(image_path, image_output)
        shutil.copy2(label_path, label_output)
        copied = 1

    for index in range(copies):
        aug = _augment_object_aware(img, labels, classes, rng, np_rng)
        if aug is None:
            skipped += 1
            continue
        augmented_relative = relative.with_name(f"{relative.stem}_oa{index + 1}.jpg")
        augmented_image = build_root / split_name / "images" / augmented_relative
        augmented_label = (build_root / split_name / "labels" / augmented_relative).with_suffix(".txt")
        augmented_image.parent.mkdir(parents=True, exist_ok=True)
        augmented_label.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(augmented_image), aug):
            skipped += 1
            continue
        shutil.copy2(label_path, augmented_label)
        generated += 1
    return copied, generated, skipped


def main():
    parser = argparse.ArgumentParser(description="Generate OA-Mix style YOLO augmentations.")
    parser.add_argument("--data", default="识别/madata.yaml", help="YOLO dataset yaml.")
    parser.add_argument("--out", default="", help="Output dataset root. Default: <src>_oa_mix")
    parser.add_argument("--copies", type=int, default=3, help="Augmented copies per source image.")
    parser.add_argument("--classes", default="", help="Class ids to object-augment, e.g. 0,1,7. Empty means all.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--include-original", action="store_true", help="Copy original images/labels too.")
    parser.add_argument("--split", default="0.8,0.1,0.1", help="Train,val,test ratios (default: 0.8,0.1,0.1).")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing output dataset.")
    args = parser.parse_args()

    data_yaml = Path(args.data).resolve()
    data = _load_yaml(data_yaml)
    dataset_root, records, source_count, missing_labels, empty_labels = _collect_records(data_yaml, data)
    if not records:
        raise ValueError(
            f"没有找到可处理的图片/YOLO 标签对；共发现 {source_count} 张图片，"
            f"缺失标签 {missing_labels}，空标签 {empty_labels}"
        )
    if args.copies < 1:
        raise ValueError("--copies 必须至少为 1")
    out_root = Path(args.out).resolve() if args.out else dataset_root.with_name(dataset_root.name + "_oa_mix")
    if out_root.exists() and any(out_root.iterdir()) and not args.overwrite:
        raise FileExistsError(f"输出目录非空: {out_root}；确认替换时添加 --overwrite")

    names = data.get("names", [])
    nc = int(data.get("nc", len(names)))
    if isinstance(names, dict):
        name_ids = {int(key) for key in names}
        if name_ids != set(range(nc)):
            raise ValueError(f"names 键必须完整覆盖 0..{nc - 1}")
    elif len(names) != nc:
        raise ValueError(f"nc={nc}，但 names 有 {len(names)} 项")
    observed_classes = {label[0] for record in records for label in record["labels"]}
    invalid_classes = sorted(class_id for class_id in observed_classes if class_id < 0 or class_id >= nc)
    if invalid_classes:
        raise ValueError(f"标签类别超出 YAML 的 0..{nc - 1}: {invalid_classes}")

    classes = _parse_classes(args.classes)
    if classes is not None and not (classes & observed_classes):
        raise ValueError(
            f"--classes={args.classes} 与数据集类别不相交；当前标签类别为 "
            f"{min(observed_classes)}-{max(observed_classes)}，删除 --classes 可增强全部类别"
        )
    ratios = _parse_split_ratios(args.split)
    split_records = _split_records(records, ratios, args.seed)
    rng = random.Random(args.seed)
    np_rng = np.random.default_rng(args.seed)
    out_root.parent.mkdir(parents=True, exist_ok=True)
    if args.overwrite:
        for stale_build in out_root.parent.glob(f".{out_root.name}-*"):
            if stale_build.is_dir():
                shutil.rmtree(stale_build)
    build_root = Path(tempfile.mkdtemp(prefix=f".{out_root.name}-", dir=out_root.parent))
    copied = generated = skipped = 0
    processed = 0
    try:
        for split_name in SPLIT_NAMES:
            for record in split_records[split_name]:
                result = _write_record(
                    record, split_name, build_root, classes, args.copies,
                    args.include_original, rng, np_rng,
                )
                copied += result[0]
                generated += result[1]
                skipped += result[2]
                processed += 1
                if processed % 50 == 0 or processed == len(records):
                    print(f"[OA-Mix] progress: {processed}/{len(records)} source images", flush=True)
        _write_yaml(build_root, out_root, names, nc)
        if out_root.exists():
            shutil.rmtree(out_root)
        build_root.rename(out_root)
    except Exception:
        shutil.rmtree(build_root, ignore_errors=True)
        raise

    print(f"[OA-Mix] discovered images: {source_count}")
    print(f"[OA-Mix] valid labeled images: {len(records)}")
    print(f"[OA-Mix] missing labels: {missing_labels}, empty labels: {empty_labels}")
    print(f"[OA-Mix] source split: " + ", ".join(
        f"{name}={len(split_records[name])}" for name in SPLIT_NAMES
    ))
    print(f"[OA-Mix] copied originals: {copied}")
    print(f"[OA-Mix] generated: {generated}, skipped: {skipped}")
    print(f"[OA-Mix] dataset yaml: {out_root / 'dataset.yaml'}")


if __name__ == "__main__":
    main()
