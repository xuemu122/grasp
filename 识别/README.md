# 零售商品识别检测算法说明

本模块负责零售货架场景下的商品检测、商品身份解析、OCR 辅助识别、RGB-D 三维定位和货架位置映射。它是后续机械臂抓取流程的视觉入口。

当前算法以 `dataset/yolo_rpc_test` 中的 RPC 200 类商品数据集为主要检测依据：YOLO 类别 ID 表示商品 SKU/商品类别身份，OCR 和商品词典作为辅助信息。

## 目录结构

```text
识别/
├── orbbec_detect.py                  # 识别检测主程序
├── capture_frames.py                 # 数据集抽帧脚本，按毫秒时间命名图片
├── best.pt                           # YOLO 权重文件
├── madata.yaml                       # 训练/数据集配置，指向 ../dataset/yolo_rpc_test
├── shelf_mapping.yaml                # 商品词典、类别兜底货架映射、OCR匹配策略
├── requirements.txt                  # 基础运行依赖
├── requirements_retail_optional.txt  # 可选增强依赖
├── run.sh                            # 启动脚本，支持透传 orbbec_detect.py 参数
├── dataset_captures/                 # 默认抽帧输出目录
└── results/                          # 截图和JSON结果输出目录

dataset/yolo_rpc_test/
├── dataset.yaml                      # 完整 RPC 200 类 YOLO 数据集配置
├── images/                           # 约 24000 张图片
└── labels/                           # 约 24000 个标签文件
```

## 算法目标

输入：

- RGB 图像
- 可选深度图
- YOLO 200 类商品检测权重
- 商品/货架映射配置

输出：

- 商品检测框 `bbox`
- YOLO 类别 ID 和类别名
- 商品 ID / 商品名 / 大类
- OCR 文本
- 相机坐标系三维位置 `camera_xyz`
- 货架层位兜底信息
- 光照质量评估

## 核心流程

```text
RealSense / OpenCV 取图
        ↓
YOLO 200 类商品检测
        ↓
检测框中心深度估计
        ↓
像素 + 深度反投影到相机3D坐标
        ↓
按间隔帧执行 OCR，并使用跨帧缓存
        ↓
商品身份融合：
  1. RPC YOLO class_id 作为主身份
  2. product_catalog 可做已知商品精确覆盖
  3. category_fallback_mapping 提供大类货架兜底
        ↓
可视化显示 / JSON 保存
```

## 主要代码组件

### `RealSenseCamera`

位置：[orbbec_detect.py](./orbbec_detect.py)

功能：

- 优先使用 `pyrealsense2` 读取 RealSense 彩色图和深度图。
- 如果 RealSense SDK 不可用，回退到 OpenCV 摄像头模式。
- 读取实际相机内参 `fx/fy/cx/cy`。
- 提供 `pixel_to_camera_3d()` 做针孔模型反投影。

当前深度估计方式：

```text
在 bbox 中心附近取 30x30 左右 ROI
过滤无效深度 0
使用中位数作为目标深度
```

### `ProductMappingManager`

负责商品身份和货架映射。

优先级：

```text
默认：YOLO RPC class_id -> product_id/product_name
可选：OCR product_catalog 匹配
兜底：category_fallback_mapping / yolo_fallback_mapping
```

当前默认策略为：

```yaml
matching:
  prefer_ocr_over_yolo: false
```

原因是当前数据集是 200 类 SKU 检测集，YOLO 类别本身已经是主要商品身份。如果将 OCR 放在最高优先级，容易因包装文字、规格数字等误覆盖 YOLO 结果。

### `RetailProductLocatorSystem`

主系统类，负责：

- 加载 YOLO 权重。
- 加载商品映射表和数据集类别表。
- 初始化 OCR。
- 处理每一帧图像。
- 绘制检测结果。
- 保存截图和 JSON。

保留兼容别名：

```python
MedicineLocatorSystem = RetailProductLocatorSystem
ShelfMappingManager = ProductMappingManager
```

这样旧的药店抓取链路仍可以逐步迁移。

## 数据结构

每个检测目标使用 `DetectionResult` 表示，关键字段如下：

| 字段 | 说明 |
|------|------|
| `class_id` | YOLO 类别 ID，RPC 数据集范围为 0-199 |
| `class_name` | YOLO 类别名，例如 `甜点`、`drink` |
| `confidence` | YOLO 置信度 |
| `bbox` | 检测框 `(x1, y1, x2, y2)` |
| `center_pixel` | 检测框中心像素 |
| `depth_m` | 中心区域深度，中位数，单位米 |
| `camera_xyz` | 相机坐标系三维坐标 |
| `product_id` | 商品 ID，例如 `rpc_063` |
| `product_name` | 商品名，例如 `甜点 #64` |
| `category` | 商品大类 |
| `package_type` | 包装形态，当前 RPC 类别中通常为空 |
| `ocr_text` | OCR 文本 |
| `mapping_source` | 映射来源，如 `yolo_class`、`ocr`、`yolo_fallback` |
| `match_confidence` | 商品身份匹配置信度 |
| `shelf_pos` | 货架映射信息 |

## 商品身份融合策略

### 1. YOLO 作为主身份

`dataset/yolo_rpc_test` 是 200 类商品数据集，类别 ID 代表 SKU/商品类别编号。

示例：

```text
class_id = 63
class_name = 甜点
product_id = rpc_063
product_name = 甜点 #64
mapping_source = yolo_class
```

### 2. OCR 作为辅助

OCR 每隔 `ocr_interval` 帧执行一次，默认入口配置为：

```python
ocr_interval=30
```

OCR 前处理：

- bbox 外扩 10 像素
- 裁剪商品区域
- LAB 亮度通道 CLAHE 增强
- 2 倍放大
- EasyOCR 识别中文简体和英文

OCR 结果会进入跨帧缓存：

```text
缓存键：class_id + bbox IoU
命中条件：IoU >= 0.5
TTL：max(ocr_interval * 4, 120) 帧
```

这样非 OCR 帧不会立刻丢失最近识别到的文字。

### 3. 商品词典精确覆盖

`shelf_mapping.yaml` 中的 `product_catalog` 可配置少量明确商品，例如：

```yaml
product_catalog:
  coca_cola_500ml:
    name: "可口可乐 500ml"
    aliases: ["可口可乐", "可乐", "coca cola", "coke"]
    category: "drink"
    package_type: "bottle"
    shelf_id: "S01"
    layer: 3
    layer_height_m: 1.40
    horizontal_offset: 0.3
```

默认不让 OCR 覆盖 YOLO 结果。如需 OCR 优先，可改：

```yaml
matching:
  prefer_ocr_over_yolo: true
```

不建议在 200 类模型稳定时开启该选项，除非 OCR 词典已经足够完整且经过验证。

### 4. 大类货架兜底

RPC 类别名通常为“大类 + 编号”，例如：

```text
膨化食品 #1
甜点 #64
drink #82
纸巾 #174
```

因此 `category_fallback_mapping` 用大类给默认货架层：

```yaml
category_fallback_mapping:
  drink:
    shelf_id: "S01"
    layer: 3
    layer_height_m: 1.40
    horizontal_offset: 0.4
```

这不是精确 SKU 位置，只是抓取或导航前的粗定位兜底。

## 数据集说明

当前完整数据集位于：

```text
dataset/yolo_rpc_test
```

已检查结果：

| 项目 | 数量 |
|------|------|
| 类别数 | 200 |
| 图片数 | 约 24000 |
| 标签文件数 | 约 24000 |
| 标注框数 | 约 294333 |
| 有标注类别 | 200 |

配置文件：

- [dataset/yolo_rpc_test/dataset.yaml](../dataset/yolo_rpc_test/dataset.yaml)
- [识别/madata.yaml](./madata.yaml)

注意：

- 当前 `train`、`val`、`test` 都指向同一 `images` 目录。
- 正式评估前建议拆分训练集、验证集和测试集。
- 如果直接使用同一目录评估，指标会偏乐观。

## 运行方式

### 安装基础依赖

```bash
cd /home/nvidia/grasp/识别
pip install -r requirements.txt
```

基础依赖中包含：

- `ultralytics`
- `easyocr`
- `opencv-python`
- `pyrealsense2`
- `PyYAML`
- `numpy`

### 启动识别

```bash
cd /home/nvidia/grasp/识别
./run.sh
```

程序默认加载：

```text
识别/best.pt
识别/shelf_mapping.yaml
dataset/yolo_rpc_test/dataset.yaml
```

当前默认显示阈值较低，便于排查新模型召回：

```text
conf=0.15
raw_conf=0.01
```

`raw_conf` 用来让 YOLO 先返回低置信度候选；`conf` 决定最终显示和输出。程序每 30 帧会打印：

```text
[YOLO] raw=<原始候选数> 显示=<通过阈值数量> 低于阈值=<过滤数量>
[YOLO候选Top] 类别:置信度@bbox | ...
```

判断方式：

- `raw=0`：模型没有输出候选框，检查权重、画面、训练类别或输入。
- `raw>0 显示=0`：候选置信度太低，可以继续降低 `--conf`。
- `显示>0` 但画面没框：优先查 OpenCV 窗口或显示环境。

常用调试启动：

```bash
./run.sh --conf 0.05 --no-ocr
```

可用参数：

```text
--conf <float>          显示/输出阈值，默认 0.15
--raw-conf <float>      原始候选阈值，默认 0.01
--ocr-interval <int>    OCR 间隔帧数，默认 30
--no-ocr                关闭 OCR，用于只调 YOLO
```

### 操作

| 操作 | 功能 |
|------|------|
| 鼠标点击检测框 | 选中/取消选中目标 |
| `s` | 保存截图和 JSON |
| `q` | 退出 |

## 数据集抽帧

`capture_frames.py` 用于从相机保存训练图片，不加载 YOLO/OCR。默认抽 10 张，文件名精确到毫秒：

```bash
cd /home/nvidia/grasp/识别
./capture_frames.py
```

输出示例：

```text
dataset_captures/20260618_064812_153.jpg
```

常用参数：

```bash
./capture_frames.py -n 10 --interval 0.5 -o dataset_captures
./capture_frames.py -n 50 -o /home/nvidia/grasp/识别/my_dataset_images
```

脚本优先使用 RealSense；如果 RealSense SDK 不可用，会回退到 OpenCV 摄像头。

## 输出结果

按 `s` 后会输出：

```text
results/capture_<timestamp>.jpg
results/result_<timestamp>.json
```

JSON 示例：

```json
[
  {
    "class": "甜点",
    "class_id": 63,
    "confidence": 0.97,
    "bbox": [438, 200, 478, 286],
    "center_pixel": [458, 243],
    "product_id": "rpc_063",
    "product_name": "甜点 #64",
    "category": "甜点",
    "package_type": null,
    "medicine_name": "甜点 #64",
    "mapping_source": "yolo_class",
    "match_confidence": 1.0,
    "match_sources": ["yolo"],
    "shelf": {
      "shelf_id": "S01",
      "layer": 4,
      "layer_height_m": 1.75,
      "horizontal_offset": 0.8
    },
    "ocr_text": null,
    "depth_m": 1.2345,
    "camera_xyz": [0.12, -0.03, 1.23]
  }
]
```

## 训练建议

使用当前数据集训练 YOLO：

```bash
yolo detect train data=识别/madata.yaml model=yolov8n.pt imgsz=640 epochs=100
```

建议：

- 先拆分 `train/val/test`，不要三个阶段共用同一目录。
- 200 类商品检测建议优先保证每类样本均衡。
- 如果部署在 Jetson Orin，可优先尝试 `yolov8n` 或 `yolov8s`。
- 商品密集货架建议关注小目标召回率。

## 当前局限

1. **RGB-D 未显式对齐**
   
   当前 RealSense 彩色流和深度流没有使用 `rs.align(rs.stream.color)`。密集货架中，深度可能取到相邻商品或背景。

2. **深度估计仍较粗**
   
   只取 bbox 中心附近 ROI 的中位数。薄包装、反光包装或倾斜商品可能导致深度不稳定。

3. **无目标跟踪 ID**
   
   当前只有 OCR 缓存，没有完整的目标跟踪器。检测顺序变化时，鼠标选中仍可能不稳定。

4. **OCR 不是主要身份来源**
   
   在 RPC 200 类检测模型下，OCR 默认不覆盖 YOLO 类别。若需要真实品牌/SKU 文本，需要扩展商品词典或引入视觉检索/条码识别。

5. **货架位置是配置兜底**
   
   `category_fallback_mapping` 只是大类默认层位，不是自动货架理解。若要精确货架位置，需要检测货架层板、价签，并进行商品-价签关联。

## 后续增强方向

优先级从高到低：

1. RealSense 深度对齐：`rs.align(rs.stream.color)`。
2. 更稳的深度估计：框内深度聚类、前景深度过滤、异常值剔除。
3. 目标跟踪：ByteTrack / SORT / IoU Tracker。
4. OCR 模糊匹配：使用 `rapidfuzz` 替代简单包含关系。
5. 条码识别：使用 `pyzbar` 或 OpenCV barcode detector。
6. 视觉检索：使用 CLIP/DINOv2 + FAISS 建商品图库。
7. 价签识别和关联：检测 `price_tag`，OCR 价签文本并与上方商品绑定。

可选依赖见：

```text
requirements_retail_optional.txt
```

## 与旧药店流程兼容

为避免影响已有抓取链路，代码保留以下兼容字段和别名：

```python
MedicineLocatorSystem = RetailProductLocatorSystem
ShelfMappingManager = ProductMappingManager
medicine_name = product_name
```

后续新代码建议统一使用：

```python
RetailProductLocatorSystem
ProductMappingManager
product_id
product_name
category
```
