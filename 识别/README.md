# 零售商品识别检测算法说明

本模块负责零售货架场景下的商品检测、商品身份解析、OCR 辅助识别、RGB-D 三维定位和货架位置映射。它是后续机械臂抓取流程的视觉入口。

当前算法以 `dataset/yolo_rpc_test` 中的现场 8 类商品数据集为主要检测依据：YOLO 类别 ID 表示商品身份，OCR 和商品词典作为辅助信息。

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
├── images/                           # 580 张现场原图
└── labels/                           # 580 个 YOLO 标签文件
```

## 算法目标

输入：

- RGB 图像
- 可选深度图
- YOLO 8 类商品检测权重
- 商品/货架映射配置

输出：

- 商品检测框 `bbox`
- YOLO 类别 ID 和类别名
- 商品 ID / 商品名 / 大类
- OCR 文本
- 相机坐标系三维位置 `camera_xyz`
- 货架层位兜底信息
- 光照质量评估
- 弱光增强状态（每帧是否触发增强、累计次数）
- 增强前后对比图（默认 `/tmp/enhance_compare_latest.jpg`）

## 核心流程

```text
RealSense / OpenCV 取图
        ↓
光照评估 + 可选弱光增强（CLAHE + 白平衡 + Gamma）
        ↓
YOLO 8 类商品检测
        ↓
检测框中心深度估计
        ↓
像素 + 深度反投影到相机3D坐标
        ↓
按间隔帧执行 OCR，并使用跨帧缓存
        ↓
商品身份融合：
  1. YOLO class_id 作为主身份
  2. product_catalog 可做已知商品精确覆盖
  3. category_fallback_mapping 提供大类货架兜底
        ↓
可视化显示 / JSON 保存 / 增强前后对比图
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
默认：YOLO class_id -> product_id/product_name
可选：OCR product_catalog 匹配
兜底：category_fallback_mapping / yolo_fallback_mapping
```

当前默认策略为：

```yaml
matching:
  prefer_ocr_over_yolo: false
```

原因是当前数据集的 8 个 YOLO 类别已经对应具体商品。如果将 OCR 放在最高优先级，容易因包装文字、规格数字等误覆盖 YOLO 结果。

### `RetailProductLocatorSystem`

主系统类，负责：

- 加载 YOLO 权重。
- 加载商品映射表和数据集类别表。
- 初始化 OCR。
- 处理每一帧图像。
- 绘制检测结果。
- 保存截图和 JSON。

弱光增强相关接口：

| 成员 | 说明 |
|------|------|
| `_fast_light_stats(img)` | 快速计算 HSV-V 均值/中心区标准差 |
| `_enhance_low_light(...)` | CLAHE + 白平衡 + Gamma 增强（不改原图） |
| `save_enhance_compare(...)` | 生成并保存"增强前 vs 增强后"并排对比图 |
| `last_enhance_before/after` | 最近一次增强前后的图像副本 |
| `last_enhance_light` | 最近一次增强前后的光照统计 |
| `enhance_compare_saved` | 本会话已保存对比图数量 |

外部脚本（如抓取流程）可在失败时直接调用 `locator.save_enhance_compare()` 取图。

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
| `class_id` | YOLO 类别 ID，当前范围为 0-7 |
| `class_name` | YOLO 类别名，例如 `农夫山泉`、`雪碧` |
| `confidence` | YOLO 置信度 |
| `bbox` | 检测框 `(x1, y1, x2, y2)` |
| `center_pixel` | 检测框中心像素 |
| `depth_m` | 中心区域深度，中位数，单位米 |
| `camera_xyz` | 相机坐标系三维坐标 |
| `product_id` | 商品 ID，例如 `rpc_000` |
| `product_name` | 商品名，例如 `农夫山泉 #1` |
| `category` | 商品大类 |
| `package_type` | 包装形态，未配置时为空 |
| `ocr_text` | OCR 文本 |
| `mapping_source` | 映射来源，如 `yolo_class`、`ocr`、`yolo_fallback` |
| `match_confidence` | 商品身份匹配置信度 |
| `shelf_pos` | 货架映射信息 |

## 商品身份融合策略

### 1. YOLO 作为主身份

`dataset/yolo_rpc_test` 是现场采集的 8 类商品数据集，类别 ID 直接代表商品。

示例：

```text
class_id = 0
class_name = 农夫山泉
product_id = rpc_000
product_name = 农夫山泉 #1
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

不建议在当前 8 类模型稳定时开启该选项，除非 OCR 词典已经足够完整且经过验证。

### 4. 大类货架兜底

当前类别名是具体商品，例如：

```text
农夫山泉
冰红茶
可口可乐
雪碧
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
| 类别数 | 8 |
| 图片数 | 580 |
| 标签文件数 | 580 |
| 标注框数 | 2843 |
| 有标注类别 | 8 |

配置文件：

- [识别/madata.yaml](./madata.yaml)
- 增强后：[dataset/yolo_rpc_test_oa_mix/dataset.yaml](../dataset/yolo_rpc_test_oa_mix/dataset.yaml)

注意：

- 原始目录未拆分，仅作为增强脚本的输入池。
- `oa_mix_augment_yolo.py` 按原图族自动拆分，当前增强结果为 461/60/59 张源图。
- 训练、验证和测试分别使用增强结果中的 `train/images`、`val/images`、`test/images`。

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
识别/madata.yaml
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

### 弱光增强 (补光/光平衡)

夜间或货架背光场景下 YOLO 容易漏检。主流程在调用 YOLO 前会自动判断画面亮度，对送入模型的图像副本做增强（CLAHE + 自动白平衡 + Gamma），原始图仍用于显示与深度对齐，因此不会影响可视化或 `camera_xyz`。

增强策略：

- 默认 `auto`：当画面中心区 HSV-V 均值 `< light_threshold`（默认 90）时启用；或中心区标准差 `< 12` 且均值 `< 130` 时也启用。
- `force`：每帧都启用（适合全黑环境先验证效果）。
- `off`：关闭（命令行加 `--no-enhance` 等价）。

启用后控制台会按 30 帧间隔打印：

```text
  [增强] 已应用(弱光) 累计:24帧
```

面板右侧也会显示 `补光增强: ON(弱光)`。

弱光场景推荐启动：

```bash
# 默认 auto：弱光自动触发
./run.sh --enhance-mode auto --gamma 0.8 --clahe-clip 3.0

# 强制每帧增强（适合纯夜灯场景）
./run.sh --enhance-mode force --gamma 0.7 --clahe-clip 3.0

# 关闭增强（恢复原图直送 YOLO）
./run.sh --no-enhance
```

参数：

```text
--enhance-mode {auto,force,off}    增强模式，默认 auto
--no-enhance                       关闭增强（= --enhance-mode off）
--light-threshold <float>          auto 触发阈值（HSV-V 0-255），默认 90
--clahe-clip <float>               CLAHE clipLimit，默认 2.5；设 0 关闭 CLAHE
--clahe-tile <int>                 CLAHE 网格大小，默认 8
--gamma <float>                    Gamma 矫正，<1 提亮 >1 压暗，1.0 不启用，默认 1.0
--no-awb                           关闭简易灰度世界白平衡
--no-enhance-compare               关闭增强前后对比图自动保存
--enhance-compare-path <path>      对比图输出路径，默认 /tmp/enhance_compare_latest.jpg
--enhance-compare-dir <dir>        对比图历史存档目录（可选，按时间戳命名）
--enhance-compare-interval <int>   对比图最小保存间隔帧数，默认 30
```

调参建议：

- 画面整体偏色（黄/蓝）→ 保留默认 `--awb`（开），三通道会被拉回中灰。
- 暗部细节仍糊 → 提高 `--clahe-clip` 到 3.0-4.0，或设 `--gamma 0.7`。
- 反光物体仍被压得过亮 → 关闭白平衡 `--no-awb`，只保留 CLAHE。
- 增强后画面噪声也变大 → 降低 `--clahe-tile` 让局部更平滑，或下调 `--gamma`。

### 增强前后对比图

为了直观看补光到底有没有效果，程序会在触发增强时自动保存一张**左右并排对比图**：

```text
+---------------- 增强前 BEFORE (原始) ----------------+---------------- 增强后 AFTER ----------------+
| mean_v=40  std_v=15                                  | mean_v=75  std_v=25                             |
| 触发=弱光  CLAHE(clip=2.5,tile=8)  gamma=0.8  ...    |                                                 |
+------------------------------------------------------+-------------------------------------------------+
|              [原始画面 + YOLO 框]                    |              [增强画面 + YOLO 框]               |
+------------------------------------------------------+-------------------------------------------------+
```

要点：

- 两侧使用**同一套检测框坐标**，可直接对比"增强前后召回是否变多"。
- 标题条显示触发原因、`mean_v / std_v` 变化、CLAHE/Gamma/AWB 参数、检测数。
- 中文用 PIL + 系统 Noto CJK 字体渲染；无中文字体时自动回退英文（`cv2.putText`）。
- 未触发增强（光照正常）时不会重复写盘；`auto` 模式下只有在弱光帧才产出。

输出位置：

```text
/tmp/enhance_compare_latest.jpg         # 默认，始终覆盖最新一张
<--enhance-compare-dir>/enhance_compare_<时间戳>.jpg   # 可选历史存档
```

调试启动示例：

```bash
# 弱光自动触发 + 保留最近一张对比图（默认行为）
./run.sh --enhance-mode auto --gamma 0.8

# 强制每帧增强，并存档到指定目录，方便回看
./run.sh --enhance-mode force --gamma 0.7 \
         --enhance-compare-dir /home/nvidia/grasp/识别/results/enhance_compare

# 关闭对比图（只在性能紧张时使用）
./run.sh --no-enhance-compare
```

也可以在识别窗口里按 `c` 手动保存当前对比图。

相关环境变量：

```text
ENHANCE_COMPARE_ENABLE=1            对比图总开关（默认 1）
ENHANCE_COMPARE_PATH=/tmp/enhance_compare_latest.jpg
ENHANCE_COMPARE_DIR=<目录>          历史存档目录，留空则只保留最新一张
ENHANCE_COMPARE_INTERVAL=30         最小保存间隔帧数
```

### 抓取流程中的对比图

`dipan/shelf_grasp.py` 在抓取失败时除了原有的 `shelf_failure_latest.jpg`，还会一并输出增强对比图：

```text
[失败诊断图] /tmp/shelf_failure_latest.jpg
[增强对比图] /tmp/shelf_enhance_compare_latest.jpg (左=增强前, 右=增强后)
```

- 路径可用环境变量 `SHELF_ENHANCE_COMPARE_PATH` 覆盖，默认与失败诊断图同目录。
- 仅当识别模块确实记录过增强帧时才产出；光照正常时不会额外生成。
- 每轮抓取开始时会与失败诊断图一起清理旧文件，避免查看过期结果。
- 对比图内容受识别模块的 `ENHANCE_COMPARE_ENABLE` / `--no-enhance` 等配置控制。

### 操作

| 操作 | 功能 |
|------|------|
| 鼠标点击检测框 | 选中/取消选中目标 |
| `s` | 保存截图和 JSON |
| `c` | 手动保存增强前后对比图 |
| `q` | 退出 |

## 数据集抽帧

`capture_frames.py` 用于从相机保存训练数据，不加载 YOLO/OCR。默认会同时保存 RGB、深度和 mask，文件名精确到毫秒：

```bash
cd /home/nvidia/grasp/识别
./capture_frames.py
```

输出示例：

```text
dataset_captures/images/20260618_064812_153.jpg
dataset_captures/depth_raw/20260618_064812_153.png
dataset_captures/depth_vis/20260618_064812_153.jpg
dataset_captures/masks/20260618_064812_153.png
```

常用参数：

```bash
./capture_frames.py -n 10 --interval 0.5 -o dataset_captures
./capture_frames.py -n 50 -o /home/nvidia/grasp/识别/my_dataset_images
./capture_frames.py -n 20 --seg-model /home/nvidia/grasp/识别/best-seg.pt --mask-source seg-or-depth
```

脚本优先使用 RealSense；如果 RealSense SDK 不可用，会回退到 OpenCV 摄像头。
如果提供 `--seg-model`，会优先使用分割模型输出实例 mask；否则默认用深度阈值生成前景 mask。

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

复杂光照/塑料瓶反光场景可以先生成一版 OA-DG 思路的对象感知增强数据集。脚本会按原图分组自动划分 `train/val/test`，同一原图的增强副本不会跨集合：

```bash
python3 识别/oa_mix_augment_yolo.py --data 识别/madata.yaml --copies 3 --include-original --overwrite
yolo detect train data=/home/nvidia/grasp/dataset/yolo_rpc_test_oa_mix/dataset.yaml model=yolov8n.pt imgsz=640 epochs=100
```

`--classes` 可限制需要增强的类别 ID，例如 `--classes 0,1,7`。不传时增强全部类别；若指定类别与标签不相交，脚本会直接报错而不是生成空数据集。默认划分比例为 `0.8,0.1,0.1`，可通过 `--split` 修改。

建议：

- 使用生成的 `dataset.yaml`，不要直接用未拆分的原始图片目录训练和评估。
- 类别 2 和类别 5 样本很少，后续采集应优先补充这两类。
- 如果部署在 Jetson Orin，可优先尝试 `yolov8n` 或 `yolov8s`。
- 商品密集货架建议关注小目标召回率。

## 当前局限

1. **RGB-D 对齐后仍可能有遮挡误差**
   
   当前采集脚本和识别主流程都已做彩色/深度对齐，但密集货架中，深度仍可能取到相邻商品或背景。

2. **深度估计仍较粗**
   
   只取 bbox 中心附近 ROI 的中位数。薄包装、反光包装或倾斜商品可能导致深度不稳定。

3. **无目标跟踪 ID**
   
   当前只有 OCR 缓存，没有完整的目标跟踪器。检测顺序变化时，鼠标选中仍可能不稳定。

4. **OCR 不是主要身份来源**
   
   在当前 8 类检测模型下，OCR 默认不覆盖 YOLO 类别。若需要区分更多规格，需要扩展类别、商品词典或引入条码识别。

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

## 透视补偿

`hand_eye.py` 里默认开启远距透视补偿：

- 只压纵向像素，不收左右
- 目标越远，纵向像素偏移压得越多
- 可用 `PERSPECTIVE_COMP_GAIN` 调强弱，`PERSPECTIVE_COMP_Z_REF_M` 调参考距离

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
