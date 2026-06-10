# dipan — 机械臂抓取与底盘控制

与 DOBOT Nova5 机械臂、Orca 灵巧手、自主导航底盘的集成控制脚本。

## 文件

| 文件 | 说明 |
|------|------|
| `shelf_grasp.py` | **货架边缘抓取主程序**: YOLO 检测 → 选最靠边盒子 → 侧边缘+上边缘抓取 |
| `box_grasp.py` | 盒子抓取位姿计算库 (手眼标定 + 坐标转换 + Orca 控制) |
| `hand_eye_calib.py` | **手眼标定**: 求解法兰→相机精确变换矩阵 |
| `move_robot.py` | **底盘控制**: TCP 指令移动底盘到预置点位 |
| `capture_one.py` | 单帧采集工具: 拍一帧 D405 彩色+深度图 |
| `calib_eye_in_hand.json` | 手眼标定结果 (hand_eye_calib.py 生成, shelf_grasp.py 自动加载) |

## 主程序 1: shelf_grasp.py

### 功能

- YOLO 检测货架上所有盒子
- 自动选中**最靠边**的盒子 (X 方向离画面中心最远)
- 抓取**侧边缘 + 上边缘** (从外侧上方进入，避免撞到相邻盒子)
- 自动连接 DOBOT (使能 + 拖动示教) + Orca 灵巧手
- 一键抓取流程: 张手 → 悬停 → 下降 → 握拳 → 抬升

### 启动

```bash
cd /home/nvidia/grasp/dipan
python3 shelf_grasp.py
```

### 操作流程

1. 拖动机械臂到能看见货架的观察位姿
2. 按 `Enter` → 拍照 → 识别 → 自动抓取最靠边的盒子
3. 抓完后拖动到放货位, 再按 `Enter` 抓下一个
4. 按 `q` 退出

### 参数

可在脚本顶部修改:
- `DOBOT_IP`: 机械臂 IP (默认 192.168.1.6)
- `HAND_LENGTH`: 灵巧手长度 0.35m
- `HAND_WEIGHT`: 灵巧手重量 1.2kg
- `HAND_COG_Z`: 重心距法兰 0.10m

### 抓取策略

```
货架排列 (俯视):
  ┌──┐ ┌──┐ ┌──┐ ┌──┐
  │A │ │B │ │C │ │D │    A 和 D 是最靠边的
  └──┘ └──┘ └──┘ └──┘
  ↑抓                          抓↑
```

- 选中最靠边的盒子 → 从**外侧**上方进入
- 抓取 **侧边缘 (x1/x2) + 上边缘 (y1)**
- 加 3cm 外偏偏移量，避免撞到相邻盒子

### 手眼标定 (使用前建议执行一次)

`hand_eye_calib.py` 求解法兰→相机精确变换。标定后 `shelf_grasp.py` 自动加载，抓取无需反复微调。

```bash
python3 hand_eye_calib.py
```

流程:
1. 生成 ArUco marker → 打印贴墙上
2. 量 marker 实际边长 (mm)，改脚本里 `MARKER_SIZE_MM`
3. 拖动机械臂在 marker 前方不同姿态按 Enter 采集 (10-15 组)
4. 按 `q` 求解，结果保存为 `calib_eye_in_hand.json`
5. 之后 `shelf_grasp.py` 启动时自动加载

### 调试微调

shelf_grasp.py 每次抓取前进入交互界面:

| 按键 | 功能 |
|------|------|
| `Enter` | 执行抓取 |
| `p` | 预览悬停位 |
| `wasd` | 微调 XY (±5mm) |
| `e/r` | 微调 Z (±5mm) |
| `=/ -` | 改变步长 |
| `f` | 手动输入偏移 |
| `F5` | 重新拍照 |
| `Q` | 跳过 |

### 硬件连接

- DOBOT: TCP 192.168.1.6:29999
- Orca 灵巧手: U2D2 串口
- RealSense D405: USB 3.0

---

## 主程序 2: move_robot.py

底盘 TCP 控制，移动到预置导航点位。

### 启动

```bash
# 交互模式
python3 /home/nvidia/grasp/dipan/move_robot.py

# 直接下发
python3 /home/nvidia/grasp/dipan/move_robot.py yaojia1   # 去药架1
python3 /home/nvidia/grasp/dipan/move_robot.py charge    # 去充电桩
python3 /home/nvidia/grasp/dipan/move_robot.py cancel    # 取消任务
```

### 连接

底盘 TCP: `192.168.124.90:31001`

---

## box_grasp.py

抓取位姿计算类库，提供:
- `LightweightBoxGrasper`: 手眼标定 + 像素深度→世界坐标转换
- `OrcaHandGripper`: 灵巧手预设姿态 (张开 / power_grasp)
- `BoxDetection`: 统一数据结构, 兼容 orbbec_detect 输出

### 使用

```python
from box_grasp import LightweightBoxGrasper, BoxDetection

grasper = LightweightBoxGrasper()
grasper.set_flange_pose(x, y, z, rx, ry, rz)  # 设置当前法兰位姿
pose = grasper.compute_grasp(detection)         # 计算抓取点
```

## capture_one.py

单帧采集工具。

```bash
python3 capture_one.py
```

保存到 `~/dipan/`: `color.png`, `depth_raw.png`, `depth_colormap.png`
