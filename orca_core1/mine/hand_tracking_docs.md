# hand_tracking.py 详细文档

## 概述

本程序实现了 **摄像头实时手部跟踪 → OrcaHand 机械手控制** 的完整链路。通过摄像头捕获人手姿态，使用 MediaPipe 深度学习模型检测 21 个手部关键点，将关节角度映射到机械手电机位置，实现人手到机械手的实时跟随。

## 系统架构

```
┌──────────┐    ┌──────────────┐    ┌───────────────┐    ┌──────────────┐    ┌──────────┐
│  摄像头   │───→│ MediaPipe    │───→│ 角度→电机映射  │───→│ 指令队列(线程) │───→│ 机械手   │
│ (主线程)  │    │ 关键点检测   │    │ + 平滑滤波    │    │ (10Hz限频)   │    │ (串口)   │
└──────────┘    └──────────────┘    └───────────────┘    └──────────────┘    └──────────┘
```

**关键设计**：电机控制运行在独立线程中，与 UI/摄像头主线程解耦，避免串口通信阻塞导致窗口卡死。

---

## 依赖安装

```bash
pip install mediapipe opencv-python
```

模型文件 `hand_landmarker.task` 需放在 `mine/` 目录下（首次运行时会自动下载）。

---

## 用法

```bash
# 默认配置（右手，0号摄像头）
python mine/hand_tracking.py

# 指定配置文件
python mine/hand_tracking.py --config orca_core/models/v2/orcahand_touch_left/config.yaml

# 选择1号摄像头
python mine/hand_tracking.py --camera 1

# 调整平滑度（0=无延迟但抖动，1=非常平滑但延迟大）
python mine/hand_tracking.py --smoothing 0.7

# 自定义握紧电流
python mine/hand_tracking.py --grip-current 150
```

### 键盘操作

| 按键 | 功能 |
|------|------|
| `q` | 退出程序 |
| `r` | 重置到中性位置（张手） |
| `g` | 切换跟踪/握紧模式 |

---

## 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--config` | str | None | config.yaml 路径，默认使用右手配置 |
| `--camera` | int | 0 | 摄像头设备号 |
| `--smoothing` | float | 0.5 | 平滑系数 (0~1)，越大越平滑但延迟越大 |
| `--max-current` | int | 300 | 最大电流 (mA) |
| `--no-grip` | flag | False | 禁用握紧功能 |
| `--grip-current` | int | 120 | 握紧保持电流 (mA) |
| `--grip-threshold` | float | 0.7 | 自动握紧触发的屈曲阈值 (0~1) |

---

## 核心模块详解

### 1. MediaPipe 手部关键点检测

使用 MediaPipe **Tasks API**（v0.10.35+）的 `HandLandmarker` 模型，从每帧图像中检测 21 个手部关键点：

```
        4(拇指尖)
        |
    3(拇指IP)
        |
    2(拇指MCP)
        |
    1(拇指CMC)
        |
8(食指尖)  12(中指尖)  16(无名指尖)  20(小指尖)
|           |            |             |
7(食指DIP)  11(中指DIP)  15(无名指DIP)  19(小指DIP)
|           |            |             |
6(食指PIP)  10(中指PIP)  14(无名指PIP)  18(小指PIP)
|           |            |             |
5(食指MCP)  9(中指MCP)   13(无名指MCP)  17(小指MCP)
  \          |            |            /
   \_________|____________|___________/
                        |
                    0(手腕)
```

### 2. 手指屈曲度计算 — `compute_finger_flex()`

**输入**：MediaPipe 检测到的 21 个关键点坐标，以及指定手指的关键点索引。

**输出**：屈曲度 0~1（0=伸直，1=完全弯曲）。

**算法**：

- **拇指**（特殊处理）：计算 CMC→MCP 和 MCP→IP 两向量之间的夹角
  ```
  v1 = MCP - IP    (近端方向)
  v2 = TIP - IP    (远端方向)
  angle = arccos(v1 · v2 / (|v1| × |v2|))
  flex = angle / π
  ```

- **其他四指**：计算 PIP 和 DIP 两个关节的弯曲角，取平均
  ```
  angle1 = arccos((MCP-PIP) · (DIP-PIP) / ...)   # PIP关节角
  angle2 = arccos((PIP-DIP) · (TIP-DIP) / ...)   # DIP关节角
  flex = (angle1 + angle2) / 2 / π
  ```

### 3. 手指外展度计算 — `compute_abduction()`

**输入**：两根手指的 MCP 关键点索引。

**输出**：外展度 0~1（0=并拢，1=最大展开）。

**算法**：计算两指 MCP 点之间的距离，除以手掌大小进行归一化：
```
spread = |MCP1 - MCP2|
hand_size = |MCP1 - wrist| + |MCP2 - wrist|
abduction = clip(spread / hand_size * 3.0 - 0.5, 0, 1)
```

### 4. 手腕偏转计算 — `compute_wrist_angle()`

**输出**：-1~1（负=左偏，正=右偏）。

**算法**：计算中指 MCP 与手腕在水平方向的偏差。

---

## HandTracker 类

核心跟踪器，封装了 MediaPipe 模型初始化、帧处理、电机映射、平滑滤波。

### 初始化参数

| 参数 | 说明 |
|------|------|
| `smoothing` | 平滑系数 (0~1)。0=无平滑（响应快但抖动），1=极平滑（延迟大） |

### 关键方法

| 方法 | 功能 |
|------|------|
| `process_frame(frame_rgb)` | 处理一帧 RGB 图像，返回 MediaPipe 检测结果 |
| `get_motor_commands(detection_result)` | 将检测结果映射为 17 个电机目标位置 (rad) |
| `get_avg_flex(detection_result)` | 获取四指平均屈曲度 (0~1) |
| `draw_landmarks(frame, detection_result)` | 在图像上绘制手部关键点和连接线 |
| `release()` | 释放 MediaPipe 模型资源 |

### 电机映射逻辑

映射的核心思想是**线性插值**：每个电机在"张手"（`NEUTRAL_MOTOR_POS`）和"握拳"（`FIST_MOTOR_POS`）之间根据屈曲度插值：

```python
motor_pos = neutral + flex * (fist - neutral)
```

具体映射表（右手配置）：

| 手指 | 外展电机 | 屈曲电机(MCP) | 屈曲电机(PIP) |
|------|----------|---------------|---------------|
| 食指 | 1(index_abd) | 2(index_mcp) | 3(index_pip) |
| 中指 | 5(middle_abd) | 9(middle_mcp) | 4(middle_pip) |
| 无名指 | 11(ring_abd) | 10(ring_mcp) | 12(ring_pip) |
| 小指 | 7(pinky_abd) | 6(pinky_mcp) | 8(pinky_pip) |
| 拇指 | 13(thumb_abd) | 14(thumb_cmc) + 15(thumb_mcp) + 16(thumb_dip) |
| 手腕 | 17(wrist) | — | — |

**注意**：拇指的 CMC、MCP、DIP 都使用 `thumb_flex` 屈曲度，`thumb_abd` 单独用于外展电机。手腕的偏转范围被缩放到 30%（`* 0.3`），避免过大运动。

### 平滑滤波

使用指数移动平均（EMA）防止电机抖动：

```python
new_pos = α × prev_pos + (1-α) × raw_pos
```

- `α = 0`：无平滑，直接响应
- `α = 0.5`：中等平滑（默认）
- `α = 0.8`：非常平滑，但有明显延迟

---

## motor_worker() — 电机控制线程

在独立线程中运行，通过 `Queue` 接收主线程的指令，避免串口通信阻塞 UI。

### 指令类型

| 指令 | 数据 | 说明 |
|------|------|------|
| `"motor_pos"` | `{motor_id: position_rad}` | 设置电机目标位置 |
| `"grip_on"` | None | 切换到纯电流模式握紧 |
| `"grip_off"` | None | 切回位置控制模式 |
| `"reset"` | None | 重置到中性位置 |
| `None` | — | 退出线程 |

### 关键设计

1. **限频 10Hz**（`min_interval = 0.1s`）：避免串口被指令淹没导致卡死
2. **握紧时不发位置命令**：`current` 模式下位置命令无效，发送反而会导致模式冲突
3. **队列 `maxsize=1`**：只保留最新指令，旧指令自动丢弃，避免延迟累积

---

## 两种工作模式

### TRACK 模式（默认）

机械手处于 `current_based_position` 模式，根据摄像头检测的手指屈曲度实时调整电机位置。电机碰到物体后会以设定电流保持位置。

### GRIP 模式（按 `g` 切换）

机械手切换到 `current` 模式，所有手指（除手腕）以 `--grip-current` 设定的电流持续施力握紧。此时位置命令暂停发送，避免模式冲突。再次按 `g` 切回 TRACK 模式。

---

## UI 显示

窗口分为两部分：

**左侧 — 摄像头画面**：
- 绿色线：手部关键点连接
- 红色点：21 个关键点
- 文字信息：屈曲度、模式、FPS

**右侧 — 电机状态面板**：
- 每个电机的进度条显示归一化位置（0=张手，1=握拳）
- 颜色：蓝色=张开，绿色=握拳
- 格式：`电机ID:关节名 [进度条]`

---

## 预设位置配置

| 变量 | 含义 | 来源 |
|------|------|------|
| `NEUTRAL_MOTOR_POS` | 张手时各电机位置 (rad) | 来自 preset_action 的位置1 |
| `FIST_MOTOR_POS` | 握拳时各电机位置 (rad) | 来自 preset_action3 的位置2 |

**如需调整**：修改脚本顶部的这两个字典即可。建议用 `read_motor_status.py` 录制你自己的张手和握拳位置。

---

## 常见问题

### Q: 窗口黑屏
A: 检查摄像头是否被其他程序占用，尝试 `--camera 1` 选择其他摄像头。

### Q: 机械手不动
A: 检查右侧面板的进度条是否在变化。如果不变化，说明 MediaPipe 未检测到手；如果变化但手不动，检查串口连接。

### Q: 机械手方向反了
A: 修改 `NEUTRAL_MOTOR_POS` 和 `FIST_MOTOR_POS` 中对应电机的位置值，或在 config.yaml 中给电机 ID 加负号。

### Q: 跟随延迟大
A: 降低 `--smoothing` 值（如 0.3），但会更抖动。这是平滑度和延迟的权衡。

### Q: 机械手抖动
A: 增大 `--smoothing` 值（如 0.7），或降低 `--max-current` 减小电机力度。

### Q: 握紧后无法松开
A: 按 `r` 键重置到中性位置，或按 `g` 切回 TRACK 模式。

---

## 文件结构

```
mine/
├── hand_tracking.py          # 本脚本
├── hand_landmarker.task      # MediaPipe 模型文件
├── preset_action.py          # 预设动作1（拇指后动）
├── preset_action2.py         # 预设动作2（拇指后动）
├── preset_action3.py         # 预设动作3（同步）
├── preset_action4.py         # 预设动作4（同步）
└── read_motor_status.py      # 电机状态读取
```
