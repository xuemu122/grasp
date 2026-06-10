# orca_core1 — Orca 灵巧手 SDK

Orca 17 关节灵巧手控制库，含硬件驱动、校准、手部追踪等。

## 目录

```
orca_core1/
├── orca_core/        核心库 (硬件驱动、手配置、关节计算)
│   ├── hardware/     Feetech / Dynamixel 马达驱动
│   ├── models/v2/    手模型配置 (orcahand_right)
│   └── utils/        工具函数
├── scripts/          官方工具脚本
├── mine/             **自定义程序**
└── tests/            单元测试
```

## 自定义程序 (mine/)

| 文件 | 说明 |
|------|------|
| `hand_tracking.py` | **手部追踪**: 摄像头捕捉人手 → MediaPipe 关键点 → 灵巧手实时跟随 |
| `preset_action.py` | **预设动作**: 位置1→位置2 平滑过渡 |
| `preset_action2-5.py` | 不同预设动作序列变体 |
| `demo.py` | 终端命令控制: 输入 1 抓取, 2 释放 |
| `read_motor_status.py` | 读取马达状态 |
| `hand_landmarker.task` | MediaPipe 手部关键点模型 |

### hand_tracking.py

摄像头捕捉人手姿态，MediaPipe 21 点手部关键点映射到 Orca 17 关节电机，实时跟随。

```bash
cd /home/nvidia/grasp/orca_core1

# 安装依赖
pip install mediapipe opencv-python

# 启动
python mine/hand_tracking.py
```

### demo.py

终端命令控制灵巧手。

```bash
python mine/demo.py --steps 40 --delay 0.03
# 输入 1 抓取
# 输入 2 释放
```

### preset_action.py

预设动画演示。

```bash
python mine/preset_action.py
python mine/preset_action.py --steps 50 --delay 0.03
```

## 官方工具 (scripts/)

| 脚本 | 功能 |
|------|------|
| `main_demo.py` | 完整演示 |
| `calibrate.py` | 关节校准 |
| `check_motor.py` | 马达检查 |
| `configure_motor_chain.py` | 配置马达链 |
| `record_angles.py` | 记录关节角度 → YAML |
| `replay_angles.py` | 回放关节角度 |
| `record_continuous.py` | 连续录制轨迹 |
| `replay_continuous.py` | 连续回放轨迹 |
| `tension.py` / `neutral.py` / `zero.py` | 手部姿态预设 |
| `stress_test.py` | 压力测试 |

### 录制和回放

```bash
# 录制关节角度 (手动摆姿后记录)
python scripts/record_angles.py

# 回放
python scripts/replay_angles.py path/to/sequence.yaml
```

### 校准

```bash
python scripts/calibrate.py
```

## 安装

```bash
cd /home/nvidia/grasp/orca_core1
pip install -e .
```

## 在代码中使用

```python
import sys
sys.path.insert(0, "/home/nvidia/grasp/orca_core1")
from orca_core import OrcaHand, OrcaJointPositions

hand = OrcaHand()
hand.connect()
hand.init_joints()
# ... 控制 ...
hand.disconnect()
```
