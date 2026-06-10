# 货架药品抓取系统

基于 Jetson Orin + DOBOT Nova5 + Orca 灵巧手 + RealSense D405 的端到端药品货架识别与抓取系统。

## 硬件

| 部件 | 型号 |
|------|------|
| 机械臂 | DOBOT Nova5 |
| 灵巧手 | Orca 17 关节灵巧手 |
| 深度相机 | Intel RealSense D405 |
| 计算平台 | NVIDIA Jetson Orin (L4T 36.4) |
| 底盘 | 自主导航底盘 (192.168.124.90:31001) |

## 目录

```
grasp/
├── 识别/          视觉识别 (YOLO + EasyOCR + RealSense)
├── dipan/         机械臂抓取 / 底盘控制
├── orca_core1/    Orca 灵巧手 SDK + 手部追踪
└── dobot_ws/      DOBOT ROS2 Humble 工作空间
```

## 环境

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **Python**: 3.10
- **PyTorch**: >= 2.5
- **关键依赖**: ultralytics, easyocr, pyrealsense2, opencv-python, orca_core

## 快速开始

```bash
# 1. 视觉识别
cd 识别 && python3 orbbec_detect.py

# 2. 货架抓取
cd dipan && python3 shelf_grasp.py

# 3. 底盘控制
cd dipan && python3 move_robot.py
```

各子模块详细说明见对应目录下的 README。
