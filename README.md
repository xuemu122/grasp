# 货架商品识别与抓取系统

基于 Jetson Orin、DOBOT Nova5、RealSense D405、Aero Hand 和自主导航底盘的货架商品识别、抓取、放置与底盘移动控制项目。

## 目录

```text
grasp/
├── 识别/          视觉识别、低阈值调试、数据集抽帧
├── dipan/         机械臂抓取、手眼标定、底盘 TCP 控制
├── aero-hand-open/ Aero Hand SDK
├── dobot_ws/      DOBOT ROS2 工作空间
└── orca_core1/    旧 Orca 灵巧手相关代码
```

## 常用命令

```bash
# 货架抓取
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py

# 动作节奏调试：缩短运动后缓冲、状态轮询和抓取等待
DETECT_CONF_THRESHOLD=0.3 MOTION_SETTLE_S=0.15 GRASP_SETTLE_S=0.3 HAND_CLOSE_WAIT_S=2.5 ROBOT_POLL_S=0.15 PLACE_TRANSITION_CP=60 PLACE_TRANSITION_R=30 PLACE_RETURN_CLEARANCE_MM=140 PLACE_RETURN_MAX_Z_MM=820 \
  DOBOT_IP=192.168.5.1 python3 shelf_grasp.py

# 手眼标定
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 hand_eye_calib.py

# 底盘移动
cd /home/nvidia/grasp/dipan
python3 move_robot.py yaojia1
python3 move_robot.py charge
python3 move_robot.py place
python3 move_robot.py cancel

# 视觉识别调试
cd /home/nvidia/grasp/识别
./run.sh --conf 0.05 --no-ocr

# DRP / Nova5 预览（MobaXterm X11）
cd /home/nvidia/grasp
python3 drp/nova5_drp_test.py --vis --no-camera

# 数据集抽帧，默认保存 10 张
cd /home/nvidia/grasp/识别
./capture_frames.py -o dataset_captures
```

## 当前关键行为

- 抓取程序启动时固定默认回到第 2 层，避免从上次低层状态恢复造成碰撞。
- 统一入口仍是 `dipan/shelf_grasp.py`；DOBOT 控制在 `dipan/dobot_arm.py`，灵巧手在 `dipan/aero_gripper.py`，手眼变换在 `dipan/hand_eye.py`，目标选择/抓取规划在 `dipan/grasp_planner.py`。
- 货架层位姿保存在 `dipan/shelf_layers.json`，放置点保存在 `dipan/place_point.json`，放置过渡点保存在 `dipan/place_transition.json`，放置弧线中点保存在 `dipan/place_arc_via.json`，收起姿态保存在 `dipan/stow_pose.json`。
- 第 4 层允许低/负 Z，因为当前基座下方是可用空间；其他层仍保留低 Z 拒绝保护。
- 第 3/4 层抓取后先上撤到第 1 层再放置；第 4 层会上撤经过第 3 层再到第 1 层。
- 如果记录了放置过渡点，放置可选走 `Arc` 或 `MovL` 平滑曲线；放置后回层会先原地抬高再回层。
- 抓取目标会先按画面纵向 ROI 过滤上下层，再在当前层从右往左选择目标。
- 抓取主程序会按当前层记录位姿的世界 Z 再过滤一次候选，默认容差 `LAYER_Z_TOLERANCE_M=0.16`，防止误抓上一层/下一层。
- 侧面抓取点会保持在物体底部上方至少 6cm；如果物体高度不够，会自动切换为上面抓取。
- 当前灵巧手侧面抓默认使用瓶子抓取手势；上端抓取仍使用上端手势。如需侧面抓也回到原手势，可用 `HAND_GRASP_MODE=auto` 启动。
- `drp/nova5_drp_test.py` 默认走本地 X11/OpenCV 预览；在当前 Jetson 上 `pointnet2_ops` CUDA 扩展不可用时会自动降级到纯 PyTorch 实现。
- 机械臂 `MovL`/`MovJ` 会把代码里的 `speed` 参数下发为 DOBOT `v=1..100`；动作节奏可用 `MOTION_SETTLE_S`、`GRASP_SETTLE_S`、`HAND_CLOSE_WAIT_S`、`ROBOT_POLL_S` 微调，放置回程安全高度可用 `PLACE_RETURN_CLEARANCE_MM` 调整。
- 抓取主程序里 `b` 可切换底盘放置流程：开启后会调用 `move_robot.py place`，但不再强制在放置前自动收起机械臂。
- 抓取主程序识别阈值默认 `DETECT_CONF_THRESHOLD=0.3`；独立识别调试程序会打印 YOLO 原始候选数量，方便排查新模型无框问题。
- `move_robot.py` 下发底盘移动后默认等待 `/api/robot_status` 的 `move_status=succeeded`，并额外查询 `/api/get_power_status` 返回 `power_status`。

各子目录的详细操作见对应 README。
