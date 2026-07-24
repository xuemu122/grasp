# 代码地图

这份仓库以“盒子端订单服务 -> 常驻抓取 worker -> 机械臂/灵巧手/底盘”为主线。
目录中的标定、调试和厂商资源不参与每次订单执行，不能和生产入口混用。

## 生产链路

```text
qianduan/server.js
  -> dipan/shelf_grasp.py
       -> orbbec_detect.py       相机与 YOLO/RGB-D 识别
       -> target_selection.py    目标筛选与多帧融合
       -> grasp_planner.py       层板边界、抓取点和抓取方向
       -> hand_eye.py             像素/深度 -> 相机 -> 法兰 -> 基座
       -> depth_collision_guard.py  深度障碍护栏
       -> dobot_arm.py            DOBOT TCP、运动等待、报警恢复
       -> self_collision_guard.py  URDF/SRDF 自碰撞检查
       -> aero_gripper.py         Aero Hand 开合与持续保持
       -> move_robot.py            底盘导航到货架/放置/充电点
```

`qianduan/script.js` 和 `admin.html` 只负责页面交互；订单状态、worker 生命周期、底盘锁和完成时序均由 `server.js` 负责。`识别/` 是视觉模型和采集工具目录，生产运行时由 `shelf_grasp.py` 导入其中的 `orbbec_detect.py`。

## 配置与数据

- `dipan/shelf_layers.json`：货架层观察位。
- `dipan/place_point.json`、`place_transition.json`：放置点和过渡点。
- `dipan/stow_pose.json`：收起姿态。
- `dipan/calib_eye_in_hand.json`：手眼标定结果。
- `qianduan/data/products.json`：商品库；`uploads/`、`chassis_map_cache/` 为运行时数据。
- `aero-hand-open/`、`dobot_ws/`：第三方 SDK、URDF 和 ROS2 工作区，属于依赖资源，不要在业务重构时移动或删除。

## 工具与测试

- 标定：`dipan/hand_eye_calib.py`。
- 底盘命令行：`dipan/move_robot.py`。
- Aero 电流/接触实验：`dipan/test_aero_current_grasp.py`、`dipan/demo_aero_contact_states.py`。
- 相机单帧和可视化：`dipan/capture_one.py`、`dipan/debug_visualize.py`。
- Python 单元测试：`dipan/tests/` 以及 `dipan/test_*.py` 中的硬件模拟测试。
- Node 协议测试：`qianduan/test/`，不连接真实机械臂。

`dipan/box_grasp.py` 是旧 Orca 手的历史实验脚本，当前入口没有引用；它保留用于查阅旧实验，不应作为生产抓取程序启动。

## 启动入口

真实盒子端：

```bash
cd /home/nvidia/grasp/qianduan
set -a && source /etc/grasp-qianduan.env && set +a
ROBOT_GRASP_MODE=real node server.js
```

直接调试机械臂：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

本地模拟和测试不需要真实设备；真实密码、会话密钥和运行日志不应提交到仓库。

## 清理边界

可以清理的内容：`__pycache__/`、ROS2 `build/`/`log/`、本地 `.env`、运行日志和生成的调试图片。当前 `.gitignore` 已覆盖这些常见产物。

不要直接清理的内容：`dataset/`、`dataset_captures/`、模型权重、标定 JSON、货架/放置/收起位姿、`aero-hand-open/` 和 `dobot_ws/`。这些文件虽然不在订单主链路中，但会影响重新部署、标定复现或碰撞检查。
