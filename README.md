# 货架商品识别与抓取系统

当前主线是 Jetson Orin + DOBOT Nova5 + RealSense/Orbbec 兼容 RGB-D 相机 + Aero Hand + 自主导航底盘的货架商品识别、抓取、放置流程。

统一入口是：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

## 当前目录

```text
grasp/
├── dipan/           当前机械臂抓取主程序、手眼标定、底盘控制
├── qianduan/        点单前端和 Node WebSocket/REST 服务
├── 识别/            YOLO/RGB-D 识别、候选诊断、数据采集工具
├── aero-hand-open/  当前 Aero Hand SDK
├── dobot_ws/        DOBOT Nova5 ROS2/URDF/MoveIt 资源
├── dataset/         本地训练/测试数据集
└── dataset_captures/现场采集样本
```

历史/实验目录：

```text
drp/        DRP/Franka/pointnet2/robofin 实验代码，当前货架抓取主线不用
orca_core1/ 旧 Orca 灵巧手代码，当前已经切换到 Aero Hand
```

如果当前仓库只保留 Nova5 + Aero Hand 货架抓取主线，可以删除 `drp/` 和 `orca_core1/`，并同步提交删除。

## 常用命令

货架抓取：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

指定目标商品/类别：

```bash
TARGET_PRODUCT_QUERY=雪碧 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
TARGET_PRODUCT_QUERY=class_id:63 DOBOT_IP=192.168.5.1 python3 shelf_grasp.py
```

主程序里也可以输入 `?` 查看模型类别，输入 `c` 设置目标。

前端点单服务：

```bash
cd /home/nvidia/grasp/qianduan
npm start
```

`npm start` 会自动执行 `init_db.js`，把 Excel 商品表导入到本地 JSON 商品库 `qianduan/data/products.json`。不需要安装或启动 MySQL/MariaDB。前端依赖在 `package.json` 中，目前只有 `ws`、`xlsx`；目录里已有 `node_modules` 时不需要重复安装，缺依赖时执行 `npm install`。

默认是模拟抓取，浏览器打开 `http://localhost:3000`。如果要把订单接到真实机械臂抓取：

```bash
cd /home/nvidia/grasp/qianduan
ROBOT_GRASP_MODE=real DOBOT_IP=192.168.5.1 npm start
```

前端每个商品会通过 `TARGET_PRODUCT_QUERY` 调用一次 `dipan/shelf_grasp.py` 的单次自动抓取模式；真实模式下请先确认机械臂、相机、货架层和放置点已经按主程序调好。

手眼标定：

```bash
cd /home/nvidia/grasp/dipan
DOBOT_IP=192.168.5.1 python3 hand_eye_calib.py
```

底盘移动：

```bash
cd /home/nvidia/grasp/dipan
python3 move_robot.py yaojia1
python3 move_robot.py charge
python3 move_robot.py place
python3 move_robot.py cancel
```

视觉识别调试：

```bash
cd /home/nvidia/grasp/识别
./run.sh --conf 0.05 --no-ocr
```

采集数据集样本：

```bash
cd /home/nvidia/grasp/识别
./capture_frames.py -o dataset_captures
```

## 当前关键行为

- 抓取程序启动时固定默认回到第 2 层，避免从上次低层状态恢复造成碰撞。
- 货架层位姿保存在 `dipan/shelf_layers.json`。
- 放置点保存在 `dipan/place_point.json`，放置过渡点保存在 `dipan/place_transition.json`。
- 收起姿态保存在 `dipan/stow_pose.json`，退出程序时会主动回收起姿态。
- 第 4 层允许低/负 Z，因为当前基座下方是可用空间；其他层仍保留低 Z 拒绝保护。
- 抓取目标先按指定商品/类别过滤，再按 ROI、当前层世界 Z、深度分层和右侧优先规则选择。
- 当前灵巧手使用 Aero Hand；侧抓默认瓶子手势，上抓使用上端手势。
- 深度护栏会排除检测框、自手部深度点，并检查对齐/探入路径。
- Nova5 自碰撞护栏默认开启，基于 URDF/SRDF 和胶囊近似在 MovL/MovJ 前检查。
- 抓取主程序里 `b` 可切换底盘放置流程，开启后会调用 `move_robot.py place`。

## 清理建议

当前主线不需要以下历史目录：

```bash
git rm -r drp orca_core1
```

如果暂时不想提交删除，只想清掉未追踪依赖缓存：

```bash
rm -rf drp/pointnet2_ops drp/robofin
```

可随时清理的缓存/构建产物：

```bash
rm -rf dipan/__pycache__ 识别/__pycache__ drp/__pycache__
rm -rf dobot_ws/build dobot_ws/log
```

`dobot_ws/install/` 是 ROS2 安装产物；如果当前环境还依赖它，先不要删。

更多现场操作见 [dipan/README.md](dipan/README.md)。
