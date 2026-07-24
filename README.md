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

点单前端、云端 Nginx、FRP、盒子 WebSocket 协议和排障说明见
[`qianduan/README.md`](qianduan/README.md)。

## 当前推荐启动方式

盒子上的订单服务、机械臂 worker、底盘导航和管理页地图统一由 `qianduan/server.js` 启动。管理员账号和共享会话密钥从 `/etc/grasp-qianduan.env` 加载，不要把真实密钥直接写进命令或仓库。

先确认没有旧服务占用端口：

```bash
ss -ltnp | grep -E ':3000|:3001'
```

没有输出时，在盒子上前台启动：

```bash
cd /home/nvidia/grasp/qianduan

set -a
source /etc/grasp-qianduan.env
set +a

PORT=3000 \
WS_PORT=3001 \
ROBOT_GRASP_MODE=real \
DOBOT_IP=192.168.5.1 \
CHASSIS_HOST=192.168.124.90 \
CHASSIS_PORT=31001 \
CHASSIS_MAP_HTTP_HOST=192.168.124.90 \
CHASSIS_MAP_HTTP_PORT=9001 \
CHASSIS_PLACE_MARKER=yaojia1 \
node server.js
```

该命令前台运行，终端不能关闭；按 `Ctrl+C` 停止。看到以下两行表示 HTTP 和 WebSocket 已启动：

```text
HTTP 服务已启动: http://0.0.0.0:3000
WebSocket 服务已启动: ws://0.0.0.0:3001
```

本机管理页是 `http://localhost:3000/admin.html`。通过云端访问时，Nginx/FRP 仍分别把 `/api/chassis/` 和 `/ws/` 转发到盒子 `3000/3001`。管理员账号是 `admin`，当前内网密码是 `admin123`。

启动后可在另一个终端检查：

```bash
curl http://127.0.0.1:3000/api/admin/session
curl http://192.168.124.90:9001/maps/shiyanshi_01/1/map.yaml
```

第一条返回 JSON 说明盒子 HTTP 服务正常；第二条返回 `origin`、`resolution` 等 YAML 字段说明桥接后的底盘地图服务正常。Node 每次启动会更新一次 `qianduan/data/chassis_map_cache/` 本地地图快照，管理页平时读取本地 PNG/YAML 和点位元数据，底盘临时断网时继续使用上一次成功快照。管理页中的“抓取/放置前导航”默认关闭，放置点默认 `yaojia1`，开启后配置会保存到 `qianduan/data/chassis_settings.json`。首次订单保持机械臂现有的收起姿态，先让底盘到达商品点位；只有到点后才启动抓取 worker，依次进入 Home、初始观察位和商品对应层观察位。worker 已运行时，后续跨点位订单会先把机械臂收起，再移动底盘。到达目标观察层后会明确张开 Aero Hand，再开始拍照识别。导航开启时，未绑定底盘点位的商品会直接报错并停止，不会在错误位置继续抓取。

仓库职责、生产依赖方向、工具脚本和清理边界见 [`CODEMAP.md`](CODEMAP.md)。旧 Orca 实验脚本
`dipan/box_grasp.py` 仍保留在历史文件中，但不属于当前生产入口。

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

## 云端点单部署

点单系统拆成云端和盒子端两部分：

- 云端 Nginx 提供静态网页，`/api/` 连接云端商品库服务。
- 云端网页使用同域 WebSocket `/ws/` 下发订单。
- Nginx 将 `/ws/` 转发到 FRP 服务端 `127.0.0.1:5061`。
- FRP 将 `5061` 转到盒子 `127.0.0.1:3001`。
- 盒子 `server.js` 接收订单并按商品数量调用 `shelf_grasp.py`。

完整链路：

```text
浏览器 wss://<站点>/ws/
  -> 云端 Nginx
  -> 云端 127.0.0.1:5061
  -> FRP
  -> 盒子 127.0.0.1:3001
  -> qianduan/server.js
  -> dipan/shelf_grasp.py
```

盒子端启动命令：

```bash
cd /home/nvidia/grasp/qianduan

set -a
source /etc/grasp-qianduan.env
set +a

PORT=3000 \
WS_PORT=3001 \
ROBOT_GRASP_MODE=real \
DOBOT_IP=192.168.5.1 \
CHASSIS_HOST=192.168.124.90 \
CHASSIS_PORT=31001 \
CHASSIS_MAP_HTTP_HOST=192.168.124.90 \
CHASSIS_MAP_HTTP_PORT=9001 \
CHASSIS_PLACE_MARKER=yaojia1 \
node server.js
```

真实抓取直接使用云端订单里的商品 `name`，不依赖盒子本地商品库校验。每件商品按 `qty` 逐件写入常驻 worker；收到收起指令后，服务向 worker 写入 `**stow**` 并等待其真正退出。

FRP 增加 WebSocket TCP 通道：

```toml
[[proxies]]
name = "admin_order_ws"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3001
remotePort = 5061
```

FRP 在线只表示隧道可用，不代表盒子后端或机械臂就绪。页面状态区分为：后端未连接、后端在线但机械臂待启动、机器人就绪、执行中和收起中。只有常驻 worker 输出“等待商品名”后才算机器人就绪。

真实模式下请先确认机械臂、相机、货架层和放置点已经按主程序调好。详细的端口、完整 FRP 示例、WebSocket JSON 协议、部署步骤和排障命令见 [`qianduan/README.md`](qianduan/README.md)。

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
- 抓取目标先按指定商品/类别过滤，再优先使用当前深度帧识别的上下层板边界过滤；识别失败时回退 ROI/当前层世界 Z，之后按深度分层和右侧优先规则选择。详细动态层板参数见 [`dipan/README.md`](dipan/README.md)。
- `dipan/shelf_grasp.py` 支持只读层板过滤预览：交互模式输入 `f`，常驻 worker 写入 `__filter_preview__`，生成 `/tmp/shelf_filter_preview.jpg` 和对应 JSON 报告，不会移动或抓取。
- 当前灵巧手使用 Aero Hand；侧抓默认瓶子手势，上抓使用上端手势。
- 深度护栏会排除检测框、自手部深度点，并检查对齐/探入路径。
- Nova5 自碰撞护栏默认开启，基于 URDF/SRDF 和胶囊近似在 MovL/MovJ 前检查。
- 抓取主程序里 `b` 可切换底盘放置流程，开启后会调用 `move_robot.py place`。

## 本地清理

常见缓存和构建产物可以在确认没有 ROS2 进程使用后清理：

```bash
rm -rf dipan/__pycache__ 识别/__pycache__
rm -rf dobot_ws/build dobot_ws/log
```

`dobot_ws/install/` 是 ROS2 安装产物；如果当前环境还依赖它，先不要删。模型、标定和位姿 JSON
属于运行资源，不要当作缓存删除。

更多现场操作见 [dipan/README.md](dipan/README.md)。
