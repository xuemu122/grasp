# Jetson 盒子部署说明（依赖与注意事项）

本文说明如何在一台 Jetson 盒子上部署本仓库：机械臂抓取（`dipan/` + `识别/`）和
点单/订单服务（`qianduan/`）。一台盒子对应一台设备、一个门店；两台设备分别放两个
门店时，两台盒子使用同一套部署流程，只改设备/门店标识和 FRP 端口。

## 1. 硬件与系统前提

| 项目 | 说明 |
| --- | --- |
| 主控 | Jetson Orin（JetPack 6 / Ubuntu 22.04，Python 3.10） |
| 机械臂 | DOBOT Nova5，控制网段 `192.168.5.1`，控制端口 `29999` |
| 相机 | Orbbec / RealSense 兼容 RGB-D |
| 灵巧手 | DH116（默认，走 Nova 末端 RS485）或 Aero Hand（`HAND_BACKEND=aero`） |
| 底盘 | TCP `192.168.10.10:31001`，栅格地图 HTTP `192.168.10.10:9001` |
| 运行账号 | `nvidia`（仓库内的绝对路径按此账号设计） |

**目录必须放在 `/home/nvidia/grasp`。** 代码里有多处硬编码路径：

- 视觉模型和映射：`/home/nvidia/grasp/识别/best.engine`、`/home/nvidia/grasp/识别/best.pt`、`/home/nvidia/grasp/识别/shelf_mapping.yaml`
- Aero SDK：`/home/nvidia/grasp/aero-hand-open/sdk/src`
- 自碰撞 URDF/SRDF：`/home/nvidia/grasp/dobot_ws/src/DOBOT_6Axis_ROS2_V4/...`
- Aero 空载曲线：`/home/nvidia/grasp/dipan/runtime/aero_empty.csv`

如果换目录，需要自己加软链接或改代码。模型路径可用 `YOLO_MODEL_PATH` 覆盖，
映射路径暂不可覆盖。

## 2. 目录布局

```text
/home/nvidia/grasp/
├── dipan/                  抓取主程序、标定、底盘控制（Python）
├── 识别/                   YOLO/RGB-D 识别（Python，需模型文件）
├── qianduan/               点单页 + Node 订单服务
├── aero-hand-open/         Aero Hand SDK（可选）
├── dobot_ws/               Nova5 URDF/SRDF（自碰撞护栏用，不需要跑 ROS2）
├── LHandProLib-API-Linux/  DH116 灵巧手 SDK
└── start_frontend.sh       启动 qianduan 的示例脚本
```

## 3. 依赖安装

### 3.1 系统包

```bash
sudo apt update
sudo apt install -y git curl build-essential python3-pip python3-venv \
  v4l-utils usbutils
```

### 3.2 Node（qianduan）

Node **18 以上**（`server.js` 使用全局 `fetch`，测试使用 `node:test`）。推荐 20 LTS：

```bash
node -v   # 需要 >= 18
npm -v
cd /home/nvidia/grasp/qianduan
npm install          # 依赖：ws、qrcode、xlsx
```

本次新增的设备/门店管理没有引入新的 npm 依赖，正常 `npm install` 即可。

### 3.3 Python（dipan + 识别）

抓取主链路最小依赖：

```bash
pip3 install numpy opencv-python PyYAML ultralytics pyserial matplotlib
```

- `ultralytics`：YOLO 推理（`识别/orbbec_detect.py`）。
- `opencv-python`：图像处理和 `cv2.aruco` 标定。
- `pyserial`：DH116 / Aero 串口。
- `matplotlib`：仅调试可视化使用，可选。
- OCR（`easyocr`）默认在抓取链路关闭（`use_ocr=False`），不装也能跑。

相机 SDK（二选一或都装）：

```bash
# Orbbec（当前主用）
# 按官方文档安装 pyorbbecsdk / pyorbbecsdk2，并配置库路径
export PYTHONPATH=/home/nvidia/pyorbbecsdk/install/lib:$PYTHONPATH
export LD_LIBRARY_PATH=/home/nvidia/pyorbbecsdk/install/lib:$LD_LIBRARY_PATH

# RealSense（回退方案）
pip3 install pyrealsense2
```

灵巧手 SDK：

- DH116（默认 `HAND_BACKEND=lhandpro_dobot`）：SDK 已随仓库放在
  `LHandProLib-API-Linux/`，`dipan/lhandpro_gripper.py` 会按相对路径加载
  `lib/libLHandProLib.so` 和 RS485 Python 示例；只需 `pyserial`。
- Aero Hand（`HAND_BACKEND=aero`）：按 `aero-hand-open/` 内说明安装 `aero_open_sdk`
  等依赖（其 `ros2/requirements.txt` 含 `torch`、`dex_retargeting` 等，仅重定向功能需要）。

> **不要直接在 Jetson 上 `pip install -r 识别/requirements.txt`。**
> 该文件是桌面环境的完整 `pip freeze`（含 CUDA 版 `torch==2.12`/`torchvision`、
> `numpy==2.2`、`paddlepaddle` 等），和 JetPack 的 CUDA/Python 版本不匹配。
> Jetson 上请使用 NVIDIA 提供的 JetPack 对应 torch/torchvision 轮子，或沿用盒子
> 现有环境；只补装缺失的基础包。

### 3.4 相机 udev 与串口权限

```bash
# Orbbec udev 规则（按 SDK 自带脚本）
cd /home/nvidia/OrbbecSDK*/Script
sudo ./install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

# 灵巧手串口权限（USB-RS485 或直连）
sudo usermod -aG dialout nvidia
# 重新登录后生效；临时可用 sudo chmod 666 /dev/ttyUSB0
ls -l /dev/ttyUSB*
```

### 3.5 模型与标定文件（不在仓库里，必须单独拷入）

- `识别/best.engine`（Jetson 优先，TensorRT）或 `识别/best.pt`。
  仓库 `.gitignore` 忽略了 `*.pt`/`*.onnx`/`*.engine`，**模型不会随 git 下发**。
- `识别/shelf_mapping.yaml` 随仓库提供，如现场商品有变化需同步更新。
- `dipan/calib_eye_in_hand.json`：手眼标定结果，随仓库提供；换手/换相机/重装后必须重新标定。
- `dipan/shelf_layers.json`、`place_point.json`、`place_transition.json`、`stow_pose.json`：
  现场层位/放置点/收起姿态，随仓库提供，重新部署后需按现场复核。
- `dipan/runtime/aero_empty.csv`：使用 Aero 柔性抓取前必须先录制（见 `dipan/README.md`）。

## 4. 环境配置

管理员账号、会话密钥等写进 `/etc/grasp-qianduan.env`（权限 `640`，组 `nvidia`）：

```bash
sudo chown root:nvidia /etc/grasp-qianduan.env
sudo chmod 640 /etc/grasp-qianduan.env
```

盒子端关键变量（按盒子区分）：

```text
# 本台设备的门店归属（两台盒子分别不同）
DEVICE_SN=GRASP-ROBOT-01
STORE_ID=STORE-001
STORE_NAME=门店A
OPERATIONS_DB_FILE=/home/nvidia/grasp/qianduan/data/operations.json

# 执行模式与硬件地址
ROBOT_GRASP_MODE=real
DOBOT_IP=192.168.5.1
CHASSIS_HOST=192.168.10.10
CHASSIS_PORT=31001
CHASSIS_MAP_HTTP_HOST=192.168.10.10
CHASSIS_MAP_HTTP_PORT=9001
CHASSIS_PLACE_MARKER=fangzhi

# 管理员与订单密钥（至少 32 字节，云端与盒子保持一致）
ADMIN_USERNAME=admin
ADMIN_PASSWORD=<至少16位随机密码>
ADMIN_SESSION_SECRET=<至少32字节随机值>
ORDER_TOKEN_SECRET=<至少32字节随机值>
ORDER_STATUS_CALLBACK_URL=https://rzza.dxtxl.com:8083
ORDER_STATUS_CALLBACK_SECRET=<至少32字节随机值>
```

第二台盒子改成 `GRASP-ROBOT-02` / `STORE-002` / `门店B`。

## 5. 启动

### 5.1 手动前台启动（首次调试）

```bash
cd /home/nvidia/grasp/qianduan
set -a
source /etc/grasp-qianduan.env
set +a

PORT=3000 WS_PORT=3001 ROBOT_GRASP_MODE=real DOBOT_IP=192.168.5.1 \
CHASSIS_HOST=192.168.10.10 CHASSIS_PORT=31001 \
CHASSIS_MAP_HTTP_HOST=192.168.10.10 CHASSIS_MAP_HTTP_PORT=9001 \
CHASSIS_PLACE_MARKER=fangzhi \
node server.js
```

正常日志：

```text
HTTP 服务已启动: http://0.0.0.0:3000
商品管理页面: http://localhost:3000/admin.html
抓取模式: 真实机械臂 (real)
WebSocket 服务已启动: ws://0.0.0.0:3001
```

### 5.2 systemd 常驻（推荐）

`/etc/systemd/system/grasp-qianduan.service`：

```ini
[Unit]
Description=Grasp Qianduan Order Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=nvidia
Group=nvidia
WorkingDirectory=/home/nvidia/grasp/qianduan
EnvironmentFile=/etc/grasp-qianduan.env
Environment=PORT=3000
Environment=WS_PORT=3001
Environment=ROBOT_GRASP_MODE=real
Environment=DOBOT_IP=192.168.5.1
Environment=CHASSIS_HOST=192.168.10.10
Environment=CHASSIS_PORT=31001
Environment=CHASSIS_MAP_HTTP_HOST=192.168.10.10
Environment=CHASSIS_MAP_HTTP_PORT=9001
Environment=CHASSIS_PLACE_MARKER=fangzhi
ExecStart=/usr/bin/node server.js
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now grasp-qianduan
sudo systemctl status grasp-qianduan --no-pager
```

> 生产不要用 `npm start`：它会先跑 `init_db.js`，可能覆盖现有商品库。
> 直接 `node server.js`。

## 6. 部署后验证

```bash
# 端口是否只被一个服务占用
ss -ltnp | grep -E ':3000|:3001'

# 盒子 HTTP 服务
curl http://127.0.0.1:3000/api/admin/session
# 新接口已生效：未登录返回 401 而不是 404
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:3000/api/admin/devices

# 底盘地图
curl http://192.168.10.10:9001/maps/ceshichangjing/1/map.yaml

# FRP 隧道
sudo systemctl status frpc --no-pager
```

浏览器打开 `http://<盒子IP>:3000/admin.html`，登录后确认左侧有“设备管理/运营管理”。

## 7. 注意事项

1. **路径固定**：除非加软链接，必须部署到 `/home/nvidia/grasp`。
2. **不要动第三方资源**：`aero-hand-open/`、`dobot_ws/`、`LHandProLib-API-Linux/`
   是依赖，不要在业务重构时移动或删除。生产不跑 `dobot_ws` 的 ROS2，只读取其中的
   URDF/SRDF 做自碰撞检查。
3. **模型与标定是运行资源**：`best.pt/best.engine`、`calib_eye_in_hand.json`、
   货架/放置/收起位姿 JSON 不在缓存清理范围内，别删。`__pycache__`、`dobot_ws/build|log`、
   `/tmp/*.jpg` 可清。
4. **真实模式前先离线验证**：先用模拟模式或 `TARGET_PRODUCT_QUERY` 单目标测试，
   确认相机、手眼标定、层板过滤和深度/自碰撞护栏正常后再接单。
5. **灵巧手不能直接换 SDK**：Nova 若只提供 Modbus RTU 透传，不能承载 SDK 的原始
   RS485 回调帧，先用 Jetson USB-RS485 测试；换手后必须重新实测末端几何参数。
6. **网络**：机械臂 `192.168.5.1`、底盘 `192.168.10.10` 通常在不同网口/网段，
   确认两块网卡都在，否则抓取或导航会失败。首次订单会先导航到商品点位再启动抓取。
7. **端口冲突**：不要同时跑两个 `server.js`（3000/3001 会占用失败）。启动前用
   `ss -ltnp` 检查，必要时停掉旧进程。
8. **证书时间**：Jetson 时间不同步会导致云端 HTTPS 回调失败，`sudo timedatectl set-ntp true`。
9. **两台设备的部署关系**：机械臂/相机/底盘是单机能力，每台盒子独立；设备/门店台账
   建议只放在云端集中管理，盒子只需配置 `DEVICE_SN/STORE_ID/STORE_NAME` 给订单打归属。
   两台盒子接入同一域名时 FRP 远程端口要错开（如 A `5060/5061`、B `5062/5063`）。
10. **急停**：管理页“急停”会直接停 DOBOT 和底盘并锁定服务，之后必须人工确认安全再重启后端。

更多现场参数、标定和测试命令见 [`dipan/README.md`](dipan/README.md) 和
[`识别/README.md`](识别/README.md)；服务器与 8083 站点部署见
[`qianduan/DEPLOY.md`](qianduan/DEPLOY.md)。
