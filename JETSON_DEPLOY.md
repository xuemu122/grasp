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

Node **18 以上**（`server.js` 使用全局 `fetch`，测试使用 `node:test`）。推荐 20 LTS。

> **注意（实测坑）**：Ubuntu 22.04 / JetPack 自带的 `nodejs` 只有 **12.x**，太老，
> 会报 `SyntaxError: Unexpected token '?'`。而且盒子上的 `apt` 可能因为
> `librealsense` 源未签名而 `apt update` 失败，导致 NodeSource 脚本装不上。
> 最稳的是直接下官方 arm64 压缩包，解到 `/usr/local`：

```bash
cd /tmp
VER=$(curl -fsSL https://nodejs.org/dist/latest-v20.x/ | grep -oE 'node-v[0-9.]+-linux-arm64\.tar\.xz' | head -n1)
curl -fsSLO "https://nodejs.org/dist/latest-v20.x/$VER"
sudo tar -xJf "$VER" -C /usr/local --strip-components=1
hash -r
node -v   # 期望 v20.x
npm -v
```

```bash
cd /home/nvidia/grasp/qianduan
npm install          # 依赖：ws、qrcode、xlsx
```

（若坚持走 apt：先 `sudo sed -i '/librealsense.intel.com/s/^/#/' /etc/apt/sources.list.d/*.list`
修掉未签名源，再 `sudo apt-get update && sudo apt-get install -y nodejs`。）

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
  在没有该文件的盒子上，用 Aero 后端直接录一条（灵巧手接着、机械臂上电）：
  ```bash
  cd /home/nvidia/grasp/dipan && mkdir -p runtime
  HAND_BACKEND=aero TOOL_HAND_MODEL=aero DOBOT_IP=192.168.5.1 \
  python3 test_aero_current_grasp.py --active-joints 1,2,3,4,5,6 \
    --max 60 --step 2 --fast --disable-trigger --release-after \
    --log /home/nvidia/grasp/dipan/runtime/aero_empty.csv
  ```
  （缺该文件时 Aero 后端不会报错，只会退化成绝对电流阈值，抓取判定略差。）

### 3.6 PyTorch / torchvision（JetPack 6.1，最容易踩坑）

Jetson 上必须用 **NVIDIA 的 CUDA 版** torch，不能用 PyPI 的 CPU 版，否则抓取 worker 报
`AssertionError: Torch not compiled with CUDA enabled`。本次实测可用组合：

- torch `2.5.0a0+872d972e41.nv24.08`（CUDA 12.6，与第一台一致）
- torchvision `0.20.0a0`（本机源码编译）

**torch**（走 NVIDIA 中国镜像，快；注意文件名里的构建号 `.17622132`，少了会 404）：
```bash
python3 -m pip uninstall -y torch torchvision
python3 -m pip install --no-deps \
  https://developer.download.nvidia.cn/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

若 `import torch` 报 `libcusparseLt.so.0: cannot open shared object file`，装 cuSPARSELt
（0.6.2 仅 13MB，配 CUDA 12.6）：
```bash
cd /tmp
wget -c https://developer.download.nvidia.cn/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.6.2.3-archive.tar.xz
tar xJf libcusparse_lt-linux-aarch64-0.6.2.3-archive.tar.xz
sudo cp -a libcusparse_lt-linux-aarch64-0.6.2.3-archive/lib/libcusparseLt.so* /usr/lib/aarch64-linux-gnu/
sudo ldconfig
```

**nvcc（编 torchvision 需要）**：JetPack 只带 CUDA 运行库，`nvcc` 要单独装：
```bash
sudo sed -i '/librealsense.intel.com/s/^/#/' /etc/apt/sources.list.d/*.list 2>/dev/null
sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-6
export PATH=/usr/local/cuda/bin:$PATH
nvcc --version   # release 12.6
```

**torchvision**：PyPI 的 aarch64 轮子和 Jetson torch ABI 不匹配（报
`RuntimeError: operator torchvision::nms does not exist`）；从别的盒子拷现成的又会因
编译器/libstdc++ 不同而断言崩溃（`stl_vector.h:1130 ... Aborted`）。所以必须**本机源码编译**：
```bash
sudo apt-get install -y git libjpeg-dev libpng-dev libtiff-dev ninja-build
python3 -m pip install -U setuptools wheel
cd /tmp
wget -c https://github.com/pytorch/vision/archive/refs/tags/v0.20.0.tar.gz -O /tmp/vision-0.20.0.tar.gz
tar xzf /tmp/vision-0.20.0.tar.gz
nohup bash -c 'export PATH=/usr/local/cuda/bin:$PATH; export MAX_JOBS=2; cd /tmp/vision-0.20.0 && python3 setup.py bdist_wheel && python3 -m pip install --no-deps dist/torchvision-0.20.0*.whl' > /tmp/tvbuild.log 2>&1 &
tail -f /tmp/tvbuild.log     # 约 5~15 分钟，看到 Successfully installed 即完成
python3 -c "import torch, torchvision; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torchvision.__version__)"
```
（PyPI 上 torchvision 0.20.0 只有轮子、没有 sdist，所以用 GitHub 源码。GitHub 慢时可换镜像前缀。）

### 3.7 OCR 依赖：本机不要装 onnxruntime / onnx / easyocr

`easyocr→onnxruntime→onnx` 的 aarch64 轮子是用较新工具链（gcc-toolset-14）编的，在
JetPack 的 libstdc++ 上导入会直接崩溃：
```text
stl_vector.h:1130: ... Assertion '__n < this->size()' failed.
Aborted (core dumped)
```
抓取链路默认 `use_ocr=False` 用不到它们；**不要装**。已装的话卸载（否则连
`import torchvision` 都会经 onnx 的注册链被带崩）：
```bash
python3 -m pip uninstall -y easyocr onnxruntime onnx
```

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

# 灵巧手后端（按硬件选）：DH116 走 Nova 末端 RS485；Aero 走 aero-hand-open SDK
# 注意：worker 实际用的型号来自 data/chassis_settings.json 的 hand_model（管理页“机器人末端灵巧手”可改），
# 这里的 TOOL_HAND_MODEL 只是“文件里没有 hand_model”时的默认值。
HAND_BACKEND=lhandpro_dobot      # DH116（默认）；Aero 手改为 aero
TOOL_HAND_MODEL=dh116            # DH116 用 dh116；Aero 手用 aero
# HAND_GRASP_MODE=flex           # 默认电流柔性；Aero 缺空载曲线时可设 auto 先跑通

# 管理员与订单密钥（至少 32 字节，云端与盒子保持一致）
ADMIN_USERNAME=admin
ADMIN_PASSWORD=<至少16位随机密码>
ADMIN_SESSION_SECRET=<至少32字节随机值>
ORDER_TOKEN_SECRET=<至少32字节随机值>
ORDER_STATUS_CALLBACK_URL=https://rzza.dxtxl.com:8083
ORDER_STATUS_CALLBACK_SECRET=<至少32字节随机值>
```

第二台盒子把 `DEVICE_SN` / `STORE_ID` / `STORE_NAME` 改成它自己的（例如云端生成的
`GRASP-6B0CDA` / `STORE-002` / `科技园`）。

关于 `DEVICE_SN`：盒子读取优先级为 **环境变量 `DEVICE_SN` > 文件 `/etc/grasp-qianduan.device-sn`
> `PAYMENT_DEVICE_SN` > 默认 `GRASP-ROBOT-01`**。云端“新增设备”会自动生成 SN，把同一个值写到
盒子即可（env 或文件二选一，`env` 会覆盖文件）：
```bash
echo 'GRASP-6B0CDA' | sudo tee /etc/grasp-qianduan.device-sn
sudo chmod 644 /etc/grasp-qianduan.device-sn
```
只有 `curl http://127.0.0.1:3000/api/device/info` 返回的 `sn` 与云端台账里该设备一致，
管理页/商家页才会显示“在线”。

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

## 8. 多台盒子接入 FRP

两台盒子接同一个 frps，**`serverAddr`/`serverPort`/`auth.token` 相同，`remotePort` 和
proxy `name` 必须不同**（否则第二台报 `port already used` 或 `proxy already exists`）。
端口建议错开：第一台 `6001/5060/5061`，第二台 `6002/5062/5063`。

第二台 `frpc.toml` 示例（`/etc/frp/frpc.toml`）：
```toml
serverAddr = "1.194.211.170"
serverPort = 7000
auth.method = "token"
auth.token = "<与 frps 相同的 token>"

[[proxies]]
name = "edge_ssh_admin2"
type = "tcp"
localIP = "127.0.0.1"
localPort = 22
remotePort = 6002

[[proxies]]
name = "admin_backend2"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3000
remotePort = 5062

[[proxies]]
name = "admin_order_ws2"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3001
remotePort = 5063
```

`frpc` 二进制要**对应 CPU 架构**（Jetson 是 `linux_arm64`；用错会 `cannot execute binary file: Exec format error`）。
systemd 常驻：
```ini
# /etc/systemd/system/frpc.service
[Unit]
Description=frp client
After=network.target

[Service]
Type=simple
Restart=on-failure
RestartSec=5
ExecStart=/usr/local/bin/frpc -c /etc/frp/frpc.toml

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl daemon-reload && sudo systemctl enable --now frpc
journalctl -u frpc -n 30 --no-pager | grep -iE 'success|error|token'
```

云端侧（多盒子）需要配置门店→盒子映射，并把 nginx `/ws` 指到云端 WS（而非单个盒子）。
详见 [`qianduan/DEPLOY.md`](qianduan/DEPLOY.md)：`BOX_PROXY_MAP`、`BOX_PROXY_WS_MAP`、
`location /test-api/`、`/ws → 127.0.0.1:3101`。

## 9. 常见问题速查（本次多台部署踩坑）

| 现象 | 原因 / 处理 |
| --- | --- |
| `frpc: cannot execute binary file: Exec format error` | frpc 架构不对，换 `frp_*_linux_arm64` |
| frps 日志 `token in login doesn't match` | 第二台 frpc 的 `auth.token` 和 frps 不一致 |
| frps 日志 `port already used` / `already exists` | 两台 remotePort 或 proxy name 撞了 |
| `SyntaxError: Unexpected token '?'` | 系统自带 Node 12 太老，装 Node 20（见 3.2） |
| `ImportError: libcusparseLt.so.0` | 缺 cuSPARSELt，装 0.6.2（见 3.6） |
| `Torch not compiled with CUDA enabled` | 装了 PyPI CPU 版 torch，换 NVIDIA CUDA 版（见 3.6） |
| `operator torchvision::nms does not exist` | torchvision 轮子与 Jetson torch ABI 不匹配，源码编译（见 3.6） |
| `stl_vector.h ... Assertion ... Aborted` | onnxruntime/onnx/easyocr 的轮子与 JetPack libstdc++ 不兼容，卸载（见 3.7） |
| `柔性抓取缺少空载曲线: .../lhandpro_empty.csv` | 用了 lhandpro 后端但没录曲线；Aero 手应设 `HAND_BACKEND=aero TOOL_HAND_MODEL=aero`，或录 `aero_empty.csv`（见 3.5） |
| URDF/模型路径 404（`/home/nvidia/grasp/...`） | 目录名不是 `grasp`；要么改名，要么用相对路径版代码并用 `GRASP_ROOT` 指定 |
| 云端管理页设备一直“未接入” | 盒子 `/api/device/info` 的 `sn` 与云端台账该设备 `sn` 不一致（见 4） |
| 云端探测报 `fetch failed: bad port` | Node 的 `fetch` 禁用了 5060/5061 等端口；云端访问盒子要用 `http` 模块（当前代码已改） |
| 商家页提示“盒子端管理员会话配置不一致” | 云端代管签发的管理员会话被盒子拒。两端 `ADMIN_SESSION_SECRET`（共享密钥，信任边界）必须一致；新版已放宽 `readAdminSession`：只要求「签名有效 + user 非空 + 未过期」，因此各盒子可用不同 `ADMIN_USERNAME`（如 `admin01`）。旧代码还要求 `ADMIN_USERNAME` 与云端相同 |
| 盒子本机 `curl -b <本机登录cookie> .../api/chassis/map` 401 | 本机管理员账号密码不对或 `server.js` 进程没加载 env；用 `tr '\0' '\n' < /proc/$(pgrep -f 'node server.js'|head -1)/environ \| grep ADMIN_` 看进程实际值 |
| 点单页一直不弹登录 | 浏览器里已有同源的商家登录 Cookie（SSO）；点“退出”换账号，或新开无痕窗口 |
| worker 仍是 `lhandpro_dobot` / 缺 `lhandpro_empty.csv`（但本机是 Aero 手） | 灵巧手型号取 `data/chassis_settings.json` 的 `hand_model`，不是 `HAND_BACKEND` 环境变量；管理页「机器人末端灵巧手」改成 Aero，或改该文件 `hand_model=aero` 后重启 |
| MySQL 迁移导入报 `near '> qianduan@1.0.0 export-mysql ...'` | 用了 `npm run export-mysql`，npm 横幅混进 SQL；改用 `node tools/migrate-json-to-mysql.js` |
| 迁移报 `Duplicate entry '' for key 'stores.uk_stores_code'` | 空 `code` 撞唯一键；脚本已改为写 NULL，旧 SQL 先 `ALTER TABLE stores DROP INDEX uk_stores_code;` |

## 10. 点单页登录与多门店

- **生产点单页**（`/`）用**商家账号**登录（商家后台里创建的账号，如 `admin`/北校区、
  `admin01`/科技园），登录后顶部可选门店/设备，商品按门店过滤，下单带上所选门店/设备。
  盒子本地 http 访问会自动跳过登录。
- **测试页**（`/test.html`）改用**平台管理员账号**登录后才免支付（`/test-api/orders/prepare`
  需要已登录管理员）。

更多现场参数、标定和测试命令见 [`dipan/README.md`](dipan/README.md) 和
[`识别/README.md`](识别/README.md)；服务器与 8083 站点部署见
[`qianduan/DEPLOY.md`](qianduan/DEPLOY.md)。
