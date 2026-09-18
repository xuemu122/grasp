# 点单前端与订单服务

本目录包含静态点单页面、商品 REST API 和盒子端订单 WebSocket 服务。生产环境采用“云端页面和商品库、盒子执行机械臂”的部署方式。

## 支付、测试页与订单管理

订单记录保存在 `ORDERS_DB_FILE`（默认 `data/orders.json`），商品库存仍保存在 `PRODUCTS_DB_FILE`。订单准备接口会先校验库存并落库；`PAYMENT_MODE=free` 时直接标记为测试支付成功，`PAYMENT_MODE=lyorder` 时调用 LyOrder 的 `/api/Order/create` 和 `/api/WeChatPay/native`。云端主动查询未支付订单，不依赖浏览器在线；确认付款成功时立即预占库存，完成时消费预占，退款时幂等释放。

正式服务不要把支付密钥写进仓库，使用环境文件配置：

```text
PAYMENT_MODE=lyorder
PAYMENT_API_BASE_URL=http://127.0.0.1:8080
PAYMENT_API_TOKEN=...
PAYMENT_DEVICE_SN=GRASP-ROBOT-01
ALLOW_FREE_ORDERS=0
```

测试服务使用单独的 Node/WS 端口和数据目录，设置 `PAYMENT_MODE=free`、`ALLOW_FREE_ORDERS=1`，并把测试虚拟主机（例如现有的 `8083text`）的 `/`、`/api/`、`/ws/` 全部代理到该实例。也可以把同一静态目录的 `/test.html` 暴露为测试入口；它复用点单页面但自动请求免费模式。不要让正式虚拟主机把 `ALLOW_FREE_ORDERS` 设为 `1`。

管理员页面新增订单分页接口：

```text
GET  /api/admin/orders?page=1&pageSize=10&status=COMPLETED&search=咖啡
GET  /api/admin/orders/:orderId
POST /api/admin/orders/:orderId/refund   {"reason":"用户取消"}
```

管理员页面新增设备与门店管理接口（均需管理员会话）：

```text
GET    /api/admin/stores
POST   /api/admin/stores
PUT    /api/admin/stores/:storeId
DELETE /api/admin/stores/:storeId?force=1
GET    /api/admin/devices?status=&storeId=&search=
POST   /api/admin/devices
PUT    /api/admin/devices/:deviceId
DELETE /api/admin/devices/:deviceId
POST   /api/admin/devices/:deviceId/checkout   {"storeId":"STORE-001","location":"一层大厅","notes":"..."}
POST   /api/admin/devices/:deviceId/checkin    {"location":"总仓A区","notes":"..."}
GET    /api/admin/operations/summary
PUT    /api/admin/orders/:orderId/assignment   {"storeId":"STORE-001","deviceId":"DEV-001"}
GET    /api/admin/merchants
POST   /api/admin/merchants                    {"username":"storeA","password":"...","storeIds":["STORE-001"]}
PUT    /api/admin/merchants/:merchantId
DELETE /api/admin/merchants/:merchantId
```

设备状态为 `in_stock`（在库）、`deployed`（已投放）、`repair`（维修中）、`retired`（已停用）。出库会把设备状态改为 `deployed` 并绑定门店，入库会解绑门店并回到 `in_stock`；删除门店时其下设备会自动回库。订单 API 也支持 `storeId`、`deviceId`（含 `unassigned`）筛选。

管理端拆成两个独立界面，各自独立登录：

| 页面 | 使用者 | 能力 |
| --- | --- | --- |
| `admin.html` | 平台方（我们） | 全部功能 + 设备管理、门店管理、商家账号 |
| `merchant.html` | 商家 | 只看绑定门店：订单、门店信息、只读设备；无出入库权限 |

商家账号由平台在 `admin.html` → “商家账号”里创建，可绑定一个或多个门店，密码以 scrypt 加盐哈希存入 `operations.json`。商家端接口：

```text
GET    /api/merchant/session
POST   /api/merchant/login    {"username":"storeA","password":"..."}
POST   /api/merchant/logout
GET    /api/merchant/overview
GET    /api/merchant/products?search=&category=
POST   /api/merchant/products             {"name":"...","price":8,"stock":100,"storeId":"STORE-001",...}
PUT    /api/merchant/products/:id
DELETE /api/merchant/products/:id
POST   /api/merchant/uploads              （商品图片，multipart/form-data，字段名 file）
GET    /api/merchant/orders?page=1&pageSize=10&status=&search=
```

商家接口只返回、且只能修改 `storeIds` 内门店的订单、设备与商品，无法调用任何 `/api/admin/*` 接口：
商品新增会强制归属到自己的门店，修改/删除他人门店商品返回 403。设备对商家只读。
商家端会话使用独立 Cookie `grasp_merchant_session`，签名密钥为 `MERCHANT_SESSION_SECRET`（未配置时复用 `ADMIN_SESSION_SECRET`）。

商品现在带 `storeId`（归属门店）：平台在 `admin.html` 商品表单里选择“归属门店”，商家端“我的商品”只显示自己门店的商品。现有仓库示例商品统一归属 `STORE-001`（北校区）。平台商品接口 `GET /api/products` 也支持 `storeId` 筛选。

免费订单退款只恢复本地库存；真实订单退款会转发到 LyOrder `/api/WeChatPay/refund`。每笔退款持久化唯一 `refundRequestId`，并通过请求体与 `Idempotency-Key` 重复使用；LyOrder 应按该键幂等处理。部分退款请求同时携带 `partial=true`、元单位的 `refundAmount/totalAmount`，以及分单位的 `amount.refund/amount.total`；LyOrder 必须按明确金额退款，不能忽略这些字段后退整单。库存预占和恢复都具有持久化幂等标记。

支付订单按单瓶记录履约。某一瓶识别、碰撞检查、导航或动作失败后，机器人先安全收起并继续尝试后续商品；全部单瓶任务结束后，只退款未交付商品并只恢复相应库存。每瓶成功放置后立即持久化交付记录，进程中途失败或执行超时也按该记录结算；只有一瓶都未交付时才整单退款。商品已经成功放置后，收起或回充失败不退该瓶。云端还会巡检付款后长期未开始执行、以及长时间停留在 `PROCESSING` 且没有 `placedAt` 的订单，默认均为 30 分钟后自动退款，用于覆盖页面关闭、盒子断电或进程崩溃。

机器人繁忙不属于履约失败：繁忙时不锁单、不执行、不退款，页面保留当前订单并在机器人恢复空闲后自动重试。正常情况下页面会在准备支付前根据 WebSocket 机器人状态直接禁止下单。

云端与盒子必须配置同一份随机回调密钥。云端不设置回调地址；盒子设置云端 API 地址：

```text
# 云端商品/支付服务
ORDER_TOKEN_SECRET=<云端与盒子相同的32字节以上随机值>
ORDER_STATUS_CALLBACK_SECRET=<32字节以上随机值>

# 盒子执行服务
ORDER_STATUS_CALLBACK_URL=https://rzza.dxtxl.com:8083
ORDER_TOKEN_SECRET=<与云端相同>
ORDER_STATUS_CALLBACK_SECRET=<与云端相同>
```

`ORDER_TOKEN_SECRET` 用于盒子验证云端签发的订单凭证，两端必须完全一致；它应与管理员会话密钥、回调密钥分别生成，不能复用。

订单执行顺序是“盒子签名锁单为 PROCESSING → 开始运动 → 成功放置后落盘完成回调 → 云端确认并扣减库存”。完成回调使用和失败回调相同的 HMAC 密钥，盒子会把未确认事件保存到 `ORDER_CALLBACK_OUTBOX_FILE`，断网或重启后继续重试；浏览器不再负责扣库存。云端收到完成回调后先持久化 `PLACED`，再扣库存，避免成功放置订单被超时退款。失败回调和退款均是幂等的，重复上报不会重复退款；未完成扣库存的失败订单不会错误增加库存。自动退款失败时订单保留为 `FAILED/PAID`，退款平台返回处理中时服务会周期性恢复巡检。

## 架构

```text
商品展示：浏览器 -> 云端 Nginx /api/ -> 云端 Node :3100 -> products.json

订单下发：浏览器 -> wss://<站点>/ws/ -> 云端 Nginx
         -> 云端 FRP :5061 -> 盒子 :3001 -> server.js
         -> shelf_grasp.py -> DOBOT / Aero Hand
```

订单提交前，云端商品 API 会校验商品与库存并签发订单凭证。盒子成功放置后通过 HMAC 完成回调通知云端，完成事件先落盘再重试，浏览器不参与库存扣减。云端先持久化 `PLACED` 再扣库存；完成记录持久化在商品 JSON 中，同一订单重复回传不会重复扣减，抓取失败不会扣库存。

正式 `lyorder` 模式下，盒子只接受带有效 `completionToken` 的抓取命令；缺失、过期或签名不正确的订单会在任何机械臂动作前被拒绝。免费测试模式保留无凭证的调试指令兼容性，不能暴露为正式点单服务。

云端商品服务和盒子订单服务都运行 `server.js`，但职责和环境变量不同：

| 位置 | HTTP | WebSocket | 模式 | 用途 |
| --- | ---: | ---: | --- | --- |
| 云端 | `127.0.0.1:3100` | `127.0.0.1:3101` | `sim` | 商品 REST API |
| 盒子 | `3000` | `3001` | `real` | 接收订单并控制机械臂 |
| FRP 服务端 | `5060` | `5061` | TCP | 分别转到盒子 `3000`、`3001` |

云端网页只通过 `/api/` 访问商品服务，通过 `/ws/` 访问盒子。浏览器不应直接连接或暴露 Node 内部端口。

## 文件

- `index.html`、`style.css`、`script.js`：点单页面。
- `admin.html`：平台管理页面，包含仪表盘、商品、订单、设备管理、门店管理、商家账号、自动巡架和底盘地图。
- `merchant.html`：商家端页面，登录后只看自己门店的订单、门店信息和只读设备。
- `data/operations.json`：设备与门店台账（出入库、门店归属），可通过 `OPERATIONS_DB_FILE` 调整路径。
- `uploads/`：管理端上传的商品图片（默认与商品 JSON 位于同一数据目录）。
- `server.js`：HTTP/API 与订单 WebSocket 服务。
- `start.js`：初始化商品数据后启动 `server.js`。
- `init_db.js`：从 Excel 初始化 `data/products.json`。
- `data/products.json`：默认 JSON 商品库。
- `tools/migrate-json-to-mysql.js`：把商品、门店、设备、商家和订单台账导出为 MySQL SQL；仅用于迁移评估，当前服务仍读取 JSON。
- `test/ws_protocol.test.js`：不连接真实机械臂的 WebSocket 集成测试。

本地环境变量示例见 [`.env.example`](.env.example)。`.env` 只用于现场临时保存配置，已加入忽略列表；
生产环境建议继续使用 `/etc/grasp-qianduan.env`，通过 `source` 注入进程环境，不要把真实密码和会话密钥提交到仓库。

## 双设备 / 双门店部署说明

> 本仓库到服务器、以及 8083 站点 Nginx 的具体部署步骤见 [`DEPLOY.md`](DEPLOY.md)。新页面仍在
> `admin.html` 内、新接口都在 `/api/` 下，沿用现有 8083 站点的代理，无需新增端口。

现在有两台机器人分别投放到两个不同的门店。推荐结构是“一台盒子对应一台设备和一个门店”，设备/门店台账由云端服务集中管理：

```text
门店A 点单页 (store-a.example.com) ─┐
门店B 点单页 (store-b.example.com) ─┼─> 云端 Node :3100  商品/订单/设备门店台账
云端管理中心 (admin.example.com)   ─┘
门店A 点单页 /ws/ -> 云端 Nginx -> FRP 5061 -> 盒子A :3001 -> 机械臂A
门店B 点单页 /ws/ -> 云端 Nginx -> FRP 5063 -> 盒子B :3001 -> 机械臂B
```

关键点：

1. **每台盒子独立启动** `server.js`，端口都用 `3000/3001`，但 FRP 远程端口必须错开，例如盒子 A 用 `5060/5061`、盒子 B 用 `5062/5063`。
2. **每台盒子写入自己的设备与门店标识**，订单会自动带上归属：
   ```text
   DEVICE_SN=GRASP-ROBOT-01
   STORE_ID=STORE-001
   STORE_NAME=门店A
   ```
   第二台改为 `GRASP-ROBOT-02` / `STORE-002` / `门店B`。
3. **云端服务保存设备/门店台账**（`OPERATIONS_DB_FILE`，默认 `data/operations.json`）。首次启动可用仓库自带的示例文件，随后在管理页“运营管理”里把门店名称、地址、联系人改成现场真实信息，并在“设备管理”里核对两台设备的 SN。
4. **按门店划分访问入口**：给每个门店一个独立域名或子路径，各自把 `/` 和 `/ws/` 代理到对应盒子；点单页链接带上门店与设备标识，订单就会归属到该门店/设备，例如：
   ```text
   https://store-a.example.com/?store=STORE-001&device=DEV-001
   https://store-b.example.com/?store=STORE-002&device=DEV-002
   ```
   没有带参数时订单仍可创建，只是归属为空，可在“运营管理 → 订单归属”里手动划分。
5. **设备出入库**：在“设备管理”或“运营管理”里对设备点“出库”并选择门店，即可把设备投放到门店并绑定归属；点“入库”则从门店收回并解除归属。两台设备可分别归属不同门店。
6. **底盘/机械臂控制仍是单机能力**。地图、急停、巡架等页面请通过对应盒子自己的管理页操作（`http://<盒子IP>:3000/admin.html` 或该门店域名）；云端的“运营管理”只负责订单、门店和设备的集中台账。

云端 `nginx.conf` 需要给每个门店增加一组 `/ws/` 转发，例如：

```nginx
# 门店A
location ^~ /ws/ {
    proxy_pass http://127.0.0.1:5061;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_read_timeout 86400s;
}

# 门店B（部署在另一个 server 块或子域名中）
location ^~ /ws/ {
    proxy_pass http://127.0.0.1:5063;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_read_timeout 86400s;
}
```

两台盒子的 `frpc.toml` 除远程端口外完全相同（SSH、后台和 WebSocket 各一条），例如盒子 B：

```toml
[[proxies]]
name = "edge_ssh_admin_b"
type = "tcp"
localIP = "127.0.0.1"
localPort = 22
remotePort = 6002

[[proxies]]
name = "admin_backend_b"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3000
remotePort = 5062

[[proxies]]
name = "admin_order_ws_b"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3001
remotePort = 5063
```

## 云端部署

云端 Nginx 直接托管静态文件，并使用以下两个反向代理：

```nginx
client_max_body_size 5m;

location ^~ /api/chassis/ {
    proxy_pass http://127.0.0.1:5060;
}

location ^~ /api/ {
    proxy_pass http://127.0.0.1:3100;
}

location ^~ /ws/ {
    proxy_pass http://127.0.0.1:5061;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
    proxy_buffering off;
    proxy_cache_bypass $http_upgrade;
    proxy_connect_timeout 10s;
    proxy_read_timeout 86400s;
    proxy_send_timeout 86400s;
}
```

云端商品服务建议仅绑定回环地址：

```bash
cd /path/to/grasp/qianduan
HOST=127.0.0.1 \
PORT=3100 \
WS_HOST=127.0.0.1 \
WS_PORT=3101 \
ROBOT_GRASP_MODE=sim \
PRODUCTS_DB_FILE=/var/lib/grasp-qianduan/products.json \
ADMIN_USERNAME=admin \
ADMIN_PASSWORD='<至少16位随机密码>' \
ADMIN_SESSION_SECRET='<云端与盒子共用的至少32字节随机密钥>' \
node server.js
```

管理端可上传或替换商品正面、反面图片，支持 JPG、PNG 和 WebP，默认单张最大 5 MB。图片上传接口需要管理员会话，图片读取地址保持公开供点单页展示。默认保存到 `PRODUCTS_DB_FILE` 同级的 `uploads/` 目录，也可通过以下环境变量调整：

```text
PRODUCT_UPLOAD_DIR=/var/lib/grasp-qianduan/uploads
PRODUCT_UPLOAD_MAX_BYTES=5242880
```

上传目录必须对运行 Node 服务的用户可写。若 Nginx 位于服务前方，其 `client_max_body_size` 不应小于 Node 的上传限制。

生产环境应使用 systemd 或其他进程管理器保持服务运行。`PRODUCTS_DB_FILE` 必须对运行服务的用户可写，否则商品新增、修改和删除会失败。

### 管理员登录

`/admin.html` 使用服务端会话登录。商品列表查询保持公开，供点单页读取；新增、修改和删除商品必须持有有效的管理员会话 Cookie。

当前内网部署使用以下管理员账号：

```text
ADMIN_USERNAME=admin
ADMIN_PASSWORD=<至少16位随机密码>
ADMIN_SESSION_SECRET=<云端与盒子共用的至少32字节随机密钥>
ADMIN_SESSION_TTL_MS=28800000
```

程序不提供管理员默认密码；正式支付实例检测到常见默认密码会拒绝启动。`ADMIN_SESSION_SECRET` 不要提交到仓库，云端和盒子必须配置完全相同的值。

Cookie 使用 `HttpOnly`、`Secure` 和 `SameSite=Strict`，默认有效期为 8 小时。同一来源 15 分钟内连续登录失败 5 次后会被临时限制。修改账号、密码或密钥后需要重启云端商品服务；修改密钥会使现有会话立即失效。

管理页的底盘地图接口通过 `/api/chassis/` 转发到盒子。云端和盒子必须设置相同的 `ADMIN_USERNAME` 与 `ADMIN_SESSION_SECRET`，盒子才能验证云端登录签发的管理员 Cookie。盒子每次启动从底盘 Web 服务的 `/maps/<地图名>/<楼层>/map.yaml` 和 `map.png` 更新一次本地地图快照，默认保存到 `data/chassis_map_cache/`：`current.json` 保存地图元数据和 marker，按地图/楼层目录保存原始 YAML 和 PNG。管理页普通刷新只读本地快照；底盘临时不可达时继续使用上一次成功快照。点击“刷新地图”才会强制重新从底盘更新，且没有可用快照时才降级为坐标网格。

管理页顶部的“抓取/放置前导航”“订单结束自动回充”开关和放置点保存在盒子的 `chassis_settings.json`。首次没有设置文件时默认关闭导航、开启自动回充，默认放置点为 `fangzhi`。开启导航后，首次订单保持机械臂现有收起姿态，先判断并前往商品绑定的 `chassis_marker`；底盘到点后才启动 worker，进入 Home、初始观察位和商品层观察位。worker 已运行时，后续跨点位订单会先收起机械臂再导航。到达目标观察层后 worker 会明确张开 Aero Hand，再开始拍照识别。导航开启但商品未绑定点位时，订单会直接失败，不会在错误位置继续抓取。抓取完成后检查底盘是否已在配置的放置点，已在点位则直接执行机械臂放置，否则先收起机械臂、导航到放置点并等待到达后再放置。整单完成后始终自动收起机械臂；自动回充开启时再继续回充，关闭时保持收起位置。收起和底盘导航期间服务端会拒绝新订单。常驻 worker 每次放置前重新读取配置，管理页修改后无需重启 worker。

管理页“机器人控制”中：

- “关闭并回充”会拒绝新操作，依次收起机械臂、断开灵巧手并关闭 DOBOT，最后让底盘回到 `CHASSIS_CHARGE_MARKER`。
- “急停”会直接通过 DOBOT 29999 控制端口发送 `EmergencyStop()`/`Stop()`，同时取消底盘导航并终止 worker。急停后服务端保持锁定，必须人工确认机械臂和底盘安全后重启后端服务。

管理页“自动巡架”配置使用 A-D 货架卡片。每张卡片独立保存或删除，每个货架可以配置 1-6 个底盘 marker，新货架默认生成 2 个点位；巡架顺序为货架、点位、第 1/2 层。旧版 1-7 号货架配置会在读取时按 A/B/C/D 自动迁移。管理页的每个点位和地图页都提供“前往点位”调试按钮，该操作以异步任务执行，真实模式下会拒绝正在执行订单、巡架或急停等操作。

巡架结果状态从盒子读取，但“保存扫描商品信息”使用云端 `/api/admin/inventory-products` 接口写入正式商品库。保存时只允许按唯一的已有商品名称更新货位；未知名称或重名记录会要求先在商品管理中确认，扫描结果不会创建商品，也不会覆盖图片、规格、分类等商品资料。

## 盒子部署

> Jetson 盒子的系统安装、Python/Node 依赖、模型与标定文件、常见注意事项见仓库根目录
> [`JETSON_DEPLOY.md`](../JETSON_DEPLOY.md)。

### 当前启动命令

管理员账号、密码和云端共享会话密钥保存在 `/etc/grasp-qianduan.env`。不要把真实 `ADMIN_SESSION_SECRET` 写入仓库。当前文件应当允许 `nvidia` 用户通过所属组读取，例如权限为 `640` 且组为 `nvidia`。

启动前先检查端口，避免同时运行两个服务：

```bash
ss -ltnp | grep -E ':3000|:3001'
```

没有旧服务占用时，在盒子上运行：

```bash
cd /home/nvidia/grasp/qianduan

set -a
source /etc/grasp-qianduan.env
set +a

PORT=3000 \
WS_PORT=3001 \
ROBOT_GRASP_MODE=real \
DOBOT_IP=192.168.5.1 \
CHASSIS_HOST=192.168.10.10 \
CHASSIS_PORT=31001 \
CHASSIS_MAP_HTTP_HOST=192.168.10.10 \
CHASSIS_MAP_HTTP_PORT=9001 \
CHASSIS_PLACE_MARKER=fangzhi \
node server.js
```

这条命令前台运行，保持终端打开；按 `Ctrl+C` 停止。正常启动日志应包含：

```text
HTTP 服务已启动: http://0.0.0.0:3000
商品管理页面: http://localhost:3000/admin.html
抓取模式: 真实机械臂 (real)
WebSocket 服务已启动: ws://0.0.0.0:3001
```

若 `source /etc/grasp-qianduan.env` 报 `Permission denied`，修正文件所属组和权限后重新加载：

```bash
sudo chown root:nvidia /etc/grasp-qianduan.env
sudo chmod 640 /etc/grasp-qianduan.env
```

### 启动后验证

在另一个终端执行：

```bash
curl http://127.0.0.1:3000/api/admin/session
curl http://192.168.10.10:9001/maps/ceshichangjing/1/map.yaml
```

第一条无需登录也会返回会话状态 JSON，用于确认盒子 HTTP 服务可达。第二条应返回 `origin` 和 `resolution` 等 YAML 字段，用于确认桥接网络下的底盘地图服务可达。启动时盒子会把该地图写入 `data/chassis_map_cache/`；之后即使底盘地图 HTTP 暂时断开，管理页仍可显示上一次快照。

本机管理页地址为 `http://localhost:3000/admin.html`。管理员密码仅从部署环境读取。云端页面出现“盒子端管理员会话配置不一致”时，需要确认云端和盒子使用完全相同的 `ADMIN_USERNAME` 与 `ADMIN_SESSION_SECRET`，重启两端服务后退出管理页并重新登录。

“抓取/放置前导航”默认关闭，默认放置 marker 为 `fangzhi`；“订单结束自动回充”默认开启。管理页修改任一开关或点位后，配置写入 `data/chassis_settings.json`，订单结束时实时读取，无需重启 Node 或 worker。

`server.js` 在收到第一单时懒启动常驻 `../dipan/shelf_grasp.py` worker。worker 准备完成后保持 DOBOT 和 Aero Hand 连接及使能，后续订单直接向其标准输入写入商品名。

真实模式启动日志会同时打印底盘 TCP 地址、栅格地图 HTTP 地址、导航开关、自动回充开关和放置点。若订单没有调用底盘，先看启动日志中的 `底盘导航: 已关闭` 或 `自动回充: 已关闭`；管理页修改开关后会写入 `data/chassis_settings.json`，无需重启 Node。地图区域显示不可用时，先用上面的 `curl` 验证 `map.yaml`，再确认启动日志中的地图地址与可访问地址一致。

盒子真实模式直接信任订单中的 `id/name`，不要求盒子商品库与云端同步。每个商品按 `qty` 逐件执行。

## FRP 配置

盒子 `/etc/frp/frpc.toml` 示例：

```toml
serverAddr = "<FRP_SERVER_IP>"
serverPort = 7000

auth.method = "token"
auth.token = "<FRP_TOKEN>"

[[proxies]]
name = "edge_ssh_admin"
type = "tcp"
localIP = "127.0.0.1"
localPort = 22
remotePort = 6001

[[proxies]]
name = "admin_backend"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3000
remotePort = 5060

[[proxies]]
name = "admin_order_ws"
type = "tcp"
localIP = "127.0.0.1"
localPort = 3001
remotePort = 5061
```

应用配置：

```bash
sudo systemctl restart frpc
sudo systemctl status frpc --no-pager
```

`5061` 注册成功只代表 FRP 隧道在线。只有盒子 `3001` 上的 WebSocket 服务正在运行，云端 `/ws/` 才能完成握手；只有机械臂 worker 准备完成，页面才应显示机器人就绪。

建议在云安全组或系统防火墙中禁止公网直接访问 `5060`、`5061` 和 FRP 控制端口，只允许云端 Nginx 从本机使用业务转发端口。

## WebSocket 协议

客户端发送抓取订单：

```json
{
  "type": "grab",
  "orderId": "ORD-ABC123",
  "items": [
    {
      "id": 23,
      "name": "咖啡",
      "emoji": "coffee",
      "qty": 2,
      "price": 8,
      "shelf_layer": 2,
      "chassis_marker": "shelf-a"
    }
  ],
  "timestamp": 1784010000000
}
```

约束：`orderId` 必填，`items` 不得为空，商品 `name` 必填，`qty` 必须是正整数。`shelf_layer` 可选，提供时必须是 `1..10` 的整数。`chassis_marker` 可选，对应底盘当前地图中的 marker 名称。商品数量不设固定上限，只按云端商品实时库存校验。

云端订单准备接口会根据商品 `location` 返回规范化的 `shelf_layer`。盒子真实模式优先使用订单中的层号，在抓取前发送 `__layer_N__` 并等待 worker 确认真正到层，随后才发送商品名。盒子仍兼容未携带层号的旧客户端，并在本地商品库存在时回退到本地映射。

收起与心跳：

```json
{"type":"stow","timestamp":1784010000000}
{"type":"ping"}
```

层板过滤只读预览：

```json
{"type":"filter-preview"}
```

真实模式下服务会向常驻 `shelf_grasp.py` worker 写入 `__filter_preview__`。该操作只采集当前帧，不移动机械臂、不闭合灵巧手、不执行抓取。服务返回：

```json
{"type":"filter-preview","status":"started","message":"正在采集层板过滤预览"}
{"type":"filter-preview","status":"completed","imageMime":"image/jpeg","imageBase64":"...","report":{"mode":"dynamic","current_layer":2,"boards":[],"detections":[]}}
```

`imageBase64` 可拼成 `data:image/jpeg;base64,...` 显示；`report` 中 `mode=dynamic` 表示使用当前帧上下层板，`fallback_roi` 表示回退旧 ROI/Z 过滤。订单执行或机械臂收起时会返回 `status: "error"`。

服务端连接响应：

```json
{
  "type": "connected",
  "message": "已连接到盒子后端",
  "robotStatus": "standby"
}
```

`robotStatus` 含义：

| 值 | 含义 |
| --- | --- |
| `standby` | 盒子后端在线，机械臂 worker 尚未准备好 |
| `ready` | worker 已准备好，可立即接收商品名 |
| `busy` | 正在执行订单 |
| `stowing` | 正在收起并关闭 worker |
| `simulated` | 模拟后端在线，不代表真实机械臂就绪 |

订单状态响应：

```json
{"type":"status","status":"started","orderId":"ORD-ABC123","message":"订单已接收"}
{"type":"status","status":"grabbing","orderId":"ORD-ABC123","stage":"picking","progress":50,"message":"正在抓取咖啡"}
{"type":"status","status":"completed","orderId":"ORD-ABC123","progress":100,"message":"所有商品抓取完成"}
{"type":"status","status":"error","orderId":"ORD-ABC123","message":"抓取失败原因"}
```

收起完成也使用 `status: "completed"`，但不带 `orderId`：

```json
{"type":"status","status":"completed","progress":100,"message":"机械臂已收起"}
```

前端必须通过 `orderId` 区分订单完成和收起完成，不能把所有 `completed` 都显示为订单或支付完成。

## Worker 控制

真实模式启动 worker 时设置：

```text
SHELF_GRASP_WORKER=1
```

worker 输出包含“等待商品名”后，服务才将其标记为 ready。抓取时写入：

```text
<商品名>\n
```

收起时写入：

```text
**stow**\n
```

`shelf_grasp.py` 同时兼容旧控制词 `__stow__`。服务等待 worker 真正退出后才返回收起完成。

过滤预览时写入：

```text
__filter_preview__\n
```

## 环境变量

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `HOST` | `0.0.0.0` | HTTP 监听地址 |
| `PORT` | `3000` | HTTP/API 端口 |
| `WS_HOST` | 与 `HOST` 相同 | WebSocket 监听地址 |
| `WS_PORT` | `3001` | WebSocket 端口 |
| `ROBOT_GRASP_MODE` | `sim` | `real`/`robot`/`hardware` 启用真实抓取 |
| `DOBOT_IP` | `192.168.5.1` | DOBOT 地址 |
| `DOBOT_PORT` | `29999` | DOBOT 控制端口，急停按钮直接使用此端口 |
| `CHASSIS_HOST` | `192.168.10.10` | 底盘 TCP 地址 |
| `CHASSIS_PORT` | `31001` | 底盘 TCP 端口 |
| `CHASSIS_MAP_HTTP_PROTOCOL` | `http` | 底盘栅格地图 Web 服务协议 |
| `CHASSIS_MAP_HTTP_HOST` | 与 `CHASSIS_HOST` 相同 | 底盘栅格地图 Web 服务地址；桥接网络下使用底盘桥接后的地址 |
| `CHASSIS_MAP_HTTP_PORT` | `9001` | 底盘栅格地图 Web 服务端口 |
| `CHASSIS_MAP_CACHE_DIR` | 商品 JSON 同目录下的 `chassis_map_cache` | 地图 PNG/YAML 与 `current.json` 快照目录 |
| `CHASSIS_NAVIGATION_ENABLE` | 空/关闭 | 设为 `1` 时在抓取前导航到商品绑定的 marker |
| `CHASSIS_PLACE_MARKER` | `fangzhi` | 首次没有设置文件时使用的默认放置 marker |
| `CHASSIS_CHARGE_MARKER` | `charge_point_1F_35744` | 订单结束后自动回充的充电桩 marker |
| `ORDER_CALLBACK_OUTBOX_FILE` | 订单 JSON 同目录下的 `order_callback_outbox.json` | 盒子完成回调持久化队列 |
| `ORDER_CALLBACK_RETRY_INTERVAL_MS` | `15000` | 完成回调重试基础间隔 |
| `ORDER_CALLBACK_TLS_INSECURE` | `0` | 仅现场自签名证书临时设为 `1`，生产保持 `0` |
| `PAYMENT_TLS_INSECURE` | `0` | 仅现场自签名支付 API 临时设为 `1`，生产保持 `0` |
| `PAYMENT_STATUS_SWEEP_INTERVAL_MS` | `10000` | 云端支付状态巡检间隔 |
| `PAYMENT_STATUS_QUERY_INTERVAL_MS` | `5000` | 同一订单主动查单最短间隔 |
| `ORDER_PAID_START_TIMEOUT_MS` | `1800000` | 已付款但未开始执行的自动退款等待时间 |
| `ROBOT_AUTO_RETURN_CHARGE` | `1` | 管理页没有保存设置时的自动回充默认值，设为 `0` 可默认关闭 |
| `CHASSIS_SETTINGS_FILE` | 商品 JSON 同目录下的 `chassis_settings.json` | 管理页底盘导航、自动回充开关和放置点持久化文件 |
| `CHASSIS_REQUEST_TIMEOUT_MS` | `10000` | 单次底盘 TCP 请求超时 |
| `CHASSIS_NAVIGATION_TIMEOUT_MS` | `180000` | 抓取前导航任务总超时 |
| `CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M` | `0.15` | 判断已在目标点的距离容差（米） |
| `CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD` | `0.25` | 判断已在目标点的角度容差（弧度） |
| `CHASSIS_NAVIGATION_POLL_MS` | `1000` | 导航状态轮询间隔 |
| `GRASP_SCRIPT` | `../dipan/shelf_grasp.py` | 抓取脚本路径 |
| `PRODUCTS_DB_FILE` | `data/products.json` | 商品 JSON 路径 |
| `OPERATIONS_DB_FILE` | 商品 JSON 同目录下的 `operations.json` | 设备与门店台账 JSON 路径 |
| `DEVICE_SN` | 与 `PAYMENT_DEVICE_SN` 相同 | 本盒子设备 SN，用于订单归属 |
| `STORE_ID` | 空 | 本盒子所属门店 ID，用于订单归属 |
| `STORE_NAME` | 空 | 本盒子所属门店名称，用于订单归属 |
| `MERCHANT_SESSION_SECRET` | 同 `ADMIN_SESSION_SECRET` | 商家端会话签名密钥 |
| `MERCHANT_SESSION_TTL_MS` | `28800000` | 商家端会话有效期（8 小时） |
| `ROBOT_MAX_ITEM_QTY` | 已移除 | 商品数量只受商品库存限制 |
| `ROBOT_MAX_ORDER_UNITS` | 已移除 | 整单数量只受各商品库存之和限制 |
| `ROBOT_WORKER_READY_TIMEOUT_MS` | `90000` | worker 启动超时 |
| `ROBOT_GRAB_TIMEOUT_MS` | `120000` | 单件抓取超时 |
| `ROBOT_STOW_TIMEOUT_MS` | `120000` | 收起超时 |
| `ROBOT_FILTER_PREVIEW_TIMEOUT_MS` | `30000` | 层板过滤预览超时 |
| `ROBOT_BROADCAST_LOGS` | 空 | 设为 `1` 时向前端广播内部日志 |
| `VOICE_LLM_ENABLED` | `1` | 语音大模型校正开关，设为 `0` 关闭（前端自动回退原关键词匹配） |
| `VOICE_LLM_API_KEY` | 空 | 大模型 API Key；未设置时不调用大模型，语音走原 Web Speech 关键词逻辑 |
| `VOICE_LLM_BASE_URL` | `https://api.deepseek.com` | OpenAI 兼容接口地址（默认 DeepSeek） |
| `VOICE_LLM_MODEL` | `deepseek-flash` | 大模型 Model-Id |
| `VOICE_LLM_TIMEOUT_MS` | `30000` | 单次校正调用超时 |
| `VOICE_LLM_RETRIES` | `2` | 单次校正调用失败（超时/网络/5xx）自动重试次数 |
| `VOICE_LLM_REASONING_EFFORT` | `none` | 推理力度；DeepSeek 设为 `none` 关闭思考降低延迟，设为空字符串则不下发 |
| `VOICE_LLM_CATALOG_URL` | 空 | 语音校正商品库来源；设为云端商品接口（如 `https://<站点>/api/products`）时与点单商品一致，未配置则用本地 `products.json` |
| `VOICE_LLM_CATALOG_CACHE_MS` | `300000` | 云端商品库缓存时间 |
| `VOICE_LLM_ALIASES` | 空 | 追加/覆盖误识别对照表，JSON 对象 `{"用户说词":"商品库词"}` |

## 语音交互（检索 + 点单）

前端顶部有 **🎤 检索** 和 **🛒 点单** 两个按钮，点哪个就直接开始哪种语音输入（再点同一个停止）：

- **🎤 检索**：说商品或品类（"我要一瓶可乐"），大模型把谐音/口语校正为商品库标准词，
  前端按关键词（品牌/系列基础名）筛选商品列表。未命中时回退原 `extractVoiceKeyword`
  + 关键词/分类匹配逻辑。
- **🛒 点单**：说"来两瓶可乐""去掉雪碧""清空购物车""结算""我买了什么"等，大模型返回
  `action`（add/remove/set/clear/checkout/query/none）和 `items`（每项含商品名/关键词/数量），
  一句话可说多个商品（"我要一瓶可乐，一瓶外星人"），前端逐项操作购物车并把结果显示在
  `voiceHintText`。同一商品有多个品牌/口味时（如"可乐"、"外星人"），按关键词匹配到的
  第一种加入，用户可在购物车里增减或改选。

`server.js` 的 `POST /api/voice/correct`（`mode=search|order`）用商品库作提示词，返回
结构化 JSON：`matched_product` / `category` / `keyword` / `corrected_text` / `note`
（点单模式另有 `action` / `items` / `quantity`）。

- 商品库来源优先用 `VOICE_LLM_CATALOG_URL`（建议指向云端商品接口，与点单/订单一致），
  拉取失败或未配置时回退本地 `products.json`。
- 不配置 `VOICE_LLM_API_KEY` 或大模型调用失败时，`used=false`，检索模式自动回退原关键词
  匹配，点单模式提示"识别失败，请再说一次"。
- `VOICE_LLM_API_KEY` 应放在 `/etc/grasp-qianduan.env` 或启动命令环境变量中，不要把
  Token 写进仓库。

## 数据初始化与测试

### MySQL 迁移（grasp_catalog，服务仍读 JSON）

当前运行服务仍读 `products.json` / `operations.json` / 订单 JSON。已可用脚本把商品、门店、
设备、商家、订单及**运行态**数据导出到独立数据库 `grasp_catalog`：

```bash
cd /root/system/grasp/qianduan
export PRODUCTS_DB_FILE=/var/lib/grasp-qianduan/products.json
export ORDERS_DB_FILE=/var/lib/grasp-qianduan/orders.json
export OPERATIONS_DB_FILE=/var/lib/grasp-qianduan/operations.json
export INVENTORY_SCAN_FILE=/var/lib/grasp-qianduan/inventory_scan.json
export CHASSIS_SETTINGS_FILE=/var/lib/grasp-qianduan/chassis_settings.json
export ORDER_CALLBACK_OUTBOX_FILE=/var/lib/grasp-qianduan/order_callback_outbox.json
export MYSQL_DATABASE=grasp_catalog

# 用 node 直接跑，不要用 npm run（npm 横幅会写进 stdout 污染 SQL）
node tools/migrate-json-to-mysql.js > /tmp/grasp_catalog.sql
head -n 1 /tmp/grasp_catalog.sql          # 应为 -- Generated from JSON stores.
tail -n 1 /tmp/grasp_catalog.sql          # 结尾的汇总计数

mysql -h <mysql-host> -u <mysql-user> -p grasp_catalog < /tmp/grasp_catalog.sql
```

导出的表（15 张）：主数据 `products`、`stores`、`devices`、`merchants`、`merchant_stores`、
`orders`、`order_items`、`order_refunds`；运行态 `device_events`、`reserved_orders`、
`completed_orders`、`inventory_scan_shelves`、`inventory_scan_tasks`、`chassis_settings`、
`order_callback_outbox`。重复执行会清空并重建这些表（幂等）；订单明细/退款历史既保留原始 JSON，
也拆成可查询明细表。二维码图片、访问令牌等临时敏感字段不复制。

**踩坑**：

- `npm run export-mysql` 的脚本横幅会被 `>` 一起写进 SQL，导入报
  `You have an error in your SQL syntax ... near '> qianduan@1.0.0 export-mysql'`。
  请用 `node tools/migrate-json-to-mysql.js`。
- `stores` 有 `UNIQUE(code)`，两个门店 `code` 为空串会报
  `Duplicate entry '' for key 'stores.uk_stores_code'`。脚本已把空 `code`/空 `sn` 写成 `NULL`
  （MySQL 唯一索引允许多个 NULL）；用旧脚本生成的 SQL 需先执行
  `ALTER TABLE stores DROP INDEX uk_stores_code;`。

导入后核对（示例为线上当前状态）：

```bash
mysql -h <mysql-host> -u <mysql-user> -p -D grasp_catalog -e 'SHOW TABLES;'
# 15 张表；merchants=2, devices=2, products=17, orders=361,
# device_events=4, reserved_orders=324, completed_orders=160
```

连接配置用环境变量（不写进仓库）：`MYSQL_HOST`、`MYSQL_PORT`、`MYSQL_USER`、`MYSQL_PASSWORD`、
`MYSQL_DATABASE=grasp_catalog`。支付系统用的是另一个库（`ly_order_system`），两者独立。

> 迁移只用于建库/核对；**Node 服务目前仍读 JSON**。真正切换读取源是下一步，计划用
> `DB_CLIENT=file|mysql`（默认 `file`，切换后内存缓存 + 异步落库 + 事务，可一键回滚）。

安装依赖并初始化商品库：

```bash
npm install
npm run init-db
```

`npm start` 会先运行 `init_db.js`，再启动服务。已经有生产商品数据时，不要在未确认覆盖行为前重复初始化；生产部署可直接执行 `node server.js`。

运行测试：

```bash
npm test
```

WebSocket 集成测试使用假的抓取 worker，不会驱动真实机械臂。

## 排障

检查盒子服务和监听：

```bash
ps -ef | grep '[n]ode.*server.js'
sudo ss -lntp | grep -E ':(3000|3001)\b'
sudo systemctl status frpc --no-pager
```

检查云端 FRP 与 Nginx：

```bash
sudo ss -lntp | grep -E ':(5061|8083)\b'
sudo journalctl -u frps.service -n 100 --no-pager
sudo nginx -t
```

常见现象：

- 商品为空：检查 `/api/products` 是否返回 JSON，并排除浏览器旧缓存。
- `/ws/` 返回 `502`：FRP `5061` 未注册，或盒子 `3001` 没有服务。
- 显示“后端在线，机械臂待启动”：WebSocket 正常，但 worker 尚未准备完成。
- 收起后出现金额为零的订单提示：前端使用了旧缓存；新版按 `orderId` 区分收起与订单完成。
- FRP 在线但无法运动：隧道在线不等于 Node 服务、worker、DOBOT、相机或 Aero Hand 已就绪。
