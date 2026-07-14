# 点单前端与订单服务

本目录包含静态点单页面、商品 REST API 和盒子端订单 WebSocket 服务。生产环境采用“云端页面和商品库、盒子执行机械臂”的部署方式。

## 架构

```text
商品展示：浏览器 -> 云端 Nginx /api/ -> 云端 Node :3100 -> products.json

订单下发：浏览器 -> wss://<站点>/ws/ -> 云端 Nginx
         -> 云端 FRP :5061 -> 盒子 :3001 -> server.js
         -> shelf_grasp.py -> DOBOT / Aero Hand
```

云端商品服务和盒子订单服务都运行 `server.js`，但职责和环境变量不同：

| 位置 | HTTP | WebSocket | 模式 | 用途 |
| --- | ---: | ---: | --- | --- |
| 云端 | `127.0.0.1:3100` | `127.0.0.1:3101` | `sim` | 商品 REST API |
| 盒子 | `3000` | `3001` | `real` | 接收订单并控制机械臂 |
| FRP 服务端 | `5060` | `5061` | TCP | 分别转到盒子 `3000`、`3001` |

云端网页只通过 `/api/` 访问商品服务，通过 `/ws/` 访问盒子。浏览器不应直接连接或暴露 Node 内部端口。

## 文件

- `index.html`、`style.css`、`script.js`：点单页面。
- `admin.html`：商品管理页面。
- `server.js`：HTTP/API 与订单 WebSocket 服务。
- `start.js`：初始化商品数据后启动 `server.js`。
- `init_db.js`：从 Excel 初始化 `data/products.json`。
- `data/products.json`：默认 JSON 商品库。
- `test/ws_protocol.test.js`：不连接真实机械臂的 WebSocket 集成测试。

## 云端部署

云端 Nginx 直接托管静态文件，并使用以下两个反向代理：

```nginx
location ^~ /api/ {
    proxy_pass http://127.0.0.1:3100;
}

location ^~ /ws/ {
    proxy_pass http://127.0.0.1:5061;
    proxy_http_version 1.1;
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
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
node server.js
```

生产环境应使用 systemd 或其他进程管理器保持服务运行。`PRODUCTS_DB_FILE` 必须对运行服务的用户可写，否则商品新增、修改和删除会失败。

## 盒子部署

盒子运行真实模式：

```bash
cd /home/nvidia/grasp/qianduan

PORT=3000 \
WS_PORT=3001 \
ROBOT_GRASP_MODE=real \
DOBOT_IP=192.168.5.1 \
node server.js
```

`server.js` 在收到第一单时懒启动常驻 `../dipan/shelf_grasp.py` worker。worker 准备完成后保持 DOBOT 和 Aero Hand 连接及使能，后续订单直接向其标准输入写入商品名。

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
      "price": 8
    }
  ],
  "timestamp": 1784010000000
}
```

约束：`orderId` 必填，`items` 不得为空，商品 `name` 必填，`qty` 必须是整数。默认单项最多 `20` 件，整单最多 `50` 件。

收起与心跳：

```json
{"type":"stow","timestamp":1784010000000}
{"type":"ping"}
```

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

## 环境变量

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `HOST` | `0.0.0.0` | HTTP 监听地址 |
| `PORT` | `3000` | HTTP/API 端口 |
| `WS_HOST` | 与 `HOST` 相同 | WebSocket 监听地址 |
| `WS_PORT` | `3001` | WebSocket 端口 |
| `ROBOT_GRASP_MODE` | `sim` | `real`/`robot`/`hardware` 启用真实抓取 |
| `DOBOT_IP` | `192.168.5.1` | DOBOT 地址 |
| `GRASP_SCRIPT` | `../dipan/shelf_grasp.py` | 抓取脚本路径 |
| `PRODUCTS_DB_FILE` | `data/products.json` | 商品 JSON 路径 |
| `ROBOT_MAX_ITEM_QTY` | `20` | 单项最大数量 |
| `ROBOT_MAX_ORDER_UNITS` | `50` | 整单最大数量 |
| `ROBOT_WORKER_READY_TIMEOUT_MS` | `90000` | worker 启动超时 |
| `ROBOT_GRAB_TIMEOUT_MS` | `120000` | 单件抓取超时 |
| `ROBOT_STOW_TIMEOUT_MS` | `120000` | 收起超时 |
| `ROBOT_BROADCAST_LOGS` | 空 | 设为 `1` 时向前端广播内部日志 |

## 数据初始化与测试

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
