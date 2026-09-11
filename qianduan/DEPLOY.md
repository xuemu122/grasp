# 服务器部署说明（8083 站点）

本文说明如何把本仓库（点单页、管理页、Node 服务）部署到服务器并挂到 Nginx 上。
当前线上站点走 **8083**，点单页（商品页）和管理页都在这个站点下，本次新增的
“设备管理 / 运营管理”页面属于同一个 `admin.html`，新的接口都在 `/api/` 下，
**不需要为新页面新增 Nginx 端口或 location**，只要更新文件和重启 Node 即可。

## 1. 部署拓扑

8083 站点的典型结构：

```text
浏览器
  ├── /index.html、/admin.html、/style.css、/script.js   -> 静态页面（/var/www/qianduan 或 Node）
  ├── /api/...                                            -> Nginx 反向代理 -> Node 服务
  └── /ws/                                                -> Nginx 反向代理 -> 盒子 WebSocket
```

- `admin.html` 里已经包含“设备管理”和“运营管理”两个新页面。
- 设备/门店接口位于 `/api/admin/stores`、`/api/admin/devices`、
  `/api/admin/operations/summary`，都被现有的 `location /api/` 一并代理，无需单独配置。
- 只有运行 `server.js` 的 Node 进程负责这些接口，所以 Node 必须换成新版本代码。

> 注意：如果 Nginx 用 `location = /admin.html` 直接代理到 Node（仓库自带 `nginx.conf` 就是
> 这种写法），那么线上真正的 `admin.html` 是 **Node 程序目录下的那一份**，不是
> `/var/www/qianduan/admin.html`。两份都更新最稳妥。

## 2. 需要更新的文件

| 文件 | 说明 |
| --- | --- |
| `server.js` | 新增门店/设备/出入库接口、订单归属筛选 |
| `admin.html` | 新增“设备管理”“运营管理”页面 |
| `script.js` | 点单页支持 `?store=&device=` 归属参数 |
| `data/operations.json` | 设备与门店台账（首次可拷示例，之后会被接口改写） |
| `nginx.conf` | 仅作参考，通常无需修改 |

本次没有新增 npm 依赖，**不需要** 重新 `npm install`。

## 3. Nginx（8083）配置要点

找到线上监听 8083、`server_name` 为正式站点的那个 `server` 块，确认包含以下能力即可：

```nginx
server {
    listen 8083 ssl;                 # 正式站点端口
    server_name <你的域名>;

    root /var/www/qianduan;          # 点单页静态文件目录
    index index.html;

    # HTML/JS/CSS 禁用缓存，保证新管理页立即生效
    location ~* \.(html|css|js)$ {
        expires -1;
        add_header Cache-Control "no-store, max-age=0" always;
    }

    # 管理页：代理到 Node（这样新页面随 Node 一起更新）
    location = /admin.html {
        proxy_pass http://127.0.0.1:3000/admin.html;   # 云端服务是 3100 就改成 3100
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        add_header Cache-Control "no-store, max-age=0" always;
    }

    # 所有 /api/ 接口（含新增的设备、门店、出入库接口）
    location /api/ {
        proxy_pass http://127.0.0.1:3000/api/;          # 云端服务是 3100 就改成 3100
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    # 其他页面回退
    location / {
        try_files $uri $uri/ @node;
    }
    location @node {
        proxy_pass http://127.0.0.1:3000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }
}
```

要点：

- 新增的 `/api/admin/devices`、`/api/admin/stores`、`/api/admin/operations/summary`
  全部落在 `location /api/` 里，**不用为新页面加 location**。
- `location = /admin.html` 的 `proxy_pass` 端口要和实际运行 `server.js` 的端口一致
  （盒子本地是 `3000`，云端商品服务是 `3100`）。
- 修改后先 `nginx -t` 通过再 reload。

## 4. 部署步骤

在准备发布的机器（本地开发机）打包，然后上传到服务器：

```bash
# 1) 本地：确认改动并打包（在仓库 qianduan 目录）
git status
tar -czf qianduan-deploy.tar.gz \
  server.js admin.html script.js style.css index.html \
  package.json data/operations.json

# 2) 上传到服务器（示例）
scp qianduan-deploy.tar.gz <user>@<server>:/tmp/
```

在服务器上：

```bash
# 3) 备份现有运行目录
cd /opt/grasp/qianduan        # 换成实际运行 server.js 的目录
cp -a server.js server.js.bak.$(date +%F)
cp -a admin.html admin.html.bak.$(date +%F)

# 4) 解包覆盖（Node 运行目录，管理页和接口都在这里生效）
tar -xzf /tmp/qianduan-deploy.tar.gz -C /opt/grasp/qianduan

# 5) 同步一份到 Nginx 静态目录（若 /admin.html 走静态时有需要）
sudo mkdir -p /var/www/qianduan
sudo cp admin.html script.js style.css index.html /var/www/qianduan/
sudo chown -R nginx:nginx /var/www/qianduan    # 按现场用户调整

# 6) 确认台账文件可写（新增接口会写入）
ls -l data/operations.json
chmod 640 data/operations.json                 # 属主为运行 Node 的用户
```

启动/重启 Node 服务（按现场是 systemd 还是前台进程选择一种）：

```bash
# 方式A：systemd
sudo systemctl restart grasp-qianduan
sudo systemctl status grasp-qianduan --no-pager

# 方式B：前台/自定义脚本
cd /opt/grasp/qianduan
set -a && source /etc/grasp-qianduan.env && set +a
PORT=3000 WS_PORT=3001 node server.js
```

Nginx 校验并重载：

```bash
sudo nginx -t
sudo systemctl reload nginx
```

## 5. 环境变量

新功能涉及以下变量，写进 `/etc/grasp-qianduan.env` 或启动命令：

```text
# 设备 / 门店台账文件（默认与 products.json 同目录的 operations.json）
OPERATIONS_DB_FILE=/var/lib/grasp-qianduan/operations.json

# 本机设备与门店归属（点单页不带参数时用，或每台盒子各配一份）
DEVICE_SN=GRASP-ROBOT-01
STORE_ID=STORE-001
STORE_NAME=门店A
```

第二台设备改为 `GRASP-ROBOT-02` / `STORE-002` / `门店B`。
`OPERATIONS_DB_FILE` 所在目录必须对运行 Node 的用户可写。

## 6. 部署后验证

```bash
# 页面能打开
curl -sI https://<你的域名>:8083/admin.html | head -n 1

# 新接口已生效：未登录应返回 401（说明路由存在），不是 404
curl -s -o /dev/null -w '%{http_code}\n' https://<你的域名>:8083/api/admin/devices
curl -s -o /dev/null -w '%{http_code}\n' https://<你的域名>:8083/api/admin/stores
```

浏览器里打开 `https://<你的域名>:8083/admin.html`，登录后应看到左侧多出
**设备管理**、**运营管理** 两项：

1. 运营管理 → 新增门店（门店A、门店B）。
2. 设备管理 → 新增两台设备，分别出库到不同门店。
3. 运营管理 → 设备分布与出入库 能看到两台设备各自归属的门店。

## 7. 双设备 / 双门店说明

- 两个门店各用一台盒子，盒子端 `server.js`、机械臂仍是独立运行，盒子内部端口都是 `3000/3001`。
- 若两台盒子都要接同一个域名，FRP 远程端口要错开，例如盒子A `5060/5061`、盒子B `5062/5063`，
  Nginx 的 `/ws/` 按门店子域名分别指向 `5061`、`5063`。
- 设备/门店台账建议只放在 **一个** 中心 Node 上（例如云端 3100），管理页的
  `/api/` 指向这个中心 Node；两个盒子通过 `DEVICE_SN/STORE_ID/STORE_NAME`
  给订单打上各自的门店归属。
- 点单页可带归属参数：`https://<域名>:8083/?store=STORE-001&device=DEV-001`。

## 8. 回滚

```bash
cd /opt/grasp/qianduan
cp -a server.js.bak.<日期> server.js
cp -a admin.html.bak.<日期> admin.html
sudo systemctl restart grasp-qianduan
sudo systemctl reload nginx
```

## 9. 相关文档

- Jetson 盒子的系统、依赖、标定文件和注意事项见仓库根目录
  [`JETSON_DEPLOY.md`](../JETSON_DEPLOY.md)。
- 现场抓取参数、标定和测试命令见 [`README.md`](README.md) 与
  [`../dipan/README.md`](../dipan/README.md)。
