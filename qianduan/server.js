/**
 * 机器人点餐后端服务
 * 功能：
 *   1. 提供静态页面
 *   2. WebSocket 实时通信，接收前端下单命令
 *   3. REST API - 商品管理（增删改查）
 *   4. 模拟机器人抓取逻辑
 */
const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const { WebSocket, WebSocketServer } = require('ws');

const PORT = Number(process.env.PORT || 3000);
const WS_PORT = Number(process.env.WS_PORT || 3001);
const HOST = process.env.HOST || '0.0.0.0';
const WS_HOST = process.env.WS_HOST || HOST;
const ROBOT_GRASP_MODE = (process.env.ROBOT_GRASP_MODE || 'sim').toLowerCase();
const ROBOT_GRASP_REAL = ['real', 'robot', 'hardware'].includes(ROBOT_GRASP_MODE);
const GRASP_SCRIPT = process.env.GRASP_SCRIPT || path.resolve(__dirname, '../dipan/shelf_grasp.py');
const GRASP_CWD = path.dirname(GRASP_SCRIPT);
const DOBOT_IP = process.env.DOBOT_IP || '192.168.5.1';
const PRODUCTS_DB_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
const ROBOT_WORKER_READY_TIMEOUT_MS = Number(process.env.ROBOT_WORKER_READY_TIMEOUT_MS || 90000);
const ROBOT_GRAB_TIMEOUT_MS = Number(process.env.ROBOT_GRAB_TIMEOUT_MS || 120000);
const ROBOT_STOW_TIMEOUT_MS = Number(process.env.ROBOT_STOW_TIMEOUT_MS || 120000);
const ROBOT_STOW_COMMAND = process.env.ROBOT_STOW_COMMAND || '**stow**';
const ROBOT_MAX_ITEM_QTY = Number(process.env.ROBOT_MAX_ITEM_QTY || 20);
const ROBOT_MAX_ORDER_UNITS = Number(process.env.ROBOT_MAX_ORDER_UNITS || 50);
const ROBOT_SIM_DELAY_SCALE = Math.max(0, Number(process.env.ROBOT_SIM_DELAY_SCALE || 1));
const ROBOT_BROADCAST_LOGS = /^(1|true|yes|on)$/i.test(process.env.ROBOT_BROADCAST_LOGS || '');
let activeOrderId = null;
let robotWorker = null;
let robotWorkerReady = false;
let robotWorkerStarting = null;
let robotStowPromise = null;

// 机器人初始位置
const ROBOT_HOME = { x: 0, y: 0, z: 100 };

// ========== 日志 ==========
function log(msg) {
    const time = new Date().toLocaleTimeString('zh-CN');
    console.log(`[${time}] ${msg}`);
}

function createLineHandler(onLine) {
    let pending = '';
    return (chunk) => {
        pending += chunk.toString();
        const lines = pending.split(/\r?\n/);
        pending = lines.pop();
        for (const line of lines) onLine(line);
    };
}

// ========== MIME 类型 ==========
const MIME = {
    '.html': 'text/html; charset=utf-8',
    '.css':  'text/css; charset=utf-8',
    '.js':   'application/javascript; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.png':  'image/png',
    '.jpg':  'image/jpeg',
    '.svg':  'image/svg+xml',
    '.ico':  'image/x-icon',
};

// ========== 辅助：解析请求 body ==========
function parseBody(req) {
    return new Promise((resolve) => {
        let body = '';
        req.on('data', chunk => body += chunk);
        req.on('end', () => {
            try { resolve(JSON.parse(body)); }
            catch (e) { resolve(null); }
        });
    });
}

function sendJSON(res, data, status = 200) {
    res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Access-Control-Allow-Origin': '*' });
    res.end(JSON.stringify(data));
}

function normalizeProduct(raw, fallbackId = 1) {
    const id = Number(raw.id || fallbackId);
    return {
        id,
        name: raw.name || '',
        spec: raw.spec || '',
        packaging: raw.packaging || '',
        category: raw.category || '',
        manufacturer: raw.manufacturer || '',
        photo_front: raw.photo_front || '',
        photo_back: raw.photo_back || '',
        location: raw.location || '',
        shelf_x: Number(raw.shelf_x || 0),
        shelf_y: Number(raw.shelf_y || 0),
        shelf_z: Number(raw.shelf_z || 50),
        emoji: raw.emoji || '📦',
        price: Number(raw.price || 0),
        stock: Number(raw.stock ?? 100),
        status: Number(raw.status ?? 1),
    };
}

function readProducts() {
    if (!fs.existsSync(PRODUCTS_DB_FILE)) return [];
    try {
        const data = JSON.parse(fs.readFileSync(PRODUCTS_DB_FILE, 'utf8'));
        return Array.isArray(data.products) ? data.products.map((p, idx) => normalizeProduct(p, idx + 1)) : [];
    } catch (e) {
        log(`⚠️ 商品数据读取失败: ${e.message}`);
        return [];
    }
}

function writeProducts(products) {
    fs.mkdirSync(path.dirname(PRODUCTS_DB_FILE), { recursive: true });
    fs.writeFileSync(PRODUCTS_DB_FILE, JSON.stringify({ products }, null, 2), 'utf8');
}

function nextProductId(products) {
    return products.reduce((max, item) => Math.max(max, Number(item.id || 0)), 0) + 1;
}

function filterProducts(products, { search = '', category = '' } = {}) {
    const keyword = String(search || '').trim().toLowerCase();
    return products
        .filter((item) => {
            if (category && item.category !== category) return false;
            if (!keyword) return true;
            return [item.name, item.spec, item.manufacturer, item.location]
                .some((value) => String(value || '').toLowerCase().includes(keyword));
        })
        .sort((a, b) => Number(b.id) - Number(a.id));
}

function normalizeOrderItems(items) {
    const normalized = items.map((item, index) => {
        if (!item || typeof item !== 'object' || Array.isArray(item)) {
            throw new Error(`第 ${index + 1} 个商品格式无效`);
        }
        const name = String(item.name || '').trim();
        if (!name) throw new Error(`第 ${index + 1} 个商品缺少 name`);
        if (/\r|\n/.test(name)) throw new Error(`商品名称不能包含换行: ${name}`);
        if (['**stow**', '__stow__', '__shutdown__', '__quit__', '__exit__'].includes(name)) {
            throw new Error(`商品名称使用了保留控制词: ${name}`);
        }

        const qty = Number(item.qty ?? 1);
        if (!Number.isInteger(qty) || qty < 1 || qty > ROBOT_MAX_ITEM_QTY) {
            throw new Error(`商品 ${name} 的 qty 必须是 1..${ROBOT_MAX_ITEM_QTY} 的整数`);
        }
        return { ...item, name, qty };
    });

    const totalUnits = normalized.reduce((sum, item) => sum + item.qty, 0);
    if (totalUnits > ROBOT_MAX_ORDER_UNITS) {
        throw new Error(`订单商品总数不能超过 ${ROBOT_MAX_ORDER_UNITS}`);
    }
    return { items: normalized, totalUnits };
}

// ========== HTTP 服务 ==========
const server = http.createServer(async (req, res) => {
    // CORS 预检
    if (req.method === 'OPTIONS') {
        res.writeHead(204, { 'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Methods': 'GET,POST,PUT,DELETE,OPTIONS', 'Access-Control-Allow-Headers': 'Content-Type' });
        res.end();
        return;
    }

    const url = new URL(req.url, `http://localhost:${PORT}`);
    const pathname = url.pathname;

    // ========== REST API 路由 ==========
    // GET /api/products - 获取所有商品
    if (pathname === '/api/products' && req.method === 'GET') {
        try {
            const search = url.searchParams.get('search') || '';
            const category = url.searchParams.get('category') || '';
            const rows = filterProducts(readProducts(), { search, category });
            sendJSON(res, { code: 0, data: rows, total: rows.length });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // GET /api/products/:id - 获取单个商品
    const singleMatch = pathname.match(/^\/api\/products\/(\d+)$/);
    if (singleMatch && req.method === 'GET') {
        try {
            const product = readProducts().find((item) => String(item.id) === singleMatch[1]);
            if (!product) { sendJSON(res, { code: 1, message: '商品不存在' }, 404); return; }
            sendJSON(res, { code: 0, data: product });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // POST /api/products - 新增商品
    if (pathname === '/api/products' && req.method === 'POST') {
        const body = await parseBody(req);
        if (!body || !body.name) { sendJSON(res, { code: 1, message: '商品名称不能为空' }, 400); return; }
        try {
            const products = readProducts();
            const product = normalizeProduct({ ...body, id: nextProductId(products) });
            products.push(product);
            writeProducts(products);
            log(`➕ 新增商品: ${body.name} (id=${product.id})`);
            sendJSON(res, { code: 0, message: '添加成功', id: product.id });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // PUT /api/products/:id - 更新商品
    const updateMatch = pathname.match(/^\/api\/products\/(\d+)$/);
    if (updateMatch && req.method === 'PUT') {
        const body = await parseBody(req);
        if (!body) { sendJSON(res, { code: 1, message: '无效数据' }, 400); return; }
        try {
            const products = readProducts();
            const index = products.findIndex((item) => String(item.id) === updateMatch[1]);
            if (index < 0) { sendJSON(res, { code: 1, message: '商品不存在' }, 404); return; }
            const allowed = ['name','spec','packaging','category','manufacturer','photo_front','photo_back','location','shelf_x','shelf_y','shelf_z','emoji','price','stock','status'];
            const patch = {};
            for (const key of allowed) {
                if (body[key] !== undefined) {
                    patch[key] = body[key];
                }
            }
            if (Object.keys(patch).length === 0) { sendJSON(res, { code: 1, message: '没有要更新的字段' }, 400); return; }
            products[index] = normalizeProduct({ ...products[index], ...patch, id: products[index].id });
            writeProducts(products);
            log(`✏️ 更新商品 id=${updateMatch[1]}`);
            sendJSON(res, { code: 0, message: '更新成功' });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // DELETE /api/products/:id - 删除商品
    const delMatch = pathname.match(/^\/api\/products\/(\d+)$/);
    if (delMatch && req.method === 'DELETE') {
        try {
            const before = readProducts();
            const products = before.filter((item) => String(item.id) !== delMatch[1]);
            writeProducts(products);
            log(`🗑 删除商品 id=${delMatch[1]}`);
            sendJSON(res, { code: 0, message: '删除成功' });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // GET /api/categories - 获取所有品类
    if (pathname === '/api/categories' && req.method === 'GET') {
        try {
            const categories = [...new Set(readProducts().map((item) => item.category).filter(Boolean))].sort();
            sendJSON(res, { code: 0, data: categories });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // ========== 静态文件服务 ==========
    let filePath = pathname === '/' ? '/index.html' : pathname;
    filePath = path.join(__dirname, filePath);

    const ext = path.extname(filePath);
    const contentType = MIME[ext] || 'application/octet-stream';

    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(404, { 'Content-Type': 'text/plain' });
            res.end('404 Not Found');
            return;
        }
        res.writeHead(200, { 'Content-Type': contentType });
        res.end(data);
    });
});

server.listen(PORT, HOST, () => {
    log(`HTTP 服务已启动: http://${HOST}:${PORT}`);
    log(`商品管理页面: http://localhost:${PORT}/admin.html`);
    log(`抓取模式: ${ROBOT_GRASP_REAL ? '真实机械臂' : '模拟'} (${ROBOT_GRASP_MODE})`);
});

// ========== WebSocket 服务 ==========
const wss = new WebSocketServer({ port: WS_PORT, host: WS_HOST, maxPayload: 64 * 1024 });

log(`WebSocket 服务已启动: ws://localhost:${WS_PORT}`);

function broadcast(message) {
    const data = JSON.stringify(message);
    for (const client of wss.clients) {
        if (client.readyState === WebSocket.OPEN) client.send(data);
    }
}

function sendMessage(ws, message) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        log(`WebSocket 已断开，无法回传 ${message.type || '消息'}`);
        return false;
    }
    try {
        ws.send(JSON.stringify(message));
        return true;
    } catch (error) {
        log(`WebSocket 回传失败: ${error.message}`);
        return false;
    }
}

function currentRobotStatus() {
    if (robotStowPromise) return 'stowing';
    if (activeOrderId) return 'busy';
    if (!ROBOT_GRASP_REAL) return 'simulated';
    return robotWorkerReady ? 'ready' : 'standby';
}

wss.on('connection', (ws) => {
    log('✅ 前端已连接');

    ws.on('message', async (raw) => {
        let msg;
        try { msg = JSON.parse(raw.toString()); }
        catch (e) { sendMessage(ws, { type: 'error', message: '无效的 JSON 格式' }); return; }
        if (!msg || typeof msg !== 'object' || Array.isArray(msg)) {
            sendMessage(ws, { type: 'error', message: 'JSON 指令必须是对象' });
            return;
        }

        log(`📩 收到命令: ${JSON.stringify(msg)}`);

        if (msg.type === 'grab') {
            handleGrabCommand(ws, msg);
        } else if (msg.type === 'stow') {
            handleStowCommand(ws);
        } else if (msg.type === 'ping') {
            sendMessage(ws, { type: 'pong', time: Date.now() });
        } else {
            sendMessage(ws, { type: 'error', message: `未知命令类型: ${msg.type}` });
        }
    });

    ws.on('close', () => { log('❌ 前端已断开'); });
    ws.on('error', (err) => { log(`⚠️ WebSocket 错误: ${err.message}`); });

    sendMessage(ws, {
        type: 'connected',
        message: '已连接到盒子后端',
        robotStatus: currentRobotStatus()
    });
});

function ensureRobotWorker() {
    if (!ROBOT_GRASP_REAL) return Promise.resolve(null);
    if (robotWorker && !robotWorker.killed && robotWorkerReady) return Promise.resolve(robotWorker);
    if (robotWorkerStarting) return robotWorkerStarting;

    robotWorkerStarting = new Promise((resolve, reject) => {
        log('🤖 启动常驻抓取 worker...');
        robotWorkerReady = false;
        let settled = false;
        const child = spawn('python3', [GRASP_SCRIPT], {
            cwd: GRASP_CWD,
            env: {
                ...process.env,
                PYTHONUNBUFFERED: '1',
                DOBOT_IP,
                SHELF_GRASP_WORKER: '1'
            }
        });
        robotWorker = child;
        const readyTimer = setTimeout(() => {
            if (settled) return;
            settled = true;
            robotWorkerStarting = null;
            reject(new Error(`抓取 worker 启动超时 (${ROBOT_WORKER_READY_TIMEOUT_MS}ms)`));
        }, ROBOT_WORKER_READY_TIMEOUT_MS);

        const handleOutputLine = (line) => {
            const trimmed = line.trim();
            if (!trimmed) return;
            log(`  [worker] ${trimmed}`);
            if (!settled && trimmed.includes('等待商品名')) {
                settled = true;
                clearTimeout(readyTimer);
                robotWorkerReady = true;
                resolve(child);
            }
            if (ROBOT_BROADCAST_LOGS) {
                broadcast({ type: 'robot_log', line: trimmed, timestamp: Date.now() });
            }
        };

        child.stdout.on('data', createLineHandler(handleOutputLine));
        child.stderr.on('data', createLineHandler(handleOutputLine));
        child.on('error', (e) => {
            clearTimeout(readyTimer);
            robotWorker = null;
            robotWorkerReady = false;
            robotWorkerStarting = null;
            if (!settled) {
                settled = true;
                reject(e);
            }
        });
        child.on('close', (code) => {
            clearTimeout(readyTimer);
            log(`🤖 抓取 worker 已退出 code=${code}`);
            robotWorker = null;
            robotWorkerReady = false;
            robotWorkerStarting = null;
            if (!settled) {
                settled = true;
                reject(new Error(`抓取 worker 启动失败，退出码 ${code}`));
            }
        });
    });

    return robotWorkerStarting;
}

async function handleStowCommand(ws) {
    if (!ROBOT_GRASP_REAL) {
        sendStatus(ws, 'completed', { message: '模拟模式无需收起' });
        return;
    }
    if (activeOrderId) {
        sendMessage(ws, { type: 'error', message: `订单 ${activeOrderId} 正在执行，完成后再收起` });
        return;
    }
    if (robotStowPromise) {
        sendMessage(ws, { type: 'error', message: '机械臂正在收起，请勿重复发送' });
        return;
    }
    if (!robotWorker || robotWorker.exitCode !== null) {
        sendStatus(ws, 'completed', { message: '机器人 worker 未启动，无需收起' });
        return;
    }
    sendStatus(ws, 'started', { message: '正在回收起姿态并关闭机器人连接' });

    const worker = robotWorker;
    robotWorkerReady = false;
    robotStowPromise = new Promise((resolve, reject) => {
        let settled = false;
        const finish = (error) => {
            if (settled) return;
            settled = true;
            clearTimeout(timeout);
            worker.off('close', onClose);
            worker.off('error', onError);
            if (error) reject(error);
            else resolve();
        };
        const onClose = (code) => {
            if (code === 0) finish();
            else finish(new Error(`机器人 worker 收起时退出，退出码 ${code}`));
        };
        const onError = (error) => finish(error);
        const timeout = setTimeout(() => {
            finish(new Error(`机械臂收起超时 (${ROBOT_STOW_TIMEOUT_MS}ms)`));
        }, ROBOT_STOW_TIMEOUT_MS);

        worker.once('close', onClose);
        worker.once('error', onError);
        worker.stdin.write(`${ROBOT_STOW_COMMAND}\n`, (error) => {
            if (error) finish(error);
        });
    });

    try {
        await robotStowPromise;
        sendStatus(ws, 'completed', { progress: 100, message: '机械臂已收起' });
    } catch (error) {
        sendStatus(ws, 'error', { message: error.message });
    } finally {
        robotStowPromise = null;
    }
}

// ========== 处理抓取命令 ==========
async function handleGrabCommand(ws, msg) {
    const orderId = String(msg.orderId || '').trim();
    if (!orderId || !Array.isArray(msg.items) || msg.items.length === 0) {
        sendMessage(ws, { type: 'error', message: '缺少 orderId 或 items' });
        return;
    }

    let items;
    let totalUnits;
    try {
        ({ items, totalUnits } = normalizeOrderItems(msg.items));
    } catch (error) {
        sendMessage(ws, { type: 'error', message: error.message });
        return;
    }
    if (robotStowPromise) {
        sendMessage(ws, { type: 'error', message: '机械臂正在收起，请稍后再试' });
        return;
    }
    if (activeOrderId) {
        sendMessage(ws, { type: 'error', message: `机器人正在处理订单 ${activeOrderId}，请稍后再试` });
        return;
    }

    log(`🤖 开始处理订单 ${orderId}，共 ${totalUnits} 件商品`);
    activeOrderId = orderId;

    try {
        sendStatus(ws, 'started', {
            orderId,
            message: '订单已接收'
        });

        let completedUnits = 0;
        for (let i = 0; i < items.length; i++) {
            const item = items[i];
            for (let n = 0; n < item.qty; n++) {
                const progressBase = (completedUnits / totalUnits) * 100;
                if (ROBOT_GRASP_REAL) {
                    await runRealGrab(ws, orderId, item, progressBase, totalUnits, n + 1);
                } else {
                    await runSimulatedGrab(ws, orderId, item, progressBase, totalUnits, n + 1);
                }
                completedUnits += 1;
            }
        }

        sendStatus(ws, 'completed', { orderId, message: '所有商品抓取完成', progress: 100 });
        log(`✅ 订单 ${orderId} 处理完成`);
    } catch (e) {
        log(`❌ 订单 ${orderId} 失败: ${e.message}`);
        sendStatus(ws, 'error', { orderId, message: e.message });
    } finally {
        activeOrderId = null;
    }
}

async function runSimulatedGrab(ws, orderId, item, progressBase, totalUnits, itemNo) {
    const itemName = item.name;
    const label = item.qty > 1 ? `${itemName} (${itemNo}/${item.qty})` : itemName;
    const progressStep = 100 / totalUnits;
    const pos = await getProductPosition(itemName, itemNo - 1);

    log(`  📦 模拟抓取: ${label} (${pos.shelf})`);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'planning', progress: progressBase, message: `正在规划到 ${pos.shelf} 的路径...`, position: ROBOT_HOME, targetPosition: pos });
    await sleep(600 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'moving', progress: progressBase + progressStep * 0.5, message: `机器人移动到 ${pos.shelf}`, position: { x: pos.x * 0.8, y: pos.y * 0.8, z: pos.z }, targetPosition: pos });
    await sleep(800 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'picking', progress: progressBase + progressStep * 0.8, message: `正在抓取 ${label}`, position: pos, targetPosition: pos });
    await sleep(1000 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'delivering', progress: progressBase + progressStep * 0.95, message: `${label} 正在运送至取货口`, position: { x: pos.x * 0.3, y: pos.y * 0.3, z: 50 }, targetPosition: { x: 0, y: 0, z: 50 } });
    await sleep(700 * ROBOT_SIM_DELAY_SCALE);

    log(`  ✅ ${label} 模拟完成`);
}

function runRealGrab(ws, orderId, item, progressBase, totalUnits, itemNo) {
    return ensureRobotWorker().then((worker) => new Promise((resolve, reject) => {
        const itemName = item.name;
        const label = item.qty > 1 ? `${itemName} (${itemNo}/${item.qty})` : itemName;
        const progressStep = 100 / totalUnits;
        let outputTail = '';
        let settled = false;

        log(`  📦 真实抓取(worker): ${label}`);
        sendStatus(ws, 'grabbing', {
            orderId,
            item: itemStatus(item),
            stage: 'planning',
            progress: progressBase,
            message: `正在识别并抓取 ${label}...`
        });

        let stdoutData;
        let stderrData;
        const cleanup = () => {
            clearTimeout(timeout);
            worker.stdout.off('data', stdoutData);
            worker.stderr.off('data', stderrData);
            worker.off('close', onClose);
        };
        const finishOk = () => {
            if (settled) return;
            settled = true;
            cleanup();
            sendStatus(ws, 'grabbing', {
                orderId,
                item: itemStatus(item),
                stage: 'delivering',
                progress: Math.min(99, progressBase + progressStep),
                message: `${label} 抓取完成，机器人保持使能等待下一单`
            });
            resolve();
        };
        const finishError = (message) => {
            if (settled) return;
            settled = true;
            cleanup();
            reject(new Error(`${label}: ${message}`));
        };
        const timeout = setTimeout(() => {
            finishError(`抓取超时 (${ROBOT_GRAB_TIMEOUT_MS}ms): ${summarizeOutput(outputTail) || '无输出'}`);
        }, ROBOT_GRAB_TIMEOUT_MS);
        const onLine = (line) => {
            if (settled) return;
            const trimmed = line.trim();
            if (!trimmed) return;
            const stage = inferGrabStage(trimmed);
            if (stage) {
                const messages = {
                    planning: `正在识别 ${label}`,
                    moving: '机械臂正在移动',
                    picking: `正在抓取 ${label}`,
                    delivering: `${label} 正在运送至取货口`
                };
                sendStatus(ws, 'grabbing', {
                    orderId,
                    item: itemStatus(item),
                    stage: stage.stage,
                    progress: Math.min(99, progressBase + stage.weight * progressStep),
                    message: messages[stage.stage]
                });
            }
            if (trimmed.includes('[前端常驻模式][完成]')) {
                finishOk();
                return;
            }
            if (trimmed.includes('[前端常驻模式][失败]')) {
                finishError(trimmed);
            }
        };
        const makeDataHandler = () => {
            const handleLines = createLineHandler(onLine);
            return (chunk) => {
                outputTail = (outputTail + chunk.toString()).slice(-4000);
                handleLines(chunk);
            };
        };
        const onClose = (code) => finishError(`worker 已退出 code=${code}: ${summarizeOutput(outputTail)}`);

        stdoutData = makeDataHandler();
        stderrData = makeDataHandler();
        worker.stdout.on('data', stdoutData);
        worker.stderr.on('data', stderrData);
        worker.on('close', onClose);
        worker.stdin.write(`${itemName}\n`, (error) => {
            if (error) finishError(`无法写入抓取 worker: ${error.message}`);
        });
    }));
}

async function getProductPosition(itemName, index) {
    const product = readProducts().find((item) => item.name === itemName || String(item.id) === String(itemName));
    if (product) {
        return { x: product.shelf_x, y: product.shelf_y, z: product.shelf_z, shelf: product.location || '货架' };
    }
    return { x: 100, y: (index + 1) * 100, z: 50, shelf: '未知' };
}

function itemStatus(item) {
    return {
        id: item.id,
        name: item.name,
        qty: item.qty,
        emoji: item.emoji
    };
}

function inferGrabStage(line) {
    if (line.includes('拍照中') || line.includes('多帧稳定候选') || line.includes('目标过滤')) {
        return { stage: 'planning', weight: 0.2 };
    }
    if (line.includes('MovL') || line.includes('MovJ') || line.includes('移动到') || line.includes('对齐路径')) {
        return { stage: 'moving', weight: 0.45 };
    }
    if (line.includes('执行抓取动作') || line.includes('探入路径') || line.includes('闭合')) {
        return { stage: 'picking', weight: 0.75 };
    }
    if (line.includes('Place') || line.includes('放置') || line.includes('抓取完成')) {
        return { stage: 'delivering', weight: 0.95 };
    }
    return null;
}

function summarizeOutput(text) {
    return text.split(/\r?\n/).map(s => s.trim()).filter(Boolean).slice(-8).join(' | ');
}

function sendStatus(ws, status, data) {
    sendMessage(ws, { type: 'status', status, timestamp: Date.now(), ...data });
}

function sleep(ms) { return new Promise(resolve => setTimeout(resolve, ms)); }
