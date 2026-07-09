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
const { WebSocketServer } = require('ws');

const PORT = Number(process.env.PORT || 3000);
const WS_PORT = Number(process.env.WS_PORT || 3001);
const ROBOT_GRASP_MODE = (process.env.ROBOT_GRASP_MODE || 'sim').toLowerCase();
const ROBOT_GRASP_REAL = ['real', 'robot', 'hardware'].includes(ROBOT_GRASP_MODE);
const GRASP_SCRIPT = process.env.GRASP_SCRIPT || path.resolve(__dirname, '../dipan/shelf_grasp.py');
const GRASP_CWD = path.dirname(GRASP_SCRIPT);
const DOBOT_IP = process.env.DOBOT_IP || '192.168.5.1';
const PRODUCTS_DB_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
let activeOrderId = null;
let robotWorker = null;
let robotWorkerReady = false;
let robotWorkerStarting = null;

// 机器人初始位置
const ROBOT_HOME = { x: 0, y: 0, z: 100 };

// ========== 日志 ==========
function log(msg) {
    const time = new Date().toLocaleTimeString('zh-CN');
    console.log(`[${time}] ${msg}`);
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

function isPlaceholderProductName(value) {
    const name = String(value || '').trim();
    return !name || ['商品名', '商品名称', 'name', 'product', 'placeholder'].includes(name.toLowerCase());
}

function findProductByOrderItem(item) {
    const products = readProducts();
    const itemId = item.id == null ? '' : String(item.id);
    const itemName = String(item.name || '').trim();
    return products.find((product) => String(product.id) === itemId)
        || products.find((product) => product.name === itemName)
        || null;
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

server.listen(PORT, () => {
    log(`HTTP 服务已启动: http://localhost:${PORT}`);
    log(`商品管理页面: http://localhost:${PORT}/admin.html`);
    log(`抓取模式: ${ROBOT_GRASP_REAL ? '真实机械臂' : '模拟'} (${ROBOT_GRASP_MODE})`);
});

// ========== WebSocket 服务 ==========
const wss = new WebSocketServer({ port: WS_PORT });

log(`WebSocket 服务已启动: ws://localhost:${WS_PORT}`);

function broadcast(message) {
    const data = JSON.stringify(message);
    for (const client of wss.clients) {
        if (client.readyState === 1) client.send(data);
    }
}

wss.on('connection', (ws) => {
    log('✅ 前端已连接');

    ws.on('message', async (raw) => {
        let msg;
        try { msg = JSON.parse(raw.toString()); }
        catch (e) { ws.send(JSON.stringify({ type: 'error', message: '无效的 JSON 格式' })); return; }

        log(`📩 收到命令: ${JSON.stringify(msg)}`);

        if (msg.type === 'grab') {
            handleGrabCommand(ws, msg);
        } else if (msg.type === 'stow') {
            handleStowCommand(ws);
        } else if (msg.type === 'ping') {
            ws.send(JSON.stringify({ type: 'pong', time: Date.now() }));
        } else {
            ws.send(JSON.stringify({ type: 'error', message: `未知命令类型: ${msg.type}` }));
        }
    });

    ws.on('close', () => { log('❌ 前端已断开'); });
    ws.on('error', (err) => { log(`⚠️ WebSocket 错误: ${err.message}`); });

    ws.send(JSON.stringify({ type: 'connected', message: '已连接到机器人控制系统', robotStatus: 'idle' }));
});

function ensureRobotWorker() {
    if (!ROBOT_GRASP_REAL) return Promise.resolve(null);
    if (robotWorker && !robotWorker.killed && robotWorkerReady) return Promise.resolve(robotWorker);
    if (robotWorkerStarting) return robotWorkerStarting;

    robotWorkerStarting = new Promise((resolve, reject) => {
        log('🤖 启动常驻抓取 worker...');
        robotWorkerReady = false;
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

        const handleOutput = (chunk) => {
            const text = chunk.toString();
            for (const line of text.split(/\r?\n/)) {
                const trimmed = line.trim();
                if (!trimmed) continue;
                log(`  [worker] ${trimmed}`);
                if (trimmed.includes('等待商品名')) {
                    robotWorkerReady = true;
                    resolve(child);
                }
                broadcast({ type: 'robot_log', line: trimmed, timestamp: Date.now() });
            }
        };

        child.stdout.on('data', handleOutput);
        child.stderr.on('data', handleOutput);
        child.on('error', (e) => {
            robotWorker = null;
            robotWorkerReady = false;
            robotWorkerStarting = null;
            reject(e);
        });
        child.on('close', (code) => {
            log(`🤖 抓取 worker 已退出 code=${code}`);
            robotWorker = null;
            robotWorkerReady = false;
            robotWorkerStarting = null;
        });
    });

    return robotWorkerStarting;
}

async function handleStowCommand(ws) {
    if (!ROBOT_GRASP_REAL) {
        ws.send(JSON.stringify({ type: 'status', status: 'completed', message: '模拟模式无需收起', timestamp: Date.now() }));
        return;
    }
    if (activeOrderId) {
        ws.send(JSON.stringify({ type: 'error', message: `订单 ${activeOrderId} 正在执行，完成后再收起` }));
        return;
    }
    if (!robotWorker || robotWorker.killed) {
        ws.send(JSON.stringify({ type: 'status', status: 'completed', message: '机器人 worker 未启动，无需收起', timestamp: Date.now() }));
        return;
    }
    ws.send(JSON.stringify({ type: 'status', status: 'started', message: '正在回收起姿态并关闭机器人连接', timestamp: Date.now() }));
    robotWorker.stdin.write('__stow__\n');
}

// ========== 处理抓取命令（从数据库读取位置信息） ==========
async function handleGrabCommand(ws, msg) {
    const { orderId, items } = msg;
    if (!orderId || !items || !Array.isArray(items)) {
        ws.send(JSON.stringify({ type: 'error', message: '缺少 orderId 或 items' }));
        return;
    }
    if (activeOrderId) {
        ws.send(JSON.stringify({ type: 'error', message: `机器人正在处理订单 ${activeOrderId}，请稍后再试` }));
        return;
    }
    if (ROBOT_GRASP_REAL) {
        const invalid = items.find((item) => {
            const product = findProductByOrderItem(item);
            return !product || isPlaceholderProductName(product.name);
        });
        if (invalid) {
            ws.send(JSON.stringify({ type: 'error', message: `无效商品，已拒绝启动真实抓取: ${invalid.name || invalid.id || '空'}` }));
            return;
        }
    }

    log(`🤖 开始处理订单 ${orderId}，共 ${items.length} 件商品`);
    activeOrderId = orderId;

    try {
        sendStatus(ws, 'started', {
            orderId,
            message: ROBOT_GRASP_REAL ? '订单已接收，开始调用真实抓取程序' : '订单已接收，开始执行模拟抓取任务'
        });

        for (let i = 0; i < items.length; i++) {
            const item = items[i];
            const qty = Math.max(1, Number(item.qty || 1));
            for (let n = 0; n < qty; n++) {
                const itemProgressIndex = i + n / qty;
                const progressBase = (itemProgressIndex / items.length) * 100;
                if (ROBOT_GRASP_REAL) {
                    await runRealGrab(ws, orderId, item, progressBase, items.length, n + 1, qty);
                } else {
                    await runSimulatedGrab(ws, orderId, item, i, items.length);
                    break;
                }
            }
        }

        sendStatus(ws, 'completed', { orderId, message: '所有商品抓取完成！请取走您的商品', progress: 100, position: { x: 0, y: 0, z: 100 } });
        log(`✅ 订单 ${orderId} 处理完成`);
    } catch (e) {
        log(`❌ 订单 ${orderId} 失败: ${e.message}`);
        sendStatus(ws, 'error', { orderId, message: e.message });
    } finally {
        activeOrderId = null;
    }
}

async function runSimulatedGrab(ws, orderId, item, index, total) {
    const itemName = item.name || item.id;
    const pos = await getProductPosition(itemName, index);

    log(`  📦 模拟抓取: ${itemName} (${pos.shelf})`);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'planning', progress: (index / total) * 100, message: `正在规划到 ${pos.shelf} 的路径...`, position: ROBOT_HOME, targetPosition: pos });
    await sleep(600);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'moving', progress: (index / total) * 100 + (50 / total), message: `机器人移动到 ${pos.shelf}`, position: { x: pos.x * 0.8, y: pos.y * 0.8, z: pos.z }, targetPosition: pos });
    await sleep(800);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'picking', progress: (index / total) * 100 + (80 / total), message: `正在从 ${pos.shelf} 抓取 ${itemName} × ${item.qty}`, position: pos, targetPosition: pos });
    await sleep(1000);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'delivering', progress: (index / total) * 100 + (95 / total), message: `${itemName} × ${item.qty} 正在运送至取货口`, position: { x: pos.x * 0.3, y: pos.y * 0.3, z: 50 }, targetPosition: { x: 0, y: 0, z: 50 } });
    await sleep(700);

    log(`  ✅ ${itemName} × ${item.qty} 模拟完成`);
}

function runRealGrab(ws, orderId, item, progressBase, totalItems, itemNo, itemQty) {
    const product = findProductByOrderItem(item);
    if (!product || isPlaceholderProductName(product.name)) {
        return Promise.reject(new Error(`无效商品或商品未入库: ${item.name || item.id || '空'}`));
    }

    return ensureRobotWorker().then((worker) => new Promise((resolve, reject) => {
        const itemName = product.name;
        const label = itemQty > 1 ? `${itemName} (${itemNo}/${itemQty})` : itemName;
        const progressStep = 80 / totalItems / Math.max(1, itemQty);
        let outputTail = '';

        log(`  📦 真实抓取(worker): ${label}`);
        sendStatus(ws, 'grabbing', {
            orderId,
            item: itemStatus(item),
            stage: 'planning',
            progress: progressBase,
            message: `正在识别并抓取 ${label}...`
        });

        const cleanup = () => {
            worker.stdout.off('data', onData);
            worker.stderr.off('data', onData);
            worker.off('close', onClose);
        };
        const finishOk = () => {
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
            cleanup();
            reject(new Error(`${label}: ${message}`));
        };
        const onData = (chunk) => {
            const text = chunk.toString();
            outputTail = (outputTail + text).slice(-4000);
            for (const line of text.split(/\r?\n/)) {
                const trimmed = line.trim();
                if (!trimmed) continue;
                const stage = inferGrabStage(trimmed);
                if (stage) {
                    sendStatus(ws, 'grabbing', {
                        orderId,
                        item: itemStatus(item),
                        stage: stage.stage,
                        progress: Math.min(99, progressBase + stage.weight * progressStep),
                        message: stage.message || trimmed
                    });
                }
                if (trimmed.includes('[前端常驻模式][完成]')) finishOk();
                if (trimmed.includes('[前端常驻模式][失败]')) finishError(trimmed);
            }
        };
        const onClose = (code) => finishError(`worker 已退出 code=${code}: ${summarizeOutput(outputTail)}`);

        worker.stdout.on('data', onData);
        worker.stderr.on('data', onData);
        worker.on('close', onClose);
        worker.stdin.write(`${itemName}\n`);
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
    const product = findProductByOrderItem(item);
    return {
        id: product ? product.id : item.id,
        name: product ? product.name : (item.name || item.id),
        qty: item.qty,
        emoji: product ? product.emoji : item.emoji
    };
}

function inferGrabStage(line) {
    if (line.includes('拍照中') || line.includes('多帧稳定候选') || line.includes('目标过滤')) {
        return { stage: 'planning', weight: 0.2, message: line };
    }
    if (line.includes('MovL') || line.includes('MovJ') || line.includes('移动到') || line.includes('对齐路径')) {
        return { stage: 'moving', weight: 0.45, message: line };
    }
    if (line.includes('执行抓取动作') || line.includes('探入路径') || line.includes('闭合')) {
        return { stage: 'picking', weight: 0.75, message: line };
    }
    if (line.includes('Place') || line.includes('放置') || line.includes('抓取完成')) {
        return { stage: 'delivering', weight: 0.95, message: line };
    }
    return null;
}

function summarizeOutput(text) {
    return text.split(/\r?\n/).map(s => s.trim()).filter(Boolean).slice(-8).join(' | ');
}

function sendStatus(ws, status, data) {
    ws.send(JSON.stringify({ type: 'status', status, timestamp: Date.now(), ...data }));
}

function sleep(ms) { return new Promise(resolve => setTimeout(resolve, ms)); }
