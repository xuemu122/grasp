/**
 * 机器人点餐后端服务
 * 功能：
 *   1. 提供静态页面
 *   2. WebSocket 实时通信，接收前端下单命令
 *   3. REST API - 商品管理（增删改查）
 *   4. 模拟机器人抓取逻辑
 */
const http = require('http');
const https = require('https');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const { spawn } = require('child_process');
const { WebSocket, WebSocketServer } = require('ws');
const { ChassisClient } = require('./chassis_client');

const PORT = Number(process.env.PORT || 3000);
const WS_PORT = Number(process.env.WS_PORT || 3001);
const HOST = process.env.HOST || '0.0.0.0';
const WS_HOST = process.env.WS_HOST || HOST;
const ROBOT_GRASP_MODE = (process.env.ROBOT_GRASP_MODE || 'sim').toLowerCase();
const ROBOT_GRASP_REAL = ['real', 'robot', 'hardware'].includes(ROBOT_GRASP_MODE);
const GRASP_SCRIPT = process.env.GRASP_SCRIPT || path.resolve(__dirname, '../dipan/shelf_grasp.py');
const GRASP_CWD = path.dirname(GRASP_SCRIPT);
const DOBOT_IP = process.env.DOBOT_IP || '192.168.5.1';
const CHASSIS_HOST = process.env.CHASSIS_HOST || process.env.MOVE_ROBOT_HOST || '192.168.124.90';
const CHASSIS_PORT = Number(process.env.CHASSIS_PORT || process.env.MOVE_ROBOT_PORT || 31001);
const CHASSIS_MAP_HTTP_PROTOCOL = String(process.env.CHASSIS_MAP_HTTP_PROTOCOL || 'http').replace(/:$/, '');
const CHASSIS_MAP_HTTP_HOST = process.env.CHASSIS_MAP_HTTP_HOST || CHASSIS_HOST;
const CHASSIS_MAP_HTTP_PORT = Number(process.env.CHASSIS_MAP_HTTP_PORT || 9001);
const CHASSIS_PLACE_MARKER_DEFAULT = process.env.CHASSIS_PLACE_MARKER || 'yaojia1';
const CHASSIS_NAVIGATION_DEFAULT_ENABLE = /^(1|true|yes|on)$/i.test(process.env.CHASSIS_NAVIGATION_ENABLE || '');
const CHASSIS_REQUEST_TIMEOUT_MS = Number(process.env.CHASSIS_REQUEST_TIMEOUT_MS || 10000);
const CHASSIS_NAVIGATION_TIMEOUT_MS = Number(process.env.CHASSIS_NAVIGATION_TIMEOUT_MS || 180000);
const CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M = Number(process.env.CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M || 0.15);
const CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD = Number(process.env.CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD || 0.25);
const CHASSIS_NAVIGATION_POLL_MS = Number(process.env.CHASSIS_NAVIGATION_POLL_MS || 1000);
const PRODUCTS_DB_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
const CHASSIS_SETTINGS_FILE = process.env.CHASSIS_SETTINGS_FILE || path.join(path.dirname(PRODUCTS_DB_FILE), 'chassis_settings.json');
const CHASSIS_MAP_CACHE_DIR = process.env.CHASSIS_MAP_CACHE_DIR || path.join(path.dirname(PRODUCTS_DB_FILE), 'chassis_map_cache');
const CHASSIS_MAP_SNAPSHOT_FILE = path.join(CHASSIS_MAP_CACHE_DIR, 'current.json');
const PRODUCT_UPLOAD_DIR = process.env.PRODUCT_UPLOAD_DIR || path.join(path.dirname(PRODUCTS_DB_FILE), 'uploads');
const PRODUCT_UPLOAD_MAX_BYTES = Number(process.env.PRODUCT_UPLOAD_MAX_BYTES || 5 * 1024 * 1024);
const ROBOT_WORKER_READY_TIMEOUT_MS = Number(process.env.ROBOT_WORKER_READY_TIMEOUT_MS || 90000);
const ROBOT_GRAB_TIMEOUT_MS = Number(process.env.ROBOT_GRAB_TIMEOUT_MS || 120000);
const ROBOT_STOW_TIMEOUT_MS = Number(process.env.ROBOT_STOW_TIMEOUT_MS || 120000);
const ROBOT_FILTER_PREVIEW_TIMEOUT_MS = Number(process.env.ROBOT_FILTER_PREVIEW_TIMEOUT_MS || 30000);
const ROBOT_STOW_COMMAND = process.env.ROBOT_STOW_COMMAND || '**stow**';
const ROBOT_MAX_ITEM_QTY = Number(process.env.ROBOT_MAX_ITEM_QTY || 20);
const ROBOT_MAX_ORDER_UNITS = Number(process.env.ROBOT_MAX_ORDER_UNITS || 50);
const ROBOT_SIM_DELAY_SCALE = Math.max(0, Number(process.env.ROBOT_SIM_DELAY_SCALE || 1));
const ROBOT_BROADCAST_LOGS = /^(1|true|yes|on)$/i.test(process.env.ROBOT_BROADCAST_LOGS || '');
const ADMIN_USERNAME = process.env.ADMIN_USERNAME || '';
const ADMIN_PASSWORD = process.env.ADMIN_PASSWORD || '';
const ADMIN_SESSION_SECRET = process.env.ADMIN_SESSION_SECRET || crypto.randomBytes(32).toString('hex');
const ADMIN_SESSION_TTL_MS = Number(process.env.ADMIN_SESSION_TTL_MS || 8 * 60 * 60 * 1000);
const ADMIN_COOKIE_NAME = 'grasp_admin_session';
const ORDER_TOKEN_SECRET = process.env.ORDER_TOKEN_SECRET || ADMIN_SESSION_SECRET;
const ORDER_TOKEN_TTL_MS = Number(process.env.ORDER_TOKEN_TTL_MS || 24 * 60 * 60 * 1000);
const loginAttempts = new Map();
let activeOrderId = null;
let robotWorker = null;
let robotWorkerReady = false;
let robotWorkerStarting = null;
let robotStowPromise = null;
let robotFilterPreviewPromise = null;
let chassisRasterCache = null;
let chassisMapRefreshPromise = null;

// 机器人初始位置
const SHELF_LAYERS_FILE = process.env.SHELF_LAYERS_FILE || path.resolve(__dirname, '../dipan/shelf_layers.json');

// ========== 货架层位姿管理 ==========
function loadShelfLayers() {
    try {
        if (!fs.existsSync(SHELF_LAYERS_FILE)) {
            log(`⚠️ 货架层位姿文件不存在: ${SHELF_LAYERS_FILE}`);
            return { layers: {}, current: null };
        }
        const raw = JSON.parse(fs.readFileSync(SHELF_LAYERS_FILE, 'utf8'));
        return {
            layers: raw.layers || {},
            current: raw.current || null
        };
    } catch (e) {
        log(`⚠️ 货架层位姿读取失败: ${e.message}`);
        return { layers: {}, current: null };
    }
}

/**
 * 从商品 location 字段中提取货架层号
 * 支持格式: "饮料区A1货架", "饮料区A货架3层", "零食区B货架2层", "饮料区A货架三层", "饮料区A货架4层"
 * 返回数字层号，如无法解析返回 null
 */
function parseLayerFromLocation(location) {
    if (!location) return null;

    // 匹配 "X层" 或 "X 层" 或 "第X层" 模式（支持中文数字 一 到 九）
    const cnDigits = { '一':1,'二':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9 };

    // 尝试匹配: "3层", "3 层", "三层", "第3层", "第三层"
    let m = location.match(/第?(\d+|[一二三四五六七八九])\s*层/);
    if (m) {
        const val = m[1];
        return cnDigits[val] !== undefined ? cnDigits[val] : parseInt(val, 10);
    }

    // 尝试匹配 "A3" 这种格式（3表示层号）
    m = location.match(/[A-Za-z](\d+)/);
    if (m) {
        return parseInt(m[1], 10);
    }

    return null;
}

/**
 * 根据商品查询对应货架层的机械臂位姿
 * @returns {{ layer: number, name: string, pose: number[] } | null}
 */
function getShelfPoseForProduct(product) {
    const layerIndex = parseLayerFromLocation(product.location);
    if (layerIndex === null) return null;

    const shelfData = loadShelfLayers();
    const layer = shelfData.layers[String(layerIndex)];
    if (!layer) return null;

    return {
        layer: layerIndex,
        name: layer.name || `第${layerIndex}层`,
        pose: layer.pose  // [x, y, z, rx, ry, rz] mm & deg
    };
}

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
    '.jpeg': 'image/jpeg',
    '.webp': 'image/webp',
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

function sendJSON(res, data, status = 200, headers = {}) {
    res.writeHead(status, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store',
        ...headers
    });
    res.end(JSON.stringify(data));
}

function parseBinaryBody(req, maxBytes) {
    return new Promise((resolve, reject) => {
        const chunks = [];
        let size = 0;
        let settled = false;

        req.on('data', (chunk) => {
            if (settled) return;
            size += chunk.length;
            if (size > maxBytes) {
                settled = true;
                const error = new Error(`图片不能超过 ${Math.floor(maxBytes / 1024 / 1024)} MB`);
                error.statusCode = 413;
                reject(error);
                return;
            }
            chunks.push(chunk);
        });
        req.on('end', () => {
            if (!settled) resolve(Buffer.concat(chunks));
        });
        req.on('error', (error) => {
            if (!settled) reject(error);
        });
    });
}

function detectImageType(buffer) {
    if (buffer.length >= 8 && buffer.subarray(0, 8).equals(Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]))) {
        return { extension: 'png', contentType: 'image/png' };
    }
    if (buffer.length >= 3 && buffer[0] === 0xff && buffer[1] === 0xd8 && buffer[2] === 0xff) {
        return { extension: 'jpg', contentType: 'image/jpeg' };
    }
    if (buffer.length >= 12 && buffer.subarray(0, 4).toString('ascii') === 'RIFF' && buffer.subarray(8, 12).toString('ascii') === 'WEBP') {
        return { extension: 'webp', contentType: 'image/webp' };
    }
    return null;
}

function safeEqual(left, right) {
    const a = crypto.createHash('sha256').update(String(left)).digest();
    const b = crypto.createHash('sha256').update(String(right)).digest();
    return crypto.timingSafeEqual(a, b);
}

function parseCookies(req) {
    const cookies = {};
    for (const part of String(req.headers.cookie || '').split(';')) {
        const index = part.indexOf('=');
        if (index < 0) continue;
        cookies[part.slice(0, index).trim()] = part.slice(index + 1).trim();
    }
    return cookies;
}

function signSession(payload) {
    return crypto.createHmac('sha256', ADMIN_SESSION_SECRET).update(payload).digest('base64url');
}

function createAdminSession() {
    const payload = Buffer.from(JSON.stringify({ user: ADMIN_USERNAME, exp: Date.now() + ADMIN_SESSION_TTL_MS })).toString('base64url');
    return `${payload}.${signSession(payload)}`;
}

function readAdminSession(req) {
    const token = parseCookies(req)[ADMIN_COOKIE_NAME];
    if (!token) return null;
    const separator = token.lastIndexOf('.');
    if (separator < 1) return null;
    const payload = token.slice(0, separator);
    if (!safeEqual(token.slice(separator + 1), signSession(payload))) return null;
    try {
        const session = JSON.parse(Buffer.from(payload, 'base64url').toString('utf8'));
        return session.user === ADMIN_USERNAME && Number(session.exp) > Date.now() ? session : null;
    } catch (error) {
        return null;
    }
}

function requireAdmin(req, res) {
    if (readAdminSession(req)) return true;
    sendJSON(res, { code: 1, message: '请先登录管理员账号' }, 401);
    return false;
}

function clientIp(req) {
    return String(req.headers['x-real-ip'] || req.socket.remoteAddress || '').trim();
}

function loginBlocked(ip) {
    const now = Date.now();
    const attempt = loginAttempts.get(ip);
    if (!attempt || now - attempt.startedAt > 15 * 60 * 1000) {
        loginAttempts.set(ip, { count: 0, startedAt: now });
        return false;
    }
    return attempt.count >= 5;
}

function recordLoginFailure(ip) {
    const attempt = loginAttempts.get(ip) || { count: 0, startedAt: Date.now() };
    attempt.count += 1;
    loginAttempts.set(ip, attempt);
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
        chassis_marker: raw.chassis_marker || raw.marker_name || '',
        shelf_x: Number(raw.shelf_x || 0),
        shelf_y: Number(raw.shelf_y || 0),
        shelf_z: Number(raw.shelf_z || 50),
        emoji: raw.emoji || '📦',
        price: Number(raw.price || 0),
        stock: Number(raw.stock ?? 100),
        status: Number(raw.status ?? 1),
    };
}

function readProductStore() {
    if (!fs.existsSync(PRODUCTS_DB_FILE)) return { products: [], completedOrders: {} };
    try {
        const data = JSON.parse(fs.readFileSync(PRODUCTS_DB_FILE, 'utf8'));
        return {
            products: Array.isArray(data.products) ? data.products.map((p, idx) => normalizeProduct(p, idx + 1)) : [],
            completedOrders: data.completedOrders && typeof data.completedOrders === 'object' ? data.completedOrders : {}
        };
    } catch (e) {
        log(`⚠️ 商品数据读取失败: ${e.message}`);
        return { products: [], completedOrders: {} };
    }
}

function readProducts() {
    return readProductStore().products;
}

function writeProductStore(store) {
    fs.mkdirSync(path.dirname(PRODUCTS_DB_FILE), { recursive: true });
    const tempFile = `${PRODUCTS_DB_FILE}.${process.pid}.tmp`;
    fs.writeFileSync(tempFile, JSON.stringify(store, null, 2), 'utf8');
    fs.renameSync(tempFile, PRODUCTS_DB_FILE);
}

function writeProducts(products) {
    const store = readProductStore();
    writeProductStore({ ...store, products });
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
            return [item.name, item.spec, item.manufacturer, item.location, item.chassis_marker]
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
        const rawShelfLayer = item.shelf_layer ?? item.shelfLayer;
        const shelfLayer = rawShelfLayer === undefined || rawShelfLayer === null || rawShelfLayer === ''
            ? null
            : Number(rawShelfLayer);
        if (shelfLayer !== null && (!Number.isInteger(shelfLayer) || shelfLayer < 1 || shelfLayer > 10)) {
            throw new Error(`商品 ${name} 的 shelf_layer 必须是 1..10 的整数`);
        }
        const marker = String(item.chassis_marker || item.marker_name || '').trim();
        if (/\r|\n/.test(marker)) throw new Error(`商品 ${name} 的底盘点位不能包含换行`);
        return { ...item, name, qty, shelf_layer: shelfLayer, chassis_marker: marker || null };
    });

    const totalUnits = normalized.reduce((sum, item) => sum + item.qty, 0);
    if (totalUnits > ROBOT_MAX_ORDER_UNITS) {
        throw new Error(`订单商品总数不能超过 ${ROBOT_MAX_ORDER_UNITS}`);
    }
    return { items: normalized, totalUnits };
}

function normalizeInventoryItems(items) {
    if (!Array.isArray(items) || items.length === 0) throw new Error('订单商品不能为空');
    return items.map((item, index) => {
        const id = Number(item && item.id);
        const qty = Number(item && item.qty);
        if (!Number.isInteger(id) || id < 1) throw new Error(`第 ${index + 1} 个商品 id 无效`);
        if (!Number.isInteger(qty) || qty < 1 || qty > ROBOT_MAX_ITEM_QTY) {
            throw new Error(`第 ${index + 1} 个商品数量必须是 1..${ROBOT_MAX_ITEM_QTY} 的整数`);
        }
        return { id, qty, name: String(item.name || '').trim() };
    });
}

function signOrderPayload(payload) {
    return crypto.createHmac('sha256', ORDER_TOKEN_SECRET).update(payload).digest('base64url');
}

function createOrderToken(orderId, items) {
    const payload = Buffer.from(JSON.stringify({ orderId, items, exp: Date.now() + ORDER_TOKEN_TTL_MS })).toString('base64url');
    return `${payload}.${signOrderPayload(payload)}`;
}

function readOrderToken(token) {
    const value = String(token || '');
    const separator = value.lastIndexOf('.');
    if (separator < 1) return null;
    const payload = value.slice(0, separator);
    if (!safeEqual(value.slice(separator + 1), signOrderPayload(payload))) return null;
    try {
        const order = JSON.parse(Buffer.from(payload, 'base64url').toString('utf8'));
        return Number(order.exp) > Date.now() ? order : null;
    } catch (error) {
        return null;
    }
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
    if (pathname === '/api/admin/session' && req.method === 'GET') {
        const session = readAdminSession(req);
        sendJSON(res, { code: 0, authenticated: Boolean(session), username: session ? session.user : null });
        return;
    }

    if (pathname === '/api/admin/login' && req.method === 'POST') {
        if (!ADMIN_USERNAME || !ADMIN_PASSWORD) {
            sendJSON(res, { code: 1, message: '管理员登录尚未配置' }, 503);
            return;
        }
        const ip = clientIp(req);
        if (loginBlocked(ip)) {
            sendJSON(res, { code: 1, message: '登录失败次数过多，请 15 分钟后重试' }, 429);
            return;
        }
        const body = await parseBody(req);
        if (!body || !safeEqual(body.username, ADMIN_USERNAME) || !safeEqual(body.password, ADMIN_PASSWORD)) {
            recordLoginFailure(ip);
            sendJSON(res, { code: 1, message: '账号或密码错误' }, 401);
            return;
        }
        loginAttempts.delete(ip);
        const maxAge = Math.max(1, Math.floor(ADMIN_SESSION_TTL_MS / 1000));
        sendJSON(res, { code: 0, username: ADMIN_USERNAME }, 200, {
            'Set-Cookie': `${ADMIN_COOKIE_NAME}=${createAdminSession()}; Path=/; HttpOnly; Secure; SameSite=Strict; Max-Age=${maxAge}`
        });
        return;
    }

    if (pathname === '/api/admin/logout' && req.method === 'POST') {
        sendJSON(res, { code: 0 }, 200, {
            'Set-Cookie': `${ADMIN_COOKIE_NAME}=; Path=/; HttpOnly; Secure; SameSite=Strict; Max-Age=0`
        });
        return;
    }

    if (pathname === '/api/admin/uploads' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
        try {
            const image = await parseBinaryBody(req, PRODUCT_UPLOAD_MAX_BYTES);
            if (image.length === 0) {
                sendJSON(res, { code: 1, message: '请选择要上传的图片' }, 400);
                return;
            }
            const imageType = detectImageType(image);
            if (!imageType) {
                sendJSON(res, { code: 1, message: '仅支持 JPG、PNG 和 WebP 图片' }, 415);
                return;
            }

            fs.mkdirSync(PRODUCT_UPLOAD_DIR, { recursive: true });
            const fileName = `${Date.now()}-${crypto.randomBytes(8).toString('hex')}.${imageType.extension}`;
            const filePath = path.join(PRODUCT_UPLOAD_DIR, fileName);
            const tempPath = `${filePath}.${process.pid}.tmp`;
            fs.writeFileSync(tempPath, image, { flag: 'wx', mode: 0o640 });
            fs.renameSync(tempPath, filePath);
            sendJSON(res, { code: 0, url: `/api/uploads/${fileName}` }, 201);
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message || '图片上传失败' }, error.statusCode || 500);
        }
        return;
    }

    const uploadedImageMatch = pathname.match(/^\/api\/uploads\/([a-zA-Z0-9-]+\.(?:png|jpg|webp))$/);
    if (uploadedImageMatch && (req.method === 'GET' || req.method === 'HEAD')) {
        const fileName = uploadedImageMatch[1];
        const filePath = path.join(PRODUCT_UPLOAD_DIR, fileName);
        fs.readFile(filePath, (error, data) => {
            if (error) {
                res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
                res.end('404 Not Found');
                return;
            }
            res.writeHead(200, {
                'Content-Type': MIME[path.extname(fileName)] || 'application/octet-stream',
                'Content-Length': data.length,
                'Cache-Control': 'public, max-age=31536000, immutable',
                'X-Content-Type-Options': 'nosniff'
            });
            res.end(req.method === 'HEAD' ? undefined : data);
        });
        return;
    }

    if (pathname === '/api/orders/prepare' && req.method === 'POST') {
        const body = await parseBody(req);
        try {
            const items = normalizeInventoryItems(body && body.items);
            const products = readProducts();
            for (const item of items) {
                const product = products.find((candidate) => Number(candidate.id) === item.id);
                if (!product || Number(product.status) !== 1) throw new Error(`商品不可用: ${item.name || item.id}`);
                if (Number(product.stock) < item.qty) throw new Error(`${product.name} 库存不足，当前库存 ${product.stock}`);
                item.name = product.name;
                item.shelf_layer = parseLayerFromLocation(product.location);
                item.chassis_marker = product.chassis_marker || null;
            }
            const orderId = `ORD-${Date.now().toString(36).toUpperCase()}-${crypto.randomBytes(4).toString('hex').toUpperCase()}`;
            sendJSON(res, { code: 0, orderId, items, completionToken: createOrderToken(orderId, items) });
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message }, 400);
        }
        return;
    }

    const completeOrderMatch = pathname.match(/^\/api\/orders\/([^/]+)\/complete$/);
    if (completeOrderMatch && req.method === 'POST') {
        const body = await parseBody(req);
        const orderId = decodeURIComponent(completeOrderMatch[1]);
        const order = readOrderToken(body && body.completionToken);
        if (!order || order.orderId !== orderId) {
            sendJSON(res, { code: 1, message: '订单凭证无效或已过期' }, 401);
            return;
        }
        try {
            const store = readProductStore();
            if (store.completedOrders[orderId]) {
                sendJSON(res, { code: 0, alreadyCompleted: true, updated: store.completedOrders[orderId].updated });
                return;
            }
            const updated = [];
            for (const item of normalizeInventoryItems(order.items)) {
                const product = store.products.find((candidate) => Number(candidate.id) === item.id);
                if (!product) throw new Error(`商品不存在: ${item.name || item.id}`);
                if (Number(product.stock) < item.qty) throw new Error(`${product.name} 库存不足，当前库存 ${product.stock}`);
                product.stock = Number(product.stock) - item.qty;
                updated.push({ id: product.id, name: product.name, qty: item.qty, stock: product.stock });
            }
            store.completedOrders[orderId] = { completedAt: Date.now(), updated };
            const orderIds = Object.keys(store.completedOrders);
            for (const oldOrderId of orderIds.slice(0, Math.max(0, orderIds.length - 1000))) {
                delete store.completedOrders[oldOrderId];
            }
            writeProductStore(store);
            log(`📉 订单 ${orderId} 已扣减库存: ${updated.map((item) => `${item.name} x${item.qty}`).join(', ')}`);
            sendJSON(res, { code: 0, alreadyCompleted: false, updated });
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message }, 409);
        }
        return;
    }

    // GET /api/products - 获取所有商品
    if (pathname === '/api/products' && req.method === 'GET') {
        try {
            const search = url.searchParams.get('search') || '';
            const category = url.searchParams.get('category') || '';
            const includeInactive = url.searchParams.get('include_inactive') === '1' && Boolean(readAdminSession(req));
            const source = includeInactive ? readProducts() : readProducts().filter((item) => Number(item.status) === 1);
            const rows = filterProducts(source, { search, category });
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
            if (!product || (Number(product.status) !== 1 && !readAdminSession(req))) {
                sendJSON(res, { code: 1, message: '商品不存在' }, 404);
                return;
            }
            sendJSON(res, { code: 0, data: product });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // POST /api/products - 新增商品
    if (pathname === '/api/products' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
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
        if (!requireAdmin(req, res)) return;
        const body = await parseBody(req);
        if (!body) { sendJSON(res, { code: 1, message: '无效数据' }, 400); return; }
        try {
            const products = readProducts();
            const index = products.findIndex((item) => String(item.id) === updateMatch[1]);
            if (index < 0) { sendJSON(res, { code: 1, message: '商品不存在' }, 404); return; }
            const allowed = ['name','spec','packaging','category','manufacturer','photo_front','photo_back','location','chassis_marker','shelf_x','shelf_y','shelf_z','emoji','price','stock','status'];
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
        if (!requireAdmin(req, res)) return;
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
            const includeInactive = url.searchParams.get('include_inactive') === '1' && Boolean(readAdminSession(req));
            const source = includeInactive ? readProducts() : readProducts().filter((item) => Number(item.status) === 1);
            const categories = [...new Set(source.map((item) => item.category).filter(Boolean))].sort();
            sendJSON(res, { code: 0, data: categories });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // GET/PUT /api/chassis/navigation-config - 抓取/放置导航开关与放置点
    if (pathname === '/api/chassis/navigation-config' && req.method === 'GET') {
        if (!requireAdmin(req, res)) return;
        sendJSON(res, { code: 0, data: readChassisSettings() });
        return;
    }
    if (pathname === '/api/chassis/navigation-config' && req.method === 'PUT') {
        if (!requireAdmin(req, res)) return;
        const body = await parseBody(req);
        if (!body || (body.enabled !== undefined && typeof body.enabled !== 'boolean')) {
            sendJSON(res, { code: 1, message: 'enabled 必须是布尔值' }, 400);
            return;
        }
        try {
            const current = readChassisSettings();
            const placeMarker = body.place_marker === undefined
                ? current.place_marker
                : validateChassisMarkerName(body.place_marker);
            const settings = {
                navigation_enabled: body.enabled === undefined ? current.navigation_enabled : body.enabled,
                place_marker: placeMarker
            };
            writeChassisSettings(settings);
            log(`🚙 底盘导航已${settings.navigation_enabled ? '开启' : '关闭'}，放置点=${settings.place_marker}`);
            sendJSON(res, { code: 0, data: settings });
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message }, 500);
        }
        return;
    }

    // GET /api/chassis/map - 读取本地地图快照；refresh=1 时先从底盘更新
    if (pathname === '/api/chassis/map' && req.method === 'GET') {
        if (!requireAdmin(req, res)) return;
        try {
            const forceRefresh = url.searchParams.get('refresh') === '1';
            sendJSON(res, { code: 0, data: await getChassisMapSnapshot({ forceRefresh }) });
        } catch (error) {
            log(`⚠️ 底盘地图读取失败: ${error.message}`);
            sendJSON(res, { code: 1, message: error.message }, 502);
        }
        return;
    }

    // GET /api/chassis/map-image - 代理底盘内网栅格图，避免浏览器跨域和无法访问底盘网段
    if (pathname === '/api/chassis/map-image' && req.method === 'GET') {
        if (!requireAdmin(req, res)) return;
        try {
            const mapName = validateChassisMapName(url.searchParams.get('map'));
            const floor = validateChassisFloor(url.searchParams.get('floor'));
            const raster = await getCachedChassisRaster(mapName, floor);
            res.writeHead(200, {
                'Content-Type': 'image/png',
                'Content-Length': raster.image.length,
                'Cache-Control': 'private, no-store',
                'X-Content-Type-Options': 'nosniff'
            });
            res.end(raster.image);
        } catch (error) {
            log(`⚠️ 底盘栅格图读取失败: ${error.message}`);
            sendJSON(res, { code: 1, message: error.message }, 502);
        }
        return;
    }

    // GET /api/shelf/layers - 获取所有货架层位姿
    if (pathname === '/api/shelf/layers' && req.method === 'GET') {
        try {
            const shelfData = loadShelfLayers();
            sendJSON(res, { code: 0, data: shelfData.layers, current: shelfData.current });
        } catch (e) {
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // POST /api/robot/goto-layer - 让机械臂移动到指定货架层
    // body: { productName, productId, layer }
    if (pathname === '/api/robot/goto-layer' && req.method === 'POST') {
        const body = await parseBody(req);
        try {
            const layerIndex = await resolveLayerIndex(body);
            if (!layerIndex) {
                sendJSON(res, { code: 1, message: '无法确定目标货架层，请提供 productName/productId 或 layer 参数' }, 400);
                return;
            }

            const shelfData = loadShelfLayers();
            const layer = shelfData.layers[String(layerIndex)];
            if (!layer) {
                sendJSON(res, { code: 1, message: `未找到货架层 ${layerIndex} 的位姿数据` }, 404);
                return;
            }

            const pose = layer.pose;
            log(`🎯 移动到货架层 ${layerIndex} (${layer.name}): [${pose.join(', ')}]`);

            const result = await moveRobotToLayer(layerIndex, layer, pose);

            sendJSON(res, {
                code: 0,
                message: `机械臂已到达第${layerIndex}层 (${layer.name})`,
                data: {
                    layer: layerIndex,
                    name: layer.name,
                    pose
                },
                ...result
            });
        } catch (e) {
            log(`❌ 移动机械臂失败: ${e.message}`);
            sendJSON(res, { code: 1, message: e.message }, 500);
        }
        return;
    }

    // POST /api/robot/movel - 直接控制机械臂移动到指定位姿
    // body: { x, y, z, rx, ry, rz }
    if (pathname === '/api/robot/movel' && req.method === 'POST') {
        const body = await parseBody(req);
        try {
            if (!body || body.x === undefined || body.y === undefined || body.z === undefined) {
                sendJSON(res, { code: 1, message: '缺少位姿参数 x, y, z' }, 400);
                return;
            }
            const pose = [
                Number(body.x),
                Number(body.y),
                Number(body.z),
                Number(body.rx ?? 135),
                Number(body.ry ?? 82),
                Number(body.rz ?? -52)
            ];
            log(`🎯 移动到指定位姿: [${pose.join(', ')}]`);

            const result = await moveRobotToLayer(null, null, pose);

            sendJSON(res, {
                code: 0,
                message: '机械臂已移动到目标位姿',
                data: { pose },
                ...result
            });
        } catch (e) {
            log(`❌ 移动机械臂失败: ${e.message}`);
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
        const cacheControl = ['.html', '.js', '.css'].includes(ext)
            ? 'no-store, max-age=0'
            : 'public, max-age=3600';
        res.writeHead(200, {
            'Content-Type': contentType,
            'Cache-Control': cacheControl,
            'X-Content-Type-Options': 'nosniff'
        });
        res.end(data);
    });
});

server.listen(PORT, HOST, () => {
    log(`HTTP 服务已启动: http://${HOST}:${PORT}`);
    log(`商品管理页面: http://localhost:${PORT}/admin.html`);
    log(`抓取模式: ${ROBOT_GRASP_REAL ? '真实机械臂' : '模拟'} (${ROBOT_GRASP_MODE})`);
    const chassisSettings = readChassisSettings();
    log(`底盘接口: ${CHASSIS_HOST}:${CHASSIS_PORT}，栅格地图: ${CHASSIS_MAP_HTTP_PROTOCOL}://${CHASSIS_MAP_HTTP_HOST}:${CHASSIS_MAP_HTTP_PORT}`);
    log(`底盘导航: ${chassisSettings.navigation_enabled ? '已开启' : '已关闭'}，放置点: ${chassisSettings.place_marker}`);
    refreshChassisMapSnapshot()
        .then((snapshot) => log(`底盘地图已更新到本地: ${snapshot.map.map_name || snapshot.map.hotel_id}/${snapshot.map.floor}`))
        .catch((error) => {
            try {
                const cached = readChassisMapSnapshot();
                log(`⚠️ 底盘地图启动更新失败，继续使用本地快照 ${cached.map.map_name || cached.map.hotel_id}/${cached.map.floor}: ${error.message}`);
            } catch {
                log(`⚠️ 底盘地图启动更新失败且没有本地快照: ${error.message}`);
            }
        });
});

// ========== WebSocket 服务 ==========
const wss = new WebSocketServer({ port: WS_PORT, host: WS_HOST, maxPayload: 64 * 1024 });

wss.on('listening', () => {
    log(`WebSocket 服务已启动: ws://${WS_HOST}:${WS_PORT}`);
});

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
        } else if (msg.type === 'goto-layer') {
            handleGotoLayerCommand(ws, msg);
        } else if (msg.type === 'filter-preview') {
            handleFilterPreviewCommand(ws);
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

async function handleFilterPreviewCommand(ws) {
    if (!ROBOT_GRASP_REAL) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: '模拟模式没有真实相机过滤预览' });
        return;
    }

    if (activeOrderId) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: `订单 ${activeOrderId} 正在执行，请稍后再试` });
        return;
    }
    if (robotStowPromise) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: '机械臂正在收起，请稍后再试' });
        return;
    }
    if (robotFilterPreviewPromise) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: '已有过滤预览正在执行，请稍后再试' });
        return;
    }

    let worker;
    try {
        worker = await ensureRobotWorker();
    } catch (error) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: error.message });
        return;
    }
    if (!worker) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: '机器人 worker 未启动' });
        return;
    }

    sendMessage(ws, { type: 'filter-preview', status: 'started', message: '正在采集层板过滤预览' });
    robotFilterPreviewPromise = new Promise((resolve, reject) => {
        let settled = false;
        const finish = (error, result) => {
            if (settled) return;
            settled = true;
            clearTimeout(timeout);
            worker.stdout.off('data', onStdout);
            worker.stderr.off('data', onStderr);
            worker.off('close', onClose);
            if (error) reject(error);
            else resolve(result);
        };
        const onLine = (line) => {
            const trimmed = line.trim();
            if (!trimmed) return;
            const completed = trimmed.match(/\[过滤预览\]\[完成\] image=(\S+) report=(\S+)/);
            if (completed) {
                try {
                    const imagePath = completed[1];
                    const reportPath = completed[2];
                    const report = JSON.parse(fs.readFileSync(reportPath, 'utf8'));
                    const imageBase64 = fs.readFileSync(imagePath).toString('base64');
                    finish(null, {
                        type: 'filter-preview',
                        status: 'completed',
                        imageMime: 'image/jpeg',
                        imageBase64,
                        report
                    });
                } catch (error) {
                    finish(error);
                }
                return;
            }
            if (trimmed.includes('[过滤预览][失败]')) finish(new Error(trimmed));
        };
        const stdoutLines = createLineHandler(onLine);
        const stderrLines = createLineHandler(onLine);
        const onStdout = (chunk) => stdoutLines(chunk);
        const onStderr = (chunk) => stderrLines(chunk);
        const onClose = (code) => finish(new Error(`过滤预览 worker 已退出 code=${code}`));
        const timeout = setTimeout(() => finish(new Error(`过滤预览超时 (${ROBOT_FILTER_PREVIEW_TIMEOUT_MS}ms)`)), ROBOT_FILTER_PREVIEW_TIMEOUT_MS);
        worker.stdout.on('data', onStdout);
        worker.stderr.on('data', onStderr);
        worker.once('close', onClose);
        worker.stdin.write('__filter_preview__\n', (error) => {
            if (error) finish(error);
        });
    });

    try {
        const result = await robotFilterPreviewPromise;
        sendMessage(ws, result);
    } catch (error) {
        sendMessage(ws, { type: 'filter-preview', status: 'error', message: error.message });
    } finally {
        robotFilterPreviewPromise = null;
    }
}

function readChassisSettings() {
    try {
        if (!fs.existsSync(CHASSIS_SETTINGS_FILE)) {
            return { navigation_enabled: CHASSIS_NAVIGATION_DEFAULT_ENABLE, place_marker: CHASSIS_PLACE_MARKER_DEFAULT };
        }
        const data = JSON.parse(fs.readFileSync(CHASSIS_SETTINGS_FILE, 'utf8'));
        return {
            navigation_enabled: Boolean(data.navigation_enabled),
            place_marker: validateChassisMarkerName(data.place_marker || CHASSIS_PLACE_MARKER_DEFAULT)
        };
    } catch (error) {
        log(`⚠️ 底盘设置读取失败: ${error.message}`);
        return { navigation_enabled: CHASSIS_NAVIGATION_DEFAULT_ENABLE, place_marker: CHASSIS_PLACE_MARKER_DEFAULT };
    }
}

function writeChassisSettings(settings) {
    fs.mkdirSync(path.dirname(CHASSIS_SETTINGS_FILE), { recursive: true });
    const tempFile = `${CHASSIS_SETTINGS_FILE}.${process.pid}.tmp`;
    fs.writeFileSync(tempFile, JSON.stringify(settings, null, 2), 'utf8');
    fs.renameSync(tempFile, CHASSIS_SETTINGS_FILE);
}

function validateChassisMarkerName(value) {
    const marker = String(value || '').trim();
    if (!marker || marker.length > 128 || /[\r\n\u0000]/.test(marker) || marker.includes('/')) {
        throw new Error('底盘放置点名称无效');
    }
    return marker;
}

function chassisClient() {
    return new ChassisClient({ host: CHASSIS_HOST, port: CHASSIS_PORT, timeoutMs: CHASSIS_REQUEST_TIMEOUT_MS });
}

function chassisResults(response) {
    return response && response.results && typeof response.results === 'object' ? response.results : {};
}

function chassisResponseError(response) {
    const status = response && response.status ? response.status : 'UNKNOWN';
    const message = response && response.error_message ? `: ${response.error_message}` : '';
    return `status=${status}${message}`;
}

function normalizeAngle(angle) {
    let value = Number(angle) || 0;
    while (value > Math.PI) value -= Math.PI * 2;
    while (value < -Math.PI) value += Math.PI * 2;
    return value;
}

function markerTheta(orientation = {}) {
    return normalizeAngle(2 * Math.atan2(Number(orientation.z) || 0, Number(orientation.w) || 1));
}

function normalizeMarkers(results) {
    if (!results || typeof results !== 'object') return [];
    return Object.entries(results).map(([name, marker]) => {
        const position = marker && marker.pose && marker.pose.position ? marker.pose.position : {};
        const orientation = marker && marker.pose && marker.pose.orientation ? marker.pose.orientation : {};
        return {
            name: (marker && marker.marker_name) || name,
            marker_name: (marker && marker.marker_name) || name,
            floor: Number((marker && marker.floor) || 0),
            type: Number(marker && (marker.key ?? marker.type) || 0),
            x: Number(position.x || 0),
            y: Number(position.y || 0),
            z: Number(position.z || 0),
            theta: markerTheta(orientation),
            orientation,
            raw: marker || {}
        };
    });
}

function validateChassisMapName(value) {
    const mapName = String(value || '').trim();
    if (!mapName || mapName.length > 128 || !/^[\p{L}\p{N}_.-]+$/u.test(mapName)) {
        throw new Error('底盘地图名无效');
    }
    return mapName;
}

function validateChassisFloor(value) {
    const floor = Number(value);
    if (!Number.isInteger(floor) || floor < -100 || floor > 100) throw new Error('底盘楼层无效');
    return floor;
}

function chassisMapAssetPath(mapName, floor, filename) {
    return `/maps/${encodeURIComponent(validateChassisMapName(mapName))}/${validateChassisFloor(floor)}/${filename}`;
}

function fetchChassisHttp(assetPath, maxBytes = 20 * 1024 * 1024) {
    return new Promise((resolve, reject) => {
        const transport = CHASSIS_MAP_HTTP_PROTOCOL === 'https' ? https : http;
        const request = transport.get({
            protocol: `${CHASSIS_MAP_HTTP_PROTOCOL}:`,
            hostname: CHASSIS_MAP_HTTP_HOST,
            port: CHASSIS_MAP_HTTP_PORT,
            path: `${assetPath}?r=${Date.now()}`,
            timeout: CHASSIS_REQUEST_TIMEOUT_MS,
            headers: { Accept: '*/*', Cookie: 'scan=false' },
            rejectUnauthorized: false
        }, (response) => {
            if (response.statusCode !== 200) {
                response.resume();
                reject(new Error(`底盘地图 HTTP ${response.statusCode}: ${assetPath}`));
                return;
            }
            const chunks = [];
            let size = 0;
            response.on('data', (chunk) => {
                size += chunk.length;
                if (size > maxBytes) {
                    request.destroy(new Error(`底盘地图文件超过 ${maxBytes} 字节`));
                    return;
                }
                chunks.push(chunk);
            });
            response.once('end', () => resolve(Buffer.concat(chunks)));
        });
        request.once('timeout', () => request.destroy(new Error(`底盘地图 HTTP 请求超时: ${assetPath}`)));
        request.once('error', reject);
    });
}

function parseMapYaml(buffer) {
    const metadata = {};
    for (const line of buffer.toString('utf8').split(/\r?\n/)) {
        const match = line.match(/^\s*([A-Za-z_][\w-]*)\s*:\s*(.*?)\s*$/);
        if (!match) continue;
        const key = match[1];
        const value = match[2].replace(/^['"]|['"]$/g, '');
        if (/^\[.*\]$/.test(value)) {
            metadata[key] = value.slice(1, -1).split(',').map((item) => Number(item.trim()));
        } else if (value !== '' && Number.isFinite(Number(value))) {
            metadata[key] = Number(value);
        } else {
            metadata[key] = value;
        }
    }
    return metadata;
}

function pngDimensions(buffer) {
    if (buffer.length < 24 || buffer.toString('ascii', 1, 4) !== 'PNG') {
        throw new Error('底盘 map.png 不是有效的 PNG 文件');
    }
    return { width: buffer.readUInt32BE(16), height: buffer.readUInt32BE(20) };
}

async function fetchChassisRaster(mapName, floor) {
    const safeMapName = validateChassisMapName(mapName);
    const safeFloor = validateChassisFloor(floor);
    const key = `${safeMapName}/${safeFloor}`;
    if (chassisRasterCache && chassisRasterCache.key === key && Date.now() - chassisRasterCache.cachedAt < 5000) {
        return chassisRasterCache.value;
    }
    const [yaml, image] = await Promise.all([
        fetchChassisHttp(chassisMapAssetPath(safeMapName, safeFloor, 'map.yaml'), 1024 * 1024),
        fetchChassisHttp(chassisMapAssetPath(safeMapName, safeFloor, 'map.png'))
    ]);
    const value = {
        mapName: safeMapName,
        floor: safeFloor,
        yaml,
        metadata: parseMapYaml(yaml),
        ...pngDimensions(image),
        image
    };
    chassisRasterCache = { key, cachedAt: Date.now(), value };
    return value;
}

function chassisMapCachePaths(mapName, floor) {
    const safeMapName = validateChassisMapName(mapName);
    const safeFloor = validateChassisFloor(floor);
    const directory = path.join(CHASSIS_MAP_CACHE_DIR, encodeURIComponent(safeMapName), String(safeFloor));
    return {
        directory,
        yaml: path.join(directory, 'map.yaml'),
        image: path.join(directory, 'map.png')
    };
}

function writeFileAtomic(filePath, data, options = {}) {
    fs.mkdirSync(path.dirname(filePath), { recursive: true });
    const temporary = `${filePath}.${process.pid}.${Date.now()}.tmp`;
    fs.writeFileSync(temporary, data, options);
    fs.renameSync(temporary, filePath);
}

function writeChassisMapSnapshot(map, markers, asset) {
    const mapName = validateChassisMapName(map.map_name || map.hotel_id);
    const floor = validateChassisFloor(map.floor);
    const paths = chassisMapCachePaths(mapName, floor);
    const fetchedAt = new Date().toISOString();
    writeFileAtomic(paths.yaml, asset.yaml, { mode: 0o640 });
    writeFileAtomic(paths.image, asset.image, { mode: 0o640 });
    const snapshot = {
        schema_version: 1,
        map,
        markers,
        raster: {
            available: true,
            image_url: `/api/chassis/map-image?map=${encodeURIComponent(mapName)}&floor=${floor}`,
            width: asset.width,
            height: asset.height,
            metadata: asset.metadata,
            storage: 'local_snapshot',
            updated_at: fetchedAt
        },
        host: CHASSIS_HOST,
        port: CHASSIS_PORT,
        fetchedAt
    };
    writeFileAtomic(CHASSIS_MAP_SNAPSHOT_FILE, JSON.stringify(snapshot, null, 2), { encoding: 'utf8', mode: 0o640 });
    return snapshot;
}

function readChassisMapSnapshot(expectedMapName, expectedFloor) {
    const snapshot = JSON.parse(fs.readFileSync(CHASSIS_MAP_SNAPSHOT_FILE, 'utf8'));
    const map = snapshot && snapshot.map;
    const mapName = validateChassisMapName(map && (map.map_name || map.hotel_id));
    const floor = validateChassisFloor(map && map.floor);
    if (expectedMapName !== undefined && validateChassisMapName(expectedMapName) !== mapName) {
        throw new Error(`本地地图快照不是 ${expectedMapName}`);
    }
    if (expectedFloor !== undefined && validateChassisFloor(expectedFloor) !== floor) {
        throw new Error(`本地地图快照不是第${expectedFloor}层`);
    }
    const paths = chassisMapCachePaths(mapName, floor);
    const image = fs.readFileSync(paths.image);
    const dimensions = pngDimensions(image);
    const yaml = fs.readFileSync(paths.yaml);
    return {
        ...snapshot,
        raster: {
            ...snapshot.raster,
            available: true,
            image_url: `/api/chassis/map-image?map=${encodeURIComponent(mapName)}&floor=${floor}`,
            width: dimensions.width,
            height: dimensions.height,
            metadata: snapshot.raster && snapshot.raster.metadata
                ? snapshot.raster.metadata
                : parseMapYaml(yaml),
            storage: 'local_snapshot'
        }
    };
}

async function refreshChassisMapSnapshot() {
    if (chassisMapRefreshPromise) return chassisMapRefreshPromise;
    const refresh = (async () => {
        const client = chassisClient();
        const [mapResponse, markerResponse] = await Promise.all([
            client.request('/api/map/get_current_map'),
            client.request('/api/markers/query_list')
        ]);
        if (mapResponse.status !== 'OK') throw new Error(`获取当前地图失败 (${chassisResponseError(mapResponse)})`);
        if (markerResponse.status !== 'OK') throw new Error(`获取 marker 失败 (${chassisResponseError(markerResponse)})`);
        const map = chassisResults(mapResponse);
        const mapName = map.map_name || map.hotel_id;
        const floor = Number(map.floor);
        const asset = await fetchChassisRaster(mapName, floor);
        return writeChassisMapSnapshot(map, normalizeMarkers(chassisResults(markerResponse)), asset);
    })();
    chassisMapRefreshPromise = refresh;
    try {
        return await refresh;
    } finally {
        if (chassisMapRefreshPromise === refresh) chassisMapRefreshPromise = null;
    }
}

async function getChassisMapSnapshot({ forceRefresh = false } = {}) {
    if (forceRefresh) return refreshChassisMapSnapshot();
    if (chassisMapRefreshPromise) {
        try {
            await chassisMapRefreshPromise;
        } catch {
            // Startup refresh failure is handled by the persisted snapshot below.
        }
    }
    try {
        return readChassisMapSnapshot();
    } catch (cacheError) {
        try {
            return await refreshChassisMapSnapshot();
        } catch (refreshError) {
            throw new Error(`本地地图快照不可用 (${cacheError.message})，底盘更新也失败 (${refreshError.message})`);
        }
    }
}

async function getCachedChassisRaster(mapName, floor) {
    try {
        readChassisMapSnapshot(mapName, floor);
    } catch {
        await refreshChassisMapSnapshot();
    }
    const paths = chassisMapCachePaths(mapName, floor);
    const image = fs.readFileSync(paths.image);
    return { image, ...pngDimensions(image) };
}

function currentPoseAtMarker(statusInfo, marker) {
    const pose = statusInfo && statusInfo.current_pose;
    if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return false;
    const currentFloor = Number(statusInfo.current_floor || 0);
    if (marker.floor && currentFloor && marker.floor !== currentFloor) return false;
    const distance = Math.hypot(Number(pose.x) - marker.x, Number(pose.y) - marker.y);
    const thetaDistance = Math.abs(normalizeAngle(Number(pose.theta || 0) - marker.theta));
    return distance <= CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M
        && thetaDistance <= CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD;
}

async function navigateChassisToMarker(markerName, { beforeMove } = {}) {
    const client = chassisClient();
    const [markerResponse, statusResponse] = await Promise.all([
        client.request('/api/markers/query_list'),
        client.request('/api/robot_status')
    ]);
    if (markerResponse.status !== 'OK') throw new Error(`查询底盘点位失败 (${chassisResponseError(markerResponse)})`);
    if (statusResponse.status !== 'OK') throw new Error(`查询底盘状态失败 (${chassisResponseError(statusResponse)})`);

    const marker = normalizeMarkers(chassisResults(markerResponse))
        .find((item) => item.name === markerName);
    if (!marker) throw new Error(`底盘点位不存在: ${markerName}`);

    const initialStatus = chassisResults(statusResponse);
    if (currentPoseAtMarker(initialStatus, marker)) {
        log(`  🚙 底盘已在点位 ${markerName} 容差范围内，跳过导航`);
        return { skipped: true, marker };
    }

    if (beforeMove) await beforeMove({ marker, initialStatus });

    const params = new URLSearchParams({
        marker: markerName,
        distance_tolerance: String(CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M),
        theta_tolerance: String(CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD)
    });
    const moveResponse = await client.request(`/api/move?${params.toString()}`);
    if (moveResponse.status !== 'OK') throw new Error(`底盘导航任务下发失败 (${chassisResponseError(moveResponse)})`);

    log(`  🚙 底盘开始前往 ${markerName}`);
    const deadline = Date.now() + CHASSIS_NAVIGATION_TIMEOUT_MS;
    let seenCurrentTask = false;
    let lastState = '';
    while (Date.now() < deadline) {
        await sleep(CHASSIS_NAVIGATION_POLL_MS);
        let response;
        try {
            response = await client.request('/api/robot_status');
        } catch (error) {
            log(`  [底盘] 状态查询暂时失败: ${error.message}`);
            continue;
        }
        if (response.status !== 'OK') continue;
        const info = chassisResults(response);
        const moveStatus = info.move_status;
        const moveTarget = info.move_target;
        const targetMatches = moveTarget === markerName;
        if (targetMatches && moveStatus === 'running') seenCurrentTask = true;
        const state = `${moveStatus}/${info.running_status || ''}/${moveTarget || ''}`;
        if (state !== lastState) {
            log(`  [底盘] ${state}`);
            lastState = state;
        }
        if (moveStatus === 'succeeded' && (targetMatches || seenCurrentTask)) {
            log(`  ✅ 底盘已到达 ${markerName}`);
            return { skipped: false, marker, taskId: moveResponse.task_id || chassisResults(moveResponse).task_id || null };
        }
        if (['failed', 'canceled'].includes(moveStatus) && (targetMatches || seenCurrentTask)) {
            throw new Error(`底盘前往 ${markerName} 未成功: ${moveStatus}`);
        }
    }

    try {
        await client.request('/api/move/cancel');
    } catch (error) {
        log(`  [底盘] 导航超时后取消失败: ${error.message}`);
    }
    throw new Error(`底盘前往 ${markerName} 超时 (${CHASSIS_NAVIGATION_TIMEOUT_MS}ms)`);
}

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
                CHASSIS_HOST,
                CHASSIS_PORT: String(CHASSIS_PORT),
                CHASSIS_SETTINGS_FILE,
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
    if (robotFilterPreviewPromise) {
        sendMessage(ws, { type: 'error', message: '正在采集层板过滤预览，请稍后再试' });
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

// ========== 处理移动到层命令 ==========
async function handleGotoLayerCommand(ws, msg) {
    try {
        if (robotFilterPreviewPromise) {
            throw new Error('正在采集层板过滤预览，请稍后再试');
        }
        const layerIndex = await resolveLayerIndex(msg);
        if (!layerIndex) {
            sendMessage(ws, { type: 'error', message: '请提供 productName/productId 或 layer 参数' });
            return;
        }

        const shelfData = loadShelfLayers();
        const layer = shelfData.layers[String(layerIndex)];
        if (!layer) {
            sendMessage(ws, { type: 'error', message: `未找到货架层 ${layerIndex} 的位姿数据` });
            return;
        }

        const pose = layer.pose;
        const layerName = layer.name || `第${layerIndex}层`;
        log(`🎯 WebSocket 移动到层 ${layerIndex} (${layerName}): [${pose.join(', ')}]`);

        sendStatus(ws, 'layer-moving', {
            layer: layerIndex,
            name: layerName,
            pose,
            message: `机械臂正在移动到第${layerIndex}层 (${layerName})...`
        });

        const result = await moveRobotToLayer(layerIndex, layer, pose);

        sendStatus(ws, 'layer-reached', {
            layer: layerIndex,
            name: layerName,
            pose,
            message: `机械臂已到达第${layerIndex}层 (${layerName})`,
            ...result
        });

        log(`✅ 已到达层 ${layerIndex} (${layerName})`);
    } catch (e) {
        log(`❌ 移动到层失败: ${e.message}`);
        sendStatus(ws, 'error', { message: e.message });
    }
}

// ========== 处理抓取命令 ==========
async function handleGrabCommand(ws, msg) {
    const orderId = String(msg.orderId || '').trim();
    if (!orderId || !Array.isArray(msg.items) || msg.items.length === 0) {
        sendMessage(ws, { type: 'error', message: '缺少 orderId 或 items' });
        return;
    }

    let orderItems = msg.items;
    if (msg.completionToken) {
        const preparedOrder = readOrderToken(msg.completionToken);
        if (preparedOrder && preparedOrder.orderId === orderId && Array.isArray(preparedOrder.items)) {
            orderItems = preparedOrder.items;
            log(`🔐 订单 ${orderId} 已从签名凭证恢复货架层和底盘点位`);
        } else {
            log(`⚠️ 订单 ${orderId} 的签名凭证无效，改用 WebSocket items`);
        }
    }

    let items;
    let totalUnits;
    try {
        ({ items, totalUnits } = normalizeOrderItems(orderItems));
    } catch (error) {
        sendMessage(ws, { type: 'error', message: error.message });
        return;
    }
    if (robotStowPromise) {
        sendMessage(ws, { type: 'error', message: '机械臂正在收起，请稍后再试' });
        return;
    }
    if (robotFilterPreviewPromise) {
        sendMessage(ws, { type: 'error', message: '正在采集层板过滤预览，请稍后再试' });
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

    const shelfInfo = pos.layer ? `第${pos.layer}层(${pos.layerName || ''})` : pos.shelf;
    log(`  📦 模拟抓取: ${label} → ${shelfInfo} 位姿[${pos.x.toFixed(1)}, ${pos.y.toFixed(1)}, ${pos.z.toFixed(1)}]`);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'planning', progress: progressBase, message: `正在规划到 ${shelfInfo} 的路径...`, position: ROBOT_HOME, targetPosition: pos, shelfLayer: pos.layer });
    await sleep(600 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'moving', progress: progressBase + progressStep * 0.35, message: `机器人移动到 ${shelfInfo}`, position: { x: pos.x * 0.8, y: pos.y * 0.8, z: pos.z }, targetPosition: pos, shelfLayer: pos.layer });
    await sleep(800 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'arrived', progress: progressBase + progressStep * 0.5, message: `已到达 ${shelfInfo}`, position: pos, targetPosition: pos, shelfLayer: pos.layer });
    await sleep(300 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'picking', progress: progressBase + progressStep * 0.8, message: `正在抓取 ${label} (${shelfInfo})`, position: pos, targetPosition: pos, shelfLayer: pos.layer });
    await sleep(1000 * ROBOT_SIM_DELAY_SCALE);

    sendStatus(ws, 'grabbing', { orderId, item: itemStatus(item), stage: 'delivering', progress: progressBase + progressStep * 0.95, message: `${label} 正在运送至取货口`, position: { x: pos.x * 0.3, y: pos.y * 0.3, z: 50 }, targetPosition: { x: 0, y: 0, z: 50 }, shelfLayer: pos.layer });
    await sleep(700 * ROBOT_SIM_DELAY_SCALE);

    log(`  ✅ ${label} 模拟完成`);
}

async function runRealGrab(ws, orderId, item, progressBase, totalUnits, itemNo) {
    const itemName = item.name;
    const label = item.qty > 1 ? `${itemName} (${itemNo}/${item.qty})` : itemName;
    const progressStep = 100 / totalUnits;

    log(`  📦 真实抓取(worker): ${label}`);
    sendStatus(ws, 'grabbing', {
        orderId,
        item: itemStatus(item),
        stage: 'planning',
        progress: progressBase,
        message: `正在查询 ${label} 的货架位置...`
    });

    const product = readProducts().find((candidate) => String(candidate.id) === String(item.id));
    const chassisMarker = item.chassis_marker || (product && product.chassis_marker) || '';
    const chassisNavigationEnabled = readChassisSettings().navigation_enabled;
    if (chassisNavigationEnabled && !chassisMarker) {
        throw new Error(`${label}: 已开启底盘导航，但商品未配置底盘点位`);
    }
    if (chassisNavigationEnabled && chassisMarker) {
        const workerAlreadyRunning = Boolean(robotWorker && !robotWorker.killed && robotWorkerReady);
        const navigationOptions = workerAlreadyRunning ? {
            beforeMove: async () => {
                sendStatus(ws, 'grabbing', {
                    orderId,
                    item: itemStatus(item),
                    stage: 'moving',
                    progress: progressBase + progressStep * 0.02,
                    message: '底盘移动前，机械臂正在收起...',
                    chassisMarker
                });
                await moveRobotToStowPose();
                sendStatus(ws, 'grabbing', {
                    orderId,
                    item: itemStatus(item),
                    stage: 'moving',
                    progress: progressBase + progressStep * 0.03,
                    message: `机械臂已收起，底盘正在前往 ${chassisMarker}...`,
                    chassisMarker
                });
            }
        } : {};
        if (!workerAlreadyRunning) {
            log('  [底盘] 首次订单保持机械臂现有收起姿态；到达商品点位后再启动抓取 worker');
            sendStatus(ws, 'grabbing', {
                orderId,
                item: itemStatus(item),
                stage: 'moving',
                progress: progressBase + progressStep * 0.03,
                message: `机械臂保持收起，底盘正在前往 ${chassisMarker}...`,
                chassisMarker
            });
        }
        await navigateChassisToMarker(chassisMarker, navigationOptions);
        log(`  [底盘] 已完成 ${chassisMarker} 到点检查，现在允许机械臂进入初始位和观察位`);
    } else if (chassisMarker) {
        log(`  🚙 抓取前底盘导航已关闭，跳过点位 ${chassisMarker}`);
    } else if (!chassisNavigationEnabled) {
        log('  🚙 抓取前底盘导航已关闭，机械臂将在当前位置执行');
    }
    let shelfPose = null;
    if (item.shelf_layer) {
        const shelfData = loadShelfLayers();
        const layerInfo = shelfData.layers[String(item.shelf_layer)];
        if (!layerInfo || !Array.isArray(layerInfo.pose)) {
            throw new Error(`${label}: 第${item.shelf_layer}层没有对应的货架层位姿`);
        }
        shelfPose = {
            layer: item.shelf_layer,
            name: layerInfo.name || `第${item.shelf_layer}层`,
            pose: layerInfo.pose
        };
    } else if (product) {
        shelfPose = getShelfPoseForProduct(product);
        if (!shelfPose) {
            throw new Error(`${label}: 库位“${product.location || '未设置'}”没有对应的货架层位姿`);
        }
    }

    if (shelfPose) {
        log(`  🎯 ${label} → 第${shelfPose.layer}层 (${shelfPose.name})，先移动再抓取`);
        sendStatus(ws, 'grabbing', {
            orderId,
            item: itemStatus(item),
            stage: 'moving',
            progress: progressBase + progressStep * 0.1,
            message: `正在移动到第${shelfPose.layer}层 (${shelfPose.name})...`,
            targetPosition: {
                x: shelfPose.pose[0],
                y: shelfPose.pose[1],
                z: shelfPose.pose[2],
                rx: shelfPose.pose[3],
                ry: shelfPose.pose[4],
                rz: shelfPose.pose[5]
            },
            shelfLayer: shelfPose.layer
        });
        await moveRobotToLayer(shelfPose.layer, shelfPose, shelfPose.pose);
        sendStatus(ws, 'grabbing', {
            orderId,
            item: itemStatus(item),
            stage: 'arrived',
            progress: progressBase + progressStep * 0.2,
            message: `已到达第${shelfPose.layer}层，开始识别 ${label}`,
            shelfLayer: shelfPose.layer
        });
    } else {
        log(`  ⚠️ 订单未携带货架层且商品 id=${item.id ?? '未提供'} 不在本地商品库，保持当前层执行兼容抓取`);
    }

    const worker = await ensureRobotWorker();
    return new Promise((resolve, reject) => {
        let outputTail = '';
        let settled = false;

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
            if (trimmed === `[前端常驻模式][完成] ${itemName}`) {
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
    });
}

async function getProductPosition(itemName, index) {
    const product = readProducts().find((item) => item.name === itemName || String(item.id) === String(itemName));
    if (product) {
        // 优先使用 shelf_layers.json 中的层位姿
        const shelfPose = getShelfPoseForProduct(product);
        if (shelfPose) {
            return {
                x: shelfPose.pose[0],
                y: shelfPose.pose[1],
                z: shelfPose.pose[2],
                rx: shelfPose.pose[3],
                ry: shelfPose.pose[4],
                rz: shelfPose.pose[5],
                shelf: product.location || '货架',
                layer: shelfPose.layer,
                layerName: shelfPose.name
            };
        }
        // 兜底：使用商品自身的 shelf_x/y/z
        return {
            x: product.shelf_x,
            y: product.shelf_y,
            z: product.shelf_z,
            shelf: product.location || '货架',
            layer: parseLayerFromLocation(product.location)
        };
    }
    return { x: 100, y: (index + 1) * 100, z: 50, shelf: '未知', layer: null };
}

function itemStatus(item) {
    return {
        id: item.id,
        name: item.name,
        qty: item.qty,
        emoji: item.emoji,
        chassis_marker: item.chassis_marker || null
    };
}

/**
 * 解析目标货架层索引
 * 优先级: body.layer > body.productName > body.productId
 */
async function resolveLayerIndex(body) {
    if (!body || typeof body !== 'object') return null;

    // 直接指定层号
    if (body.layer !== undefined && body.layer !== null) {
        const layer = parseInt(body.layer, 10);
        if (layer >= 1 && layer <= 10) return layer;
    }

    // 通过商品名查找
    if (body.productName) {
        const products = readProducts();
        const product = products.find((p) =>
            p.name === body.productName || String(p.id) === String(body.productName));
        if (product) {
            const idx = parseLayerFromLocation(product.location);
            if (idx) {
                log(`📦 商品"${product.name}" 位于 ${product.location} → 货架层 ${idx}`);
                return idx;
            }
        }
    }

    // 通过商品ID查找
    if (body.productId) {
        const products = readProducts();
        const product = products.find((p) => String(p.id) === String(body.productId));
        if (product) {
            const idx = parseLayerFromLocation(product.location);
            if (idx) {
                log(`📦 商品 id=${product.id} "${product.name}" 位于 ${product.location} → 货架层 ${idx}`);
                return idx;
            }
        }
    }

    return null;
}

/**
 * 移动机械臂到指定货架层
 * @returns {{ mode: string }}
 */
async function moveRobotToLayer(layerIndex, layerInfo, pose) {
    if (!ROBOT_GRASP_REAL) {
        const name = layerInfo ? layerInfo.name : `位姿 [${pose.join(', ')}]`;
        log(`  🟡 模拟模式: 机械臂移动到 ${name}`);
        return { mode: 'simulated', detail: `模拟移动到 ${name}` };
    }

    // 真实模式: 通过 worker 发送层号移动命令
    try {
        const worker = await ensureRobotWorker();
        if (!worker) {
            throw new Error('机器人 worker 未启动');
        }

        // 构造移动到层的命令: __layer_N__
        const layerCmd = layerIndex ? `__layer_${layerIndex}__` : `__movel_${pose.join(',')}__`;

        return new Promise((resolve, reject) => {
            let settled = false;
            let outputTail = '';

            const cleanup = () => {
                clearTimeout(timeout);
                worker.stdout.off('data', onData);
                worker.stderr.off('data', onData);
                worker.off('close', onClose);
            };

            const finishOk = () => {
                if (settled) return;
                settled = true;
                cleanup();
                resolve({ mode: 'real', detail: `已到达目标层/位姿` });
            };

            const finishError = (message) => {
                if (settled) return;
                settled = true;
                cleanup();
                reject(new Error(message));
            };

            const timeout = setTimeout(() => {
                finishError(`移动超时 (30s): ${summarizeOutput(outputTail) || '无输出'}`);
            }, 30000);

            const handleLines = createLineHandler((line) => {
                if (settled) return;
                const trimmed = line.trim();
                if (!trimmed) return;
                // “正在移动”日志也带有[完成]前缀，只在明确到达后才放行抓取。
                const moveCompleted = layerIndex
                    ? trimmed.includes(`[前端常驻模式][完成] 已到达第${layerIndex}层`)
                    : trimmed.includes('[前端常驻模式][完成] 已到达目标位姿');
                if (moveCompleted) {
                    finishOk();
                    return;
                }
                const moveFailed = trimmed.includes('[前端常驻模式][失败]')
                    && (trimmed.includes('移动到第') || trimmed.includes('移动到指定位姿'));
                if (moveFailed) {
                    finishError(trimmed);
                    return;
                }
            });

            const onData = (chunk) => {
                outputTail = (outputTail + chunk.toString()).slice(-4000);
                handleLines(chunk);
            };

            const onClose = (code) => {
                if (!settled) {
                    finishError(`worker 已退出 code=${code}`);
                }
            };

            worker.stdout.on('data', onData);
            worker.stderr.on('data', onData);
            worker.on('close', onClose);

            worker.stdin.write(`${layerCmd}\n`, (error) => {
                if (error) finishError(`无法写入 worker: ${error.message}`);
            });
        });
    } catch (e) {
        throw new Error(`真实模式移动失败: ${e.message}`);
    }
}

async function moveRobotToStowPose() {
    if (!ROBOT_GRASP_REAL) return { mode: 'simulated' };
    const worker = await ensureRobotWorker();
    if (!worker) throw new Error('机器人 worker 未启动');

    return new Promise((resolve, reject) => {
        let settled = false;
        let outputTail = '';
        const cleanup = () => {
            clearTimeout(timeout);
            worker.stdout.off('data', onData);
            worker.stderr.off('data', onData);
            worker.off('close', onClose);
        };
        const finish = (error) => {
            if (settled) return;
            settled = true;
            cleanup();
            if (error) reject(error);
            else resolve({ mode: 'real', detail: '已到达收起姿态' });
        };
        const timeout = setTimeout(() => {
            finish(new Error(`安全收起超时 (${ROBOT_STOW_TIMEOUT_MS}ms): ${summarizeOutput(outputTail) || '无输出'}`));
        }, ROBOT_STOW_TIMEOUT_MS);
        const handleLines = createLineHandler((line) => {
            const trimmed = line.trim();
            if (trimmed.includes('[前端常驻模式][完成] 已到达收起姿态')) finish();
            else if (trimmed.includes('[前端常驻模式][失败] 移动到收起姿态失败')) finish(new Error(trimmed));
        });
        const onData = (chunk) => {
            outputTail = (outputTail + chunk.toString()).slice(-4000);
            handleLines(chunk);
        };
        const onClose = (code) => finish(new Error(`worker 在安全收起时退出 code=${code}`));

        worker.stdout.on('data', onData);
        worker.stderr.on('data', onData);
        worker.on('close', onClose);
        worker.stdin.write('__stow_pose__\n', (error) => {
            if (error) finish(new Error(`无法发送安全收起命令: ${error.message}`));
        });
    });
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
