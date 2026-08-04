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
const net = require('node:net');
const { spawn } = require('child_process');
const { WebSocket, WebSocketServer } = require('ws');
const { ChassisClient } = require('./chassis_client');
const { PaymentClient } = require('./payment_client');
const { orderItemsByNearestPickup } = require('./order_routing');

const PORT = Number(process.env.PORT || 3000);
const WS_PORT = Number(process.env.WS_PORT || 3001);
const HOST = process.env.HOST || '0.0.0.0';
const WS_HOST = process.env.WS_HOST || HOST;
const ROBOT_GRASP_MODE = (process.env.ROBOT_GRASP_MODE || 'sim').toLowerCase();
const ROBOT_GRASP_REAL = ['real', 'robot', 'hardware'].includes(ROBOT_GRASP_MODE);
const GRASP_SCRIPT = process.env.GRASP_SCRIPT || path.resolve(__dirname, '../dipan/shelf_grasp.py');
const GRASP_CWD = path.dirname(GRASP_SCRIPT);
const DOBOT_IP = process.env.DOBOT_IP || '192.168.5.1';
const DOBOT_PORT = Number(process.env.DOBOT_PORT || 29999);
const CHASSIS_HOST = process.env.CHASSIS_HOST || process.env.MOVE_ROBOT_HOST || '192.168.10.10';
const CHASSIS_PORT = Number(process.env.CHASSIS_PORT || process.env.MOVE_ROBOT_PORT || 31001);
const CHASSIS_MAP_HTTP_PROTOCOL = String(process.env.CHASSIS_MAP_HTTP_PROTOCOL || 'http').replace(/:$/, '');
const CHASSIS_MAP_HTTP_HOST = process.env.CHASSIS_MAP_HTTP_HOST || CHASSIS_HOST;
const CHASSIS_MAP_HTTP_PORT = Number(process.env.CHASSIS_MAP_HTTP_PORT || 9001);
const CHASSIS_PLACE_MARKER_DEFAULT = process.env.CHASSIS_PLACE_MARKER || 'fangzhi';
const CHASSIS_CHARGE_MARKER = process.env.CHASSIS_CHARGE_MARKER || 'charge_point_1F_35744';
const CHASSIS_NAVIGATION_DEFAULT_ENABLE = /^(1|true|yes|on)$/i.test(process.env.CHASSIS_NAVIGATION_ENABLE || '');
const ROBOT_AUTO_RETURN_CHARGE = !/^(0|false|no|off)$/i.test(process.env.ROBOT_AUTO_RETURN_CHARGE || '1');
const CHASSIS_REQUEST_TIMEOUT_MS = Number(process.env.CHASSIS_REQUEST_TIMEOUT_MS || 10000);
const CHASSIS_NAVIGATION_TIMEOUT_MS = Number(process.env.CHASSIS_NAVIGATION_TIMEOUT_MS || 180000);
const CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M = Number(process.env.CHASSIS_NAVIGATION_DISTANCE_TOLERANCE_M || 0.15);
const CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD = Number(process.env.CHASSIS_NAVIGATION_THETA_TOLERANCE_RAD || 0.25);
const CHASSIS_NAVIGATION_POLL_MS = Number(process.env.CHASSIS_NAVIGATION_POLL_MS || 1000);
const PRODUCTS_DB_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
const ORDERS_DB_FILE = process.env.ORDERS_DB_FILE || path.join(path.dirname(PRODUCTS_DB_FILE), 'orders.json');
const PAYMENT_MODE = String(process.env.PAYMENT_MODE || 'lyorder').trim().toLowerCase();
const REQUIRE_SIGNED_ORDERS = PAYMENT_MODE === 'lyorder';
const PAYMENT_API_BASE_URL = process.env.PAYMENT_API_BASE_URL || '';
const PAYMENT_API_TOKEN = process.env.PAYMENT_API_TOKEN || '';
const PAYMENT_DEVICE_SN = process.env.PAYMENT_DEVICE_SN || 'GRASP-ROBOT-01';
const PAYMENT_REQUEST_TIMEOUT_MS = Number(process.env.PAYMENT_REQUEST_TIMEOUT_MS || 15000);
const PAYMENT_STATUS_SWEEP_INTERVAL_MS = Number(process.env.PAYMENT_STATUS_SWEEP_INTERVAL_MS || 10 * 1000);
const PAYMENT_STATUS_QUERY_INTERVAL_MS = Number(process.env.PAYMENT_STATUS_QUERY_INTERVAL_MS || 5 * 1000);
const ORDER_PAID_START_TIMEOUT_MS = Number(process.env.ORDER_PAID_START_TIMEOUT_MS || 30 * 60 * 1000);
const ALLOW_FREE_ORDERS = /^(1|true|yes|on)$/i.test(process.env.ALLOW_FREE_ORDERS || '') || PAYMENT_MODE === 'free';
const ORDER_STATUS_CALLBACK_URL = String(process.env.ORDER_STATUS_CALLBACK_URL || '').replace(/\/$/, '');
const ORDER_STATUS_CALLBACK_SECRET = process.env.ORDER_STATUS_CALLBACK_SECRET || '';
const ORDER_EXECUTION_REFUND_TIMEOUT_MS = Number(process.env.ORDER_EXECUTION_REFUND_TIMEOUT_MS || 30 * 60 * 1000);
const ORDER_REFUND_SWEEP_INTERVAL_MS = Number(process.env.ORDER_REFUND_SWEEP_INTERVAL_MS || 60 * 1000);
const ORDER_CALLBACK_RETRY_INTERVAL_MS = Number(process.env.ORDER_CALLBACK_RETRY_INTERVAL_MS || 15 * 1000);
const ORDER_CALLBACK_TLS_INSECURE = /^(1|true|yes|on)$/i.test(process.env.ORDER_CALLBACK_TLS_INSECURE || '');
const PAYMENT_TLS_INSECURE = /^(1|true|yes|on)$/i.test(process.env.PAYMENT_TLS_INSECURE || '');
if (ORDER_STATUS_CALLBACK_SECRET && Buffer.byteLength(ORDER_STATUS_CALLBACK_SECRET, 'utf8') < 32) {
    throw new Error('ORDER_STATUS_CALLBACK_SECRET 必须至少 32 字节');
}
if ((ORDER_STATUS_CALLBACK_URL || PAYMENT_MODE === 'lyorder') && !ORDER_STATUS_CALLBACK_SECRET) {
    throw new Error('支付订单服务必须配置 ORDER_STATUS_CALLBACK_SECRET');
}
const paymentClient = new PaymentClient({
    baseUrl: PAYMENT_API_BASE_URL,
    token: PAYMENT_API_TOKEN,
    timeoutMs: PAYMENT_REQUEST_TIMEOUT_MS,
    rejectUnauthorized: !PAYMENT_TLS_INSECURE
});
const ORDER_CALLBACK_OUTBOX_FILE = process.env.ORDER_CALLBACK_OUTBOX_FILE
    || path.join(path.dirname(ORDERS_DB_FILE), 'order_callback_outbox.json');
const CHASSIS_SETTINGS_FILE = process.env.CHASSIS_SETTINGS_FILE || path.join(path.dirname(PRODUCTS_DB_FILE), 'chassis_settings.json');
const CHASSIS_MAP_CACHE_DIR = process.env.CHASSIS_MAP_CACHE_DIR || path.join(path.dirname(PRODUCTS_DB_FILE), 'chassis_map_cache');
const CHASSIS_MAP_SNAPSHOT_FILE = path.join(CHASSIS_MAP_CACHE_DIR, 'current.json');
const PRODUCT_UPLOAD_DIR = process.env.PRODUCT_UPLOAD_DIR || path.join(path.dirname(PRODUCTS_DB_FILE), 'uploads');
const PRODUCT_UPLOAD_MAX_BYTES = Number(process.env.PRODUCT_UPLOAD_MAX_BYTES || 5 * 1024 * 1024);
const ROBOT_WORKER_READY_TIMEOUT_MS = Number(process.env.ROBOT_WORKER_READY_TIMEOUT_MS || 90000);
const ROBOT_GRAB_TIMEOUT_MS = Number(process.env.ROBOT_GRAB_TIMEOUT_MS || 120000);
const ROBOT_STOW_TIMEOUT_MS = Number(process.env.ROBOT_STOW_TIMEOUT_MS || 120000);
const ORDER_READY_CLOSE_DELAY_MS = Number(process.env.ORDER_READY_CLOSE_DELAY_MS || 3000);
const ROBOT_FILTER_PREVIEW_TIMEOUT_MS = Number(process.env.ROBOT_FILTER_PREVIEW_TIMEOUT_MS || 30000);
const ROBOT_MAX_ITEM_QTY = Number(process.env.ROBOT_MAX_ITEM_QTY || 20);
const ROBOT_MAX_ORDER_UNITS = Number(process.env.ROBOT_MAX_ORDER_UNITS || 50);
const ROBOT_SIM_DELAY_SCALE = Math.max(0, Number(process.env.ROBOT_SIM_DELAY_SCALE || 1));
const ROBOT_BROADCAST_LOGS = /^(1|true|yes|on)$/i.test(process.env.ROBOT_BROADCAST_LOGS || '');
const ADMIN_USERNAME = process.env.ADMIN_USERNAME || 'admin';
const ADMIN_PASSWORD = process.env.ADMIN_PASSWORD || '';
const ADMIN_SESSION_SECRET = process.env.ADMIN_SESSION_SECRET || crypto.randomBytes(32).toString('hex');
const ADMIN_SESSION_TTL_MS = Number(process.env.ADMIN_SESSION_TTL_MS || 8 * 60 * 60 * 1000);
const ADMIN_COOKIE_NAME = 'grasp_admin_session';
const ORDER_TOKEN_SECRET = process.env.ORDER_TOKEN_SECRET || ADMIN_SESSION_SECRET;
const ORDER_TOKEN_TTL_MS = Number(process.env.ORDER_TOKEN_TTL_MS || 24 * 60 * 60 * 1000);
if (process.env.ORDER_TOKEN_SECRET && Buffer.byteLength(ORDER_TOKEN_SECRET, 'utf8') < 32) {
    throw new Error('ORDER_TOKEN_SECRET 必须至少 32 字节');
}
if (PAYMENT_MODE === 'lyorder' && !ROBOT_GRASP_REAL) {
    if (!PAYMENT_API_BASE_URL) throw new Error('云端支付服务必须配置 PAYMENT_API_BASE_URL');
    if (!process.env.ORDER_TOKEN_SECRET) throw new Error('云端支付服务必须单独配置 ORDER_TOKEN_SECRET');
    if (!ADMIN_PASSWORD || ['123456', 'admin123', 'change-me'].includes(ADMIN_PASSWORD)) {
        throw new Error('云端支付服务禁止使用默认管理员密码');
    }
    if (ALLOW_FREE_ORDERS) throw new Error('正式支付服务禁止启用 ALLOW_FREE_ORDERS');
}
const loginAttempts = new Map();
const requestRateLimits = new Map();
const paymentReconcilePromises = new Map();
let activeOrderId = null;
let robotWorker = null;
let robotWorkerReady = false;
let robotWorkerStarting = null;
let robotStowPromise = null;
let robotPoweroffPromise = null;
let robotShutdownPromise = null;
let robotEmergencyPromise = null;
let robotFailureRecoveryPromise = null;
let robotPoweredOff = false;
let robotEmergencyStopped = false;
let robotFilterPreviewPromise = null;
let chassisNavigationPromise = null;
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
    const direct = String(req.socket.remoteAddress || '').replace(/^::ffff:/, '').trim();
    const trustedProxy = direct === '127.0.0.1' || direct === '::1';
    return String(trustedProxy && req.headers['x-real-ip'] ? req.headers['x-real-ip'] : direct).trim();
}

function enforceRateLimit(req, res, bucket, limit, windowMs) {
    const now = Date.now();
    const key = `${bucket}:${clientIp(req)}`;
    let state = requestRateLimits.get(key);
    if (!state || now >= state.resetAt) state = { count: 0, resetAt: now + windowMs };
    state.count += 1;
    requestRateLimits.set(key, state);
    if (state.count <= limit) return true;
    const retryAfter = Math.max(1, Math.ceil((state.resetAt - now) / 1000));
    sendJSON(res, { code: 1, message: '请求过于频繁，请稍后重试' }, 429, { 'Retry-After': String(retryAfter) });
    return false;
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
    if (!fs.existsSync(PRODUCTS_DB_FILE)) return { products: [], completedOrders: {}, reservedOrders: {} };
    try {
        const data = JSON.parse(fs.readFileSync(PRODUCTS_DB_FILE, 'utf8'));
        return {
            products: Array.isArray(data.products) ? data.products.map((p, idx) => normalizeProduct(p, idx + 1)) : [],
            completedOrders: data.completedOrders && typeof data.completedOrders === 'object' ? data.completedOrders : {},
            reservedOrders: data.reservedOrders && typeof data.reservedOrders === 'object' ? data.reservedOrders : {}
        };
    } catch (e) {
        log(`⚠️ 商品数据读取失败: ${e.message}`);
        return { products: [], completedOrders: {}, reservedOrders: {} };
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

function readOrderStore() {
    if (!fs.existsSync(ORDERS_DB_FILE)) return { orders: [] };
    try {
        const data = JSON.parse(fs.readFileSync(ORDERS_DB_FILE, 'utf8'));
        return { orders: Array.isArray(data.orders) ? data.orders : [] };
    } catch (error) {
        log(`⚠️ 订单数据读取失败: ${error.message}`);
        return { orders: [] };
    }
}

function writeOrderStore(store) {
    fs.mkdirSync(path.dirname(ORDERS_DB_FILE), { recursive: true });
    const temporary = `${ORDERS_DB_FILE}.${process.pid}.${Date.now()}.tmp`;
    fs.writeFileSync(temporary, JSON.stringify({ orders: store.orders || [] }, null, 2), 'utf8');
    fs.renameSync(temporary, ORDERS_DB_FILE);
}

function orderById(orderId) {
    return readOrderStore().orders.find((item) => item.orderId === orderId) || null;
}

function saveOrder(order) {
    const store = readOrderStore();
    const index = store.orders.findIndex((item) => item.orderId === order.orderId);
    const savedOrder = {
        ...(index >= 0 ? store.orders[index] : {}),
        ...order,
        updatedAt: new Date().toISOString()
    };
    Object.assign(order, savedOrder);
    if (index >= 0) store.orders[index] = savedOrder;
    else store.orders.push(savedOrder);
    store.orders.sort((a, b) => String(b.createdAt).localeCompare(String(a.createdAt)));
    if (store.orders.length > 10000) store.orders.length = 10000;
    writeOrderStore(store);
    return order;
}

function migrateUncreatedPaymentOrders() {
    const store = readOrderStore();
    let changed = 0;
    const migratedAt = new Date().toISOString();
    for (const order of store.orders) {
        const legacyUncreated = order.paymentMode === 'lyorder'
            && !order.externalOrderId
            && Array.isArray(order.items)
            && order.items.every((item) => !Object.prototype.hasOwnProperty.call(item, 'productStock'));
        if (legacyUncreated && order.source !== 'test') {
            order.source = 'test';
            order.updatedAt = migratedAt;
            changed += 1;
        }
        if (order.paymentMode !== 'lyorder' || order.paymentStatus !== 'PENDING_PAYMENT' || order.externalOrderId) continue;
        order.paymentStatus = 'PAYMENT_FAILED';
        order.status = 'PAYMENT_FAILED';
        order.failureReason = order.failureReason || '支付业务订单未创建成功，请重新下单';
        order.updatedAt = migratedAt;
        changed += 1;
    }
    if (changed) {
        writeOrderStore(store);
        log(`已将 ${changed} 条未创建支付业务订单的历史记录标记为支付创建失败`);
    }
}

function readOrderCallbackOutbox() {
    if (!fs.existsSync(ORDER_CALLBACK_OUTBOX_FILE)) return { events: [] };
    try {
        const data = JSON.parse(fs.readFileSync(ORDER_CALLBACK_OUTBOX_FILE, 'utf8'));
        return { events: Array.isArray(data.events) ? data.events : [] };
    } catch (error) {
        log(`⚠️ 订单完成回调队列读取失败: ${error.message}`);
        return { events: [] };
    }
}

function writeOrderCallbackOutbox(store) {
    fs.mkdirSync(path.dirname(ORDER_CALLBACK_OUTBOX_FILE), { recursive: true });
    const temporary = `${ORDER_CALLBACK_OUTBOX_FILE}.${process.pid}.${Date.now()}.tmp`;
    fs.writeFileSync(temporary, JSON.stringify({ events: store.events || [] }, null, 2), { encoding: 'utf8', mode: 0o600 });
    fs.renameSync(temporary, ORDER_CALLBACK_OUTBOX_FILE);
}

function queueOrderCompletedCallback(orderId, completionToken) {
    if (!ORDER_STATUS_CALLBACK_URL || !completionToken) return false;
    const store = readOrderCallbackOutbox();
    if (store.events.some((event) => event.type === 'completed' && event.orderId === orderId)) return false;
    store.events.push({
        type: 'completed',
        orderId,
        completionToken,
        placedAt: new Date().toISOString(),
        createdAt: new Date().toISOString(),
        attempts: 0,
        nextAttemptAt: 0
    });
    writeOrderCallbackOutbox(store);
    return true;
}

function updateQueuedCallback(orderId, update) {
    const store = readOrderCallbackOutbox();
    const index = store.events.findIndex((event) => event.type === 'completed' && event.orderId === orderId);
    if (index < 0) return;
    if (update === null) store.events.splice(index, 1);
    else store.events[index] = { ...store.events[index], ...update };
    writeOrderCallbackOutbox(store);
}

function orderStatusLabel(status) {
    return ({
        PENDING_PAYMENT: '待支付',
        PAID: '已支付',
        PROCESSING: '抓取中',
        PLACED: '已放置待确认',
        COMPLETED: '已完成',
        PARTIAL_REFUNDING: '部分交付，退款中',
        PARTIALLY_COMPLETED: '部分交付',
        FAILED: '失败',
        REFUNDING: '退款中',
        REFUNDED: '已退款',
        PAYMENT_FAILED: '支付创建失败',
        CLOSED: '已关闭'
    })[status] || status || '未知';
}

function paymentStatusLabel(status) {
    return ({
        PENDING_PAYMENT: '待支付',
        PAID: '已支付',
        PARTIAL_REFUNDING: '部分退款中',
        PARTIALLY_REFUNDED: '部分退款',
        REFUNDING: '退款中',
        REFUNDED: '已退款',
        PAYMENT_FAILED: '支付失败',
        CLOSED: '已关闭'
    })[status] || status || '未知';
}

function publicOrder(order) {
    const partialRefunded = order.paymentStatus === 'PARTIALLY_REFUNDED'
        || (order.refundStatus === 'SUCCESS' && order.refundIsPartial === true)
        || Boolean(order.partiallyRefundedAt);
    const fullyRefunded = order.paymentStatus === 'REFUNDED'
        || (order.refundStatus === 'SUCCESS' && !partialRefunded)
        || Boolean(order.refundedAt && !order.refundIsPartial);
    const publicPaymentStatus = fullyRefunded
        ? 'REFUNDED'
        : partialRefunded
            ? 'PARTIALLY_REFUNDED'
            : (order.paymentStatus || 'PENDING_PAYMENT');
    const publicStatus = fullyRefunded
        ? 'REFUNDED'
        : partialRefunded
            ? 'PARTIALLY_COMPLETED'
            : (order.status || 'PENDING_PAYMENT');
    const publicPaymentStatusLabel = order.refundStatus === 'FAIL' && publicPaymentStatus === 'PAID'
        ? '退款失败（仍已支付）'
        : paymentStatusLabel(publicPaymentStatus);
    return {
        orderId: order.orderId,
        externalOrderId: order.externalOrderId || null,
        items: order.items || [],
        totalAmount: Number(order.totalAmount || 0),
        paymentMode: order.paymentMode || PAYMENT_MODE,
        paymentStatus: publicPaymentStatus,
        paymentStatusLabel: publicPaymentStatusLabel,
        status: publicStatus,
        statusLabel: orderStatusLabel(publicStatus),
        refundId: order.refundId || null,
        refundStatus: order.refundStatus || null,
        refundReason: order.refundReason || null,
        refundTrigger: order.refundTrigger || null,
        refundAmount: Number(order.refundAmount || 0),
        deliveredItems: order.deliveredItems || [],
        deliveredUnits: Number(order.deliveredUnits || 0),
        deliveredAmount: Number(order.deliveredAmount || 0),
        failureReason: order.failureReason || null,
        createdAt: order.createdAt,
        paidAt: order.paidAt || null,
        placedAt: order.placedAt || null,
        completedAt: order.completedAt || null,
        refundedAt: order.refundedAt || null,
        updatedAt: order.updatedAt || null,
        source: order.source || 'web'
    };
}

function paymentStatusMessage(order) {
    if (order.paymentStatus === 'REFUNDED') {
        return order.refundReason ? `订单已退款：${order.refundReason}` : '订单已退款';
    }
    if (order.paymentStatus === 'PARTIALLY_REFUNDED') {
        return `订单部分交付，未交付商品已退款 ¥${Number(order.refundAmount || 0).toFixed(2)}`;
    }
    if (order.paymentStatus === 'PARTIAL_REFUNDING') return '订单部分交付，未交付商品正在退款';
    if (order.paymentStatus === 'REFUNDING') return '订单正在退款，请勿重复支付';
    if (order.paymentStatus === 'CLOSED') return '支付订单已关闭，请重新下单';
    if (order.paymentStatus === 'PAYMENT_FAILED') {
        return order.failureReason || '支付订单创建失败，请重新下单';
    }
    if (order.status === 'FAILED') {
        return order.refundStatus === 'FAIL'
            ? '订单处理失败，自动退款暂未成功，请联系管理员处理'
            : (order.failureReason || '订单处理失败，请重新下单');
    }
    if (order.status === 'PARTIALLY_COMPLETED' && order.refundStatus === 'FAIL') {
        return '订单已部分交付，未交付商品退款暂未成功，请联系管理员处理';
    }
    return null;
}

async function createPaymentQrCode(codeUrl) {
    const QRCode = require('qrcode');
    return QRCode.toBuffer(codeUrl, {
        type: 'image/png',
        errorCorrectionLevel: 'M',
        margin: 2,
        width: 320,
        color: { dark: '#000000', light: '#FFFFFF' }
    });
}

function setOrderStatus(order, status, paymentStatus = order.paymentStatus) {
    order.status = status;
    order.paymentStatus = paymentStatus;
    order.updatedAt = new Date().toISOString();
    return saveOrder(order);
}

function setOrderDeliveredItems(order, deliveredItems) {
    const orderedById = new Map((order.items || []).map((item) => [Number(item.id), item]));
    const deliveredById = new Map();
    for (const raw of deliveredItems || []) {
        const id = Number(raw && raw.id);
        const qty = Number(raw && raw.qty);
        const ordered = orderedById.get(id);
        if (!ordered || !Number.isInteger(qty) || qty < 1) throw new Error('已交付商品记录无效');
        const nextQty = (deliveredById.get(id) || 0) + qty;
        if (nextQty > Number(ordered.qty)) throw new Error(`商品 ${ordered.name || id} 的交付数量超过下单数量`);
        deliveredById.set(id, nextQty);
    }

    order.deliveredItems = [...deliveredById.entries()].map(([id, qty]) => {
        const ordered = orderedById.get(id);
        const unitPrice = Number(ordered.unitPrice || 0);
        return {
            id,
            name: ordered.name || '',
            qty,
            unitPrice,
            lineTotal: Number((unitPrice * qty).toFixed(2))
        };
    });
    order.deliveredUnits = order.deliveredItems.reduce((sum, item) => sum + item.qty, 0);
    order.deliveredAmount = Number(order.deliveredItems.reduce((sum, item) => sum + item.lineTotal, 0).toFixed(2));
    order.lastDeliveredAt = order.deliveredUnits > 0 ? new Date().toISOString() : null;
    return saveOrder(order);
}

function recordOrderUnitDelivered(orderId, item) {
    const order = orderById(orderId);
    if (!order) return null;
    const delivered = Array.isArray(order.deliveredItems)
        ? order.deliveredItems.map((entry) => ({ id: entry.id, qty: entry.qty }))
        : [];
    const existing = delivered.find((entry) => Number(entry.id) === Number(item.id));
    if (existing) existing.qty += 1;
    else delivered.push({ id: Number(item.id), qty: 1 });
    return setOrderDeliveredItems(order, delivered);
}

function orderDeliveryBreakdown(order) {
    const deliveredById = new Map((order.deliveredItems || []).map((item) => [Number(item.id), Number(item.qty)]));
    const undeliveredItems = [];
    for (const item of order.items || []) {
        const deliveredQty = Math.min(Number(item.qty), Math.max(0, deliveredById.get(Number(item.id)) || 0));
        const qty = Number(item.qty) - deliveredQty;
        if (qty <= 0) continue;
        const unitPrice = Number(item.unitPrice || 0);
        undeliveredItems.push({
            id: Number(item.id),
            name: item.name || '',
            qty,
            unitPrice,
            lineTotal: Number((unitPrice * qty).toFixed(2))
        });
    }
    return {
        deliveredUnits: Number(order.deliveredUnits || 0),
        deliveredAmount: Number(order.deliveredAmount || 0),
        undeliveredItems,
        refundAmount: Number(undeliveredItems.reduce((sum, item) => sum + item.lineTotal, 0).toFixed(2))
    };
}

function reserveOrderInventory(order) {
    const store = readProductStore();
    const existing = store.reservedOrders[order.orderId];
    if (existing) {
        if (existing.releasedAt) throw new Error('订单库存预占已经释放');
        order.inventoryReservedAt = order.inventoryReservedAt || existing.reservedAt;
        return existing;
    }
    const items = normalizeInventoryItems(order.items || []);
    const changes = [];
    for (const item of items) {
        const product = store.products.find((candidate) => Number(candidate.id) === item.id);
        if (!product || Number(product.status) !== 1) throw new Error(`商品不可用: ${item.name || item.id}`);
        if (Number(product.stock) < item.qty) throw new Error(`${product.name} 库存不足，当前库存 ${product.stock}`);
        changes.push({ product, item });
    }
    const reservedAt = new Date().toISOString();
    const updated = [];
    for (const { product, item } of changes) {
        product.stock = Number(product.stock) - item.qty;
        updated.push({ id: product.id, name: product.name, qty: item.qty, stock: product.stock });
    }
    const reservation = { reservedAt, items, updated, completedAt: null, releasedAt: null };
    store.reservedOrders[order.orderId] = reservation;
    writeProductStore(store);
    order.inventoryReservedAt = reservedAt;
    return reservation;
}

function markOrderPaid(order, source = 'payment') {
    if (!order.paidAt) order.paidAt = new Date().toISOString();
    order.paidSource = source;
    try {
        reserveOrderInventory(order);
        order.inventoryReservationError = null;
    } catch (error) {
        order.inventoryReservationError = error.message;
        setOrderStatus(order, 'PAID', 'PAID');
        throw error;
    }
    return setOrderStatus(order, 'PAID', 'PAID');
}

function orderCallbackSignature(timestamp, orderId, completionToken, reason) {
    return crypto.createHmac('sha256', ORDER_STATUS_CALLBACK_SECRET)
        .update(`${timestamp}:${orderId}:${completionToken}:${reason}`)
        .digest('base64url');
}

function verifyOrderCallback(req, orderId, completionToken, reason) {
    if (!ORDER_STATUS_CALLBACK_SECRET) return false;
    const timestamp = String(req.headers['x-grasp-timestamp'] || '');
    const signature = String(req.headers['x-grasp-signature'] || '');
    if (!/^\d{10,16}$/.test(timestamp) || Math.abs(Date.now() - Number(timestamp)) > 5 * 60 * 1000) return false;
    return safeEqual(signature, orderCallbackSignature(timestamp, orderId, completionToken, reason));
}

function completePaidOrder(orderId, completionToken) {
    const tokenOrder = readOrderToken(completionToken, { allowExpired: true });
    if (!tokenOrder || tokenOrder.orderId !== orderId) {
        const error = new Error('订单凭证无效');
        error.statusCode = 401;
        throw error;
    }
    const order = orderById(orderId);
    if (!order) {
        const error = new Error('订单不存在');
        error.statusCode = 404;
        throw error;
    }
    if (order.paymentStatus !== 'PAID') {
        const error = new Error(`订单支付状态不允许完成: ${order.paymentStatus}`);
        error.statusCode = 409;
        throw error;
    }

    if (!order.placedAt) order.placedAt = new Date().toISOString();
    order.completionReportedAt = order.completionReportedAt || new Date().toISOString();
    if (order.status !== 'COMPLETED') setOrderStatus(order, 'PLACED', 'PAID');

    const store = readProductStore();
    const existingCompletion = store.completedOrders[orderId];
    if (existingCompletion) {
        if (existingCompletion.inventoryRestoredAt) {
            const error = new Error('该订单库存已经随退款恢复，不能重新完成');
            error.statusCode = 409;
            throw error;
        }
        order.completedAt = order.completedAt
            || new Date(Number(existingCompletion.completedAt) || Date.now()).toISOString();
        setOrderStatus(order, 'COMPLETED', 'PAID');
        return { alreadyCompleted: true, updated: existingCompletion.updated || [] };
    }

    const reservation = store.reservedOrders[orderId];
    let updated;
    if (reservation) {
        if (reservation.releasedAt) throw new Error('订单库存预占已释放，不能完成订单');
        updated = Array.isArray(reservation.updated) ? reservation.updated : [];
        reservation.completedAt = reservation.completedAt || new Date().toISOString();
    } else {
        updated = [];
        for (const item of normalizeInventoryItems(order.items || [])) {
            const product = store.products.find((candidate) => Number(candidate.id) === item.id);
            if (!product) throw new Error(`商品不存在: ${item.name || item.id}`);
            if (Number(product.stock) < item.qty) throw new Error(`${product.name} 库存不足，当前库存 ${product.stock}`);
            product.stock = Number(product.stock) - item.qty;
            updated.push({ id: product.id, name: product.name, qty: item.qty, stock: product.stock });
        }
    }
    store.completedOrders[orderId] = { completedAt: Date.now(), updated, inventoryRestoredAt: null };
    const orderIds = Object.keys(store.completedOrders);
    for (const oldOrderId of orderIds.slice(0, Math.max(0, orderIds.length - 1000))) delete store.completedOrders[oldOrderId];
    writeProductStore(store);
    order.completedAt = new Date().toISOString();
    setOrderStatus(order, 'COMPLETED', 'PAID');
    log(`📉 订单 ${orderId} 已扣减库存: ${updated.map((item) => `${item.name} x${item.qty}`).join(', ')}`);
    return { alreadyCompleted: false, updated };
}

function restoredInventoryQuantities(reservation) {
    const restored = new Map();
    for (const entry of Object.values(reservation.partialRestores || {})) {
        for (const item of entry.items || []) {
            const id = Number(item.id);
            restored.set(id, (restored.get(id) || 0) + Number(item.qty || 0));
        }
    }
    return restored;
}

function restoreOrderInventoryItems(order, items, restoreKey) {
    const store = readProductStore();
    const reservation = store.reservedOrders[order.orderId];
    if (!reservation) throw new Error('订单缺少库存预占记录，拒绝恢复部分库存');
    reservation.partialRestores = reservation.partialRestores || {};
    if (reservation.partialRestores[restoreKey]) return false;

    const requested = normalizeInventoryItems(items);
    const orderedById = new Map(normalizeInventoryItems(reservation.items || order.items || []).map((item) => [item.id, item.qty]));
    const restored = restoredInventoryQuantities(reservation);
    for (const item of requested) {
        const remaining = Number(orderedById.get(item.id) || 0) - Number(restored.get(item.id) || 0);
        if (item.qty > remaining) throw new Error(`商品 ${item.name || item.id} 的恢复库存数量超过未恢复数量`);
    }

    const restoredAt = new Date().toISOString();
    for (const item of requested) {
        const product = store.products.find((candidate) => Number(candidate.id) === item.id);
        if (!product) throw new Error(`恢复库存时商品不存在: ${item.name || item.id}`);
        product.stock = Number(product.stock) + item.qty;
    }
    reservation.partialRestores[restoreKey] = { restoredAt, items: requested };
    order.partialStockRestoredAt = restoredAt;
    writeProductStore(store);
    return true;
}

function restoreCompletedOrderInventory(order) {
    if (order.stockRestoredAt) return false;
    const store = readProductStore();
    const reservation = store.reservedOrders[order.orderId];
    const completion = store.completedOrders[order.orderId];
    if (reservation) {
        if (reservation.releasedAt) {
            order.stockRestoredAt = reservation.releasedAt;
            return false;
        }
        const releasedAt = new Date().toISOString();
        const alreadyRestored = restoredInventoryQuantities(reservation);
        for (const item of normalizeInventoryItems(reservation.items || order.items || [])) {
            const qty = Math.max(0, item.qty - Number(alreadyRestored.get(item.id) || 0));
            if (qty <= 0) continue;
            const product = store.products.find((candidate) => Number(candidate.id) === item.id);
            if (product) product.stock = Number(product.stock) + qty;
        }
        reservation.releasedAt = releasedAt;
        if (completion) completion.inventoryRestoredAt = releasedAt;
        writeProductStore(store);
        order.stockRestoredAt = releasedAt;
        return true;
    }
    if (!completion) return false;
    if (completion.inventoryRestoredAt) {
        order.stockRestoredAt = completion.inventoryRestoredAt;
        return false;
    }
    for (const item of normalizeInventoryItems(order.items || [])) {
        const product = store.products.find((candidate) => Number(candidate.id) === item.id);
        if (product) product.stock = Number(product.stock) + item.qty;
    }
    completion.inventoryRestoredAt = new Date().toISOString();
    writeProductStore(store);
    order.stockRestoredAt = completion.inventoryRestoredAt;
    return true;
}

async function refundOrder(order, reason, trigger = 'admin', {
    resume = false,
    refundItems = null,
    refundAmount = null
} = {}) {
    if (order.status === 'REFUNDED' || order.paymentStatus === 'REFUNDED') return order;
    if (order.paymentStatus === 'PARTIALLY_REFUNDED') return order;
    const refunding = ['REFUNDING', 'PARTIAL_REFUNDING'].includes(order.status)
        || ['REFUNDING', 'PARTIAL_REFUNDING'].includes(order.paymentStatus);
    if (refunding && !resume) return order;
    if (!['PAID', 'REFUNDING', 'PARTIAL_REFUNDING'].includes(order.paymentStatus)) {
        throw new Error('订单尚未支付，不能退款');
    }

    const amount = Number(resume && order.refundAmount ? order.refundAmount : refundAmount);
    const totalAmount = Number(order.totalAmount || 0);
    const partial = Boolean(
        (resume && order.refundIsPartial)
        || (Number.isFinite(amount) && amount > 0 && amount < totalAmount - 0.001)
    );
    const selectedItems = partial
        ? normalizeInventoryItems(resume && order.refundItems ? order.refundItems : refundItems)
        : null;
    const effectiveAmount = partial ? Number(amount.toFixed(2)) : totalAmount;
    const previousRefundId = order.refundId;
    const previousRefundRequestId = order.refundRequestId;

    order.refundReason = reason;
    order.refundTrigger = trigger;
    order.refundAmount = effectiveAmount;
    order.refundIsPartial = partial;
    order.refundItems = selectedItems;
    order.refundRequestedAt = order.refundRequestedAt || new Date().toISOString();
    order.refundRequestId = order.refundRequestId || `GRASP-REFUND-${crypto.createHash('sha256')
        .update(`${order.orderId}:${effectiveAmount.toFixed(2)}`)
        .digest('hex').slice(0, 24)}`;
    order.refundAttemptCount = Number(order.refundAttemptCount || 0) + 1;
    order.refundLastAttemptAt = new Date().toISOString();
    setOrderStatus(order, partial ? 'PARTIAL_REFUNDING' : 'REFUNDING', partial ? 'PARTIAL_REFUNDING' : 'REFUNDING');

    let refund;
    try {
        if (order.paymentMode === 'free') {
            refund = { refundId: `FREE-${Date.now().toString(36).toUpperCase()}`, refundStatus: 'SUCCESS' };
        } else {
            if (!order.externalOrderId) throw new Error('订单没有外部支付订单号');
            refund = await paymentClient.refund(order.externalOrderId, reason, order.refundRequestId, {
                refundAmount: partial ? effectiveAmount : null,
                totalAmount
            });
        }
    } catch (error) {
        // LyOrder marks an accepted PROCESSING refund as orderStatus=3 immediately.
        // A later idempotent retry then says the order is no longer payable; that
        // response means the original refund was accepted, not that it was undone.
        const alreadyAccepted = Boolean(previousRefundId && previousRefundRequestId)
            && /仅已支付订单可退款/.test(String(error.message || error));
        if (alreadyAccepted) {
            order.refundId = previousRefundId;
            order.refundStatus = 'SUCCESS';
            if (partial) {
                order.partiallyRefundedAt = order.partiallyRefundedAt || new Date().toISOString();
                restoreOrderInventoryItems(order, selectedItems, order.refundRequestId);
                setOrderStatus(order, 'PARTIALLY_COMPLETED', 'PARTIALLY_REFUNDED');
            } else {
                order.refundedAt = order.refundedAt || new Date().toISOString();
                restoreCompletedOrderInventory(order);
                setOrderStatus(order, 'REFUNDED', 'REFUNDED');
            }
            return order;
        }
        order.refundStatus = 'FAIL';
        order.refundReason = `${reason} (${error.message})`;
        setOrderStatus(order, partial ? 'PARTIALLY_COMPLETED' : 'FAILED', 'PAID');
        throw error;
    }

    order.refundId = refund.refundId || order.refundId || null;
    order.refundStatus = refund.refundStatus;
    if (refund.refundStatus === 'SUCCESS') {
        if (partial) {
            order.partiallyRefundedAt = new Date().toISOString();
            restoreOrderInventoryItems(order, selectedItems, order.refundRequestId);
            setOrderStatus(order, 'PARTIALLY_COMPLETED', 'PARTIALLY_REFUNDED');
        } else {
            order.refundedAt = new Date().toISOString();
            restoreCompletedOrderInventory(order);
            setOrderStatus(order, 'REFUNDED', 'REFUNDED');
        }
    } else {
        setOrderStatus(order, partial ? 'PARTIAL_REFUNDING' : 'REFUNDING', partial ? 'PARTIAL_REFUNDING' : 'REFUNDING');
    }
    return order;
}

function validatePaymentResult(order, payment) {
    if (payment.tradeState !== 'SUCCESS') return;
    if (payment.totalAmount !== null && Math.abs(payment.totalAmount - Number(order.totalAmount)) > 0.001) {
        throw new Error(`支付金额不一致: 支付系统 ¥${payment.totalAmount.toFixed(2)}，本地 ¥${Number(order.totalAmount).toFixed(2)}`);
    }
    if (payment.currency && payment.currency !== 'CNY') throw new Error(`支付币种不支持: ${payment.currency}`);
}

async function reconcileOrderPayment(order, source = 'server-query') {
    const existing = paymentReconcilePromises.get(order.orderId);
    if (existing) return existing;
    const operation = (async () => {
        let latest = orderById(order.orderId) || order;
        if (latest.paymentMode !== 'lyorder' || ['REFUNDED', 'REFUNDING'].includes(latest.paymentStatus)) return latest;
        if (!latest.externalOrderId) throw new Error('支付业务订单尚未创建');
        const payment = await paymentClient.query(latest.externalOrderId);
        latest = orderById(order.orderId) || latest;
        if (latest.paymentMode !== 'lyorder' || ['REFUNDED', 'REFUNDING'].includes(latest.paymentStatus)) return latest;
        latest.paymentTradeState = payment.tradeState;
        latest.paymentTradeStateDesc = payment.tradeStateDesc;
        latest.paymentLastCheckedAt = new Date().toISOString();
        validatePaymentResult(latest, payment);
        if (payment.tradeState === 'SUCCESS' && latest.paymentStatus !== 'PAID') {
            try {
                markOrderPaid(latest, source);
            } catch (error) {
                latest.failureReason = `付款成功但库存预占失败: ${error.message}`;
                latest.failedAt = new Date().toISOString();
                saveOrder(latest);
                await refundOrder(latest, latest.failureReason, 'inventory-reservation');
            }
        } else if (payment.tradeState === 'CLOSED' && latest.paymentStatus === 'PENDING_PAYMENT') {
            setOrderStatus(latest, 'CLOSED', 'CLOSED');
        } else {
            saveOrder(latest);
        }
        if (latest.cancelRequestedAt && latest.paymentStatus === 'PAID' && !latest.executionStartedAt) {
            await refundOrder(latest, '用户取消支付页面，订单尚未开始执行', 'payment-cancel');
        }
        return orderById(latest.orderId) || latest;
    })();
    paymentReconcilePromises.set(order.orderId, operation);
    try { return await operation; }
    finally { paymentReconcilePromises.delete(order.orderId); }
}

function postOrderStatusCallback(pathname, body, headers = {}) {
    return new Promise((resolve, reject) => {
        const target = new URL(pathname, `${ORDER_STATUS_CALLBACK_URL}/`);
        const transport = target.protocol === 'https:' ? https : http;
        const payload = Buffer.from(JSON.stringify(body));
        const request = transport.request({
            protocol: target.protocol,
            hostname: target.hostname,
            port: target.port || undefined,
            path: `${target.pathname}${target.search}`,
            method: 'POST',
            timeout: PAYMENT_REQUEST_TIMEOUT_MS,
            rejectUnauthorized: !ORDER_CALLBACK_TLS_INSECURE,
            headers: { 'Content-Type': 'application/json', 'Content-Length': payload.length, ...headers }
        }, (response) => {
            const chunks = [];
            response.on('data', (chunk) => chunks.push(chunk));
            response.once('end', () => {
                const text = Buffer.concat(chunks).toString('utf8');
                let data;
                try { data = text ? JSON.parse(text) : {}; }
                catch { reject(new Error(`自动退款回调返回非 JSON (${response.statusCode})`)); return; }
                if (response.statusCode < 200 || response.statusCode >= 300 || data.code !== 0) {
                    reject(new Error(data.message || `自动退款回调失败 (${response.statusCode})`));
                    return;
                }
                resolve(data);
            });
        });
        request.once('timeout', () => request.destroy(new Error('自动退款回调超时')));
        request.once('error', reject);
        request.end(payload);
    });
}

async function refundFailedExecution(orderId, completionToken, reason, deliveredItems = []) {
    const localOrder = orderById(orderId);
    if (localOrder && localOrder.paymentStatus === 'PAID') {
        if (Array.isArray(deliveredItems) && deliveredItems.length > 0) {
            setOrderDeliveredItems(localOrder, deliveredItems);
        }
        localOrder.failureReason = reason;
        localOrder.failedAt = new Date().toISOString();
        saveOrder(localOrder);
        const delivery = orderDeliveryBreakdown(localOrder);
        if (delivery.deliveredUnits > 0 && delivery.refundAmount > 0) {
            return publicOrder(await refundOrder(localOrder, reason, 'robot-failure', {
                refundItems: delivery.undeliveredItems,
                refundAmount: delivery.refundAmount
            }));
        }
        return publicOrder(await refundOrder(localOrder, reason, 'robot-failure'));
    }
    if (!ORDER_STATUS_CALLBACK_URL || !ORDER_STATUS_CALLBACK_SECRET) {
        throw new Error('未配置 ORDER_STATUS_CALLBACK_URL/SECRET，无法通知云端自动退款');
    }
    const timestamp = String(Date.now());
    const body = { orderId, completionToken, reason, deliveredItems };
    const signature = orderCallbackSignature(timestamp, orderId, completionToken, reason);
    const response = await postOrderStatusCallback(`/api/orders/${encodeURIComponent(orderId)}/failure`, body, {
        'X-Grasp-Timestamp': timestamp,
        'X-Grasp-Signature': signature
    });
    return response.data;
}

async function notifyRemoteOrderCompleted(orderId, completionToken) {
    if (!ORDER_STATUS_CALLBACK_URL || !ORDER_STATUS_CALLBACK_SECRET || !completionToken) return null;
    const timestamp = String(Date.now());
    const reason = 'completed';
    const signature = orderCallbackSignature(timestamp, orderId, completionToken, reason);
    return postOrderStatusCallback(
        `/api/orders/${encodeURIComponent(orderId)}/complete`,
        { orderId, completionToken },
        { 'X-Grasp-Timestamp': timestamp, 'X-Grasp-Signature': signature }
    );
}

let callbackOutboxRunning = false;
async function flushOrderCallbackOutbox({ force = false } = {}) {
    if (callbackOutboxRunning || !ORDER_STATUS_CALLBACK_URL || !ORDER_STATUS_CALLBACK_SECRET) return;
    callbackOutboxRunning = true;
    try {
        const now = Date.now();
        const events = readOrderCallbackOutbox().events.filter((event) => (
            event.type === 'completed' && (force || Number(event.nextAttemptAt || 0) <= now)
        ));
        for (const event of events) {
            try {
                await notifyRemoteOrderCompleted(event.orderId, event.completionToken);
                updateQueuedCallback(event.orderId, null);
                log(`✅ 订单 ${event.orderId} 完成回调已获云端确认`);
            } catch (error) {
                const attempts = Number(event.attempts || 0) + 1;
                const retryDelay = Math.min(5 * 60 * 1000, ORDER_CALLBACK_RETRY_INTERVAL_MS * (2 ** Math.min(attempts - 1, 5)));
                updateQueuedCallback(event.orderId, {
                    attempts,
                    lastAttemptAt: new Date().toISOString(),
                    lastError: String(error.message || error).slice(0, 500),
                    nextAttemptAt: Date.now() + retryDelay
                });
                log(`⚠️ 订单 ${event.orderId} 完成回调失败，已落盘并将在 ${Math.round(retryDelay / 1000)} 秒后重试: ${error.message}`);
            }
        }
    } finally {
        callbackOutboxRunning = false;
    }
}

let refundSweepRunning = false;
async function refundTimedOutOrders() {
    if (refundSweepRunning || PAYMENT_MODE !== 'lyorder') return;
    refundSweepRunning = true;
    try {
        const now = Date.now();
        const pendingPayments = readOrderStore().orders.filter((order) => (
            order.paymentMode === 'lyorder'
            && order.paymentStatus === 'PENDING_PAYMENT'
            && order.externalOrderId
            && (!Number.isFinite(Date.parse(order.paymentLastCheckedAt))
                || now - Date.parse(order.paymentLastCheckedAt) >= PAYMENT_STATUS_QUERY_INTERVAL_MS)
        )).slice(0, 50);
        for (const order of pendingPayments) {
            try {
                await reconcileOrderPayment(order, 'payment-sweep');
            } catch (error) {
                const failed = orderById(order.orderId) || order;
                failed.paymentLastCheckedAt = new Date().toISOString();
                failed.paymentCheckError = String(error.message || error).slice(0, 500);
                saveOrder(failed);
                log(`⚠️ 订单 ${order.orderId} 服务端查单失败: ${error.message}`);
            }
        }
        const paidNotStarted = readOrderStore().orders.filter((order) => (
            order.status === 'PAID'
            && order.paymentStatus === 'PAID'
            && !order.executionStartedAt
            && !order.placedAt
            && Number.isFinite(Date.parse(order.paidAt))
            && now - Date.parse(order.paidAt) >= ORDER_PAID_START_TIMEOUT_MS
        ));
        for (const order of paidNotStarted) {
            try {
                const reason = `付款后超过 ${Math.round(ORDER_PAID_START_TIMEOUT_MS / 60000)} 分钟仍未开始执行`;
                await refundOrder(order, reason, 'paid-not-started-timeout');
                log(`💸 订单 ${order.orderId} 付款后未执行，已触发自动退款`);
            } catch (error) {
                log(`❌ 订单 ${order.orderId} 未执行自动退款失败: ${error.message}`);
            }
        }
        const timedOut = readOrderStore().orders.filter((order) => (
            order.status === 'PROCESSING'
            && order.paymentStatus === 'PAID'
            && !order.placedAt
            && Number.isFinite(Date.parse(order.executionStartedAt))
            && now - Date.parse(order.executionStartedAt) >= ORDER_EXECUTION_REFUND_TIMEOUT_MS
        ));
        for (const order of timedOut) {
            try {
                const reason = `机器人执行超过 ${Math.round(ORDER_EXECUTION_REFUND_TIMEOUT_MS / 60000)} 分钟且未确认成功放置`;
                order.failureReason = reason;
                order.failedAt = new Date().toISOString();
                saveOrder(order);
                const delivery = orderDeliveryBreakdown(order);
                if (delivery.deliveredUnits > 0 && delivery.refundAmount > 0) {
                    await refundOrder(order, reason, 'execution-timeout', {
                        refundItems: delivery.undeliveredItems,
                        refundAmount: delivery.refundAmount
                    });
                    log(`💸 订单 ${order.orderId} 执行超时，仅退款未交付商品 ¥${delivery.refundAmount.toFixed(2)}`);
                } else {
                    await refundOrder(order, reason, 'execution-timeout');
                    log(`💸 订单 ${order.orderId} 执行超时，已触发自动退款`);
                }
            } catch (error) {
                log(`❌ 订单 ${order.orderId} 超时自动退款失败: ${error.message}`);
            }
        }
        const pendingRefunds = readOrderStore().orders.filter((order) => (
            ['REFUNDING', 'PARTIAL_REFUNDING'].includes(order.status)
            && ['REFUNDING', 'PARTIAL_REFUNDING'].includes(order.paymentStatus)
            && (!Number.isFinite(Date.parse(order.refundLastAttemptAt))
                || now - Date.parse(order.refundLastAttemptAt) >= ORDER_REFUND_SWEEP_INTERVAL_MS)
        ));
        for (const order of pendingRefunds) {
            try {
                const reason = order.refundReason || '继续处理未完成退款';
                const updated = await refundOrder(order, reason, order.refundTrigger || 'refund-recovery', { resume: true });
                log(['REFUNDED', 'PARTIALLY_COMPLETED'].includes(updated.status)
                    ? `💸 订单 ${order.orderId} 退款状态已恢复为成功`
                    : `⏳ 订单 ${order.orderId} 退款仍在处理中`);
            } catch (error) {
                log(`❌ 订单 ${order.orderId} 退款恢复失败: ${error.message}`);
            }
        }
    } finally {
        refundSweepRunning = false;
    }
}

async function notifyOrderExecutionStarted(orderId, completionToken) {
    const localOrder = orderById(orderId);
    if (localOrder) {
        if (localOrder.paymentStatus !== 'PAID' || !['PAID', 'PROCESSING'].includes(localOrder.status)) {
            throw new Error(`订单状态不允许开始抓取: ${localOrder.status}/${localOrder.paymentStatus}`);
        }
        if (!localOrder.executionStartedAt) localOrder.executionStartedAt = new Date().toISOString();
        setOrderStatus(localOrder, 'PROCESSING', 'PAID');
        return publicOrder(localOrder);
    }
    if (!completionToken) {
        if (REQUIRE_SIGNED_ORDERS) throw new Error('支付订单缺少签名凭证，拒绝开始抓取');
        return null;
    }
    if (!ORDER_STATUS_CALLBACK_URL || !ORDER_STATUS_CALLBACK_SECRET) {
        throw new Error('未配置 ORDER_STATUS_CALLBACK_URL/SECRET，不能安全开始支付订单');
    }
    const reason = 'started';
    const timestamp = String(Date.now());
    const signature = orderCallbackSignature(timestamp, orderId, completionToken, reason);
    const response = await postOrderStatusCallback(
        `/api/orders/${encodeURIComponent(orderId)}/started`,
        { orderId, completionToken },
        { 'X-Grasp-Timestamp': timestamp, 'X-Grasp-Signature': signature }
    );
    return response.data;
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
        if (['**stow**', '__stow__', '__shutdown__', '__poweroff__', '__shutdown_power__', '__quit__', '__exit__'].includes(name)) {
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
    const combined = new Map();
    items.forEach((item, index) => {
        const id = Number(item && item.id);
        const qty = Number(item && item.qty);
        if (!Number.isInteger(id) || id < 1) throw new Error(`第 ${index + 1} 个商品 id 无效`);
        if (!Number.isInteger(qty) || qty < 1 || qty > ROBOT_MAX_ITEM_QTY) {
            throw new Error(`第 ${index + 1} 个商品数量必须是 1..${ROBOT_MAX_ITEM_QTY} 的整数`);
        }
        const existing = combined.get(id) || { id, qty: 0, name: String(item.name || '').trim() };
        existing.qty += qty;
        if (existing.qty > ROBOT_MAX_ITEM_QTY) throw new Error(`商品 ${id} 合计数量不能超过 ${ROBOT_MAX_ITEM_QTY}`);
        combined.set(id, existing);
    });
    const normalized = [...combined.values()];
    if (normalized.reduce((sum, item) => sum + item.qty, 0) > ROBOT_MAX_ORDER_UNITS) {
        throw new Error(`订单商品总数不能超过 ${ROBOT_MAX_ORDER_UNITS}`);
    }
    return normalized;
}

function signOrderPayload(payload) {
    return crypto.createHmac('sha256', ORDER_TOKEN_SECRET).update(payload).digest('base64url');
}

function createOrderToken(orderId, items) {
    const payload = Buffer.from(JSON.stringify({ orderId, items, exp: Date.now() + ORDER_TOKEN_TTL_MS })).toString('base64url');
    return `${payload}.${signOrderPayload(payload)}`;
}

function readOrderToken(token, { allowExpired = false } = {}) {
    const value = String(token || '');
    const separator = value.lastIndexOf('.');
    if (separator < 1) return null;
    const payload = value.slice(0, separator);
    if (!safeEqual(value.slice(separator + 1), signOrderPayload(payload))) return null;
    try {
        const order = JSON.parse(Buffer.from(payload, 'base64url').toString('utf8'));
        return allowExpired || Number(order.exp) > Date.now() ? order : null;
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
        if (!enforceRateLimit(req, res, 'admin-login', 10, 15 * 60 * 1000)) return;
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
        if (!enforceRateLimit(req, res, 'order-prepare', 12, 60 * 1000)) return;
        const body = await parseBody(req);
        let paymentStarted = false;
        let createdOrder = null;
        try {
            const items = normalizeInventoryItems(body && body.items);
            const products = readProducts();
            const pricedItems = items.map((item) => {
                const product = products.find((candidate) => Number(candidate.id) === item.id);
                if (!product || Number(product.status) !== 1) throw new Error(`商品不可用: ${item.name || item.id}`);
                if (Number(product.stock) < item.qty) throw new Error(`${product.name} 库存不足，当前库存 ${product.stock}`);
                const unitPrice = Number(product.price || 0);
                if (!Number.isFinite(unitPrice) || unitPrice <= 0 || unitPrice > 100000) {
                    throw new Error(`商品 ${product.name} 价格无效`);
                }
                return {
                    ...item,
                    name: product.name,
                    spec: product.spec || '',
                    unitPrice,
                    lineTotal: Number((unitPrice * item.qty).toFixed(2)),
                    paymentMenuItemId: null,
                    productStock: Number(product.stock) || 0,
                    shelf_layer: parseLayerFromLocation(product.location),
                    chassis_marker: product.chassis_marker || null
                };
            });
            const totalAmount = Number(pricedItems.reduce((sum, item) => sum + item.lineTotal, 0).toFixed(2));
            const orderId = `ORD-${Date.now().toString(36).toUpperCase()}-${crypto.randomBytes(4).toString('hex').toUpperCase()}`;
            const trustedTestRequest = String(req.headers['x-grasp-test-mode'] || '') === '1';
            const paymentMode = trustedTestRequest ? 'free' : PAYMENT_MODE;
            if (!['free', 'lyorder'].includes(paymentMode)) throw new Error(`不支持的支付模式: ${paymentMode}`);
            if (paymentMode === 'lyorder') {
                for (const item of pricedItems) {
                    item.paymentMenuItemId = await paymentClient.ensureMenuItem({
                        deviceSn: PAYMENT_DEVICE_SN,
                        productId: item.id,
                        name: item.name,
                        spec: item.spec,
                        unitPrice: item.unitPrice,
                        stock: item.productStock
                    });
                }
            }

            let order = {
                orderId,
                items: pricedItems,
                totalAmount,
                paymentMode,
                paymentStatus: paymentMode === 'free' ? 'PAID' : 'PENDING_PAYMENT',
                status: paymentMode === 'free' ? 'PAID' : 'PENDING_PAYMENT',
                source: trustedTestRequest ? 'test' : String(body && body.source || 'web').slice(0, 32),
                createdAt: new Date().toISOString(),
                updatedAt: new Date().toISOString()
            };
            saveOrder(order);
            createdOrder = order;

            if (paymentMode === 'free') {
                order = markOrderPaid(order, 'free');
                sendJSON(res, {
                    code: 0,
                    orderId,
                    items: pricedItems,
                    totalAmount,
                    paymentRequired: false,
                    paymentStatus: order.paymentStatus,
                    completionToken: createOrderToken(orderId, pricedItems)
                });
                return;
            }

            paymentStarted = true;
            const business = await paymentClient.createBusinessOrder({ deviceSn: PAYMENT_DEVICE_SN, items: pricedItems });
            if (business.totalAmount === null) throw new Error('支付业务订单接口未返回可核验金额');
            if (Math.abs(business.totalAmount - totalAmount) > 0.001) {
                throw new Error(`支付系统金额 ¥${business.totalAmount.toFixed(2)} 与本地金额 ¥${totalAmount.toFixed(2)} 不一致`);
            }
            order.externalOrderId = business.orderId;
            saveOrder(order);
            createdOrder = order;
            const native = await paymentClient.createNativePayment(business.orderId, `机器人商品订单-${orderId}`);
            const paymentQrPng = await createPaymentQrCode(native.codeUrl);
            order.paymentQrPngBase64 = paymentQrPng.toString('base64');
            order.paymentQrAccessToken = crypto.randomBytes(24).toString('base64url');
            saveOrder(order);
            createdOrder = order;
            sendJSON(res, {
                code: 0,
                orderId,
                items: pricedItems,
                totalAmount,
                paymentRequired: true,
                paymentStatus: order.paymentStatus,
                payment: {
                    qrImageUrl: `/api/orders/${encodeURIComponent(orderId)}/payment-qr/${order.paymentQrAccessToken}`
                },
                completionToken: createOrderToken(orderId, pricedItems)
            });
        } catch (error) {
            if (paymentStarted && createdOrder) {
                createdOrder.paymentStatus = 'PAYMENT_FAILED';
                createdOrder.status = 'PAYMENT_FAILED';
                createdOrder.failureReason = error.message;
                createdOrder.updatedAt = new Date().toISOString();
                saveOrder(createdOrder);
            }
            sendJSON(res, { code: 1, message: error.message }, paymentStarted ? 502 : 400);
        }
        return;
    }

    const paymentQrMatch = pathname.match(/^\/api\/orders\/([^/]+)\/payment-qr\/([A-Za-z0-9_-]{32})$/);
    if (paymentQrMatch && req.method === 'GET') {
        const orderId = decodeURIComponent(paymentQrMatch[1]);
        const accessToken = paymentQrMatch[2];
        const order = orderById(orderId);
        if (!order || !order.paymentQrAccessToken || !safeEqual(accessToken, order.paymentQrAccessToken)
            || !order.paymentQrPngBase64) {
            res.writeHead(404, { 'Cache-Control': 'no-store' });
            res.end();
            return;
        }
        const image = Buffer.from(order.paymentQrPngBase64, 'base64');
        const pngSignature = Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]);
        if (image.length < pngSignature.length || !image.subarray(0, pngSignature.length).equals(pngSignature)) {
            res.writeHead(500, { 'Cache-Control': 'no-store' });
            res.end();
            return;
        }
        res.writeHead(200, {
            'Content-Type': 'image/png',
            'Content-Length': image.length,
            'Cache-Control': 'no-store, no-cache, must-revalidate',
            'X-Content-Type-Options': 'nosniff'
        });
        res.end(image);
        return;
    }

    const paymentStatusMatch = pathname.match(/^\/api\/orders\/([^/]+)\/payment$/);
    if (paymentStatusMatch && req.method === 'GET') {
        sendJSON(res, { code: 1, message: '支付查询凭证禁止放在 URL，请使用 POST' }, 405, { Allow: 'POST' });
        return;
    }
    if (paymentStatusMatch && req.method === 'POST') {
        if (!enforceRateLimit(req, res, 'payment-query', 120, 60 * 1000)) return;
        const orderId = decodeURIComponent(paymentStatusMatch[1]);
        const body = await parseBody(req);
        const orderToken = readOrderToken(body && body.completionToken);
        if (!orderToken || orderToken.orderId !== orderId) {
            sendJSON(res, { code: 1, message: '订单凭证无效或已过期' }, 401);
            return;
        }
        const order = orderById(orderId);
        if (!order) {
            sendJSON(res, { code: 1, message: '订单不存在' }, 404);
            return;
        }
        try {
            const latest = order.paymentMode === 'lyorder'
                ? await reconcileOrderPayment(order, 'client-query')
                : order;
            const paid = latest.paymentStatus === 'PAID' && latest.status === 'PAID';
            sendJSON(res, {
                code: 0,
                orderId,
                paid,
                paymentStatus: latest.paymentStatus,
                status: latest.status,
                tradeState: latest.paymentTradeState || null,
                message: paymentStatusMessage(latest)
            });
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message }, 502);
        }
        return;
    }

    const cancelPaymentMatch = pathname.match(/^\/api\/orders\/([^/]+)\/cancel-payment$/);
    if (cancelPaymentMatch && req.method === 'POST') {
        if (!enforceRateLimit(req, res, 'payment-cancel', 20, 60 * 1000)) return;
        const body = await parseBody(req);
        const orderId = decodeURIComponent(cancelPaymentMatch[1]);
        const completionToken = String(body && body.completionToken || '');
        const tokenOrder = readOrderToken(completionToken);
        if (!tokenOrder || tokenOrder.orderId !== orderId) {
            sendJSON(res, { code: 1, message: '订单凭证无效或已过期' }, 401);
            return;
        }
        const order = orderById(orderId);
        if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
        if (order.executionStartedAt || ['PROCESSING', 'PLACED', 'COMPLETED'].includes(order.status)) {
            sendJSON(res, { code: 1, message: '订单已经开始执行，不能取消支付' }, 409);
            return;
        }
        order.cancelRequestedAt = order.cancelRequestedAt || new Date().toISOString();
        saveOrder(order);
        try {
            let latest = order;
            if (order.paymentMode === 'lyorder') {
                if (order.externalOrderId) await paymentClient.close(order.externalOrderId);
                latest = await reconcileOrderPayment(order, 'cancel-query');
            }
            sendJSON(res, { code: 0, data: publicOrder(latest) });
        } catch (error) {
            sendJSON(res, { code: 1, message: error.message }, 502);
        }
        return;
    }

    const startedOrderMatch = pathname.match(/^\/api\/orders\/([^/]+)\/started$/);
    if (startedOrderMatch && req.method === 'POST') {
        const body = await parseBody(req);
        const orderId = decodeURIComponent(startedOrderMatch[1]);
        const completionToken = String(body && body.completionToken || '');
        const tokenOrder = readOrderToken(completionToken);
        if (!tokenOrder || tokenOrder.orderId !== orderId || !verifyOrderCallback(req, orderId, completionToken, 'started')) {
            sendJSON(res, { code: 1, message: '订单开始回调签名无效' }, 401);
            return;
        }
        const order = orderById(orderId);
        if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
        if (order.paymentStatus !== 'PAID' || !['PAID', 'PROCESSING'].includes(order.status)) {
            sendJSON(res, { code: 1, message: `订单状态不允许开始抓取: ${order.status}/${order.paymentStatus}` }, 409);
            return;
        }
        if (!order.executionStartedAt) order.executionStartedAt = new Date().toISOString();
        setOrderStatus(order, 'PROCESSING', 'PAID');
        sendJSON(res, { code: 0, data: publicOrder(order) });
        return;
    }

    const failureOrderMatch = pathname.match(/^\/api\/orders\/([^/]+)\/failure$/);
    if (failureOrderMatch && req.method === 'POST') {
        const body = await parseBody(req);
        const orderId = decodeURIComponent(failureOrderMatch[1]);
        const completionToken = String(body && body.completionToken || '');
        const reason = String(body && body.reason || '机器人未能完成抓取和放置').slice(0, 500);
        const tokenOrder = readOrderToken(completionToken);
        if (!tokenOrder || tokenOrder.orderId !== orderId || !verifyOrderCallback(req, orderId, completionToken, reason)) {
            sendJSON(res, { code: 1, message: '自动退款回调签名无效' }, 401);
            return;
        }
        const order = orderById(orderId);
        if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
        if (Array.isArray(body && body.deliveredItems) && body.deliveredItems.length > 0) {
            try {
                setOrderDeliveredItems(order, body.deliveredItems);
            } catch (error) {
                sendJSON(res, { code: 1, message: error.message }, 400);
                return;
            }
        }
        if (order.placedAt || order.completedAt || ['PLACED', 'COMPLETED'].includes(order.status)) {
            sendJSON(res, { code: 0, skipped: true, message: '订单已成功放置，不执行自动退款', data: publicOrder(order) });
            return;
        }
        try {
            order.failureReason = reason;
            order.failedAt = new Date().toISOString();
            saveOrder(order);
            const delivery = orderDeliveryBreakdown(order);
            const refunded = delivery.deliveredUnits > 0 && delivery.refundAmount > 0
                ? await refundOrder(order, reason, 'robot-failure', {
                    refundItems: delivery.undeliveredItems,
                    refundAmount: delivery.refundAmount
                })
                : await refundOrder(order, reason, 'robot-failure');
            sendJSON(res, { code: 0, data: publicOrder(refunded) });
        } catch (error) {
            order.refundStatus = 'FAIL';
            order.refundReason = `${reason} (${error.message})`;
            setOrderStatus(
                order,
                Number(order.deliveredUnits || 0) > 0 ? 'PARTIALLY_COMPLETED' : 'FAILED',
                order.paymentStatus
            );
            sendJSON(res, { code: 1, message: `自动退款失败: ${error.message}` }, 502);
        }
        return;
    }

    const customerRefundMatch = pathname.match(/^\/api\/orders\/([^/]+)\/refund$/);
    if (customerRefundMatch && req.method === 'POST') {
        if (!enforceRateLimit(req, res, 'customer-refund', 20, 60 * 1000)) return;
        const body = await parseBody(req);
        const orderId = decodeURIComponent(customerRefundMatch[1]);
        const completionToken = String(body && body.completionToken || '');
        const reason = String(body && body.reason || '订单未能开始或完成抓取放置').slice(0, 500);
        const tokenOrder = readOrderToken(completionToken);
        if (!tokenOrder || tokenOrder.orderId !== orderId) {
            sendJSON(res, { code: 1, message: '订单凭证无效或已过期' }, 401);
            return;
        }
        const order = orderById(orderId);
        if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
        if (order.completedAt || order.placedAt || order.status === 'COMPLETED') {
            sendJSON(res, { code: 0, skipped: true, message: '订单已成功放置，不执行自动退款', data: publicOrder(order) });
            return;
        }
        if (order.status === 'PROCESSING' || order.executionStartedAt) {
            sendJSON(res, { code: 1, message: '机器人已经开始执行，只能由盒子失败回调触发退款' }, 409);
            return;
        }
        try {
            order.failureReason = reason;
            order.failedAt = new Date().toISOString();
            saveOrder(order);
            const refunded = await refundOrder(order, reason, 'client-fallback');
            sendJSON(res, { code: 0, data: publicOrder(refunded) });
        } catch (error) {
            order.refundStatus = 'FAIL';
            order.refundReason = `${reason} (${error.message})`;
            setOrderStatus(order, 'FAILED', order.paymentStatus);
            sendJSON(res, { code: 1, message: `自动退款失败: ${error.message}` }, 502);
        }
        return;
    }

    const completeOrderMatch = pathname.match(/^\/api\/orders\/([^/]+)\/complete$/);
    if (completeOrderMatch && req.method === 'POST') {
        const body = await parseBody(req);
        const orderId = decodeURIComponent(completeOrderMatch[1]);
        const completionToken = String(body && body.completionToken || '');
        if (String(body && body.orderId || '') !== orderId
            || !verifyOrderCallback(req, orderId, completionToken, 'completed')) {
            sendJSON(res, { code: 1, message: '订单完成回调签名无效' }, 401);
            return;
        }
        try {
            const completed = completePaidOrder(orderId, completionToken);
            sendJSON(res, { code: 0, ...completed });
        } catch (error) {
            const latest = orderById(orderId);
            if (latest && latest.placedAt && latest.status !== 'COMPLETED') {
                latest.completionError = String(error.message || error).slice(0, 500);
                setOrderStatus(latest, 'PLACED', 'PAID');
            }
            sendJSON(res, { code: 1, message: error.message }, error.statusCode || 409);
        }
        return;
    }

    const adminOrderMatch = pathname.match(/^\/api\/admin\/orders(?:\/([^/]+))?(?:\/refund)?$/);
    if (adminOrderMatch && pathname.startsWith('/api/admin/orders')) {
        if (!requireAdmin(req, res)) return;
        const orderId = adminOrderMatch[1] ? decodeURIComponent(adminOrderMatch[1]) : null;
        const isRefund = /\/refund$/.test(pathname);
        if (isRefund && req.method === 'POST' && orderId) {
            const body = await parseBody(req);
            const order = orderById(orderId);
            if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
            if (!['PAID', 'PLACED', 'COMPLETED', 'FAILED'].includes(order.status) || order.paymentStatus !== 'PAID') {
                sendJSON(res, { code: 1, message: '仅已支付或已完成订单可退款' }, 409); return;
            }
            const reason = String(body && body.reason || '管理员退款').slice(0, 200);
            try {
                const refunded = await refundOrder(order, reason, 'admin');
                sendJSON(res, { code: 0, data: publicOrder(refunded) });
            } catch (error) {
                order.refundStatus = 'FAIL';
                order.refundReason = `${reason} (${error.message})`;
                saveOrder(order);
                sendJSON(res, { code: 1, message: `退款失败: ${error.message}` }, 502);
            }
            return;
        }
        if (orderId && req.method === 'GET') {
            const order = orderById(orderId);
            if (!order) { sendJSON(res, { code: 1, message: '订单不存在' }, 404); return; }
            sendJSON(res, { code: 0, data: publicOrder(order) });
            return;
        }
        if (!orderId && req.method === 'GET') {
            const page = Math.max(1, Number(url.searchParams.get('page') || 1));
            const pageSize = Math.min(100, Math.max(1, Number(url.searchParams.get('pageSize') || url.searchParams.get('size') || 10)));
            const status = String(url.searchParams.get('status') || '').trim();
            const search = String(url.searchParams.get('search') || '').trim().toLowerCase();
            const includeTest = url.searchParams.get('include_test') === '1';
            let orders = readOrderStore().orders;
            if (!includeTest) orders = orders.filter((item) => item.source !== 'test');
            if (status) orders = orders.filter((item) => item.status === status || item.paymentStatus === status);
            if (search) orders = orders.filter((item) => [item.orderId, item.externalOrderId, ...(item.items || []).map((i) => i.name)].some((value) => String(value || '').toLowerCase().includes(search)));
            const total = orders.length;
            const data = orders.slice((page - 1) * pageSize, page * pageSize).map(publicOrder);
            sendJSON(res, { code: 0, data, page, pageSize, total, totalPages: Math.max(1, Math.ceil(total / pageSize)) });
            return;
        }
        sendJSON(res, { code: 1, message: '订单接口方法不支持' }, 405);
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

    if (pathname === '/api/robot/status' && req.method === 'GET') {
        if (!requireAdmin(req, res)) return;
        const operation = currentRobotOperation();
        sendJSON(res, {
            code: 0,
            data: {
                status: currentRobotStatus(),
                label: operation
                    ? operation.label
                    : !ROBOT_GRASP_REAL
                        ? '模拟抓取模式'
                        : robotWorkerReady
                            ? '机器人已上电待机'
                            : '机器人正在启动',
                worker_ready: robotWorkerReady,
                powered_off: robotPoweredOff,
                emergency_stopped: robotEmergencyStopped,
                busy: Boolean(operation && operation.status !== 'powered-off')
            }
        });
        return;
    }

    if (pathname === '/api/robot/stow' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
        const operation = currentRobotOperation();
        if (operation) {
            sendJSON(res, { code: 1, message: `${operation.label}，暂不能收起` }, 409);
            return;
        }
        robotStowPromise = moveRobotToStowPose();
        broadcastRobotStatus('stowing', '管理员正在收起机械臂');
        try {
            const result = await robotStowPromise;
            log('✅ 管理员操作：机械臂已收起并保持上电');
            sendJSON(res, { code: 0, message: '机械臂已收起并保持上电', data: result });
        } catch (error) {
            log(`❌ 管理员收起失败: ${error.message}`);
            sendJSON(res, { code: 1, message: error.message }, 500);
        } finally {
            robotStowPromise = null;
            broadcastRobotStatus();
        }
        return;
    }

    if (pathname === '/api/robot/shutdown' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
        if (robotEmergencyStopped) {
            sendJSON(res, { code: 1, message: '急停已触发，请人工检查并重启后端' }, 409);
            return;
        }
        const operation = currentRobotOperation();
        if (operation) {
            sendJSON(res, { code: 1, message: `${operation.label}，暂不能关闭` }, 409);
            return;
        }
        robotShutdownPromise = (async () => {
            broadcastRobotStatus('shutting-down', '正在收起机械臂');
            await moveRobotToStowPose();
            broadcastRobotStatus('powering-off', '机械臂已收起，正在断电关闭');
            await powerOffRobotWorker();
            if (!ROBOT_GRASP_REAL) {
                return { mode: 'simulated', detail: '模拟模式已关闭' };
            }
            broadcastRobotStatus('chassis-moving', `机器人已关闭，底盘正在回充电桩 ${CHASSIS_CHARGE_MARKER}`);
            await navigateChassisToMarker(CHASSIS_CHARGE_MARKER, {
                arriveOnDocking: true,
                statusMessage: `机器人已关闭，底盘正在回充电桩 ${CHASSIS_CHARGE_MARKER}`
            });
            return { mode: ROBOT_GRASP_REAL ? 'real' : 'simulated', detail: '已收起、断电并回到充电桩' };
        })();
        try {
            const result = await robotShutdownPromise;
            log('⏻ 管理员操作：机器人已收起、断电并回到充电桩');
            sendJSON(res, { code: 0, message: '机器人已收起、断电并回到充电桩', data: result });
        } catch (error) {
            log(`❌ 关闭并回充失败: ${error.message}`);
            sendJSON(res, { code: 1, message: error.message }, 500);
        } finally {
            robotShutdownPromise = null;
            broadcastRobotStatus();
        }
        return;
    }

    if (pathname === '/api/robot/emergency-stop' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
        if (robotEmergencyStopped) {
            sendJSON(res, { code: 0, message: '急停已经触发，请人工检查并重启后端' });
            return;
        }
        if (robotEmergencyPromise) {
            try {
                await robotEmergencyPromise;
                sendJSON(res, { code: 0, message: '急停已触发，请人工检查并重启后端' });
            } catch (error) {
                sendJSON(res, { code: 1, message: error.message }, 500);
            }
            return;
        }
        robotEmergencyStopped = true;
        robotPoweredOff = true;
        robotWorkerReady = false;
        robotEmergencyPromise = emergencyStopRobot();
        try {
            await robotEmergencyPromise;
            sendJSON(res, { code: 0, message: '急停已触发，机械臂和底盘已停止，请人工检查并重启后端' });
        } catch (error) {
            log(`❌ 急停执行异常: ${error.message}`);
            sendJSON(res, { code: 1, message: `急停指令执行异常: ${error.message}` }, 500);
        } finally {
            robotEmergencyPromise = null;
            broadcastRobotStatus('emergency-stopped', '急停已触发，请人工检查并重启后端');
        }
        return;
    }

    if (pathname === '/api/robot/poweroff' && req.method === 'POST') {
        if (!requireAdmin(req, res)) return;
        if (robotPoweredOff) {
            sendJSON(res, { code: 0, message: '机器人已经断电关闭' });
            return;
        }
        const operation = currentRobotOperation();
        if (operation) {
            sendJSON(res, { code: 1, message: `${operation.label}，暂不能断电关闭` }, 409);
            return;
        }
        try {
            const result = await powerOffRobotWorker();
            log('⏻ 管理员操作：机器人已断电，抓取程序已关闭');
            sendJSON(res, { code: 0, message: '机器人已断电，抓取程序已关闭', data: result });
        } catch (error) {
            log(`❌ 机器人断电关闭失败: ${error.message}`);
            sendJSON(res, { code: 1, message: error.message }, 500);
        }
        return;
    }

    // GET/PUT /api/chassis/navigation-config - 底盘导航、自动回充开关与放置点
    if (pathname === '/api/chassis/navigation-config' && req.method === 'GET') {
        if (!requireAdmin(req, res)) return;
        sendJSON(res, { code: 0, data: readChassisSettings() });
        return;
    }
    if (pathname === '/api/chassis/navigation-config' && req.method === 'PUT') {
        if (!requireAdmin(req, res)) return;
        const body = await parseBody(req);
        if (!body ||
            (body.enabled !== undefined && typeof body.enabled !== 'boolean') ||
            (body.auto_return_charge !== undefined && typeof body.auto_return_charge !== 'boolean')) {
            sendJSON(res, { code: 1, message: 'enabled 和 auto_return_charge 必须是布尔值' }, 400);
            return;
        }
        try {
            const current = readChassisSettings();
            const placeMarker = body.place_marker === undefined
                ? current.place_marker
                : validateChassisMarkerName(body.place_marker);
            const settings = {
                navigation_enabled: body.enabled === undefined ? current.navigation_enabled : body.enabled,
                place_marker: placeMarker,
                auto_return_charge: body.auto_return_charge === undefined
                    ? current.auto_return_charge
                    : body.auto_return_charge
            };
            writeChassisSettings(settings);
            log(`🚙 底盘导航已${settings.navigation_enabled ? '开启' : '关闭'}，自动回充已${settings.auto_return_charge ? '开启' : '关闭'}，放置点=${settings.place_marker}`);
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
    migrateUncreatedPaymentOrders();
    log(`HTTP 服务已启动: http://${HOST}:${PORT}`);
    log(`商品管理页面: http://localhost:${PORT}/admin.html`);
    log(`抓取模式: ${ROBOT_GRASP_REAL ? '真实机械臂' : '模拟'} (${ROBOT_GRASP_MODE})`);
    log(`支付模式: ${PAYMENT_MODE}${PAYMENT_MODE === 'lyorder' ? `，支付 API: ${PAYMENT_API_BASE_URL || '未配置'}` : '（免费/测试）'}`);
    if (PAYMENT_MODE === 'lyorder') {
        setInterval(refundTimedOutOrders, Math.max(1000, PAYMENT_STATUS_SWEEP_INTERVAL_MS)).unref();
        refundTimedOutOrders().catch((error) => log(`⚠️ 超时退款巡检失败: ${error.message}`));
    }
    if (ORDER_STATUS_CALLBACK_URL) {
        setInterval(flushOrderCallbackOutbox, Math.max(5000, ORDER_CALLBACK_RETRY_INTERVAL_MS)).unref();
        flushOrderCallbackOutbox({ force: true }).catch((error) => log(`⚠️ 完成回调队列启动重试失败: ${error.message}`));
    }
    const chassisSettings = readChassisSettings();
    log(`底盘接口: ${CHASSIS_HOST}:${CHASSIS_PORT}，栅格地图: ${CHASSIS_MAP_HTTP_PROTOCOL}://${CHASSIS_MAP_HTTP_HOST}:${CHASSIS_MAP_HTTP_PORT}`);
    log(`底盘导航: ${chassisSettings.navigation_enabled ? '已开启' : '已关闭'}，自动回充: ${chassisSettings.auto_return_charge ? '已开启' : '已关闭'}，放置点: ${chassisSettings.place_marker}`);
    if (ROBOT_GRASP_REAL) {
        ensureRobotWorker()
            .then(() => {
                log('✅ 机器人启动上电完成，保持当前位置待机');
                broadcastRobotStatus('ready', '机器人已上电待机');
            })
            .catch((error) => {
                log(`❌ 机器人启动上电失败: ${error.message}`);
                broadcastRobotStatus('standby', `机器人启动上电失败: ${error.message}`);
            });
    }
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

function currentRobotOperation() {
    if (robotEmergencyPromise) return { status: 'emergency-stopping', label: '正在执行急停' };
    if (robotEmergencyStopped) return { status: 'emergency-stopped', label: '急停已触发，请人工检查并重启后端' };
    if (robotFailureRecoveryPromise) return { status: 'returning', label: '订单失败后正在收起并回充' };
    if (chassisNavigationPromise) return { status: 'chassis-moving', label: '底盘正在运动' };
    if (robotShutdownPromise) return { status: 'shutting-down', label: '机器人正在收起、断电并回充' };
    if (robotPoweroffPromise) return { status: 'powering-off', label: '机器人正在断电关闭' };
    if (robotStowPromise) return { status: 'stowing', label: '机械臂正在收起' };
    if (activeOrderId) return { status: 'busy', label: `订单 ${activeOrderId} 正在执行` };
    if (robotFilterPreviewPromise) return { status: 'busy', label: '正在采集层板过滤预览' };
    if (robotPoweredOff) return { status: 'powered-off', label: '机器人已断电关闭，请重启服务重新上电' };
    return null;
}

function currentRobotStatus() {
    const operation = currentRobotOperation();
    if (operation) return operation.status;
    if (!ROBOT_GRASP_REAL) return 'simulated';
    return robotWorkerReady ? 'ready' : 'standby';
}

function rejectIfRobotBusy(ws, action, responseType = 'error') {
    const operation = currentRobotOperation();
    if (!operation) return false;
    const response = { type: responseType, message: `${operation.label}，暂不能${action}` };
    if (responseType !== 'error') response.status = 'error';
    sendMessage(ws, response);
    return true;
}

async function rejectOrderWithRefund(ws, msg, reason) {
    const orderId = String(msg && msg.orderId || '').trim();
    let message = reason;
    let refundTriggered = false;
    if (orderId) {
        try {
            const refunded = await refundFailedExecution(orderId, msg.completionToken, reason);
            refundTriggered = true;
            message = `${reason}；订单已${refunded.status === 'REFUNDED' ? '自动退款' : '提交退款'}`;
        } catch (refundError) {
            message = `${reason}；自动退款失败，请联系管理员：${refundError.message}`;
            log(`❌ 订单 ${orderId} 拒绝执行后的自动退款失败: ${refundError.message}`);
        }
    }
    if (orderId && !currentRobotOperation()) {
        await recoverRobotAfterFailedOrder(ws, orderId, reason);
    }
    sendMessage(ws, { type: 'error', orderId: orderId || undefined, message, refundTriggered });
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

        const logMessage = msg.completionToken ? { ...msg, completionToken: '[REDACTED]' } : msg;
        log(`📩 收到命令: ${JSON.stringify(logMessage)}`);

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

    if (rejectIfRobotBusy(ws, '采集过滤预览', 'filter-preview')) return;

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

function broadcastRobotStatus(status = currentRobotStatus(), message = '') {
    broadcast({ type: 'robot_status', status, message, timestamp: Date.now() });
}

function readChassisSettings() {
    try {
        if (!fs.existsSync(CHASSIS_SETTINGS_FILE)) {
            return {
                navigation_enabled: CHASSIS_NAVIGATION_DEFAULT_ENABLE,
                auto_return_charge: ROBOT_AUTO_RETURN_CHARGE,
                place_marker: CHASSIS_PLACE_MARKER_DEFAULT
            };
        }
        const data = JSON.parse(fs.readFileSync(CHASSIS_SETTINGS_FILE, 'utf8'));
        return {
            navigation_enabled: Boolean(data.navigation_enabled),
            auto_return_charge: data.auto_return_charge === undefined
                ? ROBOT_AUTO_RETURN_CHARGE
                : Boolean(data.auto_return_charge),
            place_marker: validateChassisMarkerName(data.place_marker || CHASSIS_PLACE_MARKER_DEFAULT)
        };
    } catch (error) {
        log(`⚠️ 底盘设置读取失败: ${error.message}`);
        return {
            navigation_enabled: CHASSIS_NAVIGATION_DEFAULT_ENABLE,
            auto_return_charge: ROBOT_AUTO_RETURN_CHARGE,
            place_marker: CHASSIS_PLACE_MARKER_DEFAULT
        };
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

async function navigateChassisToMarker(markerName, options = {}) {
    if (chassisNavigationPromise) {
        throw new Error('底盘正在运动，暂不接受新的导航任务');
    }
    const operation = navigateChassisToMarkerUnlocked(markerName, options);
    chassisNavigationPromise = operation;
    broadcastRobotStatus(
        'chassis-moving',
        options.statusMessage || `底盘正在前往 ${markerName}`
    );
    try {
        return await operation;
    } finally {
        if (chassisNavigationPromise === operation) chassisNavigationPromise = null;
        broadcastRobotStatus();
    }
}

async function navigateChassisToMarkerUnlocked(markerName, { beforeMove, arriveOnDocking = false } = {}) {
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
    let stablePosePolls = 0;
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
        const runningStatus = info.running_status;
        const moveTarget = info.move_target;
        const targetMatches = moveTarget === markerName;
        if (targetMatches && moveStatus === 'running') seenCurrentTask = true;
        const state = `${moveStatus}/${info.running_status || ''}/${moveTarget || ''}`;
        if (state !== lastState) {
            log(`  [底盘] ${state}`);
            lastState = state;
        }
        if (targetMatches && arriveOnDocking && runningStatus === 'dock_to_charging_pile') {
            log(`  ✅ 底盘已到达 ${markerName}，正在自动对接充电桩`);
            return { skipped: false, marker, docking: true };
        }
        if (!arriveOnDocking && currentPoseAtMarker(info, marker)) {
            stablePosePolls += 1;
            if (stablePosePolls >= 2) {
                log(`  ✅ 底盘实时位姿连续 ${stablePosePolls} 次进入容差，确认已到达 ${markerName}`);
                return { skipped: false, marker, poseConfirmed: true };
            }
        } else {
            stablePosePolls = 0;
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
    if (robotPoweredOff) return Promise.reject(new Error('机器人已由管理员断电关闭，请重启服务重新上电'));
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
                SHELF_GRASP_WORKER: '1',
                SHELF_GRASP_IDLE_STARTUP: '1'
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

function handleStowCommand(ws) {
    sendMessage(ws, { type: 'error', message: '收起操作仅可在管理页执行' });
}

// ========== 处理移动到层命令 ==========
async function handleGotoLayerCommand(ws, msg) {
    if (rejectIfRobotBusy(ws, '移动机械臂')) return;
    try {
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

async function returnRobotToCharge(
    ws,
    orderId,
    autoReturnCharge = readChassisSettings().auto_return_charge,
    { failed = false } = {}
) {
    if (!ROBOT_GRASP_REAL) return;
    sendStatus(ws, 'returning', {
        orderId,
        progress: 99,
        phase: 'stowing',
        message: failed ? '订单未完成，机械臂正在安全收起...' : '订单已拣选完成，机械臂正在收起...'
    });
    broadcastRobotStatus('stowing', failed ? '订单失败恢复中，机械臂正在安全收起' : '订单完成，机械臂正在收起');
    await moveRobotToStowPose();

    if (!autoReturnCharge) {
        sendStatus(ws, 'returning', {
            orderId,
            progress: 100,
            phase: 'stowed',
            message: '机械臂已收起，自动回充已关闭，保持当前位置'
        });
        broadcastRobotStatus('returning', '机械臂已收起，自动回充已关闭');
        return;
    }

    sendStatus(ws, 'returning', {
        orderId,
        progress: 99,
        phase: 'charging',
        message: failed ? '失败恢复：机械臂已收起，底盘正在回充电桩...' : '机械臂已收起，底盘正在回充电桩...'
    });
    await navigateChassisToMarker(CHASSIS_CHARGE_MARKER, {
        arriveOnDocking: true,
        statusMessage: '订单已完成，底盘正在回充电桩'
    });
    sendStatus(ws, 'returning', {
        orderId,
        progress: 100,
        phase: 'settling',
        message: `机器人已到达充电桩，等待 ${Math.round(ORDER_READY_CLOSE_DELAY_MS / 1000)} 秒后恢复点单...`
    });
    broadcastRobotStatus('returning', '机器人已到达充电桩，正在等待系统稳定');
}

async function recoverRobotAfterFailedOrder(ws, orderId, reason) {
    if (!ROBOT_GRASP_REAL) return true;
    if (robotFailureRecoveryPromise) return robotFailureRecoveryPromise;
    robotFailureRecoveryPromise = (async () => {
    try {
        log(`🛟 订单 ${orderId} 未完成放置，开始安全收起并强制回充: ${reason}`);
        await returnRobotToCharge(ws, orderId, true, { failed: true });
        await sleep(ORDER_READY_CLOSE_DELAY_MS);
        sendStatus(ws, 'order-ready', {
            orderId,
            message: '失败恢复完成：机械臂已收起并回到充电桩，可以继续点单',
            progress: 100
        });
        log(`✅ 订单 ${orderId} 失败恢复完成，机械臂已收起并回到充电桩`);
        return true;
    } catch (error) {
        const message = `订单失败后安全恢复失败：${error.message}`;
        log(`❌ 订单 ${orderId} ${message}`);
        sendStatus(ws, 'error', {
            orderId,
            message,
            recoveryFailed: true,
            originalFailure: reason
        });
        broadcastRobotStatus('standby', message);
        return false;
    }
    })();
    try {
        return await robotFailureRecoveryPromise;
    } finally {
        robotFailureRecoveryPromise = null;
        broadcastRobotStatus();
    }
}

function resolveOrderItems(msg, orderId) {
    let orderItems = msg.items;
    if (!msg.completionToken) {
        if (REQUIRE_SIGNED_ORDERS) throw new Error('支付订单缺少签名凭证，拒绝执行');
        return normalizeOrderItems(orderItems);
    }

    const preparedOrder = readOrderToken(msg.completionToken);
    if (preparedOrder && preparedOrder.orderId === orderId && Array.isArray(preparedOrder.items)) {
        orderItems = preparedOrder.items;
        log(`🔐 订单 ${orderId} 已从签名凭证恢复货架层和底盘点位`);
    } else {
        if (REQUIRE_SIGNED_ORDERS) throw new Error('支付订单签名凭证无效或已过期，拒绝执行');
        log(`⚠️ 订单 ${orderId} 的签名凭证无效，改用 WebSocket items`);
    }
    return normalizeOrderItems(orderItems);
}

function chassisPointFromStatus(status) {
    const pose = chassisResults(status).current_pose || {};
    const x = Number(pose.x);
    const y = Number(pose.y);
    return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

async function optimizeOrderItemSequence(items) {
    const settings = readChassisSettings();
    if (!ROBOT_GRASP_REAL || !settings.navigation_enabled || items.length < 2) return items;

    const productsById = new Map(readProducts().map((product) => [String(product.id), product]));
    let markerResponse;
    let statusResponse;
    try {
        const client = chassisClient();
        [markerResponse, statusResponse] = await Promise.all([
            client.request('/api/markers/query_list'),
            client.request('/api/robot_status')
        ]);
    } catch (error) {
        log(`  [订单排序] 无法读取底盘位置，保持下单顺序: ${error.message}`);
        return items;
    }
    if (markerResponse.status !== 'OK' || statusResponse.status !== 'OK') {
        log('  [订单排序] 底盘状态不可用，保持下单顺序');
        return items;
    }

    const markers = new Map(normalizeMarkers(chassisResults(markerResponse)).map((marker) => [marker.name, marker]));
    const placePoint = markers.get(settings.place_marker);
    const currentPoint = chassisPointFromStatus(statusResponse)
        || markers.get(CHASSIS_CHARGE_MARKER)
        || placePoint;
    if (!currentPoint || !placePoint) {
        log('  [订单排序] 缺少当前位姿或放置点坐标，保持下单顺序');
        return items;
    }

    const markerNameForItem = (item) => {
        const product = productsById.get(String(item.id));
        return item.chassis_marker || (product && product.chassis_marker) || '';
    };
    const ordered = orderItemsByNearestPickup(items, {
        markers,
        startPoint: currentPoint,
        placePoint,
        markerNameForItem
    });
    if (ordered === items) {
        log('  [订单排序] 商品点位坐标不完整，保持下单顺序');
        return items;
    }

    const sequence = ordered.map((item) => `${item.name}x${item.qty}@${markerNameForItem(item)}`);
    log(`  🧭 订单逐瓶取放排序: ${sequence.join(' -> ')}`);
    return ordered;
}

async function executeOrderItems(ws, orderId, items, totalUnits, deliveryProgress) {
    const productsById = ROBOT_GRASP_REAL
        ? new Map(readProducts().map((product) => [String(product.id), product]))
        : null;
    const units = [];
    for (const item of items) {
        for (let itemNo = 1; itemNo <= item.qty; itemNo++) {
            units.push({ item, itemNo });
        }
    }

    for (let index = 0; index < units.length; index += 1) {
        const { item, itemNo } = units[index];
        const label = item.qty > 1 ? `${item.name} (${itemNo}/${item.qty})` : item.name;
        const progressBase = (index / totalUnits) * 100;
        try {
            if (ROBOT_GRASP_REAL) {
                await runRealGrab(
                    ws,
                    orderId,
                    item,
                    progressBase,
                    totalUnits,
                    itemNo,
                    productsById.get(String(item.id)),
                );
            } else {
                await runSimulatedGrab(ws, orderId, item, progressBase, totalUnits, itemNo);
            }

            const deliveredById = new Map((deliveryProgress.deliveredItems || []).map((entry) => [Number(entry.id), entry]));
            const delivered = deliveredById.get(Number(item.id));
            if (delivered) delivered.qty += 1;
            else deliveryProgress.deliveredItems.push({ id: Number(item.id), qty: 1 });
            deliveryProgress.completedUnits += 1;
            const saved = recordOrderUnitDelivered(orderId, item);
            if (saved) {
                log(`  ✅ 已交付并记账: ${item.name} (${saved.deliveredUnits}/${totalUnits})`);
            }
        } catch (error) {
            const failure = {
                id: Number(item.id),
                name: item.name,
                itemNo,
                message: error.message
            };
            deliveryProgress.failedUnits.push(failure);
            log(`  ⚠️ 单瓶失败，暂不终止订单: ${error.message}`);

            const hasNext = index + 1 < units.length;
            sendStatus(ws, 'grabbing', {
                orderId,
                item: itemStatus(item),
                stage: 'item-failed',
                progress: Math.min(99, ((index + 1) / totalUnits) * 100),
                message: hasNext
                    ? `${label} 失败，正在安全收起后继续下一件商品`
                    : `${label} 失败，已完成本订单全部单瓶尝试`
            });
            if (!hasNext) continue;

            if (ROBOT_GRASP_REAL) {
                try {
                    await moveRobotToStowPose();
                    log(`  🔄 ${label} 失败后已安全收起，继续下一件商品`);
                } catch (stowError) {
                    deliveryProgress.continuationError = `单瓶失败后无法安全收起: ${stowError.message}`;
                    log(`  [停止] ${deliveryProgress.continuationError}`);
                    break;
                }
            }
        }
    }

    if (deliveryProgress.failedUnits.length > 0) {
        const failures = deliveryProgress.failedUnits.map((item) => item.message).join('；');
        const suffix = deliveryProgress.continuationError ? `；${deliveryProgress.continuationError}` : '';
        throw new Error(`部分商品抓取失败: ${failures}${suffix}`);
    }
}

async function completeOrder(ws, orderId) {
    const autoReturnCharge = ROBOT_GRASP_REAL && readChassisSettings().auto_return_charge;
    sendStatus(ws, 'completed', {
        orderId,
        message: ROBOT_GRASP_REAL
            ? (autoReturnCharge
                ? '商品已放置，订单完成，机器人正在收起并回充'
                : '商品已放置，订单完成，机器人正在收起，自动回充已关闭')
            : '所有商品抓取完成',
        progress: 100,
        phase: 'placed'
    });
    log(`✅ 订单 ${orderId} 商品已放置并完成，机器人${ROBOT_GRASP_REAL ? (autoReturnCharge ? '继续收起回充' : '继续收起但不回充') : '无需回充'}`);

    if (ROBOT_GRASP_REAL) {
        await returnRobotToCharge(ws, orderId, autoReturnCharge);
        await sleep(ORDER_READY_CLOSE_DELAY_MS);
    }
    sendStatus(ws, 'order-ready', {
        orderId,
        message: !ROBOT_GRASP_REAL
            ? '机器人可以继续点单'
            : (autoReturnCharge ? '机器人已回到充电桩，可以继续点单' : '机器人已收起，可以继续点单'),
        progress: 100
    });
    log(`✅ 订单 ${orderId} 收尾完成，已等待 ${ORDER_READY_CLOSE_DELAY_MS}ms`);
}

// ========== 处理抓取命令 ==========
async function handleGrabCommand(ws, msg) {
    const orderId = String(msg.orderId || '').trim();
    if (!orderId || !Array.isArray(msg.items) || msg.items.length === 0) {
        sendMessage(ws, { type: 'error', message: '缺少 orderId 或 items' });
        return;
    }

    const currentOperation = currentRobotOperation();
    if (currentOperation) {
        const message = activeOrderId === orderId
            ? `订单 ${orderId} 已在执行，忽略重复提交`
            : `${currentOperation.label}，暂不接受新订单，请稍后重试`;
        sendMessage(ws, {
            type: 'error',
            orderId,
            message,
            retryable: true,
            refundTriggered: false
        });
        return;
    }

    const savedOrder = orderById(orderId);
    if (savedOrder && savedOrder.paymentStatus !== 'PAID') {
        sendMessage(ws, { type: 'error', message: '订单尚未支付，不能开始抓取' });
        return;
    }

    let items;
    let totalUnits;
    try {
        ({ items, totalUnits } = resolveOrderItems(msg, orderId));
    } catch (error) {
        await rejectOrderWithRefund(ws, msg, error.message);
        return;
    }
    try {
        await notifyOrderExecutionStarted(orderId, msg.completionToken);
    } catch (error) {
        await rejectOrderWithRefund(ws, msg, error.message);
        return;
    }

    log(`🤖 开始处理订单 ${orderId}，共 ${totalUnits} 件商品`);
    activeOrderId = orderId;
    broadcastRobotStatus('busy', `正在处理订单 ${orderId}`);
    let fulfilled = false;
    const deliveryProgress = {
        completedUnits: 0,
        deliveredItems: [],
        failedUnits: [],
        continuationError: null
    };

    try {
        sendStatus(ws, 'started', {
            orderId,
            message: '订单已接收'
        });

        const orderedItems = await optimizeOrderItemSequence(items);
        await executeOrderItems(ws, orderId, orderedItems, totalUnits, deliveryProgress);
        fulfilled = true;
        const placedOrder = orderById(orderId);
        if (placedOrder) {
            try {
                completePaidOrder(orderId, msg.completionToken);
            } catch (completionError) {
                log(`⚠️ 订单 ${orderId} 已成功放置，但本地完成入账失败，将保留 PLACED 状态: ${completionError.message}`);
            }
        } else if (ORDER_STATUS_CALLBACK_URL) {
            queueOrderCompletedCallback(orderId, msg.completionToken);
            await flushOrderCallbackOutbox({ force: true });
        }
        await completeOrder(ws, orderId);
    } catch (e) {
        log(`❌ 订单 ${orderId} 失败: ${e.message}`);
        const failedOrder = orderById(orderId);
        let failureMessage = e.message;
        if (!fulfilled) {
            if (failedOrder) {
                failedOrder.failureReason = e.message;
                failedOrder.failedAt = new Date().toISOString();
                saveOrder(failedOrder);
            }
            try {
                const refunded = await refundFailedExecution(
                    orderId,
                    msg.completionToken,
                    e.message,
                    deliveryProgress.deliveredItems
                );
                const partial = refunded.paymentStatus === 'PARTIALLY_REFUNDED'
                    || refunded.paymentStatus === 'PARTIAL_REFUNDING';
                failureMessage = partial
                    ? `${e.message}；已交付 ${refunded.deliveredUnits} 件，仅退未交付商品 ¥${Number(refunded.refundAmount).toFixed(2)}`
                    : `${e.message}；订单已${refunded.status === 'REFUNDED' ? '自动退款' : '提交退款'}`;
                log(partial
                    ? `💸 订单 ${orderId} 已交付 ${refunded.deliveredUnits} 件，仅退款未交付商品 ¥${Number(refunded.refundAmount).toFixed(2)}`
                    : `💸 订单 ${orderId} 未完成抓取放置，已触发自动退款`);
            } catch (refundError) {
                failureMessage = `${e.message}；自动退款失败，请联系管理员：${refundError.message}`;
                log(`❌ 订单 ${orderId} 自动退款失败: ${refundError.message}`);
                const latest = orderById(orderId);
                if (latest) {
                    setOrderStatus(
                        latest,
                        Number(latest.deliveredUnits || 0) > 0 ? 'PARTIALLY_COMPLETED' : 'FAILED',
                        latest.paymentStatus
                    );
                }
            }
        } else {
            failureMessage = `${e.message}；商品已成功放置，不自动退款`;
            if (failedOrder) {
                failedOrder.postDeliveryError = e.message;
                saveOrder(failedOrder);
            }
        }
        if (!fulfilled) {
            await recoverRobotAfterFailedOrder(ws, orderId, e.message);
        }
        sendStatus(ws, 'error', { orderId, message: failureMessage, refundTriggered: !fulfilled });
    } finally {
        activeOrderId = null;
        broadcastRobotStatus();
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

async function runRealGrab(ws, orderId, item, progressBase, totalUnits, itemNo, product) {
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

function sendDobotDashboardCommand(command, timeoutMs = 1500) {
    return new Promise((resolve, reject) => {
        const socket = net.createConnection({ host: DOBOT_IP, port: DOBOT_PORT });
        let response = '';
        let settled = false;
        const finish = (error) => {
            if (settled) return;
            settled = true;
            clearTimeout(timer);
            socket.destroy();
            if (error) reject(error);
            else resolve(response.trim());
        };
        const timer = setTimeout(() => finish(new Error(`DOBOT ${command} 超时`)), timeoutMs);
        socket.once('error', (error) => finish(new Error(`DOBOT ${command} 连接失败: ${error.message}`)));
        socket.on('data', (chunk) => {
            response += chunk.toString();
            if (response.includes(';') || response.includes('}')) finish();
        });
        socket.once('connect', () => socket.write(`${command}\n`));
    });
}

async function emergencyStopRobot() {
    const errors = [];
    let successfulCommands = 0;
    if (ROBOT_GRASP_REAL) {
        for (const command of ['EmergencyStop()', 'Stop()']) {
            try {
                const response = await sendDobotDashboardCommand(command);
                successfulCommands += 1;
                log(`🛑 DOBOT ${command} 响应: ${response}`);
            } catch (error) {
                errors.push(error.message);
                log(`🛑 DOBOT ${command} 失败: ${error.message}`);
            }
        }
        const worker = robotWorker;
        if (worker && worker.exitCode === null) {
            worker.kill('SIGTERM');
        }
    }

    if (chassisNavigationPromise) {
        try {
            await chassisClient().request('/api/move/cancel');
            log('🛑 底盘当前导航任务已取消');
        } catch (error) {
            errors.push(error.message);
            log(`🛑 底盘取消导航失败: ${error.message}`);
        }
    }

    if (errors.length && successfulCommands === 0 && ROBOT_GRASP_REAL) {
        throw new Error(errors.join('；'));
    }
    if (errors.length) log(`🛑 急停部分指令失败，但已有 ${successfulCommands} 条停止指令成功发送`);
    broadcastRobotStatus('emergency-stopped', '急停已触发，请人工检查并重启后端');
    return { mode: ROBOT_GRASP_REAL ? 'real' : 'simulated', detail: '急停指令已发送' };
}

async function powerOffRobotWorker() {
    robotPoweredOff = true;
    robotWorkerReady = false;

    if (!ROBOT_GRASP_REAL) {
        broadcastRobotStatus('powered-off', '模拟抓取程序已关闭');
        return { mode: 'simulated' };
    }

    const worker = robotWorker;
    if (!worker || worker.exitCode !== null) {
        broadcastRobotStatus('powered-off', '机器人抓取程序已关闭');
        return { mode: 'real', detail: 'worker 已退出' };
    }

    robotPoweroffPromise = new Promise((resolve, reject) => {
        let settled = false;
        const finish = (error) => {
            if (settled) return;
            settled = true;
            clearTimeout(timeout);
            worker.off('close', onClose);
            worker.off('error', onError);
            if (error) reject(error);
            else resolve({ mode: 'real', detail: '硬件已断电，worker 已退出' });
        };
        const onClose = (code) => {
            if (code === 0) finish();
            else finish(new Error(`机器人断电时 worker 异常退出，退出码 ${code}`));
        };
        const onError = (error) => finish(error);
        const timeout = setTimeout(() => {
            finish(new Error(`机器人断电关闭超时 (${ROBOT_STOW_TIMEOUT_MS}ms)`));
        }, ROBOT_STOW_TIMEOUT_MS);

        worker.once('close', onClose);
        worker.once('error', onError);
        worker.stdin.write('__poweroff__\n', (error) => {
            if (error) finish(new Error(`无法发送断电关闭命令: ${error.message}`));
        });
    });

    broadcastRobotStatus('powering-off', '正在断开灵巧手并关闭 DOBOT 使能');
    try {
        return await robotPoweroffPromise;
    } finally {
        robotPoweroffPromise = null;
        broadcastRobotStatus('powered-off', '机器人已断电，抓取程序已关闭');
    }
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
