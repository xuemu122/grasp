window.GRASP_TEST_MODE = Boolean(window.GRASP_TEST_MODE)
    || window.location.pathname.endsWith('/test.html')
    || new URLSearchParams(window.location.search).get('mode') === 'free';
const GRASP_API_BASE = window.GRASP_TEST_MODE ? '/test-api' : '/api';
const graspApi = (pathname) => `${GRASP_API_BASE}${pathname}`;

// ========== 全局状态 ==========
let products = [];           // 所有商品
let categories = [];         // 所有分类
let cart = {};               // { productId: qty }
let currentCategory = 'all'; // 当前选中的分类
let searchKeyword = '';      // 搜索关键词
let currentOrderId = null;
let currentOrderToken = null;
let currentOrderItems = [];
let currentOrderPayload = null;
let orderRetryTimer = null;
let shelfLayers = {};        // 货架层位姿缓存 { "1": {pose:[...], name:"第1层"}, ... }

// ========== 商家 / 门店 / 设备上下文 ==========
let orderMerchant = null;        // 当前登录的商家资料（含 stores）
let orderStoreList = [];         // [{ id, name }]
let orderDeviceList = [];        // 该商家各门店的设备
let orderSelectedStoreId = '';
let orderSelectedDeviceId = '';
let orderLoginResolve = null;    // 登录成功后的 resolve
let orderAuthMode = 'merchant';  // 'merchant' | 'admin'（test.html 用管理员登录）

// ========== WebSocket 连接 ==========
let ws = null;
let wsConnected = false;
let robotAcceptingOrders = false;
let catalogRefreshPromise = null;
let robotConnectionProbe = null;
let wsReconnectTimer = null;
let wsHeartbeatTimer = null;
let wsDisconnectStatusTimer = null;
let wsGeneration = 0;
let wsConnectedAt = 0;
let wsLastPongAt = 0;

const WS_CONNECT_TIMEOUT_MS = 8000;
const WS_HEARTBEAT_INTERVAL_MS = 15000;
const WS_STALE_TIMEOUT_MS = 45000;
const WS_DISCONNECT_GRACE_MS = 5000;
const WS_ACTION_PROBE_TIMEOUT_MS = 5000;
const WS_ACTION_RECONNECT_TIMEOUT_MS = 8000;
const WS_BACKGROUND_PROBE_TIMEOUT_MS = 5000;

function settleRobotConnectionProbe(connected) {
    if (!robotConnectionProbe) return;
    const probe = robotConnectionProbe;
    robotConnectionProbe = null;
    clearTimeout(probe.timeout);
    if (connected && probe.socket === ws) wsLastPongAt = Date.now();
    probe.resolve(connected);
}

function verifyRobotConnection(timeoutMs = WS_ACTION_PROBE_TIMEOUT_MS) {
    if (!wsConnected || !ws || ws.readyState !== WebSocket.OPEN) return Promise.resolve(false);
    if (robotConnectionProbe && robotConnectionProbe.socket === ws) return robotConnectionProbe.promise;

    const socket = ws;
    const generation = wsGeneration;
    let resolveProbe;
    const promise = new Promise((resolve) => { resolveProbe = resolve; });
    const timeout = setTimeout(() => {
        if (robotConnectionProbe && robotConnectionProbe.socket === socket) {
            settleRobotConnectionProbe(false);
            resetWebSocketConnection(socket, generation, 0);
        }
    }, timeoutMs);
    robotConnectionProbe = { socket, promise, resolve: resolveProbe, timeout };
    try {
        socket.send(JSON.stringify({ type: 'ping', timestamp: Date.now() }));
    } catch (error) {
        console.error('机器人连接探测发送失败:', error);
        settleRobotConnectionProbe(false);
        resetWebSocketConnection(socket, generation, 0);
    }
    return promise;
}

function clearDisconnectedStatusTimer() {
    if (wsDisconnectStatusTimer) clearTimeout(wsDisconnectStatusTimer);
    wsDisconnectStatusTimer = null;
}

function scheduleDisconnectedStatus() {
    if (wsDisconnectStatusTimer) return;
    wsDisconnectStatusTimer = setTimeout(() => {
        wsDisconnectStatusTimer = null;
        if (!wsConnected) updateConnectionStatus('disconnected');
    }, WS_DISCONNECT_GRACE_MS);
}

function waitForReconnect(timeoutMs = WS_ACTION_RECONNECT_TIMEOUT_MS) {
    ensureWebSocketConnection();
    const deadline = Date.now() + timeoutMs;
    return new Promise((resolve) => {
        const check = () => {
            if (wsConnected && ws && ws.readyState === WebSocket.OPEN) {
                resolve(true);
                return;
            }
            if (Date.now() >= deadline) {
                resolve(false);
                return;
            }
            setTimeout(check, 200);
        };
        check();
    });
}

async function verifyRobotConnectionForAction() {
    if (await verifyRobotConnection(WS_ACTION_PROBE_TIMEOUT_MS)) return true;
    if (!(await waitForReconnect())) return false;
    return verifyRobotConnection(WS_ACTION_PROBE_TIMEOUT_MS);
}

function scheduleWebSocketReconnect(delayMs = 3000) {
    if (wsReconnectTimer) {
        if (delayMs > 0) return;
        clearTimeout(wsReconnectTimer);
        wsReconnectTimer = null;
    }
    wsReconnectTimer = setTimeout(() => {
        wsReconnectTimer = null;
        connectWebSocket();
    }, delayMs);
}

function stopWebSocketHeartbeat() {
    if (wsHeartbeatTimer) clearInterval(wsHeartbeatTimer);
    wsHeartbeatTimer = null;
}

function resetWebSocketConnection(socket, generation, delayMs = 0) {
    if (ws !== socket || wsGeneration !== generation) return;
    settleRobotConnectionProbe(false);
    stopWebSocketHeartbeat();
    wsConnected = false;
    wsConnectedAt = 0;
    wsLastPongAt = 0;
    ws = null;
    if ([WebSocket.CONNECTING, WebSocket.OPEN].includes(socket.readyState)) {
        try {
            socket.close();
        } catch (error) {
            console.error('关闭失效 WebSocket 失败:', error);
        }
    }
    scheduleDisconnectedStatus();
    scheduleWebSocketReconnect(delayMs);
}

function startWebSocketHeartbeat(socket, generation) {
    stopWebSocketHeartbeat();
    wsHeartbeatTimer = setInterval(() => {
        if (ws !== socket || wsGeneration !== generation || socket.readyState !== WebSocket.OPEN) {
            stopWebSocketHeartbeat();
            return;
        }
        verifyRobotConnection(WS_BACKGROUND_PROBE_TIMEOUT_MS).then((connected) => {
            if (!connected) resetWebSocketConnection(socket, generation, 0);
        });
    }, WS_HEARTBEAT_INTERVAL_MS);
}

function ensureWebSocketConnection() {
    if (!ws || [WebSocket.CLOSING, WebSocket.CLOSED].includes(ws.readyState)) {
        scheduleWebSocketReconnect(0);
        return;
    }
    const now = Date.now();
    if (ws.readyState === WebSocket.CONNECTING) {
        if (wsConnectedAt && now - wsConnectedAt > WS_CONNECT_TIMEOUT_MS) {
            resetWebSocketConnection(ws, wsGeneration, 0);
        }
        return;
    }
    if (ws.readyState === WebSocket.OPEN) {
        const socket = ws;
        if (wsLastPongAt && now - wsLastPongAt > WS_STALE_TIMEOUT_MS) {
            logWS('⚠️ WebSocket 长时间无响应，正在重建连接');
            resetWebSocketConnection(socket, wsGeneration, 0);
            return;
        }
        verifyRobotConnection(WS_BACKGROUND_PROBE_TIMEOUT_MS).then((connected) => {
            if (!connected && ws === socket) {
                resetWebSocketConnection(socket, wsGeneration, 0);
            }
        });
    }
}

function syncOrderButton() {
    const btnSubmit = document.getElementById('btnSubmit');
    if (!btnSubmit) return;
    const locked = !wsConnected || !robotAcceptingOrders || Boolean(currentOrderId);
    btnSubmit.disabled = locked;
    btnSubmit.textContent = currentOrderId
        ? '机器人运动中...'
        : (!wsConnected ? '后端未连接' : (robotAcceptingOrders ? '开始拣选' : '机器人移动中...'));
}

function connectWebSocket() {
    if (ws && [WebSocket.OPEN, WebSocket.CONNECTING].includes(ws.readyState)) return;
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const storeQuery = orderSelectedStoreId ? `?store=${encodeURIComponent(orderSelectedStoreId)}` : '';
    const socket = new WebSocket(`${wsProtocol}//${window.location.host}/ws/${storeQuery}`);
    const generation = ++wsGeneration;
    wsConnectedAt = Date.now();
    const connectTimeout = setTimeout(() => {
        if (ws === socket && wsGeneration === generation && socket.readyState === WebSocket.CONNECTING) {
            logWS('⚠️ WebSocket 握手超时，正在重试');
            resetWebSocketConnection(socket, generation, 0);
        }
    }, WS_CONNECT_TIMEOUT_MS);
    ws = socket;

    socket.onopen = () => {
        if (ws !== socket || wsGeneration !== generation) return;
        clearTimeout(connectTimeout);
        clearDisconnectedStatusTimer();
        wsConnected = true;
        wsConnectedAt = Date.now();
        wsLastPongAt = wsConnectedAt;
        logWS('✅ 已连接到盒子后端');
        updateConnectionStatus('backend');
        startWebSocketHeartbeat(socket, generation);
    };

    socket.onmessage = (event) => {
        if (ws !== socket || wsGeneration !== generation) return;
        try {
            handleWSMessage(JSON.parse(event.data));
        } catch (error) {
            console.error('WebSocket 消息解析失败:', error);
        }
    };

    socket.onclose = () => {
        if (ws !== socket || wsGeneration !== generation) return;
        clearTimeout(connectTimeout);
        logWS('❌ 与后端短暂断开，正在重连...');
        resetWebSocketConnection(socket, generation, 0);
    };

    socket.onerror = () => {
        if (ws !== socket || wsGeneration !== generation) return;
        clearTimeout(connectTimeout);
        logWS('⚠️ WebSocket 连接错误');
        resetWebSocketConnection(socket, generation, 1000);
    };
}

function logWS(msg) {
    console.log(`[WS] ${msg}`);
}

function updateConnectionStatus(status, message = '') {
    const robotStatus = document.getElementById('robotStatus');
    if (!robotStatus) return;
    robotAcceptingOrders = !['busy', 'stowing', 'chassis-moving', 'returning'].includes(status);
    if (status === 'backend' || status === 'standby' || status === 'idle') {
        robotStatus.innerHTML = '<span class="status-dot busy"></span> 后端在线，机械臂待启动';
    } else if (status === 'ready') {
        robotStatus.innerHTML = '<span class="status-dot online"></span> 机器人就绪';
    } else if (status === 'busy' || status === 'stowing') {
        robotStatus.innerHTML = `<span class="status-dot busy"></span> ${message || (status === 'stowing' ? '机械臂收起中' : '机器人执行中')}`;
    } else if (status === 'chassis-moving' || status === 'returning') {
        robotStatus.innerHTML = `<span class="status-dot busy"></span> ${message || (status === 'chassis-moving' ? '底盘运动中' : '订单收尾/回充中')}`;
    } else if (status === 'simulated') {
        robotStatus.innerHTML = '<span class="status-dot busy"></span> 模拟后端在线';
    } else {
        robotStatus.innerHTML = '<span class="status-dot" style="background:#EF4444;box-shadow:0 0 8px rgba(239,68,68,0.6)"></span> 后端未连接';
    }
    syncOrderButton();
}

function handleWSMessage(msg) {
    switch (msg.type) {
        case 'connected':
            logWS(msg.message);
            updateConnectionStatus(msg.robotStatus || 'backend');
            break;
        case 'robot_status':
            updateConnectionStatus(msg.status || 'backend', msg.message || '');
            if (msg.message) logWS(msg.message);
            break;
        case 'status':
            if (msg.status === 'started') {
                updateRobotStatus('busy', msg.message || '机器人抓取中...');
            } else if (msg.status === 'grabbing') {
                updateGrabProgress(msg);
            } else if (msg.status === 'returning') {
                updateRobotStatus('busy', msg.message || '订单收尾中...');
                updateGrabProgress(msg);
                const modalTitle = document.querySelector('#modalContent .modal-title');
                const robotAnim = document.getElementById('robotAnim');
                if (modalTitle) {
                    if (msg.phase === 'settling') modalTitle.textContent = '机器人已到充电桩';
                    else if (msg.phase === 'stowed') modalTitle.textContent = '机械臂已收起';
                    else if (msg.phase === 'charging') modalTitle.textContent = '机器人正在回充...';
                    else modalTitle.textContent = '机械臂正在收起...';
                }
                if (robotAnim) robotAnim.textContent = ['stowing', 'stowed'].includes(msg.phase) ? '🤖' : '🚚';
            } else if (msg.status === 'layer-moving') {
                updateRobotStatus('busy', msg.message || `移动中: 第${msg.layer}层`);
                updateGrabProgress(msg);
            } else if (msg.status === 'layer-reached') {
                updateRobotStatus('ready', msg.message || `已到达第${msg.layer}层`);
            } else if (msg.status === 'order-ready') {
                if (msg.orderId && currentOrderId && msg.orderId === currentOrderId) {
                    onOrderReady(msg);
                }
            } else if (msg.status === 'completed') {
                if (msg.orderId && currentOrderId && msg.orderId === currentOrderId) {
                    onGrabCompleted(msg);
                } else {
                    onStowCompleted(msg);
                }
            } else if (msg.status === 'error') {
                alert('抓取失败: ' + msg.message);
                if (msg.retryable) scheduleOrderRetry(msg.message);
                else onGrabFailed(msg.message, msg.refundTriggered !== true);
            }
            break;
        case 'error':
            if (msg.retryable) {
                scheduleOrderRetry(msg.message);
            } else if (/盒子\s*WebSocket|ECONNREFUSED|connect\s+.*:506\d/.test(String(msg.message || ''))) {
                // 云端转发到盒子失败（盒子离线）：只更新状态，不弹窗打断
                logWS(`盒子离线: ${msg.message}`);
                updateConnectionStatus('offline', msg.message);
            } else {
                alert('后端错误: ' + msg.message);
                onGrabFailed(msg.message);
            }
            break;
        case 'pong':
            settleRobotConnectionProbe(true);
            break;
    }
}

// ========== 熄屏唤醒 ==========
(function initSplash() {
    const splash = document.getElementById('splashScreen');
    if (!splash) return;

    let awake = false;
    let idleTimer = null;
    const IDLE_TIMEOUT = 120000; // 120秒无操作返回待机页

    function goToSleep() {
        awake = false;
        splash.classList.remove('hidden');
        // 停止语音播报
        stopSpeak();
        // 取消语音筛选计时器
        cancelVoiceResetTimer();
        if (idleTimer) {
            clearTimeout(idleTimer);
            idleTimer = null;
        }
        removeIdleListeners();
    }

    function resetIdleTimer() {
        if (idleTimer) clearTimeout(idleTimer);
        idleTimer = setTimeout(goToSleep, IDLE_TIMEOUT);
    }

    function onUserActivity() {
        if (awake) resetIdleTimer();
    }

    function addIdleListeners() {
        document.addEventListener('mousedown', onUserActivity);
        document.addEventListener('touchstart', onUserActivity, { passive: true });
        document.addEventListener('keydown', onUserActivity);
        document.addEventListener('scroll', onUserActivity, { passive: true });
    }

    function removeIdleListeners() {
        document.removeEventListener('mousedown', onUserActivity);
        document.removeEventListener('touchstart', onUserActivity);
        document.removeEventListener('keydown', onUserActivity);
        document.removeEventListener('scroll', onUserActivity);
    }

    function wakeUp() {
        if (awake) return;
        awake = true;
        splash.classList.add('hidden');

        // 启动空闲计时器
        resetIdleTimer();
        addIdleListeners();

        speakText('欢迎光临，请点单!');
    }

    splash.addEventListener('click', wakeUp);
    splash.addEventListener('touchend', (e) => {
        e.preventDefault();
        wakeUp();
    });
})();

// ========== 初始化 ==========
async function init() {
    syncModalViewport();
    window.addEventListener('resize', syncModalViewport);
    window.addEventListener('orientationchange', () => setTimeout(syncModalViewport, 100));
    if (window.visualViewport) {
        window.visualViewport.addEventListener('resize', syncModalViewport);
        window.visualViewport.addEventListener('scroll', syncModalViewport);
    }
    syncOrderButton();
    const loginForm = document.getElementById('orderLoginForm');
    if (loginForm) loginForm.addEventListener('submit', submitOrderLogin);
    if (window.GRASP_TEST_MODE) {
        orderAuthMode = 'admin';
        await ensureAdminTestContext();
    } else {
        await ensureOrderContext();
    }
    connectWebSocket();
    await refreshCatalog();
    bindSearch();
    window.addEventListener('focus', () => {
        ensureWebSocketConnection();
        refreshCatalog();
    });
    window.addEventListener('pageshow', ensureWebSocketConnection);
    window.addEventListener('online', ensureWebSocketConnection);
    document.addEventListener('visibilitychange', () => {
        if (!document.hidden) {
            ensureWebSocketConnection();
            refreshCatalog();
        }
    });
    setInterval(() => {
        if (!document.hidden) refreshCatalog();
    }, 15000);
    refreshPowerStatus();
    setInterval(refreshPowerStatus, 15000);
}

async function refreshCatalog() {
    if (catalogRefreshPromise) return catalogRefreshPromise;
    const refresh = Promise.all([
        loadShelfLayers(),
        loadCategories(false),
        loadProducts(false),
    ]).then(() => {
        updateCartUI();
        renderCategories();
        renderProducts();
    });
    catalogRefreshPromise = refresh;
    try {
        return await refresh;
    } finally {
        if (catalogRefreshPromise === refresh) catalogRefreshPromise = null;
    }
}

// ========== 加载分类 ==========
async function loadCategories(render = true) {
    try {
        const res = await fetch(graspApi('/categories'));
        const data = await res.json();
        if (data.code === 0) {
            categories = data.data || [];
        }
    } catch (e) {
        console.error('加载分类失败:', e);
        categories = [];
    }
    if (render) renderCategories();
}

// ========== 加载商品 ==========
async function loadProducts(render = true) {
    try {
        const storeQuery = orderSelectedStoreId ? `?storeId=${encodeURIComponent(orderSelectedStoreId)}` : '';
        const res = await fetch(graspApi(`/products${storeQuery}`));
        const data = await res.json();
        if (data.code === 0) {
            products = data.data || [];
            const productMap = new Map(products.map(product => [String(product.id), product]));
            for (const [id, qty] of Object.entries(cart)) {
                const product = productMap.get(String(id));
                if (!product || Number(product.status) !== 1 || Number(product.stock) <= 0) {
                    delete cart[id];
                } else if (qty > Number(product.stock)) {
                    cart[id] = Number(product.stock);
                }
            }
        }
    } catch (e) {
        console.error('加载商品失败:', e);
        products = [];
    }
    if (render) {
        updateCartUI();
        renderCategories();
        renderProducts();
    }
}

// ========== 渲染分类导航 ==========
function renderCategories() {
    const list = document.getElementById('categoryList');
    if (!list) return;

    const categoryCounts = new Map();
    for (const product of products) {
        categoryCounts.set(product.category, (categoryCounts.get(product.category) || 0) + 1);
    }
    const items = [
        { key: 'all', label: '全部商品' },
        ...categories.map(c => ({ key: c, label: c }))
    ];

    list.innerHTML = items.map(item => `
        <div class="category-item ${item.key === currentCategory ? 'active' : ''}"
             data-category="${item.key}"
             onclick="selectCategory('${item.key}')">
            <span class="cat-name">${item.label}</span>
            <span class="cat-count">${item.key === 'all' ? products.length : (categoryCounts.get(item.key) || 0)}</span>
        </div>
    `).join('');
}

// ========== 选择分类 ==========
function selectCategory(category) {
    currentCategory = category;
    renderCategories();
    renderProducts();

    // 更新标题
    const title = document.getElementById('currentCategoryTitle');
    if (title) {
        title.textContent = category === 'all' ? '全部商品' : category;
    }
}

// ========== 搜索绑定 ==========
function bindSearch() {
    const input = document.getElementById('searchInput');
    if (!input) return;
    input.addEventListener('input', (e) => {
        searchKeyword = e.target.value.trim().toLowerCase();
        // 用户手动搜索时取消自动回全部商品，避免打断输入
        cancelVoiceResetTimer();
        renderProducts();
    });
    input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') renderProducts();
    });
}

function handleSearch() {
    const input = document.getElementById('searchInput');
    if (input) {
        searchKeyword = input.value.trim().toLowerCase();
        renderProducts();
    }
}

// ========== 获取过滤后的商品 ==========
// 商品是否命中关键词（支持空格分隔的多词，需全部命中）
function productMatchesKeywords(product, keyword) {
    const words = (keyword || '').toLowerCase().split(/\s+/).filter(Boolean);
    if (words.length === 0) return false;
    const name = (product.name || '').toLowerCase();
    return words.every(w => name.includes(w));
}

function getFilteredProducts() {
    return products.filter(p => {
        const matchCategory = currentCategory === 'all' || p.category === currentCategory;
        const words = searchKeyword ? searchKeyword.toLowerCase().split(/\s+/).filter(Boolean) : [];
        const name = (p.name || '').toLowerCase();
        const matchSearch = words.length === 0 || words.every(w => name.includes(w));
        return matchCategory && matchSearch;
    });
}

// ========== 渲染商品列表 ==========
function renderProducts() {
    const grid = document.getElementById('productsGrid');
    const countEl = document.getElementById('productsCount');
    if (!grid) return;

    const filtered = getFilteredProducts();
    if (countEl) countEl.textContent = `共 ${filtered.length} 件商品`;

    if (filtered.length === 0) {
        grid.innerHTML = `
            <div class="empty-products">
                <span class="empty-icon">🔍</span>
                <p>未找到相关商品</p>
            </div>
        `;
        return;
    }

    grid.innerHTML = filtered.map(p => {
        const qty = cart[p.id] || 0;
        const soldOut = Number(p.stock) <= 0;
        const code = p.location ? String(p.location).replace(/\s/g, '') : ('#' + String(p.id).padStart(12, '0'));
        const layerIndex = parseLayerFromLocation(p.location);
        const locationLabel = layerIndex
            ? (shelfLayers[String(layerIndex)] ? '📍 第' + layerIndex + '层' : '📍 层' + layerIndex)
            : (p.location || '');
        return `
            <div class="product-card ${qty > 0 ? 'selected' : ''} ${soldOut ? 'sold-out' : ''}" id="card-${p.id}"
                 title="${p.location ? '货架：' + p.location : p.name}">
                <div class="product-image">
                    ${p.photo_front ? `<img src="${p.photo_front}" alt="${p.name}" loading="lazy">` : `<span class="product-emoji">${p.emoji || '📦'}</span>`}
                    ${qty > 0 ? `<span class="product-badge">${qty}</span>` : ''}
                </div>
                <div class="product-code">${code}</div>
                <div class="product-name" title="${p.name}">${p.name}</div>
                <div class="product-spec">${p.spec || ''}</div>
                <div class="product-manufacturer">${p.manufacturer || ''}</div>
                <div class="product-location">${locationLabel}</div>
                <div class="product-footer">
                    <div class="product-price">
                        <span class="price-symbol">¥</span>${Number(p.price || 0).toFixed(2)}
                    </div>
                    <button class="btn-add" onclick="addToCart('${p.id}', event)" ${soldOut ? 'disabled' : ''}>
                        ${soldOut ? '售罄' : (qty > 0 ? '+' : '添加')}
                    </button>
                </div>
            </div>
        `;
    }).join('');
}

// ========== 加入购物车 ==========
function addToCart(productId, event) {
    if (event) event.stopPropagation();
    const product = products.find(p => String(p.id) === String(productId));
    if (!product || Number(product.stock) <= Number(cart[productId] || 0)) {
        alert('该商品库存不足');
        return;
    }
    if (!cart[productId]) cart[productId] = 0;
    cart[productId] += 1;
    updateCartUI();
    renderProducts();
}

// ========== 修改数量 ==========
function changeQty(productId, delta, event) {
    if (event) event.stopPropagation();
    if (!cart[productId]) return;
    if (delta > 0) {
        const product = products.find((item) => String(item.id) === String(productId));
        const stock = Number(product?.stock);
        if (!product || !Number.isFinite(stock) || cart[productId] >= stock) {
            alert('该商品库存不足');
            return;
        }
    }
    cart[productId] += delta;
    if (cart[productId] <= 0) delete cart[productId];
    updateCartUI();
    renderProducts();
}

// ========== 移除商品 ==========
function removeItem(productId) {
    delete cart[productId];
    updateCartUI();
    renderProducts();
}

// ========== 清空购物车 ==========
function clearCart() {
    if (Object.keys(cart).length === 0) return;
    cart = {};
    updateCartUI();
    renderProducts();
}

function toggleCart(event) {
    if (event) event.stopPropagation();
    const sidebar = document.getElementById('cartSidebar');
    if (!sidebar) return;
    const expanded = sidebar.classList.toggle('is-open');
    const header = sidebar.querySelector('.cart-header');
    if (header) {
        header.setAttribute('aria-expanded', String(expanded));
        header.setAttribute('aria-label', expanded ? '收起购物车' : '展开购物车');
    }
}

function handleCartKeydown(event) {
    if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        toggleCart(event);
    }
}

document.addEventListener('click', (event) => {
    const sidebar = document.getElementById('cartSidebar');
    const clickPath = typeof event.composedPath === 'function' ? event.composedPath() : [];
    if (!sidebar || sidebar.contains(event.target) || clickPath.includes(sidebar)) return;
    sidebar.classList.remove('is-open');
    const header = sidebar.querySelector('.cart-header');
    if (header) {
        header.setAttribute('aria-expanded', 'false');
        header.setAttribute('aria-label', '展开购物车');
    }
});

// ========== 获取购物车列表 ==========
function getCartItems() {
    return Object.entries(cart).map(([id, qty]) => {
        const product = products.find(p => String(p.id) === id);
        return product ? { ...product, qty } : null;
    }).filter(Boolean);
}

// ========== 计算总价 ==========
function getTotalPrice() {
    return Object.entries(cart).reduce((sum, [id, qty]) => {
        const product = products.find(p => String(p.id) === id);
        return sum + (product ? product.price * qty : 0);
    }, 0);
}

// ========== 计算总件数 ==========
function getTotalQty() {
    return Object.values(cart).reduce((sum, qty) => sum + qty, 0);
}

// ========== 更新购物车UI ==========
function updateCartUI() {
    const cartItemsDiv = document.getElementById('cartItems');
    const cartFooter = document.getElementById('cartFooter');
    const totalPriceEl = document.getElementById('totalPrice');
    const cartQtyEl = document.getElementById('cartQty');

    const items = getCartItems();

    if (items.length === 0) {
        cartItemsDiv.innerHTML = `
            <div class="cart-empty">
                <span class="empty-icon">🛒</span>
                <p>购物车是空的</p>
                <p class="empty-hint">点击商品添加</p>
            </div>
        `;
        cartFooter.style.display = 'none';
    } else {
        cartItemsDiv.innerHTML = items.map(item => `
            <div class="cart-item">
                <div class="cart-item-img">
                    ${item.photo_front ? `<img src="${item.photo_front}" alt="${item.name}">` : `<span>${item.emoji || '📦'}</span>`}
                </div>
                <div class="cart-item-info">
                    <div class="cart-item-name" title="${item.name}">${item.name}</div>
                    <div class="cart-item-spec">${item.spec || ''}</div>
                    <div class="cart-item-price">¥${Number(item.price).toFixed(2)}</div>
                </div>
                <div class="cart-item-controls">
                    <button type="button" class="btn-qty" aria-label="减少商品数量" onclick="changeQty('${item.id}', -1, event)">−</button>
                    <span class="qty-num">${item.qty}</span>
                    <button type="button" class="btn-qty" aria-label="增加商品数量" onclick="changeQty('${item.id}', 1, event)">+</button>
                </div>
            </div>
        `).join('');
        cartFooter.style.display = 'block';
    }

    if (totalPriceEl) totalPriceEl.textContent = '¥' + getTotalPrice().toFixed(2);
    if (cartQtyEl) cartQtyEl.textContent = getTotalQty();

    // 更新浮窗徽章
    const badge = document.getElementById('cartBadge');
    const qty = getTotalQty();
    if (badge) {
        badge.textContent = qty;
        badge.setAttribute('data-count', qty);
    }
}

// ========== 提交订单 ==========
async function submitOrder() {
    const items = getCartItems();
    if (items.length === 0) return;

    const btnSubmit = document.getElementById('btnSubmit');
    btnSubmit.disabled = true;
    btnSubmit.textContent = '正在确认连接...';

    if (!(await verifyRobotConnectionForAction())) {
        alert('后端未连接，暂时无法下单');
        syncOrderButton();
        return;
    }

    if (!robotAcceptingOrders) {
        alert('机器人正在移动或回充，请稍后再下单');
        return;
    }

    btnSubmit.disabled = true;
    btnSubmit.textContent = '⏳ 提交中...';

    let prepared;
    try {
        const res = await fetch(graspApi('/orders/prepare'), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                items: items.map(item => ({ id: item.id, name: item.name, qty: item.qty })),
                source: window.GRASP_TEST_MODE ? 'test' : 'web',
                payment_mode: window.GRASP_TEST_MODE ? 'free' : undefined,
                storeId: orderSelectedStoreId || undefined,
                deviceId: orderSelectedDeviceId || undefined
            })
        });
        prepared = await res.json();
        if (!res.ok || prepared.code !== 0) throw new Error(prepared.message || '订单校验失败');
    } catch (error) {
        alert(`订单提交失败: ${error.message}`);
        syncOrderButton();
        return;
    }

    if (!(await verifyRobotConnectionForAction())) {
        if (prepared.paymentRequired) {
            try {
                await fetch(graspApi(`/orders/${encodeURIComponent(prepared.orderId)}/cancel-payment`), {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ completionToken: prepared.completionToken })
                });
            } catch (error) {
                console.error('后端断线后的支付关单失败:', error);
            }
        } else {
            await requestAutomaticRefund(
                prepared.orderId,
                prepared.completionToken,
                '订单创建期间后端断开，机器人未开始执行'
            );
        }
        alert('后端连接已断开，订单未开始执行');
        syncOrderButton();
        return;
    }

    const orderId = prepared.orderId;
    currentOrderId = orderId;
    currentOrderToken = prepared.completionToken;
    currentOrderItems = items.map(item => ({ ...item }));
    syncOrderButton();

    if (prepared.paymentRequired) {
        const paid = await waitForPayment(orderId, prepared.completionToken, prepared.payment, prepared.totalAmount);
        if (!paid) {
            currentOrderId = null;
            currentOrderToken = null;
            currentOrderItems = [];
            syncOrderButton();
            return;
        }
    }

    if (!(await verifyRobotConnectionForAction())) {
        alert('⚠️ 后端服务未连接，请先启动后端！\n\n在终端执行: node server.js');
        await requestAutomaticRefund(orderId, prepared.completionToken, '支付成功后机器人连接断开，订单未开始');
        currentOrderId = null;
        currentOrderToken = null;
        currentOrderItems = [];
        syncOrderButton();
        return;
    }

    showModal(orderId, items);

    const preparedItems = new Map((prepared.items || []).map(item => [String(item.id), item]));
    const payload = {
        type: 'grab',
        orderId: orderId,
        completionToken: prepared.completionToken,
        items: items.map(i => {
            const preparedItem = preparedItems.get(String(i.id));
            return {
                id: i.id,
                name: i.name,
                emoji: i.emoji || '📦',
                qty: i.qty,
                price: i.price,
                shelf_layer: preparedItem && preparedItem.shelf_layer != null
                    ? preparedItem.shelf_layer
                    : parseLayerFromLocation(i.location),
                chassis_marker: preparedItem && preparedItem.chassis_marker
                    ? preparedItem.chassis_marker
                    : (i.chassis_marker || null)
            };
        }),
        timestamp: Date.now()
    };
    currentOrderPayload = payload;

    logWS(`📤 发送抓取命令: orderId=${orderId}, items=${payload.items.length}`);
    try {
        ws.send(JSON.stringify(payload));
    } catch (error) {
        await requestAutomaticRefund(orderId, prepared.completionToken, `抓取命令发送失败：${error.message}`);
        onGrabFailed(error.message, false);
    }
}

async function requestAutomaticRefund(orderId, completionToken, reason) {
    if (!orderId || !completionToken) return null;
    try {
        const res = await fetch(graspApi(`/orders/${encodeURIComponent(orderId)}/refund`), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ completionToken, reason })
        });
        const json = await res.json();
        if (!res.ok || json.code !== 0) throw new Error(json.message || '自动退款失败');
        return json;
    } catch (error) {
        alert(`自动退款失败，请联系管理员处理订单 ${orderId}：${error.message}`);
        return null;
    }
}

let paymentCancelled = false;

function syncModalViewport() {
    const overlay = document.getElementById('modalOverlay');
    if (!overlay) return;
    const viewport = window.visualViewport;
    if (!viewport) {
        overlay.style.inset = '0';
        overlay.style.width = 'auto';
        overlay.style.height = 'auto';
        return;
    }
    overlay.style.inset = 'auto';
    overlay.style.left = `${viewport.offsetLeft}px`;
    overlay.style.top = `${viewport.offsetTop}px`;
    overlay.style.width = `${viewport.width}px`;
    overlay.style.height = `${viewport.height}px`;
}

function openModalOverlay() {
    // 弹窗（支付/抓取）打开时取消自动回全部商品
    cancelVoiceResetTimer();
    const overlay = document.getElementById('modalOverlay');
    syncModalViewport();
    overlay.style.display = 'flex';
}

function showPaymentModal(orderId, payment, totalAmount) {
    const overlay = document.getElementById('modalOverlay');
    const content = document.getElementById('modalContent');
    const qrImageUrl = payment && (payment.qrImageUrl || payment.qrCodeBase64)
        ? String(payment.qrImageUrl || payment.qrCodeBase64)
        : '';
    paymentCancelled = false;
    content.innerHTML = `
        <div class="modal-title">请使用微信扫码支付</div>
        <div class="modal-text"><span class="order-id-display">订单号：${orderId}</span></div>
        <div class="payment-amount">¥${Number(totalAmount || 0).toFixed(2)}</div>
        <div class="payment-qr-frame">
            <img class="payment-qr-code" id="paymentQrImage" src="${escapeHtml(qrImageUrl)}" alt="微信支付二维码">
        </div>
        <div class="modal-text" id="paymentStatusText">打开微信“扫一扫”完成付款，支付结果会自动确认。</div>
        <div class="payment-actions">
            <button class="btn-secondary" type="button" id="cancelPayment">取消订单</button>
        </div>
    `;
    openModalOverlay();
    const qrImage = document.getElementById('paymentQrImage');
    let qrLoadRetries = 0;
    qrImage.onerror = () => {
        if (qrLoadRetries < 2 && qrImageUrl) {
            qrLoadRetries += 1;
            const separator = qrImageUrl.includes('?') ? '&' : '?';
            setTimeout(() => { qrImage.src = `${qrImageUrl}${separator}retry=${Date.now()}`; }, 800);
            return;
        }
        showPaymentStatus('二维码加载失败，请检查大屏网络后重新下单');
    };
    document.getElementById('cancelPayment').onclick = () => { paymentCancelled = true; overlay.style.display = 'none'; };
    speakText('请扫码支付。');
}

function scheduleOrderRetry(reason) {
    if (!currentOrderId || !currentOrderPayload) return;
    updateRobotStatus('busy', `${reason}，等待机器人空闲后重试`);
    if (orderRetryTimer) clearTimeout(orderRetryTimer);
    orderRetryTimer = setTimeout(() => {
        orderRetryTimer = null;
        if (!currentOrderId || !currentOrderPayload) return;
        if (!wsConnected || !robotAcceptingOrders) {
            scheduleOrderRetry('机器人仍然繁忙');
            return;
        }
        logWS(`🔁 重新发送订单 ${currentOrderId}`);
        try {
            ws.send(JSON.stringify(currentOrderPayload));
        } catch (error) {
            scheduleOrderRetry(`订单重试发送失败：${error.message}`);
        }
    }, 3000);
}

function showPaymentStatus(message) {
    const node = document.getElementById('paymentStatusText');
    if (node) node.textContent = message;
}

function escapeHtml(value) {
    const div = document.createElement('div');
    div.textContent = String(value || '');
    return div.innerHTML;
}

async function waitForPayment(orderId, completionToken, payment, totalAmount) {
    showPaymentModal(orderId, payment, totalAmount);
    const deadline = Date.now() + 10 * 60 * 1000;
    while (!paymentCancelled && Date.now() < deadline) {
        try {
            const res = await fetch(graspApi(`/orders/${encodeURIComponent(orderId)}/payment`), {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                cache: 'no-store',
                body: JSON.stringify({ completionToken })
            });
            const data = await res.json();
            if (data.code === 0 && data.paid) {
                document.getElementById('modalOverlay').style.display = 'none';
                speakText('请等待机器人取货。');
                return true;
            }
            const terminalStatus = ['REFUNDED', 'REFUNDING', 'CLOSED', 'FAILED', 'PAYMENT_FAILED'].includes(data.paymentStatus)
                || ['REFUNDED', 'REFUNDING', 'CLOSED', 'FAILED', 'PAYMENT_FAILED'].includes(data.status);
            if (data.code === 0 && terminalStatus) {
                const message = data.message || '订单未能进入执行状态，请重新下单';
                showPaymentStatus(message);
                await new Promise(resolve => setTimeout(resolve, 1200));
                document.getElementById('modalOverlay').style.display = 'none';
                alert(message);
                return false;
            }
            if (data.message) showPaymentStatus(data.message);
        } catch (error) {
            showPaymentStatus(`支付状态查询失败：${error.message}`);
        }
        await new Promise(resolve => setTimeout(resolve, 2500));
    }
    if (paymentCancelled) {
        try {
            await fetch(graspApi(`/orders/${encodeURIComponent(orderId)}/cancel-payment`), {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                keepalive: true,
                body: JSON.stringify({ completionToken })
            });
        } catch (error) {
            console.error('取消支付状态同步失败:', error);
        }
    } else {
        showPaymentStatus('支付等待超时，系统会继续确认支付状态');
    }
    return false;
}

// ========== 货架层缓存加载 ==========
async function loadShelfLayers() {
    try {
        const res = await fetch(graspApi('/shelf/layers'));
        const data = await res.json();
        if (data.code === 0) {
            shelfLayers = data.data || {};
            logWS(`📋 货架层加载完成: ${Object.keys(shelfLayers).length} 层`);
        }
    } catch (e) {
        console.error('加载货架层数据失败:', e);
    }
}

// ========== 前端版 location 解析（与后端一致）==========
function parseLayerFromLocation(location) {
    if (!location) return null;
    const cnDigits = { '一':1,'二':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9 };
    // "3层" / "三层" / "第3层"
    let m = location.match(/第?(\d+|[一二三四五六七八九])\s*层/);
    if (m) {
        const val = m[1];
        return cnDigits[val] !== undefined ? cnDigits[val] : parseInt(val, 10);
    }
    // "A3货架" → 3
    m = location.match(/[A-Za-z](\d+)/);
    if (m) return parseInt(m[1], 10);
    return null;
}

// ========== 点击商品卡片 → 导航机械臂到对应货架位置 ==========
async function onProductClick(productId, event) {
    // 如果点的是按钮（添加/移除等），不触发导航
    if (event && (event.target.closest('button') || event.target.tagName === 'BUTTON')) return;

    const product = products.find(p => String(p.id) === String(productId));
    if (!product) return;

    if (!wsConnected) {
        alert('后端服务未连接，无法移动机械臂');
        return;
    }

    const layerIndex = parseLayerFromLocation(product.location);
    const hasLayer = layerIndex && shelfLayers[String(layerIndex)];

    const label = hasLayer
        ? `${shelfLayers[String(layerIndex)].name} (${product.location || ''})`
        : (layerIndex ? `第${layerIndex}层（无精确位姿）` : (product.location || product.name));

    logWS(`📍 点击导航: ${product.name} → ${label}`);
    updateRobotStatus('busy', `正在移动到 ${label}...`);

    if (hasLayer) {
        // 有精确货架层位姿，直接 goto-layer
        try {
            await gotoLayerDirect(product.id, layerIndex);
            return;
        } catch (e) {
            console.error('goto-layer 失败, 尝试坐标兜底:', e.message);
        }
    }

    // 兜底：使用商品自身 shelf_x/y/z 坐标
    if (product.shelf_x !== undefined || product.shelf_y !== undefined) {
        await gotoProductXYZ(product, label);
    } else {
        alert(`商品 "${product.name}" 没有货架位置信息，请在管理后台设置。`);
        updateRobotStatus('ready', '机器人就绪');
    }
}

// ========== 直接 HTTP 调用 goto-layer（返回 Promise，失败抛异常）==========
async function gotoLayerDirect(productId, layerIndex) {
    const res = await fetch(graspApi('/robot/goto-layer'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ productId })
    });
    const data = await res.json();
    if (!res.ok || data.code !== 0) throw new Error(data.message || '移动失败');

    const layerName = data.data ? data.data.name : ('第' + layerIndex + '层');
    logWS(`✅ ${layerName} 已到达`);
    updateRobotStatus('ready', `${layerName} 已到达`);
    return data;
}

// ========== 使用商品自身坐标移动（兜底方案）==========
async function gotoProductXYZ(product, label) {
    const pose = [
        Number(product.shelf_x || 0),
        Number(product.shelf_y || 0),
        Number(product.shelf_z || 50),
        Number(product.shelf_rx == null ? 135 : product.shelf_rx),
        Number(product.shelf_ry == null ? 82 : product.shelf_ry),
        Number(product.shelf_rz == null ? -52 : product.shelf_rz)
    ];

    logWS(`📍 坐标兜底移动: [${pose.join(', ')}]`);

    const res = await fetch(graspApi('/robot/movel'), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            x: pose[0], y: pose[1], z: pose[2],
            rx: pose[3], ry: pose[4], rz: pose[5]
        })
    });
    const data = await res.json();
    if (!res.ok || data.code !== 0) throw new Error(data.message || '坐标移动失败');

    logWS(`✅ ${data.message || '已到达目标位姿'}`);
    updateRobotStatus('ready', `已到达 ${label}`);
}

/**
 * 发送 goto-layer 命令让机械臂移动到指定货架层
 * @param {Object} opts - { productName?, productId?, layer? }
 */
async function gotoLayer(opts) {
    if (!wsConnected && !opts.httpOnly) {
        alert('后端服务未连接');
        return;
    }

    // 优先使用 HTTP API（更通用）
    try {
        const res = await fetch(graspApi('/robot/goto-layer'), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(opts)
        });
        const data = await res.json();
        if (res.ok && data.code === 0) {
            logWS(`✅ ${data.message}`);
            updateRobotStatus('ready', data.message);
            return data;
        }
        throw new Error(data.message || '移动失败');
    } catch (e) {
        console.error('goto-layer HTTP失败:', e.message);
        // 回退到 WebSocket
        if (wsConnected) {
            ws.send(JSON.stringify({ type: 'goto-layer', ...opts, timestamp: Date.now() }));
        } else {
            throw e;
        }
    }
}

function onStowCompleted(msg) {
    updateConnectionStatus('standby');
    logWS(`✅ ${msg.message || '机械臂已收起'}`);
}

// ========== 更新抓取进度 ==========
function updateGrabProgress(msg) {
    const progressBar = document.getElementById('progressBar');
    const progressText = document.getElementById('progressText');

    if (progressBar) progressBar.style.width = Math.round(msg.progress || 0) + '%';
    if (progressText) progressText.textContent = msg.message || '执行中...';
}

// ========== 抓取完成 ==========
async function onGrabCompleted(msg) {
    const progressBar = document.getElementById('progressBar');
    const progressText = document.getElementById('progressText');
    const robotAnim = document.getElementById('robotAnim');
    const modalTitle = document.querySelector('#modalContent .modal-title');

    if (progressBar) progressBar.style.width = '100%';
    if (modalTitle) modalTitle.textContent = '商品已放置，机器人正在回充...';
    if (progressText) progressText.textContent = '✅ 订单已完成，机器人运动中，请稍候...';
    if (robotAnim) {
        robotAnim.classList.add('moving');
        robotAnim.textContent = '🚚';
    }
    updateRobotStatus('busy', '商品已放置，机器人正在回充...');

    try {
        await loadProducts();
    } catch (error) {
        console.error('订单已完成，但商品列表刷新失败:', error);
    }

}

function onOrderReady(msg) {
    const overlay = document.getElementById('modalOverlay');
    const items = currentOrderItems.map(item => ({ ...item }));
    if (overlay) overlay.style.display = 'none';
    updateRobotStatus('online', msg.message || '机器人已回到充电桩');
    showOrderToast(msg.orderId, items);
    cart = {};
    currentOrderId = null;
    currentOrderToken = null;
    currentOrderItems = [];
    currentOrderPayload = null;
    if (orderRetryTimer) clearTimeout(orderRetryTimer);
    orderRetryTimer = null;
    updateCartUI();
    renderProducts();
    syncOrderButton();
    speakText('出货完成，请核对商品是否已经全部取出，欢迎下次光临！');
}

// ========== 抓取失败 ==========
async function onGrabFailed(reason = '机器人未能完成抓取和放置', requestRefund = true) {
    const overlay = document.getElementById('modalOverlay');
    if (overlay) overlay.style.display = 'none';
    updateRobotStatus('online', '机器人就绪');
    const failedOrderId = currentOrderId;
    const failedOrderToken = currentOrderToken;
    if (requestRefund && failedOrderId && failedOrderToken) {
        await requestAutomaticRefund(failedOrderId, failedOrderToken, reason);
    }
    currentOrderId = null;
    currentOrderToken = null;
    currentOrderItems = [];
    currentOrderPayload = null;
    if (orderRetryTimer) clearTimeout(orderRetryTimer);
    orderRetryTimer = null;

    syncOrderButton();
    speakText('机器人取货失败，费用已原路退还。');
}

// ========== 显示抓取弹窗 ==========
function showModal(orderId, items) {
    const overlay = document.getElementById('modalOverlay');
    const content = document.getElementById('modalContent');

    const itemList = items.map(i => `${i.emoji || '📦'} ${i.name} × ${i.qty}`).join('<br>');

    content.innerHTML = `
        <span class="robot-animation moving" id="robotAnim">🤖</span>
        <div class="modal-title">机器人正在抓取中...</div>
        <div class="modal-text">
            <span class="order-id-display">订单号：${orderId}</span><br>
            ${itemList}
        </div>
        <div class="progress-bar-container">
            <div class="progress-bar-fill" id="progressBar" style="width: 0%;"></div>
        </div>
        <div class="modal-text" id="progressText" style="font-weight: 600; color: var(--primary);">
            等待后端响应...
        </div>
    `;

    openModalOverlay();
    updateRobotStatus('busy', '机器人抓取中...');
}

// ========== 显示订单完成提示 ==========
function showOrderToast(orderId, items) {
    const toast = document.getElementById('orderToast');
    const content = document.getElementById('toastContent');
    const total = items.reduce((sum, item) => sum + (item.price || 0) * item.qty, 0);
    const time = new Date().toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

    content.innerHTML = `
        <div class="toast-title">✅ 订单完成</div>
        <div class="toast-order-id">${orderId}</div>
        <ul class="toast-items">
            ${items.map(item => `
                <li>
                    <span>${item.emoji || '📦'} ${item.name} × ${item.qty}</span>
                    <span>¥${((item.price || 0) * item.qty).toFixed(2)}</span>
                </li>
            `).join('')}
        </ul>
        <div class="toast-total">
            <span>合计</span>
            <span>¥${total.toFixed(2)}</span>
        </div>
        <div class="toast-time">${time} · 机器人已完成抓取</div>
    `;

    toast.style.display = 'flex';
    setTimeout(() => toast.style.display = 'none', 5000);
}

// ========== 更新机器人状态 ==========
function updateRobotStatus(status, text) {
    const robotStatus = document.getElementById('robotStatus');
    if (!robotStatus) return;
    const dotClass = status === 'busy' ? 'busy' : 'online';
    robotStatus.innerHTML = `<span class="status-dot ${dotClass}"></span> ${text}`;
}

// ========== 电量显示（485 电池板 + 底盘）==========
function renderPowerStatus(power) {
    const el = document.getElementById('powerStatus');
    if (!el) return;
    const parts = [];
    const battery = power && power.battery;
    const chassis = power && power.chassis;
    if (battery && battery.online) {
        const pct = battery.percent == null ? '--' : `${battery.percent}%`;
        const volt = battery.voltage == null ? '' : ` ${Number(battery.voltage).toFixed(1)}V`;
        parts.push(`🔋 电池 ${pct}${volt}`);
    }
    if (chassis && chassis.online) {
        const cap = chassis.capacity == null ? '--' : `${chassis.capacity}%`;
        parts.push(`🛞 底盘 ${cap}${chassis.charging ? ' ⚡' : ''}`);
    }
    if (!parts.length) {
        el.hidden = true;
        el.textContent = '';
        return;
    }
    el.hidden = false;
    el.textContent = parts.join('  ');
}

async function refreshPowerStatus() {
    try {
        const res = await fetch('/api/device/info', { cache: 'no-store' });
        const json = await res.json();
        if (res.ok && json.code === 0) renderPowerStatus(json.data && json.data.power);
    } catch (error) { /* 离线时静默 */ }
}

// ========== 语音交互 ==========
let voiceRecognition = null;
let isListening = false;
let voiceFinalText = '';
let voiceInterimText = '';
let voiceMode = 'search';   // search=语音检索(筛选商品) / order=语音点单(加入购物车)
let voiceStartedAt = 0;     // 用于统计语音识别耗时

const VOICE_MODE_LABELS = { search: '检索', order: '点单' };

function applyVoiceModeUI() {
    const searchBtn = document.getElementById('voiceModeSearch');
    const orderBtn = document.getElementById('voiceModeOrder');
    const isOrder = voiceMode === 'order';
    if (searchBtn) searchBtn.classList.toggle('listening', isListening && !isOrder);
    if (orderBtn) orderBtn.classList.toggle('listening', isListening && isOrder);
    const s = document.getElementById('searchInput');
    if (s && !isListening) {
        s.placeholder = '搜索或点击语音按钮说话';
    }
}

// 两个按钮分别直接开始对应模式的语音；再点同一个按钮则停止
function handleVoiceButton(mode) {
    if (!voiceRecognition) { alert('请使用 Chrome 或 Edge 浏览器'); return; }
    if (mode !== 'order' && mode !== 'search') return;
    if (isListening && voiceMode === mode) { stopVoice(); return; }
    voiceMode = mode;
    try { localStorage.setItem('grasp_voice_mode', voiceMode); } catch (e) { /* ignore */ }
    applyVoiceModeUI();
    if (!isListening) startVoice();
}

// 品类关键词映射：语音词 → 分类名
const VOICE_CATEGORY_MAP = {
    '可乐': '碳酸饮料', '雪碧': '碳酸饮料', '芬达': '碳酸饮料', '汽水': '碳酸饮料',
    '矿泉水': '饮用水', '饮用水': '饮用水', '纯净水': '饮用水', '水': '饮用水',
    '果汁': '果汁饮料', '橙汁': '果汁饮料',
    '茶': '茶饮料', '绿茶': '茶饮料', '红茶': '茶饮料', '奶茶': '茶饮料',
    '咖啡': '咖啡饮料',
    '功能饮料': '功能饮料', '红牛': '功能饮料',
    '牛奶': '乳制品', '酸奶': '乳制品',
    '薯片': '零食', '饼干': '零食',
    '方便面': '方便食品', '泡面': '方便食品',
    '面包': '烘焙食品', '蛋糕': '烘焙食品',
};

// 去掉口语前缀/后缀、清理标点符号，提取核心关键词
function extractVoiceKeyword(rawText) {
    // 1) 清理所有中英文标点（语音识别常在句尾加句号"。"等）
    let text = String(rawText || '')
        .replace(/[，。！？、；：,.!?;:'"“”‘’（）()【】\[\]《》<>~～·\s]+/g, '')
        .trim();

    // 2) 连续剥离口语前缀（如"给我来一瓶脉动" → "脉动"），长前缀优先
    const prefixes = [
        '我想要', '我想买', '我想喝', '我想吃', '我要买', '我要喝', '我要吃',
        '给我来', '帮我拿', '帮我找', '有没有', '请给我', '帮我',
        '拿一瓶', '拿一罐', '拿一听', '拿一盒', '拿一包', '拿一个',
        '来一瓶', '来一罐', '来一听', '来一盒', '来一包', '来一个',
        '我要', '我想', '给我', '帮我',
        '一瓶', '一罐', '一听', '一盒', '一包', '一个',
        '拿瓶', '拿罐', '拿听', '拿盒', '拿包',
        '来瓶', '来罐', '来听', '来盒', '来包',
        '拿', '来', '买', '要', '我'];
    let changed = true;
    while (changed && text.length > 1) {
        changed = false;
        for (const p of prefixes) {
            if (text.startsWith(p) && text.length > p.length) {
                text = text.slice(p.length);
                changed = true;
                break;
            }
        }
    }

    // 3) 连续剥离口语后缀（"脉动吧" → "脉动"）
    const suffixes = ['吧', '啊', '呀', '呢', '哦', '呗', '嘛', '好', '好了'];
    changed = true;
    while (changed && text.length > 1) {
        changed = false;
        for (const s of suffixes) {
            if (text.endsWith(s) && text.length > s.length + 1) {
                text = text.slice(0, -s.length);
                changed = true;
                break;
            }
        }
    }

    return text.trim() || String(rawText || '').trim();
}

// 匹配分类
function matchCategory(text) {
    for (let i = 0; i < categories.length; i++) {
        if (categories[i] && text.includes(categories[i])) return categories[i];
    }
    for (const [keyword, category] of Object.entries(VOICE_CATEGORY_MAP)) {
        if (text.includes(keyword)) return category;
    }
    return null;
}

// 语音别名归一化：把识别变体/品牌简称统一成商品名称关键词
const VOICE_ALIAS_MAP = {
    '茶Π': '茶π', '茶派': '茶π', '茶pai': '茶π',
    'AD钙奶': 'AD钙', 'ad钙奶': 'AD钙',
    '娃哈哈': '哇哈哈',
    '水溶希': '水溶C', '水溶西': '水溶C',
    '无糖可乐': '无糖 可乐',
    '零度可乐': '零度 可乐',
    '外星人电解质水': '外星人电解质',
    '电解质水': '外星人电解质',
    '果粒橙': '果粒橙',
};

// 归一化语音关键词（只处理一次，避免连环替换）
function normalizeVoiceKeyword(keyword) {
    let k = keyword;
    for (const [alias, target] of Object.entries(VOICE_ALIAS_MAP)) {
        if (k.includes(alias)) {
            k = k.replace(alias, target);
            break;
        }
    }
    return k;
}

// ========== 语音筛选后无操作自动回到全部商品 ==========
const VOICE_RESET_MS = 10000; // 10 秒无操作回到全部商品
let voiceFilterTimer = null;
let voiceFilterBound = false;

function onVoiceFilterActivity() {
    // 弹窗（支付/抓取中）打开时不再计时
    const overlay = document.getElementById('modalOverlay');
    if (overlay && overlay.style.display === 'flex') {
        cancelVoiceResetTimer();
        return;
    }
    if (voiceFilterTimer) clearTimeout(voiceFilterTimer);
    voiceFilterTimer = setTimeout(backToAllProducts, VOICE_RESET_MS);
}

function cancelVoiceResetTimer() {
    if (voiceFilterTimer) {
        clearTimeout(voiceFilterTimer);
        voiceFilterTimer = null;
    }
    if (voiceFilterBound) {
        voiceFilterBound = false;
        document.removeEventListener('mousedown', onVoiceFilterActivity);
        document.removeEventListener('touchstart', onVoiceFilterActivity);
        document.removeEventListener('keydown', onVoiceFilterActivity);
        document.removeEventListener('scroll', onVoiceFilterActivity);
    }
}

function backToAllProducts() {
    voiceFilterTimer = null;
    // 弹窗（支付/抓取）打开时不打断流程
    const overlay = document.getElementById('modalOverlay');
    if (overlay && overlay.style.display === 'flex') {
        cancelVoiceResetTimer();
        return;
    }
    cancelVoiceResetTimer();
    currentCategory = 'all';
    searchKeyword = '';
    const s = document.getElementById('searchInput'); if (s) s.value = '';
    const t = document.getElementById('currentCategoryTitle'); if (t) t.textContent = '全部商品';
    renderCategories();
    renderProducts();
}

function startVoiceResetTimer() {
    cancelVoiceResetTimer();
    if (!voiceFilterBound) {
        voiceFilterBound = true;
        document.addEventListener('mousedown', onVoiceFilterActivity);
        document.addEventListener('touchstart', onVoiceFilterActivity, { passive: true });
        document.addEventListener('keydown', onVoiceFilterActivity);
        document.addEventListener('scroll', onVoiceFilterActivity, { passive: true });
    }
    voiceFilterTimer = setTimeout(backToAllProducts, VOICE_RESET_MS);
}

// 处理语音命令：商品优先全部展示，未命中再切分类
function handleVoiceCommand(rawText) {
    let keyword = extractVoiceKeyword(rawText);
    if (!keyword) return;
    keyword = normalizeVoiceKeyword(keyword);
    console.log(`🎤 语音: "${rawText}" → 关键词 "${keyword}"`);

    const s = document.getElementById('searchInput');
    const t = document.getElementById('currentCategoryTitle');

    // 1) 商品优先：在全部商品中按关键词匹配，命中则全部展示
    const hasProductMatch = products.some(p => productMatchesKeywords(p, keyword));
    if (hasProductMatch) {
        currentCategory = 'all';
        searchKeyword = keyword.toLowerCase();
        if (s) s.value = keyword;
        renderCategories();
        if (t) t.textContent = '全部商品';
    } else {
        // 2) 未命中商品：尝试切分类（如"碳酸饮料"、"矿泉水"）
        const matchedCategory = matchCategory(keyword);
        if (matchedCategory) {
            currentCategory = matchedCategory;
            searchKeyword = '';
            if (s) s.value = '';
            renderCategories();
            if (t) t.textContent = currentCategory === 'all' ? '全部商品' : currentCategory;
        } else {
            // 3) 兜底：按关键词搜索
            currentCategory = 'all';
            searchKeyword = keyword.toLowerCase();
            if (s) s.value = keyword;
            renderCategories();
            if (t) t.textContent = '全部商品';
        }
    }
    renderProducts();

    // 语音筛选后启动 10 秒无操作自动回到全部商品
    startVoiceResetTimer();

    const ps = document.getElementById('productsSection');
    if (ps) ps.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

// ========== 语音大模型校正（保留原逻辑回退） ==========
async function handleVoiceCommandSmart(rawText) {
    let llm = null;
    const t0 = Date.now();
    try {
        const res = await fetch(graspApi('/voice/correct'), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ text: rawText }),
        });
        if (res.ok) {
            const json = await res.json();
            if (json.code === 0 && json.data && json.data.used) llm = json.data;
        }
    } catch (e) {
        console.warn('语音大模型校正失败，走原逻辑:', e);
    }
    console.log(`[语音] 检索服务端调用耗时 ${Date.now() - t0}ms`);

    if (llm) {
        const hint = document.getElementById('voiceHintText');
        if (hint && llm.note) hint.textContent = `✅ ${llm.note}`;
        const ps = document.getElementById('productsSection');

        // 1) 模型给出搜索关键词 → 直接搜索（命中所有含该词的系列/口味变体）
        if (llm.keyword || llm.product) {
            const keyword = (llm.keyword || llm.product);
            currentCategory = 'all';
            searchKeyword = keyword.toLowerCase();
            const s = document.getElementById('searchInput'); if (s) s.value = keyword;
            const t = document.getElementById('currentCategoryTitle'); if (t) t.textContent = '全部商品';
            renderCategories();
            renderProducts();
            startVoiceResetTimer();
            if (ps) ps.scrollIntoView({ behavior: 'smooth', block: 'start' });
            return;
        }
        // 2) 只给了分类 → 切分类
        if (llm.category) {
            currentCategory = llm.category;
            searchKeyword = '';
            const s = document.getElementById('searchInput'); if (s) s.value = '';
            const t = document.getElementById('currentCategoryTitle'); if (t) t.textContent = currentCategory;
            renderCategories();
            renderProducts();
            startVoiceResetTimer();
            if (ps) ps.scrollIntoView({ behavior: 'smooth', block: 'start' });
            return;
        }
        // 3) 未命中商品/分类：用大模型校正后的文本走原逻辑
        if (llm.corrected_text) {
            handleVoiceCommand(llm.corrected_text);
            return;
        }
    }
    handleVoiceCommand(rawText);
}

// ========== 语音点单（加入购物车） ==========
function findProductByName(name) {
    const target = String(name || '').trim().toLowerCase();
    if (!target) return null;
    return products.find(p => (p.name || '').toLowerCase() === target)
        || products.find(p => (p.name || '').toLowerCase().includes(target))
        || products.find(p => target.includes((p.name || '').toLowerCase()) && (p.name || '').length >= 2)
        || null;
}

function addProductQty(product, qty) {
    if (!product) return 0;
    const stock = Number(product.stock);
    const current = Number(cart[product.id] || 0);
    const want = Math.max(1, Math.floor(Number(qty) || 1));
    const allowed = Number.isFinite(stock) ? Math.max(0, stock - current) : want;
    const add = Math.min(want, allowed);
    if (add <= 0) return 0;
    cart[product.id] = current + add;
    updateCartUI();
    renderProducts();
    return add;
}

function openCart() {
    const sidebar = document.getElementById('cartSidebar');
    if (!sidebar) return;
    sidebar.classList.add('is-open');
    const header = sidebar.querySelector('.cart-header');
    if (header) {
        header.setAttribute('aria-expanded', 'true');
        header.setAttribute('aria-label', '收起购物车');
    }
}

async function handleVoiceOrder(rawText) {
    const hint = document.getElementById('voiceHintText');
    const say = (text) => { if (hint) hint.textContent = text; };
    let llm = null;
    const t0 = Date.now();
    try {
        const res = await fetch(graspApi('/voice/correct'), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ text: rawText, mode: 'order' }),
        });
        if (res.ok) {
            const json = await res.json();
            if (json.code === 0 && json.data && json.data.used) llm = json.data;
        }
    } catch (e) {
        console.warn('语音点单大模型失败:', e);
    }
    console.log(`[语音] 点单服务端调用耗时 ${Date.now() - t0}ms`);
    if (!llm) { say('语音点单识别失败，请再说一次'); return; }

    const action = llm.action || (llm.matched ? 'add' : 'none');
    const qty = Math.max(1, Math.floor(Number(llm.quantity) || 1));

    if (action === 'clear') {
        clearCart();
        say('✅ 已清空购物车');
        return;
    }
    if (action === 'query') {
        const items = getCartItems();
        say(items.length
            ? '购物车：' + items.map(i => `${i.name}×${i.qty}`).join('、') + `，共 ${getTotalQty()} 件`
            : '购物车是空的');
        return;
    }
    if (action === 'checkout') {
        if (getCartItems().length === 0) { say('购物车是空的，无法结算'); return; }
        say('✅ 正在结算…');
        await submitOrder();
        return;
    }

    // 组装商品项（兼容单商品字段），支持一句话多个商品
    const orderItems = Array.isArray(llm.items) && llm.items.length
        ? llm.items
        : ((llm.product || llm.keyword) ? [{ product: llm.product, keyword: llm.keyword, quantity: qty }] : []);

    const resolveItem = (item) => {
        if (item && item.product) {
            const exact = findProductByName(item.product);
            if (exact) return exact;
        }
        const kw = String((item && (item.keyword || item.product)) || '').trim().toLowerCase();
        if (!kw) return null;
        return products.find(p => (p.name || '').toLowerCase().includes(kw)) || null;
    };

    if (action === 'none' || !orderItems.length) {
        say(`没听清要什么商品${llm.corrected_text ? `（${llm.corrected_text}）` : ''}`);
        return;
    }

    const done = [];
    const missing = [];

    if (action === 'add') {
        for (const item of orderItems) {
            const product = resolveItem(item);
            const q = Math.max(1, Math.floor(Number(item.quantity) || 1));
            if (!product) { missing.push(item.product || item.keyword || '?'); continue; }
            const added = addProductQty(product, q);
            if (!added) { say(`${product.name} 库存不足`); return; }
            done.push(`${product.name}×${added}`);
        }
        if (done.length) {
            say(`✅ 已加入购物车：${done.join('、')}${missing.length ? `；未找到：${missing.join('、')}` : ''}`);
            openCart();
        } else {
            say(`商品库中没有：${missing.join('、')}`);
        }
        return;
    }

    if (action === 'remove') {
        for (const item of orderItems) {
            const product = resolveItem(item);
            const q = Math.max(1, Math.floor(Number(item.quantity) || 1));
            if (!product) { missing.push(item.product || item.keyword || '?'); continue; }
            const current = Number(cart[product.id] || 0);
            if (current <= 0) { missing.push(product.name); continue; }
            const removeQty = Math.min(q, current);
            const left = current - removeQty;
            if (left <= 0) delete cart[product.id]; else cart[product.id] = left;
            done.push(`${product.name}×${removeQty}`);
        }
        updateCartUI();
        renderProducts();
        say(done.length
            ? `✅ 已移除：${done.join('、')}${missing.length ? `；未在购物车：${missing.join('、')}` : ''}`
            : `未在购物车：${missing.join('、')}`);
        return;
    }

    if (action === 'set') {
        for (const item of orderItems) {
            const product = resolveItem(item);
            if (!product) { missing.push(item.product || item.keyword || '?'); continue; }
            const q = Math.max(0, Math.floor(Number(item.quantity) || 0));
            const stock = Number(product.stock);
            const target = Number.isFinite(stock) ? Math.min(q, stock) : q;
            if (target <= 0) delete cart[product.id]; else cart[product.id] = target;
            done.push(`${product.name}=${Math.max(0, target)}`);
        }
        updateCartUI();
        renderProducts();
        say(done.length ? `✅ 已设置：${done.join('、')}` : `商品库中没有：${missing.join('、')}`);
        return;
    }
}

// 初始化语音识别
function initVoice() {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    const nativeAvailable = window.AndroidSpeech && window.AndroidSpeech.isAvailable && window.AndroidSpeech.isAvailable();
    if (nativeAvailable) {
        initNativeVoice();
        return;
    }
    const canCloudAsr = !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia && window.MediaRecorder);
    if ((window.AndroidSpeech || !SR) && canCloudAsr) {
        initCloudVoice();
        return;
    }
    if (!SR) {
        ['voiceModeSearch', 'voiceModeOrder'].forEach((id) => {
            const b = document.getElementById(id);
            if (b) { b.title = '浏览器不支持语音识别'; b.style.opacity = '0.4'; b.style.cursor = 'not-allowed'; }
        });
        applyVoiceModeUI();
        return;
    }
    voiceRecognition = new SR();
    voiceRecognition.lang = 'zh-CN';
    voiceRecognition.interimResults = true;
    voiceRecognition.continuous = false;
    voiceRecognition.maxAlternatives = 1;

    voiceRecognition.onresult = (event) => {
        voiceInterimText = ''; voiceFinalText = '';
        for (let i = event.resultIndex; i < event.results.length; i++) {
            const r = event.results[i];
            r.isFinal ? (voiceFinalText += r[0].transcript) : (voiceInterimText += r[0].transcript);
        }
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = (voiceFinalText || voiceInterimText) || '正在聆听...';
        if (voiceFinalText) {
            const asrMs = voiceStartedAt ? (Date.now() - voiceStartedAt) : 0;
            console.log(`[语音] 识别耗时(开始→最终文本) ${asrMs}ms: "${voiceFinalText}"`);
            if (voiceMode === 'order') handleVoiceOrder(voiceFinalText);
            else handleVoiceCommandSmart(voiceFinalText);
            setTimeout(() => { if (isListening) stopVoice(); }, 1500);
        }
    };

    voiceRecognition.onerror = (event) => {
        const err = event && event.error;
        if (err === 'not-allowed' || err === 'service-not-allowed') {
            const hint = document.getElementById('voiceHintText');
            if (hint) hint.textContent = '麦克风权限被拒绝';
            return;
        }
        // no-speech / aborted / network 等静默处理，不弹"没听清"，保持连续聆听
        if (isListening) {
            setTimeout(() => { if (isListening && voiceRecognition) { try { voiceRecognition.start(); } catch (e) {} } }, 1000);
        }
    };

    voiceRecognition.onend = () => {
        if (isListening) setTimeout(() => { if (isListening && voiceRecognition) { try { voiceRecognition.start(); } catch (e) {} } }, 300);
    };
    try {
        const saved = localStorage.getItem('grasp_voice_mode');
        if (saved === 'order' || saved === 'search') voiceMode = saved;
    } catch (e) { /* ignore */ }
    applyVoiceModeUI();
}

function initNativeVoice() {
    voiceRecognition = {
        start() { try { window.AndroidSpeech.start(); } catch (e) {} },
        stop() { try { window.AndroidSpeech.stop(); } catch (e) {} },
        abort() { try { window.AndroidSpeech.stop(); } catch (e) {} },
    };
    window.onAndroidSpeechPartial = (text) => {
        voiceInterimText = text || '';
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = text || '正在聆听...';
    };
    window.onAndroidSpeechResult = (text) => {
        voiceFinalText = text || '';
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = voiceFinalText || '正在聆听...';
        if (!voiceFinalText) return;
        const asrMs = voiceStartedAt ? (Date.now() - voiceStartedAt) : 0;
        console.log(`[语音] 原生识别耗时(开始→最终文本) ${asrMs}ms: "${voiceFinalText}"`);
        if (voiceMode === 'order') handleVoiceOrder(voiceFinalText);
        else handleVoiceCommandSmart(voiceFinalText);
        setTimeout(() => { if (isListening) stopVoice(); }, 1500);
    };
    window.onAndroidSpeechError = (err) => {
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = '麦克风权限被拒绝';
        if (isListening) {
            setTimeout(() => {
                if (isListening && voiceRecognition) { try { voiceRecognition.start(); } catch (e) {} }
            }, 1000);
        }
    };
    try {
        const saved = localStorage.getItem('grasp_voice_mode');
        if (saved === 'order' || saved === 'search') voiceMode = saved;
    } catch (e) { /* ignore */ }
    applyVoiceModeUI();
}

function initCloudVoice() {
    let stream = null;
    let recorder = null;
    let chunks = [];
    let mimeType = 'audio/webm';
    let timer = null;
    let meterTimer = null;
    let audioCtx = null;
    let analyser = null;
    let maxLevel = 0;

    const hintText = (text) => {
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = text;
    };

    const stopMeter = () => {
        if (meterTimer) { clearInterval(meterTimer); meterTimer = null; }
        if (audioCtx) { try { audioCtx.close(); } catch (e) { /* ignore */ } audioCtx = null; }
        analyser = null;
    };

    const readLevel = () => {
        if (!analyser) return 0;
        const buf = new Uint8Array(analyser.fftSize);
        analyser.getByteTimeDomainData(buf);
        let sum = 0;
        for (let i = 0; i < buf.length; i += 1) {
            const v = (buf[i] - 128) / 128;
            sum += v * v;
        }
        return Math.min(100, Math.round(Math.sqrt(sum / buf.length) * 300));
    };

    const cleanup = () => {
        if (timer) { clearTimeout(timer); timer = null; }
        stopMeter();
        if (stream) { stream.getTracks().forEach((track) => track.stop()); stream = null; }
        recorder = null;
        chunks = [];
    };

    const upload = async () => {
        if (!chunks.length) {
            hintText('没有录到声音');
            isListening = false;
            applyVoiceModeUI();
            cleanup();
            return;
        }
        const blob = new Blob(chunks, { type: mimeType || 'audio/webm' });
        const type = blob.type || mimeType || 'audio/webm';
        const level = maxLevel;
        cleanup();
        hintText('识别中...');
        try {
            const res = await fetch(graspApi('/voice/asr'), {
                method: 'POST',
                headers: { 'Content-Type': type },
                body: blob,
            });
            const json = await res.json();
            const text = (json && (json.text || (json.data && json.data.text))) || '';
            if (!res.ok || !text) {
                const err = new Error((json && json.message) || '未识别到语音');
                err.level = level;
                throw err;
            }
            voiceFinalText = text;
            hintText(text);
            const asrMs = voiceStartedAt ? (Date.now() - voiceStartedAt) : 0;
            console.log(`[语音] 云端识别耗时 ${asrMs}ms: "${text}"`);
            if (voiceMode === 'order') handleVoiceOrder(text);
            else handleVoiceCommandSmart(text);
        } catch (e) {
            console.warn('[语音] 云端识别失败', e);
            if (e && e.level !== undefined && e.level <= 2) {
                hintText('麦克风没有声音（音量 0%），请检查大屏麦克风');
            } else {
                hintText('语音识别失败: ' + e.message);
            }
        } finally {
            isListening = false;
            applyVoiceModeUI();
            const hint = document.getElementById('voiceHint');
            if (hint) {
                setTimeout(() => {
                    if (!isListening) hint.style.display = 'none';
                }, 4500);
            }
        }
    };

    const stop = () => {
        if (timer) { clearTimeout(timer); timer = null; }
        if (recorder && recorder.state !== 'inactive') recorder.stop();
        else cleanup();
    };

    const start = async () => {
        if (recorder) return;
        try {
            let media = null;
            try {
                media = await navigator.mediaDevices.getUserMedia({
                    audio: {
                        channelCount: 1,
                        echoCancellation: false,
                        noiseSuppression: false,
                        autoGainControl: false,
                    },
                });
            } catch (e) {
                media = await navigator.mediaDevices.getUserMedia({ audio: true });
            }
            stream = media;
            maxLevel = 0;
            try {
                const Ctx = window.AudioContext || window.webkitAudioContext;
                audioCtx = new Ctx();
                analyser = audioCtx.createAnalyser();
                analyser.fftSize = 2048;
                audioCtx.createMediaStreamSource(stream).connect(analyser);
            } catch (e) {
                analyser = null;
            }
            const candidates = ['audio/webm;codecs=opus', 'audio/webm', 'audio/mp4', 'audio/ogg;codecs=opus'];
            mimeType = '';
            if (window.MediaRecorder && MediaRecorder.isTypeSupported) {
                for (const candidate of candidates) {
                    if (MediaRecorder.isTypeSupported(candidate)) { mimeType = candidate; break; }
                }
            }
            chunks = [];
            recorder = mimeType ? new MediaRecorder(stream, { mimeType }) : new MediaRecorder(stream);
            mimeType = recorder.mimeType || mimeType || 'audio/webm';
            recorder.ondataavailable = (event) => {
                if (event.data && event.data.size) chunks.push(event.data);
            };
            recorder.onstop = () => { upload(); };
            recorder.start();
            hintText(voiceMode === 'order' ? '请说要点什么…' : '正在聆听...');
            if (analyser) {
                meterTimer = setInterval(() => {
                    const level = readLevel();
                    if (level > maxLevel) maxLevel = level;
                    hintText(`${voiceMode === 'order' ? '请说要点什么…' : '正在聆听...'} 音量 ${level}%`);
                }, 200);
            }
            timer = setTimeout(() => { stop(); }, 8000);
        } catch (e) {
            console.warn('[语音] 麦克风打开失败', e);
            hintText('麦克风权限被拒绝');
            isListening = false;
            applyVoiceModeUI();
            cleanup();
        }
    };

    voiceRecognition = { start, stop, abort: stop };
    try {
        const saved = localStorage.getItem('grasp_voice_mode');
        if (saved === 'order' || saved === 'search') voiceMode = saved;
    } catch (e) { /* ignore */ }
    applyVoiceModeUI();
}

function startVoice() {
    if (!voiceRecognition) return;
    isListening = true; voiceFinalText = ''; voiceInterimText = '';
    voiceStartedAt = Date.now();
    applyVoiceModeUI();
    const h = document.getElementById('voiceHint'); if (h) h.style.display = 'flex';
    const t = document.getElementById('voiceHintText'); if (t) t.textContent = voiceMode === 'order' ? '请说要点什么…' : '正在聆听...';
    const s = document.getElementById('searchInput'); if (s) s.placeholder = voiceMode === 'order' ? '🎤 语音点单中，请说商品...' : '🎤 正在聆听，请说话...';
    try { voiceRecognition.start(); } catch (e) {}
}

function stopVoice() {
    isListening = false;
    const h = document.getElementById('voiceHint');
    if (h) setTimeout(() => { h.style.display = 'none'; }, 2000);
    applyVoiceModeUI();
    try { voiceRecognition.abort(); } catch (e) {}
}

window.addEventListener('beforeunload', () => {
    if (isListening) try { voiceRecognition.abort(); } catch (e) {}
});

// ========== 通用语音播报 ==========
// 预生成音频映射：文案 -> 音频文件。
// Android 系统 WebView 不支持 speechSynthesis 语音合成，一体机上必须走 mp3 播放。
const VOICE_AUDIO_MAP = {
    '欢迎光临，请点单!': 'audio/welcome.mp3',
    '请扫码支付。': 'audio/pay.mp3',
    '请等待机器人取货。': 'audio/wait.mp3',
    '出货完成，请核对商品是否已经全部取出，欢迎下次光临！': 'audio/done.mp3',
    '机器人取货失败，费用已原路退还。': 'audio/failed.mp3',
};
let voiceAudio = null;

// 停止语音播报（mp3 与 speechSynthesis 一并停止）
function stopSpeak() {
    if (voiceAudio) {
        try {
            voiceAudio.pause();
            voiceAudio.currentTime = 0;
        } catch (e) { /* ignore */ }
    }
    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
    }
}

// 播放语音：优先播放预生成的 mp3；未配置音频的文本兜底走 Web Speech API
function speakText(text) {
    const src = VOICE_AUDIO_MAP[text];
    if (src) {
        if (!voiceAudio) voiceAudio = new Audio();
        voiceAudio.pause();
        voiceAudio.src = src + '?v=1';
        voiceAudio.play().catch(e => console.warn('语音播报播放失败:', e));
        return;
    }
    if (!('speechSynthesis' in window)) return;
    // 取消当前正在播放的语音，避免重叠
    window.speechSynthesis.cancel();
    // 等待一小段时间让 cancel 生效
    setTimeout(() => {
        const utter = new SpeechSynthesisUtterance(text);
        utter.lang = 'zh-CN';
        utter.rate = 0.9;
        utter.pitch = 1.1;
        utter.volume = 1;
        window.speechSynthesis.speak(utter);
    }, 100);
}

// ========== 商家登录 / 门店设备上下文 ==========
// 登录与门店接口始终走 /api（测试页也走 /api，避免 /test-api 缺少对应路由）。
function orderApiUrl(pathname) {
    return `/api${pathname}`;
}

async function ensureOrderContext() {
    const params = new URLSearchParams(window.location.search);
    const urlStore = params.get('store') || '';
    const urlDevice = params.get('device') || '';
    // 商家会话 Cookie 是 Secure 的，http（盒子本地/局域网触摸屏）无法保持登录，跳过登录。
    if (window.location.protocol !== 'https:') {
        orderSelectedStoreId = urlStore;
        orderSelectedDeviceId = urlDevice;
        renderOrderContext();
        return;
    }
    let session = null;
    try {
        const res = await fetch(orderApiUrl('/merchant/session'), { cache: 'no-store' });
        session = await res.json();
    } catch (error) {
        session = null;
    }
    if (!session || (!session.authenticated && !session.loginAvailable)) {
        // 没有商家体系（例如盒子本地直连）：退回 URL 参数
        orderSelectedStoreId = urlStore;
        orderSelectedDeviceId = urlDevice;
        renderOrderContext();
        return;
    }
    if (session.authenticated) {
        await applyOrderMerchant(session);
    } else {
        showOrderLogin();
        const profile = await new Promise((resolve) => { orderLoginResolve = resolve; });
        hideOrderLogin();
        if (profile) await applyOrderMerchant(profile);
    }
    if (urlStore && !orderSelectedStoreId) orderSelectedStoreId = urlStore;
    if (urlDevice && !orderSelectedDeviceId) orderSelectedDeviceId = urlDevice;
    renderOrderContext();
}

function showOrderLogin(message = '', mode = orderAuthMode) {
    orderAuthMode = mode;
    const overlay = document.getElementById('orderLoginOverlay');
    if (!overlay) return;
    overlay.hidden = false;
    const titleEl = document.getElementById('orderLoginTitle');
    const descEl = document.getElementById('orderLoginDesc');
    const userLabel = document.getElementById('orderLoginUsernameLabel');
    if (orderAuthMode === 'admin') {
        if (titleEl) titleEl.textContent = '测试页登录';
        if (descEl) descEl.textContent = '请使用平台管理员账号登录后测试';
        if (userLabel) userLabel.textContent = '管理员账号';
    } else {
        if (titleEl) titleEl.textContent = '门店点单登录';
        if (descEl) descEl.textContent = '请使用平台分配的商家账号登录';
        if (userLabel) userLabel.textContent = '商家账号';
    }
    const error = document.getElementById('orderLoginError');
    if (error) error.textContent = message;
}

function hideOrderLogin() {
    const overlay = document.getElementById('orderLoginOverlay');
    if (overlay) overlay.hidden = true;
}

async function submitOrderLogin(event) {
    event.preventDefault();
    const button = document.getElementById('orderLoginSubmit');
    const error = document.getElementById('orderLoginError');
    if (button) button.disabled = true;
    if (error) error.textContent = '';
    const isAdmin = orderAuthMode === 'admin';
    const loginPath = isAdmin ? '/admin/login' : '/merchant/login';
    const sessionPath = isAdmin ? '/admin/session' : '/merchant/session';
    try {
        const res = await fetch(orderApiUrl(loginPath), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                username: document.getElementById('orderLoginUsername').value.trim(),
                password: document.getElementById('orderLoginPassword').value
            })
        });
        const json = await res.json();
        if (!res.ok || json.code !== 0) {
            if (error) error.textContent = json.message || '登录失败';
            return;
        }
        document.getElementById('orderLoginPassword').value = '';
        const profileRes = await fetch(orderApiUrl(sessionPath), { cache: 'no-store' });
        const profile = await profileRes.json();
        if (orderLoginResolve) {
            const resolve = orderLoginResolve;
            orderLoginResolve = null;
            resolve(profile && profile.authenticated ? profile : json);
        }
    } catch (error2) {
        if (error) error.textContent = '网络错误，请稍后重试';
    } finally {
        if (button) button.disabled = false;
    }
}

async function logoutOrderMerchant() {
    const isAdmin = orderAuthMode === 'admin';
    try {
        await fetch(orderApiUrl(isAdmin ? '/admin/logout' : '/merchant/logout'), { method: 'POST' });
    } catch (error) { /* ignore */ }
    orderMerchant = null;
    orderStoreList = [];
    orderDeviceList = [];
    orderSelectedStoreId = '';
    orderSelectedDeviceId = '';
    const userInput = document.getElementById('orderLoginUsername');
    if (userInput) userInput.value = '';
    renderOrderContext();
    showOrderLogin('已退出登录', isAdmin ? 'admin' : 'merchant');
    const profile = await new Promise((resolve) => { orderLoginResolve = resolve; });
    hideOrderLogin();
    if (!profile) return;
    if (isAdmin) {
        await loadAdminTestContext();
    } else {
        await applyOrderMerchant(profile);
    }
    await refreshCatalog();
    reconnectWebSocketForStore();
    renderOrderContext();
}

async function ensureAdminTestContext() {
    orderAuthMode = 'admin';
    let session = null;
    try {
        const res = await fetch(orderApiUrl('/admin/session'), { cache: 'no-store' });
        session = await res.json();
    } catch (error) { session = null; }
    if (!session || !session.authenticated) {
        showOrderLogin('', 'admin');
        await new Promise((resolve) => { orderLoginResolve = resolve; });
        hideOrderLogin();
    }
    await loadAdminTestContext();
}

async function loadAdminTestContext() {
    orderMerchant = { name: '管理员测试', username: 'admin' };
    orderStoreList = [];
    try {
        const res = await fetch(orderApiUrl('/admin/stores'), { cache: 'no-store' });
        const json = await res.json();
        if (res.ok && json.code === 0) {
            orderStoreList = (json.data || []).map((store) => ({ id: store.id, name: store.name }));
        }
    } catch (error) { /* ignore */ }
    const params = new URLSearchParams(window.location.search);
    const urlStore = params.get('store') || '';
    if (urlStore && orderStoreList.some((store) => store.id === urlStore)) {
        orderSelectedStoreId = urlStore;
    } else if (!orderStoreList.some((store) => store.id === orderSelectedStoreId)) {
        orderSelectedStoreId = orderStoreList.length ? orderStoreList[0].id : '';
    }

    orderDeviceList = [];
    try {
        const res = await fetch(orderApiUrl('/admin/devices'), { cache: 'no-store' });
        const json = await res.json();
        if (res.ok && json.code === 0) orderDeviceList = json.data || [];
    } catch (error) { /* ignore */ }
    const visible = orderDeviceList.filter((device) => !orderSelectedStoreId || device.storeId === orderSelectedStoreId);
    const urlDevice = params.get('device') || '';
    const match = urlDevice ? visible.find((device) => device.id === urlDevice || device.sn === urlDevice) : null;
    if (match) {
        orderSelectedDeviceId = match.id;
    } else if (!visible.some((device) => device.id === orderSelectedDeviceId)) {
        const online = visible.find((device) => device.online);
        orderSelectedDeviceId = (online || visible[0] || {}).id || '';
    }
}

async function applyOrderMerchant(profile) {
    orderMerchant = profile || null;
    orderStoreList = Array.isArray(profile && profile.stores) ? profile.stores : [];
    const params = new URLSearchParams(window.location.search);
    const urlStore = params.get('store') || '';
    if (urlStore && orderStoreList.some((store) => store.id === urlStore)) {
        orderSelectedStoreId = urlStore;
    } else if (!orderStoreList.some((store) => store.id === orderSelectedStoreId)) {
        orderSelectedStoreId = orderStoreList.length ? orderStoreList[0].id : '';
    }
    await loadOrderDevices();
}

async function loadOrderDevices() {
    orderDeviceList = [];
    try {
        const res = await fetch(orderApiUrl('/merchant/overview'), { cache: 'no-store' });
        const json = await res.json();
        if (res.ok && json.code === 0) orderDeviceList = (json.data && json.data.devices) || [];
    } catch (error) { /* ignore */ }
    const visible = orderDeviceList.filter((device) => !orderSelectedStoreId || device.storeId === orderSelectedStoreId);
    const urlDevice = new URLSearchParams(window.location.search).get('device') || '';
    const urlMatch = urlDevice
        ? visible.find((device) => device.id === urlDevice || device.sn === urlDevice)
        : null;
    if (urlMatch) {
        orderSelectedDeviceId = urlMatch.id;
    } else if (!visible.some((device) => device.id === orderSelectedDeviceId)) {
        const online = visible.find((device) => device.online);
        orderSelectedDeviceId = (online || visible[0] || {}).id || '';
    }
}

function renderOrderContext() {
    const bar = document.getElementById('orderContext');
    if (!bar) return;
    const hasContext = Boolean(orderMerchant) || orderStoreList.length > 0 || Boolean(orderSelectedStoreId) || Boolean(orderSelectedDeviceId);
    bar.hidden = !hasContext;
    const nameEl = document.getElementById('orderMerchantName');
    if (nameEl) nameEl.textContent = orderMerchant ? (orderMerchant.name || orderMerchant.username || '') : '';
    const storeSelect = document.getElementById('orderStoreSelect');
    if (storeSelect) {
        storeSelect.innerHTML = orderStoreList
            .map((store) => `<option value="${escapeHtml(store.id)}">${escapeHtml(store.name || store.id)}</option>`)
            .join('');
        if (orderSelectedStoreId) storeSelect.value = orderSelectedStoreId;
        storeSelect.disabled = orderStoreList.length <= 1;
    }
    renderOrderDeviceSelect();
    const logoutBtn = document.getElementById('orderLogoutBtn');
    if (logoutBtn) logoutBtn.hidden = !orderMerchant;
}

function renderOrderDeviceSelect() {
    const deviceSelect = document.getElementById('orderDeviceSelect');
    if (!deviceSelect) return;
    const visible = orderDeviceList.filter((device) => !orderSelectedStoreId || device.storeId === orderSelectedStoreId);
    deviceSelect.innerHTML = visible.length
        ? visible.map((device) => {
            const label = `${escapeHtml(device.name || device.id)} · ${device.online ? '在线' : '离线'}${device.sn ? ` · ${escapeHtml(device.sn)}` : ''}`;
            return `<option value="${escapeHtml(device.id)}">${label}</option>`;
        }).join('')
        : '<option value="">（该门店暂无设备）</option>';
    if (orderSelectedDeviceId) deviceSelect.value = orderSelectedDeviceId;
    const selected = visible.find((device) => device.id === orderSelectedDeviceId);
    const statusEl = document.getElementById('orderDeviceStatus');
    if (statusEl) {
        statusEl.textContent = selected ? (selected.online ? '在线' : '离线') : '';
        statusEl.className = `order-context-device-status ${selected && selected.online ? 'on' : 'off'}`;
    }
}

async function onOrderStoreChange() {
    const storeSelect = document.getElementById('orderStoreSelect');
    orderSelectedStoreId = storeSelect ? storeSelect.value : '';
    await loadOrderDevices();
    renderOrderContext();
    reconnectWebSocketForStore();
    await refreshCatalog();
}

function reconnectWebSocketForStore() {
    wsGeneration += 1;
    if (ws) {
        try { ws.close(); } catch (error) { /* ignore */ }
    }
    ws = null;
    stopWebSocketHeartbeat();
    wsConnected = false;
    scheduleWebSocketReconnect(0);
}

function onOrderDeviceChange() {
    const deviceSelect = document.getElementById('orderDeviceSelect');
    orderSelectedDeviceId = deviceSelect ? deviceSelect.value : '';
    renderOrderDeviceSelect();
}

// ========== 启动 ==========
init();
initVoice();
