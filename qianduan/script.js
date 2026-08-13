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

// ========== WebSocket 连接 ==========
let ws = null;
let wsConnected = false;
let robotAcceptingOrders = false;
let catalogRefreshPromise = null;
let robotConnectionProbe = null;
let wsReconnectTimer = null;
let wsHeartbeatTimer = null;
let wsGeneration = 0;
let wsConnectedAt = 0;
let wsLastPongAt = 0;

const WS_CONNECT_TIMEOUT_MS = 8000;
const WS_HEARTBEAT_INTERVAL_MS = 15000;
const WS_STALE_TIMEOUT_MS = 45000;

function settleRobotConnectionProbe(connected) {
    if (!robotConnectionProbe) return;
    const probe = robotConnectionProbe;
    robotConnectionProbe = null;
    clearTimeout(probe.timeout);
    if (connected && probe.socket === ws) wsLastPongAt = Date.now();
    probe.resolve(connected);
}

function verifyRobotConnection(timeoutMs = 1500) {
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
    updateConnectionStatus('disconnected');
    scheduleWebSocketReconnect(delayMs);
}

function startWebSocketHeartbeat(socket, generation) {
    stopWebSocketHeartbeat();
    wsHeartbeatTimer = setInterval(() => {
        if (ws !== socket || wsGeneration !== generation || socket.readyState !== WebSocket.OPEN) {
            stopWebSocketHeartbeat();
            return;
        }
        verifyRobotConnection(2500).then((connected) => {
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
        verifyRobotConnection(2500).then((connected) => {
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
    const socket = new WebSocket(`${wsProtocol}//${window.location.host}/ws/`);
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
        logWS('❌ 与后端断开连接，3秒后重连...');
        resetWebSocketConnection(socket, generation, 3000);
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
    const IDLE_TIMEOUT = 30000; // 30秒无操作返回待机页

    function goToSleep() {
        awake = false;
        splash.classList.remove('hidden');
        // 停止语音播报
        if ('speechSynthesis' in window) {
            window.speechSynthesis.cancel();
        }
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
        const res = await fetch(graspApi('/products'));
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
    const fields = [product.name, product.spec, product.manufacturer, product.location]
        .filter(Boolean)
        .map(f => f.toLowerCase());
    return words.every(w => fields.some(f => f.includes(w)));
}

function getFilteredProducts() {
    return products.filter(p => {
        const matchCategory = currentCategory === 'all' || p.category === currentCategory;
        const words = searchKeyword ? searchKeyword.toLowerCase().split(/\s+/).filter(Boolean) : [];
        const matchSearch = words.length === 0 || words.every(w =>
            [p.name, p.spec, p.manufacturer, p.location]
                .filter(Boolean)
                .some(f => f.toLowerCase().includes(w))
        );
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

    if (!(await verifyRobotConnection())) {
        alert('后端未连接，暂时无法下单');
        syncOrderButton();
        return;
    }

    if (!robotAcceptingOrders) {
        alert('机器人正在移动或回充，请稍后再下单');
        return;
    }

    const btnSubmit = document.getElementById('btnSubmit');
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
                payment_mode: window.GRASP_TEST_MODE ? 'free' : undefined
            })
        });
        prepared = await res.json();
        if (!res.ok || prepared.code !== 0) throw new Error(prepared.message || '订单校验失败');
    } catch (error) {
        alert(`订单提交失败: ${error.message}`);
        syncOrderButton();
        return;
    }

    if (!(await verifyRobotConnection())) {
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

    if (!(await verifyRobotConnection())) {
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

// ========== 语音交互 ==========
let voiceRecognition = null;
let isListening = false;
let voiceFinalText = '';
let voiceInterimText = '';

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

    const ps = document.getElementById('productsSection');
    if (ps) ps.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

// 初始化语音识别
function initVoice() {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SR) {
        const b = document.getElementById('btnVoice');
        if (b) { b.title = '浏览器不支持语音识别'; b.style.opacity = '0.4'; b.style.cursor = 'not-allowed'; }
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
            handleVoiceCommand(voiceFinalText);
            setTimeout(() => { if (isListening) stopVoice(); }, 1500);
        }
    };

    voiceRecognition.onerror = (event) => {
        const hint = document.getElementById('voiceHintText');
        if (hint) hint.textContent = event.error === 'not-allowed' ? '麦克风权限被拒绝' : '没听清，再试一次';
        setTimeout(() => { if (isListening && voiceRecognition) { try { voiceRecognition.start(); } catch (e) {} } }, 1000);
    };

    voiceRecognition.onend = () => {
        if (isListening) setTimeout(() => { if (isListening && voiceRecognition) { try { voiceRecognition.start(); } catch (e) {} } }, 300);
    };
}

function toggleVoice() {
    if (!voiceRecognition) { alert('请使用 Chrome 或 Edge 浏览器'); return; }
    isListening ? stopVoice() : startVoice();
}

function startVoice() {
    if (!voiceRecognition) return;
    isListening = true; voiceFinalText = ''; voiceInterimText = '';
    const b = document.getElementById('btnVoice'); if (b) { b.classList.add('listening'); b.textContent = '🎙️'; }
    const h = document.getElementById('voiceHint'); if (h) h.style.display = 'flex';
    const t = document.getElementById('voiceHintText'); if (t) t.textContent = '正在聆听...';
    const s = document.getElementById('searchInput'); if (s) s.placeholder = '🎤 正在聆听，请说话...';
    try { voiceRecognition.start(); } catch (e) {}
}

function stopVoice() {
    isListening = false;
    const b = document.getElementById('btnVoice'); if (b) { b.classList.remove('listening'); b.textContent = '🎤'; }
    const h = document.getElementById('voiceHint');
    if (h) setTimeout(() => { h.style.display = 'none'; }, 2000);
    const s = document.getElementById('searchInput'); if (s) s.placeholder = '搜索或点击🎤语音搜索';
    try { voiceRecognition.abort(); } catch (e) {}
}

window.addEventListener('beforeunload', () => {
    if (isListening) try { voiceRecognition.abort(); } catch (e) {}
});

// ========== 通用语音播报 ==========
function speakText(text) {
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

// ========== 启动 ==========
init();
initVoice();
