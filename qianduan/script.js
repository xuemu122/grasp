window.GRASP_TEST_MODE = Boolean(window.GRASP_TEST_MODE)
    || window.location.pathname.endsWith('/test.html')
    || new URLSearchParams(window.location.search).get('mode') === 'free';

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
let robotAcceptingOrders = true;
let catalogRefreshPromise = null;

function syncOrderButton() {
    const btnSubmit = document.getElementById('btnSubmit');
    if (!btnSubmit) return;
    const locked = !robotAcceptingOrders || Boolean(currentOrderId);
    btnSubmit.disabled = locked;
    btnSubmit.textContent = currentOrderId
        ? '机器人运动中...'
        : (robotAcceptingOrders ? '开始拣选' : '机器人移动中...');
}

function connectWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${wsProtocol}//${window.location.host}/ws/`);

    ws.onopen = () => {
        wsConnected = true;
        logWS('✅ 已连接到盒子后端');
        updateConnectionStatus('backend');
    };

    ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        handleWSMessage(msg);
    };

    ws.onclose = () => {
        wsConnected = false;
        logWS('❌ 与后端断开连接，3秒后重连...');
        updateConnectionStatus('disconnected');
        setTimeout(connectWebSocket, 3000);
    };

    ws.onerror = () => {
        logWS('⚠️ WebSocket 连接错误');
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
            break;
    }
}

// ========== 熄屏唤醒 ==========
(function initSplash() {
    const splash = document.getElementById('splashScreen');
    if (!splash) return;

    let awake = false;
    function wakeUp() {
        if (awake) return;
        awake = true;
        splash.classList.add('hidden');

        if ('speechSynthesis' in window) {
            setTimeout(() => {
                const utter = new SpeechSynthesisUtterance('你好，请点单');
                utter.lang = 'zh-CN';
                utter.rate = 0.9;
                utter.pitch = 1.1;
                utter.volume = 1;
                window.speechSynthesis.speak(utter);
            }, 500);
        }
    }

    splash.addEventListener('click', wakeUp);
    splash.addEventListener('touchend', (e) => {
        e.preventDefault();
        wakeUp();
    });
})();

// ========== 初始化 ==========
async function init() {
    connectWebSocket();
    await refreshCatalog();
    bindSearch();
    window.addEventListener('focus', refreshCatalog);
    document.addEventListener('visibilitychange', () => {
        if (!document.hidden) refreshCatalog();
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
        const res = await fetch('/api/categories');
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
        const res = await fetch('/api/products');
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
function getFilteredProducts() {
    return products.filter(p => {
        const matchCategory = currentCategory === 'all' || p.category === currentCategory;
        const kw = searchKeyword;
        const matchSearch = !kw ||
            (p.name && p.name.toLowerCase().includes(kw)) ||
            (p.spec && p.spec.toLowerCase().includes(kw)) ||
            (p.manufacturer && p.manufacturer.toLowerCase().includes(kw)) ||
            (p.location && p.location.toLowerCase().includes(kw));
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
function changeQty(productId, delta) {
    if (!cart[productId]) return;
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
    if (!sidebar || sidebar.contains(event.target)) return;
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
                    <button class="btn-qty" onclick="changeQty('${item.id}', -1)">−</button>
                    <span class="qty-num">${item.qty}</span>
                    <button class="btn-qty" onclick="changeQty('${item.id}', 1)">+</button>
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

    if (!robotAcceptingOrders) {
        alert('机器人正在移动或回充，请稍后再下单');
        return;
    }

    const btnSubmit = document.getElementById('btnSubmit');
    btnSubmit.disabled = true;
    btnSubmit.textContent = '⏳ 提交中...';

    let prepared;
    try {
        const res = await fetch('/api/orders/prepare', {
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

    if (!wsConnected) {
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
        const res = await fetch(`/api/orders/${encodeURIComponent(orderId)}/refund`, {
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

function showPaymentModal(orderId, payment, totalAmount) {
    const overlay = document.getElementById('modalOverlay');
    const content = document.getElementById('modalContent');
    const codeUrl = payment && payment.codeUrl ? String(payment.codeUrl) : '';
    const qrCodeBase64 = payment && payment.qrCodeBase64 ? String(payment.qrCodeBase64) : '';
    paymentCancelled = false;
    content.innerHTML = `
        <div class="modal-title">请完成支付</div>
        <div class="modal-text"><span class="order-id-display">订单号：${orderId}</span><br>应付 ¥${Number(totalAmount || 0).toFixed(2)}</div>
        ${qrCodeBase64 ? `<img class="payment-qr-code" src="${escapeHtml(qrCodeBase64)}" alt="微信支付二维码">` : ''}
        <div class="payment-code-box"><code>${escapeHtml(codeUrl || '支付链接生成中...')}</code></div>
        <div class="modal-text" id="paymentStatusText">请使用微信打开上面的支付链接，页面会自动查询支付状态。</div>
        <div class="payment-actions">
            <button class="btn-primary" type="button" id="copyPaymentCode">复制支付链接</button>
            <button class="btn-secondary" type="button" id="cancelPayment">取消订单</button>
        </div>
    `;
    overlay.style.display = 'flex';
    document.getElementById('copyPaymentCode').onclick = async () => {
        try { await navigator.clipboard.writeText(codeUrl); showPaymentStatus('支付链接已复制'); }
        catch { showPaymentStatus('复制失败，请手动长按/复制支付链接'); }
    };
    document.getElementById('cancelPayment').onclick = () => { paymentCancelled = true; overlay.style.display = 'none'; };
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
            const res = await fetch(`/api/orders/${encodeURIComponent(orderId)}/payment`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                cache: 'no-store',
                body: JSON.stringify({ completionToken })
            });
            const data = await res.json();
            if (data.code === 0 && data.paid) {
                document.getElementById('modalOverlay').style.display = 'none';
                return true;
            }
            const terminalStatus = ['REFUNDED', 'REFUNDING', 'CLOSED', 'FAILED'].includes(data.paymentStatus)
                || ['REFUNDED', 'REFUNDING', 'CLOSED', 'FAILED'].includes(data.status);
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
            await fetch(`/api/orders/${encodeURIComponent(orderId)}/cancel-payment`, {
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
        const res = await fetch('/api/shelf/layers');
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
    const res = await fetch('/api/robot/goto-layer', {
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

    const res = await fetch('/api/robot/movel', {
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
        const res = await fetch('/api/robot/goto-layer', {
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

    overlay.style.display = 'flex';
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

// ========== 启动 ==========
init();
