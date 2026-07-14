// ========== 全局状态 ==========
let products = [];           // 所有商品
let categories = [];         // 所有分类
let cart = {};               // { productId: qty }
let currentCategory = 'all'; // 当前选中的分类
let searchKeyword = '';      // 搜索关键词
let currentOrderId = null;

// ========== WebSocket 连接 ==========
let ws = null;
let wsConnected = false;

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

function updateConnectionStatus(status) {
    const robotStatus = document.getElementById('robotStatus');
    if (!robotStatus) return;
    if (status === 'backend' || status === 'standby' || status === 'idle') {
        robotStatus.innerHTML = '<span class="status-dot busy"></span> 后端在线，机械臂待启动';
    } else if (status === 'ready') {
        robotStatus.innerHTML = '<span class="status-dot online"></span> 机器人就绪';
    } else if (status === 'busy' || status === 'stowing') {
        robotStatus.innerHTML = `<span class="status-dot busy"></span> ${status === 'stowing' ? '机械臂收起中' : '机器人执行中'}`;
    } else if (status === 'simulated') {
        robotStatus.innerHTML = '<span class="status-dot busy"></span> 模拟后端在线';
    } else {
        robotStatus.innerHTML = '<span class="status-dot" style="background:#EF4444;box-shadow:0 0 8px rgba(239,68,68,0.6)"></span> 后端未连接';
    }
}

function handleWSMessage(msg) {
    switch (msg.type) {
        case 'connected':
            logWS(msg.message);
            updateConnectionStatus(msg.robotStatus || 'backend');
            break;
        case 'status':
            if (msg.status === 'started') {
                updateRobotStatus('busy', msg.message || '机器人抓取中...');
            } else if (msg.status === 'grabbing') {
                updateGrabProgress(msg);
            } else if (msg.status === 'completed') {
                if (msg.orderId && currentOrderId && msg.orderId === currentOrderId) {
                    onGrabCompleted(msg);
                } else {
                    onStowCompleted(msg);
                }
            } else if (msg.status === 'error') {
                alert('抓取失败: ' + msg.message);
                onGrabFailed();
            }
            break;
        case 'error':
            alert('后端错误: ' + msg.message);
            onGrabFailed();
            break;
        case 'pong':
            break;
    }
}

// ========== 初始化 ==========
async function init() {
    connectWebSocket();
    await loadCategories();
    await loadProducts();
    bindSearch();
}

// ========== 加载分类 ==========
async function loadCategories() {
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
    renderCategories();
}

// ========== 加载商品 ==========
async function loadProducts() {
    try {
        const res = await fetch('/api/products');
        const data = await res.json();
        if (data.code === 0) {
            products = data.data || [];
        }
    } catch (e) {
        console.error('加载商品失败:', e);
        products = [];
    }
    renderProducts();
}

// ========== 渲染分类导航 ==========
function renderCategories() {
    const list = document.getElementById('categoryList');
    if (!list) return;

    const items = [
        { key: 'all', label: '全部商品' },
        ...categories.map(c => ({ key: c, label: c }))
    ];

    list.innerHTML = items.map(item => `
        <div class="category-item ${item.key === currentCategory ? 'active' : ''}"
             data-category="${item.key}"
             onclick="selectCategory('${item.key}')">
            <span class="cat-name">${item.label}</span>
            <span class="cat-count">${item.key === 'all' ? products.length : products.filter(p => p.category === item.key).length}</span>
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

    // 更新分类计数
    renderCategories();

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
        const code = p.location ? String(p.location).replace(/\s/g, '') : ('#' + String(p.id).padStart(12, '0'));
        return `
            <div class="product-card ${qty > 0 ? 'selected' : ''}" id="card-${p.id}">
                <div class="product-image">
                    ${p.photo_front ? `<img src="${p.photo_front}" alt="${p.name}" loading="lazy">` : `<span class="product-emoji">${p.emoji || '📦'}</span>`}
                    ${qty > 0 ? `<span class="product-badge">${qty}</span>` : ''}
                </div>
                <div class="product-code">${code}</div>
                <div class="product-name" title="${p.name}">${p.name}</div>
                <div class="product-spec">${p.spec || ''}</div>
                <div class="product-manufacturer">${p.manufacturer || ''}</div>
                <div class="product-footer">
                    <div class="product-price">
                        <span class="price-symbol">¥</span>${Number(p.price || 0).toFixed(2)}
                    </div>
                    <button class="btn-add" onclick="addToCart('${p.id}', event)">
                        ${qty > 0 ? '+' : '添加'}
                    </button>
                </div>
            </div>
        `;
    }).join('');
}

// ========== 加入购物车 ==========
function addToCart(productId, event) {
    if (event) event.stopPropagation();
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
}

// ========== 提交订单 ==========
function submitOrder() {
    const items = getCartItems();
    if (items.length === 0) return;

    if (!wsConnected) {
        alert('⚠️ 后端服务未连接，请先启动后端！\n\n在终端执行: node server.js');
        return;
    }

    const btnSubmit = document.getElementById('btnSubmit');
    btnSubmit.disabled = true;
    btnSubmit.textContent = '⏳ 提交中...';

    const orderId = 'ORD-' + Date.now().toString(36).toUpperCase();
    currentOrderId = orderId;

    showModal(orderId, items);

    const payload = {
        type: 'grab',
        orderId: orderId,
        items: items.map(i => ({
            id: i.id,
            name: i.name,
            emoji: i.emoji || '📦',
            qty: i.qty,
            price: i.price
        })),
        timestamp: Date.now()
    };

    logWS('📤 发送抓取命令: ' + JSON.stringify(payload));
    ws.send(JSON.stringify(payload));
}

function stowRobot() {
    if (!wsConnected) {
        alert('后端服务未连接');
        return;
    }
    if (!confirm('确认让机器人回到收起姿态并关闭连接吗？')) return;
    updateRobotStatus('busy', '机器人收起中...');
    ws.send(JSON.stringify({ type: 'stow', timestamp: Date.now() }));
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
function onGrabCompleted(msg) {
    const progressBar = document.getElementById('progressBar');
    const progressText = document.getElementById('progressText');
    const robotAnim = document.getElementById('robotAnim');
    const overlay = document.getElementById('modalOverlay');

    if (progressBar) progressBar.style.width = '100%';
    if (progressText) progressText.textContent = '✅ 抓取完成！请取走您的商品';
    if (robotAnim) {
        robotAnim.classList.remove('moving');
        robotAnim.textContent = '✅';
    }

    const items = getCartItems();

    setTimeout(() => {
        if (overlay) overlay.style.display = 'none';
        updateRobotStatus('online', '机器人就绪');
        showOrderToast(msg.orderId, items);
        cart = {};
        currentOrderId = null;
        updateCartUI();
        renderProducts();

        const btnSubmit = document.getElementById('btnSubmit');
        if (btnSubmit) {
            btnSubmit.disabled = false;
            btnSubmit.textContent = '开始拣选';
        }
    }, 1500);
}

// ========== 抓取失败 ==========
function onGrabFailed() {
    const overlay = document.getElementById('modalOverlay');
    if (overlay) overlay.style.display = 'none';
    updateRobotStatus('online', '机器人就绪');

    const btnSubmit = document.getElementById('btnSubmit');
    if (btnSubmit) {
        btnSubmit.disabled = false;
        btnSubmit.textContent = '开始拣选';
    }
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
