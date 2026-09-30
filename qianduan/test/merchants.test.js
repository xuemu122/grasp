const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const fs = require('node:fs');
const net = require('node:net');
const http = require('node:http');
const os = require('node:os');
const path = require('node:path');
const test = require('node:test');

function getFreePort() {
    return new Promise((resolve, reject) => {
        const server = net.createServer();
        server.once('error', reject);
        server.listen(0, '127.0.0.1', () => {
            const { port } = server.address();
            server.close((error) => error ? reject(error) : resolve(port));
        });
    });
}

async function requestJson(url, { method = 'GET', body, cookie } = {}) {
    const headers = {};
    if (body !== undefined) headers['Content-Type'] = 'application/json';
    if (cookie) headers.Cookie = cookie;
    const response = await fetch(url, {
        method,
        headers,
        body: body === undefined ? undefined : JSON.stringify(body)
    });
    let json = null;
    try { json = await response.json(); } catch (error) { /* ignore */ }
    return { response, json };
}

test('merchant login is scoped to the stores it owns', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-merchant-'));
    const productsFile = path.join(tempDir, 'products.json');
    const operationsFile = path.join(tempDir, 'operations.json');
    const ordersFile = path.join(tempDir, 'orders.json');
    const chassisSettingsFile = path.join(tempDir, 'chassis_settings.json');
    const boxRequests = [];
    const scanTask = { id: 'SCAN-MERCHANT', status: 'completed', results: [
        { shelf: 'A', point: 1, layer: 1, marker: 'shelf-a', dominant_name: '测试咖啡', count: 2 }
    ] };
    const box = http.createServer((req, res) => {
        boxRequests.push({ method: req.method, path: req.url });
        res.setHeader('Content-Type', 'application/json');
        res.end(JSON.stringify({ code: 0, data: { running: false, task: scanTask } }));
    });
    await new Promise((resolve) => box.listen(0, '127.0.0.1', resolve));
    t.after(() => box.close());
    const boxUrl = `http://127.0.0.1:${box.address().port}`;
    fs.writeFileSync(productsFile, JSON.stringify({
        products: [{ id: 1, name: '测试咖啡', stock: 10, status: 1, price: 8, location: '测试区A货架第2层', chassis_marker: 'shelf-a', storeId: 'STORE-001' }]
    }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    const child = spawn(process.execPath, ['server.js'], {
        cwd: __dirname + '/..',
        env: {
            ...process.env,
            HOST: '127.0.0.1',
            PORT: String(port),
            WS_HOST: '127.0.0.1',
            WS_PORT: String(wsPort),
            PRODUCTS_DB_FILE: productsFile,
            OPERATIONS_DB_FILE: operationsFile,
            ORDERS_DB_FILE: ordersFile,
            CHASSIS_SETTINGS_FILE: chassisSettingsFile,
            BOX_PROXY_MAP: `STORE-001=${boxUrl},STORE-002=${boxUrl}`,
            ORDER_TOKEN_SECRET: 'merchant-test-order-token-secret-32bytes',
            PAYMENT_MODE: 'free',
            ALLOW_FREE_ORDERS: '1',
            ADMIN_USERNAME: 'merchant-admin',
            ADMIN_PASSWORD: 'merchant-admin-password',
            ADMIN_SESSION_SECRET: 'merchant-admin-session-secret-32-bytes',
            MERCHANT_SESSION_SECRET: 'merchant-session-secret-at-least-32-bytes'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let output = '';
    child.stdout.on('data', (chunk) => output += chunk.toString());
    child.stderr.on('data', (chunk) => output += chunk.toString());
    t.after(() => {
        if (child.exitCode === null) child.kill('SIGTERM');
    });

    await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error(`服务启动超时: ${output}`)), 5000);
        const inspect = () => {
            if (!output.includes('HTTP 服务已启动')) return;
            clearTimeout(timeout);
            resolve();
        };
        child.stdout.on('data', inspect);
        child.stderr.on('data', inspect);
        child.once('exit', (code) => reject(new Error(`服务提前退出 ${code}: ${output}`)));
    });

    const baseUrl = `http://127.0.0.1:${port}`;
    const login = await fetch(`${baseUrl}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'merchant-admin', password: 'merchant-admin-password' })
    });
    const adminCookie = login.headers.get('set-cookie').split(';', 1)[0];

    const storeA = await requestJson(`${baseUrl}/api/admin/stores`, { method: 'POST', cookie: adminCookie, body: { name: '门店A' } });
    const storeB = await requestJson(`${baseUrl}/api/admin/stores`, { method: 'POST', cookie: adminCookie, body: { name: '门店B' } });
    assert.equal(storeA.response.status, 201);
    assert.equal(storeB.response.status, 201);

    const shortPassword = await requestJson(`${baseUrl}/api/admin/merchants`, {
        method: 'POST', cookie: adminCookie, body: { username: 'merchant-a', password: '123' }
    });
    assert.equal(shortPassword.response.status, 400);

    const merchant = await requestJson(`${baseUrl}/api/admin/merchants`, {
        method: 'POST', cookie: adminCookie,
        body: { username: 'merchant-a', name: '门店A商家', password: 'secret123', storeIds: [storeA.json.data.id] }
    });
    assert.equal(merchant.response.status, 201);
    assert.deepEqual(merchant.json.data.storeIds, [storeA.json.data.id]);
    assert.equal(JSON.stringify(merchant.json.data).includes('passwordHash'), false);

    const merchantList = await requestJson(`${baseUrl}/api/admin/merchants`, { cookie: adminCookie });
    assert.equal(merchantList.json.total, 1);

    const wrongLogin = await requestJson(`${baseUrl}/api/merchant/login`, { method: 'POST', body: { username: 'merchant-a', password: 'wrong' } });
    assert.equal(wrongLogin.response.status, 401);

    const merchantLogin = await fetch(`${baseUrl}/api/merchant/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'merchant-a', password: 'secret123' })
    });
    assert.equal(merchantLogin.status, 200);
    const merchantCookie = merchantLogin.headers.get('set-cookie').split(';', 1)[0];

    const orderA = await requestJson(`${baseUrl}/api/orders/prepare`, {
        method: 'POST', body: { items: [{ id: 1, qty: 1 }], storeId: storeA.json.data.id }
    });
    const orderB = await requestJson(`${baseUrl}/api/orders/prepare`, {
        method: 'POST', body: { items: [{ id: 1, qty: 1 }], storeId: storeB.json.data.id }
    });
    assert.equal(orderA.response.status, 200);
    assert.equal(orderB.response.status, 200);

    const merchantOrders = await requestJson(`${baseUrl}/api/merchant/orders`, { cookie: merchantCookie });
    assert.equal(merchantOrders.json.total, 1);
    assert.equal(merchantOrders.json.data[0].storeId, storeA.json.data.id);

    const refundOrderA = await requestJson(`${baseUrl}/api/merchant/orders/${encodeURIComponent(orderA.json.orderId)}/refund`, {
        method: 'POST', cookie: merchantCookie, body: { reason: '商家测试退款' }
    });
    assert.equal(refundOrderA.response.status, 200);
    assert.equal(refundOrderA.json.data.paymentStatus, 'REFUNDED');

    const refundForeignOrder = await requestJson(`${baseUrl}/api/merchant/orders/${encodeURIComponent(orderB.json.orderId)}/refund`, {
        method: 'POST', cookie: merchantCookie, body: { reason: '越权退款' }
    });
    assert.equal(refundForeignOrder.response.status, 404);

    const overview = await requestJson(`${baseUrl}/api/merchant/overview`, { cookie: merchantCookie });
    assert.equal(overview.json.data.stores.length, 1);
    assert.equal(overview.json.data.stores[0].id, storeA.json.data.id);
    assert.equal(overview.json.data.orderCount, 1);
    assert.equal(overview.json.data.productCount, 1);

    const merchantProducts = await requestJson(`${baseUrl}/api/merchant/products`, { cookie: merchantCookie });
    assert.equal(merchantProducts.json.total, 1);
    assert.equal(merchantProducts.json.data[0].storeId, storeA.json.data.id);

    const edited = await requestJson(`${baseUrl}/api/merchant/products/1`, {
        method: 'PUT', cookie: merchantCookie, body: { price: 9.5, stock: 3, status: 0 }
    });
    assert.equal(edited.response.status, 200);
    assert.equal(edited.json.data.price, 9.5);
    assert.equal(edited.json.data.status, 0);

    const createdProduct = await requestJson(`${baseUrl}/api/merchant/products`, {
        method: 'POST', cookie: merchantCookie, body: { name: '商家新增商品', price: 5, stock: 20, storeId: storeA.json.data.id }
    });
    assert.equal(createdProduct.response.status, 201);
    assert.equal(createdProduct.json.data.storeId, storeA.json.data.id);

    const crossStoreCreate = await requestJson(`${baseUrl}/api/merchant/products`, {
        method: 'POST', cookie: merchantCookie, body: { name: '越权商品', price: 5, stock: 1, storeId: storeB.json.data.id }
    });
    assert.equal(crossStoreCreate.response.status, 400);

    const foreignProduct = await requestJson(`${baseUrl}/api/products`, {
        method: 'POST', cookie: adminCookie, body: { name: '门店B商品', price: 6, stock: 5, storeId: storeB.json.data.id }
    });
    assert.equal(foreignProduct.response.status, 200);
    const foreignEdit = await requestJson(`${baseUrl}/api/merchant/products/${foreignProduct.json.id}`, {
        method: 'PUT', cookie: merchantCookie, body: { price: 1 }
    });
    assert.equal(foreignEdit.response.status, 403);

    const deletedProduct = await requestJson(`${baseUrl}/api/merchant/products/${createdProduct.json.data.id}`, {
        method: 'DELETE', cookie: merchantCookie
    });
    assert.equal(deletedProduct.response.status, 200);

    const forbiddenAdmin = await requestJson(`${baseUrl}/api/admin/devices`, { cookie: merchantCookie });
    assert.equal(forbiddenAdmin.response.status, 401);

    const beforeReview = JSON.parse(fs.readFileSync(productsFile, 'utf8'));
    const submitted = await requestJson(`${baseUrl}/api/merchant/console/inventory-products`, {
        method: 'POST', cookie: merchantCookie,
        body: { taskId: scanTask.id, storeId: storeB.json.data.id, rows: [{ name: 'forged' }] }
    });
    assert.equal(submitted.response.status, 201);
    assert.equal(submitted.json.data.storeId, storeA.json.data.id);
    assert.equal(submitted.json.data.rows[0].name, '测试咖啡');
    const afterReview = JSON.parse(fs.readFileSync(productsFile, 'utf8'));
    assert.deepEqual(afterReview.products, beforeReview.products);
    assert.deepEqual(afterReview.reservedOrders, beforeReview.reservedOrders);
    assert.ok(boxRequests.every((request) => request.method === 'GET'));
    const alias = await requestJson(`${baseUrl}/api/merchant/console/inventory-scan/products`, {
        method: 'POST', cookie: merchantCookie, body: { taskId: scanTask.id }
    });
    assert.equal(alias.json.data.id, submitted.json.data.id);
    const crossStoreScan = await requestJson(`${baseUrl}/api/merchant/console/inventory-products?storeId=${storeB.json.data.id}`, {
        method: 'POST', cookie: merchantCookie, body: { taskId: scanTask.id }
    });
    assert.equal(crossStoreScan.response.status, 403);
    const reviewUrl = `${baseUrl}/api/admin/inventory-reviews/${submitted.json.data.id}`;
    const forbiddenReview = await requestJson(reviewUrl, {
        method: 'POST', cookie: merchantCookie, body: { action: 'approve' }
    });
    assert.equal(forbiddenReview.response.status, 401);
    fs.writeFileSync(path.join(tempDir, 'inventory_scan.json'), JSON.stringify({ last_scan: scanTask }));
    await requestJson(`${baseUrl}/api/admin/inventory-reviews`, {
        method: 'POST', cookie: adminCookie, body: { storeId: storeB.json.data.id, taskId: scanTask.id }
    });
    const ownReviews = await requestJson(`${baseUrl}/api/merchant/inventory-reviews`, { cookie: merchantCookie });
    assert.equal(ownReviews.json.data.length, 1);
    const allReviews = await requestJson(`${baseUrl}/api/admin/inventory-reviews`, { cookie: adminCookie });
    assert.equal(allReviews.json.data.length, 2);
    const rejected = await requestJson(reviewUrl, {
        method: 'POST', cookie: adminCookie, body: { action: 'reject', reason: '重新核对规格' }
    });
    assert.equal(rejected.response.status, 200);
    const status = await requestJson(`${baseUrl}/api/merchant/inventory-reviews`, { cookie: merchantCookie });
    assert.equal(status.json.data[0].status, 'rejected');
    assert.equal(status.json.data[0].reason, '重新核对规格');
    assert.deepEqual(JSON.parse(fs.readFileSync(productsFile, 'utf8')).products, beforeReview.products);

    const disabled = await requestJson(`${baseUrl}/api/admin/merchants/${merchant.json.data.id}`, {
        method: 'PUT', cookie: adminCookie, body: { status: 'disabled' }
    });
    assert.equal(disabled.response.status, 200);
    const afterDisable = await requestJson(`${baseUrl}/api/merchant/overview`, { cookie: merchantCookie });
    assert.equal(afterDisable.response.status, 401);
});
