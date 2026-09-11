const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const fs = require('node:fs');
const net = require('node:net');
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
    fs.writeFileSync(productsFile, JSON.stringify({
        products: [{ id: 1, name: '测试咖啡', stock: 10, status: 1, price: 8, location: '测试区A货架第2层', chassis_marker: 'shelf-a' }]
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

    const overview = await requestJson(`${baseUrl}/api/merchant/overview`, { cookie: merchantCookie });
    assert.equal(overview.json.data.stores.length, 1);
    assert.equal(overview.json.data.stores[0].id, storeA.json.data.id);
    assert.equal(overview.json.data.orderCount, 1);

    const forbiddenAdmin = await requestJson(`${baseUrl}/api/admin/devices`, { cookie: merchantCookie });
    assert.equal(forbiddenAdmin.response.status, 401);

    const disabled = await requestJson(`${baseUrl}/api/admin/merchants/${merchant.json.data.id}`, {
        method: 'PUT', cookie: adminCookie, body: { status: 'disabled' }
    });
    assert.equal(disabled.response.status, 200);
    const afterDisable = await requestJson(`${baseUrl}/api/merchant/overview`, { cookie: merchantCookie });
    assert.equal(afterDisable.response.status, 401);
});
