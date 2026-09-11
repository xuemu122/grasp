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

test('devices and stores lifecycle: create, checkout, checkin, summary, order assignment', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-devices-'));
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
            ORDER_TOKEN_SECRET: 'devices-test-order-token-secret-32-bytes',
            PAYMENT_MODE: 'free',
            ALLOW_FREE_ORDERS: '1',
            ADMIN_USERNAME: 'devices-admin',
            ADMIN_PASSWORD: 'devices-password',
            ADMIN_SESSION_SECRET: 'devices-admin-session-secret-32-bytes'
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

    const unauthorized = await requestJson(`${baseUrl}/api/admin/stores`);
    assert.equal(unauthorized.response.status, 401);

    const login = await fetch(`${baseUrl}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'devices-admin', password: 'devices-password' })
    });
    const cookie = login.headers.get('set-cookie').split(';', 1)[0];

    const storeA = await requestJson(`${baseUrl}/api/admin/stores`, { method: 'POST', cookie, body: { name: '门店A', code: 'ST-001' } });
    assert.equal(storeA.response.status, 201);
    assert.equal(storeA.json.data.name, '门店A');
    const storeB = await requestJson(`${baseUrl}/api/admin/stores`, { method: 'POST', cookie, body: { name: '门店B' } });
    assert.equal(storeB.response.status, 201);
    const duplicateStore = await requestJson(`${baseUrl}/api/admin/stores`, { method: 'POST', cookie, body: { name: '门店A' } });
    assert.equal(duplicateStore.response.status, 400);

    const device1 = await requestJson(`${baseUrl}/api/admin/devices`, { method: 'POST', cookie, body: { sn: 'GRASP-ROBOT-01', name: '1号机器人' } });
    assert.equal(device1.response.status, 201);
    assert.equal(device1.json.data.status, 'in_stock');
    assert.equal(device1.json.data.storeId, null);
    const device2 = await requestJson(`${baseUrl}/api/admin/devices`, { method: 'POST', cookie, body: { sn: 'GRASP-ROBOT-02', name: '2号机器人' } });
    assert.equal(device2.response.status, 201);
    const duplicateDevice = await requestJson(`${baseUrl}/api/admin/devices`, { method: 'POST', cookie, body: { sn: 'GRASP-ROBOT-01', name: '重复' } });
    assert.equal(duplicateDevice.response.status, 400);

    const checkout1 = await requestJson(`${baseUrl}/api/admin/devices/${device1.json.data.id}/checkout`, {
        method: 'POST', cookie, body: { storeId: storeA.json.data.id, location: '一层大厅' }
    });
    assert.equal(checkout1.response.status, 200);
    assert.equal(checkout1.json.data.status, 'deployed');
    assert.equal(checkout1.json.data.storeId, storeA.json.data.id);

    const checkoutMissingStore = await requestJson(`${baseUrl}/api/admin/devices/${device2.json.data.id}/checkout`, {
        method: 'POST', cookie, body: {}
    });
    assert.equal(checkoutMissingStore.response.status, 400);

    await requestJson(`${baseUrl}/api/admin/devices/${device2.json.data.id}/checkout`, {
        method: 'POST', cookie, body: { storeId: storeB.json.data.id }
    });

    const storeADevices = await requestJson(`${baseUrl}/api/admin/devices?storeId=${storeA.json.data.id}`, { cookie });
    assert.equal(storeADevices.json.total, 1);
    assert.equal(storeADevices.json.data[0].sn, 'GRASP-ROBOT-01');

    const summary = await requestJson(`${baseUrl}/api/admin/operations/summary`, { cookie });
    assert.equal(summary.json.data.storeCount, 2);
    assert.equal(summary.json.data.deviceCount, 2);
    assert.equal(summary.json.data.deviceStatusCounts.deployed, 2);
    assert.equal(summary.json.data.storeStats.find((row) => row.name === '门店A').deviceCount, 1);

    const boundDelete = await requestJson(`${baseUrl}/api/admin/stores/${storeB.json.data.id}`, { method: 'DELETE', cookie });
    assert.equal(boundDelete.response.status, 409);
    const forcedDelete = await requestJson(`${baseUrl}/api/admin/stores/${storeB.json.data.id}?force=1`, { method: 'DELETE', cookie });
    assert.equal(forcedDelete.response.status, 200);

    const afterDelete = await requestJson(`${baseUrl}/api/admin/devices?status=in_stock`, { cookie });
    assert.equal(afterDelete.json.total, 1);
    assert.equal(afterDelete.json.data[0].sn, 'GRASP-ROBOT-02');

    const checkin = await requestJson(`${baseUrl}/api/admin/devices/${device1.json.data.id}/checkin`, {
        method: 'POST', cookie, body: { location: '总仓A区', notes: '巡检' }
    });
    assert.equal(checkin.json.data.status, 'in_stock');
    assert.equal(checkin.json.data.storeId, null);

    const prepared = await requestJson(`${baseUrl}/api/orders/prepare`, {
        method: 'POST', body: { items: [{ id: 1, qty: 1 }], storeId: storeA.json.data.id, deviceId: device1.json.data.id }
    });
    assert.equal(prepared.response.status, 200);

    const orders = await requestJson(`${baseUrl}/api/admin/orders?storeId=${storeA.json.data.id}`, { cookie });
    assert.equal(orders.json.total, 1);
    assert.equal(orders.json.data[0].storeId, storeA.json.data.id);
    assert.equal(orders.json.data[0].deviceId, device1.json.data.id);

    const reassigned = await requestJson(`${baseUrl}/api/admin/orders/${encodeURIComponent(prepared.json.orderId)}/assignment`, {
        method: 'PUT', cookie, body: { storeId: storeA.json.data.id, deviceId: device2.json.data.id }
    });
    assert.equal(reassigned.response.status, 200);
    assert.equal(reassigned.json.data.deviceId, device2.json.data.id);
});
