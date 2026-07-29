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

async function postJson(url, body) {
    const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
    });
    return { response, json: await response.json() };
}

test('completed orders decrement inventory once', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-inventory-'));
    const dataFile = path.join(tempDir, 'products.json');
    const shelfLayersFile = path.join(tempDir, 'shelf_layers.json');
    const chassisSettingsFile = path.join(tempDir, 'chassis_settings.json');
    fs.writeFileSync(dataFile, JSON.stringify({
        products: [
            { id: 1, name: '测试咖啡', stock: 5, status: 1, price: 8, location: '测试区A货架第2层', chassis_marker: 'shelf-a' },
            { id: 2, name: '下架商品', stock: 5, status: 0, price: 5 },
            { id: 3, name: '售罄商品', stock: 0, status: 1, price: 6 }
        ]
    }));
    fs.writeFileSync(shelfLayersFile, JSON.stringify({
        layers: {
            1: { name: '第1层', pose: [1, 2, 3, 4, 5, 6] },
            2: { name: '第2层', pose: [6, 5, 4, 3, 2, 1] }
        },
        current: 1
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
            PRODUCTS_DB_FILE: dataFile,
            SHELF_LAYERS_FILE: shelfLayersFile,
            CHASSIS_SETTINGS_FILE: chassisSettingsFile,
            ORDER_TOKEN_SECRET: 'inventory-test-secret',
            PAYMENT_MODE: 'free',
            ALLOW_FREE_ORDERS: '1',
            ADMIN_USERNAME: 'inventory-admin',
            ADMIN_PASSWORD: 'inventory-password',
            ADMIN_SESSION_SECRET: 'inventory-admin-session-secret-32-bytes'
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
    const chassisMapUnauthorized = await fetch(`${baseUrl}/api/chassis/map`);
    assert.equal(chassisMapUnauthorized.status, 401);

    const loginResponse = await fetch(`${baseUrl}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'inventory-admin', password: 'inventory-password' })
    });
    const adminCookie = loginResponse.headers.get('set-cookie').split(';', 1)[0];
    const navigationConfigResponse = await fetch(`${baseUrl}/api/chassis/navigation-config`, {
        headers: { Cookie: adminCookie }
    });
    const navigationConfig = await navigationConfigResponse.json();
    assert.equal(navigationConfig.data.navigation_enabled, false);
    assert.equal(navigationConfig.data.auto_return_charge, true);
    assert.equal(navigationConfig.data.place_marker, 'fangzhi');
    const updateNavigationResponse = await fetch(`${baseUrl}/api/chassis/navigation-config`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json', Cookie: adminCookie },
        body: JSON.stringify({ enabled: true, auto_return_charge: false, place_marker: 'delivery-a' })
    });
    const updatedNavigationConfig = await updateNavigationResponse.json();
    assert.equal(updatedNavigationConfig.data.navigation_enabled, true);
    assert.equal(updatedNavigationConfig.data.auto_return_charge, false);
    assert.equal(updatedNavigationConfig.data.place_marker, 'delivery-a');

    const shelfLayersResponse = await fetch(`${baseUrl}/api/shelf/layers`);
    const shelfLayers = await shelfLayersResponse.json();
    assert.equal(shelfLayersResponse.status, 200);
    assert.deepEqual(Object.keys(shelfLayers.data), ['1', '2']);
    assert.equal(shelfLayers.current, 1);

    const publicProductsResponse = await fetch(`${baseUrl}/api/products?include_inactive=1`);
    const publicProducts = await publicProductsResponse.json();
    assert.deepEqual(publicProducts.data.map((item) => item.id).sort(), [1, 3]);
    assert.equal(publicProducts.data.find((item) => item.id === 1).chassis_marker, 'shelf-a');

    const prepared = await postJson(`${baseUrl}/api/orders/prepare`, {
        items: [{ id: 1, name: '测试咖啡', qty: 2 }]
    });
    assert.equal(prepared.response.status, 200);
    assert.equal(prepared.json.code, 0);
    assert.equal(prepared.json.items[0].shelf_layer, 2);
    assert.equal(prepared.json.items[0].chassis_marker, 'shelf-a');

    const completionUrl = `${baseUrl}/api/orders/${encodeURIComponent(prepared.json.orderId)}/complete`;
    const completed = await postJson(completionUrl, { completionToken: prepared.json.completionToken });
    assert.equal(completed.response.status, 200);
    assert.equal(completed.json.alreadyCompleted, false);
    assert.equal(completed.json.updated[0].stock, 3);

    const duplicate = await postJson(completionUrl, { completionToken: prepared.json.completionToken });
    assert.equal(duplicate.response.status, 200);
    assert.equal(duplicate.json.alreadyCompleted, true);

    const productsResponse = await fetch(`${baseUrl}/api/products`);
    const products = await productsResponse.json();
    assert.equal(products.data.find((item) => item.id === 1).stock, 3);

    const insufficient = await postJson(`${baseUrl}/api/orders/prepare`, {
        items: [{ id: 1, name: '测试咖啡', qty: 4 }]
    });
    assert.equal(insufficient.response.status, 400);
    assert.match(insufficient.json.message, /库存不足/);

    const emergencyResponse = await fetch(`${baseUrl}/api/robot/emergency-stop`, {
        method: 'POST',
        headers: { Cookie: adminCookie }
    });
    const emergency = await emergencyResponse.json();
    assert.equal(emergencyResponse.status, 200);
    assert.equal(emergency.code, 0);
    const emergencyStatusResponse = await fetch(`${baseUrl}/api/robot/status`, {
        headers: { Cookie: adminCookie }
    });
    const emergencyStatus = await emergencyStatusResponse.json();
    assert.equal(emergencyStatus.data.status, 'emergency-stopped');
    assert.equal(emergencyStatus.data.emergency_stopped, true);
});
