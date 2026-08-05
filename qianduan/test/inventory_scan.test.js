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

async function requestJson(url, { method = 'GET', cookie = '', body } = {}) {
    const response = await fetch(url, {
        method,
        headers: {
            ...(body === undefined ? {} : { 'Content-Type': 'application/json' }),
            ...(cookie ? { Cookie: cookie } : {})
        },
        body: body === undefined ? undefined : JSON.stringify(body)
    });
    return { response, json: await response.json() };
}

test('admin inventory scan visits configured A-D shelves and each point at layers 1 then 2', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-inventory-scan-'));
    const productsFile = path.join(tempDir, 'products.json');
    const scanFile = path.join(tempDir, 'inventory_scan.json');
    const shelfLayersFile = path.join(tempDir, 'shelf_layers.json');
    fs.writeFileSync(productsFile, JSON.stringify({ products: [] }));
    fs.writeFileSync(shelfLayersFile, JSON.stringify({
        layers: {
            1: { name: '第1层', pose: [1, 2, 3, 4, 5, 6] },
            2: { name: '第2层', pose: [6, 5, 4, 3, 2, 1] }
        },
        current: 1
    }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    const simulatedResults = {};
    for (const [shelf, points] of [['A', 2], ['C', 1], ['D', 2]]) {
        for (let point = 1; point <= points; point += 1) {
        for (const layer of [1, 2]) {
            simulatedResults[`${shelf}-${point}-${layer}`] = {
                dominant_name: `商品${shelf}${point}-${layer}`,
                count: point + layer,
                total: point + layer + 1,
                candidates: [{ name: `商品${shelf}${point}-${layer}`, count: point + layer }]
            };
        }
        }
    }

    const child = spawn(process.execPath, ['server.js'], {
        cwd: path.join(__dirname, '..'),
        env: {
            ...process.env,
            HOST: '127.0.0.1',
            PORT: String(port),
            WS_HOST: '127.0.0.1',
            WS_PORT: String(wsPort),
            PRODUCTS_DB_FILE: productsFile,
            INVENTORY_SCAN_FILE: scanFile,
            SHELF_LAYERS_FILE: shelfLayersFile,
            PAYMENT_MODE: 'free',
            ALLOW_FREE_ORDERS: '1',
            ROBOT_SIM_DELAY_SCALE: '0',
            ROBOT_SCAN_SIM_RESULTS: JSON.stringify(simulatedResults),
            ADMIN_USERNAME: 'scan-admin',
            ADMIN_PASSWORD: 'scan-password',
            ADMIN_SESSION_SECRET: 'inventory-scan-admin-secret-32-bytes',
            ORDER_TOKEN_SECRET: 'inventory-scan-order-secret-32-bytes',
            ORDER_STATUS_CALLBACK_SECRET: 'inventory-scan-callback-secret-32-bytes'
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
    const unauthorized = await fetch(`${baseUrl}/api/admin/inventory-scan`);
    assert.equal(unauthorized.status, 401);

    const login = await requestJson(`${baseUrl}/api/admin/login`, {
        method: 'POST',
        body: { username: 'scan-admin', password: 'scan-password' }
    });
    assert.equal(login.response.status, 200);
    const cookie = login.response.headers.get('set-cookie').split(';', 1)[0];
    const debugNavigation = await requestJson(`${baseUrl}/api/chassis/navigate-marker`, {
        method: 'POST', cookie, body: { marker: 'shelf-a-1' }
    });
    assert.equal(debugNavigation.response.status, 200);
    assert.equal(debugNavigation.json.data.status, 'completed');
    const invalidConfig = await requestJson(`${baseUrl}/api/admin/inventory-scan/config`, {
        method: 'PUT', cookie, body: { shelves: [{ shelf: 'A', markers: [] }] }
    });
    assert.equal(invalidConfig.response.status, 400);
    const shelves = [
        { shelf: 'A', markers: ['shelf-a-1', 'shelf-a-2'] },
        { shelf: 'C', markers: ['shelf-c-1'] },
        { shelf: 'D', markers: ['shelf-d-1', 'shelf-d-2'] }
    ];
    const config = await requestJson(`${baseUrl}/api/admin/inventory-scan/config`, {
        method: 'PUT', cookie, body: { shelves }
    });
    assert.equal(config.response.status, 200);
    assert.deepEqual(config.json.data.map((row) => row.shelf), ['A', 'C', 'D']);
    assert.deepEqual(config.json.data.map((row) => row.markers.length), [2, 1, 2]);

    const started = await requestJson(`${baseUrl}/api/admin/inventory-scan/start`, { method: 'POST', cookie });
    assert.equal(started.response.status, 202);

    let snapshot;
    for (let attempt = 0; attempt < 100; attempt += 1) {
        snapshot = (await requestJson(`${baseUrl}/api/admin/inventory-scan`, { cookie })).json.data;
        if (snapshot.task.status === 'completed') break;
        await new Promise((resolve) => setTimeout(resolve, 20));
    }
    assert.equal(snapshot.task.status, 'completed');
    assert.equal(snapshot.task.progress, 100);
    assert.equal(snapshot.task.results.length, 10);
    assert.deepEqual(
        snapshot.task.results.map((row) => `${row.shelf}-${row.point}-${row.layer}`),
        ['A-1-1', 'A-1-2', 'A-2-1', 'A-2-2', 'C-1-1', 'C-1-2', 'D-1-1', 'D-1-2', 'D-2-1', 'D-2-2']
    );
    assert.equal(snapshot.task.results[0].dominant_name, '商品A1-1');

    const saved = await requestJson(`${baseUrl}/api/admin/inventory-scan/products`, {
        method: 'POST',
        cookie,
        body: {
            rows: [
                { shelf: 'A', point: 1, layer: 1, marker: 'shelf-a-1', name: '商品A1-1' },
                { shelf: 'D', point: 2, layer: 2, marker: 'shelf-d-2', name: '商品D2-2' }
            ]
        }
    });
    assert.equal(saved.response.status, 200);
    assert.equal(saved.json.data.length, 2);
    assert.deepEqual(
        saved.json.data.map((product) => [product.shelf_number, product.shelf_zone, product.shelf_layer]),
        [[1, 'A', 1], [2, 'D', 2]]
    );

    const products = await requestJson(`${baseUrl}/api/products`);
    assert.deepEqual(products.json.data.map((product) => product.location).sort(), [
        'A货架第1点位第1层',
        'D货架第2点位第2层'
    ].sort());
});
