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
    fs.writeFileSync(dataFile, JSON.stringify({
        products: [
            { id: 1, name: '测试咖啡', stock: 5, status: 1, price: 8 },
            { id: 2, name: '下架商品', stock: 5, status: 0, price: 5 },
            { id: 3, name: '售罄商品', stock: 0, status: 1, price: 6 }
        ]
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
            ORDER_TOKEN_SECRET: 'inventory-test-secret'
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
    const publicProductsResponse = await fetch(`${baseUrl}/api/products?include_inactive=1`);
    const publicProducts = await publicProductsResponse.json();
    assert.deepEqual(publicProducts.data.map((item) => item.id).sort(), [1, 3]);

    const prepared = await postJson(`${baseUrl}/api/orders/prepare`, {
        items: [{ id: 1, name: '测试咖啡', qty: 2 }]
    });
    assert.equal(prepared.response.status, 200);
    assert.equal(prepared.json.code, 0);

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
});
