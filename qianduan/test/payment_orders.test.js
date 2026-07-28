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
            const port = server.address().port;
            server.close((error) => error ? reject(error) : resolve(port));
        });
    });
}

async function postJson(url, body, headers = {}) {
    const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', ...headers },
        body: JSON.stringify(body)
    });
    return { response, json: await response.json() };
}

test('free payment order is persisted, completed and refundable', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-orders-'));
    const productsFile = path.join(tempDir, 'products.json');
    const ordersFile = path.join(tempDir, 'orders.json');
    fs.writeFileSync(productsFile, JSON.stringify({ products: [
        { id: 1, name: '测试咖啡', spec: '250ml', price: 8, stock: 5, status: 1, location: '测试区A货架第1层' }
    ] }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    const child = spawn(process.execPath, ['server.js'], {
        cwd: path.join(__dirname, '..'),
        env: {
            ...process.env,
            HOST: '127.0.0.1', PORT: String(port), WS_HOST: '127.0.0.1', WS_PORT: String(wsPort),
            ROBOT_GRASP_MODE: 'sim', PRODUCTS_DB_FILE: productsFile, ORDERS_DB_FILE: ordersFile,
            PAYMENT_MODE: 'free', ALLOW_FREE_ORDERS: '1',
            ADMIN_USERNAME: 'test-admin', ADMIN_PASSWORD: 'test-password',
            ADMIN_SESSION_SECRET: 'test-admin-session-secret-32-bytes', ORDER_TOKEN_SECRET: 'test-order-secret'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let output = '';
    child.stdout.on('data', (chunk) => output += chunk.toString());
    child.stderr.on('data', (chunk) => output += chunk.toString());
    t.after(() => { if (child.exitCode === null) child.kill('SIGTERM'); });
    await new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error(`服务启动超时: ${output}`)), 5000);
        const ready = () => {
            if (!output.includes('HTTP 服务已启动')) return;
            clearTimeout(timer);
            resolve();
        };
        child.stdout.on('data', ready);
        child.stderr.on('data', ready);
        child.once('exit', (code) => reject(new Error(`服务提前退出 ${code}: ${output}`)));
    });

    const base = `http://127.0.0.1:${port}`;
    const prepared = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 2 }] });
    assert.equal(prepared.response.status, 200);
    assert.equal(prepared.json.paymentRequired, false);
    assert.equal(prepared.json.totalAmount, 16);

    const payment = await fetch(`${base}/api/orders/${prepared.json.orderId}/payment?token=${encodeURIComponent(prepared.json.completionToken)}`);
    assert.equal((await payment.json()).paid, true);

    const login = await fetch(`${base}/api/admin/login`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'test-admin', password: 'test-password' })
    });
    const cookie = login.headers.get('set-cookie').split(';', 1)[0];
    const list = await fetch(`${base}/api/admin/orders?page=1&pageSize=10`, { headers: { Cookie: cookie } });
    const listJson = await list.json();
    assert.equal(listJson.total, 1);
    assert.equal(listJson.data[0].status, 'PAID');

    const completed = await postJson(`${base}/api/orders/${prepared.json.orderId}/complete`, { completionToken: prepared.json.completionToken });
    assert.equal(completed.response.status, 200);
    const refunded = await postJson(`${base}/api/admin/orders/${prepared.json.orderId}/refund`, { reason: '测试退款' }, { Cookie: cookie });
    assert.equal(refunded.response.status, 200);
    assert.equal(refunded.json.data.status, 'REFUNDED');
    const products = await (await fetch(`${base}/api/products`)).json();
    assert.equal(products.data[0].stock, 5);
});
