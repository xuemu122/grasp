const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');
const { spawn } = require('node:child_process');
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

async function waitFor(predicate, timeoutMs = 5000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        const result = await predicate();
        if (result) return result;
        await new Promise((resolve) => setTimeout(resolve, 50));
    }
    throw new Error('等待条件超时');
}

test('REFUNDING order is reconciled and inventory is restored once', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const paymentPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-refund-recovery-'));
    const productsFile = path.join(tempDir, 'products.json');
    const ordersFile = path.join(tempDir, 'orders.json');
    const orderId = 'ORD-REFUND-RECOVERY';
    fs.writeFileSync(productsFile, JSON.stringify({
        products: [{ id: 1, name: '测试商品', stock: 4, status: 1, price: 1 }],
        completedOrders: {
            [orderId]: { completedAt: Date.now() - 120_000, updated: [{ id: 1, name: '测试商品', qty: 1, stock: 4 }] }
        }
    }));
    fs.writeFileSync(ordersFile, JSON.stringify({ orders: [{
        orderId,
        externalOrderId: 'PAY-REFUND-RECOVERY',
        items: [{ id: 1, name: '测试商品', qty: 1 }],
        paymentMode: 'lyorder',
        paymentStatus: 'REFUNDING',
        status: 'REFUNDING',
        refundReason: '测试恢复退款',
        refundTrigger: 'robot-failure',
        refundLastAttemptAt: new Date(Date.now() - 120_000).toISOString(),
        completedAt: new Date(Date.now() - 180_000).toISOString(),
        createdAt: new Date(Date.now() - 300_000).toISOString(),
        updatedAt: new Date(Date.now() - 120_000).toISOString()
    }] }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    let refundRequests = 0;
    const paymentServer = http.createServer((req, res) => {
        const chunks = [];
        req.on('data', (chunk) => chunks.push(chunk));
        req.on('end', () => {
            if (req.method === 'POST' && req.url === '/api/WeChatPay/refund') {
                refundRequests += 1;
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: { refundId: 'REFUND-1', refundStatus: 'SUCCESS' } }));
                return;
            }
            res.writeHead(404, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ code: 1, message: 'not found' }));
        });
    });
    await new Promise((resolve, reject) => paymentServer.listen(paymentPort, '127.0.0.1', resolve).once('error', reject));
    t.after(() => paymentServer.close());

    const child = spawn(process.execPath, ['server.js'], {
        cwd: path.join(__dirname, '..'),
        env: {
            ...process.env,
            HOST: '127.0.0.1', PORT: String(port), WS_HOST: '127.0.0.1', WS_PORT: String(wsPort),
            ROBOT_GRASP_MODE: 'sim', PAYMENT_MODE: 'lyorder', PAYMENT_API_BASE_URL: `http://127.0.0.1:${paymentPort}`,
            PRODUCTS_DB_FILE: productsFile, ORDERS_DB_FILE: ordersFile,
            ORDER_STATUS_CALLBACK_SECRET: 'refund-recovery-callback-secret-32-bytes',
            CHASSIS_HOST: '127.0.0.1', CHASSIS_PORT: '1', CHASSIS_MAP_HTTP_HOST: '127.0.0.1', CHASSIS_MAP_HTTP_PORT: '1',
            CHASSIS_REQUEST_TIMEOUT_MS: '50'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    t.after(() => { if (child.exitCode === null) child.kill('SIGTERM'); });

    await waitFor(() => {
        const order = JSON.parse(fs.readFileSync(ordersFile, 'utf8')).orders[0];
        return order.status === 'REFUNDED' && order;
    });
    const order = JSON.parse(fs.readFileSync(ordersFile, 'utf8')).orders[0];
    const products = JSON.parse(fs.readFileSync(productsFile, 'utf8'));
    assert.equal(refundRequests, 1);
    assert.equal(order.paymentStatus, 'REFUNDED');
    assert.equal(order.refundId, 'REFUND-1');
    assert.equal(products.products[0].stock, 5);
    assert.ok(products.completedOrders[orderId].inventoryRestoredAt);

    await new Promise((resolve) => setTimeout(resolve, 150));
    assert.equal(refundRequests, 1);
    assert.equal(JSON.parse(fs.readFileSync(productsFile, 'utf8')).products[0].stock, 5);
});
