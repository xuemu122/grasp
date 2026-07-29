const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const http = require('node:http');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');
const { spawn } = require('node:child_process');
const test = require('node:test');
const WebSocket = require('ws');

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

async function postJson(url, body) {
    const response = await fetch(url, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body)
    });
    return { response, json: await response.json() };
}

async function waitFor(predicate, timeoutMs = 6000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        const result = await predicate();
        if (result) return result;
        await new Promise((resolve) => setTimeout(resolve, 50));
    }
    throw new Error('等待条件超时');
}

test('paid stock is reserved once and abandoned paid orders are refunded server-side', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const paymentPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-payment-security-'));
    const productsFile = path.join(tempDir, 'products.json');
    const ordersFile = path.join(tempDir, 'orders.json');
    const callbackSecret = 'security-callback-secret-at-least-32-bytes';
    fs.writeFileSync(productsFile, JSON.stringify({ products: [
        { id: 1, name: '最后一件', price: 10, stock: 1, status: 1, payment_menu_item_id: 101 },
        { id: 2, name: '无人页面订单', price: 5, stock: 1, status: 1, payment_menu_item_id: 102 },
        { id: 3, name: '主动取消订单', price: 6, stock: 1, status: 1, payment_menu_item_id: 103 }
    ] }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    let sequence = 0;
    const tradeStates = new Map();
    const totals = new Map();
    const refunds = [];
    const failedRefunds = new Set();
    const paymentServer = http.createServer((req, res) => {
        const chunks = [];
        req.on('data', (chunk) => chunks.push(chunk));
        req.on('end', () => {
            const body = chunks.length ? JSON.parse(Buffer.concat(chunks).toString('utf8')) : {};
            if (req.method === 'POST' && req.url === '/api/Order/create') {
                const externalOrderId = `PAY-${++sequence}`;
                const totalPrice = body.items.reduce((sum, item) => sum + Number(item.unitPrice) * Number(item.quantity), 0);
                tradeStates.set(externalOrderId, 'NOTPAY');
                totals.set(externalOrderId, totalPrice);
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: { orderId: externalOrderId, totalPrice } }));
                return;
            }
            if (req.method === 'POST' && req.url === '/api/WeChatPay/native') {
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: { codeUrl: `weixin://pay/${body.orderId}` } }));
                return;
            }
            if (req.method === 'GET' && req.url.startsWith('/api/WeChatPay/query/')) {
                const externalOrderId = decodeURIComponent(req.url.split('/').pop());
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: {
                    tradeState: tradeStates.get(externalOrderId) || 'NOTPAY',
                    totalAmount: totals.get(externalOrderId), currency: 'CNY'
                } }));
                return;
            }
            if (req.method === 'POST' && req.url === '/api/WeChatPay/refund') {
                if (failedRefunds.has(body.orderId)) {
                    res.writeHead(503, { 'Content-Type': 'application/json' });
                    res.end(JSON.stringify({ code: 1, message: 'refund temporarily unavailable' }));
                    return;
                }
                refunds.push({ body, idempotencyKey: req.headers['idempotency-key'] });
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: {
                    refundId: `RF-${body.orderId}`, refundStatus: 'SUCCESS'
                } }));
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
            ADMIN_USERNAME: 'security-admin', ADMIN_PASSWORD: 'security-admin-random-password',
            ADMIN_SESSION_SECRET: 'security-admin-session-secret-at-least-32-bytes',
            ORDER_TOKEN_SECRET: 'security-order-token-secret-at-least-32-bytes',
            ORDER_STATUS_CALLBACK_SECRET: callbackSecret,
            PAYMENT_STATUS_SWEEP_INTERVAL_MS: '100', PAYMENT_STATUS_QUERY_INTERVAL_MS: '50',
            ORDER_PAID_START_TIMEOUT_MS: '100', ORDER_REFUND_SWEEP_INTERVAL_MS: '100',
            CHASSIS_HOST: '127.0.0.1', CHASSIS_PORT: '1', CHASSIS_MAP_HTTP_HOST: '127.0.0.1', CHASSIS_MAP_HTTP_PORT: '1',
            CHASSIS_REQUEST_TIMEOUT_MS: '50'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let output = '';
    child.stdout.on('data', (chunk) => output += chunk.toString());
    child.stderr.on('data', (chunk) => output += chunk.toString());
    t.after(() => { if (child.exitCode === null) child.kill('SIGTERM'); });
    await waitFor(() => output.includes('HTTP 服务已启动'));
    const base = `http://127.0.0.1:${port}`;

    const ws = new WebSocket(`ws://127.0.0.1:${wsPort}`);
    await new Promise((resolve, reject) => ws.once('open', resolve).once('error', reject));
    t.after(() => ws.close());
    const unsignedOrderError = new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('未收到无签名订单拒绝消息')), 2000);
        ws.on('message', (raw) => {
            const message = JSON.parse(raw.toString());
            if (message.type !== 'error' || !/签名凭证/.test(message.message)) return;
            clearTimeout(timer);
            resolve(message);
        });
    });
    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-UNSIGNED',
        items: [{ id: 1, name: '最后一件', qty: 1 }]
    }));
    assert.match((await unsignedOrderError).message, /拒绝执行/);

    const first = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 1 }] });
    const second = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 1 }] });
    const refundFailure = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 1 }] });
    assert.equal(first.response.status, 200);
    assert.equal(second.response.status, 200);
    assert.equal(refundFailure.response.status, 200);
    tradeStates.set('PAY-1', 'SUCCESS');
    tradeStates.set('PAY-2', 'SUCCESS');
    tradeStates.set('PAY-3', 'SUCCESS');
    failedRefunds.add('PAY-3');

    const firstPaid = await postJson(`${base}/api/orders/${first.json.orderId}/payment`, { completionToken: first.json.completionToken });
    const secondPaid = await postJson(`${base}/api/orders/${second.json.orderId}/payment`, { completionToken: second.json.completionToken });
    assert.equal(firstPaid.json.paid, true);
    assert.equal(secondPaid.json.paid, false);
    assert.equal(secondPaid.json.paymentStatus, 'REFUNDED');
    assert.equal(refunds.length, 1);
    assert.equal(refunds[0].body.refundRequestId, refunds[0].idempotencyKey);
    assert.match(refunds[0].body.refundRequestId, /^GRASP-REFUND-/);
    assert.equal(JSON.parse(fs.readFileSync(productsFile, 'utf8')).products[0].stock, 0);

    const failedRefundAttempt = await postJson(
        `${base}/api/orders/${refundFailure.json.orderId}/payment`,
        { completionToken: refundFailure.json.completionToken }
    );
    assert.equal(failedRefundAttempt.response.status, 502);
    const failedRefundStatus = await postJson(
        `${base}/api/orders/${refundFailure.json.orderId}/payment`,
        { completionToken: refundFailure.json.completionToken }
    );
    assert.equal(failedRefundStatus.response.status, 200);
    assert.equal(failedRefundStatus.json.paid, false);
    assert.equal(failedRefundStatus.json.status, 'FAILED');
    assert.match(failedRefundStatus.json.message, /退款暂未成功/);
    const startedTimestamp = String(Date.now());
    const startedSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${startedTimestamp}:${first.json.orderId}:${first.json.completionToken}:started`)
        .digest('base64url');
    const started = await fetch(`${base}/api/orders/${first.json.orderId}/started`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-Grasp-Timestamp': startedTimestamp,
            'X-Grasp-Signature': startedSignature
        },
        body: JSON.stringify({ completionToken: first.json.completionToken })
    });
    assert.equal(started.status, 200);

    const cancelled = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 3, qty: 1 }] });
    assert.equal(cancelled.response.status, 200);
    tradeStates.set('PAY-4', 'SUCCESS');
    const cancelledResult = await postJson(
        `${base}/api/orders/${cancelled.json.orderId}/cancel-payment`,
        { completionToken: cancelled.json.completionToken }
    );
    assert.equal(cancelledResult.response.status, 200);
    assert.equal(cancelledResult.json.data.status, 'REFUNDED');
    assert.equal(JSON.parse(fs.readFileSync(productsFile, 'utf8')).products[2].stock, 1);
    assert.equal(refunds.length, 2);

    const abandoned = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 2, qty: 1 }] });
    assert.equal(abandoned.response.status, 200);
    tradeStates.set('PAY-5', 'SUCCESS');
    const abandonedOrder = await waitFor(() => {
        const order = JSON.parse(fs.readFileSync(ordersFile, 'utf8')).orders.find((item) => item.orderId === abandoned.json.orderId);
        return order && order.status === 'REFUNDED' ? order : null;
    });
    assert.equal(abandonedOrder.refundTrigger, 'paid-not-started-timeout');
    assert.equal(JSON.parse(fs.readFileSync(productsFile, 'utf8')).products[1].stock, 1);
    assert.equal(refunds.length, 3);
});
