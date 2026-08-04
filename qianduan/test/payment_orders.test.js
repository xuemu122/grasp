const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const crypto = require('node:crypto');
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
    const callbackSecret = 'payment-callback-secret-at-least-32-bytes';
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
            ADMIN_SESSION_SECRET: 'test-admin-session-secret-32-bytes', ORDER_TOKEN_SECRET: 'test-order-token-secret-at-least-32-bytes',
            ORDER_STATUS_CALLBACK_SECRET: callbackSecret
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

    const payment = await fetch(`${base}/api/orders/${prepared.json.orderId}/payment`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ completionToken: prepared.json.completionToken })
    });
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

    const completedTimestamp = String(Date.now());
    const completedSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${completedTimestamp}:${prepared.json.orderId}:${prepared.json.completionToken}:completed`)
        .digest('base64url');
    const completed = await postJson(
        `${base}/api/orders/${prepared.json.orderId}/complete`,
        { orderId: prepared.json.orderId, completionToken: prepared.json.completionToken },
        { 'X-Grasp-Timestamp': completedTimestamp, 'X-Grasp-Signature': completedSignature }
    );
    assert.equal(completed.response.status, 200);
    const postDeliveryReason = '回充失败';
    const postDeliveryTimestamp = String(Date.now());
    const postDeliverySignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${postDeliveryTimestamp}:${prepared.json.orderId}:${prepared.json.completionToken}:${postDeliveryReason}`)
        .digest('base64url');
    const postDeliveryFailure = await postJson(
        `${base}/api/orders/${prepared.json.orderId}/failure`,
        { completionToken: prepared.json.completionToken, reason: postDeliveryReason },
        { 'X-Grasp-Timestamp': postDeliveryTimestamp, 'X-Grasp-Signature': postDeliverySignature }
    );
    assert.equal(postDeliveryFailure.response.status, 200);
    assert.equal(postDeliveryFailure.json.skipped, true);
    assert.equal(postDeliveryFailure.json.data.status, 'COMPLETED');

    const refunded = await postJson(`${base}/api/admin/orders/${prepared.json.orderId}/refund`, { reason: '测试退款' }, { Cookie: cookie });
    assert.equal(refunded.response.status, 200);
    assert.equal(refunded.json.data.status, 'REFUNDED');
    assert.equal(refunded.json.data.statusLabel, '已退款');
    assert.equal(refunded.json.data.paymentStatusLabel, '已退款');
    const storedOrders = JSON.parse(fs.readFileSync(ordersFile, 'utf8'));
    const legacyRefunded = storedOrders.orders.find((item) => item.orderId === prepared.json.orderId);
    legacyRefunded.status = 'FAILED';
    legacyRefunded.paymentStatus = 'PAID';
    legacyRefunded.refundStatus = 'SUCCESS';
    legacyRefunded.refundIsPartial = false;
    delete legacyRefunded.refundedAt;
    fs.writeFileSync(ordersFile, JSON.stringify(storedOrders, null, 2));
    const normalizedList = await fetch(`${base}/api/admin/orders?page=1&pageSize=10`, { headers: { Cookie: cookie } });
    const normalizedOrder = (await normalizedList.json()).data.find((item) => item.orderId === prepared.json.orderId);
    assert.equal(normalizedOrder.status, 'REFUNDED');
    assert.equal(normalizedOrder.paymentStatus, 'REFUNDED');
    const products = await (await fetch(`${base}/api/products`)).json();
    assert.equal(products.data[0].stock, 5);

    const failedPrepared = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 1 }] });
    const startedTimestamp = String(Date.now());
    const startedSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${startedTimestamp}:${failedPrepared.json.orderId}:${failedPrepared.json.completionToken}:started`)
        .digest('base64url');
    const started = await postJson(
        `${base}/api/orders/${failedPrepared.json.orderId}/started`,
        { completionToken: failedPrepared.json.completionToken },
        { 'X-Grasp-Timestamp': startedTimestamp, 'X-Grasp-Signature': startedSignature }
    );
    assert.equal(started.response.status, 200);
    assert.equal(started.json.data.status, 'PROCESSING');
    const unsafeClientRefund = await postJson(
        `${base}/api/orders/${failedPrepared.json.orderId}/refund`,
        { completionToken: failedPrepared.json.completionToken, reason: '浏览器重复退款' }
    );
    assert.equal(unsafeClientRefund.response.status, 409);

    const reason = '识别失败：未找到可抓取目标';
    const timestamp = String(Date.now());
    const signature = crypto.createHmac('sha256', callbackSecret)
        .update(`${timestamp}:${failedPrepared.json.orderId}:${failedPrepared.json.completionToken}:${reason}`)
        .digest('base64url');
    const failure = await postJson(
        `${base}/api/orders/${failedPrepared.json.orderId}/failure`,
        { completionToken: failedPrepared.json.completionToken, reason },
        { 'X-Grasp-Timestamp': timestamp, 'X-Grasp-Signature': signature }
    );
    assert.equal(failure.response.status, 200);
    assert.equal(failure.json.data.status, 'REFUNDED');
    const duplicateFailure = await postJson(
        `${base}/api/orders/${failedPrepared.json.orderId}/failure`,
        { completionToken: failedPrepared.json.completionToken, reason },
        { 'X-Grasp-Timestamp': timestamp, 'X-Grasp-Signature': signature }
    );
    assert.equal(duplicateFailure.response.status, 200);
    assert.equal(duplicateFailure.json.data.refundId, failure.json.data.refundId);
    const productsAfterFailure = await (await fetch(`${base}/api/products`)).json();
    assert.equal(productsAfterFailure.data[0].stock, 5);

    const partialPrepared = await postJson(`${base}/api/orders/prepare`, { items: [{ id: 1, qty: 2 }] });
    const partialStartedAt = String(Date.now());
    const partialStartedSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${partialStartedAt}:${partialPrepared.json.orderId}:${partialPrepared.json.completionToken}:started`)
        .digest('base64url');
    await postJson(
        `${base}/api/orders/${partialPrepared.json.orderId}/started`,
        { completionToken: partialPrepared.json.completionToken },
        { 'X-Grasp-Timestamp': partialStartedAt, 'X-Grasp-Signature': partialStartedSignature }
    );

    const partialReason = '第二瓶抓取失败';
    const partialFailedAt = String(Date.now());
    const partialFailureSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${partialFailedAt}:${partialPrepared.json.orderId}:${partialPrepared.json.completionToken}:${partialReason}`)
        .digest('base64url');
    const partialFailure = await postJson(
        `${base}/api/orders/${partialPrepared.json.orderId}/failure`,
        {
            completionToken: partialPrepared.json.completionToken,
            reason: partialReason,
            deliveredItems: [{ id: 1, qty: 1 }]
        },
        { 'X-Grasp-Timestamp': partialFailedAt, 'X-Grasp-Signature': partialFailureSignature }
    );
    assert.equal(partialFailure.response.status, 200);
    assert.equal(partialFailure.json.data.status, 'PARTIALLY_COMPLETED');
    assert.equal(partialFailure.json.data.paymentStatus, 'PARTIALLY_REFUNDED');
    assert.equal(partialFailure.json.data.paymentStatusLabel, '部分退款');
    assert.equal(partialFailure.json.data.deliveredUnits, 1);
    assert.equal(partialFailure.json.data.deliveredAmount, 8);
    assert.equal(partialFailure.json.data.refundAmount, 8);
    assert.deepEqual(partialFailure.json.data.deliveredItems.map(({ id, qty }) => ({ id, qty })), [{ id: 1, qty: 1 }]);
    const productsAfterPartialFailure = await (await fetch(`${base}/api/products`)).json();
    assert.equal(productsAfterPartialFailure.data[0].stock, 4);

    const duplicatePartialFailure = await postJson(
        `${base}/api/orders/${partialPrepared.json.orderId}/failure`,
        {
            completionToken: partialPrepared.json.completionToken,
            reason: partialReason,
            deliveredItems: [{ id: 1, qty: 1 }]
        },
        { 'X-Grasp-Timestamp': partialFailedAt, 'X-Grasp-Signature': partialFailureSignature }
    );
    assert.equal(duplicatePartialFailure.response.status, 200);
    assert.equal(duplicatePartialFailure.json.data.refundId, partialFailure.json.data.refundId);
    const productsAfterDuplicatePartialFailure = await (await fetch(`${base}/api/products`)).json();
    assert.equal(productsAfterDuplicatePartialFailure.data[0].stock, 4);
});
