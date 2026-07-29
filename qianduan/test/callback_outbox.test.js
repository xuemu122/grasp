const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const { once } = require('node:events');
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

function createOrderToken(secret, orderId, items) {
    const payload = Buffer.from(JSON.stringify({ orderId, items, exp: Date.now() + 60_000 })).toString('base64url');
    const signature = crypto.createHmac('sha256', secret).update(payload).digest('base64url');
    return `${payload}.${signature}`;
}

async function waitFor(predicate, timeoutMs = 8000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
        const value = await predicate();
        if (value) return value;
        await new Promise((resolve) => setTimeout(resolve, 50));
    }
    throw new Error('等待条件超时');
}

test('completed callback survives failure and is retried from disk with HMAC', async (t) => {
    const httpPort = await getFreePort();
    const wsPort = await getFreePort();
    const callbackPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-callback-outbox-'));
    const productsFile = path.join(tempDir, 'products.json');
    const ordersFile = path.join(tempDir, 'orders.json');
    const outboxFile = path.join(tempDir, 'callback-outbox.json');
    const callbackSecret = 'callback-outbox-secret-at-least-32-bytes';
    const orderTokenSecret = 'callback-order-token-secret-at-least-32-bytes';
    const orderId = 'ORD-CALLBACK-OUTBOX';
    const items = [{ id: 1, name: '测试商品', qty: 1, shelf_layer: 2, chassis_marker: 'shelf-a' }];
    const completionToken = createOrderToken(orderTokenSecret, orderId, items);
    fs.writeFileSync(productsFile, JSON.stringify({ products: [
        { id: 1, name: '测试商品', stock: 5, status: 1, price: 1, location: '测试区第2层' }
    ] }));
    fs.writeFileSync(ordersFile, JSON.stringify({ orders: [] }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));

    const completionRequests = [];
    const callbackServer = http.createServer((req, res) => {
        const chunks = [];
        req.on('data', (chunk) => chunks.push(chunk));
        req.on('end', () => {
            const body = JSON.parse(Buffer.concat(chunks).toString('utf8') || '{}');
            if (req.url.endsWith('/started')) {
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify({ code: 0, data: { status: 'PROCESSING' } }));
                return;
            }
            if (req.url.endsWith('/complete')) {
                completionRequests.push({ body, headers: req.headers });
                const code = completionRequests.length === 1 ? 1 : 0;
                res.writeHead(code === 0 ? 200 : 503, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify(code === 0 ? { code: 0, data: { status: 'COMPLETED' } } : { code: 1, message: '临时不可用' }));
                return;
            }
            res.writeHead(404, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ code: 1 }));
        });
    });
    await new Promise((resolve, reject) => callbackServer.listen(callbackPort, '127.0.0.1', resolve).once('error', reject));
    t.after(() => callbackServer.close());

    const serviceEnv = {
            ...process.env,
            HOST: '127.0.0.1', PORT: String(httpPort), WS_HOST: '127.0.0.1', WS_PORT: String(wsPort),
            ROBOT_GRASP_MODE: 'sim', ROBOT_SIM_DELAY_SCALE: '0', PAYMENT_MODE: 'free', ALLOW_FREE_ORDERS: '1',
            PRODUCTS_DB_FILE: productsFile, ORDERS_DB_FILE: ordersFile,
            ORDER_CALLBACK_OUTBOX_FILE: outboxFile, ORDER_CALLBACK_RETRY_INTERVAL_MS: '100',
            ORDER_STATUS_CALLBACK_URL: `http://127.0.0.1:${callbackPort}`,
            ORDER_STATUS_CALLBACK_SECRET: callbackSecret, ORDER_TOKEN_SECRET: orderTokenSecret,
            CHASSIS_HOST: '127.0.0.1', CHASSIS_PORT: '1', CHASSIS_MAP_HTTP_HOST: '127.0.0.1', CHASSIS_MAP_HTTP_PORT: '1',
            CHASSIS_REQUEST_TIMEOUT_MS: '50'
    };
    const child = spawn(process.execPath, ['server.js'], {
        cwd: path.join(__dirname, '..'),
        env: serviceEnv,
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let output = '';
    child.stdout.on('data', (chunk) => output += chunk.toString());
    child.stderr.on('data', (chunk) => output += chunk.toString());
    t.after(() => { if (child.exitCode === null) child.kill('SIGTERM'); });
    await waitFor(() => output.includes('WebSocket 服务已启动'));

    const ws = new WebSocket(`ws://127.0.0.1:${wsPort}`);
    await new Promise((resolve, reject) => ws.once('open', resolve).once('error', reject));
    t.after(() => ws.close());
    const messages = [];
    ws.on('message', (raw) => messages.push(JSON.parse(raw.toString())));
    ws.send(JSON.stringify({ type: 'grab', orderId, completionToken, items }));
    await waitFor(() => messages.find((message) => message.type === 'status' && message.status === 'completed'));

    const queued = JSON.parse(fs.readFileSync(outboxFile, 'utf8'));
    assert.equal(queued.events.length, 1);
    assert.equal(queued.events[0].orderId, orderId);
    assert.equal(completionRequests.length, 1);

    ws.close();
    child.kill('SIGTERM');
    await once(child, 'close');
    const restarted = spawn(process.execPath, ['server.js'], {
        cwd: path.join(__dirname, '..'),
        env: serviceEnv,
        stdio: ['ignore', 'pipe', 'pipe']
    });
    t.after(() => { if (restarted.exitCode === null) restarted.kill('SIGTERM'); });
    await waitFor(() => completionRequests.length >= 2);
    await waitFor(() => JSON.parse(fs.readFileSync(outboxFile, 'utf8')).events.length === 0);
    const retried = completionRequests[1];
    const timestamp = retried.headers['x-grasp-timestamp'];
    const expectedSignature = crypto.createHmac('sha256', callbackSecret)
        .update(`${timestamp}:${orderId}:${completionToken}:completed`)
        .digest('base64url');
    assert.equal(retried.headers['x-grasp-signature'], expectedSignature);
    assert.deepEqual(retried.body, { orderId, completionToken });
});
