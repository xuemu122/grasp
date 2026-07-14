const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const net = require('node:net');
const { once } = require('node:events');
const path = require('node:path');
const test = require('node:test');
const WebSocket = require('ws');


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

function createMessageCollector(ws) {
    const messages = [];
    const waiters = [];
    ws.on('message', (raw) => {
        const message = JSON.parse(raw.toString());
        messages.push(message);
        for (const waiter of [...waiters]) {
            if (!waiter.predicate(message)) continue;
            clearTimeout(waiter.timer);
            waiters.splice(waiters.indexOf(waiter), 1);
            waiter.resolve(message);
        }
    });

    return {
        messages,
        waitFor(predicate, timeoutMs = 5000) {
            const existing = messages.find(predicate);
            if (existing) return Promise.resolve(existing);
            return new Promise((resolve, reject) => {
                const waiter = { predicate, resolve, timer: null };
                waiter.timer = setTimeout(() => {
                    waiters.splice(waiters.indexOf(waiter), 1);
                    reject(new Error(`等待 WebSocket 消息超时，已收到: ${JSON.stringify(messages)}`));
                }, timeoutMs);
                waiters.push(waiter);
            });
        }
    };
}

test('WebSocket real mode trusts cloud items, repeats qty, and stows', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const child = spawn(process.execPath, ['server.js'], {
        cwd: __dirname + '/..',
        env: {
            ...process.env,
            PORT: String(port),
            WS_PORT: String(wsPort),
            ROBOT_GRASP_MODE: 'real',
            GRASP_SCRIPT: path.join(__dirname, 'fixtures', 'fake_shelf_grasp.py'),
            ROBOT_WORKER_READY_TIMEOUT_MS: '5000',
            ROBOT_GRAB_TIMEOUT_MS: '5000',
            ROBOT_STOW_TIMEOUT_MS: '5000'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let serverOutput = '';
    child.stdout.on('data', (chunk) => serverOutput += chunk.toString());
    child.stderr.on('data', (chunk) => serverOutput += chunk.toString());
    t.after(() => {
        if (child.exitCode === null) child.kill('SIGTERM');
    });

    await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error(`服务启动超时: ${serverOutput}`)), 5000);
        const inspect = () => {
            if (!serverOutput.includes('WebSocket 服务已启动')) return;
            clearTimeout(timeout);
            resolve();
        };
        child.stdout.on('data', inspect);
        child.stderr.on('data', inspect);
        child.once('exit', (code) => reject(new Error(`服务提前退出 ${code}: ${serverOutput}`)));
    });

    const ws = new WebSocket(`ws://127.0.0.1:${wsPort}`);
    const collector = createMessageCollector(ws);
    t.after(() => ws.close());
    await once(ws, 'open');

    const connected = await collector.waitFor((message) => message.type === 'connected');
    assert.equal(connected.robotStatus, 'standby');

    ws.send(JSON.stringify({ type: 'ping' }));
    const pong = await collector.waitFor((message) => message.type === 'pong');
    assert.equal(typeof pong.time, 'number');

    ws.send(JSON.stringify({ type: 'grab', items: [] }));
    const protocolError = await collector.waitFor(
        (message) => message.type === 'error' && message.message === '缺少 orderId 或 items'
    );
    assert.equal(protocolError.message, '缺少 orderId 或 items');

    ws.send('null');
    const objectError = await collector.waitFor(
        (message) => message.type === 'error' && message.message === 'JSON 指令必须是对象'
    );
    assert.equal(objectError.message, 'JSON 指令必须是对象');

    const orderId = 'ORD-TEST123';
    ws.send(JSON.stringify({
        type: 'grab',
        orderId,
        items: [{ id: 999999, name: '云端咖啡', qty: 2, emoji: 'coffee' }]
    }));
    const started = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'started' && message.orderId === orderId
    );
    assert.equal(started.message, '订单已接收');

    const completed = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'completed' && message.orderId === orderId
    );
    assert.equal(completed.progress, 100);
    assert.equal(
        collector.messages.filter((message) => message.orderId === orderId && message.stage === 'picking').length,
        2
    );
    assert.equal(collector.messages.some((message) => message.type === 'robot_log'), false);

    ws.send(JSON.stringify({ type: 'stow' }));
    const stowed = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'completed' && message.message === '机械臂已收起'
    );
    assert.equal(stowed.progress, 100);
});
