const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const fs = require('node:fs');
const http = require('node:http');
const net = require('node:net');
const { once } = require('node:events');
const os = require('node:os');
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

async function startFakeChassisServer(port) {
    const commands = [];
    let moveStarted = false;
    let statusPolls = 0;
    const marker = {
        floor: 1,
        marker_name: 'shelf-a',
        key: 0,
        pose: {
            position: { x: 1.0, y: 2.0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
    };
    const server = net.createServer((socket) => {
        let pending = '';
        socket.on('data', (chunk) => {
            pending += chunk.toString();
            if (!pending.includes('\n')) return;
            const command = pending.split(/\r?\n/, 1)[0];
            commands.push(command);
            let response;
            if (command === '/api/map/get_current_map') {
                response = {
                    status: 'OK',
                    results: {
                        map_name: 'test-map',
                        floor: 1,
                        info: { resolution: 0.05, width: 200, height: 100, origin_x: -5, origin_y: -2.5 }
                    }
                };
            } else if (command === '/api/markers/query_list') {
                response = { status: 'OK', results: { 'shelf-a': marker } };
            } else if (command === '/api/robot_status') {
                statusPolls += 1;
                const arrived = moveStarted && statusPolls >= 3;
                response = {
                    status: 'OK',
                    results: {
                        move_status: moveStarted ? (arrived ? 'succeeded' : 'running') : 'idle',
                        running_status: moveStarted && !arrived ? 'running' : 'idle',
                        move_target: moveStarted ? 'shelf-a' : '',
                        current_floor: 1,
                        current_pose: arrived
                            ? { x: 1.0, y: 2.0, theta: 0 }
                            : { x: 0, y: 0, theta: 0 }
                    }
                };
            } else if (command.startsWith('/api/move?')) {
                moveStarted = true;
                response = { status: 'OK', task_id: 'task-1' };
            } else if (command === '/api/move/cancel') {
                response = { status: 'OK' };
            } else {
                response = { status: 'INVALID_REQUEST', error_message: command };
            }
            const payload = JSON.stringify(response);
            const split = Math.max(1, Math.floor(payload.length / 2));
            socket.write(payload.slice(0, split));
            setImmediate(() => socket.end(payload.slice(split)));
        });
    });
    await new Promise((resolve, reject) => {
        server.once('error', reject);
        server.listen(port, '127.0.0.1', resolve);
    });
    return { server, commands };
}

async function startFakeChassisMapServer(port) {
    const png = Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=', 'base64');
    const requests = [];
    const server = http.createServer((req, res) => {
        requests.push(req.url);
        if (req.url.startsWith('/maps/test-map/1/map.yaml?')) {
            res.writeHead(200, { 'Content-Type': 'text/yaml' });
            res.end('image: map.png\nresolution: 0.05\norigin: [-5.0, -2.5, 0.0]\n');
            return;
        }
        if (req.url.startsWith('/maps/test-map/1/map.png?')) {
            res.writeHead(200, { 'Content-Type': 'image/png' });
            res.end(png);
            return;
        }
        res.writeHead(404);
        res.end();
    });
    await new Promise((resolve, reject) => {
        server.once('error', reject);
        server.listen(port, '127.0.0.1', resolve);
    });
    return { server, requests, png };
}

test('WebSocket real mode navigates before worker startup, moves to each layer, and stows', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const chassisPort = await getFreePort();
    const chassisMapPort = await getFreePort();
    const fakeChassis = await startFakeChassisServer(chassisPort);
    const fakeChassisMap = await startFakeChassisMapServer(chassisMapPort);
    t.after(() => fakeChassis.server.close());
    t.after(() => {
        if (fakeChassisMap.server.listening) fakeChassisMap.server.close();
    });
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-ws-'));
    const dataFile = path.join(tempDir, 'products.json');
    const shelfLayersFile = path.join(tempDir, 'shelf_layers.json');
    fs.writeFileSync(dataFile, JSON.stringify({
        products: []
    }));
    fs.writeFileSync(shelfLayersFile, JSON.stringify({
        layers: {
            2: { name: '第2层', pose: [1, 2, 3, 4, 5, 6] }
        },
        current: 2
    }));
    t.after(() => fs.rmSync(tempDir, { recursive: true, force: true }));
    const child = spawn(process.execPath, ['server.js'], {
        cwd: __dirname + '/..',
        env: {
            ...process.env,
            PORT: String(port),
            WS_PORT: String(wsPort),
            ROBOT_GRASP_MODE: 'real',
            GRASP_SCRIPT: path.join(__dirname, 'fixtures', 'fake_shelf_grasp.py'),
            PRODUCTS_DB_FILE: dataFile,
            SHELF_LAYERS_FILE: shelfLayersFile,
            ROBOT_WORKER_READY_TIMEOUT_MS: '5000',
            ROBOT_GRAB_TIMEOUT_MS: '5000',
            ROBOT_STOW_TIMEOUT_MS: '5000',
            CHASSIS_NAVIGATION_ENABLE: '1',
            CHASSIS_HOST: '127.0.0.1',
            CHASSIS_PORT: String(chassisPort),
            CHASSIS_MAP_HTTP_HOST: '127.0.0.1',
            CHASSIS_MAP_HTTP_PORT: String(chassisMapPort),
            CHASSIS_REQUEST_TIMEOUT_MS: '500',
            CHASSIS_NAVIGATION_POLL_MS: '10',
            CHASSIS_NAVIGATION_TIMEOUT_MS: '2000',
            ADMIN_USERNAME: 'test-admin',
            ADMIN_PASSWORD: 'test-password',
            ADMIN_SESSION_SECRET: 'test-admin-session-secret-32-bytes'
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
    const loginResponse = await fetch(`http://127.0.0.1:${port}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'test-admin', password: 'test-password' })
    });
    assert.equal(loginResponse.status, 200);
    const cookie = loginResponse.headers.get('set-cookie').split(';', 1)[0];
    const mapResponse = await fetch(`http://127.0.0.1:${port}/api/chassis/map`, {
        headers: { Cookie: cookie }
    });
    const mapPayload = await mapResponse.json();
    assert.equal(mapResponse.status, 200);
    assert.equal(mapPayload.data.map.map_name, 'test-map');
    assert.equal(mapPayload.data.markers[0].name, 'shelf-a');
    assert.equal(mapPayload.data.raster.available, true);
    assert.equal(mapPayload.data.raster.metadata.resolution, 0.05);
    assert.deepEqual(mapPayload.data.raster.metadata.origin, [-5, -2.5, 0]);
    const imageResponse = await fetch(`http://127.0.0.1:${port}${mapPayload.data.raster.image_url}`, {
        headers: { Cookie: cookie }
    });
    assert.equal(imageResponse.status, 200);
    assert.equal(imageResponse.headers.get('content-type'), 'image/png');
    assert.deepEqual(Buffer.from(await imageResponse.arrayBuffer()), fakeChassisMap.png);
    const snapshotFile = path.join(tempDir, 'chassis_map_cache', 'current.json');
    const cachedImage = path.join(tempDir, 'chassis_map_cache', 'test-map', '1', 'map.png');
    assert.equal(fs.existsSync(snapshotFile), true);
    assert.deepEqual(fs.readFileSync(cachedImage), fakeChassisMap.png);
    await new Promise((resolve, reject) => fakeChassisMap.server.close((error) => error ? reject(error) : resolve()));
    const cachedImageResponse = await fetch(`http://127.0.0.1:${port}${mapPayload.data.raster.image_url}`, {
        headers: { Cookie: cookie }
    });
    assert.equal(cachedImageResponse.status, 200);
    assert.deepEqual(Buffer.from(await cachedImageResponse.arrayBuffer()), fakeChassisMap.png);

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

    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-BAD-LAYER',
        items: [{ id: 1, name: '错误层商品', qty: 1, shelf_layer: 99 }]
    }));
    const layerError = await collector.waitFor(
        (message) => message.type === 'error' && /shelf_layer/.test(message.message)
    );
    assert.match(layerError.message, /1\.\.10/);

    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-NO-MARKER',
        items: [{ id: 999998, name: '未绑定点位商品', qty: 1, shelf_layer: 2 }]
    }));
    const markerError = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'error'
            && message.orderId === 'ORD-NO-MARKER'
    );
    assert.match(markerError.message, /未配置底盘点位/);

    ws.send('null');
    const objectError = await collector.waitFor(
        (message) => message.type === 'error' && message.message === 'JSON 指令必须是对象'
    );
    assert.equal(objectError.message, 'JSON 指令必须是对象');

    const orderId = 'ORD-TEST123';
    ws.send(JSON.stringify({
        type: 'grab',
        orderId,
        items: [{ id: 999999, name: '云端咖啡', qty: 2, emoji: 'coffee', shelf_layer: 2, chassis_marker: 'shelf-a' }]
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
    const commands = [...serverOutput.matchAll(/\[fake-worker-command\] (.+)/g)]
        .map((match) => match[1].trim());
    const chassisMoveIndex = serverOutput.indexOf('底盘开始前往 shelf-a');
    const workerStartIndex = serverOutput.indexOf('启动常驻抓取 worker');
    assert.ok(chassisMoveIndex >= 0, serverOutput);
    assert.ok(workerStartIndex > chassisMoveIndex, serverOutput);
    assert.deepEqual(commands, ['__layer_2__', '云端咖啡', '__layer_2__', '云端咖啡']);
    assert.equal(fakeChassis.commands.filter((command) => command.startsWith('/api/move?')).length, 1);
    assert.equal(collector.messages.some((message) => message.type === 'robot_log'), false);

    ws.send(JSON.stringify({ type: 'stow' }));
    const stowed = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'completed' && message.message === '机械臂已收起'
    );
    assert.equal(stowed.progress, 100);

    ws.close();
    child.kill('SIGTERM');
    await once(child, 'close');

    const restartPort = await getFreePort();
    const restartWsPort = await getFreePort();
    const restarted = spawn(process.execPath, ['server.js'], {
        cwd: __dirname + '/..',
        env: {
            ...process.env,
            PORT: String(restartPort),
            WS_PORT: String(restartWsPort),
            ROBOT_GRASP_MODE: 'sim',
            PRODUCTS_DB_FILE: dataFile,
            CHASSIS_HOST: '127.0.0.1',
            CHASSIS_PORT: String(chassisPort),
            CHASSIS_MAP_HTTP_HOST: '127.0.0.1',
            CHASSIS_MAP_HTTP_PORT: String(chassisMapPort),
            CHASSIS_REQUEST_TIMEOUT_MS: '500',
            ADMIN_USERNAME: 'test-admin',
            ADMIN_PASSWORD: 'test-password',
            ADMIN_SESSION_SECRET: 'test-admin-session-secret-32-bytes'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let restartOutput = '';
    restarted.stdout.on('data', (chunk) => restartOutput += chunk.toString());
    restarted.stderr.on('data', (chunk) => restartOutput += chunk.toString());
    t.after(() => {
        if (restarted.exitCode === null) restarted.kill('SIGTERM');
    });
    await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error(`重启服务超时: ${restartOutput}`)), 5000);
        const inspect = () => {
            if (!restartOutput.includes('WebSocket 服务已启动')) return;
            clearTimeout(timeout);
            resolve();
        };
        restarted.stdout.on('data', inspect);
        restarted.stderr.on('data', inspect);
        restarted.once('exit', (code) => reject(new Error(`重启服务提前退出 ${code}: ${restartOutput}`)));
    });
    const restartLogin = await fetch(`http://127.0.0.1:${restartPort}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'test-admin', password: 'test-password' })
    });
    const restartCookie = restartLogin.headers.get('set-cookie').split(';', 1)[0];
    const persistedMapResponse = await fetch(`http://127.0.0.1:${restartPort}/api/chassis/map`, {
        headers: { Cookie: restartCookie }
    });
    const persistedMap = await persistedMapResponse.json();
    assert.equal(persistedMapResponse.status, 200);
    assert.equal(persistedMap.data.raster.storage, 'local_snapshot');
    const persistedImageResponse = await fetch(`http://127.0.0.1:${restartPort}${persistedMap.data.raster.image_url}`, {
        headers: { Cookie: restartCookie }
    });
    assert.equal(persistedImageResponse.status, 200);
    assert.deepEqual(Buffer.from(await persistedImageResponse.arrayBuffer()), fakeChassisMap.png);
});
