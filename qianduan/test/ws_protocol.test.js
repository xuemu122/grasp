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
    let moveTarget = '';
    const shelfMarker = {
        floor: 1,
        marker_name: 'shelf-a',
        key: 0,
        pose: {
            position: { x: 1.0, y: 2.0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
    };
    const chargeMarker = {
        floor: 1,
        marker_name: 'charge_point_1F_35744',
        key: 11,
        pose: {
            position: { x: -1.0, y: 0.5, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
    };
    const markers = { 'shelf-a': shelfMarker, 'charge_point_1F_35744': chargeMarker };
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
                response = { status: 'OK', results: markers };
            } else if (command === '/api/robot_status') {
                statusPolls += 1;
                const arrived = moveStarted && statusPolls >= 3;
                const docking = moveStarted && moveTarget === 'charge_point_1F_35744' && statusPolls >= 2 && !arrived;
                const targetMarker = markers[moveTarget] || shelfMarker;
                response = {
                    status: 'OK',
                    results: {
                        move_status: moveStarted ? (arrived ? 'succeeded' : 'running') : 'idle',
                        running_status: moveStarted && !arrived ? (docking ? 'dock_to_charging_pile' : 'running') : 'idle',
                        move_target: moveStarted ? moveTarget : '',
                        current_floor: 1,
                        current_pose: arrived || docking
                            ? { x: targetMarker.pose.position.x, y: targetMarker.pose.position.y, theta: 0 }
                            : { x: 0, y: 0, theta: 0 }
                    }
                };
            } else if (command.startsWith('/api/move?')) {
                moveStarted = true;
                statusPolls = 0;
                moveTarget = new URLSearchParams(command.split('?', 2)[1]).get('marker') || '';
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

test('real mode powers on at startup, runs orders, and supports admin stow and poweroff', async (t) => {
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
        products: [{
            id: 1,
            name: '云端咖啡',
            stock: 10,
            status: 1,
            location: '测试区A货架第2层',
            chassis_marker: 'shelf-a'
        }]
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
            ORDER_READY_CLOSE_DELAY_MS: '20',
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
    assert.ok(['standby', 'ready'].includes(connected.robotStatus));

    let robotStatusPayload;
    for (let attempt = 0; attempt < 50; attempt += 1) {
        const robotStatusResponse = await fetch(`http://127.0.0.1:${port}/api/robot/status`, {
            headers: { Cookie: cookie }
        });
        robotStatusPayload = await robotStatusResponse.json();
        if (robotStatusPayload.data.status === 'ready') break;
        await new Promise((resolve) => setTimeout(resolve, 20));
    }
    assert.equal(robotStatusPayload.data.status, 'ready');
    assert.equal(robotStatusPayload.data.worker_ready, true);
    assert.match(serverOutput, /\[fake-worker-idle-startup\] 1/);

    ws.send(JSON.stringify({ type: 'ping' }));
    const pong = await collector.waitFor((message) => message.type === 'pong');
    assert.equal(typeof pong.time, 'number');

    ws.send(JSON.stringify({ type: 'stow' }));
    const publicStowError = await collector.waitFor(
        (message) => message.type === 'error' && message.message === '收起操作仅可在管理页执行'
    );
    assert.equal(publicStowError.message, '收起操作仅可在管理页执行');

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

    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-HAND-FAIL',
        items: [{ id: 999997, name: '灵巧手失败商品', qty: 1, shelf_layer: 2, chassis_marker: 'shelf-a' }]
    }));
    const handError = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'error'
            && message.orderId === 'ORD-HAND-FAIL'
    );
    assert.match(handError.message, /灵巧手闭合\/保持失败/);
    assert.equal(
        collector.messages.some((message) => message.type === 'status'
            && message.status === 'completed' && message.orderId === 'ORD-HAND-FAIL'),
        false
    );

    ws.send('null');
    const objectError = await collector.waitFor(
        (message) => message.type === 'error' && message.message === 'JSON 指令必须是对象'
    );
    assert.equal(objectError.message, 'JSON 指令必须是对象');

    const prepareResponse = await fetch(`http://127.0.0.1:${port}/api/orders/prepare`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ items: [{ id: 1, name: '云端咖啡', qty: 2 }] })
    });
    const preparedOrder = await prepareResponse.json();
    assert.equal(prepareResponse.status, 200);
    assert.equal(preparedOrder.items[0].chassis_marker, 'shelf-a');
    const orderId = preparedOrder.orderId;
    ws.send(JSON.stringify({
        type: 'grab',
        orderId,
        completionToken: preparedOrder.completionToken,
        items: [{ id: 1, name: '云端咖啡', qty: 2 }]
    }));
    const started = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'started' && message.orderId === orderId
    );
    assert.equal(started.message, '订单已接收');

    const completed = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'completed' && message.orderId === orderId
    );
    assert.equal(completed.progress, 100);
    assert.equal(completed.phase, 'placed');
    assert.match(completed.message, /商品已放置/);

    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-WHILE-RETURNING',
        items: [{ id: 1, name: '云端咖啡', qty: 1, shelf_layer: 2, chassis_marker: 'shelf-a' }]
    }));
    const returningBusyError = await collector.waitFor(
        (message) => message.type === 'error' && /暂不能提交新订单/.test(message.message)
    );
    assert.match(returningBusyError.message, /暂不能提交新订单/);

    const orderReady = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'order-ready' && message.orderId === orderId
    );
    assert.equal(orderReady.progress, 100);
    const returnPhases = collector.messages
        .filter((message) => message.type === 'status' && message.status === 'returning' && message.orderId === orderId)
        .map((message) => message.phase);
    assert.deepEqual(returnPhases, ['stowing', 'charging', 'settling']);
    assert.ok(collector.messages.some((message) =>
        message.type === 'robot_status'
        && message.status === 'chassis-moving'
        && /回充电桩/.test(message.message)
    ));
    const completedIndex = collector.messages.indexOf(completed);
    const returningIndex = collector.messages.findIndex(
        (message) => message.type === 'status' && message.status === 'returning' && message.orderId === orderId
    );
    const orderReadyIndex = collector.messages.indexOf(orderReady);
    assert.ok(completedIndex >= 0 && returningIndex > completedIndex && orderReadyIndex > returningIndex);
    assert.equal(
        collector.messages.filter((message) => message.orderId === orderId && message.stage === 'picking').length,
        2
    );
    const commands = [...serverOutput.matchAll(/\[fake-worker-command\] (.+)/g)]
        .map((match) => match[1].trim());
    const chassisMoveIndex = serverOutput.indexOf('底盘开始前往 shelf-a');
    const workerStartIndex = serverOutput.indexOf('启动常驻抓取 worker');
    assert.ok(chassisMoveIndex >= 0, serverOutput);
    assert.ok(workerStartIndex >= 0 && workerStartIndex < chassisMoveIndex, serverOutput);
    assert.match(serverOutput, /已从签名凭证恢复货架层和底盘点位/);
    assert.deepEqual(commands, [
        '__stow_pose__',
        '__layer_2__',
        '灵巧手失败商品',
        '__layer_2__',
        '云端咖啡',
        '__layer_2__',
        '云端咖啡',
        '__stow_pose__'
    ]);
    assert.equal(fakeChassis.commands.filter((command) => command.startsWith('/api/move?')).length, 2);
    assert.ok(fakeChassis.commands.some((command) => command.includes('marker=charge_point_1F_35744')));
    assert.equal(collector.messages.some((message) => message.type === 'robot_log'), false);

    const disableReturnResponse = await fetch(`http://127.0.0.1:${port}/api/chassis/navigation-config`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json', Cookie: cookie },
        body: JSON.stringify({ auto_return_charge: false })
    });
    const disabledReturn = await disableReturnResponse.json();
    assert.equal(disableReturnResponse.status, 200);
    assert.equal(disabledReturn.data.auto_return_charge, false);
    const noReturnOrderId = 'ORD-NO-AUTO-RETURN';
    ws.send(JSON.stringify({
        type: 'grab',
        orderId: noReturnOrderId,
        items: [{ id: 1, name: '云端咖啡', qty: 1, shelf_layer: 2, chassis_marker: 'shelf-a' }]
    }));
    const noReturnCompleted = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'completed' && message.orderId === noReturnOrderId
    );
    assert.match(noReturnCompleted.message, /正在收起/);
    const noReturnStowed = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'returning'
            && message.orderId === noReturnOrderId && message.phase === 'stowed'
    );
    assert.match(noReturnStowed.message, /自动回充已关闭/);
    const noReturnReady = await collector.waitFor(
        (message) => message.type === 'status' && message.status === 'order-ready' && message.orderId === noReturnOrderId
    );
    assert.match(noReturnReady.message, /已收起/);
    assert.equal(
        collector.messages.some((message) =>
            message.type === 'status' && message.status === 'returning'
            && message.orderId === noReturnOrderId && message.phase === 'charging'
        ),
        false
    );

    const unauthorizedPoweroff = await fetch(`http://127.0.0.1:${port}/api/robot/poweroff`, { method: 'POST' });
    assert.equal(unauthorizedPoweroff.status, 401);

    const stowResponse = await fetch(`http://127.0.0.1:${port}/api/robot/stow`, {
        method: 'POST',
        headers: { Cookie: cookie }
    });
    const stowPayload = await stowResponse.json();
    assert.equal(stowResponse.status, 200);
    assert.equal(stowPayload.code, 0);
    assert.match(stowPayload.message, /保持上电/);

    const poweroffResponse = await fetch(`http://127.0.0.1:${port}/api/robot/poweroff`, {
        method: 'POST',
        headers: { Cookie: cookie }
    });
    const poweroffPayload = await poweroffResponse.json();
    assert.equal(poweroffResponse.status, 200);
    assert.equal(poweroffPayload.code, 0);

    const poweredOffStatusResponse = await fetch(`http://127.0.0.1:${port}/api/robot/status`, {
        headers: { Cookie: cookie }
    });
    const poweredOffStatus = await poweredOffStatusResponse.json();
    assert.equal(poweredOffStatus.data.status, 'powered-off');
    assert.equal(poweredOffStatus.data.powered_off, true);

    ws.send(JSON.stringify({
        type: 'grab',
        orderId: 'ORD-AFTER-POWEROFF',
        items: [{ id: 1, name: '云端咖啡', qty: 1, shelf_layer: 2, chassis_marker: 'shelf-a' }]
    }));
    const poweredOffError = await collector.waitFor(
        (message) => message.type === 'error' && /已断电关闭/.test(message.message)
    );
    assert.match(poweredOffError.message, /重启服务重新上电/);
    assert.equal((serverOutput.match(/启动常驻抓取 worker/g) || []).length, 1);

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
