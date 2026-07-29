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

test('admin can upload validated product images', async (t) => {
    const port = await getFreePort();
    const wsPort = await getFreePort();
    const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'grasp-upload-'));
    const dataFile = path.join(tempDir, 'products.json');
    const uploadDir = path.join(tempDir, 'uploads');
    fs.writeFileSync(dataFile, JSON.stringify({ products: [] }));

    const child = spawn(process.execPath, ['server.js'], {
        cwd: __dirname + '/..',
        env: {
            ...process.env,
            HOST: '127.0.0.1',
            PORT: String(port),
            WS_HOST: '127.0.0.1',
            WS_PORT: String(wsPort),
            PRODUCTS_DB_FILE: dataFile,
            PRODUCT_UPLOAD_DIR: uploadDir,
            PRODUCT_UPLOAD_MAX_BYTES: '128',
            PAYMENT_MODE: 'free',
            ALLOW_FREE_ORDERS: '1',
            ADMIN_USERNAME: 'admin',
            ADMIN_PASSWORD: 'test-password',
            ADMIN_SESSION_SECRET: 'upload-test-session-secret-32-bytes'
        },
        stdio: ['ignore', 'pipe', 'pipe']
    });
    let output = '';
    child.stdout.on('data', (chunk) => output += chunk.toString());
    child.stderr.on('data', (chunk) => output += chunk.toString());
    t.after(() => {
        if (child.exitCode === null) child.kill('SIGTERM');
        fs.rmSync(tempDir, { recursive: true, force: true });
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
    const png = Buffer.from('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=', 'base64');

    const unauthorized = await fetch(`${baseUrl}/api/admin/uploads`, { method: 'POST', body: png });
    assert.equal(unauthorized.status, 401);

    const login = await fetch(`${baseUrl}/api/admin/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: 'admin', password: 'test-password' })
    });
    assert.equal(login.status, 200);
    const cookie = login.headers.get('set-cookie').split(';')[0];

    const invalid = await fetch(`${baseUrl}/api/admin/uploads`, {
        method: 'POST',
        headers: { Cookie: cookie, 'Content-Type': 'image/png' },
        body: Buffer.from('not an image')
    });
    assert.equal(invalid.status, 415);

    const oversized = await fetch(`${baseUrl}/api/admin/uploads`, {
        method: 'POST',
        headers: { Cookie: cookie, 'Content-Type': 'image/png' },
        body: Buffer.alloc(129)
    });
    assert.equal(oversized.status, 413);

    const uploaded = await fetch(`${baseUrl}/api/admin/uploads`, {
        method: 'POST',
        headers: { Cookie: cookie, 'Content-Type': 'image/png' },
        body: png
    });
    assert.equal(uploaded.status, 201);
    const result = await uploaded.json();
    assert.equal(result.code, 0);
    assert.match(result.url, /^\/api\/uploads\/[a-zA-Z0-9-]+\.png$/);

    const image = await fetch(`${baseUrl}${result.url}`);
    assert.equal(image.status, 200);
    assert.equal(image.headers.get('content-type'), 'image/png');
    assert.match(image.headers.get('cache-control'), /immutable/);
    assert.deepEqual(Buffer.from(await image.arrayBuffer()), png);
    assert.equal(fs.readdirSync(uploadDir).length, 1);
});
