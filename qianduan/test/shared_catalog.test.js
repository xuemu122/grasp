const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const net = require('node:net');
const { spawn } = require('node:child_process');
const { definition, normalizedBarcode, mysqlOptions, SharedCatalog } = require('../shared_catalog');
const { migrate } = require('../tools/migrate-shared-catalog');

test('catalog identity validates GTIN and keeps variant and packaging differences', () => {
    assert.equal(normalizedBarcode('4006381333931'), '04006381333931');
    assert.throws(() => normalizedBarcode('4006381333932'));
    assert.throws(() => normalizedBarcode('abc'));
    assert.throws(() => definition({ name: 'Cola' }));
    const a = definition({ name: ' Cola ', spec: '500ml', manufacturer: 'Factory' });
    assert.equal(a.identity_hash, definition({ name: 'Ｃｏｌａ', spec: '500ML', manufacturer: 'factory' }).identity_hash);
    assert.notEqual(a.identity_hash, definition({ ...a, spec: '330ml' }).identity_hash);
    assert.notEqual(a.identity_hash, definition({ ...a, packaging: '6 bottles' }).identity_hash);
});

async function port() {
    const server = net.createServer();
    await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
    const value = server.address().port;
    await new Promise((resolve) => server.close(resolve));
    return value;
}

test('MySQL shared catalog migration and two-store HTTP workflows', { skip: process.env.CATALOG_INTEGRATION !== '1' }, async (t) => {
    const mysql = require('mysql2/promise');
    const options = mysqlOptions();
    assert.match(options.database, /^grasp_catalog_test_/);
    const pool = mysql.createPool(options);
    t.after(() => pool.end());
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'shared-catalog-'));
    t.after(() => fs.rmSync(dir, { recursive: true, force: true }));
    const file = path.join(dir, 'products.json');
    const old = { products: [{ id: 73, storeId: 'STORE-001', name: 'Cola', spec: '500ml', price: 3,
        stock: 8, status: 1, photo_front: 'images/drinks/shuirongc.svg', shelf_layer: 1 }],
        completedOrders: { old: { completedAt: 123 } }, reservedOrders: {}, scanReviews: [{ id: 'old-review' }] };
    fs.writeFileSync(file, JSON.stringify(old));
    assert.equal((await migrate(file, pool, false)).products, 1);
    assert.equal((await migrate(file, pool, true)).created, 1);
    assert.equal((await migrate(file, pool, true)).created, 0);
    const inactiveFile = path.join(dir, 'inactive.json');
    fs.writeFileSync(inactiveFile, JSON.stringify({ products: [{ id: 80, storeId: 'STORE-001', name: 'Inactive Store Item', spec: '1 bottle', status: 0 }] }));
    await migrate(inactiveFile, pool, true);
    const inactiveId = JSON.parse(fs.readFileSync(inactiveFile, 'utf8')).products[0].catalogId;
    assert.equal((await pool.query('SELECT status FROM catalog_products WHERE id=?', [inactiveId]))[0][0].status, 1);
    const migrated = JSON.parse(fs.readFileSync(file, 'utf8'));
    assert.equal(migrated.products[0].id, 73);
    assert.deepEqual(migrated.completedOrders, old.completedOrders);
    assert.deepEqual(migrated.scanReviews, old.scanReviews);
    const catalogId = migrated.products[0].catalogId;
    const catalog = new SharedCatalog(pool); await catalog.refresh();
    await assert.rejects(() => catalog.create({ name: 'Cola', spec: '500ml' }, 'test'), { statusCode: 409 });
    const httpPort = await port(); const wsPort = await port();
    const child = spawn(process.execPath, ['server.js'], { cwd: path.join(__dirname, '..'), env: { ...process.env,
        CATALOG_DB: 'mysql', HOST: '127.0.0.1', WS_HOST: '127.0.0.1', PORT: String(httpPort), WS_PORT: String(wsPort),
        PRODUCTS_DB_FILE: file, ORDERS_DB_FILE: path.join(dir, 'orders.json'), OPERATIONS_DB_FILE: path.join(dir, 'operations.json'),
        INVENTORY_SCAN_FILE: path.join(dir, 'scan.json'), CHASSIS_SETTINGS_FILE: path.join(dir, 'chassis.json'),
        ORDER_CALLBACK_OUTBOX_FILE: path.join(dir, 'outbox.json'), PRODUCT_BACKUP_DIR: path.join(dir, 'backups'),
        BOX_PROXY_MAP: '', BOX_PROXY_BASE_URL: '', STORE_ID: '', ROBOT_GRASP_MODE: 'sim', PAYMENT_MODE: 'free', ALLOW_FREE_ORDERS: '1',
        ORDER_STATUS_CALLBACK_URL: '', ADMIN_USERNAME: 'admin', ADMIN_PASSWORD: 'password-test',
        ADMIN_SESSION_SECRET: 'shared-catalog-admin-secret-32bytes', MERCHANT_SESSION_SECRET: 'shared-catalog-merchant-secret-32bytes',
        ORDER_TOKEN_SECRET: 'shared-catalog-order-secret-32bytes', ORDER_STATUS_CALLBACK_SECRET: 'shared-catalog-callback-secret-32bytes'
    }, stdio: ['ignore', 'pipe', 'pipe'] });
    t.after(async () => { if (child.exitCode === null) { child.kill(); await new Promise((resolve) => child.once('exit', resolve)); } });
    let output = ''; child.stdout.on('data', (chunk) => output += chunk); child.stderr.on('data', (chunk) => output += chunk);
    await new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error(output)), 10000);
        child.stdout.on('data', () => { if (output.includes('HTTP 服务已启动')) { clearTimeout(timer); resolve(); } });
        child.once('exit', () => { clearTimeout(timer); reject(new Error(output)); });
    });
    const base = `http://127.0.0.1:${httpPort}`;
    async function api(url, cookie = '', method = 'GET', body) {
        const res = await fetch(base + url, { method, headers: { Cookie: cookie, 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) });
        return { status: res.status, data: await res.json(), cookie: res.headers.get('set-cookie')?.split(';')[0] };
    }
    const admin = (await api('/api/admin/login', '', 'POST', { username: 'admin', password: 'password-test' })).cookie;
    for (const name of ['one', 'two']) assert.equal((await api('/api/admin/stores', admin, 'POST', { name })).status, 201);
    for (const [username, storeId] of [['one', 'STORE-001'], ['two', 'STORE-002']]) {
        assert.equal((await api('/api/admin/merchants', admin, 'POST', { username, password: 'password-test', storeIds: [storeId] })).status, 201);
    }
    const one = (await api('/api/merchant/login', '', 'POST', { username: 'one', password: 'password-test' })).cookie;
    const two = (await api('/api/merchant/login', '', 'POST', { username: 'two', password: 'password-test' })).cookie;
    assert.equal((await api('/api/merchant/catalog', two)).data.data.length, 2);
    const offer = await api('/api/merchant/products', two, 'POST', { catalogId, price: 7, stock: 4, status: 1 });
    assert.equal(offer.status, 201);
    assert.notEqual(offer.data.data.id, 73);
    const secondId = offer.data.data.id;
    assert.equal((await api('/api/merchant/products', two, 'POST', { catalogId })).status, 409);
    assert.equal((await api('/api/merchant/catalog?storeId=STORE-001', two)).status, 403);
    assert.equal((await api(`/api/merchant/products/${secondId}`, two, 'PUT', { name: 'Tampered', price: 9 })).status, 403);
    assert.equal((await api(`/api/merchant/products/${secondId}`, two, 'PUT', { price: 8, stock: 10 })).status, 200);
    assert.equal((await api('/api/merchant/products', one)).data.data[0].stock, 8);
    const current = (await api(`/api/admin/catalog/${catalogId}`, admin)).data.data;
    const update = await api(`/api/admin/catalog/${catalogId}`, admin, 'PUT', { version: current.version, name: 'Cola Updated' });
    assert.equal(update.status, 200);
    assert.equal((await api('/api/merchant/products', one)).data.data[0].name, 'Cola Updated');
    assert.equal((await api('/api/merchant/products', two)).data.data[0].name, 'Cola Updated');
    assert.equal((await api(`/api/admin/catalog/${catalogId}`, admin, 'PUT', { version: current.version, name: 'Stale' })).status, 409);
    assert.equal((await api(`/api/admin/catalog/${catalogId}`, admin, 'PUT', { version: update.data.data.version, status: 0 })).status, 409);
    const order = await api('/api/orders/prepare', '', 'POST', { items: [{ id: secondId, qty: 1 }], storeId: 'STORE-002' });
    assert.equal(order.status, 200);
    assert.equal((await api('/api/merchant/products', two)).data.data[0].stock, 9);
    assert.equal((await api('/api/orders/prepare', '', 'POST', { items: [{ id: secondId, qty: 1 }], storeId: 'STORE-001' })).status, 400);
    const storedOrders = JSON.parse(fs.readFileSync(path.join(dir, 'orders.json'), 'utf8'));
    assert.equal(storedOrders.orders[0].items[0].unitPrice, 8);
    assert.equal((await api(`/api/merchant/products/${secondId}`, two, 'DELETE')).status, 200);
    const refund = await api(`/api/admin/orders/${order.data.orderId}/refund`, admin, 'POST', { reason: 'Deleted offer refund regression' });
    assert.equal(refund.status, 200);
    assert.equal(refund.data.data.status, 'REFUNDED');
    assert.equal(JSON.parse(fs.readFileSync(file, 'utf8')).products.find((p) => p.id === secondId).stock, 10);
    const renamed = await api(`/api/admin/catalog/${catalogId}`, admin, 'PUT', { version: update.data.data.version, name: 'Cola Renamed Again' });
    assert.equal(renamed.status, 200);
    const historical = JSON.parse(fs.readFileSync(path.join(dir, 'orders.json'), 'utf8')).orders[0];
    assert.equal(historical.items[0].name, 'Cola Updated');
    assert.equal(historical.items[0].unitPrice, 8);
    const restored = await api('/api/merchant/products', two, 'POST', { catalogId, price: 9, stock: 10, status: 1 });
    assert.equal(restored.data.data.id, secondId);
    const fresh = await api('/api/merchant/products', one, 'POST', { newCatalog: { name: 'Tea', spec: '250ml' }, price: 5, stock: 3 });
    assert.equal(fresh.status, 201);
    const archived = await api('/api/admin/catalog', admin, 'POST', { name: 'Archived', spec: '250ml', status: 0 });
    assert.equal(archived.status, 201);
    assert.equal(archived.data.data.status, 0);
    assert.equal((await api('/api/merchant/products', one, 'POST', { catalogId: archived.data.data.id })).status, 400);
    assert.ok((await api('/api/merchant/catalog?search=Tea', two)).data.data.some((row) => row.name === 'Tea'));
    const history = (await pool.query('SELECT COUNT(*) AS count FROM catalog_product_changes'))[0][0].count;
    assert.ok(history >= 3);
    assert.deepEqual(JSON.parse(fs.readFileSync(file, 'utf8')).scanReviews, old.scanReviews);
});
