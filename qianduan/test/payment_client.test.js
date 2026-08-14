const assert = require('node:assert/strict');
const http = require('node:http');
const test = require('node:test');

const { PaymentClient } = require('../payment_client');

test('partial refund sends an explicit amount contract', async (t) => {
    const captured = [];
    const server = http.createServer((req, res) => {
        const chunks = [];
        req.on('data', (chunk) => chunks.push(chunk));
        req.on('end', () => {
            captured.push({
                body: JSON.parse(Buffer.concat(chunks).toString('utf8')),
                idempotencyKey: req.headers['idempotency-key']
            });
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ code: 0, data: { refundId: 'RF-PARTIAL', refundStatus: 'SUCCESS' } }));
        });
    });
    await new Promise((resolve, reject) => server.listen(0, '127.0.0.1', resolve).once('error', reject));
    t.after(() => server.close());

    const address = server.address();
    const client = new PaymentClient({ baseUrl: `http://127.0.0.1:${address.port}` });
    const result = await client.refund('PAY-1', '第二瓶失败', 'REFUND-UNIT-2', {
        refundAmount: 8,
        totalAmount: 16
    });

    assert.equal(result.refundStatus, 'SUCCESS');
    assert.equal(captured[0].idempotencyKey, 'REFUND-UNIT-2');
    assert.deepEqual(captured[0].body, {
        orderId: 'PAY-1',
        reason: '第二瓶失败',
        refundRequestId: 'REFUND-UNIT-2',
        partial: true,
        refundAmount: 8,
        totalAmount: 16,
        amount: { refund: 800, total: 1600, currency: 'CNY' }
    });

    await client.refund('PAY-1', '退还剩余金额', 'REFUND-UNIT-3', {
        refundAmount: 8,
        totalAmount: 16
    });
    assert.equal(captured[1].idempotencyKey, 'REFUND-UNIT-3');
    assert.equal(captured[1].body.refundRequestId, 'REFUND-UNIT-3');
    assert.equal(captured[1].body.refundAmount, 8);
    assert.notEqual(captured[0].idempotencyKey, captured[1].idempotencyKey);
});
