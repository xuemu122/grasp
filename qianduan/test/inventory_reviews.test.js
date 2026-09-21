const assert = require('node:assert/strict');
const test = require('node:test');
const { submitReview, decideReview, productVersion } = require('../inventory_reviews');

function fixture() {
    const store = { products: [
        { id: 1, storeId: 'S1', name: 'Cola', spec: '330ml', photo_front: '/a.jpg',
            stock: 8, price: 3, status: 1, shelf_zone: 'A', shelf_number: 1, shelf_layer: 1 },
        { id: 2, storeId: 'S1', name: 'Cola', spec: '500ml', photo_front: '/b.jpg',
            stock: 6, price: 4, status: 0, shelf_zone: 'B', shelf_number: 1, shelf_layer: 1 },
        { id: 3, storeId: 'S2', name: 'Cola', spec: '330ml', photo_front: '/c.jpg',
            stock: 10, price: 5, status: 1, shelf_zone: 'A', shelf_number: 1, shelf_layer: 1 }
    ], reservedOrders: { paid: { items: [{ id: 1, qty: 1 }] } } };
    const task = { id: 'SCAN-1', status: 'completed', results: [
        { shelf: 'A', point: 1, layer: 1, marker: 'a', dominant_name: 'Unknown', count: 200 },
        { shelf: 'B', point: 1, layer: 1, marker: 'b', dominant_name: 'Cola', count: 100 }
    ] };
    const args = { task, storeId: 'S1', source: 'box:S1', submittedBy: 'merchant:M1' };
    const review = submitReview(store, args);
    const approve = { action: 'approve', productVersion: productVersion(store.products, 'S1'),
        decisions: [{ index: 0, productId: 2 }, { index: 1, productId: 1 }] };
    return { store, review, approve, args };
}

test('submission snapshots results, is idempotent, and does not change products', () => {
    const { store, review, args } = fixture();
    const before = structuredClone(store.products);
    assert.equal(submitReview(store, args).id, review.id);
    args.task.results[0].dominant_name = 'changed';
    assert.equal(review.rows[0].name, 'Unknown');
    assert.equal(store.scanReviews.length, 1);
    assert.deepEqual(store.products, before);
});

test('review selects exact IDs, swaps placements and preserves other stores and commercial fields', () => {
    const { store, review, approve } = fixture();
    const before = structuredClone(store);
    decideReview(store, review.id, approve, 'admin');
    assert.equal(store.products[0].shelf_zone, 'B');
    assert.equal(store.products[1].shelf_zone, 'A');
    for (let i = 0; i < 2; i++) for (const field of ['name', 'spec', 'photo_front', 'stock', 'price', 'status']) {
        assert.equal(store.products[i][field], before.products[i][field]);
    }
    assert.deepEqual(store.products[2], before.products[2]);
    assert.deepEqual(store.reservedOrders, before.reservedOrders);
    assert.equal(review.status, 'approved');
    assert.equal(review.reviewedBy, 'admin');
    assert.throws(() => decideReview(store, review.id, approve, 'admin'), { statusCode: 409 });
});

test('invalid or cross-store approvals fail before changing any product or review', () => {
    for (const decisions of [
        [{ index: 0, productId: 3 }, { index: 1, productId: null }],
        [{ index: 0, productId: 1 }, { index: 1, productId: 1 }],
        [{ index: 0, productId: 1 }, { index: 0, productId: null }],
        [{ index: 0, productId: null }, { index: 1, productId: null }],
        [{ index: 0, productId: 1 }]
    ]) {
        const { store, review, approve } = fixture();
        const before = structuredClone(store);
        assert.throws(() => decideReview(store, review.id, { ...approve, decisions }, 'admin'));
        assert.deepEqual(store, before);
    }
});

test('stale placement changes require a new review while stock changes are preserved', () => {
    const { store, review, approve } = fixture();
    store.products[0].location = 'new location';
    assert.throws(() => decideReview(store, review.id, approve, 'admin'), { statusCode: 409 });
    approve.productVersion = productVersion(store.products, 'S1');
    store.products[0].stock = 4;
    decideReview(store, review.id, approve, 'admin');
    assert.equal(store.products[0].stock, 4);
});

test('rejection requires a reason and never updates products', () => {
    const { store, review } = fixture();
    const before = structuredClone(store.products);
    assert.throws(() => decideReview(store, review.id, { action: 'reject' }, 'admin'));
    decideReview(store, review.id, { action: 'reject', reason: 'Unclear scan' }, 'admin');
    assert.equal(review.status, 'rejected');
    assert.equal(review.reason, 'Unclear scan');
    assert.deepEqual(store.products, before);
});
