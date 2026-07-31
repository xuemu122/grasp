const test = require('node:test');
const assert = require('node:assert/strict');

const { orderItemsByNearestPickup } = require('../order_routing');

test('nearest pickup changes product order but preserves per-product bottle quantities', () => {
    const far = { id: 1, name: '远处可乐', qty: 2, chassis_marker: 'far' };
    const near = { id: 2, name: '近处雪碧', qty: 1, chassis_marker: 'near' };
    const middle = { id: 3, name: '中间矿泉水', qty: 1, chassis_marker: 'middle' };
    const markers = new Map([
        ['near', { x: 1, y: 0 }],
        ['middle', { x: 4, y: 0 }],
        ['far', { x: 9, y: 0 }]
    ]);

    const ordered = orderItemsByNearestPickup([far, middle, near], {
        markers,
        startPoint: { x: 0, y: 0 },
        placePoint: { x: 8, y: 0 },
        markerNameForItem: (item) => item.chassis_marker
    });

    assert.deepEqual(ordered.map((item) => item.name), ['近处雪碧', '远处可乐', '中间矿泉水']);
    assert.equal(ordered[1].qty, 2);
});

test('incomplete marker coordinates keep the paid order sequence unchanged', () => {
    const items = [
        { name: '可乐', qty: 1, chassis_marker: 'known' },
        { name: '雪碧', qty: 1, chassis_marker: 'missing' }
    ];
    const ordered = orderItemsByNearestPickup(items, {
        markers: new Map([['known', { x: 1, y: 1 }]]),
        startPoint: { x: 0, y: 0 },
        placePoint: { x: 0, y: 0 },
        markerNameForItem: (item) => item.chassis_marker
    });

    assert.equal(ordered, items);
});
