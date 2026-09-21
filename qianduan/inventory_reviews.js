const crypto = require('node:crypto');

function fail(message, statusCode = 400) {
    throw Object.assign(new Error(message), { statusCode });
}

function productVersion(products, storeId) {
    // Stock changes do not invalidate a review; identity, status and placement changes do.
    const fields = ['id', 'storeId', 'name', 'spec', 'photo_front', 'status', 'location',
        'chassis_marker', 'shelf_zone', 'shelf_number', 'shelf_layer'];
    return crypto.createHash('sha256').update(JSON.stringify(products
        .filter((product) => product.storeId === storeId)
        .map((product) => fields.map((field) => product[field])))).digest('hex');
}

function submitReview(store, { task, storeId, submittedBy, source }) {
    if (!task || !task.id || !['completed', 'canceled', 'failed'].includes(task.status)) {
        fail('扫描尚未结束，请结束后提交审核', 409);
    }
    if (!Array.isArray(task.results) || !task.results.length) fail('没有可提交的扫描结果');
    const reviews = store.scanReviews || [];
    const existing = reviews.find((review) => review.taskId === task.id
        && review.storeId === storeId && review.source === source);
    if (existing) return existing;
    const rows = task.results.map((row, index) => {
        const shelf = String(row.shelf || '').toUpperCase();
        const point = Number(row.point);
        const layer = Number(row.layer);
        const marker = String(row.marker || '').trim();
        if (!/^[A-Z]{1,2}$/.test(shelf) || !Number.isInteger(point) || point < 1 || point > 6
            || ![1, 2].includes(layer) || !marker || /[\r\n]/.test(marker)) {
            fail(`第 ${index + 1} 条扫描结果缺少有效货位`);
        }
        return { index, shelf, point, layer, marker,
            name: String(row.dominant_name || ''), count: Number(row.count) || 0,
            total: Number(row.total) || 0, scannedAt: row.scanned_at || null };
    });
    if (rows.length > 1188) fail('扫描结果数量超出限制');
    const review = { id: crypto.randomUUID(), taskId: task.id, storeId, source, submittedBy,
        submittedAt: new Date().toISOString(), status: 'pending', rows };
    store.scanReviews = [...reviews, review];
    return review;
}

function decideReview(store, id, body, reviewer) {
    const review = (store.scanReviews || []).find((item) => item.id === id);
    if (!review) fail('审核记录不存在', 404);
    if (review.status !== 'pending') fail('该扫描结果已处理，请刷新列表', 409);
    if (!body || !['approve', 'reject'].includes(body.action)) fail('审核操作无效');
    const reason = String(body.reason || '').trim().slice(0, 1000);
    if (body.action === 'reject' && !reason) fail('请填写驳回原因');
    let assignments = [];
    if (body.action === 'approve') {
        if (body.productVersion !== productVersion(store.products, review.storeId)) {
            fail('门店商品或货位已变化，请重新打开审核详情', 409);
        }
        if (!Array.isArray(body.decisions) || body.decisions.length !== review.rows.length) {
            fail('请逐条选择对应商品，或选择忽略');
        }
        const indices = new Set();
        const ids = new Set();
        const slots = new Set();
        assignments = body.decisions.flatMap((decision) => {
            if (!decision || !Number.isInteger(decision.index) || indices.has(decision.index)) fail('审核行重复或无效');
            indices.add(decision.index);
            const row = review.rows[decision.index];
            if (!row) fail('扫描结果不存在');
            if (decision.productId === null) return [];
            if (!Number.isSafeInteger(decision.productId)) fail('请选择具体商品，不能只填写名称');
            const product = store.products.find((item) => item.id === decision.productId && item.storeId === review.storeId);
            if (!product) fail('只能选择本门店的现有商品');
            if (ids.has(product.id)) fail('同一商品不能分配到多个货位');
            ids.add(product.id);
            const slot = `${row.shelf}:${row.point}:${row.layer}`;
            if (slots.has(slot)) fail('同一货位不能分配多个商品');
            slots.add(slot);
            return [{ row, product }];
        });
        if (!assignments.length) fail('至少选择一个商品；无需更新时请驳回');
        // Clear displaced products first, then assign, so swaps remain valid.
        const assigned = new Set(assignments.map(({ product }) => product.id));
        for (const product of store.products) {
            if (product.storeId !== review.storeId || assigned.has(product.id)) continue;
            if (!slots.has(`${product.shelf_zone}:${product.shelf_number}:${product.shelf_layer}`)) continue;
            Object.assign(product, { location: '', chassis_marker: '', shelf_zone: '',
                shelf_number: null, shelf_layer: null, status: 0 });
        }
        for (const { row, product } of assignments) {
            Object.assign(product, { shelf_zone: row.shelf, shelf_number: row.point,
                shelf_layer: row.layer, chassis_marker: row.marker,
                location: `${row.shelf}货架第${row.point}点位第${row.layer}层` });
        }
        review.decisions = body.decisions.map(({ index, productId }) => ({ index, productId }));
    }
    Object.assign(review, { status: body.action === 'approve' ? 'approved' : 'rejected',
        reviewedBy: reviewer, reviewedAt: new Date().toISOString(), reason,
        appliedCount: assignments.length });
    return review;
}

module.exports = { submitReview, decideReview, productVersion };
