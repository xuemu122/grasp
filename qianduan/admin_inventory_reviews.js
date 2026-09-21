let scanReviewRows = [];
let scanReviewStores = [];
let activeScanReview = null;
let scanReviewRequest = 0;
const scanReviewLabels = { pending: '待审核', approved: '已通过', rejected: '已驳回' };

async function scanReviewFetch(path, options) {
    const res = await adminFetch(path, options);
    const json = await readAdminJson(res, '扫描审核接口返回了无效数据');
    if (!res.ok || json.code !== 0) throw new Error(json.message || '扫描审核请求失败');
    return json;
}

async function loadScanReviews() {
    try {
        const json = await scanReviewFetch('/api/admin/inventory-reviews', { cache: 'no-store' });
        scanReviewRows = json.data;
        scanReviewStores = json.stores;
        renderScanReviewList();
    } catch (error) {
        document.getElementById('scanReviewList').innerHTML = `<tr><td colspan="7">${esc(error.message)}</td></tr>`;
        showToast(error.message, 'error');
    }
}

function scanReviewStoreName(id) {
    return scanReviewStores.find((store) => store.id === id)?.name || id;
}

function renderScanReviewList() {
    const filter = document.getElementById('scanReviewStatus').value;
    const rows = scanReviewRows.filter((review) => !filter || review.status === filter);
    document.getElementById('scanReviewCount').textContent = `${scanReviewRows.filter((row) => row.status === 'pending').length} 待审核`;
    document.getElementById('scanReviewList').innerHTML = rows.map((review) => `<tr>
        <td>${esc(scanReviewStoreName(review.storeId))}</td><td>${esc(review.taskId)}</td>
        <td>${esc(adminOrderTime(review.submittedAt))}</td><td>${esc(review.submittedBy)}</td>
        <td>${review.rows.length}</td><td>${scanReviewLabels[review.status] || ''}</td>
        <td><button class="btn btn-primary btn-sm" type="button" onclick="openScanReview('${review.id}')">${review.status === 'pending' ? '审核' : '查看'}</button></td>
    </tr>`).join('') || '<tr><td colspan="7" class="empty-state">暂无审核记录</td></tr>';
}

async function openScanReview(id) {
    const request = ++scanReviewRequest;
    try {
        const json = await scanReviewFetch(`/api/admin/inventory-reviews/${id}`, { cache: 'no-store' });
        if (request !== scanReviewRequest) return;
        activeScanReview = json;
        const review = json.data;
        const pending = review.status === 'pending';
        const detail = document.getElementById('scanReviewDetail');
        detail.hidden = false;
        detail.innerHTML = `<div class="card-title"><span>${esc(scanReviewStoreName(review.storeId))} · ${esc(review.taskId)}</span></div>
            <p>${scanReviewLabels[review.status]}${review.reviewedBy ? ` · ${esc(review.reviewedBy)} · ${esc(adminOrderTime(review.reviewedAt))}` : ''}</p>
            <div class="table-wrapper"><table style="min-width:760px"><thead><tr><th>扫描货位</th><th>识别结果</th><th>识别数量</th><th>本店商品</th><th>商品图片 / 当前货位</th></tr></thead><tbody>
            ${review.rows.map((row) => {
                const decision = review.decisions?.find((item) => item.index === row.index);
                const options = json.products.map((product) => `<option value="${product.id}"${decision?.productId === product.id ? ' selected' : ''}>#${product.id} ${esc(product.name)} ${esc(product.spec)}${Number(product.status) === 1 ? '' : '（已下架）'}</option>`).join('');
                return `<tr><td>${esc(row.shelf)} / ${row.point} / 第${row.layer}层<br>${esc(row.marker)}</td>
                    <td>${esc(row.name) || '未识别'}</td><td>${row.count} / ${row.total}</td>
                    <td><select class="form-select" style="width:260px;max-width:100%" aria-label="第${row.index + 1}条对应商品" data-review-index="${row.index}" onchange="previewReviewProduct(this)"${pending ? '' : ' disabled'}>
                    <option value="">请选择商品</option><option value="ignore"${decision?.productId === null ? ' selected' : ''}>忽略此条</option>${options}</select></td>
                    <td data-review-preview="${row.index}" style="min-width:150px"></td></tr>`;
            }).join('')}</tbody></table></div>
            ${pending ? `<div class="form-group" style="margin-top:16px"><label for="scanReviewReason">审核备注 / 驳回原因</label><textarea id="scanReviewReason" class="form-input" maxlength="1000" rows="2"></textarea></div>
                <div class="robot-control-row"><button type="button" class="btn btn-primary" onclick="decideScanReview('approve')">通过并更新货位</button>
                <button type="button" class="btn btn-danger" onclick="decideScanReview('reject')">驳回</button>
                <button type="button" class="btn btn-primary" onclick="switchAdminPage('products')">商品管理</button></div>`
                : `<p>已更新 ${Number(review.appliedCount) || 0} 条货位</p><p>${esc(review.reason || '')}</p>`}`;
        detail.querySelectorAll('[data-review-index]').forEach(previewReviewProduct);
    } catch (error) { showToast(error.message, 'error'); }
}

function previewReviewProduct(select) {
    const product = activeScanReview.products.find((item) => String(item.id) === select.value);
    const cell = document.querySelector(`[data-review-preview="${select.dataset.reviewIndex}"]`);
    cell.replaceChildren();
    if (!product) { cell.textContent = select.value === 'ignore' ? '不修改' : '尚未选择'; return; }
    if (product.photo_front) {
        const img = document.createElement('img');
        img.src = product.photo_front;
        img.alt = product.name;
        img.width = 48;
        img.height = 48;
        img.style.objectFit = 'contain';
        img.onerror = () => { img.hidden = true; };
        cell.append(img);
    }
    const text = document.createElement('div');
    text.textContent = product.location || '未分配货位';
    cell.append(text);
}

async function decideScanReview(action) {
    const review = activeScanReview?.data;
    if (!review || review.status !== 'pending') return;
    const reason = document.getElementById('scanReviewReason').value.trim();
    const selects = [...document.querySelectorAll('[data-review-index]')];
    if (action === 'approve' && selects.some((select) => !select.value)) return showToast('请逐条选择商品或忽略', 'warning');
    if (action === 'reject' && !reason) return showToast('请填写驳回原因', 'warning');
    if (action === 'approve' && !confirm('确认更新所选商品的货位？被替换的货位商品将清空货位并下架。')) return;
    const buttons = document.querySelectorAll('#scanReviewDetail button');
    buttons.forEach((button) => { button.disabled = true; });
    try {
        const json = await scanReviewFetch(`/api/admin/inventory-reviews/${review.id}`, {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action, reason, productVersion: activeScanReview.productVersion,
                decisions: selects.map((select) => ({ index: Number(select.dataset.reviewIndex),
                    productId: select.value === 'ignore' ? null : Number(select.value) })) })
        });
        showToast(json.message, 'success');
        await loadScanReviews();
        await openScanReview(review.id);
    } catch (error) { showToast(error.message, 'error'); }
    finally { buttons.forEach((button) => { button.disabled = false; }); }
}
