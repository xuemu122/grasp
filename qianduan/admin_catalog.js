let adminCatalogRows = [];
let adminCatalogEditing = null;
const adminCatalogFields = { name: '商品名称', spec: '规格 / 口味', packaging: '包装', category: '分类',
    manufacturer: '生产商', barcode: '商品条码', emoji: '图标', photo_front: '正面图片地址', photo_back: '反面图片地址' };

document.addEventListener('DOMContentLoaded', () => {
    const section = document.createElement('section');
    section.dataset.adminPage = 'catalog'; section.hidden = true;
    section.style.minWidth = '0';
    section.innerHTML = `<div class="card-title"><span>共享商品库</span><button class="btn btn-primary" type="button" onclick="editAdminCatalog(null)">新建库商品</button></div>
        <form class="toolbar" onsubmit="event.preventDefault();loadAdminCatalog()"><input class="search-box" id="adminCatalogSearch" placeholder="搜索名称、规格、条码" aria-label="搜索共享商品库"><button class="btn btn-primary" type="submit">搜索</button></form>
        <div class="table-wrapper"><table><thead><tr><th>图片</th><th>商品 / 规格</th><th>生产商</th><th>条码</th><th>门店数</th><th>状态</th><th>操作</th></tr></thead><tbody id="adminCatalogList"></tbody></table></div>
        <form id="adminCatalogForm" hidden style="margin-top:24px;border-top:1px solid var(--gray-200);padding-top:20px;max-width:700px" onsubmit="event.preventDefault();saveAdminCatalog()">
            <div class="card-title" id="adminCatalogTitle">公共商品资料</div>
            ${Object.entries(adminCatalogFields).map(([field, label]) => `<div class="form-group"><label for="catalogField-${field}">${label}</label><input class="form-input" id="catalogField-${field}"${['name', 'spec'].includes(field) ? ' required' : ''}></div>`).join('')}
            <div class="form-row"><div class="form-group"><label>上传正面图片<input type="file" accept="image/png,image/jpeg,image/webp" onchange="uploadCatalogImage('front',this)"></label></div>
            <div class="form-group"><label>上传反面图片<input type="file" accept="image/png,image/jpeg,image/webp" onchange="uploadCatalogImage('back',this)"></label></div></div>
            <div class="form-group"><label for="adminCatalogStatus">商品库状态</label><select class="form-select" id="adminCatalogStatus"><option value="1">可选用</option><option value="0">归档</option></select></div>
            <div class="robot-control-row"><button class="btn btn-primary" id="adminCatalogSave" type="submit">保存公共资料</button><button class="btn" type="button" onclick="document.getElementById('adminCatalogForm').hidden=true">取消</button></div>
        </form>`;
    document.querySelector('.admin-content').append(section);
    const style = document.createElement('style');
    style.textContent = '.admin-content[data-page="catalog"]{grid-template-columns:minmax(0,1fr)}[data-admin-page="catalog"] .card-title{flex-wrap:wrap;gap:12px}[data-admin-page="catalog"] input[type=file]{max-width:100%}';
    document.head.append(style);
});

async function catalogAdminFetch(url, options) {
    const res = await adminFetch(url, options);
    const json = await readAdminJson(res, '商品库接口返回无效数据');
    if (!res.ok || json.code !== 0) throw new Error(json.message || '商品库操作失败');
    return json;
}

async function loadAdminCatalog() {
    try {
        const json = await catalogAdminFetch(`/api/admin/catalog?search=${encodeURIComponent(document.getElementById('adminCatalogSearch').value)}`, { cache: 'no-store' });
        adminCatalogRows = json.data;
        const body = document.getElementById('adminCatalogList'); body.replaceChildren();
        for (const row of json.data) {
            const tr = document.createElement('tr');
            for (const value of ['', `${row.name} · ${row.spec}`, row.manufacturer, row.barcode || '-', row.storeCount, row.status ? '可选用' : '已归档']) {
                const td = document.createElement('td'); td.textContent = value; tr.append(td);
            }
            if (row.photo_front) { const img = document.createElement('img'); img.src = row.photo_front; img.alt = row.name; img.className = 'product-thumb'; tr.firstChild.append(img); }
            const td = document.createElement('td'), button = document.createElement('button');
            button.type = 'button'; button.className = 'btn btn-primary btn-sm'; button.textContent = '编辑'; button.onclick = () => editAdminCatalog(row);
            td.append(button); tr.append(td); body.append(tr);
        }
        if (!json.data.length) body.innerHTML = '<tr><td colspan="7" class="empty-state">未找到商品</td></tr>';
    } catch (error) { showToast(error.message, 'error'); }
}

function editAdminCatalog(row) {
    adminCatalogEditing = row;
    document.getElementById('adminCatalogForm').hidden = false;
    document.getElementById('adminCatalogTitle').textContent = row ? `编辑公共资料 · #${row.id}` : '新建共享商品';
    for (const key of Object.keys(adminCatalogFields)) document.getElementById(`catalogField-${key}`).value = row?.[key] || '';
    document.getElementById('adminCatalogStatus').value = String(row?.status ?? 1);
    document.getElementById('adminCatalogForm').scrollIntoView({ block: 'start' });
}

async function uploadCatalogImage(side, input) {
    const file = input.files?.[0]; if (!file) return;
    document.getElementById('adminCatalogSave').disabled = true;
    try {
        const json = await catalogAdminFetch('/api/admin/uploads', { method: 'POST', headers: { 'Content-Type': file.type }, body: file });
        document.getElementById(`catalogField-photo_${side}`).value = json.url;
    } catch (error) { showToast(error.message, 'error'); }
    finally { input.value = ''; document.getElementById('adminCatalogSave').disabled = false; }
}

async function saveAdminCatalog() {
    const payload = Object.fromEntries(Object.keys(adminCatalogFields).map((field) => [field, document.getElementById(`catalogField-${field}`).value.trim()]));
    payload.status = Number(document.getElementById('adminCatalogStatus').value);
    if (adminCatalogEditing) payload.version = adminCatalogEditing.version;
    if (adminCatalogEditing && !confirm('公共资料修改会同步展示到所有关联门店，确认保存？')) return;
    const button = document.getElementById('adminCatalogSave'); button.disabled = true;
    try {
        await catalogAdminFetch(`/api/admin/catalog${adminCatalogEditing ? `/${adminCatalogEditing.id}` : ''}`, {
            method: adminCatalogEditing ? 'PUT' : 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
        document.getElementById('adminCatalogForm').hidden = true;
        showToast('公共商品资料已保存', 'success');
        await loadAdminCatalog();
    } catch (error) { showToast(error.message, 'error'); }
    finally { button.disabled = false; }
}

function setAdminSharedReadOnly(locked) {
    for (const id of ['formName', 'formSpec', 'formPackaging', 'formCategory', 'formManufacturer', 'formEmoji', 'formPhotoFront', 'formPhotoBack']) {
        const field = document.getElementById(id); if (field) field.readOnly = locked;
    }
    for (const id of ['formPhotoFrontFile', 'formPhotoBackFile', 'formStore']) {
        const field = document.getElementById(id); if (field) field.disabled = locked;
    }
    document.querySelectorAll('#formCard .emoji-picker, #formCard .image-upload button').forEach((node) => {
        node.style.pointerEvents = locked ? 'none' : '';
        node.style.opacity = locked ? '0.5' : '';
    });
    let link = document.getElementById('sharedCatalogEditLink');
    if (!link) {
        link = document.createElement('button'); link.id = 'sharedCatalogEditLink'; link.type = 'button'; link.className = 'btn btn-primary btn-sm';
        link.textContent = '编辑公共资料'; link.onclick = () => switchAdminPage('catalog');
        document.getElementById('formCard').prepend(link);
    }
    link.hidden = !locked;
}
