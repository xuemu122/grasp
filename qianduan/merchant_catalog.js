let catalogMode = 'select';
let selectedCatalogId = null;
let merchantCatalogRequest = 0;
const catalogPublicFields = ['name', 'spec', 'packaging', 'category', 'manufacturer', 'photo_front', 'photo_back', 'emoji', 'barcode'];
const catalogFieldElements = { name: 'productFormName', spec: 'productFormSpec', packaging: 'productFormPackaging',
    category: 'productFormCategory', manufacturer: 'productFormManufacturer', emoji: 'productFormEmoji',
    barcode: 'productFormBarcode', photo_front: 'productPhotoFront', photo_back: 'productPhotoBack' };

function updateCatalogFormMode() {
    const editing = Boolean(editingProductId);
    document.getElementById('catalogModes').hidden = editing;
    document.getElementById('merchantCatalogPicker').hidden = editing || catalogMode !== 'select';
    for (const id of Object.values(catalogFieldElements)) document.getElementById(id).readOnly = editing || catalogMode !== 'new';
    document.querySelectorAll('.mc-form-card input[type=file]').forEach((input) => { input.disabled = editing || catalogMode !== 'new'; });
    document.querySelectorAll('[name=catalogMode]').forEach((input) => { input.checked = input.value === catalogMode; });
    document.getElementById('productFormSpec').required = !editing && catalogMode === 'new';
    if (!editing) {
        document.getElementById('productFormTitle').textContent = catalogMode === 'select' ? '从商品库添加' : '新建共享商品';
        document.getElementById('productFormSubmit').textContent = catalogMode === 'select' ? '添加到本店' : '创建并添加到本店';
    }
}

function setCatalogMode(mode) {
    catalogMode = mode;
    selectedCatalogId = null;
    for (const id of Object.values(catalogFieldElements)) document.getElementById(id).value = '';
    document.getElementById('productFormEmoji').value = '📦';
    document.getElementById('productPhotoFrontPreview').textContent = '未选择';
    document.getElementById('productPhotoBackPreview').textContent = '未选择';
    updateCatalogFormMode();
    if (mode === 'select') loadMerchantCatalog();
}

function selectCatalogProduct(product) {
    catalogMode = 'select';
    selectedCatalogId = String(product.id);
    for (const [field, id] of Object.entries(catalogFieldElements)) document.getElementById(id).value = product[field] || '';
    for (const side of ['Front', 'Back']) {
        const preview = document.getElementById(`productPhoto${side}Preview`);
        const url = product[`photo_${side.toLowerCase()}`];
        preview.replaceChildren();
        if (url) { const img = document.createElement('img'); img.src = url; img.alt = product.name; preview.append(img); }
        else preview.textContent = '未选择';
    }
    updateCatalogFormMode();
}

async function loadMerchantCatalog() {
    const request = ++merchantCatalogRequest;
    const results = document.getElementById('catalogSearchResults');
    const storeId = document.getElementById('productFormStore')?.value || '';
    const params = new URLSearchParams({ search: document.getElementById('catalogSearch').value.trim(), storeId });
    try {
        const res = await merchantFetch(`/api/merchant/catalog?${params}`, { cache: 'no-store' });
        const json = await res.json();
        if (request !== merchantCatalogRequest) return;
        if (!res.ok || json.code !== 0) throw new Error(json.message || '商品库读取失败');
        results.replaceChildren();
        for (const product of json.data) {
            const row = document.createElement('div');
            row.style.cssText = 'display:flex;align-items:center;gap:10px;padding:10px 0;border-bottom:1px solid var(--gray-200);';
            const img = document.createElement('img');
            img.width = 44; img.height = 44; img.style.objectFit = 'contain'; img.alt = product.name;
            if (product.photo_front) img.src = product.photo_front;
            else img.style.visibility = 'hidden';
            img.onerror = () => { img.style.visibility = 'hidden'; };
            const label = document.createElement('span');
            label.style.cssText = 'flex:1;min-width:0;overflow-wrap:anywhere;font-size:0.85rem';
            label.textContent = `${product.name} · ${product.spec}`;
            const button = document.createElement('button'); button.type = 'button';
            button.className = 'btn btn-primary btn-sm'; button.style.flexShrink = '0';
            button.textContent = product.added ? '已添加' : '选择'; button.disabled = product.added;
            button.onclick = () => selectCatalogProduct(product);
            row.append(img, label, button); results.append(row);
        }
        if (!json.data.length) results.textContent = '未找到匹配商品';
    } catch (error) { if (request === merchantCatalogRequest) results.textContent = error.message; }
}

document.addEventListener('DOMContentLoaded', updateCatalogFormMode);
