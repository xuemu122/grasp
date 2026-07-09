const fs = require('fs');
const path = require('path');
const XLSX = require('xlsx');

const DATA_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
const EXCEL_FILE = path.join(__dirname, '工作簿 2_完整商品信息.xlsx');

function normalizeProduct(raw, fallbackId) {
    return {
        id: Number(raw.id || fallbackId),
        name: raw.name || '',
        spec: raw.spec || '',
        packaging: raw.packaging || '',
        category: raw.category || '',
        manufacturer: raw.manufacturer || '',
        photo_front: raw.photo_front || '',
        photo_back: raw.photo_back || '',
        location: raw.location || '',
        shelf_x: Number(raw.shelf_x || 0),
        shelf_y: Number(raw.shelf_y || 0),
        shelf_z: Number(raw.shelf_z || 50),
        emoji: raw.emoji || '📦',
        price: Number(raw.price || 0),
        stock: Number(raw.stock ?? 100),
        status: Number(raw.status ?? 1),
    };
}

function loadExistingProducts() {
    if (!fs.existsSync(DATA_FILE)) return [];
    try {
        const data = JSON.parse(fs.readFileSync(DATA_FILE, 'utf8'));
        return Array.isArray(data.products) ? data.products : [];
    } catch {
        return [];
    }
}

function parseExcelProducts() {
    const wb = XLSX.readFile(EXCEL_FILE);
    const ws = wb.Sheets[wb.SheetNames[0]];
    const data = XLSX.utils.sheet_to_json(ws, { header: 1, defval: '' });

    const categoryEmoji = {
        '碳酸饮料': '🥤', '矿泉水': '💧', '茶饮料': '🍵', '乳饮料': '🥛',
        '运动饮料': '⚡', '膨化零食': '🍿', '糕点面包': '🍞', '饼干': '🍪'
    };
    const categoryPrice = {
        '碳酸饮料': 4, '矿泉水': 2, '茶饮料': 4, '乳饮料': 6,
        '运动饮料': 5, '膨化零食': 6, '糕点面包': 8, '饼干': 6
    };

    const products = [];
    for (let i = 1; i < data.length; i++) {
        const row = data[i];
        if (!row[0]) continue;
        const name = String(row[0] || '').trim();
        const spec = String(row[1] || '').trim();
        const packaging = String(row[2] || '').trim();
        const category = String(row[3] || '').trim();
        const manufacturer = String(row[4] || '').trim();
        const photos = String(row[5] || '待补充').trim();
        const location = String(row[6] || '').trim();

        let photo_front = '';
        let photo_back = '';
        if (photos && photos !== '待补充') {
            const parts = photos.split(/[\/,，]/);
            photo_front = (parts[0] || '').trim();
            photo_back = (parts[1] || '').trim();
        }

        let shelf_x = 0;
        let shelf_y = 0;
        const locMatch = location.match(/A(\d+)/i) || location.match(/B(\d+)/i);
        if (locMatch) {
            shelf_x = location.includes('B') ? 200 : 100;
            shelf_y = parseInt(locMatch[1], 10) * 100;
        }

        products.push(normalizeProduct({
            id: products.length + 1,
            name,
            spec,
            packaging,
            category,
            manufacturer,
            photo_front,
            photo_back,
            location,
            shelf_x,
            shelf_y,
            shelf_z: 50,
            emoji: categoryEmoji[category] || '📦',
            price: categoryPrice[category] || 5,
            stock: 100,
            status: 1,
        }, products.length + 1));
    }
    return products;
}

function main() {
    const existing = loadExistingProducts();
    if (existing.length > 0) {
        console.log(`⚠️ 本地商品库已有 ${existing.length} 条数据，跳过导入: ${DATA_FILE}`);
        return;
    }

    if (!fs.existsSync(EXCEL_FILE)) {
        fs.mkdirSync(path.dirname(DATA_FILE), { recursive: true });
        fs.writeFileSync(DATA_FILE, JSON.stringify({ products: [] }, null, 2), 'utf8');
        console.log(`⚠️ 未找到 Excel，已创建空商品库: ${DATA_FILE}`);
        return;
    }

    const products = parseExcelProducts();
    fs.mkdirSync(path.dirname(DATA_FILE), { recursive: true });
    fs.writeFileSync(DATA_FILE, JSON.stringify({ products }, null, 2), 'utf8');
    console.log(`✅ 已导入 ${products.length} 条商品到本地商品库: ${DATA_FILE}`);
}

main();
