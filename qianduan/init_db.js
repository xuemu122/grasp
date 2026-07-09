const fs = require('fs');
const path = require('path');
const XLSX = require('xlsx');

const DATA_FILE = process.env.PRODUCTS_DB_FILE || path.join(__dirname, 'data', 'products.json');
const EXCEL_FILE = path.join(__dirname, '工作簿 2_完整商品信息.xlsx');

const PRODUCT_IMAGES = {
    '可乐|330ml/罐': 'images/drinks/A_single_Coca_Cola_red_can__33_2026-07-08T01-49-34.png',
    '可乐|500ml/瓶': 'images/drinks/A_single_Coca_Cola_500ml_plast_2026-07-08T01-50-05.png',
    '雪碧|330ml/罐': 'images/drinks/A_single_Sprite_green_can__330_2026-07-08T01-49-36.png',
    '雪碧|500ml/瓶': 'images/drinks/A_single_Sprite_500ml_plastic__2026-07-08T01-50-06.png',
    '芬达|330ml/罐': 'images/drinks/A_single_Fanta_orange_can__330_2026-07-08T01-49-38.png',
    '芬达|500ml/瓶': 'images/drinks/A_single_Fanta_orange_500ml_pl_2026-07-08T01-50-08.png',
    '农夫山泉|550ml/瓶': 'images/drinks/A_single_Nongfu_Spring_mineral_2026-07-08T01-50-33.png',
    '冰红茶|500ml/瓶': 'images/drinks/A_single_Ice_Tea__Bing_Hong_Ch_2026-07-08T01-50-34.png',
    '旺仔牛奶|245ml/罐': 'images/drinks/A_single_Wangzai_milk_small_re_2026-07-08T01-50-36.png',
    '康师傅绿茶|500ml/瓶': 'images/drinks/A_single_Kangshifu_green_tea_5_2026-07-08T01-51-01.png',
    '尖叫运动饮料|550ml/瓶': 'images/drinks/A_single_Jianjiao_sports_drink_2026-07-08T01-51-03.png',
    'AD钙奶|220ml/瓶': 'images/drinks/A_single_AD_calcium_milk_small_2026-07-08T01-51-04.png',
    '奶|250ml/盒': 'images/drinks/A_single_Wangzai_milk_small_re_2026-07-08T01-50-36.png',
    '水溶C|500ml/瓶': 'images/drinks/shuirongc.svg',
    '咖啡|250ml/杯': 'images/drinks/coffee.svg',
    '上好佳鲜虾条|40g/袋': 'images/snacks/shanghaojia.png',
    '奇多粟米棒|35g/袋': 'images/snacks/qiduoli.jpg',
    '可比克薯片|60g/袋': 'images/snacks/kebike.jpg',
    '米多奇雪饼|50g/袋': 'images/snacks/miduoqi.png',
    '三只松鼠山药脆片|35g/袋': 'images/snacks/sanzhisongshu.png',
    '乐事薯片|70g / 袋': 'images/snacks/O1CN01250A672DnaHCWdDOB_!!1791258654.png',
    '呀土豆|40g / 袋': 'images/snacks/yatudou.png',
    '盼盼小面包|400g / 袋': 'images/snacks/panpan.png',
};

const DEFAULT_EXTRA_PRODUCTS = [
    {
        name: '奶',
        spec: '250ml/盒',
        packaging: '盒装',
        category: '乳饮料',
        manufacturer: '',
        location: '饮料区A4货架',
        shelf_x: 100,
        shelf_y: 400,
        shelf_z: 50,
        emoji: '🥛',
        price: 6,
        stock: 100,
        status: 1,
    },
    {
        name: '水溶C',
        spec: '500ml/瓶',
        packaging: '瓶装',
        category: '运动饮料',
        manufacturer: '',
        location: '饮料区A5货架',
        shelf_x: 100,
        shelf_y: 500,
        shelf_z: 50,
        emoji: '⚡',
        price: 5,
        stock: 100,
        status: 1,
    },
    {
        name: '咖啡',
        spec: '250ml/杯',
        packaging: '杯装',
        category: '冲泡饮料',
        manufacturer: '',
        location: '饮料区A6货架',
        shelf_x: 100,
        shelf_y: 600,
        shelf_z: 50,
        emoji: '☕',
        price: 8,
        stock: 100,
        status: 1,
    },
];

function productImage(product) {
    return PRODUCT_IMAGES[`${product.name || ''}|${product.spec || ''}`] || '';
}

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

function applyCatalogDefaults(products) {
    let changed = false;
    const next = products.map((product, idx) => {
        const normalized = normalizeProduct(product, idx + 1);
        if (!normalized.photo_front) {
            const image = productImage(normalized);
            if (image) {
                changed = true;
                return { ...normalized, photo_front: image };
            }
        }
        return normalized;
    });

    for (const extra of DEFAULT_EXTRA_PRODUCTS) {
        const exists = next.some((item) => item.name === extra.name && item.category === extra.category);
        if (!exists) {
            const product = normalizeProduct({
                ...extra,
                id: next.reduce((max, item) => Math.max(max, Number(item.id || 0)), 0) + 1,
                photo_front: productImage(extra),
            }, next.length + 1);
            next.push(product);
            changed = true;
        }
    }

    return { products: next, changed };
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
        '运动饮料': '⚡', '冲泡饮料': '☕', '膨化零食': '🍿', '糕点面包': '🍞', '饼干': '🍪'
    };
    const categoryPrice = {
        '碳酸饮料': 4, '矿泉水': 2, '茶饮料': 4, '乳饮料': 6,
        '运动饮料': 5, '冲泡饮料': 8, '膨化零食': 6, '糕点面包': 8, '饼干': 6
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
            photo_front: photo_front || productImage({ name, spec }),
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
    return applyCatalogDefaults(products).products;
}

function main() {
    const existing = loadExistingProducts();
    if (existing.length > 0) {
        const result = applyCatalogDefaults(existing);
        if (result.changed) {
            fs.mkdirSync(path.dirname(DATA_FILE), { recursive: true });
            fs.writeFileSync(DATA_FILE, JSON.stringify({ products: result.products }, null, 2), 'utf8');
            console.log(`✅ 已补全本地商品库图片/默认商品: ${DATA_FILE}`);
        } else {
            console.log(`⚠️ 本地商品库已有 ${existing.length} 条数据，跳过导入: ${DATA_FILE}`);
        }
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
