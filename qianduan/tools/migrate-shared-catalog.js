#!/usr/bin/env node
const fs = require('node:fs');
const path = require('node:path');
const { SharedCatalog, mysqlOptions, initializeSchema, definition, FIELDS } = require('../shared_catalog');

async function migrate(file, pool, apply = false) {
    const original = fs.readFileSync(file);
    const store = JSON.parse(original);
    if (!Array.isArray(store.products)) throw new Error('商品文件格式无效');
    const ids = new Set();
    const prepared = store.products.map((product) => {
        if (!Number.isSafeInteger(product.id) || product.id <= 0 || ids.has(product.id) || !product.storeId) {
            throw new Error(`商品 ID 或门店无效: ${product.id}`);
        }
        ids.add(product.id);
        return { product, data: definition(product) };
    });
    if (!apply) return { mode: 'dry-run', products: prepared.length,
        definitions: new Set(prepared.map(({ data }) => data.identity_hash)).size };
    await initializeSchema(pool);
    const catalog = new SharedCatalog(pool);
    await catalog.refresh();
    let created = 0;
    const seen = new Set();
    for (const { product, data } of prepared) {
        let row = product.catalogId ? catalog.get(product.catalogId)
            : [...catalog.rows.values()].find((item) => item.identity_hash === data.identity_hash || (data.barcode && item.barcode === data.barcode));
        if (!row) { row = await catalog.create({ ...product, status: 1 }, 'migration'); created++; }
        else if (!product.catalogId && FIELDS.some((field) => (row[field] || '') !== (data[field] || ''))) {
            throw new Error(`商品 #${product.id} 与库商品 #${row.id} 资料冲突，请人工核对后重试`);
        }
        const key = `${product.storeId}:${row.id}`;
        if (seen.has(key)) throw new Error(`同门店存在重复商品关联: ${key}，未写入商品文件`);
        seen.add(key);
        product.catalogId = String(row.id);
    }
    if (!original.equals(fs.readFileSync(file))) throw new Error('迁移期间商品文件变化，请暂停服务后重试');
    const stat = fs.statSync(file);
    const backup = `${file}.before-shared-${Date.now()}`;
    fs.copyFileSync(file, backup);
    fs.chmodSync(backup, stat.mode & 0o777);
    fs.chownSync(backup, stat.uid, stat.gid);
    const temporary = `${file}.catalog-${process.pid}.tmp`;
    fs.writeFileSync(temporary, JSON.stringify(store, null, 2), { mode: stat.mode & 0o777 });
    fs.chownSync(temporary, stat.uid, stat.gid);
    fs.renameSync(temporary, file);
    return { mode: 'applied', products: prepared.length, created, catalogCount: catalog.rows.size, backup };
}

if (require.main === module) {
    const pool = require('mysql2/promise').createPool(mysqlOptions());
    migrate(process.env.PRODUCTS_DB_FILE || path.join(__dirname, '../data/products.json'), pool, process.argv.includes('--apply'))
        .then((result) => console.log(JSON.stringify(result)))
        .catch((error) => { console.error(error.message); process.exitCode = 1; })
        .finally(() => pool.end());
}
module.exports = { migrate };
