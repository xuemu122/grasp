#!/usr/bin/env node
/**
 * 把商品和订单的门店归属统一迁移到指定门店（默认 STORE-001 北校区）。
 *
 * 用法：
 *   node tools/migrate-store.js                     # 迁移到 STORE-001 北校区
 *   node tools/migrate-store.js --store STORE-002 --name 门店B
 *   node tools/migrate-store.js --dry-run           # 只统计不写入
 *
 * 数据文件优先取环境变量 PRODUCTS_DB_FILE / ORDERS_DB_FILE，
 * 否则用 ./data/products.json、./data/orders.json。
 * 写入前会自动备份为 <文件>.bak.<时间戳>。
 */
const fs = require('fs');
const path = require('path');

function parseArgs(argv) {
    const args = { store: 'STORE-001', name: '北校区', dry: false };
    for (let i = 2; i < argv.length; i += 1) {
        const key = argv[i];
        if (key === '--store') args.store = argv[++i];
        else if (key === '--name') args.name = argv[++i];
        else if (key === '--dry-run') args.dry = true;
    }
    return args;
}

function resolveFile(envKey, fallback) {
    const fromEnv = process.env[envKey];
    if (fromEnv && fs.existsSync(fromEnv)) return fromEnv;
    return fallback;
}

function listOf(raw, key) {
    if (Array.isArray(raw)) return raw;
    if (raw && Array.isArray(raw[key])) return raw[key];
    return null;
}

function main() {
    const args = parseArgs(process.argv);
    const base = path.join(__dirname, '..');
    const targets = [
        { kind: '商品', file: resolveFile('PRODUCTS_DB_FILE', path.join(base, 'data', 'products.json')), key: 'products' },
        { kind: '订单', file: resolveFile('ORDERS_DB_FILE', path.join(base, 'data', 'orders.json')), key: 'orders' }
    ];
    for (const target of targets) {
        if (!fs.existsSync(target.file)) {
            console.log(`[跳过] ${target.kind}文件不存在: ${target.file}`);
            continue;
        }
        const raw = JSON.parse(fs.readFileSync(target.file, 'utf8'));
        const list = listOf(raw, target.key);
        if (!list) {
            console.log(`[跳过] ${target.file} 结构无法识别`);
            continue;
        }
        let changed = 0;
        let already = 0;
        for (const item of list) {
            if (!item || typeof item !== 'object') continue;
            if (item.storeId !== args.store) {
                item.storeId = args.store;
                item.storeName = args.name;
                item.updatedAt = new Date().toISOString();
                changed += 1;
            } else {
                already += 1;
            }
        }
        console.log(`[${target.kind}] ${target.file}: 迁移 ${changed} 条，已属 ${args.store} ${already} 条`);
        if (args.dry) {
            console.log('  (dry-run，未写入)');
            continue;
        }
        if (changed > 0) {
            const stamp = new Date().toISOString().replace(/[:.]/g, '-');
            const backup = `${target.file}.bak.${stamp}`;
            fs.copyFileSync(target.file, backup);
            fs.writeFileSync(target.file, JSON.stringify(raw, null, 2));
            console.log(`  已写入，备份: ${backup}`);
        }
    }
}

main();
