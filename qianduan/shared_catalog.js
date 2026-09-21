const crypto = require('node:crypto');
const FIELDS = ['name', 'spec', 'packaging', 'category', 'manufacturer', 'photo_front', 'photo_back', 'emoji', 'barcode'];
const LOCAL_FIELDS = ['price', 'stock', 'status', 'location', 'chassis_marker', 'shelf_number', 'shelf_zone', 'shelf_layer', 'shelf_x', 'shelf_y', 'shelf_z'];
const fail = (message, statusCode = 400, extra = {}) => Object.assign(new Error(message), { statusCode, ...extra });
const normalize = (value) => String(value || '').normalize('NFKC').trim().replace(/\s+/g, ' ').toLowerCase();

function mysqlOptions(env = process.env) {
    if (!env.MYSQL_HOST || !env.MYSQL_USER || !env.MYSQL_PASSWORD || !env.MYSQL_DATABASE) {
        throw new Error('共享商品库缺少 MYSQL_HOST/USER/PASSWORD/DATABASE 配置');
    }
    return { host: env.MYSQL_HOST, port: Number(env.MYSQL_PORT || 3306),
        user: env.MYSQL_USER, password: env.MYSQL_PASSWORD, database: env.MYSQL_DATABASE,
        charset: 'utf8mb4', connectionLimit: 4, connectTimeout: 5000,
        supportBigNumbers: true, bigNumberStrings: true };
}

function normalizedBarcode(input) {
    const value = String(input || '').normalize('NFKC').trim();
    if (!value) return null;
    if (!/^(?:\d{8}|\d{12}|\d{13}|\d{14})$/.test(value)) throw fail('条码须为有效的 8、12、13 或 14 位商品条码');
    const digits = value.slice(0, -1).split('').reverse();
    const sum = digits.reduce((total, digit, index) => total + Number(digit) * (index % 2 ? 1 : 3), 0);
    if ((10 - sum % 10) % 10 !== Number(value.at(-1))) throw fail('商品条码校验位不正确');
    return value.padStart(14, '0');
}

function definition(input) {
    const result = {};
    for (const field of FIELDS) {
        if (field === 'barcode') continue;
        const text = String(input[field] || '').trim();
        const max = field.startsWith('photo_') ? 1024 : field === 'emoji' ? 32 : 255;
        if (text.length > max) throw fail(`${field} 长度不能超过 ${max}`);
        result[field] = text;
    }
    if (!result.name || !result.spec) throw fail('请填写商品名称和完整规格（含容量、口味或包装数量）');
    if (/[\r\n]/.test(result.name)) throw fail('商品名称不能包含换行');
    if (/^(?:__.*__|\*\*.*\*\*)$/.test(result.name)) throw fail('商品名称不能使用设备控制词');
    for (const field of ['photo_front', 'photo_back']) {
        if (result[field] && !/^(?:https?:\/\/|\/?(?:api\/uploads|images)\/)/i.test(result[field])) {
            throw fail('图片必须使用已上传的图片地址或 HTTP(S) 地址');
        }
    }
    result.barcode = normalizedBarcode(input.barcode);
    result.identity_hash = crypto.createHash('sha256').update(JSON.stringify(
        [result.name, result.spec, result.manufacturer, result.packaging].map(normalize))).digest('hex');
    return result;
}

async function initializeSchema(db) {
    await db.query(`CREATE TABLE IF NOT EXISTS catalog_products (
        id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
        identity_hash CHAR(64) CHARACTER SET ascii COLLATE ascii_bin NOT NULL UNIQUE,
        barcode VARCHAR(14) CHARACTER SET ascii COLLATE ascii_bin NULL UNIQUE,
        name VARCHAR(255) NOT NULL, spec VARCHAR(255) NOT NULL, packaging VARCHAR(255) NOT NULL,
        category VARCHAR(255) NOT NULL, manufacturer VARCHAR(255) NOT NULL,
        photo_front VARCHAR(1024) NOT NULL, photo_back VARCHAR(1024) NOT NULL, emoji VARCHAR(32) NOT NULL,
        status TINYINT NOT NULL DEFAULT 1, version INT NOT NULL DEFAULT 1,
        created_by VARCHAR(128) NOT NULL, created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
    ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4`);
    await db.query(`CREATE TABLE IF NOT EXISTS catalog_product_changes (
        id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
        catalog_id BIGINT UNSIGNED NOT NULL, actor VARCHAR(128) NOT NULL,
        before_json JSON NULL, after_json JSON NOT NULL,
        created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
        KEY idx_catalog_change (catalog_id),
        FOREIGN KEY (catalog_id) REFERENCES catalog_products(id)
    ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4`);
}

class SharedCatalog {
    constructor(pool) {
        this.pool = pool;
        this.rows = new Map();
        this.generation = 0;
        this.ready = false;
    }
    async refresh() {
        const generation = this.generation;
        const [rows] = await this.pool.query('SELECT * FROM catalog_products');
        if (this.generation === generation) this.rows = new Map(rows.map((row) => [String(row.id), row]));
        this.ready = true;
    }
    get(id) { return this.rows.get(String(id)); }
    overlay(product) {
        if (!product.catalogId) return product;
        const row = this.get(product.catalogId);
        if (!row) throw fail(`商品 #${product.id} 的共享资料不存在`, 503);
        const shared = Object.fromEntries(FIELDS.map((field) => [field, row[field] || '']));
        return { ...product, ...shared, catalogId: String(row.id), catalogStatus: row.status, catalogVersion: row.version };
    }
    list({ search = '', includeArchived = false } = {}) {
        const words = normalize(search).split(' ').filter(Boolean);
        return [...this.rows.values()].filter((row) => (includeArchived || row.status === 1)
            && words.every((word) => normalize([row.name, row.spec, row.manufacturer, row.barcode, row.category].join(' ')).includes(word)))
            .sort((a, b) => Number(b.id) - Number(a.id));
    }
    async create(input, actor) {
        const data = definition(input);
        const requestedStatus = input.status === undefined ? 1 : Number(input.status);
        if (![0, 1].includes(requestedStatus)) throw fail('商品库状态无效');
        const connection = await this.pool.getConnection();
        try {
            await connection.beginTransaction();
            const fields = [...FIELDS, 'identity_hash', 'created_by'];
            const [insert] = await connection.execute(`INSERT INTO catalog_products (${fields.join(',')}) VALUES (${fields.map(() => '?').join(',')})`,
                fields.map((field) => field === 'created_by' ? actor : data[field]));
            if (requestedStatus === 0) await connection.execute('UPDATE catalog_products SET status=0 WHERE id=?', [insert.insertId]);
            const [rows] = await connection.query('SELECT * FROM catalog_products WHERE id=?', [insert.insertId]);
            const row = rows[0];
            await connection.execute('INSERT INTO catalog_product_changes(catalog_id,actor,after_json) VALUES (?,?,?)',
                [row.id, actor, JSON.stringify(row)]);
            await connection.commit();
            this.generation++;
            this.rows.set(String(row.id), row);
            return row;
        } catch (error) {
            await connection.rollback();
            if (error.code === 'ER_DUP_ENTRY') {
                const [rows] = await connection.query('SELECT * FROM catalog_products WHERE identity_hash=? OR barcode=?', [data.identity_hash, data.barcode]);
                throw fail('商品库已有相同条码或相同资料，请确认后从库中选择', 409, { existing: rows[0] });
            }
            throw error;
        } finally { connection.release(); }
    }
    async update(id, input, actor) {
        const connection = await this.pool.getConnection();
        try {
            await connection.beginTransaction();
            const [rows] = await connection.query('SELECT * FROM catalog_products WHERE id=? FOR UPDATE', [id]);
            const before = rows[0];
            if (!before) throw fail('库商品不存在', 404);
            if (Number(input.version) !== before.version) throw fail('公共资料已被修改，请重新打开商品', 409);
            const data = definition({ ...before, ...input });
            if (input.status !== undefined && ![0, 1].includes(input.status)) throw fail('商品库状态无效');
            const fields = [...FIELDS, 'identity_hash', 'status'];
            await connection.execute(`UPDATE catalog_products SET ${fields.map((field) => `${field}=?`).join(',')},version=version+1 WHERE id=?`,
                [...fields.map((field) => field === 'status' ? (input.status ?? before.status) : data[field]), id]);
            const [updated] = await connection.query('SELECT * FROM catalog_products WHERE id=?', [id]);
            await connection.execute('INSERT INTO catalog_product_changes(catalog_id,actor,before_json,after_json) VALUES (?,?,?,?)',
                [id, actor, JSON.stringify(before), JSON.stringify(updated[0])]);
            await connection.commit();
            this.generation++;
            this.rows.set(String(id), updated[0]);
            return updated[0];
        } catch (error) {
            await connection.rollback();
            if (error.code === 'ER_DUP_ENTRY') throw fail('修改后与其他库商品冲突，请核对条码、名称和规格', 409);
            throw error;
        } finally { connection.release(); }
    }
}

module.exports = { SharedCatalog, mysqlOptions, initializeSchema, definition, normalizedBarcode, FIELDS, LOCAL_FIELDS, fail };
