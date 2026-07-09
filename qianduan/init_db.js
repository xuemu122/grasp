const mysql = require('mysql2/promise');
const XLSX = require('xlsx');

const DB_CONFIG = {
    host: 'localhost',
    user: 'root',
    password: 'nvidia',
    database: 'robot_order',
    multipleStatements: true
};

async function main() {
    let conn;
    try {
        // 先不指定 database，创建数据库
        let conn2 = await mysql.createConnection({ host: 'localhost', user: 'root', password: 'nvidia', multipleStatements: true });
        await conn2.query('CREATE DATABASE IF NOT EXISTS robot_order CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci');
        await conn2.end();

        conn = await mysql.createConnection(DB_CONFIG);
        console.log('✅ MySQL 连接成功');


        // 建表
        await conn.execute(`
            CREATE TABLE IF NOT EXISTS products (
                id INT AUTO_INCREMENT PRIMARY KEY,
                name VARCHAR(100) NOT NULL COMMENT '商品名称',
                spec VARCHAR(100) DEFAULT '' COMMENT '规格(重量/容量)',
                packaging VARCHAR(50) DEFAULT '' COMMENT '包装形式',
                category VARCHAR(50) DEFAULT '' COMMENT '品类',
                manufacturer VARCHAR(100) DEFAULT '' COMMENT '生产商',
                photo_front VARCHAR(500) DEFAULT '' COMMENT '包装正面照片',
                photo_back VARCHAR(500) DEFAULT '' COMMENT '包装反面照片',
                location VARCHAR(100) DEFAULT '' COMMENT '货架位置',
                shelf_x INT DEFAULT 0 COMMENT '货架X坐标',
                shelf_y INT DEFAULT 0 COMMENT '货架Y坐标',
                shelf_z INT DEFAULT 50 COMMENT '货架Z坐标',
                emoji VARCHAR(10) DEFAULT '📦' COMMENT '图标',
                price DECIMAL(10,2) DEFAULT 0 COMMENT '价格',
                stock INT DEFAULT 100 COMMENT '库存',
                status TINYINT DEFAULT 1 COMMENT '状态 1:在售 0:下架',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
            ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci COMMENT='商品信息表'
        `);
        console.log('✅ 数据表 products 已创建');

        // 检查是否已有数据
        const [rows] = await conn.execute('SELECT COUNT(*) as cnt FROM products');
        if (rows[0].cnt > 0) {
            console.log(`⚠️ 表中已有 ${rows[0].cnt} 条数据，跳过导入`);
            await conn.end();
            return;
        }

        // 读取 Excel
        const wb = XLSX.readFile('工作簿 2_完整商品信息.xlsx');
        const ws = wb.Sheets[wb.SheetNames[0]];
        const data = XLSX.utils.sheet_to_json(ws, { header: 1, defval: '' });

        // 品类-emoji 映射
        const categoryEmoji = {
            '碳酸饮料': '🥤', '矿泉水': '💧', '茶饮料': '🍵', '乳饮料': '🥛',
            '运动饮料': '⚡', '膨化零食': '🍿', '糕点面包': '🍞', '饼干': '🍪'
        };

        // 品类-默认价格
        const categoryPrice = {
            '碳酸饮料': 4, '矿泉水': 2, '茶饮料': 4, '乳饮料': 6,
            '运动饮料': 5, '膨化零食': 6, '糕点面包': 8, '饼干': 6
        };

        let imported = 0;
        for (let i = 1; i < data.length; i++) {
            const row = data[i];
            if (!row[0]) continue;
            const name = row[0].trim();
            const spec = (row[1] || '').trim();
            const packaging = (row[2] || '').trim();
            const category = (row[3] || '').trim();
            const manufacturer = (row[4] || '').trim();
            const photos = (row[5] || '待补充').trim();
            const location = (row[6] || '').trim();

            // 解析照片（正/反）
            let photo_front = '', photo_back = '';
            if (photos && photos !== '待补充') {
                const parts = photos.split(/[\/,，]/);
                photo_front = (parts[0] || '').trim();
                photo_back = (parts[1] || '').trim();
            }

            const emoji = categoryEmoji[category] || '📦';
            const price = categoryPrice[category] || 5;

            // 解析位置坐标
            let shelf_x = 0, shelf_y = 0;
            const locMatch = location.match(/A(\d+)/i) || location.match(/B(\d+)/i);
            if (locMatch) {
                shelf_x = (location.includes('B') ? 200 : 100);
                shelf_y = parseInt(locMatch[1]) * 100;
            }

            await conn.execute(
                `INSERT INTO products (name, spec, packaging, category, manufacturer, photo_front, photo_back, location, shelf_x, shelf_y, shelf_z, emoji, price, stock, status)
                 VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 50, ?, ?, 100, 1)`,
                [name, spec, packaging, category, manufacturer, photo_front, photo_back, location, shelf_x, shelf_y, emoji, price]
            );
            imported++;
        }

        console.log(`✅ 成功导入 ${imported} 条商品数据`);

    } catch (err) {
        console.error('❌ 错误:', err.message);
    } finally {
        if (conn) await conn.end();
    }
}

main();
