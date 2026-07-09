const https = require('https');
const http = require('http');
const fs = require('fs');
const path = require('path');

const OUT = path.join(__dirname, 'images', 'drinks');

// 映射：输出文件名 -> 下载URL
// pngimg.com 的图片路径模式已验证
const downloads = [
  // === 可口可乐 (pngimg.com) ===
  // 罐装
  { file: 'coke_can.png',   url: 'https://pngimg.com/uploads/coca_cola/coca_cola_PNG15.png' },
  // 瓶装 500ml
  { file: 'coke_bottle.png', url: 'https://pngimg.com/uploads/coca_cola/coca_cola_PNG10.png' },

  // === 雪碧 (pngimg.com) ===
  // 罐装
  { file: 'sprite_can.png',    url: 'https://pngimg.com/uploads/sprite/sprite_PNG21.png' },
  // 瓶装
  { file: 'sprite_bottle.png', url: 'https://pngimg.com/uploads/sprite/sprite_PNG18.png' },

  // === 芬达 (pngimg.com) ===
  // 罐装
  { file: 'fanta_can.png',    url: 'https://pngimg.com/uploads/fanta/fanta_PNG1.png' },
  // 瓶装
  { file: 'fanta_bottle.png', url: 'https://pngimg.com/uploads/fanta/fanta_PNG8.png' },

  // === 农夫山泉 ===
  // 尝试从 pngimg 搜索 water bottle
  { file: 'nongfu_water.png', url: 'https://pngimg.com/uploads/water_bottle/water_bottle_PNG4.png' },

  // === 冰红茶 (用冰茶/茶饮料代替) ===
  { file: 'ice_tea.png', url: 'https://pngimg.com/uploads/ice_tea/ice_tea_PNG1.png' },

  // === 康师傅绿茶 (用绿茶代替) ===
  { file: 'green_tea.png', url: 'https://pngimg.com/uploads/green_tea/green_tea_PNG1.png' },

  // === 旺仔牛奶 (用牛奶代替) ===
  { file: 'milk.png', url: 'https://pngimg.com/uploads/milk/milk_PNG2.png' },

  // === 尖叫运动饮料 (用运动饮料代替) ===
  { file: 'sports_drink.png', url: 'https://pngimg.com/uploads/energy_drink/energy_drink_PNG1.png' },

  // === AD钙奶 (用酸奶/牛奶饮料代替) ===
  { file: 'ad_milk.png', url: 'https://pngimg.com/uploads/milk/milk_PNG4.png' },
];

function downloadFile(url, filePath) {
  return new Promise((resolve, reject) => {
    const protocol = url.startsWith('https') ? https : http;
    protocol.get(url, (res) => {
      // 处理重定向
      if (res.statusCode >= 300 && res.statusCode < 400 && res.headers.location) {
        downloadFile(res.headers.location, filePath).then(resolve).catch(reject);
        return;
      }
      if (res.statusCode !== 200) {
        reject(new Error(`HTTP ${res.statusCode} for ${url}`));
        return;
      }
      const file = fs.createWriteStream(filePath);
      res.pipe(file);
      file.on('finish', () => {
        file.close();
        resolve();
      });
      file.on('error', reject);
    }).on('error', reject);
  });
}

(async () => {
  if (!fs.existsSync(OUT)) fs.mkdirSync(OUT, { recursive: true });

  for (const item of downloads) {
    const filePath = path.join(OUT, item.file);
    console.log(`⬇️ Downloading: ${item.file}`);
    try {
      await downloadFile(item.url, filePath);
      const stat = fs.statSync(filePath);
      console.log(`  ✅ ${item.file} (${(stat.size / 1024).toFixed(1)} KB)`);
    } catch (e) {
      console.log(`  ❌ ${item.file}: ${e.message}`);
    }
  }
  console.log('\n🎉 Done!');
})();
