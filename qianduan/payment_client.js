const http = require('node:http');
const https = require('node:https');

class PaymentClient {
    constructor({ baseUrl, token = '', timeoutMs = 15000, rejectUnauthorized = true } = {}) {
        this.baseUrl = String(baseUrl || '').replace(/\/$/, '');
        this.token = String(token || '');
        this.timeoutMs = Number(timeoutMs) || 15000;
        this.rejectUnauthorized = rejectUnauthorized !== false;
        this.menuItemSyncs = new Map();
    }

    enabled() {
        return Boolean(this.baseUrl);
    }

    request(method, pathname, body, extraHeaders = {}) {
        if (!this.enabled()) throw new Error('未配置 PAYMENT_API_BASE_URL');
        const target = new URL(pathname, `${this.baseUrl}/`);
        const transport = target.protocol === 'https:' ? https : http;
        const payload = body === undefined ? null : Buffer.from(JSON.stringify(body));
        const headers = { Accept: 'application/json', ...extraHeaders };
        if (payload) {
            headers['Content-Type'] = 'application/json';
            headers['Content-Length'] = payload.length;
        }
        if (this.token) headers.Authorization = `Bearer ${this.token}`;

        return new Promise((resolve, reject) => {
            const req = transport.request({
                protocol: target.protocol,
                hostname: target.hostname,
                port: target.port || undefined,
                path: `${target.pathname}${target.search}`,
                method,
                headers,
                timeout: this.timeoutMs,
                rejectUnauthorized: this.rejectUnauthorized
            }, (res) => {
                const chunks = [];
                res.on('data', (chunk) => chunks.push(chunk));
                res.once('end', () => {
                    const text = Buffer.concat(chunks).toString('utf8');
                    let data;
                    try { data = text ? JSON.parse(text) : {}; }
                    catch { reject(new Error(`支付 API 返回非 JSON (${res.statusCode})`)); return; }
                    if (res.statusCode < 200 || res.statusCode >= 300 || data.code === 1 || data.success === false) {
                        reject(new Error(data.message || data.msg || `支付 API 请求失败 (${res.statusCode})`));
                        return;
                    }
                    resolve(data);
                });
            });
            req.once('timeout', () => req.destroy(new Error('支付 API 请求超时')));
            req.once('error', reject);
            if (payload) req.write(payload);
            req.end();
        });
    }

    async ensureMenuItem({ deviceSn, productId, name, spec, unitPrice, stock }) {
        const price = Math.round(Number(unitPrice) * 100);
        if (!Number.isInteger(price) || price < 1) throw new Error(`商品 ${name} 的支付价格无效`);
        const externalName = `机器人商品-${productId}-${name}${spec ? `（${spec}）` : ''}`;
        const cacheKey = `${deviceSn}:\0${externalName}:\0${price}`;
        if (this.menuItemSyncs.has(cacheKey)) return this.menuItemSyncs.get(cacheKey);

        const sync = (async () => {
            const query = new URLSearchParams({
                page: '1',
                size: '200',
                name: externalName,
                deviceSn
            });
            const listed = await this.request('GET', `/api/MenuItem/list?${query.toString()}`);
            const listData = listed.data || listed.result || {};
            const items = Array.isArray(listData) ? listData : (listData.items || []);
            let menuItem = items.find((item) => String(item.name || '') === externalName);

            if (!menuItem) {
                const created = await this.request('POST', '/api/MenuItem/create', {
                    name: externalName,
                    description: `${name}${spec ? ` ${spec}` : ''}`,
                    price,
                    isActive: true,
                    stock: Math.max(0, Number(stock) || 0),
                    sortOrder: Number(productId) || 0,
                    deviceSn,
                    source: '抓取机器人'
                });
                menuItem = created.data || created.result || created;
            } else if (Number(menuItem.price) !== price || menuItem.isActive === false) {
                const updated = await this.request('POST', '/api/MenuItem/update', {
                    id: Number(menuItem.id),
                    name: externalName,
                    description: `${name}${spec ? ` ${spec}` : ''}`,
                    price,
                    isActive: true,
                    stock: Math.max(0, Number(stock) || 0),
                    sortOrder: Number(productId) || 0,
                    source: '抓取机器人'
                });
                menuItem = updated.data || updated.result || updated;
            }

            const id = Number(menuItem && menuItem.id);
            if (!Number.isInteger(id) || id < 1) throw new Error(`支付系统未返回商品 ${name} 的有效菜品 ID`);
            return id;
        })();
        this.menuItemSyncs.set(cacheKey, sync);
        try {
            return await sync;
        } catch (error) {
            this.menuItemSyncs.delete(cacheKey);
            throw error;
        }
    }

    async createBusinessOrder({ deviceSn, items }) {
        const response = await this.request('POST', '/api/Order/create', {
            deviceSn,
            payChannel: 'wechat',
            items: items.map((item) => ({
                menuItemId: Number(item.paymentMenuItemId),
                quantity: Number(item.qty),
                unitPrice: Number(item.unitPrice)
            }))
        });
        const data = response.data || response.result || response;
        const orderId = data.orderId || data.order_id;
        if (!orderId) throw new Error('支付业务订单接口未返回 orderId');
        const totalAmount = Number(data.totalPrice ?? data.total_price);
        return { orderId: String(orderId), totalAmount: Number.isFinite(totalAmount) ? totalAmount : null, raw: response };
    }

    async createNativePayment(orderId, description) {
        const response = await this.request('POST', '/api/WeChatPay/native', {
            orderId,
            description
        });
        const data = response.data || response.result || response;
        const codeUrl = data.codeUrl || data.code_url || data.qrCodeUrl;
        if (!codeUrl) throw new Error('支付接口未返回 codeUrl');
        const qrCodeBase64 = data.qrCodeBase64 || data.qr_code_base64 || null;
        return { codeUrl: String(codeUrl), qrCodeBase64: qrCodeBase64 ? String(qrCodeBase64) : null, raw: response };
    }

    async query(orderId) {
        const response = await this.request('GET', `/api/WeChatPay/query/${encodeURIComponent(orderId)}`);
        const data = response.data || response.result || response;
        const amount = data.totalAmount ?? data.total_amount ?? data.totalPrice ?? data.total_price;
        return {
            tradeState: String(data.tradeState || data.trade_state || '').toUpperCase(),
            tradeStateDesc: data.tradeStateDesc || data.trade_state_desc || '',
            totalAmount: amount === undefined || amount === null || !Number.isFinite(Number(amount)) ? null : Number(amount),
            currency: String(data.currency || data.feeType || data.fee_type || '').toUpperCase() || null,
            raw: response
        };
    }

    async refund(orderId, reason, refundRequestId) {
        const response = await this.request(
            'POST',
            '/api/WeChatPay/refund',
            { orderId, reason, refundRequestId },
            refundRequestId ? { 'Idempotency-Key': refundRequestId } : {}
        );
        const data = response.data || response.result || response;
        return {
            refundId: data.refundId || data.refund_id || null,
            refundStatus: String(data.refundStatus || data.refund_status || 'PROCESSING').toUpperCase(),
            raw: response
        };
    }
}

module.exports = { PaymentClient };
