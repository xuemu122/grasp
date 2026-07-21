const net = require('node:net');

function extractJsonObject(buffer) {
    const text = buffer.toString('utf8').trimStart();
    if (!text.startsWith('{') && !text.startsWith('[')) return null;

    let depth = 0;
    let inString = false;
    let escaped = false;
    for (let index = 0; index < text.length; index += 1) {
        const char = text[index];
        if (inString) {
            if (escaped) escaped = false;
            else if (char === '\\') escaped = true;
            else if (char === '"') inString = false;
            continue;
        }
        if (char === '"') {
            inString = true;
            continue;
        }
        if (char === '{' || char === '[') depth += 1;
        else if (char === '}' || char === ']') {
            depth -= 1;
            if (depth === 0) {
                try {
                    return JSON.parse(text.slice(0, index + 1));
                } catch {
                    return null;
                }
            }
        }
    }
    return null;
}

class ChassisClient {
    constructor({ host, port, timeoutMs = 10000 } = {}) {
        this.host = host || '192.168.124.90';
        this.port = Number(port || 31001);
        this.timeoutMs = Number(timeoutMs);
    }

    request(command, timeoutMs = this.timeoutMs) {
        return new Promise((resolve, reject) => {
            const socket = net.createConnection({ host: this.host, port: this.port });
            let buffer = Buffer.alloc(0);
            let settled = false;
            const finish = (error, result) => {
                if (settled) return;
                settled = true;
                clearTimeout(timer);
                socket.destroy();
                if (error) reject(error);
                else resolve(result);
            };
            const timer = setTimeout(() => {
                finish(new Error(`底盘 TCP 请求超时: ${command}`));
            }, timeoutMs);

            socket.once('error', (error) => finish(new Error(`底盘连接失败 ${this.host}:${this.port}: ${error.message}`)));
            socket.on('data', (chunk) => {
                buffer = Buffer.concat([buffer, chunk]);
                const result = extractJsonObject(buffer);
                if (result !== null) finish(null, result);
            });
            socket.once('connect', () => socket.write(`${command}\n`));
        });
    }
}

module.exports = { ChassisClient };
