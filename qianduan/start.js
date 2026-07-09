const { spawn, spawnSync } = require('child_process');

function runNodeScript(script) {
    const result = spawnSync(process.execPath, [script], {
        cwd: __dirname,
        stdio: 'inherit',
        env: process.env,
    });
    if (result.status !== 0) {
        throw new Error(`${script} 退出码 ${result.status}`);
    }
}

function startServer() {
    const child = spawn(process.execPath, ['server.js'], {
        cwd: __dirname,
        stdio: 'inherit',
        env: process.env,
    });
    child.on('exit', (code) => process.exit(code ?? 0));
    child.on('error', (e) => {
        console.error(e);
        process.exit(1);
    });
}

try {
    runNodeScript('init_db.js');
    startServer();
} catch (e) {
    console.error(e.message || e);
    process.exit(1);
}
