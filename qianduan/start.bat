@echo off
chcp 65001 >nul
title 智能点单系统

cd /d "%~dp0"

:: 设置 Node.js 路径
set "NODE_PATH=C:\Program Files\Lenovo\AIAgent\mcp\node-v22.16.0-win-x64"
set "PATH=%NODE_PATH%;%PATH%"

:: ---- 环境变量 ----
set PAYMENT_MODE=free
set ALLOW_FREE_ORDERS=1
set ROBOT_GRASP_MODE=sim
set ADMIN_USERNAME=admin
set ADMIN_PASSWORD=admin123
set ADMIN_SESSION_SECRET=local-dev-session-key-32bytes!!
set ORDER_TOKEN_SECRET=local-dev-order-token-key-32bytes!!
set ORDER_STATUS_CALLBACK_SECRET=local-dev-callback-key-32bytes!!

echo.
echo ============================================
echo           🤖 智能点单系统 启动中...
echo ============================================
echo.

where node >nul 2>nul
if %errorlevel% neq 0 (
    echo [错误] 未找到 Node.js
    pause
    exit /b 1
)

echo [信息] Node.js 版本:
node -v
echo.

if not exist "node_modules\" (
    echo [信息] 正在安装依赖...
    call npm install
    if %errorlevel% neq 0 (
        echo [错误] 依赖安装失败
        pause
        exit /b 1
    )
    echo [信息] 依赖安装完成
    echo.
)

echo [信息] 启动服务器...
echo.
echo 访问地址:
echo   用户端:  http://localhost:3000
echo   管理端:  http://localhost:3000/admin.html
echo.
echo 按 Ctrl+C 停止服务器
echo ============================================
echo.

node server.js

pause
