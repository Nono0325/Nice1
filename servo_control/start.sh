#!/bin/bash
# ================================================================
# 樹莓派步進馬達控制系統 (Rust 2.0 架構) - 前台手動啟動腳本
# Raspberry Pi Stepper Motor Controller (Rust Edition) - Start Script
# ================================================================

cd "$(dirname "$0")"
export PATH="$HOME/.cargo/bin:$PATH"

echo "=================================================="
echo "  🦀 樹莓派步進馬達與電磁鐵控制系統 (Rust 2.0)"
echo "  Raspberry Pi Stepper Motor Controller (Rust)"
echo "=================================================="

if [ ! -x "./target/release/servo_control" ]; then
    if command -v cargo &>/dev/null; then
        echo "[INFO] 正在編譯 Rust Release 執行檔 (cargo build --release)..."
        cargo build --release
    else
        echo "[WARN] 未偵測到 cargo，請先執行 sudo bash install.sh 安裝 Rust 工具鏈"
        exit 1
    fi
fi

IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo ""
echo "[WEB] Rust Axum 伺服器啟動中..."
echo "[WEB] 本機瀏覽器存取 : http://localhost:5000"
if [ -n "$IP" ]; then
    echo "[WEB] 區網裝置存取   : http://${IP}:5000"
fi
echo "[WEB] 預設登入密碼   : Nice"
echo ""

exec ./target/release/servo_control
