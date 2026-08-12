#!/bin/bash
# ================================================================
# 樹梅派步進馬達控制系統 - 前台手動啟動腳本
# Raspberry Pi Stepper Motor Controller - Manual Start Script
# ================================================================

# 切換到腳本所在目錄
cd "$(dirname "$0")"

echo "=================================================="
echo "  🤖 樹梅派步進馬達控制系統 (前台啟動模式)"
echo "  Raspberry Pi Stepper Motor Controller"
echo "=================================================="

# 檢查是否在樹梅派上
if grep -qi "Raspberry Pi" /proc/cpuinfo 2>/dev/null || [ -f /proc/device-tree/model ]; then
    echo "[INFO] 偵測到 Raspberry Pi 硬體"
    echo "[INFO] 檢查 / 啟動 pigpio 守護進程..."
    sudo pigpiod 2>/dev/null || true
    sleep 1
else
    echo "[INFO] 非 Raspberry Pi 硬體環境，自動啟用 PC 模擬模式"
fi

# 建立虛擬環境（若不存在）
if [ ! -d "venv" ]; then
    echo "[INFO] 建立 Python 虛擬環境..."
    python3 -m venv venv || python -m venv venv
fi

# 啟動虛擬環境
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
elif [ -f "venv/Scripts/activate" ]; then
    source venv/Scripts/activate
fi

# 檢查依賴
if [ -f "requirements.txt" ]; then
    pip install -q -r requirements.txt 2>/dev/null || true
fi

# 獲取本機 IP
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo ""
echo "[WEB] 控制系統伺服器啟動中..."
echo "[WEB] 本機瀏覽器存取 : http://localhost:5000"
if [ -n "$IP" ]; then
    echo "[WEB] 區網裝置存取   : http://${IP}:5000"
fi
echo "[WEB] 預設登入密碼   : Nice (樹梅派使用 Linux 系統密碼)"
echo ""

# 啟動主程式
python3 servo_controller.py || python servo_controller.py
