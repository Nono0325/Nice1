#!/bin/bash
# ================================================================
#  Raspberry Pi Stepper Motor Controller - One-Click Installer
#  衝壓機自動化搬運系統 - 樹梅派步進馬達控制器一鍵安裝腳本
#
#  Usage (一行指令安裝):
#    curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash
#  or:
#    wget -qO- https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash
# ================================================================

set -e  # 遇到錯誤立即中止

# ── 顏色與日誌定義 ─────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
log_ok()    { echo -e "${GREEN}[  OK]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
log_step()  { echo -e "\n${BOLD}${GREEN}>>> $1${NC}"; }

# ── 設定參數 ───────────────────────────────────────────────────
REPO_URL="https://github.com/Nono0325/Nice1.git"
REPO_BRANCH="main"
RUN_USER="${SUDO_USER:-pi}"
RUN_HOME=$(getent passwd "$RUN_USER" 2>/dev/null | cut -d: -f6)
RUN_HOME="${RUN_HOME:-/home/$RUN_USER}"

REPO_DIR="${RUN_HOME}/Nice1"
INSTALL_DIR="${REPO_DIR}/servo_control"
SERVICE_NAME="servo-control"
PYTHON_CMD="python3"
PORT=5000

# ── 橫幅 ───────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${CYAN}========================================================${NC}"
echo -e "${BOLD}${CYAN}||   Raspberry Pi Stepper Motor Controller Installer  ||${NC}"
echo -e "${BOLD}${CYAN}||   樹梅派步進馬達控制系統 — 自動安裝與配置工具      ||${NC}"
echo -e "${BOLD}${CYAN}========================================================${NC}"
echo ""

# ── 檢查 Root / sudo 權限 ───────────────────────────────────────
if [ "$EUID" -ne 0 ]; then
    log_warn "未以 root/sudo 執行。系統服務 (systemd) 安裝將被略過。"
    log_warn "建議使用: sudo bash $0"
fi

# ── 平台環境偵測 ───────────────────────────────────────────────
log_step "1/7 偵測系統硬體與環境"

IS_PI=false
PI_MODEL="Unknown"
if grep -qi "Raspberry Pi" /proc/cpuinfo 2>/dev/null || grep -qi "BCM" /proc/cpuinfo 2>/dev/null || [ -f /proc/device-tree/model ]; then
    IS_PI=true
    if [ -f /proc/device-tree/model ]; then
        PI_MODEL=$(tr -d '\0' < /proc/device-tree/model)
    elif grep -q "Model" /proc/cpuinfo; then
        PI_MODEL=$(grep "Model" /proc/cpuinfo | head -1 | awk -F: '{print $2}' | xargs)
    fi
    log_ok "偵測到樹梅派硬體: ${PI_MODEL}"
else
    log_warn "非樹梅派硬體環境 — 系統將啟用 PC 虛擬模擬模式 (Simulation Mode)"
fi

OS_ID=$(grep "^ID=" /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '"')
OS_VERSION=$(grep "^VERSION_ID=" /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '"')
log_info "作業系統: ${OS_ID:-unknown} ${OS_VERSION:-}"
log_info "執行身分: ${RUN_USER} (家目錄: ${RUN_HOME})"

# ── 安裝系統相依套件 ───────────────────────────────────────────
log_step "2/7 安裝 Linux 系統相依套件"

if command -v apt-get &>/dev/null; then
    log_info "更新 apt 套件清單..."
    apt-get update -qq || log_warn "apt-get update 提示警告，繼續執行..."

    log_info "安裝 Python3 / pip / venv / git / curl..."
    apt-get install -y -qq python3 python3-pip python3-venv git curl 2>/dev/null || true

    if [ "$IS_PI" = true ]; then
        log_info "安裝樹梅派 GPIO 與 pigpio 底層驅動庫..."
        apt-get install -y -qq python3-rpi.gpio pigpio python3-pigpio python3-lgpio 2>/dev/null || true
    fi
    log_ok "系統套件安裝完成"
else
    log_warn "未偵測到 apt-get 套件管理器，略過系統套件安裝步驟"
fi

# ── 下載或更新專案倉庫 ─────────────────────────────────────────
log_step "3/7 下載 / 同步 Git 倉庫 (${REPO_URL})"

if [ -d "$REPO_DIR/.git" ]; then
    log_info "已存在 Git 倉庫，正在拉取最新版本: ${REPO_DIR}"
    cd "$REPO_DIR"
    git fetch origin "$REPO_BRANCH" --quiet 2>/dev/null || true
    git reset --hard "origin/$REPO_BRANCH" --quiet 2>/dev/null || git pull origin "$REPO_BRANCH" --quiet 2>/dev/null || true
    log_ok "倉庫已同步至最新 Commit"
else
    # 支援遷移舊版直接安裝於 /home/pi/servo_control 的情境
    if [ -d "${RUN_HOME}/servo_control/.git" ]; then
        log_info "偵測到既有獨立目錄，同步中: ${RUN_HOME}/servo_control"
        REPO_DIR="${RUN_HOME}/servo_control"
        INSTALL_DIR="${RUN_HOME}/servo_control"
        cd "$INSTALL_DIR"
        git fetch origin "$REPO_BRANCH" --quiet 2>/dev/null || true
        git reset --hard "origin/$REPO_BRANCH" --quiet 2>/dev/null || true
    else
        log_info "開始完整 Clone 倉庫至: ${REPO_DIR}"
        mkdir -p "$REPO_DIR"
        git clone --branch "$REPO_BRANCH" --depth=1 "$REPO_URL" "$REPO_DIR"
        log_ok "倉庫下載成功"
    fi
fi

# 確保主程式目錄存在
if [ ! -d "$INSTALL_DIR" ]; then
    if [ -f "$REPO_DIR/servo_controller.py" ]; then
        INSTALL_DIR="$REPO_DIR"
    else
        log_error "找不到 servo_control 程式目錄，請檢查倉庫結構！"
    fi
fi

cd "$INSTALL_DIR"

# ── 建立 Python 虛擬環境 ───────────────────────────────────────
log_step "4/7 配置 Python 虛擬環境 (Virtualenv)"

if [ ! -d "$INSTALL_DIR/venv" ]; then
    log_info "建立虛擬環境於: $INSTALL_DIR/venv"
    $PYTHON_CMD -m venv "$INSTALL_DIR/venv"
    log_ok "虛擬環境建立成功"
else
    log_info "虛擬環境已存在: $INSTALL_DIR/venv"
fi

# 啟動虛擬環境
source "$INSTALL_DIR/venv/bin/activate"

# ── 安裝 Python 相依套件 ───────────────────────────────────────
log_step "5/7 安裝 Python 依賴套件"

log_info "升級 pip 工具..."
pip install --upgrade pip -q 2>/dev/null || true

if [ -f "$INSTALL_DIR/requirements.txt" ]; then
    log_info "透過 requirements.txt 安裝依賴..."
    pip install -r "$INSTALL_DIR/requirements.txt" -q 2>/dev/null || {
        log_warn "requirements.txt 安裝遇到非致命問題，嘗試個別安裝核心套件..."
        pip install flask flask-socketio eventlet python-socketio -q
    }
else
    log_info "安裝核心 Web 套件..."
    pip install flask flask-socketio eventlet python-socketio -q
fi

# 針對樹梅派硬體額外安裝相容 GPIO 模組
if [ "$IS_PI" = true ]; then
    log_info "配置樹梅派 GPIO Python 套件 (RPi.GPIO / rpi-lgpio / pigpio)..."
    pip install RPi.GPIO -q 2>/dev/null || pip install rpi-lgpio -q 2>/dev/null || log_warn "GPIO 函式庫由系統 apt 提供或啟用模擬模式"
    pip install pigpio -q 2>/dev/null || true
fi

log_ok "Python 依賴套件安裝完成"

# ── 檔案權限設定 ───────────────────────────────────────────────
log_step "6/7 設定檔案執行權限"

chmod +x "$INSTALL_DIR/start.sh" 2>/dev/null || true
chmod +x "$INSTALL_DIR/install.sh" 2>/dev/null || true
if [ "$EUID" -eq 0 ]; then
    chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
fi
log_ok "檔案權限配置完成"

# ── 設定 pigpio 與 systemd 開機自啟服務 ────────────────────────
log_step "7/7 配置開機自動啟動服務 (systemd)"

if [ "$IS_PI" = true ] && [ "$EUID" -eq 0 ]; then
    log_info "啟動並開機啟用 pigpio 守護進程..."
    systemctl enable pigpiod 2>/dev/null || true
    systemctl start pigpiod 2>/dev/null || true
    log_ok "pigpiod 守護服務已啟用"
fi

if [ "$EUID" -eq 0 ]; then
    SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
    log_info "寫入服務設定檔: ${SERVICE_FILE}"

    cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Raspberry Pi Stepper Motor Controller
Documentation=https://github.com/Nono0325/Nice
After=network.target pigpiod.service
Wants=network.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${INSTALL_DIR}
ExecStart=${INSTALL_DIR}/venv/bin/python ${INSTALL_DIR}/servo_controller.py
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}.service" --quiet 2>/dev/null || true
    systemctl restart "${SERVICE_NAME}.service" --quiet 2>/dev/null || true
    sleep 2

    if systemctl is-active --quiet "${SERVICE_NAME}"; then
        log_ok "systemd 服務已成功啟動並設為開機自動執行！"
    else
        log_warn "服務啟動中或需檢查日誌: sudo journalctl -u ${SERVICE_NAME} -n 20"
    fi
else
    log_warn "非 root 權限，略過 systemd 服務安裝。手動啟動請執行: cd ${INSTALL_DIR} && ./start.sh"
fi

# ── 安裝完成資訊展示 ───────────────────────────────────────────
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
HOSTNAME_STR=$(hostname 2>/dev/null || echo "raspberrypi")

echo ""
echo -e "${BOLD}${GREEN}========================================================${NC}"
echo -e "${BOLD}${GREEN}||               🎉 安裝完成！Installation Complete!     ||${NC}"
echo -e "${BOLD}${GREEN}========================================================${NC}"
echo ""
echo -e "  📁 安裝主目錄     : ${CYAN}${INSTALL_DIR}${NC}"
echo -e "  🌐 本地連線網址   : ${CYAN}http://localhost:${PORT}${NC}"
if [ -n "$IP" ]; then
    echo -e "  📡 區網連線網址   : ${CYAN}http://${IP}:${PORT}${NC}"
    echo -e "  🔗 主機名連線     : ${CYAN}http://${HOSTNAME_STR}.local:${PORT}${NC}"
fi
echo ""
echo -e "  🔐 預設登入密碼   : ${YELLOW}Nice${NC} (樹梅派請輸入 Linux 系統密碼)"
echo ""
echo -e "  ${BOLD}常用管理指令：${NC}"
echo -e "  手動前台啟動 : ${YELLOW}cd ${INSTALL_DIR} && ./start.sh${NC}"
if [ "$EUID" -eq 0 ]; then
    echo -e "  檢視服務狀態 : ${YELLOW}sudo systemctl status ${SERVICE_NAME}${NC}"
    echo -e "  檢視即時日誌 : ${YELLOW}sudo journalctl -u ${SERVICE_NAME} -f${NC}"
    echo -e "  重啟控制服務 : ${YELLOW}sudo systemctl restart ${SERVICE_NAME}${NC}"
    echo -e "  停止控制服務 : ${YELLOW}sudo systemctl stop ${SERVICE_NAME}${NC}"
    echo -e "  關閉開機自啟 : ${YELLOW}sudo systemctl disable ${SERVICE_NAME}${NC}"
fi
echo ""
echo -e "${BOLD}${CYAN}========================================================${NC}"
