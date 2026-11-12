#!/bin/bash
# ================================================================
#  Raspberry Pi Stepper Motor Controller (Rust Edition) - Installer
#  衝壓機自動化搬運系統 - 樹莓派步進馬達控制器 (Rust 架構) 一鍵安裝腳本
#
#  Usage (一行指令安裝):
#    curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash
#  or:
#    wget -qO- https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash
# ================================================================

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
log_ok()    { echo -e "${GREEN}[  OK]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
log_step()  { echo -e "\n${BOLD}${GREEN}>>> $1${NC}"; }

REPO_URL="https://github.com/Nono0325/Nice1.git"
REPO_BRANCH="main"
RUN_USER="${SUDO_USER:-pi}"
RUN_HOME=$(getent passwd "$RUN_USER" 2>/dev/null | cut -d: -f6)
RUN_HOME="${RUN_HOME:-/home/$RUN_USER}"

REPO_DIR="${RUN_HOME}/Nice1"
INSTALL_DIR="${REPO_DIR}/servo_control"
SERVICE_NAME="servo-control"
PORT=5000

echo ""
echo -e "${BOLD}${CYAN}========================================================${NC}"
echo -e "${BOLD}${CYAN}||  🦀 Raspberry Pi Stepper Controller (Rust 2.0)     ||${NC}"
echo -e "${BOLD}${CYAN}||   樹莓派步進馬達與電磁鐵控制系統 — Rust 自動安裝   ||${NC}"
echo -e "${BOLD}${CYAN}========================================================${NC}"
echo ""

if [ "$EUID" -ne 0 ]; then
    log_warn "未以 root/sudo 執行。系統服務 (systemd) 安裝將被略過。"
    log_warn "建議使用: sudo bash $0"
fi

log_step "1/6 偵測系統硬體與環境"

IS_PI=false
PI_MODEL="Unknown"
if grep -qi "Raspberry Pi" /proc/cpuinfo 2>/dev/null || grep -qi "BCM" /proc/cpuinfo 2>/dev/null || [ -f /proc/device-tree/model ]; then
    IS_PI=true
    if [ -f /proc/device-tree/model ]; then
        PI_MODEL=$(tr -d '\0' < /proc/device-tree/model)
    elif grep -q "Model" /proc/cpuinfo; then
        PI_MODEL=$(grep "Model" /proc/cpuinfo | head -1 | awk -F: '{print $2}' | xargs)
    fi
    log_ok "偵測到樹莓派硬體: ${PI_MODEL}"
else
    log_warn "非樹莓派硬體環境 — 系統將啟用 PC 虛擬模擬模式 (Simulation Mode)"
fi

OS_ID=$(grep "^ID=" /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '"')
OS_VERSION=$(grep "^VERSION_ID=" /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '"')
log_info "作業系統: ${OS_ID:-unknown} ${OS_VERSION:-}"
log_info "執行身分: ${RUN_USER} (家目錄: ${RUN_HOME})"

log_step "2/6 安裝 Linux 編譯工具與系統相依套件"

if command -v apt-get &>/dev/null; then
    log_info "更新 apt 套件清單..."
    apt-get update -qq || log_warn "apt-get update 提示警告，繼續執行..."
    log_info "安裝 build-essential / pkg-config / git / curl..."
    apt-get install -y -qq build-essential pkg-config libssl-dev git curl ca-certificates 2>/dev/null || true
    log_ok "系統套件安裝完成"
fi

log_step "3/6 下載 / 同步 Git 倉庫 (${REPO_URL})"

# 設定 safe.directory 避免 sudo/root 與一般使用者不同身分觸發 dubious ownership
git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true
git config --global --add safe.directory "${RUN_HOME}/Nice" 2>/dev/null || true
git config --global --add safe.directory "*" 2>/dev/null || true

if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
    su - "$RUN_USER" -c "git config --global --add safe.directory '*' 2>/dev/null || true" 2>/dev/null || true
    if [ -d "$REPO_DIR" ]; then
        chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
    fi
    if [ -d "${RUN_HOME}/Nice" ]; then
        chown -R "${RUN_USER}:${RUN_USER}" "${RUN_HOME}/Nice" 2>/dev/null || true
    fi
fi

sync_git_repo() {
    local dir="$1"
    cd "$dir"
    # 自動清除可能因斷電造成的 0-byte 損壞 git 物件
    find .git/objects/ -type f -empty -delete 2>/dev/null || true

    local fetch_ok=0
    git fetch origin "$REPO_BRANCH" 2>/dev/null && git reset --hard "origin/$REPO_BRANCH" 2>/dev/null || fetch_ok=1

    if [ "$fetch_ok" -ne 0 ] || ! git log -1 &>/dev/null; then
        log_warn "偵測到本機 Git 倉庫損壞 (可能是先前斷電或中斷造成 0-byte 損壞物件)"
        log_info "正在自動修復：備份設定檔並重新 Clone 乾淨倉庫..."
        mkdir -p /tmp/nice1_cfg_backup
        cp -a "${dir}/servo_control/"*.json /tmp/nice1_cfg_backup/ 2>/dev/null || true
        cd "$RUN_HOME"
        rm -rf "$dir"
        git clone --branch "$REPO_BRANCH" --depth=1 "$REPO_URL" "$dir"
        cp -a /tmp/nice1_cfg_backup/*.json "${dir}/servo_control/" 2>/dev/null || true
        rm -rf /tmp/nice1_cfg_backup
    fi
    cd "$dir"
    git log -1 --format="%h (%s)" 2>/dev/null || echo "Unknown"
}

if [ -d "$REPO_DIR/.git" ]; then
    log_info "已存在 Git 倉庫，正在檢查並同步最新版本: ${REPO_DIR}"
    if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
        chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
        CURRENT_COMMIT=$(su - "$RUN_USER" -c "$(declare -f sync_git_repo); sync_git_repo '${REPO_DIR}'")
    else
        CURRENT_COMMIT=$(sync_git_repo "$REPO_DIR")
    fi
    log_ok "倉庫已同步至最新 Commit: ${CURRENT_COMMIT}"
elif [ -d "${RUN_HOME}/Nice/.git" ]; then
    REPO_DIR="${RUN_HOME}/Nice"
    INSTALL_DIR="${REPO_DIR}/servo_control"
    log_info "偵測到既有倉庫目錄: ${REPO_DIR}"
    if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
        chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
        CURRENT_COMMIT=$(su - "$RUN_USER" -c "$(declare -f sync_git_repo); sync_git_repo '${REPO_DIR}'")
    else
        CURRENT_COMMIT=$(sync_git_repo "$REPO_DIR")
    fi
    log_ok "倉庫已同步至最新 Commit: ${CURRENT_COMMIT}"
else
    log_info "開始 Clone 倉庫至: ${REPO_DIR}"
    if [ -d "$REPO_DIR" ] && [ ! -d "$REPO_DIR/.git" ]; then
        rm -rf "$REPO_DIR" 2>/dev/null || true
    fi
    if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
        chown -R "${RUN_USER}:${RUN_USER}" "$RUN_HOME" 2>/dev/null || true
        su - "$RUN_USER" -c "git clone --branch '${REPO_BRANCH}' --depth=1 '${REPO_URL}' '${REPO_DIR}'" || {
            log_warn "Clone 失敗，正在清理目標目錄並重新嘗試..."
            rm -rf "$REPO_DIR" 2>/dev/null || true
            su - "$RUN_USER" -c "git clone --branch '${REPO_BRANCH}' --depth=1 '${REPO_URL}' '${REPO_DIR}'"
        }
        chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
    else
        git clone --branch "$REPO_BRANCH" --depth=1 "$REPO_URL" "$REPO_DIR" || {
            log_warn "Clone 失敗，正在清理目標目錄並重新嘗試..."
            rm -rf "$REPO_DIR" 2>/dev/null || true
            git clone --branch "$REPO_BRANCH" --depth=1 "$REPO_URL" "$REPO_DIR"
        }
    fi
    CURRENT_COMMIT=$(cd "$REPO_DIR" && git log -1 --format="%h (%s)" 2>/dev/null || echo "Unknown")
    log_ok "倉庫下載成功，當前 Commit: ${CURRENT_COMMIT}"
fi

cd "$INSTALL_DIR"

log_step "4/6 安裝 Rust 工具鏈 (rustup / cargo)"

CARGO_BIN="${RUN_HOME}/.cargo/bin/cargo"
if [ ! -x "$CARGO_BIN" ] && ! command -v cargo &>/dev/null; then
    log_info "正在為使用者 ${RUN_USER} 安裝 Rust 編譯環境 (rustup)..."
    if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
        su - "$RUN_USER" -c "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"
    else
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    fi
    log_ok "Rust 工具鏈安裝完成"
else
    log_ok "已偵測到 Rust cargo 工具鏈"
fi

export PATH="${RUN_HOME}/.cargo/bin:/root/.cargo/bin:${PATH}"

log_step "5/6 編譯 Rust 高效能控制核心 (cargo build --release)"

if [ "$EUID" -eq 0 ]; then
    chown -R "${RUN_USER}:${RUN_USER}" "$REPO_DIR" 2>/dev/null || true
    # 加入 gpio 群組權限，確保無須 root 亦可直接存取 /dev/gpiomem
    usermod -aG gpio "$RUN_USER" 2>/dev/null || true
fi

log_info "開始編譯 Release 最佳化執行檔 (初次編譯約需 2~4 分鐘)..."
CARGO_CMD="${RUN_HOME}/.cargo/bin/cargo"
if [ ! -x "$CARGO_CMD" ]; then
    CARGO_CMD=$(command -v cargo || echo "cargo")
fi
CARGO_VER=$($CARGO_CMD --version 2>/dev/null || echo "Unknown")
log_info "編譯環境: ${CARGO_VER} (編譯路徑: ${INSTALL_DIR})"

if [ "$EUID" -eq 0 ] && [ "$RUN_USER" != "root" ]; then
    su - "$RUN_USER" -c "cd '${INSTALL_DIR}' && '$CARGO_CMD' build --release"
else
    cd "${INSTALL_DIR}"
    $CARGO_CMD build --release
fi

chmod +x "$INSTALL_DIR/target/release/servo_control" 2>/dev/null || true
chmod +x "$INSTALL_DIR/start.sh" 2>/dev/null || true
chmod +x "$INSTALL_DIR/install.sh" 2>/dev/null || true
log_ok "Rust 控制核心編譯完成: ${INSTALL_DIR}/target/release/servo_control"

log_step "6/6 配置 systemd 開機自動啟動服務"

if [ "$EUID" -eq 0 ]; then
    SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
    log_info "寫入服務設定檔: ${SERVICE_FILE}"

    cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Raspberry Pi Stepper Motor Controller (Rust Axum Backend)
Documentation=https://github.com/Nono0325/Nice1
After=network.target
Wants=network.target

[Service]
Type=simple
User=${RUN_USER}
SupplementaryGroups=gpio
WorkingDirectory=${INSTALL_DIR}
ExecStart=${INSTALL_DIR}/target/release/servo_control
Restart=always
RestartSec=2
StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}
Environment=RUST_LOG=info

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}.service" --quiet 2>/dev/null || true
    systemctl restart "${SERVICE_NAME}.service" --quiet 2>/dev/null || true
    sleep 2

    if systemctl is-active --quiet "${SERVICE_NAME}"; then
        log_ok "systemd Rust 服務已成功啟動並設為開機自動執行！"
    else
        log_warn "服務啟動中或需檢查日誌: sudo journalctl -u ${SERVICE_NAME} -n 20"
    fi
else
    log_warn "非 root 權限，略過 systemd 服務安裝。手動啟動請執行: cd ${INSTALL_DIR} && ./start.sh"
fi

IP=$(hostname -I 2>/dev/null | awk '{print $1}')
HOSTNAME_STR=$(hostname 2>/dev/null || echo "raspberrypi")

echo ""
echo -e "${BOLD}${GREEN}========================================================${NC}"
echo -e "${BOLD}${GREEN}||        🎉 Rust 2.0 架構安裝完成！Complete!         ||${NC}"
echo -e "${BOLD}${GREEN}========================================================${NC}"
echo ""
echo -e "  📁 安裝主目錄     : ${CYAN}${INSTALL_DIR}${NC}"
echo -e "  🦀 執行檔路徑     : ${CYAN}${INSTALL_DIR}/target/release/servo_control${NC}"
echo -e "  🌐 本地連線網址   : ${CYAN}http://localhost:${PORT}${NC}"
if [ -n "$IP" ]; then
    echo -e "  📡 區網連線網址   : ${CYAN}http://${IP}:${PORT}${NC}"
    echo -e "  🔗 主機名連線     : ${CYAN}http://${HOSTNAME_STR}.local:${PORT}${NC}"
fi
echo ""
echo -e "  🔐 預設登入密碼   : ${YELLOW}Nice${NC} (或樹莓派 Linux 系統密碼)"
echo ""
echo -e "${BOLD}${CYAN}========================================================${NC}"
