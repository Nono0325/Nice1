# 🤖 樹梅派步進馬達控制系統 (Raspberry Pi Stepper Motor Controller)

> **衝壓機自動化搬運系統** — Raspberry Pi X/Y 軸步進馬達控制器 + 即時網頁監控 + 3D 數位分身模擬 + G-code/巨集編輯器 + 系統終端機

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)](https://python.org)
[![Flask](https://img.shields.io/badge/Flask-3.x-lightgrey?logo=flask)](https://flask.palletsprojects.com)
[![Socket.IO](https://img.shields.io/badge/Socket.IO-4.x-black?logo=socketdotio)](https://socket.io)
[![Three.js](https://img.shields.io/badge/Three.js-r128-orange?logo=threedotjs)](https://threejs.org)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-GPIO-red?logo=raspberry-pi)](https://www.raspberrypi.org)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## ⚡ 一鍵安裝（樹梅派）

SSH 連線到樹梅派後，執行以下**任一指令**即可完成安裝＋設定開機自動執行：

```bash
# 方式一：curl
curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash

# 方式二：wget
wget -qO- https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash
```

安裝完成後，**瀏覽器開啟** `http://<樹梅派IP>:5000` 即可使用。

---

## 🖥️ Windows / PC 本地測試（模擬模式）

```bash
git clone https://github.com/Nono0325/Nice.git
cd Nice/servo_control
pip install -r requirements.txt
python servo_controller.py
```

瀏覽器開啟：`http://localhost:5000`  
- 🔐 模擬模式預設登入密碼：`Nice`

---

## 📸 介面亮點與功能特色

- 🔐 **全站登入防護與安全驗證**：進入系統即啟用安全遮罩，支援密碼登入驗證、自動閒置超時鎖定與密碼變更。
- 🎮 **工業級控制面板（仿 PPT HMI）**：START_PB / STOP_PB 金屬質感按鈕、6 顆狀態指示燈、X/Y 軸可拖曳位置條、自訂步進數值輸入。
- 🤖 **動作巨集管理器**：可自訂動作腳本指令（`MOVE`, `DELAY`, `VACUUM`, `ARM`, `HOME`, `MSG`），支援執行、暫停與腳本儲存。
- 📁 **G-code 解析與即時編輯器**：支援 `.gcode` / `.nc` 檔案上傳與即時 2D 路徑繪製，提供線上 G-code 編輯與快速指令插入。
- 🕹️ **3D 實時機械模擬（Three.js）**：3D 數位分身動態聯動真實馬達位置，具備自由視角旋轉、縮放與平移。
- 💻 **Web 系統終端機**：受密碼保護的 Web Console，支援 `sudo`、`cd` 目錄切換、快捷常用系統監控指令與即時輸出。
- 🔄 **GitHub 線上 OTA 自動更新**：一鍵檢查遠端最新 Commit 並透過 Git 同步更新，樹梅派環境下自動重啟服務。
- 🎨 **介面字體與外觀自訂**：內建 Inter、Roboto、Outfit、JetBrains Mono、Fira Code 等字體與字型大小即時調節。

---

## 🔌 硬體接線（BCM 編號）- 步進馬達配置

| 功能 | GPIO | 說明 |
|------|------|------|
| **X_PUL 步進脈衝** | GPIO 12 | X 軸步進馬達脈衝輸入 (Row 3 - IO12) |
| **X_DIR 步進方向** | GPIO 16 | X 軸步進馬達方向輸入 (Row 2 - IO16) |
| **X_ENA 步進致能** | GPIO 20 | X 軸步進馬達致能（LOW=啟動, HIGH=釋放）|
| **Y_PUL 步進脈衝** | GPIO 13 | Y 軸步進脈衝輸入 (Row 4 - IO13) |
| **Y_DIR 步進方向** | GPIO 19 | Y 軸步進方向輸入 (Row 3 - IO19) |
| **Y_ENA 步進致能** | GPIO 26 | Y 軸步進致能（LOW=啟動, HIGH=釋放）|
| **吸盤繼電器** | GPIO 24 | 數位輸出 HIGH=ON (Row 2 - IO24) |
| **Z_DOWN 繼電器** | GPIO 25 | 手臂下降繼電器 HIGH=ON (Row 2 - IO25) |
| **限位 LEFT** | GPIO 17 | INPUT_PULLUP（觸發=LOW）(Row 1 - IO17) |
| **限位 RIGHT** | GPIO 27 | INPUT_PULLUP（觸發=LOW）(Row 4 - IO27) |
| **限位 UP** | GPIO 22 | INPUT_PULLUP（觸發=LOW）(Row 3 - IO22) |
| **限位 DOWN** | GPIO 23 | INPUT_PULLUP（觸發=LOW）(Row 3 - IO23) |
| **GND** | GND | 接地 |
| **5V / 24V** | 外部電源 | 馬達電源（建議外接電源，避免從 Pi 供電） |

> ⚠️ Y1/Y2 目前共用同一組 Y_PUL/DIR/ENA 引腳（同一個步進馬達實體同步驅動）。  
> ⚠️ 步進馬達電流需求較大，請使用**外部 5V/24V 獨立電源**供電。

---

## 📐 位置與行程單位說明

| 參數 | 數值 |
|------|------|
| 位置表示方式 | **虛擬 µs 單元** (500 ~ 2500)，對應步進馬達行程 |
| 最小位置（LEFT / DOWN） | **500 µs** = 行程 0% |
| 中心位置（IDLE / HOME） | **1500 µs** = 行程 50% |
| 最大位置（RIGHT / UP） | **2500 µs** = 行程 100% |
| 比例換算 | 預設 10 µs = 1 mm（可於範圍設定頁動態校準）|

---

## 🎮 網頁介面功能（9 大 Tab）

### 1️⃣ 控制面板
- **START_PB / STOP_PB** 工業質感按鈕（仿 PPT HMI）
- 6 顆狀態指示燈：`Start Pickup` / `Arm Down` / `Vacuum` / `Lim LEFT` / `Lim RIGHT` / `Error`
- **X 軸 / Y 軸位置條**：可拖曳滑桿、支援 mm 與百分比雙重即時顯示
- 方向按鈕 + 步進值選擇（10 / 50 / 100 / 200 µs + 自訂數值輸入）
- Arm Down / Vacuum 快速切換按鈕
- 🔄 **自動取件流程**：X 移動 → 手臂下降 → 吸盤吸取 → 搬運 → 放置 → 歸零

### 2️⃣ 即時監控
- X / Y1 / Y2 三軸儀表量規（Canvas 動態繪製）
- 數位輸出狀態即時指示（VACUUM / Z_DOWN）
- 限位開關狀態即時反饋（LEFT / RIGHT / UP / DOWN）
- 系統即時操作與歷史日誌

### 3️⃣ 訊號檢測
- 即時 **PWM 波形圖**（X 軸 + Y1/Y2 軸分軌即時 Chart.js 繪製）
- Pulse Width（µs）+ Duty Cycle（%）即時參數表
- PWM 參數視覺化說明

### 4️⃣ 範圍設定與系統管理
- X 軸與 Y 軸最小/最大行程設定（µs 滑桿）
- 速度規劃與梯形加減速參數（Max Speed Hz, Accel Steps, Scale）
- 外觀與字體選擇（Inter / Roboto / Outfit / JetBrains Mono / Fira Code）及基礎字級調節
- 自動鎖定時間（1~480 分鐘）與模擬模式密碼修改
- 樹梅派系統管理：遠端重新啟動 (Reboot) / 關機 (Shutdown)
- 🔄 **GitHub 系統更新**：線上檢查版本、差異對比與一鍵升級
- GPIO 腳位完整對照表

### 5️⃣ 馬達診斷
- GPIO 模式偵測（Raspberry Pi Real GPIO / Simulation Mode）
- 馬達連續掃描測試（X / Y 軸）
- 吸盤與繼電器作動測試
- 即時診斷日誌輸出

### 6️⃣ 動作巨集
- 巨集腳本動態建立、儲存與刪除
- 動作指令支援：`MOVE X/Y/Y1/Y2 [us]`, `DELAY [ms]`, `VACUUM ON/OFF`, `ARM UP/DOWN`, `HOME [axis]`, `MSG [text]`
- 巨集執行控制：▶ 執行、⏸ 暫停、⏹ 停止、單步進度顯示

### 7️⃣ G-code 預覽與控制
- `.gcode` / `.nc` 檔案拖曳與載入
- 2D Canvas 幾何路徑視覺化預覽
- G-code 線上文字編輯器與快速指令（G0, G1, G28, M106, M107）插入
- 編輯器直接執行與進度反饋

### 8️⃣ 3D 實體模擬 (Three.js)
- 3D 實時機械結構運動模擬（基座、X 軸滑台、Y 軸立柱、Z 軸手臂、吸盤）
- 支援滑鼠左鍵旋轉、右鍵平移、滾輪縮放與視角重置
- 與真實硬體馬達位置毫秒級同步

### 9️⃣ 系統終端機 (Web Terminal)
- 🔒 密碼保護解鎖機制
- 完整支援 Linux Bash 指令執行與 `sudo` 權限提升
- 智慧 `cd` 目錄切換與狀態維持
- 內建常用監控快捷晶片（CPU 溫度、服務狀態、日誌、磁碟空間等）
- 支援指令歷史導覽（↑ / ↓ 鍵）與 Ctrl+C 中止

---

## ⌨️ 鍵盤快捷鍵

| 按鍵 | 功能 |
|------|------|
| `←` `→` | X 軸左/右移動 |
| `↑` `↓` | Y 軸上/下移動 |
| `V` | 吸盤切換 (Toggle Vacuum) |
| `Z` | 手臂升降切換 (Toggle Arm Down) |
| `H` | 全軸歸零 (Homing) |

---

## 🔧 系統管理指令

```bash
# 查看服務狀態
sudo systemctl status servo-control

# 查看即時日誌
sudo journalctl -u servo-control -f

# 重啟服務
sudo systemctl restart servo-control

# 停止服務
sudo systemctl stop servo-control

# 關閉開機自動執行
sudo systemctl disable servo-control

# 更新程式（手動方式）
cd /home/pi/servo_control
git pull
sudo systemctl restart servo-control
```

---

## 📁 目錄結構

```
Nice/
├── servo_control/
│   ├── install.sh              ← 🚀 一鍵安裝腳本（含 systemd 開機自啟）
│   ├── start.sh                ← 手動啟動腳本
│   ├── servo_controller.py     ← 後端主程式（Flask + Socket.IO + 步進驅動）
│   ├── requirements.txt        ← Python 依賴清單
│   ├── history_log.json        ← 系統操作歷史記錄
│   ├── macros.json             ← 動作巨集儲存檔
│   ├── README.md               ← 完整使用說明文件
│   ├── templates/
│   │   └── index.html          ← 網頁前端介面（9 大 Tab + 登入遮罩）
│   └── static/
│       ├── css/
│       │   └── style.css       ← 工業深色主題樣式與設計系統
│       └── js/
│           ├── app.js          ← 核心前端邏輯與 WebSocket 通訊
│           └── simulator3d.js  ← Three.js 3D 數位分身渲染引擎
└── 冲压16.pptx                  ← 衝壓系統設計參考規格簡報
```

---

## 🍓 開機自動執行原理

安裝腳本會建立 `/etc/systemd/system/servo-control.service`，使系統在網路就緒後自動啟動控制程式：

- **自動重啟**：若程式崩潰，5 秒後自動重啟
- **依賴服務**：在 `network.target` 和 `pigpiod.service` 之後啟動
- **日誌整合**：可透過 `journalctl -u servo-control` 檢視完整日誌

---

## 📄 License

MIT License — 歡迎自由使用與修改
