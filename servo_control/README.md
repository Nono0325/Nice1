# 🤖 樹梅派步進馬達控制系統 (Raspberry Pi Stepper Motor Controller)

> **衝壓機自動化搬運系統** — Raspberry Pi X/Z 軸步進馬達控制器 (HN-1805-E1 電感接近開關 + 圓形強力電磁鐵 + 手臂霍爾金屬檢測) + 即時網頁監控 + 3D 數位分身模擬 + G-code/巨集編輯器 + 系統終端機

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
curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash

# 方式二：wget
wget -qO- https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash
```

安裝完成後，**瀏覽器開啟** `http://<樹梅派IP>:5000` 即可使用。

---

## 🖥️ Windows / PC 本地測試（模擬模式）

```bash
git clone https://github.com/Nono0325/Nice1.git
cd Nice1/servo_control
pip install -r requirements.txt
python servo_controller.py
```

瀏覽器開啟：`http://localhost:5000`  
- 🔐 模擬模式預設登入密碼：`Nice`

---

## 📸 介面亮點與功能特色

- 🔐 **全站登入防護與安全驗證**：進入系統即啟用安全遮罩，支援密碼登入驗證、自動閒置超時鎖定與密碼變更。
- 🎮 **工業級控制面板（HMI）**：START_PB / STOP_PB 金屬質感按鈕、9 顆狀態指示燈、X/Z 軸可拖曳位置條、累加 jog 微調移動與 14-Step 衝壓循環。
- 🤖 **動作巨集管理器**：可自訂動作腳本指令（`MOVE`, `DELAY`, `MAGNET`, `CHECK METAL`, `WAIT PRESS`, `HOME`, `MSG`），支援執行、暫停與腳本儲存。
- 📁 **G-code 解析與即時編輯器**：支援 `.gcode` / `.nc` 檔案上傳與即時 2D 路徑繪製，提供線上 G-code 編輯與快速指令插入。
- 🕹️ **3D 實時機械模擬（Three.js）**：3D 數位分身動態聯動真實馬達位置（圓形電磁鐵吸取金屬工件、Z 軸垂直升降）。
- 💻 **Web 系統終端機**：受密碼保護的 Web Console，支援 `sudo`、`cd` 目錄切換、快捷常用系統監控指令與即時輸出。
- 🔄 **GitHub 線上 OTA 自動更新**：一鍵檢查遠端最新 Commit 並透過 Git 同步更新，樹梅派環境下自動重啟服務。
- 🎨 **介面字體與外觀自訂**：內建 Inter、Roboto、Outfit、JetBrains Mono、Fira Code 等字體與字型大小即時調節。

---

## 🔌 硬體接線（BCM 編號）- 電感式感測器與步進配置

| 功能 | GPIO | 實體腳位 | 說明 |
|------|------|----------|------|
| **X_PUL 步進脈衝** | GPIO 12 | Pin 32 | X 軸步進馬達脈衝輸入 |
| **X_DIR 步進方向** | GPIO 16 | Pin 36 | X 軸步進馬達方向輸入 |
| **X_ENA 步進致能** | GPIO 20 | Pin 38 | X 軸步進致能（LOW=啟動, HIGH=釋放）|
| **Z_PUL 步進脈衝** | GPIO 13 | Pin 33 | Z 軸垂直步進脈衝輸入 (原 Y_PUL) |
| **Z_DIR 步進方向** | GPIO 19 | Pin 35 | Z 軸垂直步進方向輸入 (原 Y_DIR) |
| **Z_ENA 步進致能** | GPIO 26 | Pin 37 | Z 軸垂直步進致能（LOW=啟動, HIGH=釋放）|
| **圓形電磁鐵吸盤** | GPIO 24 | Pin 18 | 直流 DC 5V/6V/12V/24V 繼電器/MOSFET 控制 (HIGH=吸取) |
| **X軸 A點 接近開關** | GPIO 17 | Pin 11 | HN-1805-E1 方形 24V NPN 常開 (向左歸零原點) |
| **X軸 B點 接近開關** | GPIO 25 | Pin 22 | HN-1805-E1 方形 24V NPN 常開 (中間模具工位) |
| **X軸 C點 接近開關** | GPIO 27 | Pin 13 | HN-1805-E1 方形 24V NPN 常開 (向右極限/卸料工位) |
| **Z軸 D點 接近開關** | GPIO 22 | Pin 15 | HN-1805-E1 方形 24V NPN 常開 (垂直向上歸零原點) |
| **Z軸 E點 接近開關** | GPIO 23 | Pin 16 | HN-1805-E1 方形 24V NPN 常開 (垂直向下極限/取放高度) |
| **手臂金屬霍爾感測** | GPIO 6 | Pin 31 | 手臂金屬在位檢測 (LOW=吸附到金屬工件) |
| **衝壓完成訊號** | GPIO 5 | Pin 29 | 模具衝壓完成觸發 (LOW=完成) |
| **GND / POWER** | GND / 24V | - | 建議 HN-1805-E1 與馬達使用獨立 24V 電源，Pi 與 24V 電源必須共地 (GND) |

> ⚠️ **HN-1805-E1 配線說明**：棕色接 +24V，藍色接 0V/GND，黑色為 NPN 常開信號輸出（金屬接近時導通至 0V）。因為樹梅派 GPIO 為 3.3V 電位，請搭配光耦 (PC817) 或分壓電路隔離保護！

---

## 📐 位置與行程單位說明

| 參數 | 數值 |
|------|------|
| 位置表示方式 | **虛擬 µs 單元** (500 ~ 2500)，對應步進馬達行程 |
| 最小位置（A點原點 / E點下限） | **500 µs** = 行程 0% |
| 中心位置（B點模具位 / 50% 避讓高度） | **1500 µs** = 行程 50% |
| 最大位置（C點右限 / D點上歸零） | **2500 µs** = 行程 100% |
| 比例換算 | 預設 10 µs = 1 mm（可於範圍設定頁動態校準）|

---

## 🎮 網頁介面功能（9 大 Tab）

### 1️⃣ 控制面板
- **START_PB / STOP_PB** 工業金屬觸感按鈕
- 9 顆狀態指示燈：`Cycle Active` / `Electromagnet` / `Arm Metal Hall` / `Lim X-a` / `Lim X-b` / `Lim X-c` / `Lim Z-d` / `Lim Z-e` / `Error`
- **X 軸 / Z 軸位置條**：可拖曳滑桿、支援 mm 與百分比雙重即時顯示
- 方向按鈕 + 步進值選擇（10 / 50 / 100 / 200 µs 或 mm 自訂數值輸入）
- 圓形電磁鐵通斷切換按鈕
- 🔄 **14-Step 衝壓自動搬運循環**：X 回 A ➔ Z 降 E ➔ 電磁鐵通電 ➔ Z 升 D ➔ 霍爾驗證 ➔ X 移 C ➔ Z 降 E ➔ 電磁鐵斷電 ➔ Z 升 D ➔ X 移 B ➔ Z 降 50% ➔ 等待衝壓完 ➔ Z 升 D ➔ X 回 A
- 🔁 **24/7/365 連續生產運轉機制**：
  - 單次循環 / 連續循環一鍵切換
  - 累計生產次數計數器與目標件數自動停機設定
  - 每循環回到 A 點與升至 D 點接近開關時，韌體自動校準坐標消除開環步進累積失步
  - 記憶體環形緩衝日誌（RAM Buffer），消除 MicroSD 卡頻繁寫入磨損（Flash Wear-Out）
  - 3-Sample 數位抗干擾濾波與電磁鐵 45 秒超時防燒毀保護
  - 整合樹梅派 BCM2835 硬體看門狗晶片（/dev/watchdog）心跳守護

### 2️⃣ 即時監控
- X / Z 雙軸動態儀表量規（Canvas 即時繪製）
- 系統累計開機運轉時間（Uptime）與硬體看門狗運作狀態
- 數位輸出狀態即時指示（MAGNET / Z_DOWN）
- HN-1805-E1 電感接近開關與手臂霍爾感測器即時反饋
- 系統即時操作與歷史日誌

### 3️⃣ 訊號檢測
- 即時 **PWM 波形圖**（X 軸 + Z 軸分軌即時 Chart.js 繪製）
- Pulse Width（µs）+ Duty Cycle（%）即時參數表
- PWM 參數視覺化說明

### 4️⃣ 範圍設定與系統管理
- X 軸與 Z 軸最小/最大行程設定（µs 與 mm 滑桿）
- 速度規劃與梯形加減速參數（Max Speed Hz, Accel Steps, Scale）
- 外觀與字體選擇（Inter / Roboto / Outfit / JetBrains Mono / Fira Code）及基礎字級調節
- 自動鎖定時間（1~480 分鐘）與模擬模式密碼修改
- 樹梅派系統管理：遠端重新啟動 (Reboot) / 關機 (Shutdown)
- 🔄 **GitHub 系統更新**：線上檢查版本、差異對比與一鍵升級
- GPIO 腳位完整對照表

### 5️⃣ 馬達診斷
- GPIO 模式偵測（Raspberry Pi Real GPIO / Simulation Mode）
- 馬達連續掃描測試（X / Z 軸）
- 電磁鐵吸盤動作與感測器觸發測試
- 即時診斷日誌輸出

### 6️⃣ 動作巨集
- 巨集腳本動態建立、儲存與刪除
- 動作指令支援：`MOVE X/Z [us/mm]`, `DELAY [ms]`, `MAGNET ON/OFF`, `CHECK METAL`, `WAIT PRESS`, `HOME [axis]`, `MSG [text]`
- 巨集執行控制：▶ 執行、⏸ 暫停、⏹ 停止、單步進度顯示

### 7️⃣ G-code 預覽與控制
- `.gcode` / `.nc` 檔案拖曳與載入
- 2D Canvas 幾何路徑視覺化預覽
- G-code 線上文字編輯器與快速指令（G0, G1, G28, M106, M107）插入
- 編輯器直接執行與進度反饋

### 8️⃣ 3D 實體模擬 (Three.js)
- 3D 實時機械結構運動模擬（龍門導軌、Z 軸立柱、圓形電磁鐵吸盤、工件金屬吸附）
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

| 按鍵 | 功能說明 |
|:---:|---|
| `←` / `→` | X 軸向左 (A 點原點) / 向右 (C 點極限) 微調移動 |
| `↑` / `↓` | Z 軸垂直向上 (D 點原點) / 向下 (E 點極限) 微調移動 |
| `M` / `V` | 圓形強力直流電磁鐵吸盤切換 (通電吸料 ON / 斷電釋放 OFF) |
| `Z` / `z` | 手臂升降輔助切換 (Toggle Arm Up/Down) |
| `H` / `h` | 全軸高速並行歸零 (X 軸回 A 點，Z 軸向上回 D 點) |

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
