# 🤖 樹梅派步進馬達控制系統 (v2.2.0 Industrial Edition)

> **衝壓機自動化搬運系統** — Raspberry Pi X/Z 軸步進馬達控制器 (QN-1805-E1 電感接近開關 + 4組 LSD-P25/20 圓形強力電磁鐵 + 8顆末端霍爾金屬檢測) + 原生 WebSocket 即時網頁監控 + 積木式移動程式工作坊 + G-code 編輯器 + 錯誤日誌輸出中心 + 系統終端機

[![Version](https://img.shields.io/badge/version-v2.2.0-blue?logo=rust)](Cargo.toml)
[![Rust](https://img.shields.io/badge/Rust-2021%20Edition-DE522D?logo=rust&logoColor=white)](https://www.rust-lang.org)
[![Axum](https://img.shields.io/badge/Axum-0.7%20Tokio-000000?logo=rust)](https://github.com/tokio-rs/axum)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)](https://python.org)
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
cargo run --release
```

瀏覽器開啟：`http://localhost:5000`  
- 🔐 模擬模式預設登入密碼：`Nice`

---

## 📸 介面亮點與功能特色

- 🔐 **全站登入防護與安全驗證**：進入系統即啟用安全遮罩，支援密碼登入驗證、自動閒置超時鎖定與密碼變更。
- 🎮 **工業級控制面板（HMI）**：START_PB / STOP_PB 金屬質感按鈕、9 顆狀態指示燈、X/Z 軸可拖曳位置條、累加 jog 微調移動與 14-Step 衝壓循環。
- 🧩 **積木式移動程式工作坊 & 動作巨集**：圖像化拖曳拼圖積木編程（`GOTO`, `MOVE_MM`, `STEP`, `MAGNET 1~4`, `CHECK GROUP 1~4`, `WAIT PRESS`, `REPEAT ... END`），支援一鍵教導當前座標、單步測試與雙向腳本同步。
- 📁 **G-code 解析與即時編輯器**：支援 `.gcode` / `.nc` 檔案上傳與即時 2D 路徑繪製，提供線上 G-code 編輯與快速指令插入。
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
| **圓形電磁鐵 第 1 組 (M1)** | GPIO 24 | Pin 18 | 對應霍爾 G1 (H1/H2)，經 **JQC-3FF-S-Z #1 (5VDC 光耦繼電器，H/L跳線設H)** 驅動 LSD-P25/20 (吸力8KG, HIGH=吸取) |
| **圓形電磁鐵 第 2 組 (M2)** | GPIO 18 | Pin 12 | 對應霍爾 G2 (H3/H4)，經 **JQC-3FF-S-Z #2 (5VDC 光耦繼電器，H/L跳線設H)** 驅動 LSD-P25/20 (吸力8KG, HIGH=吸取) |
| **圓形電磁鐵 第 3 組 (M3)** | GPIO 14 | Pin 8 | 對應霍爾 G3 (H5/H6)，經 **JQC-3FF-S-Z #3 (5VDC 光耦繼電器，H/L跳線設H)** 驅動 LSD-P25/20 (吸力8KG, HIGH=吸取) |
| **圓形電磁鐵 第 4 組 (M4)** | GPIO 15 | Pin 10 | 對應霍爾 G4 (H7/H8)，經 **JQC-3FF-S-Z #4 (5VDC 光耦繼電器，H/L跳線設H)** 驅動 LSD-P25/20 (吸力8KG, HIGH=吸取) |
| **X軸 A點 接近開關** | GPIO 17 | Pin 11 | QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (向左歸零原點，檢測距離 5mm) |
| **X軸 B點 接近開關** | GPIO 25 | Pin 22 | QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (中間模具工位，檢測距離 5mm) |
| **X軸 C點 接近開關** | GPIO 27 | Pin 13 | QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (向右極限/卸料工位，檢測距離 5mm) |
| **Z軸 D點 接近開關** | GPIO 22 | Pin 15 | QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (垂直向上歸零原點，檢測距離 5mm) |
| **Z軸 E點 接近開關** | GPIO 23 | Pin 16 | QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (垂直向下極限/取放高度，檢測距離 5mm) |
| **末端霍爾第 1 組 (G1: H1+H2)** | GPIO 6 / 7 | Pin 31 / 26 | 兩個一組 (H1+H2)，經 **8路 817 光耦電壓隔離板 (CH1~2)** 隔離輸入 (LOW=吸附到金屬工件) |
| **末端霍爾第 2 組 (G2: H3+H4)** | GPIO 8 / 9 | Pin 24 / 21 | 兩個一組 (H3+H4)，經 **8路 817 光耦電壓隔離板 (CH3~4)** 隔離輸入 (LOW=吸附到金屬工件) |
| **末端霍爾第 3 組 (G3: H5+H6)** | GPIO 10 / 11 | Pin 19 / 23 | 兩個一組 (H5+H6)，經 **8路 817 光耦電壓隔離板 (CH5~6)** 隔離輸入 (LOW=吸附到金屬工件) |
| **末端霍爾第 4 組 (G4: H7+H8)** | GPIO 4 / 21 | Pin 7 / 40 | 兩個一組 (H7+H8)，經 **8路 817 光耦電壓隔離板 (CH7~8)** 隔離輸入 (LOW=吸附到金屬工件) |
| **衝壓完成訊號** | GPIO 5 | Pin 29 | 模具衝壓完成觸發，經 817 光耦電壓隔離板輸入 (LOW=完成) |
| **GND / POWER** | GND / 24V | - | 樹梅派 5V/GND 供電給 JQC-3FF-S-Z 繼電器控制端；QN-1805-E1、霍爾開關與 LSD-P25/20 使用外部獨立電源 |

> ⚠️ **硬體模組配線說明**：
> - **4 組 JQC-3FF-S-Z 光耦隔離繼電器 (控制 4 顆 LSD-P25/20 電磁鐵 M1~M4)**：`DC+` 接樹梅派 5V、`DC-` 接 GND、4 組 `IN` 依序接 **GPIO 24 (M1)**、**GPIO 18 (M2)**、**GPIO 14 (M3)**、**GPIO 15 (M4)**，跳線帽全部插在 **`H`（高電平觸發）**；各組輸出端 `COM` 接外部電源正極、`NO` 接對應電磁鐵正極。
> - **8路 817 光耦電壓隔離板 (連接 8 顆末端霍爾開關，兩個一組共 4 組)**：輸入端 `IN1`~`IN8` 接外部 +24V/`+V`、對應 `G` 接 8 顆霍爾開關輸出線；輸出端依序連接樹梅派 **G1 (GPIO 6/7)**、**G2 (GPIO 8/9)**、**G3 (GPIO 10/11)**、**G4 (GPIO 4/21)** 與 GND，安全將 12V/24V 訊號隔離轉換為 3.3V 低電位觸發！

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

## 🎮 網頁介面功能（8 大 Tab）

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
- 數位輸出狀態即時指示（MAGNET LSD-P25/20 / Z_DOWN）
- QN-1805-E1 電感接近開關與手臂霍爾感測器即時反饋
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

### 8️⃣ 系統終端機 (Web Terminal)
- 🔒 密碼保護解鎖機制
- 完整支援 Linux Bash 指令執行與 `sudo` 權限提升
- 智慧 `cd` 目錄切換與狀態維持
- 內建常用監控快捷晶片（CPU 溫度、服務狀態、日誌、磁碟空間等）
- 支援指令歷史導覽（↑ / ↓ 鍵）與 Ctrl+C 中止

### 9️⃣ 錯誤日誌中心 (Error Log Output Center)
- 🚨 集中顯示所有硬體限位、霍爾感測掉件、電磁鐵過熱、馬達超時與前端異常
- 支援 `ALL` / `ERROR` / `WARN` / `UNRESOLVED` 即時篩選與關鍵字搜尋
- 一鍵解除活躍警報、匯出完整 `.log` 診斷報告與手動模擬測試記錄

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
│   ├── Cargo.toml              ← 🦀 Rust 專案設定檔 (Axum + Tokio + rppal)
│   ├── src/                    ← 🦀 Rust 核心原始碼 (main/state/gpio/automation/api)
│   ├── install.sh              ← 🚀 一鍵安裝腳本（含 systemd 開機自啟）
│   ├── start.sh                ← 手動啟動腳本
│   ├── servo_controller.py     ← Python 相容備援主程式
│   ├── requirements.txt        ← Python 依賴清單
│   ├── history_log.json        ← 系統操作歷史記錄
│   ├── macros.json             ← 動作巨集儲存檔
│   ├── README.md               ← 完整使用說明文件
│   ├── templates/
│   │   └── index.html          ← 網頁前端介面（8 大 Tab + 登入遮罩）
│   └── static/
│       ├── css/
│       │   └── style.css       ← 工業深色主題樣式與設計系統
│       └── js/
│           └── app.js          ← 核心前端邏輯、WebSocket 通訊與積木工作坊
└── 冲压16.pptx                  ← 衝壓系統設計參考規格簡報
```

---

## 🍓 開機自動執行原理

安裝腳本會建立 `/etc/systemd/system/servo-control.service`，使系統在網路就緒後自動啟動控制程式：

- **自動重啟**：若程式崩潰，5 秒後自動重啟
- **依賴服務**：在 `network.target` 和 `pigpiod.service` 之後啟動
- **日誌整合**：可透過 `journalctl -u servo-control` 檢視完整日誌

---

## 📜 版本發行紀錄 (Version Changelog)

| 版本 | 發布亮點與變更項目 |
|:---:|---|
| **v2.2.0** *(最新穩定版)* | • **全行程安全歸零機制 (Safe Homing Cycle)**：即便目前座標為 0 亦可自動安全退開後重新尋找物理極限開關與原點。<br>• **微秒級硬體 GPIO 即時防撞聯鎖**：向後移動時觸發霍爾或接近開關立即硬體停止脈衝並強制鎖定坐標至 0.0 mm。<br>• **物理原點校準徹底修復**：霍爾/接近開關歸零觸發後精準重設為 `0.0 mm / 0 µs (0.0%)`。<br>• **閉迴路步進驅動器優化**：適配 57BYG-112 (3.6Nm) 閉迴路步進馬達與數位步進驅動器（細分、方向與電流參數標準化），解決馬達低速抖動與行程縮放問題。<br>• **行程邊界設定**：全面支援 `µs` 與 `mm` 雙單位切換，消弭數值超界與極限錯位問題。 |
| **v2.1.0** | • **即時錯誤日誌輸出中心 (Error Log Center)**：集中式展示即時系統錯誤、警告與操作日誌，支援一鍵匯出與診斷。<br>• **GitHub OTA 自動版本檢查與一鍵更新**：後台自動輪詢與手動檢查遠端更新，在網頁前端彈出版本通知並自動重啟。<br>• **UI 2D 動態雙軸 mm 刻度軌跡**：移除 3D 冗餘負載，專注於工業級高更新率動態軌跡視覺化。 |
| **v2.0.0** | • **Rust 2.0 高效能核心重構**：全面導入 Rust (Axum + Tokio + rppal)，突破 Python GIL 限制。<br>• **8 顆末端霍爾金屬檢測開關**：兩兩一組共 4 組（817 光耦隔離板防突波與硬體抗噪）。<br>• **4 組 LSD-P25/20 圓形強力直流電磁鐵**：JQC-3FF-S-Z 繼電器驅動。 |
| **v1.0.0** | • **初始原型系統**：Python Flask + Socket.IO + pigpio/RPi.GPIO 步進馬達控制系統。 |

---

## 📄 License

MIT License — 歡迎自由使用與修改
