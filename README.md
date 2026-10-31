# 🤖 衝壓機自動化搬運與步進馬達控制系統 (Raspberry Pi Stepper Controller)

> **工業級衝壓自動化系統** — 樹梅派 X/Z 軸步進馬達控制器 (HN-1805-E1 方形電感接近開關 + 圓形強力直流電磁鐵 + 手臂霍爾檢測) + 雙向 WebSocket 即時監控 + Three.js 3D 數位分身模擬 + G-code/巨集編輯器 + Web 系統終端機 + OTA 線上更新

[![GitHub Repo](https://img.shields.io/badge/GitHub-Nono0325%2FNice1-181717?logo=github)](https://github.com/Nono0325/Nice1)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://python.org)
[![Flask](https://img.shields.io/badge/Flask-3.x-000000?logo=flask)](https://flask.palletsprojects.com)
[![Socket.IO](https://img.shields.io/badge/Socket.IO-4.x-010101?logo=socketdotio)](https://socket.io)
[![Three.js](https://img.shields.io/badge/Three.js-r128-000000?logo=threedotjs)](https://threejs.org)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-GPIO%20%2F%20pigpio-C51A4A?logo=raspberry-pi)](https://www.raspberrypi.org)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## ⚡ 一鍵安裝（樹梅派）

SSH 連線至樹梅派終端機後，複製並執行以下**任一行指令**，系統將自動完成相依套件安裝、虛擬環境建立與 systemd 開機自啟設定：

```bash
# 方式一：使用 curl 一行安裝
curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash
```

```bash
# 方式二：使用 wget 一行安裝
wget -qO- https://raw.githubusercontent.com/Nono0325/Nice/main/servo_control/install.sh | sudo bash
```

安裝完成後，開啟瀏覽器造訪：  
👉 **`http://<樹梅派IP>:5000`** 或 **`http://raspberrypi.local:5000`**

---

## 🖥️ PC / Windows 本地測試（模擬模式）

本系統具備智慧環境辨識，在無 GPIO 硬體的 PC 或筆電上會自動啟用虛擬模擬引擎，方便快速開發與路徑測試：

```bash
# 1. 複製倉庫
git clone https://github.com/Nono0325/Nice.git
cd Nice/servo_control

# 2. 安裝相依套件
pip install -r requirements.txt

# 3. 啟動主程式
python servo_controller.py
```

開啟瀏覽器造訪：`http://localhost:5000`  
- 🔐 **模擬模式預設登入密碼**：`Nice`

---

## 📸 核心功能與介面亮點

| 模組 | 功能描述 |
|------|----------|
| 🔐 **全站資安登入防護** | 系統啟動即展示安全認證遮罩，支援 Linux Sudo 系統密碼驗證、自動閒置逾時鎖定與模擬密碼自訂。 |
| 🎮 **工業級控制面板 (HMI)** | START / STOP 金屬觸感按鈕、9 顆即時狀態指示燈、X/Z 軸高解析 mm 座標軌跡、累加目標位移 (100+100 連續 jog 移動)、自訂 mm 移動量與 14-Step 衝壓自動搬運循環。 |
| 📊 **即時量規與數位監控** | Canvas 動態多軸儀表、圓形強力電磁鐵輸出狀態反饋、HN-1805-E1 五點接近開關與手臂霍爾感應指示。 |
| 📡 **訊號與 PWM 軌跡檢測** | Chart.js 即時脈衝訊號波形圖、頻率與佔空比 (Duty Cycle) 參數動態計算。 |
| ⚙️ **行程極限、慢速歸零與系統管理** | 支援 mm 單位行程極限設定 (0.0~200.0 mm)、梯形加減速規劃 (Max Speed Hz, Accel Steps)、緩慢平穩歸零頻率設定 (Homing Speed Hz)、全站字體切換與遠端 Reboot / Shutdown。 |
| 🔬 **馬達診斷工具** | 即時偵測 GPIO 驅動模式 (Real / Simulation)、X/Z 軸連續巡迴測試、電磁鐵吸盤動作與感測器觸發測試。 |
| 🤖 **動作巨集管理器** | 自訂腳本指令庫 (`MOVE`, `DELAY`, `MAGNET`, `CHECK METAL`, `WAIT PRESS`, `HOME`, `MSG`)，支援執行、暫停、繼續與單步追蹤。 |
| 📁 **G-code 解析與 2D 模擬** | 支援 `.gcode` / `.nc` 拖曳上傳與 2D 軌跡繪製，內建線上 G-code 編輯器與直接執行功能。 |
| 🕹️ **Three.js 3D 數位分身** | 3D 實時機械結構運動模擬（龍門導軌、Z軸立柱滑台、圓形電磁鐵、金屬工件吸附），視角自由平移旋轉縮放。 |
| 💻 **Web 系統終端機** | 具備權限驗證的 Web Console，支援 `sudo`、`cd` 目錄記憶、快捷監控指令與 Process 強制中斷。 |
| 🔄 **GitHub OTA 線上一鍵更新** | 線上檢查 GitHub 最新 Commit SHA、差異對比、一鍵下載升級並自動重啟背景服務。 |

---

## 🔌 硬體接線定義 (BCM Pinout)

本系統全面升級為**電感式接近開關（HN-1805-E1 方形 24V NPN 常開）**、**圓形工業強力直流電磁鐵吸盤**、**機械手臂金屬霍爾檢測**與 **X/Z 雙軸步進馬達驅動**：

| 功能代號 | GPIO (BCM) | 實體腳位 | 類型 / 規格 | 說明 |
|----------|------------|----------|-------------|------|
| **X_PUL** | GPIO 12 | Pin 32 | 數位輸出 (PWM) | X 軸步進脈衝訊號 (Pulse) |
| **X_DIR** | GPIO 16 | Pin 36 | 數位輸出 | X 軸步進方向訊號 (Direction) |
| **X_ENA** | GPIO 20 | Pin 38 | 數位輸出 | X 軸步進致能訊號 (LOW=通電鎖定, HIGH=釋放手推) |
| **Z_PUL** | GPIO 13 | Pin 33 | 數位輸出 (PWM) | Z 軸垂直步進脈衝訊號 (Pulse，原 Y_PUL) |
| **Z_DIR** | GPIO 19 | Pin 35 | 數位輸出 | Z 軸垂直步進方向訊號 (Direction，原 Y_DIR) |
| **Z_ENA** | GPIO 26 | Pin 37 | 數位輸出 | Z 軸垂直步進致能訊號 (LOW=通電鎖定, HIGH=釋放手推) |
| **MAGNET** | GPIO 24 | Pin 18 | 繼電器 / MOSFET | 圓形強力直流電磁鐵 (DC 5V/6V/12V/24V，HIGH=通電吸附) |
| **LIM_X_A** | GPIO 17 | Pin 11 | HN-1805-E1 (NPN NO) | X 軸 A 點感測器 (左側 / 歸零原點) |
| **LIM_X_B** | GPIO 25 | Pin 22 | HN-1805-E1 (NPN NO) | X 軸 B 點感測器 (中間模具工位 / 衝壓等待位) |
| **LIM_X_C** | GPIO 27 | Pin 13 | HN-1805-E1 (NPN NO) | X 軸 C 點感測器 (向右極限 / 卸料落料工位) |
| **LIM_Z_D** | GPIO 22 | Pin 15 | HN-1805-E1 (NPN NO) | Z 軸 D 點感測器 (垂直向上歸零原點) |
| **LIM_Z_E** | GPIO 23 | Pin 16 | HN-1805-E1 (NPN NO) | Z 軸 E 點感測器 (垂直向下極限 / 工件取放高度) |
| **HALL_ARM** | GPIO 6 | Pin 31 | 霍爾感測器 (HALL) | 手臂末端金屬檢測 (檢測工件是否被吸住) |
| **PRESS_DONE** | GPIO 5 | Pin 29 | 數位輸入 (PUD_UP) | 衝壓機模具衝壓完成訊號 (LOW=完成) |
| **GND** | GND | Pin 6/9/14/20/25/30/34/39 | 共地 | 樹梅派 GND 與 24V 電源負極必須共地 |
| **POWER** | 外部電源 | - | 12V/24V 工業電源 | HN-1805-E1 及馬達驅動器必須由外部 24V 獨立供電 |

> ⚠️ **HN-1805-E1 24V NPN 常開接線注意事項**：
> - 感測器配線：**棕色線 (Brown)** 接 +24V，**藍色線 (Blue)** 接 24V 負極 (GND)，**黑色線 (Black)** 為 NPN 開集極輸出訊號 (Signal Out)。
> - NPN 常開 (NO) 感測器未感測到金屬時黑色訊號線浮接；感測到金屬時內部電晶體導通拉低至 GND (0V)。
> - **安全防護**：樹梅派 GPIO 最大承受 3.3V 電壓。建議在黑色訊號線與 GPIO 間加裝**光耦隔離模組 (如 PC817)** 或分壓電阻，並透過樹梅派內部上拉電阻 (`GPIO.PUD_UP`) 讀取低電位觸發訊號。

---

## 🔄 14-Step 衝壓搬運自動化運動模式

系統內建全自動衝壓上下料 14 步循環週期：
1. **X 軸回到 A 點**（向左歸零原點）
2. **Z 軸下降**（向下至 E 點感測器位置）
3. **電磁鐵通電**（吸盤產生強磁吸取金屬板料）
4. **Z 軸上升**（垂直回到頂部 D 點歸零位）
5. **手臂霍爾感測**（即時檢驗金屬工件是否確實吸附，防掉料保護）
6. **X 軸橫移至 C 點**（向右移動至卸料工位）
7. **Z 軸下降**（向下至 E 點）
8. **電磁鐵斷電**（退磁釋放金屬工件至模具/落料槽）
9. **Z 軸上升**（回到頂部 D 點安全高度）
10. **X 軸橫移至 B 點**（移動至衝壓模具中心工位）
11. **Z 軸下降一半 (50%)**（預備與避位等待）
12. **等待模具衝壓完成**（監聽 `PRESS_DONE` 外部觸發訊號）
13. **Z 軸上升**（回到頂部 D 點）
14. **X 軸回到 A 點**（完成一個完整循環 Cycle）

---

## 📁 專案目錄結構

```text
Nice/
├── .gitignore                  ← 全域 Git 忽略清單
├── LICENSE                     ← MIT 開源授權條款
├── README.md                   ← 專案總說明文件 (本檔)
├── 冲压16.pptx                  ← 衝壓系統機構設計與規格簡報
└── servo_control/
    ├── install.sh              ← 🚀 樹梅派一鍵安裝與自啟配置腳本
    ├── start.sh                ← 手動前台啟動腳本
    ├── servo_controller.py     ← 後端主程式 (Flask + Socket.IO + 步進驅動引擎)
    ├── requirements.txt        ← Python 相依套件清單
    ├── history_log.json        ← 系統歷史操作與診斷日誌
    ├── macros.json             ← 動作巨集指令設定檔
    ├── README.md               ← 控制模組詳細說明文件
    ├── templates/
    │   └── index.html          ← 前端響應式介面 (9 大 Tab + 資安登入遮罩)
    └── static/
        ├── css/
        │   └── style.css       ← 工業深色主題設計系統樣式
        └── js/
            ├── app.js          ← 核心前端邏輯、WebSocket 通訊與事件處理
            └── simulator3d.js  ← Three.js 3D 實時機械模擬渲染引擎
```

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

## 🔧 樹梅派 systemd 系統服務管理

```bash
# 查看服務當前運行狀態
sudo systemctl status servo-control

# 檢視即時系統與馬達運作日誌
sudo journalctl -u servo-control -f

# 重新啟動控制服務
sudo systemctl restart servo-control

# 停止控制服務
sudo systemctl stop servo-control

# 關閉開機自動執行
sudo systemctl disable servo-control
```

---

## 📄 授權條款

本專案採用 [MIT License](LICENSE) 授權開源。
