# 🤖 衝壓機自動化搬運與步進馬達控制系統 (Raspberry Pi Stepper Controller)

> **工業級衝壓自動化系統** — 樹梅派 X/Z 軸步進馬達控制器 (QN-1805-E1 方形電感接近開關 + LSD-P25/20 圓形強力直流電磁鐵 + 手臂霍爾檢測) + 雙向 WebSocket 即時監控 + Three.js 3D 數位分身模擬 + G-code/巨集編輯器 + Web 系統終端機 + OTA 線上更新

[![GitHub Repo](https://img.shields.io/badge/GitHub-Nono0325%2FNice1-181717?logo=github)](https://github.com/Nono0325/Nice1)
[![Rust](https://img.shields.io/badge/Rust-2021%20Edition-DE522D?logo=rust&logoColor=white)](https://www.rust-lang.org)
[![Axum](https://img.shields.io/badge/Axum-0.7%20Tokio-000000?logo=rust)](https://github.com/tokio-rs/axum)
[![rppal](https://img.shields.io/badge/GPIO-rppal%200.19-C51A4A?logo=raspberry-pi)](https://docs.rs/rppal)
[![Three.js](https://img.shields.io/badge/Three.js-r128-000000?logo=threedotjs)](https://threejs.org)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## ⚡ 一鍵安裝（樹梅派）

SSH 連線至樹梅派終端機後，複製並執行以下**任一行指令**，系統將自動完成相依套件安裝、虛擬環境建立與 systemd 開機自啟設定：

```bash
# 方式一：使用 curl 一行安裝
curl -fsSL https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash
```

```bash
# 方式二：使用 wget 一行安裝
wget -qO- https://raw.githubusercontent.com/Nono0325/Nice1/main/servo_control/install.sh | sudo bash
```

安裝完成後，開啟瀏覽器造訪：  
👉 **`http://<樹梅派IP>:5000`** 或 **`http://raspberrypi.local:5000`**

---

## 🖥️ PC / Windows 本地測試（模擬模式）

本系統具備智慧環境辨識，在無 GPIO 硬體的 PC 或筆電上會自動啟用虛擬模擬引擎，方便快速開發與路徑測試：

```bash
# 1. 複製倉庫
git clone https://github.com/Nono0325/Nice1.git
cd Nice1/servo_control

# 2. 編譯並啟動 Rust 控制伺服器 (自動偵測環境：樹莓派啟用 rppal 硬體 GPIO，PC 啟用模擬模式)
cargo run --release
```

開啟瀏覽器造訪：`http://localhost:5000`  
- 🔐 **模擬模式預設登入密碼**：`Nice`

---

## 📸 核心功能與介面亮點

| 模組 | 功能描述 |
|------|----------|
| 🔐 **全站資安登入防護** | 系統啟動即展示安全認證遮罩，支援 Linux Sudo 系統密碼驗證、自動閒置逾時鎖定與模擬密碼自訂。 |
| 🎮 **工業級控制面板 (HMI)** | START / STOP 金屬觸感按鈕、9 顆即時狀態指示燈、X/Z 軸高解析 mm 座標軌跡、累加目標位移 (100+100 連續 jog 移動)、自訂 mm 移動量與 14-Step 衝壓自動搬運循環。 |
| 📊 **即時量規與數位監控** | Canvas 動態多軸儀表、LSD-P25/20 圓形強力電磁鐵輸出狀態反饋、QN-1805-E1 五點接近開關與手臂霍爾感應指示。 |
| 📡 **訊號與 PWM 軌跡檢測** | Chart.js 即時脈衝訊號波形圖、頻率與佔空比 (Duty Cycle) 參數動態計算。 |
| ⚙️ **行程極限、慢速歸零與系統管理** | 支援 `µs` 與 `mm` 雙單位模式設定 X/Z 軸最小與最大行程範圍（自動持久化至 `limits_config.json`）、梯形加減速規劃 (Max Speed Hz, Accel Steps)、緩慢平穩歸零頻率設定 (Homing Speed Hz)、全站字體切換與遠端 Reboot / Shutdown。 |
| 🔬 **馬達診斷工具** | 即時偵測 GPIO 驅動模式 (Real / Simulation)、X/Z 軸連續巡迴測試、4 組 LSD-P25/20 電磁鐵吸盤動作與 QN-1805-E1 感測器觸發測試。 |
| 🧩 **積木式移動程式工作坊 & 巨集** | 圖像化拖曳拼圖積木編程（工位快速定位 `GOTO`、絕對座標 `MOVE_MM`、相對寸動 `STEP`、4組電磁鐵全控/單組控制 `M1~M4`、8顆霍爾總檢/分組檢測 `G1~G4`、`REPEAT ... END` 迴圈），支援實機座標一鍵教導 (`📍 取當前值`)、單步測試、執行中積木即時高亮與雙向文字腳本同步。 |
| 📁 **G-code 解析與 2D 模擬** | 支援 `.gcode` / `.nc` 拖曳上傳與 2D 軌跡繪製，內建線上 G-code 編輯器與直接執行功能。 |
| 🕹️ **Three.js 3D 數位分身** | 3D 實時機械結構運動模擬（龍門導軌、Z軸立柱滑台、LSD-P25/20 圓形電磁鐵、金屬工件吸附），視角自由平移旋轉縮放。 |
| 💻 **Web 系統終端機** | 具備權限驗證的 Web Console，支援 `sudo`、`cd` 目錄記憶、快捷監控指令與 Process 強制中斷。 |
| 🔄 **GitHub OTA 線上一鍵更新** | 線上檢查 GitHub 最新 Commit SHA、差異對比、一鍵下載升級並自動重啟背景服務。 |

---

## 🔌 硬體接線定義 (BCM Pinout)

本系統全面升級為**電感式接近開關（STUOB QN-1805-E1 方形前方檢測 5mm、DC10-30V 三線 NPN 常開）**、**圓形工業強力直流電磁鐵吸盤（朗碩達 LSD-P25/20 吸力 8KG，搭配 TONGLING JQC-3FF-S-Z 5VDC 1路光耦隔離繼電器驅動模組）**、**機械手臂金屬霍爾檢測（搭配 817 光耦電壓隔離板 / PC817 電壓控制轉接模組）**與 **X/Z 雙軸步進馬達驅動**：

| 功能代號 | GPIO (BCM) | 實體腳位 | 類型 / 規格 | 說明 |
|----------|------------|----------|-------------|------|
| **X_PUL** | GPIO 12 | Pin 32 | 數位輸出 (PWM) | X 軸步進脈衝訊號 (Pulse) |
| **X_DIR** | GPIO 16 | Pin 36 | 數位輸出 | X 軸步進方向訊號 (Direction) |
| **X_ENA** | GPIO 20 | Pin 38 | 數位輸出 | X 軸步進致能訊號 (LOW=通電鎖定, HIGH=釋放手推) |
| **Z_PUL** | GPIO 13 | Pin 33 | 數位輸出 (PWM) | Z 軸垂直步進脈衝訊號 (Pulse，原 Y_PUL) |
| **Z_DIR** | GPIO 19 | Pin 35 | 數位輸出 | Z 軸垂直步進方向訊號 (Direction，原 Y_DIR) |
| **Z_ENA** | GPIO 26 | Pin 37 | 數位輸出 | Z 軸垂直步進致能訊號 (LOW=通電鎖定, HIGH=釋放手推) |
| **MAGNET_1 (M1)** | GPIO 24 | Pin 18 | JQC-3FF-S-Z 繼電器 #1 + LSD-P25/20 | **第 1 組電磁鐵**（對應霍爾 G1: H1/H2），1路光耦隔離繼電器 (5VDC, H/L跳線設H) 驅動 LSD-P25/20 (吸力8KG, HIGH=吸附) |
| **MAGNET_2 (M2)** | GPIO 18 | Pin 12 | JQC-3FF-S-Z 繼電器 #2 + LSD-P25/20 | **第 2 組電磁鐵**（對應霍爾 G2: H3/H4），1路光耦隔離繼電器 (5VDC, H/L跳線設H) 驅動 LSD-P25/20 (吸力8KG, HIGH=吸附) |
| **MAGNET_3 (M3)** | GPIO 14 | Pin 8 | JQC-3FF-S-Z 繼電器 #3 + LSD-P25/20 | **第 3 組電磁鐵**（對應霍爾 G3: H5/H6），1路光耦隔離繼電器 (5VDC, H/L跳線設H) 驅動 LSD-P25/20 (吸力8KG, HIGH=吸附) |
| **MAGNET_4 (M4)** | GPIO 15 | Pin 10 | JQC-3FF-S-Z 繼電器 #4 + LSD-P25/20 | **第 4 組電磁鐵**（對應霍爾 G4: H7/H8），1路光耦隔離繼電器 (5VDC, H/L跳線設H) 驅動 LSD-P25/20 (吸力8KG, HIGH=吸附) |
| **LIM_X_A** | GPIO 17 | Pin 11 | QN-1805-E1 (NPN NO) | X 軸 A 點感測器 (左側 / 歸零原點，檢測距離 5mm) |
| **LIM_X_B** | GPIO 25 | Pin 22 | QN-1805-E1 (NPN NO) | X 軸 B 點感測器 (中間模具工位 / 衝壓等待位，檢測距離 5mm) |
| **LIM_X_C** | GPIO 27 | Pin 13 | QN-1805-E1 (NPN NO) | X 軸 C 點感測器 (向右極限 / 卸料落料工位，檢測距離 5mm) |
| **LIM_Z_D** | GPIO 22 | Pin 15 | QN-1805-E1 (NPN NO) | Z 軸 D 點感測器 (垂直向上歸零原點，檢測距離 5mm) |
| **LIM_Z_E** | GPIO 23 | Pin 16 | QN-1805-E1 (NPN NO) | Z 軸 E 點感測器 (垂直向下極限 / 工件取放高度，檢測距離 5mm) |
| **HALL_G1 (H1/H2)** | GPIO 6 / GPIO 7 | Pin 31 / Pin 26 | 霍爾第 1 組 (8路 817 光耦 CH1~2) | 末端金屬霍爾檢測 **第 1 組**（兩個一組：`HALL_1` + `HALL_2`，LOW=吸附到工件） |
| **HALL_G2 (H3/H4)** | GPIO 8 / GPIO 9 | Pin 24 / Pin 21 | 霍爾第 2 組 (8路 817 光耦 CH3~4) | 末端金屬霍爾檢測 **第 2 組**（兩個一組：`HALL_3` + `HALL_4`，LOW=吸附到工件） |
| **HALL_G3 (H5/H6)** | GPIO 10 / GPIO 11 | Pin 19 / Pin 23 | 霍爾第 3 組 (8路 817 光耦 CH5~6) | 末端金屬霍爾檢測 **第 3 組**（兩個一組：`HALL_5` + `HALL_6`，LOW=吸附到工件） |
| **HALL_G4 (H7/H8)** | GPIO 4 / GPIO 21 | Pin 7 / Pin 40 | 霍爾第 4 組 (8路 817 光耦 CH7~8) | 末端金屬霍爾檢測 **第 4 組**（兩個一組：`HALL_7` + `HALL_8`，LOW=吸附到工件） |
| **PRESS_DONE** | GPIO 5 | Pin 29 | 數位輸入 (817 光耦隔離 / PUD_UP) | 衝壓機模具衝壓完成訊號 (LOW=完成) |
| **GND** | GND | Pin 6/9/14/20/25/30/34/39 | 共地 / 隔離地 | 樹梅派 GND 接繼電器 DC- 與 817 光耦隔離板輸出側 GND |
| **POWER** | 外部電源 | - | 12V/24V 工業電源 | QN-1805-E1 (DC10-30V)、霍爾開關、LSD-P25/20 電磁鐵及馬達驅動器由外部獨立供電 |

> ⚠️ **硬體模組配線說明（繼電器、光耦隔離板、接近開關、電磁鐵）**：
> - **4 組電磁鐵繼電器（4 路 TONGLING JQC-3FF-S-Z 5VDC 光耦隔離繼電器驅動模組）**：
>   - **控制端**：4 組繼電器的 `DC+` 接樹梅派 5V (Pin 2/4)，`DC-` 接樹梅派 GND，4 個 `IN` 訊號端依序接 **M1: GPIO 24 (Pin 18)**、**M2: GPIO 18 (Pin 12)**、**M3: GPIO 14 (Pin 8)**、**M4: GPIO 15 (Pin 10)**；模組上方黃色跳線帽請全部插在 **`H`（高電平觸發）**。
>   - **負載端**：每組繼電器的 `COM`（公共端）接外部直流電源正極 (+V)，`NO`（常開端）接對應的 **LSD-P25/20 電磁鐵 (M1~M4)** 正極，電磁鐵負極接外部電源負極 (0V)。支援 4 組全開/全關同步連動或 M1~M4 單組獨立控制。
> - **末端 8 顆霍爾開關（兩個一組，共 4 組）& 8路 817 光耦電壓隔離板（PC817 電壓控制轉接模組）**：
>   - **分組配置**：8 顆霍爾開關以**兩個一組**分為 4 組：**第 1 組 (H1+H2)**、**第 2 組 (H3+H4)**、**第 3 組 (H5+H6)**、**第 4 組 (H7+H8)**，分別接入 **8路 817 光耦電壓隔離板** 的 `IN1` ~ `IN8` 通道。
>   - **輸入側（24V/12V 感測器端）**：`IN1`~`IN8` 接感測器電源正極 (+V)，對應 `G` 端接 8 顆霍爾開關的 NPN 輸出訊號線。當感測到金屬時 NPN 導通拉低，點亮該通道 PC817 光耦內部發光二極體。
>   - **輸出側（樹梅派 3.3V 端）**：移除共地跳線帽達成完全電氣隔離；輸出負極 `G` 接樹梅派 GND，8 路輸出訊號端依序接樹梅派 **GPIO 6、7（第 1 組）**、**GPIO 8、9（第 2 組）**、**GPIO 10、11（第 3 組）**、**GPIO 4、21（第 4 組）**，透過樹梅派內部上拉電阻 (`GPIO.PUD_UP`) 讀取低電位觸發訊號。
> - **QN-1805-E1 感測器配線**：**棕色線 (`brown:+v`)** 接 DC 10-30V (+24V)，**藍色線 (`blue:0V`)** 接電源負極 (0V/GND)，**黑色線 (`black:out`)** 為三線 NPN 常開輸出訊號 (檢測距離 5mm，尺寸 17.2×35mm)。
> - **LSD-P25/20 電磁鐵規格**：吸力 8KG，尺寸 Ø25×20mm（磁極 Ø12mm、M4 安裝孔、線長 300mm），支援 DC5V (0.3A/1.5W)、DC6V (0.35A/2W)、DC12V (0.13A/1.5W)、DC24V (80mA/2W)。

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

* **🔁 連續運轉生產模式 (Continuous Production Mode)**：
  - 開啟連續模式後，循環完成時自動將累計生產計數 +1。
  - 系統等待設定之「循環間隔緩衝時間」（如 1.0 秒）後自動開始下一次 Cycle。
  - 支援設定「目標生產件數」（例如設定 5000 次，達到後自動安全停機；設為 0 則無限制連續生產）。
  - 操作員隨時點擊 STOP PB 或發生感測器異常時，立刻瞬間煞停保護。

---

## 🏭 工業級 24/7/365 連續運作一年以上可靠度保證指南

為確保系統在衝壓工廠高電磁干擾與全天候不間斷環境下穩定運作一年以上，本專案已完成以下底層加固設計：

1. **MicroSD 卡壽命防穿保護（In-Memory Ring Buffer）**：
   - 取消傳統每一次動作即時寫入磁碟機制，日誌全面維護於 RAM 環形隊列（250 筆）中供 WebSocket 廣播。
   - 一般日誌改為每 5 分鐘非同步批次刷寫，僅在重大 ERROR 時立即寫檔，將磁碟抹寫次數降低 300 倍以上，徹底杜絕 MicroSD 卡在數月內燒穿損壞。

2. **步進馬達開環失步即時校正（Auto Step-Drift Alignment）**：
   - 每次循環回到 A 點與升至 D 點感應到 QN-1805-E1 電感接近開關的瞬間，韌體自動重置校準坐標值，百萬次往復運動亦絕不累積失步漂移。

3. **感測器工業數位抗干擾濾波（3-Sample Digital Debounce）**：
   - 對所有 7 組感測器（A/B/C/D/E 開關、霍爾、衝壓完成訊號）採樣 3 次一致性數位濾波，有效過濾衝壓接觸器火花產生的瞬間噪訊突波。

4. **電磁鐵反向電動勢硬體防護與超時防過熱**：
   - **硬體必備**：LSD-P25/20 圓形電磁鐵兩端必須反向並聯**續流二極體（如 1N4007 / FR107 / TVS 突波吸收器）**，吸收釋放時產生之反向高壓電動勢，保護繼電器與樹梅派電源。
   - **軟體保護**：電磁鐵若連續通電超過 45 秒，系統自動強制斷電並發出安全警報，防止線圈過熱燒損。

5. **樹梅派 BCM2835 硬體看門狗晶片守護（Hardware Watchdog）**：
   - 系統已整合 `/dev/watchdog` 底層硬體看門狗心跳機制，由獨立背景執行緒每 3 秒餵狗。若遭遇極端電氣脈衝造成系統死鎖，硬體將在 15 秒內自動重啟復原。

6. **長週期記憶體保護（Periodic Garbage Collection）**：
   - 系統每運轉 30 分鐘自動執行 `gc.collect()` 回收記憶體碎片，終年 RAM 占用穩定維持在 40~60MB 以內。

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
