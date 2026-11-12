#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
衝壓機自動化搬運與步進馬達控制系統
Raspberry Pi Stepper Motor & Electromagnet Controller
=====================================================
硬體定義與感測器配置：
1. X 軸: 步進馬達 (PUL: GPIO 12, DIR: GPIO 16, ENA: GPIO 20)
   - a 點 (左側歸零): 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (GPIO 17)
   - b 點 (中間模具工位): 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (GPIO 25)
   - c 點 (向右極限/放料): 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (GPIO 27)
2. Z 軸 (垂直升降，原 Y 軸改為 Z 軸): 步進馬達 (PUL: GPIO 13, DIR: GPIO 19, ENA: GPIO 26)
   - d 點 (向上歸零/安全高度): 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (GPIO 22)
   - e 點 (向下極限/取放深度): 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (GPIO 23)
3. 末端手臂:
   - 圓形電磁鐵強吸力吸盤 LSD-P25/20 共 4 組 (吸力 8KG, Ø25×20mm, 經 4 組 TONGLING JQC-3FF-S-Z 5VDC 1路光耦隔離繼電器驅動 GPIO 24, 18, 14, 15)
   - 末端金屬吸附檢測霍爾感測陣列：共 8 顆霍爾開關，兩個一組（共 4 組：G1[H1,H2]、G2[H3,H4]、G3[H5,H6]、G4[H7,H8] 對應 4 組電磁鐵 M1~M4），經 8路 817 光耦電壓隔離板 (PC817 轉接模組) 輸入 GPIO 6, 7, 8, 9, 10, 11, 4, 21
4. 模具衝壓交握:
   - 衝壓機衝壓完成回授訊號 (GPIO 5，817 光耦電壓隔離板輸入)
"""

import os
import sys
import time
import math
import threading
import json
import subprocess
import secrets
import gc
from collections import deque
from datetime import datetime

# ── Flask / SocketIO ──────────────────────────────────────────────
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'servo_control_secret_2024'
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading',
    ping_timeout=60,
    ping_interval=20,
    max_http_buffer_size=1000000
)

APP_VERSION = '2.2.1'

# ── GPIO 模式偵測（非 Pi 環境使用 Mock） ───────────────────────────
IS_RASPBERRY_PI = False
try:
    import RPi.GPIO as GPIO
    # 先 cleanup() 清除前次程序殘留的 GPIO 狀態，避免重啟後 add_event_detect 失敗
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    IS_RASPBERRY_PI = True
    print("[INFO] Running on Raspberry Pi - Real GPIO mode")
except (ImportError, RuntimeError):
    print("[INFO] Running in Simulation mode (not Raspberry Pi)")

# ── GPIO 引腳定義 (BCM 編號 - 工業級配置) ──────────────────────────
PIN = {
    # X 軸步進馬達
    'X_PUL':          12,   # X 軸脈衝 (Row 3 - IO12)
    'X_DIR':          16,   # X 軸方向 (Row 2 - IO16)
    'X_ENA':          20,   # X 軸致能 (Row 3 - IO20)

    # Z 軸步進馬達 (由原 Y 軸全面升級為垂直 Z 軸)
    'Z_PUL':          13,   # Z 軸脈衝 (Row 4 - IO13)
    'Z_DIR':          19,   # Z 軸方向 (Row 3 - IO19)
    'Z_ENA':          26,   # Z 軸致能 (Row 4 - IO26)
    # 向下相容 Y 軸別名
    'Y_PUL':          13,
    'Y_DIR':          19,
    'Y_ENA':          26,

    # 圓形電磁鐵強吸力吸盤 LSD-P25/20 共 4 組 (經 TONGLING JQC-3FF-S-Z 5VDC 1路光耦隔離繼電器驅動，H/L跳線設為H高電平觸發)
    'MAGNET_1':       24,   # 第 1 組電磁鐵 M1 (Pin 18 - GPIO 24，對應霍爾 G1: H1+H2)
    'MAGNET_2':       18,   # 第 2 組電磁鐵 M2 (Pin 12 - GPIO 18，對應霍爾 G2: H3+H4)
    'MAGNET_3':       14,   # 第 3 組電磁鐵 M3 (Pin 8  - GPIO 14，對應霍爾 G3: H5+H6)
    'MAGNET_4':       15,   # 第 4 組電磁鐵 M4 (Pin 10 - GPIO 15，對應霍爾 G4: H7+H8)
    'MAGNET':         24,   # 向下相容主別名 (全組同步或 M1)
    'VACUUM':         24,   # 向下相容別名
    'Z_DOWN':         25,   # 保留舊手臂繼電器標籤兼容

    # 電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (經 817 光耦電壓隔離板轉接 -> 樹莓派內部上拉 PUD_UP 讀取為 LOW)
    'LIM_X_A':        17,   # a 點: X 軸歸零/左極限 (Row 1 - IO17)
    'LIM_X_B':        25,   # b 點: X 軸中間模具工位 (Row 2 - IO25) [新增接近開關]
    'LIM_X_C':        27,   # c 點: X 軸向右極限 (Row 4 - IO27)
    'LIM_Z_D':        22,   # d 點: Z 軸向上歸零 (Row 3 - IO22)
    'LIM_Z_E':        23,   # e 點: Z 軸向下極限 (Row 3 - IO23)

    # 舊版限位名稱相容映射
    'LIM_LEFT':       17,
    'LIM_MID':        25,
    'LIM_RIGHT':      27,
    'LIM_UP':         22,
    'LIM_DOWN':       23,

    # 末端金屬檢測霍爾感測陣列：8 顆霍爾開關，兩個一組 (共 4 組，搭配 8路 817 光耦電壓隔離板 / PC817 轉接模組)
    # 第 1 組 (Group 1)
    'HALL_1':         6,    # G1-1: 手臂霍爾 #1 (Pin 31 - GPIO 6)
    'HALL_2':         7,    # G1-2: 手臂霍爾 #2 (Pin 26 - GPIO 7)
    # 第 2 組 (Group 2)
    'HALL_3':         8,    # G2-1: 手臂霍爾 #3 (Pin 24 - GPIO 8)
    'HALL_4':         9,    # G2-2: 手臂霍爾 #4 (Pin 21 - GPIO 9)
    # 第 3 組 (Group 3)
    'HALL_5':         10,   # G3-1: 手臂霍爾 #5 (Pin 19 - GPIO 10)
    'HALL_6':         11,   # G3-2: 手臂霍爾 #6 (Pin 23 - GPIO 11)
    # 第 4 組 (Group 4)
    'HALL_7':         4,    # G4-1: 手臂霍爾 #7 (Pin 7  - GPIO 4)
    'HALL_8':         21,   # G4-2: 手臂霍爾 #8 (Pin 40 - GPIO 21)
    'HALL_ARM_METAL': 6,    # 向下相容主別名 (對應 HALL_1)
    'PRESS_DONE':     5,    # 衝壓模具衝壓完成回授訊號 (Pin 29 - IO5)
}

HALL_PIN_KEYS = ['HALL_1', 'HALL_2', 'HALL_3', 'HALL_4', 'HALL_5', 'HALL_6', 'HALL_7', 'HALL_8']
MAGNET_PIN_KEYS = ['MAGNET_1', 'MAGNET_2', 'MAGNET_3', 'MAGNET_4']

# ── 步進馬達虛擬 PWM 規格 ──────────────────────────────────────────
PWM_FREQ     = 50       
PULSE_MIN    = 500      
PULSE_MAX    = 2500     
PULSE_CENTER = 1500     
SYSTEM_PULSE_MAX = 200000  # 支援長行程步進馬達上限 (最高 200,000 µs)

def pulse_to_duty(pulse_us):
    """將脈衝寬度(µs)轉換成對應的 duty cycle，用於網頁顯示"""
    period_us = 1_000_000 / PWM_FREQ   # = 20000 µs
    return (pulse_us / period_us) * 100.0

# ── 系統狀態 ──────────────────────────────────────────────────────
state = {
    # 馬達當前位置 (對應 500~2500 µs)
    'x_pulse':  PULSE_CENTER,
    'z_pulse':  PULSE_CENTER,   # 升級 Z 軸
    'y1_pulse': PULSE_CENTER,   # 向下相容
    'y2_pulse': PULSE_CENTER,   # 向下相容

    # 物理座標 mm
    'x_mm': 100.0,
    'z_mm': 100.0,   # 升級 Z 軸
    'y_mm': 100.0,   # 向下相容

    # 轉換成百分比 0~100 (0=LEFT/DOWN, 100=RIGHT/UP)
    'x_pct':  50.0,
    'z_pct':  50.0,
    'y_pct':  50.0,

    # 數位輸出 (圓形電磁鐵強吸力吸盤 LSD-P25/20 共 4 組)
    'magnet':   False,
    'magnet_1': False,                 # 第 1 組電磁鐵 (M1)
    'magnet_2': False,                 # 第 2 組電磁鐵 (M2)
    'magnet_3': False,                 # 第 3 組電磁鐵 (M3)
    'magnet_4': False,                 # 第 4 組電磁鐵 (M4)
    'magnet_active_count': 0,          # 通電電磁鐵組數 (0~4)
    'vacuum':   False,                 # 相容舊版名稱
    'z_down':   False,

    # 數位輸入（電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 支援 NO/NC）
    'lim_x_a':        False,  # a 點: X 軸歸零 (原 lim_left)
    'lim_x_b':        False,  # b 點: X 軸中間工位 [新增]
    'lim_x_c':        False,  # c 點: X 軸向右極限 (原 lim_right)
    'lim_z_d':        False,  # d 點: Z 軸向上歸零 (原 lim_up)
    'lim_z_e':        False,  # e 點: Z 軸向下極限 (原 lim_down)
    'lim_x_a_active_high': False,
    'lim_x_b_active_high': False,
    'lim_x_c_active_high': True,  # GPIO 27 (c點): NC/Active-HIGH
    'lim_z_d_active_high': True,  # GPIO 22 (d點): NC/Active-HIGH
    'lim_z_e_active_high': False,
    # 末端金屬檢測霍爾感測陣列 (8 顆，兩個一組共 4 組)
    'hall_1': False, 'hall_2': False,  # 第 1 組 (G1)
    'hall_3': False, 'hall_4': False,  # 第 2 組 (G2)
    'hall_5': False, 'hall_6': False,  # 第 3 組 (G3)
    'hall_7': False, 'hall_8': False,  # 第 4 組 (G4)
    'hall_group_1':   False,           # 第 1 組狀態 (H1 & H2)
    'hall_group_2':   False,           # 第 2 組狀態 (H3 & H4)
    'hall_group_3':   False,           # 第 3 組狀態 (H5 & H6)
    'hall_group_4':   False,           # 第 4 組狀態 (H7 & H8)
    'hall_active_count': 0,            # 觸發顆數 (0~8)
    'hall_group_count':  0,            # 觸發組數 (0~4)
    'hall_arm_metal': False,           # 綜合判定是否有金屬工件被吸住
    'press_done':     False,           # 模具衝壓完成訊號

    # 舊有名稱相容鏡像
    'lim_left':  False,
    'lim_mid':   False,
    'lim_right': False,
    'lim_up':    False,
    'lim_down':  False,
    'hall_metal': False,
    'hall_active_high': True,

    # 系統狀態
    'system_run':    False,
    'arm_down':      False,
    'pickup_active': False,
    'error':         False,
    'error_msg':     '',

    # 範圍與物理比例設定 (µs 與 mm)
    'x_min': PULSE_MIN,
    'x_max': PULSE_MAX,
    'z_min': PULSE_MIN,
    'z_max': PULSE_MAX,
    'y_min': PULSE_MIN,  # 相容
    'y_max': PULSE_MAX,  # 相容
    'x_min_mm': 0.0,
    'x_max_mm': 300.0,
    'z_min_mm': 0.0,
    'z_max_mm': 200.0,
    'y_min_mm': 0.0,
    'y_max_mm': 200.0,
    'x_scale': 10.0,
    'z_scale': 10.0,
    'y_scale': 10.0,

    # 57BYG-112 傳動比與工業運動參數
    'x_steps_per_mm': 320.0,
    'z_steps_per_mm': 320.0,
    'y_steps_per_mm': 320.0,
    'x_microsteps': 1600,
    'z_microsteps': 1600,
    'x_lead_mm': 5.0,
    'z_lead_mm': 5.0,
    'homing_retract_mm': 3.0,
    'homing_latch_hz': 350.0,

    # 速度與加減速規劃 (高速微秒級脈衝)
    'x_max_speed': 16000.0,
    'x_accel_steps': 600.0,
    'z_max_speed': 16000.0,
    'z_accel_steps': 600.0,
    'y_max_speed': 16000.0,
    'y_accel_steps': 600.0,
    'homing_speed_hz': 2000.0,

    # 致能狀態
    'motor_locked': True,

    # 同步模式
    'y_sync': True,
    'z_sync': True,

    # 歸零執行狀態
    'is_homing': False,
    'x_homed': False,
    'z_homed': False,

    # ── 工業級連續運轉與壽命安全機制 (24/7/365 Continuous Mode) ──
    'continuous_mode':    False,   # 連續生產運轉模式
    'cycle_count':        0,       # 累計完成衝壓循環計數
    'cycle_target':       0,       # 目標生產次數 (0 = 無限制連續生產)
    'cycle_interval_sec': 1.0,     # 循環間隔緩衝時間 (秒)
    'uptime_sec':         0,       # 系統連續運行秒數
    'watchdog_active':    False,   # 硬體看門狗晶片守護狀態
    'magnet_on_time':     0.0,     # 電磁鐵通電起始時間戳 (防過熱保護)

    # 最後更新時間
    'timestamp': '',
    'gpio_mode': 'Raspberry Pi (Real)' if IS_RASPBERRY_PI else 'Simulation',
    'app_version': f'v{APP_VERSION}',
    # 安全設定
    'lock_timeout_min': 30,
}

homing_lock = threading.Lock()
homing_abort_event = threading.Event()


# 歷史波形資料（供網頁繪圖）
MAX_HISTORY = 60
pwm_history = {
    'x':  [PULSE_CENTER] * MAX_HISTORY,
    'y1': [PULSE_CENTER] * MAX_HISTORY,
    'y2': [PULSE_CENTER] * MAX_HISTORY,
    'time': list(range(MAX_HISTORY)),
}

# ── 步進馬達背景控制類別 ──────────────────────────────────────────
class StepperMotor:
    def __init__(self, name, pul_pin, dir_pin, ena_pin):
        self.name = name
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.ena_pin = ena_pin

        self.current_pos = float(PULSE_CENTER)
        self.target_pos = float(PULSE_CENTER)
        self.steps_per_unit = 5.0  # 1 us 單元對應 5 個步進脈衝
        self.lock = threading.Lock()
        
        # 速度規劃參數
        self.max_speed_hz = 8000.0
        self.min_speed_hz = 600.0
        self.accel_steps = 50.0
        self.current_speed_hz = 600.0
        
        self.enabled = True
        self.running = True
        # 歸零時暫停背景執行緒以避免競態條件
        self.pause_for_homing = False
        
        if IS_RASPBERRY_PI:
            GPIO.setup(self.pul_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.ena_pin, GPIO.OUT, initial=GPIO.LOW) # 低電位使能

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_target(self, target, min_us=0.0, max_us=200000.0):
        with self.lock:
            self.target_pos = max(float(min_us), min(float(max_us), float(target)))
            return self.target_pos

    def add_target_delta(self, delta_us, min_us=0.0, max_us=200000.0):
        """累加移動量（支援 100+100 快速連續點擊，不被實時當前位置覆蓋）"""
        with self.lock:
            self.target_pos = max(float(min_us), min(float(max_us), self.target_pos + float(delta_us)))
            return self.target_pos

    def set_position(self, pos):
        with self.lock:
            self.current_pos = float(pos)
            self.target_pos = float(pos)

    def set_enable(self, value):
        self.enabled = value
        if IS_RASPBERRY_PI:
            GPIO.output(self.ena_pin, GPIO.LOW if value else GPIO.HIGH)

    def halt(self):
        """立即停止：將目標位置設為當前位置，並重置速度"""
        with self.lock:
            self.target_pos = self.current_pos
            self.current_speed_hz = self.min_speed_hz

    def _run(self):
        while self.running:
            # [修正1] 歸零期間暫停背景執行緒，避免與 home_axis() 競爭 GPIO 寫入
            if self.pause_for_homing:
                time.sleep(0.005)
                continue

            if not self.enabled:
                time.sleep(0.01)
                continue

            diff = self.target_pos - self.current_pos
            step_val = 1.0 / self.steps_per_unit
            dist_steps = abs(diff) * self.steps_per_unit
            
            if dist_steps >= 1.0:
                # 梯形加減速規劃
                if dist_steps < self.accel_steps:
                    # 減速區
                    speed = self.min_speed_hz + (self.max_speed_hz - self.min_speed_hz) * (dist_steps / self.accel_steps)
                else:
                    # 加速區
                    speed = min(self.max_speed_hz, self.current_speed_hz + (self.max_speed_hz - self.min_speed_hz) / self.accel_steps)
                
                self.current_speed_hz = max(self.min_speed_hz, min(self.max_speed_hz, speed))
                
                # 即時感測器硬體防撞與原點觸發歸零保護（每次步進前直接硬體讀取）
                if IS_RASPBERRY_PI:
                    if self.name == 'x':
                        lim_a_hit = not GPIO.input(PIN['LIM_X_A']) or not GPIO.input(PIN['HALL_1']) or not GPIO.input(PIN['HALL_2'])
                        if diff < 0 and lim_a_hit:
                            self.halt()
                            self.current_pos = 0.0
                            state['lim_x_a'] = True
                            state['lim_left'] = True
                            state['x_pulse'] = 0
                            state['x_mm'] = 0.0
                            state['x_pct'] = 0.0
                            state['x_homed'] = True
                            time.sleep(0.01)
                            continue
                        xc_ah = bool(state.get('lim_x_c_active_high', True))
                        lim_c_hit = bool(GPIO.input(PIN['LIM_X_C'])) if xc_ah else not GPIO.input(PIN['LIM_X_C'])
                        if diff > 0 and lim_c_hit:
                            self.halt()
                            state['lim_x_c'] = True
                            state['lim_right'] = True
                            time.sleep(0.01)
                            continue
                    else: # z or y
                        zd_ah = bool(state.get('lim_z_d_active_high', True))
                        lim_origin_hit = bool(GPIO.input(PIN['LIM_Z_D'])) if zd_ah else not GPIO.input(PIN['LIM_Z_D'])
                        if diff < 0 and lim_origin_hit:
                            self.halt()
                            self.current_pos = 0.0
                            state['lim_z_d'] = True
                            state['lim_up'] = True
                            state['z_pulse'] = 0
                            state['z_mm'] = 0.0
                            state['z_pct'] = 0.0
                            state['z_homed'] = True
                            time.sleep(0.01)
                            continue
                        if diff > 0 and not GPIO.input(PIN['LIM_Z_E']):
                            self.halt()
                            state['lim_z_e'] = True
                            state['lim_down'] = True
                            time.sleep(0.01)
                            continue
                else:
                    if diff < 0 and self.name == 'x' and (state.get('lim_x_a') or state.get('lim_left')):
                        self.halt()
                        self.current_pos = 0.0
                        time.sleep(0.01)
                        continue
                    if diff > 0 and self.name == 'x' and (state.get('lim_x_c') or state.get('lim_right')):
                        self.halt()
                        time.sleep(0.01)
                        continue
                    if diff < 0 and self.name in ('z', 'y') and (state.get('lim_z_d') or state.get('lim_up') or state.get('lim_z_e') or state.get('lim_down')):
                        self.halt()
                        self.current_pos = 0.0
                        time.sleep(0.01)
                        continue

                if not IS_RASPBERRY_PI:
                    # 模擬模式：以 100Hz 高速平滑步進精確模擬運動
                    time.sleep(0.01)
                    self.current_speed_hz = self.max_speed_hz
                    step_chunk = min(abs(diff), (self.max_speed_hz / self.steps_per_unit) * 0.01)
                    if diff > 0:
                        self.current_pos += step_chunk
                    else:
                        self.current_pos -= step_chunk
                    continue

                # 實際樹莓派模式：閉迴路步進批次脈衝輸出
                direction = GPIO.HIGH if diff > 0 else GPIO.LOW
                GPIO.output(self.dir_pin, direction)
                batch = max(1, min(25, int(dist_steps)))
                step_delay = max(0.000004, (1.0 / max(100.0, self.current_speed_hz)) - 0.000004)
                actual_steps = 0
                for _ in range(batch):
                    if self.pause_for_homing or not self.enabled:
                        break
                    GPIO.output(self.pul_pin, GPIO.HIGH)
                    t0 = time.perf_counter()
                    while time.perf_counter() - t0 < 0.000004:
                        pass
                    GPIO.output(self.pul_pin, GPIO.LOW)
                    if step_delay > 0.0003:
                        time.sleep(step_delay - 0.00015)
                    t1 = time.perf_counter()
                    while time.perf_counter() - t1 < min(step_delay, 0.0002):
                        pass
                    actual_steps += 1

                if actual_steps > 0:
                    delta_pos = actual_steps * step_val
                    if diff > 0:
                        self.current_pos = min(self.target_pos, self.current_pos + delta_pos)
                    else:
                        self.current_pos = max(self.target_pos, self.current_pos - delta_pos)
            else:
                self.current_speed_hz = self.min_speed_hz
                time.sleep(0.002)

    def stop(self):
        self.running = False

# ── 步進馬達控制器物件 ─────────────────────────────────────────────
x_stepper = None
z_stepper = None
y_stepper = None  # 向下相容別名

def init_gpio():
    """初始化 GPIO 引腳、電感式接近開關、電磁鐵與步進馬達控制器"""
    global x_stepper, z_stepper, y_stepper
    
    # 建立/初始化步進馬達背景控制器
    if x_stepper is None:
        x_stepper = StepperMotor('x', PIN['X_PUL'], PIN['X_DIR'], PIN['X_ENA'])
    else:
        x_stepper.set_position(PULSE_CENTER)
        
    if z_stepper is None:
        z_stepper = StepperMotor('z', PIN['Z_PUL'], PIN['Z_DIR'], PIN['Z_ENA'])
        y_stepper = z_stepper  # 向下相容
    else:
        z_stepper.set_position(PULSE_CENTER)

    if not IS_RASPBERRY_PI:
        return

    # 數位輸出 (4 組圓形電磁鐵 LSD-P25/20 + JQC-3FF-S-Z 繼電器)
    for pin_name in MAGNET_PIN_KEYS:
        GPIO.setup(PIN[pin_name], GPIO.OUT, initial=GPIO.LOW)

    # 數位輸入（電感式接近開關 QN-1805-E1 方形 DC10-30V 三線 NPN 常開 + 8顆手臂金屬霍爾感測器(兩個一組) + 衝壓完成訊號）
    # NPN 常開開關與 817 光耦隔離板在感應金屬時輸出拉低至 GND，樹莓派端配置 PUD_UP 內部上拉，讀入為 LOW 時表示觸發
    # 採用穩定週期輪詢 (Polling) 取代 14 腳位 GPIO 中斷 (add_event_detect)，徹底消除電磁雜訊中斷風暴與 RPi.GPIO 執行緒 GIL 死鎖
    input_pins = ['LIM_X_A', 'LIM_X_B', 'LIM_X_C', 'LIM_Z_D', 'LIM_Z_E'] + HALL_PIN_KEYS + ['PRESS_DONE']
    for pin_name in input_pins:
        p = PIN[pin_name]
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        try:
            GPIO.remove_event_detect(p)
        except Exception:
            pass

    print("[GPIO] Initialized stepper motor system, QN-1805-E1 sensors, and LSD-P25/20 electromagnet successfully")

def cleanup_gpio():
    """清理 GPIO 與安全關閉硬體看門狗"""
    global x_stepper, z_stepper, y_stepper, watchdog_fd
    if x_stepper:
        x_stepper.stop()
    if z_stepper:
        z_stepper.stop()
    if watchdog_fd and not watchdog_fd.closed:
        try:
            watchdog_fd.write('V')  # Magic close character: 防止服務重啟時觸發樹莓派硬體強制重開機
            watchdog_fd.flush()
            watchdog_fd.close()
        except Exception:
            pass
    if not IS_RASPBERRY_PI:
        return
    GPIO.cleanup()

def set_servo_pulse(axis, pulse_us):
    """設定馬達目標位置（依照設定的軸極限動態限制，不受 2500 限制）"""
    axis = axis.lower()
    if axis == 'x':
        pulse_us = max(state['x_min'], min(state['x_max'], float(pulse_us)))
        if x_stepper:
            x_stepper.set_target(pulse_us, state['x_min'], state['x_max'])

    elif axis in ('z', 'y', 'y1', 'y2'):
        z_min = state.get('z_min', state['y_min'])
        z_max = state.get('z_max', state['y_max'])
        pulse_us = max(z_min, min(z_max, float(pulse_us)))
        st = z_stepper or y_stepper
        if st:
            st.set_target(pulse_us, z_min, z_max)

def set_digital_output(name, value):
    """設定數位輸出 (4 組圓形電磁鐵吸盤 M1~M4 / 手臂)，含電磁鐵超時計時器防過熱保護"""
    name = name.lower()
    val_bool = bool(value)
    if name in ('magnet', 'vacuum', 'magnet_all'):
        for g in range(1, 5):
            state[f'magnet_{g}'] = val_bool
            if IS_RASPBERRY_PI:
                GPIO.output(PIN[f'MAGNET_{g}'], GPIO.HIGH if val_bool else GPIO.LOW)
        state['magnet_active_count'] = 4 if val_bool else 0
        state['magnet'] = val_bool
        state['vacuum'] = val_bool
        state['magnet_on_time'] = time.time() if val_bool else 0.0

    elif name in ('magnet_1', 'magnet_2', 'magnet_3', 'magnet_4'):
        g = int(name.split('_')[1])
        state[f'magnet_{g}'] = val_bool
        if IS_RASPBERRY_PI:
            GPIO.output(PIN[f'MAGNET_{g}'], GPIO.HIGH if val_bool else GPIO.LOW)
        active_cnt = sum(1 for i in range(1, 5) if state.get(f'magnet_{i}'))
        state['magnet_active_count'] = active_cnt
        any_on = active_cnt > 0
        if any_on and not state.get('magnet'):
            state['magnet_on_time'] = time.time()
        elif not any_on:
            state['magnet_on_time'] = 0.0
        state['magnet'] = any_on
        state['vacuum'] = any_on

    elif name == 'z_down':
        state['z_down'] = val_bool
        state['arm_down'] = val_bool

sensor_read_lock = threading.Lock()

def read_limit_switches():
    """讀取所有電感式接近開關 (QN-1805-E1) 與 8 顆末端金屬霍爾感測器 (兩個一組共 4 組，具備 3-Sample 工業數位防彈跳濾波)"""
    global x_stepper, z_stepper, y_stepper
    if not sensor_read_lock.acquire(blocking=False):
        return
    try:
        if IS_RASPBERRY_PI:
            # 工業抗干擾數位濾波 (Digital Debounce Filter)：3 次快速取樣多數決 (>=2)，無阻塞讀取
            pins = [
                PIN['LIM_X_A'], PIN['LIM_X_B'], PIN['LIM_X_C'],
                PIN['LIM_Z_D'], PIN['LIM_Z_E'],
                PIN['HALL_1'], PIN['HALL_2'], PIN['HALL_3'], PIN['HALL_4'],
                PIN['HALL_5'], PIN['HALL_6'], PIN['HALL_7'], PIN['HALL_8'],
                PIN['PRESS_DONE']
            ]
            ah = bool(state.get('hall_active_high', True))
            xa_ah = bool(state.get('lim_x_a_active_high', False))
            xb_ah = bool(state.get('lim_x_b_active_high', False))
            xc_ah = bool(state.get('lim_x_c_active_high', True))
            zd_ah = bool(state.get('lim_z_d_active_high', True))
            ze_ah = bool(state.get('lim_z_e_active_high', False))
            readings = [0] * len(pins)
            for sample_idx in range(3):
                for i, p in enumerate(pins):
                    val = bool(GPIO.input(p))
                    if i == 0:
                        is_trig = val if xa_ah else not val
                    elif i == 1:
                        is_trig = val if xb_ah else not val
                    elif i == 2:
                        is_trig = val if xc_ah else not val
                    elif i == 3:
                        is_trig = val if zd_ah else not val
                    elif i == 4:
                        is_trig = val if ze_ah else not val
                    elif 5 <= i <= 12:
                        is_trig = val if ah else not val
                    else:
                        is_trig = not val
                    if is_trig:
                        readings[i] += 1
                if sample_idx < 2:
                    time.sleep(0.0001)

            state['lim_x_a'] = readings[0] >= 2
            state['lim_x_b'] = readings[1] >= 2
            state['lim_x_c'] = readings[2] >= 2
            state['lim_z_d'] = readings[3] >= 2
            state['lim_z_e'] = readings[4] >= 2
            for idx in range(8):
                state[f'hall_{idx+1}'] = readings[5 + idx] >= 2
            state['press_done'] = readings[13] >= 2
        else:
            # 模擬模式：根據步進馬達當前位置計算感測器觸發
            if x_stepper:
                state['lim_x_a'] = x_stepper.current_pos <= state['x_min'] + 5
                x_mid = (state['x_min'] + state['x_max']) / 2.0
                state['lim_x_b'] = abs(x_stepper.current_pos - x_mid) <= 30
                state['lim_x_c'] = x_stepper.current_pos >= state['x_max'] - 5
            st_z = z_stepper or y_stepper
            if st_z:
                z_min = state.get('z_min', state['y_min'])
                z_max = state.get('z_max', state['y_max'])
                state['lim_z_e'] = st_z.current_pos <= z_min + 5
                state['lim_z_d'] = st_z.current_pos >= z_max - 5
            # 模擬模式：4 組電磁鐵 (M1~M4) 分別對應模擬 4 組霍爾開關 (G1: H1+H2, G2: H3+H4, G3: H5+H6, G4: H7+H8)
            for g in range(1, 5):
                g_mag = bool(state.get(f'magnet_{g}', state.get('magnet', False)))
                state[f'hall_{g*2 - 1}'] = g_mag
                state[f'hall_{g*2}']     = g_mag
            state['press_done'] = True

        # 計算 8 顆霍爾開關「兩個一組」(共 4 組) 狀態：同組兩顆皆觸發 (或組內觸發)
        state['hall_group_1'] = bool(state['hall_1'] and state['hall_2'])
        state['hall_group_2'] = bool(state['hall_3'] and state['hall_4'])
        state['hall_group_3'] = bool(state['hall_5'] and state['hall_6'])
        state['hall_group_4'] = bool(state['hall_7'] and state['hall_8'])
        state['hall_active_count'] = sum(1 for i in range(1, 9) if state.get(f'hall_{i}'))
        state['hall_group_count']  = sum(1 for g in range(1, 5) if state.get(f'hall_group_{g}'))
        # 當任一組完整觸發 (或有霍爾感應到金屬) 時，判定金屬在位
        state['hall_arm_metal'] = bool(state['hall_group_count'] > 0 or state['hall_active_count'] > 0)

        # X 軸原點（a點左側）：實體霍爾模組 (HALL_1 / HALL_2 / Group 1) 或 LIM_X_A
        state['lim_x_a'] = bool(state['lim_x_a'] or state['hall_1'] or state['hall_2'] or state['hall_group_1'])
        state['lim_left'] = state['lim_x_a']
        state['lim_mid'] = state['lim_x_b']
        state['lim_right'] = state['lim_x_c']

        # Y/Z 軸頂部原點（d點向上）：僅接近開關 LIM_Z_D（不混入末端霍爾）
        state['lim_up'] = state['lim_z_d']
        state['lim_down'] = state['lim_z_e']
        state['hall_metal'] = state['hall_arm_metal']
    finally:
        sensor_read_lock.release()

def _execute_hardware_homing(target_axes):
    """樹梅派真實 GPIO 兩階段高精準歸零核心 (X回a點, Z回d點向上歸零)"""
    global x_stepper, z_stepper, y_stepper, homing_abort_event
    
    axis_configs = []
    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        if ax == 'x':
            lim_pin = PIN['LIM_X_A']
            state_key = 'lim_x_a'
            home_pos = 0.0
            min_bound = 0.0
            max_bound = float(state['x_max'])
            seek_dir = GPIO.LOW      # 負方向向左朝 a 點歸零
            backoff_dir = GPIO.HIGH  # 正方向向右退開
            step_sign = -1.0
        else: # z or y
            lim_pin = PIN['LIM_Z_D']
            state_key = 'lim_z_d'
            home_pos = 0.0
            min_bound = 0.0
            max_bound = float(state.get('z_max', state['y_max']))
            seek_dir = GPIO.LOW     # 朝原點歸零
            backoff_dir = GPIO.HIGH  # 退開
            step_sign = -1.0

        step_val = 1.0 / st.steps_per_unit
        GPIO.output(st.ena_pin, GPIO.LOW) # 確保致能通電
        
        axis_configs.append({
            'name': ax,
            'stepper': st,
            'pul_pin': st.pul_pin,
            'dir_pin': st.dir_pin,
            'lim_pin': lim_pin,
            'state_key': state_key,
            'home_pos': home_pos,
            'min_bound': min_bound,
            'max_bound': max_bound,
            'step_val': step_val,
            'step_sign': step_sign,
            'seek_dir': seek_dir,
            'backoff_dir': backoff_dir,
            'triggered': False,
            'sensor_hit': False,
            'timed_out': False
        })

    # ── 階段 A: 預先檢查 - 若原先已壓在接近開關或霍爾模組上，先慢速退開 ────
    def _is_sensor_active(c):
        if c['name'] == 'x':
            xa_ah = bool(state.get('lim_x_a_active_high', False))
            xa_hit = bool(GPIO.input(c['lim_pin'])) if xa_ah else not GPIO.input(c['lim_pin'])
            return not GPIO.input(PIN['HALL_1']) or not GPIO.input(PIN['HALL_2']) or xa_hit
        zd_ah = bool(state.get('lim_z_d_active_high', True))
        return bool(GPIO.input(c['lim_pin'])) if zd_ah else not GPIO.input(c['lim_pin'])

    for cfg in axis_configs:
        if _is_sensor_active(cfg):
            add_history(f"{cfg['name'].upper()} 軸感測器處於觸發狀態，執行預先退開...", "info")
            GPIO.output(cfg['dir_pin'], cfg['backoff_dir'])
            pre_step_interval = 1.0 / 250.0
            pre_steps = 0
            while pre_steps < 3000:
                if not _is_sensor_active(cfg):
                    break
                if homing_abort_event.is_set():
                    add_history("歸零程序被中止（緊急停止）", "warn")
                    return
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                pre_steps += 1
                time.sleep(max(0.0005, pre_step_interval - 0.00001))
            
            # 再多退 50 步作安全淨空
            for _ in range(50):
                if homing_abort_event.is_set():
                    return
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                time.sleep(max(0.0005, pre_step_interval - 0.00001))

    # ── 階段 B: 平穩逼近 (Toward Origin / Seek) ───────────────────
    homing_hz = max(150.0, min(25000.0, float(state.get('homing_speed_hz', 2500.0))))
    step_interval = 1.0 / homing_hz
    max_steps = 500000

    for cfg in axis_configs:
        GPIO.output(cfg['dir_pin'], cfg['seek_dir'])
        cfg['triggered'] = False

    steps = 0
    while steps < max_steps:
        if homing_abort_event.is_set():
            add_history("歸零程序被中止（緊急停止）", "warn")
            return

        active_pulse = []

        for cfg in axis_configs:
            if not cfg['triggered']:
                pin_triggered = _is_sensor_active(cfg)
                if pin_triggered:
                    cfg['triggered'] = True
                    cfg['sensor_hit'] = True
                    cfg['stepper'].set_position(0.0)
                    state[cfg['state_key']] = True
                    add_history(f"{cfg['name'].upper()} 軸碰觸接近開關原點，捕獲原點 (0.0 mm)！", "info")
                else:
                    GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                    active_pulse.append(cfg)

        if not active_pulse:
            break

        time.sleep(0.00001)

        for cfg in active_pulse:
            GPIO.output(cfg['pul_pin'], GPIO.LOW)
            next_pos = cfg['stepper'].current_pos + cfg['step_sign'] * cfg['step_val']
            cfg['stepper'].current_pos = max(cfg['min_bound'], min(cfg['max_bound'], next_pos))

        steps += 1
        time.sleep(max(0.0003, step_interval - 0.00001))

    for cfg in axis_configs:
        if not cfg['triggered']:
            cfg['timed_out'] = True
            cfg['stepper'].set_position(0.0)
            add_history(f"{cfg['name'].upper()} 軸歸零警告: 超時未觸發接近開關，鎖定於原點", "warn")

    # ── 階段 C: 精準退開至開關剛釋放 (Pull-off / Latch) ─────────────
    backoff_hz = 250.0
    backoff_interval = 1.0 / backoff_hz
    for cfg in axis_configs:
        if homing_abort_event.is_set():
            continue

        if cfg.get('sensor_hit') and _is_sensor_active(cfg):
            GPIO.output(cfg['dir_pin'], cfg['backoff_dir'])
            
            bo_steps = 0
            while _is_sensor_active(cfg) and bo_steps < 2000:
                if homing_abort_event.is_set():
                    return
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                bo_steps += 1
                time.sleep(max(0.0005, backoff_interval - 0.00001))

            for _ in range(25):
                if homing_abort_event.is_set():
                    return
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                time.sleep(max(0.0005, backoff_interval - 0.00001))

        # 標記歸零點座標並夾限保護，確保坐標完全歸零為 0.0
        cfg['stepper'].set_position(0.0)
        cfg['stepper'].set_target(0.0, 0.0, cfg['max_bound'])
        state[cfg['state_key']] = False
        if cfg['name'] == 'x':
            state['x_pulse'] = 0
            state['x_mm'] = 0.0
            state['x_pct'] = 0.0
            state['x_homed'] = True
        else:
            state['z_pulse'] = 0
            state['z_mm'] = 0.0
            state['z_pct'] = 0.0
            state['y1_pulse'] = 0
            state['y2_pulse'] = 0
            state['y_mm'] = 0.0
            state['y_pct'] = 0.0
            state['z_homed'] = True
        add_history(f"{cfg['name'].upper()} 軸平穩歸零完成 (坐標點已精準歸零：0.0 mm / 0 µs)", "success")

def _execute_simulation_homing(target_axes):
    """模擬環境平滑歸零核心 (就算坐標系為 0 亦可隨時完整執行退開與歸零)"""
    global x_stepper, z_stepper, y_stepper, homing_abort_event
    
    # 檢查是否已有軸處於 0 點，若已在 0 點則先執行預先安全退開
    need_backoff = False
    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        if abs(float(st.current_pos)) < 5.0:
            need_backoff = True
            break

    if need_backoff:
        add_history("當前座標已在原點 (0.0)，執行預先安全退開再重新定標歸零...", "info")
        for i in range(10):
            if homing_abort_event.is_set():
                return
            r = (i + 1) / 10.0
            for ax in target_axes:
                st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
                st.current_pos = 100.0 * r
            time.sleep(0.015)
        time.sleep(0.04)

    start_positions = {}
    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        start_positions[ax] = float(st.current_pos)

    steps_cnt = 20
    for i in range(steps_cnt):
        if homing_abort_event.is_set():
            add_history("歸零程序被中止（緊急停止）", "warn")
            return
        
        ratio = (i + 1) / steps_cnt
        for ax in target_axes:
            st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
            st.current_pos = start_positions[ax] - start_positions[ax] * ratio
        time.sleep(0.02)

    # 模擬觸發現位瞬間反饋
    for ax in target_axes:
        key = 'lim_x_a' if ax == 'x' else 'lim_z_d'
        state[key] = True
        state['lim_left' if ax == 'x' else 'lim_up'] = True
    time.sleep(0.05)

    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        st.set_position(0.0)
        st.set_target(0.0)
        key = 'lim_x_a' if ax == 'x' else 'lim_z_d'
        state[key] = False
        state['lim_left' if ax == 'x' else 'lim_up'] = False
        if ax == 'x':
            state['x_pulse'] = 0
            state['x_mm'] = 0.0
            state['x_pct'] = 0.0
            state['x_homed'] = True
        else:
            state['z_pulse'] = 0
            state['z_mm'] = 0.0
            state['z_pct'] = 0.0
            state['y1_pulse'] = 0
            state['y2_pulse'] = 0
            state['y_mm'] = 0.0
            state['y_pct'] = 0.0
            state['z_homed'] = True
        add_history(f"{ax.upper()} 軸平穩歸零完成 (坐標點已精準歸零：0.0 mm / 0 µs)", "success")

def _home_single_axis(axis_name):
    """單軸歸零相容介面"""
    return home_axis(axis_name)

def home_axis(axis='all'):
    """馬達智慧平穩精準歸零 (支援 X, Z, Y, ALL)"""
    global homing_lock, homing_abort_event, x_stepper, z_stepper, y_stepper
    
    if not homing_lock.acquire(blocking=False):
        add_history("已有歸零程序執行中，無視重複請求", "warn")
        return False

    homing_abort_event.clear()
    state['is_homing'] = True
    
    target_axes = []
    axis = axis.lower()
    if axis in ('all', 'xy', 'xz'):
        if x_stepper: target_axes.append('x')
        if z_stepper or y_stepper: target_axes.append('z')
    elif axis == 'x' and x_stepper:
        target_axes.append('x')
    elif axis in ('z', 'y', 'y1', 'y2') and (z_stepper or y_stepper):
        target_axes.append('z')

    if not target_axes:
        state['is_homing'] = False
        homing_lock.release()
        return True

    try:
        # 暫停被歸零軸的背景執行緒
        for ax in target_axes:
            st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
            st.pause_for_homing = True
        time.sleep(0.02)

        # 執行硬體或模擬歸零
        if IS_RASPBERRY_PI:
            _execute_hardware_homing(target_axes)
        else:
            _execute_simulation_homing(target_axes)

        read_limit_switches()
        return True

    finally:
        for ax in target_axes:
            st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
            st.pause_for_homing = False
        state['is_homing'] = False
        homing_lock.release()

# ── 背景廣播執行緒 ────────────────────────────────────────────────
broadcast_lock = threading.Lock()

def broadcast_state():
    """每 100ms 廣播狀態到所有客戶端"""
    global x_stepper, z_stepper, y_stepper
    sim_t = 0
    sim_tick = 0
    t_system_start = time.time()
    while True:
        try:
            # 同步實時馬達位置與百分比、物理 mm 座標至系統狀態中
            x_min_mm_base = float(state.get('x_min_mm', 0.0))
            z_min_mm_base = float(state.get('z_min_mm', 0.0))
            state['x_max_mm'] = round(x_min_mm_base + (state['x_max'] - state['x_min']) / max(0.001, state['x_scale']), 1)
            z_scale = state.get('z_scale', state['y_scale'])
            state['z_max_mm'] = round(z_min_mm_base + (state.get('z_max', state['y_max']) - state.get('z_min', state['y_min'])) / max(0.001, z_scale), 1)
            state['y_min_mm'] = z_min_mm_base
            state['y_max_mm'] = state['z_max_mm']

            if x_stepper:
                x_clamped = max(float(state['x_min']), min(float(state['x_max']), float(x_stepper.current_pos)))
                state['x_pulse'] = int(round(x_clamped))
                state['x_pct'] = max(0.0, min(100.0, (x_clamped - state['x_min']) / max(1, state['x_max'] - state['x_min']) * 100.0))
                raw_x_mm = x_min_mm_base + max(0.0, (x_clamped - state['x_min']) / max(0.001, state['x_scale']))
                state['x_mm'] = round(max(x_min_mm_base, min(state['x_max_mm'], raw_x_mm)), 1)
            
                if (state.get('lim_x_a') or state.get('lim_left')) and x_clamped <= float(state['x_min']) + 15.0:
                    state['x_pulse'] = 0
                    state['x_mm'] = 0.0
                    state['x_pct'] = 0.0
            
            st_z = z_stepper or y_stepper
            if st_z:
                z_min = state.get('z_min', state['y_min'])
                z_max = state.get('z_max', state['y_max'])
                z_clamped = max(float(z_min), min(float(z_max), float(st_z.current_pos)))
                state['z_pulse'] = int(round(z_clamped))
                state['z_pct'] = max(0.0, min(100.0, (z_clamped - z_min) / max(1, z_max - z_min) * 100.0))
                raw_z_mm = z_min_mm_base + max(0.0, (z_clamped - z_min) / max(0.001, z_scale))
                state['z_mm'] = round(max(z_min_mm_base, min(state['z_max_mm'], raw_z_mm)), 1)

                if (state.get('lim_z_d') or state.get('lim_up')) and z_clamped <= float(z_min) + 15.0:
                    state['z_pulse'] = 0
                    state['z_mm'] = 0.0
                    state['z_pct'] = 0.0

                # 同步保持 y 鍵名以確保所有舊版前端元件皆向下相容
                state['y1_pulse'] = state['z_pulse']
                state['y2_pulse'] = state['z_pulse']
                state['y_pct'] = state['z_pct']
                state['y_mm'] = state['z_mm']

            read_limit_switches()
            state['timestamp'] = datetime.now().strftime('%H:%M:%S.%f')[:-3]

            # 模擬模式：產生輕微抖動效果（視覺用）
            if not IS_RASPBERRY_PI and state['system_run']:
                noise = math.sin(sim_t * 0.3) * 3
                sim_t += 0.1
            else:
                noise = 0
                sim_t += 0.1

            # 更新 PWM 歷史
            with broadcast_lock:
                pwm_history['x'].append(round(state['x_pulse'] + noise, 1))
                pwm_history['y1'].append(round(state['y1_pulse'] + noise * 0.8, 1))
                pwm_history['y2'].append(round(state['y2_pulse'] + noise * 0.6, 1))
                if len(pwm_history['x']) > MAX_HISTORY:
                    pwm_history['x'].pop(0)
                    pwm_history['y1'].pop(0)
                    pwm_history['y2'].pop(0)

            # ── 電磁鐵連續通電過熱防護 (超過 45 秒強制斷電防燒毀) ────
            if state.get('magnet') and state.get('magnet_on_time', 0.0) > 0:
                if time.time() - state['magnet_on_time'] > 45.0:
                    set_digital_output('magnet', False)
                    state['error'] = True
                    state['error_msg'] = "電磁鐵連續通電超過 45 秒，強制斷電保護"
                    add_history("⚠️ 安全保護：電磁鐵連續吸附超過 45 秒，強制斷電防線圈過熱！", "warn")

            payload = dict(state)
            # 每 3 次廣播才附帶較大的 pwm_history 與 pin_config，大幅減輕 WebSocket 封包負載避免斷線
            if sim_tick % 3 == 0:
                payload['pwm_history'] = {
                    'x':  list(pwm_history['x']),
                    'y1': list(pwm_history['y1']),
                    'y2': list(pwm_history['y2']),
                }
                payload['pin_config'] = PIN
            socketio.emit('state_update', payload)

        except Exception as e:
            pass

        # ── 系統持續運轉壽命守護（GC 垃圾回收 + 歷史紀錄非同步批次存檔）──
        sim_tick += 1
        state['uptime_sec'] = int(time.time() - t_system_start)

        # 每 30 分鐘執行主動垃圾回收，防止長週期記憶體碎片
        if sim_tick % 10000 == 0:
            gc.collect()

        # 每 5 分鐘若有新日誌才非同步刷入 SD 卡，延長 MicroSD 壽命
        if sim_tick % 1600 == 0:
            if history_dirty:
                save_history()

        time.sleep(0.18)

# ── REST API ──────────────────────────────────────────────────────
@app.after_request
def add_header(response):
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status')
def api_status():
    return jsonify({**state, 'pwm_history': pwm_history, 'pin_config': PIN})

@app.route('/api/move', methods=['POST'])
def api_move():
    """移動馬達（支援 mm 累加或直接指定目標，支援 X 軸與 Z 軸）"""
    data = request.get_json(silent=True) or {}
    axis = str(data.get('axis', 'x')).lower()       # 'x', 'z', 'y', 'y1', 'y2'
    delta_mm = data.get('delta_mm')
    delta_us = data.get('delta')
    target_mm = data.get('target_mm')
    target_us = data.get('target')

    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動，請按 START'})

    scale = state['x_scale'] if axis == 'x' else state.get('z_scale', state['y_scale'])

    if delta_mm is not None:
        delta_us = float(delta_mm) * float(scale)
    if target_mm is not None:
        base_min = state['x_min'] if axis == 'x' else state.get('z_min', state['y_min'])
        target_us = base_min + float(target_mm) * float(scale)

    try:
        hw_lim_xa = state.get('lim_x_a') or state.get('lim_left') or (IS_RASPBERRY_PI and (not GPIO.input(PIN['LIM_X_A']) or not GPIO.input(PIN['HALL_1']) or not GPIO.input(PIN['HALL_2'])))
        hw_lim_xc = state.get('lim_x_c') or state.get('lim_right') or (IS_RASPBERRY_PI and not GPIO.input(PIN['LIM_X_C']))
        hw_lim_zd = state.get('lim_z_d') or state.get('lim_up') or (IS_RASPBERRY_PI and (not GPIO.input(PIN['LIM_Z_D']) or not GPIO.input(PIN['HALL_1']) or not GPIO.input(PIN['HALL_2'])))
        hw_lim_ze = state.get('lim_z_e') or state.get('lim_down') or (IS_RASPBERRY_PI and not GPIO.input(PIN['LIM_Z_E']))

        if axis == 'x':
            if delta_us is not None:
                delta_us = float(delta_us)
                if hw_lim_xa and delta_us < 0:
                    if x_stepper:
                        x_stepper.halt()
                        x_stepper.set_position(0.0)
                    add_error_log('WARN', 'MOTION', 'ERR_LIMIT_X_A', 'X 軸 a 點左側原點霍爾/限位開關觸發，坐標歸零並攔截向後移動')
                    return jsonify({'ok': False, 'msg': 'X 軸原點限位觸發，已歸零並禁止繼續向後移動'})
                if hw_lim_xc and delta_us > 0:
                    if x_stepper:
                        x_stepper.halt()
                    add_error_log('WARN', 'MOTION', 'ERR_LIMIT_X_C', 'X 軸 c 點右側限位開關觸發，攔截正向移動')
                    return jsonify({'ok': False, 'msg': 'X 軸 c 點右側限位觸發'})
                if x_stepper:
                    x_stepper.add_target_delta(delta_us, 0.0, state['x_max'])
            elif target_us is not None:
                target_us = float(target_us)
                if hw_lim_xa and target_us <= 0.0:
                    if x_stepper:
                        x_stepper.halt()
                        x_stepper.set_position(0.0)
                    return jsonify({'ok': False, 'msg': 'X 軸已在原點，禁止繼續向後移動'})
                if x_stepper:
                    x_stepper.set_target(target_us, 0.0, state['x_max'])

        elif axis in ('z', 'y', 'y1', 'y2'):
            st_z = z_stepper or y_stepper
            z_min = state.get('z_min', state['y_min'])
            z_max = state.get('z_max', state['y_max'])
            if delta_us is not None:
                delta_us = float(delta_us)
                if (hw_lim_zd or hw_lim_ze) and delta_us < 0:
                    if st_z:
                        st_z.halt()
                        st_z.set_position(0.0)
                    add_error_log('WARN', 'MOTION', 'ERR_LIMIT_Z_D', 'Z 軸原點霍爾/限位開關觸發，坐標歸零並攔截向後移動')
                    return jsonify({'ok': False, 'msg': 'Z 軸原點限位觸發，已歸零並禁止繼續向後移動'})
                if hw_lim_zd and delta_us > 0:
                    if st_z:
                        st_z.halt()
                    add_error_log('WARN', 'MOTION', 'ERR_LIMIT_Z_D', 'Z 軸上極限觸發，攔截正向移動')
                    return jsonify({'ok': False, 'msg': 'Z 軸限位觸發'})
                if st_z:
                    st_z.add_target_delta(delta_us, 0.0, z_max)
            elif target_us is not None:
                target_us = float(target_us)
                if (hw_lim_zd or hw_lim_ze) and target_us <= 0.0:
                    if st_z:
                        st_z.halt()
                        st_z.set_position(0.0)
                    return jsonify({'ok': False, 'msg': 'Z 軸已在原點，禁止繼續向後移動'})
                if st_z:
                    st_z.set_target(target_us, 0.0, z_max)

        return jsonify({'ok': True, 'x_pulse': state['x_pulse'],
                        'z_pulse': state['z_pulse'], 'y1_pulse': state['y1_pulse'], 'y2_pulse': state['y2_pulse']})
    except Exception as e:
        add_error_log('ERROR', 'MOTION', 'ERR_MOVE_EXCEPTION', f'馬達移動指令例外: {e}')
        return jsonify({'ok': False, 'msg': str(e)})

@app.route('/api/set_position', methods=['POST'])
def api_set_position():
    """透過百分比 0~100 設定馬達位置"""
    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動，請按 START'})

    data = request.get_json(silent=True) or {}
    axis = str(data.get('axis', 'x')).lower()
    pct  = max(0.0, min(100.0, float(data.get('pct', 50))))

    if axis == 'x':
        pulse = state['x_min'] + (state['x_max'] - state['x_min']) * pct / 100
        diff = pulse - state['x_pulse']
        if (state.get('lim_x_a') or state.get('lim_left')) and diff < 0:
            return jsonify({'ok': False, 'msg': 'X 軸左側 a 點限位'})
        if (state.get('lim_x_c') or state.get('lim_right')) and diff > 0:
            return jsonify({'ok': False, 'msg': 'X 軸右側 c 點限位'})
        set_servo_pulse('x', int(pulse))
    elif axis in ('z', 'y', 'y1', 'y2'):
        z_min = state.get('z_min', state['y_min'])
        z_max = state.get('z_max', state['y_max'])
        pulse = z_min + (z_max - z_min) * pct / 100
        diff = pulse - state['z_pulse']
        if (state.get('lim_z_d') or state.get('lim_up')) and diff > 0:
            return jsonify({'ok': False, 'msg': 'Z 軸上側 d 點限位'})
        if (state.get('lim_z_e') or state.get('lim_down')) and diff < 0:
            return jsonify({'ok': False, 'msg': 'Z 軸下側 e 點限位'})
        set_servo_pulse('z', int(pulse))

    return jsonify({'ok': True})

# ── 操作歷史紀錄與錯誤日誌中心 (MicroSD 卡壽命防護：記憶體環形緩衝區) ────────
HISTORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'history_log.json')
ERROR_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'error_log.json')
MACROS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'macros.json')

history_lock = threading.Lock()
history_buffer = deque(maxlen=250)
history_dirty = False

error_lock = threading.Lock()
error_buffer = deque(maxlen=500)
error_seq = 1

def _classify_error_meta(msg, level='ERROR'):
    m = str(msg).lower()
    if any(k in m for k in ('電磁鐵', '過熱', 'gpio', '看門狗')):
        return ('HARDWARE', 'ERR_HARDWARE_MAGNET')
    if any(k in m for k in ('霍爾', '金屬', '接近開關', '感測')):
        return ('SENSOR', 'ERR_SENSOR_CHECK')
    if any(k in m for k in ('歸零', '限位', '極限', '馬達', '移動')):
        return ('MOTION', 'ERR_MOTION_LIMIT')
    if any(k in m for k in ('巨集', '積木', '衝壓', 'g-code', 'gcode')):
        return ('MACRO', 'ERR_MACRO_EXEC')
    if any(k in m for k in ('密碼', '登入', '權限', '驗證')):
        return ('AUTH', 'ERR_AUTH_DENIED')
    if any(k in m for k in ('更新', 'github', 'git')):
        return ('OTA', 'ERR_OTA_UPDATE')
    if any(k in m for k in ('前端', 'js', '連線', 'websocket')):
        return ('FRONTEND', 'ERR_FRONTEND_RUNTIME')
    if str(level).upper() == 'WARN':
        return ('SYSTEM', 'WARN_SYSTEM_EVENT')
    return ('SYSTEM', 'ERR_SYSTEM_FAULT')

def _get_machine_snapshot():
    return (
        f"X={state.get('x_mm', 0):.1f}mm({state.get('x_pulse', 0)}µs) "
        f"Z={state.get('z_mm', 0):.1f}mm({state.get('z_pulse', 0)}µs) | "
        f"Run={state.get('system_run', False)} | M={state.get('magnet_active_count', 0)}/4 | "
        f"Hall={state.get('hall_active_count', 0)}/8(G={state.get('hall_group_count', 0)}/4)"
    )

def save_error_logs():
    with error_lock:
        data_to_write = list(error_buffer)
    try:
        with open(ERROR_LOG_FILE, 'w', encoding='utf-8') as f:
            json.dump(data_to_write, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[WARN] Failed to save error_log.json: {e}")

def add_error_log(level='ERROR', source=None, code=None, msg='', detail=None):
    global error_seq
    lvl = str(level or 'ERROR').upper()
    auto_src, auto_code = _classify_error_meta(msg, lvl)
    src_str = str(source or auto_src).upper()
    code_str = str(code or auto_code).upper()
    det_str = str(detail) if detail else _get_machine_snapshot()
    time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with error_lock:
        eid = error_seq
        error_seq += 1
        entry = {
            'id': eid,
            'time': time_str,
            'level': lvl,
            'source': src_str,
            'code': code_str,
            'msg': str(msg),
            'detail': det_str,
            'resolved': False
        }
        error_buffer.append(entry)
    save_error_logs()
    socketio.emit('new_error', entry)
    return entry

def init_history():
    """啟動時載入歷史紀錄與錯誤日誌至記憶體緩衝區"""
    global error_seq
    if os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, 'r', encoding='utf-8') as f:
                entries = json.load(f)
                if isinstance(entries, list):
                    for item in entries[-250:]:
                        history_buffer.append(item)
        except Exception as e:
            print(f"[WARN] Failed to load history: {e}")

    loaded_errors = False
    if os.path.exists(ERROR_LOG_FILE):
        try:
            with open(ERROR_LOG_FILE, 'r', encoding='utf-8') as f:
                err_entries = json.load(f)
                if isinstance(err_entries, list) and len(err_entries) > 0:
                    max_id = 0
                    for item in err_entries[-500:]:
                        error_buffer.append(item)
                        if int(item.get('id', 0)) > max_id:
                            max_id = int(item.get('id', 0))
                    error_seq = max_id + 1
                    loaded_errors = True
        except Exception as e:
            print(f"[WARN] Failed to load error_log.json: {e}")

    # 若尚無獨立 error_log.json，從既有 history_buffer 自動匯入 error 與 warn 紀錄
    if not loaded_errors:
        for h in list(history_buffer):
            htype = str(h.get('type', '')).lower()
            if htype in ('error', 'warn'):
                lvl = 'ERROR' if htype == 'error' else 'WARN'
                src, code = _classify_error_meta(h.get('msg', ''), lvl)
                error_buffer.append({
                    'id': error_seq,
                    'time': h.get('time', datetime.now().strftime('%Y-%m-%d %H:%M:%S')),
                    'level': lvl,
                    'source': src,
                    'code': code,
                    'msg': h.get('msg', ''),
                    'detail': '歷史日誌匯入紀錄',
                    'resolved': False
                })
                error_seq += 1

init_history()

def load_history():
    with history_lock:
        return list(history_buffer)

def save_history(history_list=None):
    """批次刷入磁碟（僅在背景定時或重大錯誤時寫入，保護 SD 卡壽命）"""
    global history_dirty
    with history_lock:
        if history_list is not None and isinstance(history_list, list):
            history_buffer.clear()
            for item in history_list[-250:]:
                history_buffer.append(item)
        data_to_write = list(history_buffer)
        history_dirty = False

    try:
        with open(HISTORY_FILE, 'w', encoding='utf-8') as f:
            json.dump(data_to_write, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[ERROR] Failed to save history: {e}")

def add_history(msg, type='info'):
    """新增日誌記錄（即時寫入 RAM 緩衝區並透過 WebSocket 廣播，杜絕 SD 卡磨損）"""
    global history_dirty
    time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    log_entry = {
        'time': time_str,
        'msg': msg,
        'type': type
    }
    with history_lock:
        history_buffer.append(log_entry)
        history_dirty = True

    socketio.emit('new_history', log_entry)

    # 若為 error 或 warn，自動同步寫入錯誤日誌中心
    if type in ('error', 'warn'):
        lvl = 'ERROR' if type == 'error' else 'WARN'
        add_error_log(lvl, None, None, msg, None)

    # 重大錯誤 (error) 時才立即安全刷盤存檔
    if type == 'error':
        save_history()

def load_macros():
    if os.path.exists(MACROS_FILE):
        try:
            with open(MACROS_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}
    return {
        '自動搬運範例': [
            '# 歸零與準備',
            'HOME',
            'DELAY 1000',
            '# 移動至取料點',
            'MOVE X 2000',
            'MOVE Y 1500',
            'DELAY 500',
            '# 下降並吸取',
            'ARM DOWN',
            'DELAY 1000',
            'VACUUM ON',
            'DELAY 500',
            'ARM UP',
            'DELAY 1000',
            '# 移動至放料點',
            'MOVE X 1000',
            'MOVE Y 1000',
            'DELAY 500',
            '# 下降並釋放',
            'ARM DOWN',
            'DELAY 1000',
            'VACUUM OFF',
            'DELAY 500',
            'ARM UP',
            'DELAY 1000',
            '# 完成並回原點',
            'HOME',
            'MSG 搬運任務已完成！'
        ]
    }

def save_macros(macros_dict):
    try:
        with open(MACROS_FILE, 'w', encoding='utf-8') as f:
            json.dump(macros_dict, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[ERROR] Failed to save macros: {e}")

macro_thread = None
macro_running = False
macro_paused = False

def _wait_macro_motor_target(axis, target_pulse):
    """輔助函式：在巨集/積木執行中等待 X 或 Z 軸到達目標脈衝位置"""
    st = x_stepper if axis == 'x' else (z_stepper or y_stepper)
    min_l = state['x_min'] if axis == 'x' else state.get('z_min', state['y_min'])
    max_l = state['x_max'] if axis == 'x' else state.get('z_max', state['y_max'])
    clamped_target = max(min_l, min(max_l, target_pulse))

    t_move_start = time.time()
    while macro_running and not macro_paused:
        if st is None:
            break
        if abs(st.current_pos - clamped_target) < 10:
            break
        # 限位觸發時及時跳出，防止死等卡死
        if axis == 'x' and ((clamped_target < st.current_pos and (state.get('lim_x_a') or state.get('lim_left'))) or (clamped_target > st.current_pos and (state.get('lim_x_c') or state.get('lim_right')))):
            break
        if axis in ('z', 'y', 'y1', 'y2') and ((clamped_target < st.current_pos and (state.get('lim_z_e') or state.get('lim_down'))) or (clamped_target > st.current_pos and (state.get('lim_z_d') or state.get('lim_up')))):
            break
        if time.time() - t_move_start > 15.0:
            add_history(f"巨集警告: 移動至 {target_pulse:.0f}µs 到位超時", "warn")
            break
        time.sleep(0.05)

def run_macro_loop(commands):
    global macro_running, macro_paused
    macro_running = True
    macro_paused = False
    state['system_run'] = True
    
    add_history("🧩 積木/巨集程式執行開始", "success")
    socketio.emit('macro_status', {'running': True, 'paused': False, 'current_line': 0})
    
    loop_stack = []  # 儲存 [{'start_idx': int, 'remaining': int}]
    idx = 0
    while idx < len(commands) and macro_running:
        if macro_paused:
            time.sleep(0.1)
            continue
            
        cmd = commands[idx].strip()
        idx += 1
        socketio.emit('macro_status', {'running': True, 'paused': False, 'current_line': idx})
        
        if not cmd or cmd.startswith('#') or cmd.startswith('//'):
            continue
            
        parts = cmd.split()
        op = parts[0].upper()
        
        if op == 'REPEAT':
            cnt = 2
            if len(parts) >= 2:
                try:
                    cnt = max(1, min(999, int(parts[1])))
                except ValueError:
                    cnt = 2
            loop_stack.append({'start_idx': idx, 'remaining': cnt - 1, 'total': cnt})
            add_history(f"🔁 積木迴圈開始: 重複執行 {cnt} 次", "info")
            continue

        elif op in ('END', 'END_REPEAT'):
            if loop_stack:
                top = loop_stack[-1]
                if top['remaining'] > 0:
                    top['remaining'] -= 1
                    done_cnt = top['total'] - top['remaining']
                    add_history(f"🔁 迴圈第 {done_cnt}/{top['total']} 次開始...", "info")
                    idx = top['start_idx']
                    continue
                else:
                    loop_stack.pop()
                    add_history("🔁 積木迴圈結束", "info")
            continue

        elif op == 'MOVE':
            if len(parts) >= 3:
                axis = parts[1].lower()
                try:
                    val_str = parts[2].lower()
                    if val_str.endswith('mm'):
                        mm_val = float(val_str[:-2])
                        scale = state['x_scale'] if axis == 'x' else state.get('z_scale', state['y_scale'])
                        base_min = state['x_min'] if axis == 'x' else state.get('z_min', state['y_min'])
                        target = base_min + mm_val * scale
                        set_servo_pulse(axis, target)
                        add_history(f"巨集指令: 移動 {axis.upper()} 至 {mm_val:.1f} mm ({target:.0f} µs)", "info")
                    else:
                        target = float(val_str)
                        set_servo_pulse(axis, target)
                        add_history(f"巨集指令: 移動 {axis.upper()} 至 {target:.0f} µs", "info")
                    _wait_macro_motor_target(axis, target)
                except ValueError:
                    add_history(f"巨集錯誤: 無效的移動座標 '{parts[2]}'", "error")
            else:
                add_history("巨集錯誤: MOVE 指令參數不足", "error")

        elif op == 'MOVE_MM':
            if len(parts) >= 3:
                axis = parts[1].lower()
                try:
                    mm_val = float(parts[2])
                    scale = state['x_scale'] if axis == 'x' else state.get('z_scale', state['y_scale'])
                    base_min = state['x_min'] if axis == 'x' else state.get('z_min', state['y_min'])
                    target = base_min + mm_val * scale
                    set_servo_pulse(axis, target)
                    add_history(f"🧩 積木移動: {axis.upper()} 軸前往 {mm_val:.1f} mm ({target:.0f} µs)", "info")
                    _wait_macro_motor_target(axis, target)
                except ValueError:
                    add_history(f"巨集錯誤: 無效的 mm 座標 '{parts[2]}'", "error")
            else:
                add_history("巨集錯誤: MOVE_MM 指令參數不足", "error")

        elif op == 'GOTO':
            if len(parts) >= 2:
                pt = parts[1].upper()
                z_min = float(state.get('z_min', state['y_min']))
                z_max = float(state.get('z_max', state['y_max']))
                point_map = {
                    'X_A': ('x', float(state['x_min']), 'X軸 a點 (0mm 歸零原點)'),
                    'X_B': ('x', float((state['x_min'] + state['x_max']) / 2.0), 'X軸 b點 (100mm 中間模具工位)'),
                    'X_C': ('x', float(state['x_max']), 'X軸 c點 (200mm 右極限放料位)'),
                    'Z_D': ('z', z_max, 'Z軸 d點 (200mm 向上安全高度)'),
                    'Z_E': ('z', z_min, 'Z軸 e點 (0mm 向下取放高度)'),
                    'Z_HALF': ('z', float((z_min + z_max) / 2.0), 'Z軸 50% (100mm 避讓高度)'),
                }
                if pt in point_map:
                    axis, target, label = point_map[pt]
                    set_servo_pulse(axis, target)
                    add_history(f"🧩 積木定位: 前往【{label}】", "info")
                    _wait_macro_motor_target(axis, target)
                else:
                    add_history(f"巨集錯誤: 未知的工位點 '{pt}'", "error")
            else:
                add_history("巨集錯誤: GOTO 指令參數不足", "error")

        elif op == 'STEP':
            if len(parts) >= 3:
                axis = parts[1].lower()
                try:
                    delta_mm = float(parts[2])
                    scale = state['x_scale'] if axis == 'x' else state.get('z_scale', state['y_scale'])
                    st = x_stepper if axis == 'x' else (z_stepper or y_stepper)
                    cur_pos = st.current_pos if st else (state['x_pulse'] if axis == 'x' else state['z_pulse'])
                    target = cur_pos + delta_mm * scale
                    set_servo_pulse(axis, target)
                    dir_str = f"+{delta_mm:.1f}" if delta_mm >= 0 else f"{delta_mm:.1f}"
                    add_history(f"🧩 積木相對移動: {axis.upper()} 軸 {dir_str} mm", "info")
                    _wait_macro_motor_target(axis, target)
                except ValueError:
                    add_history(f"巨集錯誤: 無效的步進距離 '{parts[2]}'", "error")
            else:
                add_history("巨集錯誤: STEP 指令參數不足", "error")
                
        elif op == 'DELAY':
            if len(parts) >= 2:
                try:
                    ms = float(parts[1])
                    add_history(f"巨集指令: 延遲 {ms:.0f} 毫秒", "info")
                    elapsed = 0.0
                    while elapsed < (ms / 1000.0) and macro_running:
                        if not macro_paused:
                            time.sleep(0.05)
                            elapsed += 0.05
                        else:
                            time.sleep(0.1)
                except ValueError:
                    add_history(f"巨集錯誤: 無效的延遲時間 '{parts[1]}'", "error")
            else:
                add_history("巨集錯誤: DELAY 指令參數不足", "error")
                
        elif op in ('MAGNET', 'VACUUM'):
            if len(parts) >= 3 and parts[1].isdigit() and int(parts[1]) in (1, 2, 3, 4):
                g = int(parts[1])
                val = parts[2].upper() == 'ON'
                set_digital_output(f'magnet_{g}', val)
                add_history(f"巨集指令: 第 {g} 組電磁鐵 (M{g}) {'通電 ON' if val else '斷電 OFF'}", "info")
                time.sleep(0.4)
            elif len(parts) >= 3 and parts[1].upper() == 'ALL':
                val = parts[2].upper() == 'ON'
                set_digital_output('magnet', val)
                add_history(f"巨集指令: 4組電磁鐵吸盤 (M1~M4) {'全部通電 ON' if val else '全部斷電 OFF'}", "info")
                time.sleep(0.4)
            elif len(parts) >= 2:
                val = parts[1].upper() == 'ON'
                set_digital_output('magnet', val)
                add_history(f"巨集指令: 4組電磁鐵吸盤 (M1~M4) {'全部通電 ON' if val else '全部斷電 OFF'}", "info")
                time.sleep(0.4)
            else:
                add_history(f"巨集錯誤: {op} 指令參數不足", "error")

        elif op == 'CHECK':
            if len(parts) >= 2 and parts[1].upper() == 'METAL':
                read_limit_switches()
                if not state.get('hall_arm_metal'):
                    add_history("巨集錯誤: 末端霍爾感測陣列 (8顆/4組) 未檢測到金屬工件！", "error")
                    set_digital_output('magnet', False)
                    break
                else:
                    add_history(f"巨集檢測: 末端霍爾感測金屬工件正常 ({state.get('hall_active_count', 0)}/8 顆觸發)！", "success")
            elif len(parts) >= 3 and parts[1].upper() == 'GROUP' and parts[2].isdigit() and int(parts[2]) in (1, 2, 3, 4):
                g = int(parts[2])
                read_limit_switches()
                if not state.get(f'hall_group_{g}'):
                    add_history(f"巨集錯誤: 第 {g} 組霍爾開關 (G{g}) 未檢測到金屬工件！", "error")
                    set_digital_output('magnet', False)
                    break
                else:
                    add_history(f"巨集檢測: 第 {g} 組霍爾開關 (G{g}) 金屬工件確認在位！", "success")
            else:
                add_history("巨集錯誤: CHECK 指令格式為 CHECK METAL 或 CHECK GROUP 1~4", "error")

        elif op == 'WAIT':
            if len(parts) >= 2 and parts[1].upper() == 'PRESS':
                add_history("巨集指令: 等待衝壓模具完成訊號...", "info")
                t_w = time.time()
                while macro_running and not macro_paused:
                    read_limit_switches()
                    if state.get('press_done') or (not IS_RASPBERRY_PI and (time.time() - t_w >= 1.5)):
                        break
                    if time.time() - t_w > 15.0:
                        add_history("巨集警告: 等待衝壓完成超時", "warn")
                        break
                    time.sleep(0.1)
            else:
                add_history("巨集錯誤: WAIT 指令格式為 WAIT PRESS", "error")
                
        elif op == 'ARM':
            if len(parts) >= 2:
                val = parts[1].upper() == 'DOWN'
                set_digital_output('z_down', val)
                add_history(f"巨集指令: 手臂 {'下降' if val else '上升'}", "info")
                time.sleep(0.5)
            else:
                add_history("巨集錯誤: ARM 指令參數不足", "error")
                
        elif op == 'HOME':
            axis = 'all'
            if len(parts) >= 2:
                axis = parts[1].lower()
            add_history(f"巨集指令: 馬達歸零 ({axis.upper()})", "info")
            home_axis(axis)
            
        elif op == 'MSG':
            msg = " ".join(parts[1:])
            add_history(f"巨集提示: {msg}", "success")
            socketio.emit('macro_msg', {'msg': msg})

        elif op in ('G0', 'G00', 'G1', 'G01'):
            target_x = None
            target_z = None
            for tok in parts[1:]:
                t_up = tok.upper()
                if t_up.startswith('X'):
                    try: target_x = float(t_up[1:])
                    except ValueError: pass
                elif t_up.startswith('Y') or t_up.startswith('Z'):
                    try: target_z = float(t_up[1:])
                    except ValueError: pass
            if target_x is not None:
                px = (state['x_min'] + target_x * state['x_scale']) if target_x <= 400.0 else target_x
                set_servo_pulse('x', px)
                _wait_motor_target('x', px)
            if target_z is not None:
                z_min = state.get('z_min', state['y_min'])
                z_sc = state.get('z_scale', state['y_scale'])
                pz = (z_min + target_z * z_sc) if target_z <= 400.0 else target_z
                set_servo_pulse('z', pz)
                _wait_motor_target('z', pz)

        elif op == 'G28':
            add_history("G-Code G28: 執行雙軸安全歸零", "info")
            home_axis('all')

        elif op in ('G4', 'G04'):
            ms = 500.0
            for tok in parts[1:]:
                t_up = tok.upper()
                if t_up.startswith('P'):
                    try: ms = float(t_up[1:])
                    except ValueError: pass
                elif t_up.startswith('S'):
                    try: ms = float(t_up[1:]) * 1000.0
                    except ValueError: pass
            time.sleep(max(0.0, ms / 1000.0))

        elif op in ('M3', 'M4', 'M106'):
            set_digital_output('magnet', True)
            add_history("G-Code: 電磁鐵通電吸附 ON", "info")
            time.sleep(0.3)

        elif op in ('M5', 'M107'):
            set_digital_output('magnet', False)
            add_history("G-Code: 電磁鐵斷電釋放 OFF", "info")
            time.sleep(0.3)

        else:
            if not op.startswith(';') and not op.startswith('('):
                add_history(f"巨集錯誤: 未知指令 '{op}'", "error")
            
    macro_running = False
    socketio.emit('macro_status', {'running': False, 'paused': False, 'current_line': 0})
    add_history("🧩 積木/巨集程式執行結束", "success")

# ── REST API ──────────────────────────────────────────────────────
@app.route('/api/start', methods=['POST'])
def api_start():
    state['system_run'] = True
    state['error'] = False
    state['error_msg'] = ''
    add_history("系統啟動", "success")
    return jsonify({'ok': True})

@app.route('/api/stop', methods=['POST'])
def api_stop():
    global macro_running
    state['system_run'] = False
    state['pickup_active'] = False
    macro_running = False
    homing_abort_event.set()
    # E-STOP：立即將步進馬達目標位置設為當前位置，強制煞車並解除歸零暫停
    if x_stepper:
        x_stepper.pause_for_homing = False
        x_stepper.halt()
    if z_stepper:
        z_stepper.pause_for_homing = False
        z_stepper.halt()
    if y_stepper:
        y_stepper.pause_for_homing = False
        y_stepper.halt()
    set_digital_output('magnet', False)
    set_digital_output('vacuum', False)
    set_digital_output('z_down', False)
    add_history("系統緊急停止（斷開電磁鐵吸盤＋強制雙軸煞車）", "warn")
    return jsonify({'ok': True})

def is_motion_busy():
    if state.get('is_homing', False):
        return '目前正在執行馬達歸零程序，請等待歸零完成'
    if state.get('pickup_active', False):
        return '目前 14 步衝壓搬運自動循環正在運行中，請先停止循環'
    if macro_running:
        return '目前積木程式 / G-code 巨集正在執行中，請先停止巨集'
    return None

@app.route('/api/home', methods=['POST'])
def api_home():
    busy_msg = is_motion_busy()
    if busy_msg:
        return jsonify({'ok': False, 'msg': busy_msg})
    data = request.get_json(silent=True) or {}
    axis = str(data.get('axis', 'all')).lower()
    add_history(f"手動點擊馬達歸零 ({axis.upper()})", "info")
    state['is_homing'] = True
    t = threading.Thread(target=home_axis, args=(axis,), daemon=True)
    t.start()
    return jsonify({'ok': True, 'msg': f'歸零中: {axis}'})

@app.route('/api/magnet', methods=['POST'])
@app.route('/api/vacuum', methods=['POST'])
def api_magnet():
    """控制 4 組圓形電磁鐵強吸力吸盤 LSD-P25/20 (支援全組同步或單組 M1~M4 獨立控制)"""
    data = request.get_json(silent=True) or {}
    group = data.get('group')
    value = bool(data.get('on', False))
    if group in (1, 2, 3, 4, '1', '2', '3', '4'):
        g = int(group)
        set_digital_output(f'magnet_{g}', value)
        add_history(f"手動切換第 {g} 組電磁鐵 (M{g}): {'通電吸附 ON' if value else '斷電釋放 OFF'}", "info")
    else:
        set_digital_output('magnet', value)
        add_history(f"手動切換全部 4 組電磁鐵 (M1~M4): {'通電吸附 ON' if value else '斷電釋放 OFF'}", "info")
    return jsonify({
        'ok': True,
        'magnet': state['magnet'],
        'magnet_1': state['magnet_1'],
        'magnet_2': state['magnet_2'],
        'magnet_3': state['magnet_3'],
        'magnet_4': state['magnet_4'],
        'magnet_active_count': state['magnet_active_count'],
        'vacuum': state['vacuum']
    })

@app.route('/api/z_down', methods=['POST'])
def api_z_down():
    data = request.get_json(silent=True) or {}
    value = bool(data.get('on', False))
    set_digital_output('z_down', value)
    state['arm_down'] = value
    add_history(f"手動切換手臂: {'下降' if value else '上升'}", "info")
    return jsonify({'ok': True, 'z_down': state['z_down']})

@app.route('/api/stamping_cycle', methods=['POST'])
@app.route('/api/pickup', methods=['POST'])
def api_pickup():
    """執行完整 14 步衝壓搬運自動化循環 (Stamping Automation Cycle)
    運動模式流程：
    1. X軸回到a點 (歸零)
    2. Z軸下去 (e點向下極限)
    3. 圓形電磁鐵通電吸料
    4. Z軸上去 (d點向上歸零)
    5. 手臂霍爾感測金屬是否存在 (安全防呆檢查)
    6. X軸到c點 (右極限)
    7. Z軸下去 (e點向下極限)
    8. 圓形電磁鐵關電釋放工件
    9. Z軸上去 (d點向上歸零)
    10. X軸到b點 (中間模具工位)
    11. Z軸下去一半 (50% 避讓高度)
    12. 等模具衝壓完 (PRESS_DONE 訊號)
    13. Z軸上去 (d點向上歸零)
    14. X軸回到a點，一個 cycle 完成！
    """
    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動，請先按下 START PB'})
    busy_msg = is_motion_busy()
    if busy_msg:
        return jsonify({'ok': False, 'msg': busy_msg})

    def _wait_motor_reach(motor, target, tolerance=15, timeout=12.0, require_metal=False):
        start = time.time()
        while state['pickup_active'] and state['system_run']:
            if require_metal:
                read_limit_switches()
                if not state.get('hall_arm_metal'):
                    if x_stepper: x_stepper.halt()
                    if z_stepper: z_stepper.halt()
                    set_digital_output('magnet', False)
                    msg = "搬運途中檢測到金屬工件脫落 (霍爾感測訊號中斷)，緊急煞車停機！"
                    state['error'] = True
                    state['error_msg'] = msg
                    state['pickup_active'] = False
                    add_error_log('CRITICAL', 'HALL', 'ERR_HALL_IN_FLIGHT_DROP', msg)
                    add_history(f"❌ 緊急警報：{msg}", "error")
                    return False
            if motor is None or abs(motor.current_pos - target) <= tolerance:
                return True
            if time.time() - start > timeout:
                add_history(f"到位警告：馬達到位超時（目標={target:.0f}µs，當前={motor.current_pos:.0f}µs）", "warn")
                return False
            time.sleep(0.03)
        return False

    def stamping_cycle_sequence():
        state['pickup_active'] = True
        mode_str = "連續生產模式" if state.get('continuous_mode') else "單次循環模式"
        add_history(f"🚀 衝壓自動化取放料開始運轉 ({mode_str}，目標: {state['cycle_target']} 次)", "success")
        try:
            if not state.get('x_homed') or not state.get('z_homed'):
                add_history("⚠️ 啟動前原點安全互鎖：雙軸尚未校準原點，自動執行雙軸安全歸零...", "info")
                home_axis('all')
            st_x = x_stepper
            st_z = z_stepper or y_stepper
            pos_x_a = float(state['x_min'])                                  # a 點 (歸零)
            pos_x_b = float((state['x_min'] + state['x_max']) / 2.0)        # b 點 (中間模具位)
            pos_x_c = float(state['x_max'])                                  # c 點 (右極限)
            pos_z_d = float(state.get('z_max', state['y_max']))              # d 點 (向上歸零/安全高度)
            pos_z_e = float(state.get('z_min', state['y_min']))              # e 點 (向下極限/取放深度)
            pos_z_half = float((pos_z_d + pos_z_e) / 2.0)                    # 下去一半 (50%)

            while state['pickup_active'] and state['system_run']:
                # 步驟 1: X 軸回到 a 點
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'home1', 'desc': '1. X軸回到a點(歸零)'})
                add_history("步驟 1/14: X 軸前往 a 點 (歸零點)...", "info")
                set_servo_pulse('x', pos_x_a)
                if not _wait_motor_reach(st_x, pos_x_a): break
                # [開環失步即時校正]: 若接近開關 a 已感應觸發，自動重置校準坐標
                if state.get('lim_x_a') and st_x:
                    st_x.set_position(pos_x_a)
                time.sleep(0.2)

                # 步驟 2: Z 軸下去 (到達 e 點)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'move-mold1', 'desc': '2. Z軸下降至e點'})
                add_history("步驟 2/14: Z 軸下降至 e 點 (向下極限取料面)...", "info")
                set_servo_pulse('z', pos_z_e)
                if not _wait_motor_reach(st_z, pos_z_e): break
                time.sleep(0.2)

                # 步驟 3: 圓形電磁鐵通電吸料
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'suck', 'desc': '3. 電磁鐵通電吸料'})
                add_history("步驟 3/14: 圓形電磁鐵通電，強磁力吸附金屬工件", "info")
                set_digital_output('magnet', True)
                time.sleep(0.4)

                # 步驟 4: Z 軸上去 (到達 d 點)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'check-mold', 'desc': '4. Z軸上升至d點'})
                add_history("步驟 4/14: Z 軸上升至 d 點 (安全高度)...", "info")
                set_servo_pulse('z', pos_z_d)
                if not _wait_motor_reach(st_z, pos_z_d): break
                # [Z軸失步校準]: 若頂部接近開關 d 觸發，對齊頂部坐標
                if state.get('lim_z_d') and st_z:
                    st_z.set_position(pos_z_d)
                time.sleep(0.2)

                # 步驟 5: 末端 8 顆霍爾感測陣列 (兩個一組共 4 組) 檢測金屬是否存在 (安全防呆檢查)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'check-suck', 'desc': '5. 8顆霍爾(4組)金屬檢測'})
                add_history("步驟 5/14: 讀取末端 8 顆霍爾感測陣列 (4 組雙重確認)，驗證金屬是否吸附...", "info")
                read_limit_switches()
                if not state.get('hall_arm_metal'):
                    add_history("❌ 警報：末端 8 顆霍爾感測器 (G1~G4) 皆未檢測到金屬工件！取料失敗，緊急中斷！", "error")
                    set_digital_output('magnet', False)
                    state['error'] = True
                    state['error_msg'] = "末端霍爾感測陣列 (8顆/4組) 未檢測到金屬工件，自動循環中止"
                    return
                add_history(f"✔ 霍爾感測確認：金屬工件吸附正常 ({state.get('hall_active_count', 8)}/8 顆觸發，{state.get('hall_group_count', 4)}/4 組就緒)！", "success")
                time.sleep(0.2)

                # 步驟 6: X 軸到 c 點 (向右極限/放料點，搬運途中即時監控防掉件)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'move-mold2', 'desc': '6. X軸前往c點'})
                add_history("步驟 6/14: X 軸移動至 c 點 (向右極限放料點，全程監控防掉件)...", "info")
                set_servo_pulse('x', pos_x_c)
                if not _wait_motor_reach(st_x, pos_x_c, require_metal=True): break
                time.sleep(0.2)

                # 步驟 7: Z 軸下去 (到達 e 點，搬運途中即時監控防掉件)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'drop', 'desc': '7. Z軸下降至e點'})
                add_history("步驟 7/14: Z 軸下降至 e 點 (放料深度)...", "info")
                set_servo_pulse('z', pos_z_e)
                if not _wait_motor_reach(st_z, pos_z_e, require_metal=True): break
                time.sleep(0.2)

                # 步驟 8: 圓形電磁鐵關電 (釋放工件)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'check-drop', 'desc': '8. 電磁鐵關電釋放'})
                add_history("步驟 8/14: 圓形電磁鐵斷電，工件已釋放", "info")
                set_digital_output('magnet', False)
                time.sleep(0.4)

                # 步驟 9: Z 軸上去 (到達 d 點)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'home2', 'desc': '9. Z軸上升至d點'})
                add_history("步驟 9/14: Z 軸上升至 d 點 (安全高度)...", "info")
                set_servo_pulse('z', pos_z_d)
                if not _wait_motor_reach(st_z, pos_z_d): break
                if state.get('lim_z_d') and st_z:
                    st_z.set_position(pos_z_d)
                time.sleep(0.2)

                # 步驟 10: X 軸到 b 點 (中間模具工位接近開關)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'move-b', 'desc': '10. X軸前往b點'})
                add_history("步驟 10/14: X 軸移動至 b 點 (中間電感接近開關工位)...", "info")
                set_servo_pulse('x', pos_x_b)
                if not _wait_motor_reach(st_x, pos_x_b): break
                time.sleep(0.2)

                # 步驟 11: Z 軸下去一半 (50% 避讓高度)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'half-down', 'desc': '11. Z軸下去一半'})
                add_history("步驟 11/14: Z 軸下降至 50% 中間避讓高度...", "info")
                set_servo_pulse('z', pos_z_half)
                if not _wait_motor_reach(st_z, pos_z_half): break
                time.sleep(0.2)

                # 步驟 12: 等模具衝壓完
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'wait-press', 'desc': '12. 等待衝壓完畢'})
                add_history("步驟 12/14: 等候模具衝壓完畢訊號 (PRESS_DONE)...", "info")
                t_press_start = time.time()
                while state['pickup_active'] and state['system_run']:
                    read_limit_switches()
                    if state.get('press_done') or (not IS_RASPBERRY_PI and (time.time() - t_press_start >= 1.5)):
                        break
                    if time.time() - t_press_start > 15.0:
                        add_history("衝壓等待警告：模具訊號超時，自動繼續", "warn")
                        break
                    time.sleep(0.1)
                add_history("✔ 模具衝壓完成！", "success")
                time.sleep(0.2)

                # 步驟 13: Z 軸上去 (到達 d 點)
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'up-again', 'desc': '13. Z軸上去d點'})
                add_history("步驟 13/14: Z 軸上升至 d 點 (安全高度)...", "info")
                set_servo_pulse('z', pos_z_d)
                if not _wait_motor_reach(st_z, pos_z_d): break
                if state.get('lim_z_d') and st_z:
                    st_z.set_position(pos_z_d)
                time.sleep(0.2)

                # 步驟 14: X 軸回到 a 點
                if not state['system_run']: break
                socketio.emit('pickup_step', {'step': 'cycle-done', 'desc': '14. X軸回a點完成'})
                add_history("步驟 14/14: X 軸回到 a 點 (原點)...", "info")
                set_servo_pulse('x', pos_x_a)
                if not _wait_motor_reach(st_x, pos_x_a): break
                if state.get('lim_x_a') and st_x:
                    st_x.set_position(pos_x_a)

                # ── 單一循環圓滿完成，更新生產計數 ──────────────────
                state['cycle_count'] += 1
                socketio.emit('cycle_update', {
                    'cycle_count': state['cycle_count'],
                    'cycle_target': state['cycle_target'],
                    'continuous_mode': state['continuous_mode']
                })
                add_history(f"🎉 衝壓循環第 {state['cycle_count']} 次完成！", "success")

                # 若非連續模式，直接跳出結束
                if not state.get('continuous_mode', False):
                    break

                # 若已達成設定之目標生產件數，自動停止
                if state.get('cycle_target', 0) > 0 and state['cycle_count'] >= state['cycle_target']:
                    add_history(f"🏁 已達設定目標生產量 ({state['cycle_target']} 次)，連續循環自動安全停機！", "success")
                    break

                # 連續模式：等待間隔秒數，快速響應取消中斷
                interval = max(0.1, float(state.get('cycle_interval_sec', 1.0)))
                t_delay_start = time.time()
                while time.time() - t_delay_start < interval:
                    if not (state['pickup_active'] and state['system_run']):
                        break
                    time.sleep(0.05)

            add_history(f"衝壓搬運流程就緒 (累計完成: {state['cycle_count']} 次)", "info")
        except Exception as e:
            state['error'] = True
            state['error_msg'] = str(e)
            add_history(f"自動衝壓循環異常: {e}", "error")
        finally:
            set_digital_output('magnet', False)
            state['pickup_active'] = False
            socketio.emit('pickup_step', {'step': 'idle'})

    t = threading.Thread(target=stamping_cycle_sequence, daemon=True)
    t.start()
    return jsonify({'ok': True, 'msg': '自動衝壓搬運循環啟動', 'continuous_mode': state['continuous_mode']})

@app.route('/api/continuous_mode', methods=['POST'])
def api_continuous_mode():
    """設定連續生產運轉模式與循環參數"""
    data = request.get_json(silent=True) or {}
    if 'enabled' in data:
        state['continuous_mode'] = bool(data['enabled'])
    if 'target' in data:
        state['cycle_target'] = max(0, int(data['target']))
    if 'interval' in data:
        state['cycle_interval_sec'] = max(0.1, min(60.0, float(data['interval'])))
    mode_text = '啟用連續生產模式' if state['continuous_mode'] else '關閉連續生產模式'
    add_history(f"{mode_text} (目標={state['cycle_target']}次, 間隔={state['cycle_interval_sec']}s)", "info")
    return jsonify({
        'ok': True,
        'continuous_mode': state['continuous_mode'],
        'cycle_target': state['cycle_target'],
        'cycle_interval_sec': state['cycle_interval_sec'],
        'cycle_count': state['cycle_count']
    })

@app.route('/api/reset_cycle_count', methods=['POST'])
def api_reset_cycle_count():
    """重置生產次數計數器"""
    state['cycle_count'] = 0
    add_history("生產累計計數器已歸零", "info")
    return jsonify({'ok': True, 'cycle_count': 0})

LIMITS_CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'limits_config.json')

def save_limits_config():
    """將行程極限與速度比例設定持久化存檔，重啟後自動載入"""
    cfg = {
        'x_min': state['x_min'], 'x_max': state['x_max'],
        'z_min': state['z_min'], 'z_max': state['z_max'],
        'y_min': state['y_min'], 'y_max': state['y_max'],
        'x_min_mm': state.get('x_min_mm', 0.0), 'x_max_mm': state.get('x_max_mm', 200.0),
        'z_min_mm': state.get('z_min_mm', 0.0), 'z_max_mm': state.get('z_max_mm', 200.0),
        'y_min_mm': state.get('y_min_mm', 0.0), 'y_max_mm': state.get('y_max_mm', 200.0),
        'x_scale': state['x_scale'], 'z_scale': state.get('z_scale', state['y_scale']), 'y_scale': state['y_scale'],
        'x_max_speed': state['x_max_speed'], 'x_accel_steps': state['x_accel_steps'],
        'z_max_speed': state['z_max_speed'], 'z_accel_steps': state['z_accel_steps'],
        'y_max_speed': state['y_max_speed'], 'y_accel_steps': state['y_accel_steps'],
        'homing_speed_hz': state['homing_speed_hz'],
        'y_sync': state['y_sync'],
    }
    try:
        with open(LIMITS_CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[WARN] Failed to save limits_config.json: {e}")

def load_limits_config():
    if os.path.exists(LIMITS_CONFIG_FILE):
        try:
            with open(LIMITS_CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
                if isinstance(cfg, dict):
                    for k in ('x_min', 'x_max', 'z_min', 'z_max', 'y_min', 'y_max',
                              'x_min_mm', 'x_max_mm', 'z_min_mm', 'z_max_mm', 'y_min_mm', 'y_max_mm',
                              'x_scale', 'z_scale', 'y_scale',
                              'x_max_speed', 'x_accel_steps', 'z_max_speed', 'z_accel_steps',
                              'y_max_speed', 'y_accel_steps', 'homing_speed_hz', 'y_sync'):
                        if k in cfg:
                            state[k] = cfg[k]
        except Exception as e:
            print(f"[WARN] Failed to load limits_config.json: {e}")

load_limits_config()

@app.route('/api/set_limits', methods=['POST'])
@app.route('/api/settings', methods=['POST'])
def api_set_limits():
    """設定馬達行程範圍（支援 mm 與 us）、速度與校準參數"""
    data = request.get_json(silent=True) or {}

    if 'hall_active_high' in data:
        state['hall_active_high'] = bool(data['hall_active_high'])
    
    # 優先更新比例參數（避免換算 mm 極限時使用舊 scale 或被 0 除）
    if 'x_scale' in data: 
        try:
            state['x_scale'] = max(0.001, float(data['x_scale']))
        except (ValueError, TypeError):
            pass
    if 'y_scale' in data or 'z_scale' in data: 
        try:
            sc = max(0.001, float(data.get('z_scale', data.get('y_scale'))))
            state['z_scale'] = sc
            state['y_scale'] = sc
        except (ValueError, TypeError):
            pass

    # 支援 mm 與 µs 設定 X 軸（支援 0 起算與閉迴路大行程脈衝，最高可設 200,000µs）
    if 'x_min' in data or 'x_max' in data:
        if 'x_min' in data:
            state['x_min'] = max(0, min(SYSTEM_PULSE_MAX, int(round(float(data['x_min'])))))
        if 'x_max' in data:
            state['x_max'] = max(state['x_min'] + 1, min(SYSTEM_PULSE_MAX, int(round(float(data['x_max'])))))
        if state['x_max'] <= state['x_min']:
            state['x_max'] = min(SYSTEM_PULSE_MAX, state['x_min'] + 1)
        state['x_min_mm'] = 0.0 if state['x_min'] in (0, PULSE_MIN) else round(state['x_min'] / max(0.001, state['x_scale']), 1)
        state['x_max_mm'] = round(state['x_min_mm'] + (state['x_max'] - state['x_min']) / max(0.001, state['x_scale']), 1)
    elif 'x_min_mm' in data or 'x_max_mm' in data:
        min_mm = max(0.0, float(data.get('x_min_mm', state.get('x_min_mm', 0.0))))
        max_mm = float(data.get('x_max_mm', state.get('x_max_mm', 200.0)))
        if max_mm <= min_mm:
            max_mm = min_mm + 1.0
        state['x_min_mm'] = round(min_mm, 1)
        state['x_max_mm'] = round(max_mm, 1)
        base_us = PULSE_MIN if (state['x_min'] == PULSE_MIN and min_mm == 0.0) else 0.0
        state['x_min'] = max(0, min(SYSTEM_PULSE_MAX, int(round(base_us + min_mm * state['x_scale']))))
        state['x_max'] = max(state['x_min'] + 1, min(SYSTEM_PULSE_MAX, int(round(base_us + max_mm * state['x_scale']))))

    # 若 X 軸當前位置超出新設定之範圍，安全夾限至範圍內
    if x_stepper:
        clamped_x = max(float(state['x_min']), min(float(state['x_max']), float(x_stepper.current_pos)))
        if abs(clamped_x - x_stepper.current_pos) > 0.5:
            x_stepper.set_position(clamped_x)
            x_stepper.set_target(clamped_x, state['x_min'], state['x_max'])
        state['x_pulse'] = int(x_stepper.current_pos)

    # 支援 mm 與 µs 設定 Z/Y 軸（支援 0 起算與閉迴路大行程脈衝）
    z_min_in = data.get('z_min', data.get('y_min'))
    z_max_in = data.get('z_max', data.get('y_max'))
    z_min_mm_in = data.get('z_min_mm', data.get('y_min_mm'))
    z_max_mm_in = data.get('z_max_mm', data.get('y_max_mm'))
    if z_min_in is not None or z_max_in is not None:
        if z_min_in is not None:
            state['z_min'] = state['y_min'] = max(0, min(SYSTEM_PULSE_MAX, int(round(float(z_min_in)))))
        if z_max_in is not None:
            state['z_max'] = state['y_max'] = max(state['z_min'] + 1, min(SYSTEM_PULSE_MAX, int(round(float(z_max_in)))))
        if state['z_max'] <= state['z_min']:
            state['z_max'] = state['y_max'] = min(SYSTEM_PULSE_MAX, state['z_min'] + 1)
        scale = state.get('z_scale', state['y_scale'])
        r_min = 0.0 if state['z_min'] in (0, PULSE_MIN) else round(state['z_min'] / max(0.001, scale), 1)
        r_max = round(r_min + (state['z_max'] - state['z_min']) / max(0.001, scale), 1)
        state['z_min_mm'] = state['y_min_mm'] = r_min
        state['z_max_mm'] = state['y_max_mm'] = r_max
    elif z_min_mm_in is not None or z_max_mm_in is not None:
        scale = state.get('z_scale', state['y_scale'])
        min_mm = max(0.0, float(z_min_mm_in if z_min_mm_in is not None else state.get('z_min_mm', 0.0)))
        max_mm = float(z_max_mm_in if z_max_mm_in is not None else state.get('z_max_mm', 200.0))
        if max_mm <= min_mm:
            max_mm = min_mm + 1.0
        state['z_min_mm'] = state['y_min_mm'] = round(min_mm, 1)
        state['z_max_mm'] = state['y_max_mm'] = round(max_mm, 1)
        base_us = PULSE_MIN if (state['z_min'] == PULSE_MIN and min_mm == 0.0) else 0.0
        state['z_min'] = state['y_min'] = max(0, min(SYSTEM_PULSE_MAX, int(round(base_us + min_mm * scale))))
        state['z_max'] = state['y_max'] = max(state['z_min'] + 1, min(SYSTEM_PULSE_MAX, int(round(base_us + max_mm * scale))))

    st_z = z_stepper or y_stepper
    if st_z:
        clamped_z = max(float(state['z_min']), min(float(state['z_max']), float(st_z.current_pos)))
        if abs(clamped_z - st_z.current_pos) > 0.5:
            st_z.set_position(clamped_z)
            st_z.set_target(clamped_z, state['z_min'], state['z_max'])
        state['z_pulse'] = state['y1_pulse'] = state['y2_pulse'] = int(st_z.current_pos)

    if 'y_sync' in data or 'z_sync' in data:
        state['y_sync'] = bool(data.get('z_sync', data.get('y_sync')))
    
    if 'x_max_speed' in data:
        state['x_max_speed'] = float(data['x_max_speed'])
        if x_stepper: x_stepper.max_speed_hz = float(data['x_max_speed'])
    if 'x_accel_steps' in data:
        state['x_accel_steps'] = float(data['x_accel_steps'])
        if x_stepper: x_stepper.accel_steps = float(data['x_accel_steps'])
        
    z_spd = data.get('z_max_speed', data.get('y_max_speed'))
    if z_spd is not None:
        spd_val = float(z_spd)
        state['z_max_speed'] = state['y_max_speed'] = spd_val
        if st_z: st_z.max_speed_hz = spd_val
        
    z_acc = data.get('z_accel_steps', data.get('y_accel_steps'))
    if z_acc is not None:
        acc_val = float(z_acc)
        state['z_accel_steps'] = state['y_accel_steps'] = acc_val
        if st_z: st_z.accel_steps = acc_val
        
    if 'homing_speed_hz' in data:
        state['homing_speed_hz'] = max(100.0, min(25000.0, float(data['homing_speed_hz'])))

    # 若單獨調整了 x_scale / z_scale，重新換算對應 mm 範圍
    if 'x_scale' in data and 'x_min_mm' not in data and 'x_max_mm' not in data:
        state['x_max_mm'] = round(state.get('x_min_mm', 0.0) + (state['x_max'] - state['x_min']) / max(0.001, state['x_scale']), 1)
    if ('y_scale' in data or 'z_scale' in data) and z_min_mm_in is None and z_max_mm_in is None:
        scale = state.get('z_scale', state['y_scale'])
        state['z_max_mm'] = state['y_max_mm'] = round(state.get('z_min_mm', 0.0) + (state['z_max'] - state['z_min']) / max(0.001, scale), 1)

    save_limits_config()

    add_history(f"更新行程範圍與參數: X({state['x_min']}~{state['x_max']}µs, {state['x_min_mm']}~{state['x_max_mm']}mm, 比例:{state['x_scale']}), Z/Y({state['z_min']}~{state['z_max']}µs, {state['z_min_mm']}~{state['z_max_mm']}mm, 比例:{state.get('z_scale', state['y_scale'])})", "success")
    return jsonify({'ok': True, 'limits': {
        'x_min': state['x_min'], 'x_max': state['x_max'],
        'z_min': state['z_min'], 'z_max': state['z_max'],
        'y_min': state['y_min'], 'y_max': state['y_max'],
        'x_min_mm': state['x_min_mm'], 'x_max_mm': state['x_max_mm'],
        'z_min_mm': state['z_min_mm'], 'z_max_mm': state['z_max_mm'],
        'y_min_mm': state['y_min_mm'], 'y_max_mm': state['y_max_mm'],
        'y_sync': state['y_sync'],
        'x_max_speed': state['x_max_speed'], 'x_accel_steps': state['x_accel_steps'],
        'z_max_speed': state['z_max_speed'], 'z_accel_steps': state['z_accel_steps'],
        'y_max_speed': state['y_max_speed'], 'y_accel_steps': state['y_accel_steps'],
        'homing_speed_hz': state['homing_speed_hz'],
        'x_scale': state['x_scale'], 'z_scale': state.get('z_scale', state['y_scale']),
        'y_scale': state['y_scale']
    }})

@app.route('/api/macros', methods=['GET', 'POST', 'DELETE'])
def api_macros():
    if request.method == 'GET':
        return jsonify({'ok': True, 'macros': load_macros()})
    elif request.method == 'POST':
        data = request.get_json(silent=True) or {}
        name = data.get('name')
        commands = data.get('commands')
        if not name or not isinstance(commands, list):
            return jsonify({'ok': False, 'msg': '無效的參數'})
        macros = load_macros()
        macros[name] = commands
        save_macros(macros)
        add_history(f"儲存巨集腳本: {name}", "success")
        return jsonify({'ok': True})
    elif request.method == 'DELETE':
        data = request.get_json(silent=True) or {}
        name = data.get('name')
        macros = load_macros()
        if name in macros:
            del macros[name]
            save_macros(macros)
            add_history(f"刪除巨集腳本: {name}", "warn")
            return jsonify({'ok': True})
        return jsonify({'ok': False, 'msg': '找不到該巨集'})

@app.route('/api/macro/run', methods=['POST'])
def api_macro_run():
    global macro_thread, macro_running, macro_paused
    data = request.get_json(silent=True) or {}
    action = data.get('action') # 'start', 'stop', 'pause', 'resume'
    name = data.get('name')
    direct_commands = data.get('commands')
    
    if action == 'start':
        if macro_running:
            return jsonify({'ok': False, 'msg': '巨集或積木程式已在運行中'})
        if isinstance(direct_commands, list) and len(direct_commands) > 0:
            commands = [str(c) for c in direct_commands]
        else:
            macros = load_macros()
            commands = macros.get(name)
        if not commands:
            return jsonify({'ok': False, 'msg': '找不到該巨集或積木指令為空'})
        macro_thread = threading.Thread(target=run_macro_loop, args=(commands,), daemon=True)
        macro_thread.start()
        return jsonify({'ok': True})
        
    elif action == 'stop':
        macro_running = False
        macro_paused = False
        return jsonify({'ok': True})
        
    elif action == 'pause':
        macro_paused = True
        socketio.emit('macro_status', {'running': True, 'paused': True})
        return jsonify({'ok': True})
        
    elif action == 'resume':
        macro_paused = False
        socketio.emit('macro_status', {'running': True, 'paused': False})
        return jsonify({'ok': True})
        
    return jsonify({'ok': False, 'msg': '無效的指令'})

@app.route('/api/set_motor_enable', methods=['POST'])
def api_set_motor_enable():
    data = request.get_json(silent=True) or {}
    locked = bool(data.get('locked', True))
    state['motor_locked'] = locked
    
    if x_stepper:
        x_stepper.set_enable(locked)
    if y_stepper:
        y_stepper.set_enable(locked)
        
    status_str = "馬達鎖定 (致能通電中)" if locked else "馬達釋放 (斷電手推模式已啟用)"
    add_history(status_str, "warn")
    return jsonify({'ok': True, 'motor_locked': locked})

@app.route('/api/history', methods=['GET', 'DELETE'])
def api_history():
    if request.method == 'GET':
        return jsonify({'ok': True, 'history': load_history()})
    elif request.method == 'DELETE':
        save_history([])
        return jsonify({'ok': True})

@app.route('/api/errors', methods=['GET', 'POST', 'DELETE'])
def api_errors():
    """錯誤日誌中心 API：查詢全部錯誤/警告、前端異常回報、清空錯誤紀錄"""
    if request.method == 'GET':
        with error_lock:
            errs = list(error_buffer)
        total = len(errs)
        err_cnt = sum(1 for e in errs if e.get('level') in ('ERROR', 'CRITICAL'))
        warn_cnt = sum(1 for e in errs if e.get('level') == 'WARN')
        unres_cnt = sum(1 for e in errs if not e.get('resolved', False))
        return jsonify({
            'ok': True,
            'active_error': bool(state.get('error', False)),
            'active_error_msg': state.get('error_msg', ''),
            'stats': {
                'total': total,
                'error_count': err_cnt,
                'warn_count': warn_cnt,
                'unresolved_count': unres_cnt
            },
            'errors': errs
        })
    elif request.method == 'POST':
        data = request.get_json(silent=True) or {}
        level = data.get('level', 'ERROR')
        source = data.get('source')
        code = data.get('code')
        msg = data.get('msg', '未知系統錯誤')
        detail = data.get('detail')
        if data.get('set_alarm'):
            state['error'] = True
            state['error_msg'] = str(msg)
        entry = add_error_log(level, source, code, msg, detail)
        return jsonify({'ok': True, 'entry': entry})
    elif request.method == 'DELETE':
        with error_lock:
            error_buffer.clear()
        state['error'] = False
        state['error_msg'] = ''
        save_error_logs()
        return jsonify({'ok': True})

@app.route('/api/errors/resolve', methods=['POST'])
def api_errors_resolve():
    """解除當前系統錯誤警報並標記錯誤日誌為已處理"""
    data = request.get_json(silent=True) or {}
    target_id = data.get('id')
    with error_lock:
        for item in error_buffer:
            if target_id is None or item.get('id') == target_id:
                item['resolved'] = True
    if target_id is None:
        state['error'] = False
        state['error_msg'] = ''
    save_error_logs()
    return jsonify({'ok': True})

@app.route('/api/diagnose')
def api_diagnose():
    """馬達與感測器診斷資訊"""
    diag = {
        'gpio_mode': state['gpio_mode'],
        'is_raspberry_pi': IS_RASPBERRY_PI,
        'pwm_freq': PWM_FREQ,
        'motors': {
            'X': {'pin': PIN['X_PUL'], 'pulse': state['x_pulse'], 'duty': round(pulse_to_duty(state['x_pulse']), 3)},
            'Z': {'pin': PIN['Z_PUL'], 'pulse': state['z_pulse'], 'duty': round(pulse_to_duty(state['z_pulse']), 3)},
            'Y1': {'pin': PIN['Y_PUL'], 'pulse': state['y1_pulse'], 'duty': round(pulse_to_duty(state['y1_pulse']), 3)},
            'Y2': {'pin': PIN['Y_PUL'], 'pulse': state['y2_pulse'], 'duty': round(pulse_to_duty(state['y2_pulse']), 3)},
        },
        'digital_outputs': {
            'MAGNET': {'pin': PIN['MAGNET'], 'state': state['magnet'], 'active_count': state.get('magnet_active_count', 0), 'model': 'LSD-P25/20 (4組)', 'relay': 'TONGLING JQC-3FF-S-Z 5VDC (光耦隔離繼電器 × 4組)', 'desc': '4 組圓形電磁鐵 LSD-P25/20 + JQC-3FF-S-Z 繼電器'},
            'MAGNET_1': {'pin': PIN['MAGNET_1'], 'state': state.get('magnet_1', False), 'name': '第1組電磁鐵 M1 (對應霍爾G1)'},
            'MAGNET_2': {'pin': PIN['MAGNET_2'], 'state': state.get('magnet_2', False), 'name': '第2組電磁鐵 M2 (對應霍爾G2)'},
            'MAGNET_3': {'pin': PIN['MAGNET_3'], 'state': state.get('magnet_3', False), 'name': '第3組電磁鐵 M3 (對應霍爾G3)'},
            'MAGNET_4': {'pin': PIN['MAGNET_4'], 'state': state.get('magnet_4', False), 'name': '第4組電磁鐵 M4 (對應霍爾G4)'},
            'VACUUM': {'pin': PIN['VACUUM'], 'state': state['vacuum']},
            'Z_DOWN': {'pin': PIN.get('Z_DOWN', 25), 'state': state['z_down']},
        },
        'inductive_sensors': {
            'sensor_model': 'QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (檢測距離 5mm)',
            'X_A': {'pin': PIN['LIM_X_A'], 'name': 'X軸a點(左歸零)', 'triggered': state.get('lim_x_a')},
            'X_B': {'pin': PIN['LIM_X_B'], 'name': 'X軸b點(中間工位)', 'triggered': state.get('lim_x_b')},
            'X_C': {'pin': PIN['LIM_X_C'], 'name': 'X軸c點(向右極限)', 'triggered': state.get('lim_x_c')},
            'Z_D': {'pin': PIN['LIM_Z_D'], 'name': 'Z軸d點(向上歸零)', 'triggered': state.get('lim_z_d')},
            'Z_E': {'pin': PIN['LIM_Z_E'], 'name': 'Z軸e點(向下極限)', 'triggered': state.get('lim_z_e')},
        },
        'hall_sensors': {
            'optocoupler': '8路 817 光耦電壓隔離板 (PC817 電壓控制轉接模組)',
            'layout': '8 顆霍爾開關，兩個一組 (共 4 組)',
            'active_count': state.get('hall_active_count', 0),
            'group_count': state.get('hall_group_count', 0),
            'groups': {
                'G1': {'name': '第1組 (H1+H2)', 'pins': [PIN['HALL_1'], PIN['HALL_2']], 'h1': state.get('hall_1'), 'h2': state.get('hall_2'), 'triggered': state.get('hall_group_1')},
                'G2': {'name': '第2組 (H3+H4)', 'pins': [PIN['HALL_3'], PIN['HALL_4']], 'h3': state.get('hall_3'), 'h4': state.get('hall_4'), 'triggered': state.get('hall_group_2')},
                'G3': {'name': '第3組 (H5+H6)', 'pins': [PIN['HALL_5'], PIN['HALL_6']], 'h5': state.get('hall_5'), 'h6': state.get('hall_6'), 'triggered': state.get('hall_group_3')},
                'G4': {'name': '第4組 (H7+H8)', 'pins': [PIN['HALL_7'], PIN['HALL_8']], 'h7': state.get('hall_7'), 'h8': state.get('hall_8'), 'triggered': state.get('hall_group_4')},
            },
            'ARM_METAL': {'pin': PIN['HALL_ARM_METAL'], 'name': '手臂金屬檢測總狀態 (8顆/4組)', 'triggered': state.get('hall_arm_metal')},
        },
        'limit_switches': {
            'LEFT':  {'pin': PIN['LIM_LEFT'],  'triggered': state['lim_left']},
            'MID':   {'pin': PIN['LIM_MID'],   'triggered': state['lim_mid']},
            'RIGHT': {'pin': PIN['LIM_RIGHT'], 'triggered': state['lim_right']},
            'UP':    {'pin': PIN['LIM_UP'],    'triggered': state['lim_up']},
            'DOWN':  {'pin': PIN['LIM_DOWN'],  'triggered': state['lim_down']},
        },
        'pwm_signal_info': {
            'frequency_hz': PWM_FREQ,
            'period_ms': round(1000 / PWM_FREQ, 2),
            'x_pulse_us':  state['x_pulse'],
            'z_pulse_us':  state['z_pulse'],
            'y1_pulse_us': state['y1_pulse'],
            'y2_pulse_us': state['y2_pulse'],
            'x_duty_pct':  round(pulse_to_duty(state['x_pulse']), 3),
            'z_duty_pct':  round(pulse_to_duty(state['z_pulse']), 3),
            'y1_duty_pct': round(pulse_to_duty(state['y1_pulse']), 3),
            'y2_duty_pct': round(pulse_to_duty(state['y2_pulse']), 3),
        }
    }
    return jsonify(diag)

# ── SocketIO 事件 ─────────────────────────────────────────────────
@socketio.on('connect')
def on_connect():
    print(f"[WS] Client connected: {request.sid}")
    emit('state_update', {**state, 'pwm_history': pwm_history, 'pin_config': PIN})

@socketio.on('disconnect')
def on_disconnect():
    print(f"[WS] Client disconnected: {request.sid}")

# ── 資安驗證 Session 管理 ─────────────────────────────────────────
terminal_sessions = {}  # { token: { 'password': str, 'expires': float } }
site_sessions    = {}   # { token: { 'password': str, 'expires': float } } — 全站認證
sim_custom_password = 'Nice'  # 模擬模式可設定的登入密碼（預設 Nice）

def verify_system_password(password):
    """驗證密碼是否符合 Linux 系統權限 (sudo -k -S -v) 或模擬模式自訂密碼"""
    global sim_custom_password
    if not password or not password.strip():
        return False
    if IS_RASPBERRY_PI or os.name == 'posix':
        try:
            proc = subprocess.run(
                ["sudo", "-k", "-S", "-v"],
                input=password.strip() + "\n",
                capture_output=True,
                text=True,
                timeout=5
            )
            return proc.returncode == 0
        except Exception:
            return False
    else:
        return password.strip() == sim_custom_password

def cleanup_expired_sessions():
    now = time.time()
    expired_t = [t for t, s in list(terminal_sessions.items()) if s['expires'] < now]
    expired_s = [t for t, s in list(site_sessions.items()) if s['expires'] < now]
    for t in expired_t:
        terminal_sessions.pop(t, None)
    for t in expired_s:
        site_sessions.pop(t, None)

def is_valid_token(token):
    cleanup_expired_sessions()
    return token in terminal_sessions

def is_valid_site_token(token):
    cleanup_expired_sessions()
    return token in site_sessions

def create_site_session(password):
    """建立全站 + 終端機共用 session"""
    token = secrets.token_hex(32)
    timeout_sec = state.get('lock_timeout_min', 30) * 60
    entry = {'password': password, 'expires': time.time() + timeout_sec}
    site_sessions[token]     = dict(entry)
    terminal_sessions[token] = dict(entry)
    return token

def refresh_session(token):
    """延長 session 有效時間"""
    timeout_sec = state.get('lock_timeout_min', 30) * 60
    expires = time.time() + timeout_sec
    if token in site_sessions:
        site_sessions[token]['expires'] = expires
    if token in terminal_sessions:
        terminal_sessions[token]['expires'] = expires

@app.route('/api/terminal/auth', methods=['POST'])
def api_terminal_auth():
    """Web 終端機密碼驗證解鎖（同時建立全站 session）"""
    data = request.get_json(silent=True) or {}
    password = data.get('password', '')

    if verify_system_password(password):
        token = create_site_session(password)
        add_history("Web 終端機通過身份驗證解鎖成功", "success")
        return jsonify({'ok': True, 'token': token, 'msg': '驗證成功'})
    else:
        add_history("Web 終端機身份驗證失敗 (密碼錯誤)", "warn")
        return jsonify({'ok': False, 'msg': '密碼錯誤，拒絕存取'})

@app.route('/api/terminal/logout', methods=['POST'])
def api_terminal_logout():
    """鎖定終端機"""
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    if token in terminal_sessions:
        del terminal_sessions[token]
    return jsonify({'ok': True})

@app.route('/api/terminal/check_token', methods=['POST'])
def api_terminal_check_token():
    """檢查 Token 是否仍有效"""
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    if is_valid_token(token):
        refresh_session(token)
        return jsonify({'ok': True, 'valid': True})
    return jsonify({'ok': False, 'valid': False})

# ── 全站認證 API ──────────────────────────────────────────────────
@app.route('/api/auth/login', methods=['POST'])
def api_auth_login():
    """全站登入驗證"""
    data = request.get_json(silent=True) or {}
    password = data.get('password', '')
    if verify_system_password(password):
        token = create_site_session(password)
        add_history("全站登入成功", "success")
        return jsonify({'ok': True, 'token': token, 'msg': '登入成功'})
    else:
        add_history("全站登入失敗（密碼錯誤）", "warn")
        return jsonify({'ok': False, 'msg': '密碼錯誤，拒絕登入'})

@app.route('/api/auth/status', methods=['POST'])
def api_auth_status():
    """檢查全站 session 是否有效，並自動續期"""
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    valid = is_valid_site_token(token)
    if valid:
        refresh_session(token)
    return jsonify({'ok': True, 'valid': valid})

@app.route('/api/auth/logout', methods=['POST'])
def api_auth_logout():
    """全站登出"""
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    site_sessions.pop(token, None)
    terminal_sessions.pop(token, None)
    add_history("使用者登出系統", "info")
    return jsonify({'ok': True})

@app.route('/api/settings/security', methods=['POST'])
def api_settings_security():
    """更新安全設定（鎖定時間、模擬模式密碼）"""
    global sim_custom_password
    data = request.get_json(silent=True) or {}
    if 'lock_timeout_min' in data:
        val = int(data['lock_timeout_min'])
        state['lock_timeout_min'] = max(1, min(480, val))
    if 'new_password' in data and not (IS_RASPBERRY_PI or os.name == 'posix'):
        new_pwd = str(data['new_password']).strip()
        if len(new_pwd) >= 4:
            sim_custom_password = new_pwd
            add_history("模擬模式登入密碼已更新", "success")
    add_history(f"安全設定已更新：自動鎖定 {state['lock_timeout_min']} 分鐘", "success")
    return jsonify({'ok': True, 'lock_timeout_min': state['lock_timeout_min'],
                    'is_raspberry_pi': IS_RASPBERRY_PI})

# 終端機活躍進程管理
active_terminal_proc = None
active_terminal_lock = threading.Lock()

def get_git_repo_dir():
    """智慧尋找專案所屬的 Git 倉庫根目錄（支援子目錄部署與標準倉庫結構）"""
    current = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.isdir(os.path.join(current, '.git')):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return parent if os.path.exists(parent) else os.path.dirname(os.path.abspath(__file__))

@app.route('/api/terminal/kill', methods=['POST'])
def api_terminal_kill():
    """中止目前執行中的終端機指令進程"""
    global active_terminal_proc
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    if not is_valid_token(token):
        return jsonify({'ok': False, 'msg': '未授權'})

    with active_terminal_lock:
        if active_terminal_proc and active_terminal_proc.poll() is None:
            try:
                active_terminal_proc.kill()
                add_history("使用者終止終端機執行中指令", "warn")
                return jsonify({'ok': True, 'msg': '已成功終止指令進程'})
            except Exception as e:
                return jsonify({'ok': False, 'msg': f'終止失敗: {e}'})
    return jsonify({'ok': True, 'msg': '目前無執行中的指令進程'})

# ── 系統更新 API ──────────────────────────────────────────────────
GITHUB_REPO = "Nono0325/Nice1"  # GitHub 倉庫名稱
GITHUB_BRANCH = "main"         # 預設分支

@app.route('/api/update/check', methods=['GET', 'POST'])
def api_update_check():
    """自動檢查 GitHub 最新版本資訊（優先使用 git fetch + origin/main，並以 GitHub API 作為備援）"""
    repo_dir = get_git_repo_dir()
    local_sha = 'unknown'
    try:
        local_sha = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=5, cwd=repo_dir
        ).stdout.strip() or 'unknown'
    except Exception:
        pass

    remote_sha = ''
    remote_msg = ''
    remote_author = ''
    remote_date = ''
    behind_count = 0
    changelog = []

    # 1. 優先透過 git fetch origin main 檢查（無 API 速率限制，且可計算落後 commit 數與更新清單）
    try:
        subprocess.run(
            ['git', 'fetch', 'origin', GITHUB_BRANCH, '--quiet'],
            capture_output=True, text=True, timeout=10, cwd=repo_dir
        )
        info_out = subprocess.run(
            ['git', 'log', '-1', '--format=%h|%s|%an|%cs', f'origin/{GITHUB_BRANCH}'],
            capture_output=True, text=True, timeout=5, cwd=repo_dir
        ).stdout.strip()
        if info_out and '|' in info_out:
            parts = info_out.split('|', 3)
            remote_sha = parts[0].strip()
            remote_msg = parts[1].strip() if len(parts) > 1 else ''
            remote_author = parts[2].strip() if len(parts) > 2 else ''
            remote_date = parts[3].strip() if len(parts) > 3 else ''

        cnt_out = subprocess.run(
            ['git', 'rev-list', '--count', f'HEAD..origin/{GITHUB_BRANCH}'],
            capture_output=True, text=True, timeout=5, cwd=repo_dir
        ).stdout.strip()
        if cnt_out.isdigit():
            behind_count = int(cnt_out)

        log_out = subprocess.run(
            ['git', 'log', '--oneline', '-n', '5', f'HEAD..origin/{GITHUB_BRANCH}'],
            capture_output=True, text=True, timeout=5, cwd=repo_dir
        ).stdout.strip()
        if log_out:
            changelog = [line.strip() for line in log_out.splitlines() if line.strip()]
    except Exception:
        pass

    # 2. 若 git 尚未取得遠端資訊，使用 GitHub REST API 備援
    if not remote_sha:
        try:
            import urllib.request, json as _json
            api_url = f"https://api.github.com/repos/{GITHUB_REPO}/commits/{GITHUB_BRANCH}"
            req = urllib.request.Request(api_url, headers={'User-Agent': 'ServoController/2.0'})
            with urllib.request.urlopen(req, timeout=8) as resp:
                data = _json.loads(resp.read().decode())
            remote_sha    = data['sha'][:7]
            remote_msg    = data['commit']['message'].split('\n')[0][:80]
            remote_author = data['commit']['author']['name']
            remote_date   = data['commit']['author']['date'][:10]
        except Exception as e:
            return jsonify({'ok': False, 'msg': f'無法連線至 GitHub: {str(e)}'})

    up_to_date = (
        local_sha != 'unknown'
        and local_sha != ''
        and (remote_sha.startswith(local_sha[:7]) or local_sha.startswith(remote_sha[:7]))
        and behind_count == 0
    )

    return jsonify({
        'ok': True,
        'local_sha': local_sha,
        'remote_sha': remote_sha,
        'remote_msg': remote_msg,
        'remote_author': remote_author,
        'remote_date': remote_date,
        'behind_count': behind_count,
        'changelog': changelog,
        'up_to_date': up_to_date,
        'repo': GITHUB_REPO,
        'branch': GITHUB_BRANCH
    })

@app.route('/api/update/pull', methods=['POST'])
def api_update_pull():
    """執行 git pull 自動更新程式（支援一鍵自動更新與自動重啟服務）"""
    data    = request.get_json(silent=True) or {}
    token   = data.get('token', '')
    restart = data.get('restart', True)
    if token:
        refresh_session(token)

    repo_dir = get_git_repo_dir()
    add_history("系統自動更新：開始執行 git fetch + reset --hard", "warn")

    try:
        # 1. git fetch
        fetch = subprocess.run(
            ['git', 'fetch', 'origin', GITHUB_BRANCH],
            capture_output=True, text=True, timeout=30, cwd=repo_dir
        )
        # 2. git reset --hard（強制覆蓋本地修改）
        pull = subprocess.run(
            ['git', 'reset', '--hard', f'origin/{GITHUB_BRANCH}'],
            capture_output=True, text=True, timeout=30, cwd=repo_dir
        )
        output = (fetch.stdout + fetch.stderr + '\n' + pull.stdout + pull.stderr).strip()
        
        if pull.returncode != 0:
            add_history(f"系統更新失敗: {output[:100]}", "error")
            return jsonify({'ok': False, 'msg': output or '更新失敗'})

        # 3. 取得最新 SHA
        new_sha = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, cwd=repo_dir
        ).stdout.strip()

        add_history(f"系統更新成功：{new_sha}", "success")

        # 4. 若在樹莓派且 restart=True，重啟服務
        if restart and (IS_RASPBERRY_PI or os.name == 'posix'):
            pwd = terminal_sessions.get(token, {}).get('password', '')
            def _do_restart():
                import time as _t
                _t.sleep(1.5)
                subprocess.run(
                    ['sudo', '-S', 'systemctl', 'restart', 'servo-control'],
                    input=pwd + '\n', capture_output=True, text=True, timeout=15
                )
            threading.Thread(target=_do_restart, daemon=True).start()
            return jsonify({'ok': True, 'sha': new_sha, 'output': output,
                            'msg': f'更新完成！正在重啟服務... ({new_sha})'})
        
        return jsonify({'ok': True, 'sha': new_sha, 'output': output,
                        'msg': f'更新完成！({new_sha}) 請手動重啟服務生效。'})

    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'msg': '更新超時（30秒）'})
    except Exception as e:
        add_history(f"系統更新例外: {str(e)}", "error")
        return jsonify({'ok': False, 'msg': f'更新例外: {str(e)}'})


@app.route('/api/system_power', methods=['POST'])
def api_system_power():
    """樹梅派系統關機與重啟 (受資安密碼驗證保護)"""
    data = request.get_json(silent=True) or {}
    action = data.get('action')  # 'shutdown' or 'reboot'
    token = data.get('token', '')
    password = data.get('password', '')

    if not (is_valid_token(token) or verify_system_password(password)):
        return jsonify({'ok': False, 'auth_required': True, 'msg': '權限不足：密碼驗證失敗'})

    user_pwd = terminal_sessions.get(token, {}).get('password', password)

    if action == 'shutdown':
        add_history("觸發樹梅派系統關機 (Shutdown)", "warn")
        if IS_RASPBERRY_PI or os.name == 'posix':
            def do_shutdown():
                time.sleep(1)
                proc = subprocess.Popen(["sudo", "-S", "shutdown", "-h", "now"], stdin=subprocess.PIPE, text=True)
                proc.communicate(input=user_pwd + "\n")
            threading.Thread(target=do_shutdown, daemon=True).start()
            return jsonify({'ok': True, 'msg': '樹梅派正在關機中...'})
        else:
            return jsonify({'ok': True, 'msg': '模擬模式：已觸發虛擬關機'})

    elif action == 'reboot':
        add_history("觸發樹梅派系統重啟 (Reboot)", "warn")
        if IS_RASPBERRY_PI or os.name == 'posix':
            def do_reboot():
                time.sleep(1)
                proc = subprocess.Popen(["sudo", "-S", "reboot"], stdin=subprocess.PIPE, text=True)
                proc.communicate(input=user_pwd + "\n")
            threading.Thread(target=do_reboot, daemon=True).start()
            return jsonify({'ok': True, 'msg': '樹梅派正在重新啟動中...'})
        else:
            return jsonify({'ok': True, 'msg': '模擬模式：已觸發虛擬重啟'})

    return jsonify({'ok': False, 'msg': '無效的電源指令'})

# 追蹤每個 session 的當前工作目錄
_terminal_cwd = {}

@app.route('/api/terminal/exec', methods=['POST'])
def api_terminal_exec():
    """Web 系統終端機指令執行（受驗證保護，支援 sudo / cd / 可配置超時）"""
    global active_terminal_proc
    data = request.get_json(silent=True) or {}
    token   = data.get('token', '')
    cmd     = data.get('cmd', '').strip()
    timeout_sec = min(int(data.get('timeout', 30)), 120)

    if not is_valid_token(token):
        return jsonify({'ok': False, 'auth_required': True,
                        'output': '🔒 權限不足：請先輸入密碼驗證解鎖終端機'})
    if not cmd:
        return jsonify({'ok': False, 'output': '空指令'})

    refresh_session(token)
    user_pwd = terminal_sessions[token]['password']
    add_history(f"終端機執行: {cmd}", "info")

    default_cwd = os.path.dirname(os.path.abspath(__file__))
    cwd = _terminal_cwd.get(token, default_cwd)

    # ── cd 指令：維持工作目錄狀態 ─────────────────────────────────
    bare_cmd = cmd.strip()
    if bare_cmd == 'cd' or bare_cmd.startswith('cd '):
        target = bare_cmd[3:].strip() if bare_cmd.startswith('cd ') else os.path.expanduser('~')
        target = os.path.expanduser(target)
        if not os.path.isabs(target):
            target = os.path.join(cwd, target)
        target = os.path.normpath(target)
        if os.path.isdir(target):
            _terminal_cwd[token] = target
            return jsonify({'ok': True, 'returncode': 0, 'output': f'[已切換目錄 → {target}]', 'cwd': target})
        else:
            return jsonify({'ok': False, 'returncode': 1, 'output': f'cd: {target}: 找不到目錄', 'cwd': cwd})

    try:
        is_sudo = cmd.startswith('sudo ') and (IS_RASPBERRY_PI or os.name == 'posix')

        if is_sudo:
            inner = cmd[5:].strip()
            # 偵測互動式 shell（需要 TTY，Web 終端機無法支援）
            interactive = ['bash', 'sh', 'zsh', 'fish', 'python', 'python3', 'irb', 'node']
            first_part  = inner.split()[0] if inner.split() else ''
            if first_part in interactive or inner.startswith('-i') or inner.startswith('-s'):
                return jsonify({'ok': False, 'returncode': 1,
                                'output': '⚠️ 互動式 Shell 指令（sudo bash / sudo -i 等）不支援 Web 終端機模式。\n請改用非互動式指令，例如：sudo systemctl restart servo-control'})

            full_cmd = f'sudo -k -S {inner}'
            proc = subprocess.Popen(
                full_cmd, shell=True,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, cwd=cwd
            )
            with active_terminal_lock:
                active_terminal_proc = proc
            try:
                stdout, stderr = proc.communicate(input=user_pwd + '\n', timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.communicate()
                return jsonify({'ok': False, 'output': f'❌ 指令執行超時（超過 {timeout_sec} 秒）\n提示：長時間指令請改用 SSH 執行'})
            finally:
                with active_terminal_lock:
                    active_terminal_proc = None

            # 過濾 sudo 的密碼提示行
            stderr_clean = '\n'.join(
                l for l in (stderr or '').splitlines()
                if not l.lower().startswith('[sudo]') and 'password for' not in l.lower()
            )
            output = (stdout or '')
            if stderr_clean.strip():
                output += f'\n[STDERR]\n{stderr_clean}'
            if not output.strip():
                output = '(指令執行完成，無輸出內容)'
            return jsonify({'ok': proc.returncode == 0, 'returncode': proc.returncode,
                            'output': output, 'cwd': cwd})

        else:
            env = os.environ.copy()
            env['TERM'] = 'xterm-256color'
            proc = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, cwd=cwd, env=env
            )
            with active_terminal_lock:
                active_terminal_proc = proc
            try:
                stdout, stderr = proc.communicate(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.communicate()
                return jsonify({'ok': False, 'output': f'❌ 指令執行超時（超過 {timeout_sec} 秒）'})
            finally:
                with active_terminal_lock:
                    active_terminal_proc = None

            stdout = stdout or ''
            stderr = stderr or ''
            output = stdout
            if stderr.strip():
                output += f'\n[STDERR]\n{stderr}'
            if not output.strip():
                output = '(指令執行完成，無輸出內容)'
            return jsonify({'ok': proc.returncode == 0, 'returncode': proc.returncode,
                            'output': output, 'cwd': cwd})

    except Exception as e:
        return jsonify({'ok': False, 'output': f'❌ 執行例外: {str(e)}'}) 

@socketio.on('cmd_move')
def on_cmd_move(data):
    """WebSocket 移動命令（低延遲，支援 X/Z 軸與 mm 累加目標 100+100）"""
    axis = str(data.get('axis', 'x')).lower()
    delta_mm = data.get('delta_mm')
    delta_us = data.get('delta')

    scale = state['x_scale'] if axis == 'x' else state.get('z_scale', state['y_scale'])
    if delta_mm is not None:
        delta = float(delta_mm) * float(scale)
    else:
        delta = float(delta_us or 0)

    if state['system_run']:
        if axis == 'x':
            if (state.get('lim_x_a') or state.get('lim_left')) and delta < 0:
                return
            if (state.get('lim_x_c') or state.get('lim_right')) and delta > 0:
                return
            if x_stepper:
                x_stepper.add_target_delta(delta, state['x_min'], state['x_max'])
        elif axis in ('z', 'y', 'y1', 'y2'):
            if (state.get('lim_z_e') or state.get('lim_down')) and delta < 0:
                return
            if (state.get('lim_z_d') or state.get('lim_up')) and delta > 0:
                return
            st_z = z_stepper or y_stepper
            z_min = state.get('z_min', state['y_min'])
            z_max = state.get('z_max', state['y_max'])
            if st_z:
                st_z.add_target_delta(delta, z_min, z_max)

# ── 樹梅派硬體看門狗晶片守護 (BCM2835 Hardware Watchdog) ─────────
watchdog_fd = None

def init_hardware_watchdog():
    """啟動樹梅派硬體看門狗 (/dev/watchdog) 心跳守護線程（預設交由 systemd 管理，設 ENABLE_HW_WATCHDOG=1 時啟用）"""
    global watchdog_fd
    if not IS_RASPBERRY_PI:
        return
    if os.environ.get('ENABLE_HW_WATCHDOG', '0') != '1':
        print("[WATCHDOG] 採 systemd 服務守護模式 (未獨佔開啟 /dev/watchdog，防止重啟時系統重開機)")
        return
    try:
        if os.path.exists('/dev/watchdog'):
            watchdog_fd = open('/dev/watchdog', 'w')
            state['watchdog_active'] = True
            print("[WATCHDOG] 硬體看門狗 /dev/watchdog 啟動成功，心跳保護已就緒")

            def _watchdog_feeder():
                while True:
                    try:
                        if watchdog_fd and not watchdog_fd.closed:
                            watchdog_fd.write('\0')
                            watchdog_fd.flush()
                    except Exception:
                        pass
                    time.sleep(3.0)

            t = threading.Thread(target=_watchdog_feeder, daemon=True)
            t.start()
        else:
            print("[WATCHDOG] 提示: 未檢測到 /dev/watchdog 節點 (需載入 bcm2835_wdt 模組)")
    except Exception as e:
        print(f"[WATCHDOG] 無法開啟硬體看門狗: {e}")

# ── 主程式 ────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("  樹梅派步進馬達控制系統 (已升級連續運行機制)")
    print("  Raspberry Pi Stepper Motor Controller")
    print(f"  模式: {state['gpio_mode']}")
    print("=" * 60)

    # 初始化 GPIO
    init_gpio()

    # 初始化硬體看門狗
    init_hardware_watchdog()

    # 啟動廣播執行緒
    broadcast_thread = threading.Thread(target=broadcast_state, daemon=True)
    broadcast_thread.start()

    try:
        print("[WEB] Server starting at http://0.0.0.0:5000")
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n[SYS] Shutting down...")
    finally:
        cleanup_gpio()
        print("[SYS] Bye!")
