#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
衝壓機自動化搬運與步進馬達控制系統
Raspberry Pi Stepper Motor & Electromagnet Controller
=====================================================
硬體定義與感測器配置：
1. X 軸: 步進馬達 (PUL: GPIO 12, DIR: GPIO 16, ENA: GPIO 20)
   - a 點 (左側歸零): 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (GPIO 17)
   - b 點 (中間模具工位): 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (GPIO 25)
   - c 點 (向右極限/放料): 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (GPIO 27)
2. Z 軸 (垂直升降，原 Y 軸改為 Z 軸): 步進馬達 (PUL: GPIO 13, DIR: GPIO 19, ENA: GPIO 26)
   - d 點 (向上歸零/安全高度): 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (GPIO 22)
   - e 點 (向下極限/取放深度): 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (GPIO 23)
3. 末端手臂:
   - 圓形電磁鐵強吸力吸盤 (直流 DC 5V/6V/12V/24V，繼電器驅動 GPIO 24)
   - 金屬吸附檢測霍爾感測器 (GPIO 6，檢測是否確實吸起金屬工件)
4. 模具衝壓交握:
   - 衝壓機衝壓完成回授訊號 (GPIO 5，光耦隔離輸入)
"""

import os
import sys
import time
import math
import threading
import json
import subprocess
import secrets
from datetime import datetime

# ── Flask / SocketIO ──────────────────────────────────────────────
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'servo_control_secret_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

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

    # 圓形電磁鐵強吸力吸盤 (直流 DC 5V/6V/12V/24V 繼電器驅動)
    'MAGNET':         24,   # 電磁鐵繼電器輸出 (Row 2 - IO24)
    'VACUUM':         24,   # 向下相容別名
    'Z_DOWN':         25,   # 保留舊手臂繼電器標籤兼容

    # 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 (金屬感應導通至 GND -> 樹莓派內部上拉 PUD_UP 讀取為 LOW)
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

    # 手臂金屬檢測霍爾感測器 [新增]
    'HALL_ARM_METAL': 6,    # 手臂霍爾感測 (Pin 31 - IO6) 檢測工件是否被電磁鐵吸住
    'PRESS_DONE':     5,    # 衝壓模具衝壓完成回授訊號 (Pin 29 - IO5)
}

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

    # 數位輸出 (圓形電磁鐵強吸力吸盤)
    'magnet':   False,
    'vacuum':   False,   # 相容舊版名稱
    'z_down':   False,

    # 數位輸入（電感式接近開關 HN-1805-E1 方形 24V NPN 常開）
    'lim_x_a':        False,  # a 點: X 軸歸零 (原 lim_left)
    'lim_x_b':        False,  # b 點: X 軸中間工位 [新增]
    'lim_x_c':        False,  # c 點: X 軸向右極限 (原 lim_right)
    'lim_z_d':        False,  # d 點: Z 軸向上歸零 (原 lim_up)
    'lim_z_e':        False,  # e 點: Z 軸向下極限 (原 lim_down)
    # 手臂金屬檢測霍爾感測器
    'hall_arm_metal': False,  # 手臂霍爾感測是否有金屬工件被吸
    'press_done':     False,  # 模具衝壓完成訊號

    # 舊有名稱相容鏡像
    'lim_left':  False,
    'lim_mid':   False,
    'lim_right': False,
    'lim_up':    False,
    'lim_down':  False,
    'hall_metal': False,

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
    'x_max_mm': 200.0,
    'z_min_mm': 0.0,
    'z_max_mm': 200.0,
    'y_min_mm': 0.0,
    'y_max_mm': 200.0,
    'x_scale': 10.0,     # 10 µs 對應 1 mm (500~2500 行程 = 0~200 mm)
    'z_scale': 10.0,
    'y_scale': 10.0,

    # 速度與加減速規劃
    'x_max_speed': 2000.0,
    'x_accel_steps': 100.0,
    'z_max_speed': 2000.0,
    'z_accel_steps': 100.0,
    'y_max_speed': 2000.0,
    'y_accel_steps': 100.0,
    'homing_speed_hz': 400.0,  # 緩慢穩定歸零頻率

    # 致能狀態
    'motor_locked': True,

    # 同步模式
    'y_sync': True,
    'z_sync': True,

    # 歸零執行狀態
    'is_homing': False,

    # 最後更新時間
    'timestamp': '',
    'gpio_mode': 'Raspberry Pi (Real)' if IS_RASPBERRY_PI else 'Simulation',
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
        self.max_speed_hz = 2000.0
        self.min_speed_hz = 200.0
        self.accel_steps = 100.0
        self.current_speed_hz = 200.0
        
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
                
                # 電感式接近開關 HN-1805-E1 方形 24V NPN 常開 安全防護（立即強制煞車，避免持續衝撞）
                if diff < 0 and self.name == 'x' and (state.get('lim_x_a') or state.get('lim_left')):
                    self.halt()
                    time.sleep(0.01)
                    continue
                if diff > 0 and self.name == 'x' and (state.get('lim_x_c') or state.get('lim_right')):
                    self.halt()
                    time.sleep(0.01)
                    continue
                # Z 軸垂直升降：diff > 0 向上靠近 d 點 (向上歸零)，diff < 0 向下靠近 e 點 (向下極限)
                if diff < 0 and self.name in ('z', 'y') and (state.get('lim_z_e') or state.get('lim_down')):
                    self.halt()
                    time.sleep(0.01)
                    continue
                if diff > 0 and self.name in ('z', 'y') and (state.get('lim_z_d') or state.get('lim_up')):
                    self.halt()
                    time.sleep(0.01)
                    continue

                if not IS_RASPBERRY_PI:
                    # 模擬模式：以 50Hz 平滑步進精確模擬運動，杜絕 Windows timer 睡眠延遲與高 CPU 占用
                    time.sleep(0.02)
                    step_chunk = min(abs(diff), (self.current_speed_hz / self.steps_per_unit) * 0.02)
                    if diff > 0:
                        self.current_pos += step_chunk
                    else:
                        self.current_pos -= step_chunk
                    continue

                # 實際樹梅派模式
                direction = GPIO.HIGH if diff > 0 else GPIO.LOW
                GPIO.output(self.dir_pin, direction)

                # 發送步進脈衝 (優化高精度定時，避免雙重 time.sleep 瓶頸)
                GPIO.output(self.pul_pin, GPIO.HIGH)
                t_pulse = time.perf_counter() + 0.000004
                while time.perf_counter() < t_pulse:
                    pass
                GPIO.output(self.pul_pin, GPIO.LOW)

                step_delay = 1.0 / self.current_speed_hz
                t_next = time.perf_counter() + step_delay
                # 先 sleep 釋放 GIL（保留最後 200µs 做忙等精準補正）
                if step_delay > 0.0002:
                    time.sleep(step_delay - 0.0002)
                while time.perf_counter() < t_next:
                    pass

                if diff > 0:
                    self.current_pos += step_val
                else:
                    self.current_pos -= step_val
            else:
                self.current_speed_hz = self.min_speed_hz
                time.sleep(0.005)

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

    # 數位輸出 (圓形電磁鐵繼電器)
    for pin_name in ['MAGNET', 'VACUUM']:
        GPIO.setup(PIN[pin_name], GPIO.OUT, initial=GPIO.LOW)

    # 數位輸入（電感式接近開關 HN-1805-E1 方形 24V NPN 常開 + 手臂金屬霍爾感測器 + 衝壓完成訊號）
    # NPN 常開開關在感應金屬時拉低至 GND，樹莓派端配置 PUD_UP 內部上拉，讀入為 LOW 時表示觸發
    input_pins = ['LIM_X_A', 'LIM_X_B', 'LIM_X_C', 'LIM_Z_D', 'LIM_Z_E', 'HALL_ARM_METAL', 'PRESS_DONE']
    for pin_name in input_pins:
        p = PIN[pin_name]
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        try:
            GPIO.remove_event_detect(p)
        except Exception:
            pass
        try:
            GPIO.add_event_detect(
                p, GPIO.BOTH,
                callback=lambda ch: read_limit_switches(),
                bouncetime=30
            )
        except Exception as e:
            print(f"[GPIO] Warning: add_event_detect failed for {pin_name} ({e}), polling fallback active.")

    print("[GPIO] Initialized stepper motor system, HN-1805-E1 sensors, and electromagnet successfully")

def cleanup_gpio():
    """清理 GPIO"""
    global x_stepper, z_stepper, y_stepper
    if x_stepper:
        x_stepper.stop()
    if z_stepper:
        z_stepper.stop()
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
    """設定數位輸出 (圓形電磁鐵吸盤 / 手臂)"""
    name = name.lower()
    if name in ('magnet', 'vacuum'):
        state['magnet'] = bool(value)
        state['vacuum'] = bool(value)
        if IS_RASPBERRY_PI:
            GPIO.output(PIN['MAGNET'], GPIO.HIGH if value else GPIO.LOW)

    elif name == 'z_down':
        state['z_down'] = bool(value)
        state['arm_down'] = bool(value)

def read_limit_switches():
    """讀取所有電感式接近開關 (HN-1805-E1 方形 24V NPN 常開) 與 手臂霍爾感測器"""
    global x_stepper, z_stepper, y_stepper
    if IS_RASPBERRY_PI:
        # NPN 常開開關感應金屬時輸出接通 GND(0V)，在內部上拉下讀入為 LOW -> not 為 True
        state['lim_x_a'] = not GPIO.input(PIN['LIM_X_A'])
        state['lim_x_b'] = not GPIO.input(PIN['LIM_X_B'])
        state['lim_x_c'] = not GPIO.input(PIN['LIM_X_C'])
        state['lim_z_d'] = not GPIO.input(PIN['LIM_Z_D'])
        state['lim_z_e'] = not GPIO.input(PIN['LIM_Z_E'])
        state['hall_arm_metal'] = not GPIO.input(PIN['HALL_ARM_METAL'])
        state['press_done'] = not GPIO.input(PIN['PRESS_DONE'])
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
        state['hall_arm_metal'] = state.get('magnet', False) # 磁鐵吸附時模擬感測到金屬
        state['press_done'] = True

    # 同步舊有名稱鏡像
    state['lim_left'] = state['lim_x_a']
    state['lim_mid'] = state['lim_x_b']
    state['lim_right'] = state['lim_x_c']
    state['lim_up'] = state['lim_z_d']
    state['lim_down'] = state['lim_z_e']
    state['hall_metal'] = state['hall_arm_metal']

def _execute_hardware_homing(target_axes):
    """樹梅派真實 GPIO 兩階段高精準歸零核心 (X回a點, Z回d點向上歸零)"""
    global x_stepper, z_stepper, y_stepper, homing_abort_event
    
    axis_configs = []
    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        if ax == 'x':
            lim_pin = PIN['LIM_X_A']
            state_key = 'lim_x_a'
            home_pos = state['x_min']
            seek_dir = GPIO.LOW      # 負方向朝 a 點歸零
            backoff_dir = GPIO.HIGH  # 正方向退開
            step_sign = -1.0
        else: # z or y
            lim_pin = PIN['LIM_Z_D']
            state_key = 'lim_z_d'
            home_pos = state.get('z_max', state['y_max'])
            seek_dir = GPIO.HIGH     # 正方向朝 d 點向上歸零
            backoff_dir = GPIO.LOW   # 負方向向下退開
            step_sign = 1.0

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
            'step_val': step_val,
            'step_sign': step_sign,
            'seek_dir': seek_dir,
            'backoff_dir': backoff_dir,
            'triggered': False,
            'timed_out': False
        })

    # ── 階段 A: 預先檢查 - 若原先已壓在接近開關上，先慢速退開 ────
    for cfg in axis_configs:
        if not GPIO.input(cfg['lim_pin']):  # PUD_UP: 低電位 = 觸發
            add_history(f"{cfg['name'].upper()} 軸接近開關處於壓下狀態，執行預先退開...", "info")
            GPIO.output(cfg['dir_pin'], cfg['backoff_dir'])
            pre_step_interval = 1.0 / 250.0
            pre_steps = 0
            while not GPIO.input(cfg['lim_pin']) and pre_steps < 3000:
                if homing_abort_event.is_set():
                    add_history("歸零程序被中止（緊急停止）", "warn")
                    return
                t_p_start = time.perf_counter()
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                t_pulse = time.perf_counter() + 0.000004
                while time.perf_counter() < t_pulse:
                    pass
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                pre_steps += 1
                t_rem = (t_p_start + pre_step_interval) - time.perf_counter()
                if t_rem > 0.0002:
                    time.sleep(t_rem - 0.0002)
                while time.perf_counter() < (t_p_start + pre_step_interval):
                    pass
            
            # 再多退 50 步作安全淨空
            for _ in range(50):
                if homing_abort_event.is_set():
                    return
                t_p_start = time.perf_counter()
                GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                t_pulse = time.perf_counter() + 0.000004
                while time.perf_counter() < t_pulse:
                    pass
                GPIO.output(cfg['pul_pin'], GPIO.LOW)
                t_rem = (t_p_start + pre_step_interval) - time.perf_counter()
                if t_rem > 0.0002:
                    time.sleep(t_rem - 0.0002)
                while time.perf_counter() < (t_p_start + pre_step_interval):
                    pass

    # ── 階段 B: 平穩逼近 (Toward Origin / Seek) ───────────────────
    homing_hz = max(150.0, min(800.0, float(state.get('homing_speed_hz', 400.0))))
    step_interval = 1.0 / homing_hz
    max_steps = 20000

    for cfg in axis_configs:
        GPIO.output(cfg['dir_pin'], cfg['seek_dir'])
        cfg['triggered'] = False

    steps = 0
    while steps < max_steps:
        if homing_abort_event.is_set():
            add_history("歸零程序被中止（緊急停止）", "warn")
            return

        t_step_start = time.perf_counter()
        active_pulse = []

        for cfg in axis_configs:
            if not cfg['triggered']:
                if not GPIO.input(cfg['lim_pin']):  # 觸發現位接近開關
                    cfg['triggered'] = True
                    state[cfg['state_key']] = True
                    add_history(f"{cfg['name'].upper()} 軸平穩觸發現位接近開關", "info")
                else:
                    GPIO.output(cfg['pul_pin'], GPIO.HIGH)
                    active_pulse.append(cfg)

        if not active_pulse:
            break

        t_pulse = time.perf_counter() + 0.000004
        while time.perf_counter() < t_pulse:
            pass

        for cfg in active_pulse:
            GPIO.output(cfg['pul_pin'], GPIO.LOW)
            cfg['stepper'].current_pos += cfg['step_sign'] * cfg['step_val']

        steps += 1
        t_rem = (t_step_start + step_interval) - time.perf_counter()
        if t_rem > 0.0002:
            time.sleep(t_rem - 0.0002)
        while time.perf_counter() < (t_step_start + step_interval):
            pass

    for cfg in axis_configs:
        if not cfg['triggered']:
            cfg['timed_out'] = True
            add_history(f"{cfg['name'].upper()} 軸歸零警告: 超時未觸發接近開關", "warn")

    # ── 階段 C: 精準退開至開關剛釋放 (Pull-off / Latch) ─────────────
    backoff_hz = 250.0
    backoff_interval = 1.0 / backoff_hz
    for cfg in axis_configs:
        if cfg['timed_out'] or homing_abort_event.is_set():
            continue

        GPIO.output(cfg['dir_pin'], cfg['backoff_dir'])
        
        bo_steps = 0
        while not GPIO.input(cfg['lim_pin']) and bo_steps < 2000:
            if homing_abort_event.is_set():
                return
            t_b_start = time.perf_counter()
            GPIO.output(cfg['pul_pin'], GPIO.HIGH)
            t_pulse = time.perf_counter() + 0.000004
            while time.perf_counter() < t_pulse:
                pass
            GPIO.output(cfg['pul_pin'], GPIO.LOW)
            bo_steps += 1
            t_rem = (t_b_start + backoff_interval) - time.perf_counter()
            if t_rem > 0.0002:
                time.sleep(t_rem - 0.0002)
            while time.perf_counter() < (t_b_start + backoff_interval):
                pass

        for _ in range(25):
            if homing_abort_event.is_set():
                return
            t_b_start = time.perf_counter()
            GPIO.output(cfg['pul_pin'], GPIO.HIGH)
            t_pulse = time.perf_counter() + 0.000004
            while time.perf_counter() < t_pulse:
                pass
            GPIO.output(cfg['pul_pin'], GPIO.LOW)
            t_rem = (t_b_start + backoff_interval) - time.perf_counter()
            if t_rem > 0.0002:
                time.sleep(t_rem - 0.0002)
            while time.perf_counter() < (t_b_start + backoff_interval):
                pass

        # 標記歸零點座標
        cfg['stepper'].set_position(cfg['home_pos'])
        cfg['stepper'].set_target(cfg['home_pos'])
        state[cfg['state_key']] = False
        add_history(f"{cfg['name'].upper()} 軸平穩歸零完成 (已精準校準原點)", "success")

def _execute_simulation_homing(target_axes):
    """模擬環境平滑歸零核心 (X回a點, Z回d點)"""
    global x_stepper, z_stepper, y_stepper, homing_abort_event
    
    start_positions = {}
    target_positions = {}
    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        start_positions[ax] = float(st.current_pos)
        target_positions[ax] = float(state['x_min'] if ax == 'x' else state.get('z_max', state['y_max']))

    steps_cnt = 30
    for i in range(steps_cnt):
        if homing_abort_event.is_set():
            add_history("歸零程序被中止（緊急停止）", "warn")
            return
        
        ratio = (i + 1) / steps_cnt
        for ax in target_axes:
            st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
            st.current_pos = start_positions[ax] - (start_positions[ax] - target_positions[ax]) * ratio
        time.sleep(0.035)

    # 模擬觸發現位瞬間反饋
    for ax in target_axes:
        key = 'lim_x_a' if ax == 'x' else 'lim_z_d'
        state[key] = True
        state['lim_left' if ax == 'x' else 'lim_up'] = True
    time.sleep(0.1)

    for ax in target_axes:
        st = x_stepper if ax == 'x' else (z_stepper or y_stepper)
        st.set_position(target_positions[ax])
        st.set_target(target_positions[ax])
        key = 'lim_x_a' if ax == 'x' else 'lim_z_d'
        state[key] = False
        state['lim_left' if ax == 'x' else 'lim_up'] = False
        add_history(f"{ax.upper()} 軸平穩歸零完成 (已回原點)", "success")

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
    while True:
        try:
            # 同步實時馬達位置與百分比、物理 mm 座標至系統狀態中
            if x_stepper:
                state['x_pulse'] = int(x_stepper.current_pos)
                state['x_pct'] = max(0.0, min(100.0, (x_stepper.current_pos - state['x_min']) / max(1, state['x_max'] - state['x_min']) * 100.0))
                state['x_mm'] = round(max(0.0, (x_stepper.current_pos - state['x_min']) / max(0.001, state['x_scale'])), 1)
            
            st_z = z_stepper or y_stepper
            if st_z:
                z_min = state.get('z_min', state['y_min'])
                z_max = state.get('z_max', state['y_max'])
                z_scale = state.get('z_scale', state['y_scale'])
                state['z_pulse'] = int(st_z.current_pos)
                state['z_pct'] = max(0.0, min(100.0, (st_z.current_pos - z_min) / max(1, z_max - z_min) * 100.0))
                state['z_mm'] = round(max(0.0, (st_z.current_pos - z_min) / max(0.001, z_scale)), 1)

                # 同步保持 y 鍵名以確保所有舊版前端元件皆向下相容
                state['y1_pulse'] = state['z_pulse']
                state['y2_pulse'] = state['z_pulse']
                state['y_pct'] = state['z_pct']
                state['y_mm'] = state['z_mm']

            # 計算當前 mm 極限（以 x_min/z_min 為起點 0.0 mm）
            state['x_min_mm'] = 0.0
            state['x_max_mm'] = round((state['x_max'] - state['x_min']) / max(0.001, state['x_scale']), 1)
            z_scale = state.get('z_scale', state['y_scale'])
            state['z_min_mm'] = 0.0
            state['z_max_mm'] = round((state.get('z_max', state['y_max']) - state.get('z_min', state['y_min'])) / max(0.001, z_scale), 1)
            state['y_min_mm'] = state['z_min_mm']
            state['y_max_mm'] = state['z_max_mm']

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

            payload = {
                **state,
                'pwm_history': {
                    'x':  list(pwm_history['x']),
                    'y1': list(pwm_history['y1']),
                    'y2': list(pwm_history['y2']),
                },
                'pin_config': PIN,
            }
            socketio.emit('state_update', payload)

        except Exception as e:
            pass

        time.sleep(0.1)

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
        if axis == 'x':
            if delta_us is not None:
                delta_us = float(delta_us)
                if (state.get('lim_x_a') or state.get('lim_left')) and delta_us < 0:
                    return jsonify({'ok': False, 'msg': 'X 軸 a 點左側限位觸發'})
                if (state.get('lim_x_c') or state.get('lim_right')) and delta_us > 0:
                    return jsonify({'ok': False, 'msg': 'X 軸 c 點右側限位觸發'})
                if x_stepper:
                    x_stepper.add_target_delta(delta_us, state['x_min'], state['x_max'])
            elif target_us is not None:
                target_us = float(target_us)
                if x_stepper:
                    x_stepper.set_target(target_us, state['x_min'], state['x_max'])

        elif axis in ('z', 'y', 'y1', 'y2'):
            st_z = z_stepper or y_stepper
            z_min = state.get('z_min', state['y_min'])
            z_max = state.get('z_max', state['y_max'])
            if delta_us is not None:
                delta_us = float(delta_us)
                if (state.get('lim_z_e') or state.get('lim_down')) and delta_us < 0:
                    return jsonify({'ok': False, 'msg': 'Z 軸 e 點下極限觸發'})
                if (state.get('lim_z_d') or state.get('lim_up')) and delta_us > 0:
                    return jsonify({'ok': False, 'msg': 'Z 軸 d 點上歸零觸發'})
                if st_z:
                    st_z.add_target_delta(delta_us, z_min, z_max)
            elif target_us is not None:
                target_us = float(target_us)
                if st_z:
                    st_z.set_target(target_us, z_min, z_max)

        return jsonify({'ok': True, 'x_pulse': state['x_pulse'],
                        'z_pulse': state['z_pulse'], 'y1_pulse': state['y1_pulse'], 'y2_pulse': state['y2_pulse']})
    except Exception as e:
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

# ── 操作歷史紀錄與巨集功能 ───────────────────────────────────────────
HISTORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'history_log.json')
MACROS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'macros.json')

def load_history():
    if os.path.exists(HISTORY_FILE):
        try:
            with open(HISTORY_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return []
    return []

def save_history(history_list):
    try:
        with open(HISTORY_FILE, 'w', encoding='utf-8') as f:
            json.dump(history_list, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[ERROR] Failed to save history: {e}")

def add_history(msg, type='info'):
    time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    log_entry = {
        'time': time_str,
        'msg': msg,
        'type': type
    }
    history = load_history()
    history.append(log_entry)
    if len(history) > 500:
        history.pop(0)
    save_history(history)
    socketio.emit('new_history', log_entry)

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

def run_macro_loop(commands):
    global macro_running, macro_paused
    macro_running = True
    macro_paused = False
    
    add_history("巨集執行開始", "success")
    socketio.emit('macro_status', {'running': True, 'paused': False, 'current_line': 0})
    
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
        
        if op == 'MOVE':
            if len(parts) >= 3:
                axis = parts[1].lower()
                try:
                    target = float(parts[2])
                    set_servo_pulse(axis, target)
                    add_history(f"巨集指令: 移動 {axis.upper()} 至 {target} µs", "info")
                    
                    st = x_stepper if axis == 'x' else (z_stepper or y_stepper)
                    min_l = state['x_min'] if axis == 'x' else state.get('z_min', state['y_min'])
                    max_l = state['x_max'] if axis == 'x' else state.get('z_max', state['y_max'])
                    clamped_target = max(min_l, min(max_l, target))
                    
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
                            add_history(f"巨集警告: 移動至 {target}µs 到位超時", "warn")
                            break
                        time.sleep(0.05)
                except ValueError:
                    add_history(f"巨集錯誤: 無效的移動座標 '{parts[2]}'", "error")
            else:
                add_history("巨集錯誤: MOVE 指令參數不足", "error")
                
        elif op == 'DELAY':
            if len(parts) >= 2:
                try:
                    ms = float(parts[1])
                    add_history(f"巨集指令: 延遲 {ms} 毫秒", "info")
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
            if len(parts) >= 2:
                val = parts[1].upper() == 'ON'
                set_digital_output('magnet', val)
                add_history(f"巨集指令: 電磁鐵吸盤 {'通電 ON' if val else '斷電 OFF'}", "info")
                time.sleep(0.4)
            else:
                add_history(f"巨集錯誤: {op} 指令參數不足", "error")

        elif op == 'CHECK':
            if len(parts) >= 2 and parts[1].upper() == 'METAL':
                read_limit_switches()
                if not state.get('hall_arm_metal'):
                    add_history("巨集錯誤: 手臂霍爾感測未檢測到金屬工件！", "error")
                    set_digital_output('magnet', False)
                    break
                else:
                    add_history("巨集檢測: 手臂霍爾感測金屬工件正常！", "success")
            else:
                add_history("巨集錯誤: CHECK 指令格式為 CHECK METAL", "error")

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
            
        else:
            add_history(f"巨集錯誤: 未知指令 '{op}'", "error")
            
    macro_running = False
    socketio.emit('macro_status', {'running': False, 'paused': False, 'current_line': 0})
    add_history("巨集執行結束", "success")

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

@app.route('/api/home', methods=['POST'])
def api_home():
    if state.get('is_homing', False):
        return jsonify({'ok': False, 'msg': '歸零流程執行中，請稍候...'})
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
    """控制圓形電磁鐵強吸力吸盤 (DC 12V/24V 繼電器驅動)"""
    data = request.get_json(silent=True) or {}
    value = bool(data.get('on', False))
    set_digital_output('magnet', value)
    add_history(f"手動切換電磁鐵吸盤: {'通電吸附 ON' if value else '斷電釋放 OFF'}", "info")
    return jsonify({'ok': True, 'magnet': state['magnet'], 'vacuum': state['vacuum']})

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
    if state.get('pickup_active', False):
        return jsonify({'ok': False, 'msg': '自動衝壓循環已在執行中'})

    def _wait_motor_reach(motor, target, tolerance=15, timeout=12.0):
        start = time.time()
        while state['pickup_active'] and state['system_run']:
            if motor is None or abs(motor.current_pos - target) <= tolerance:
                return True
            if time.time() - start > timeout:
                add_history(f"到位警告：馬達到位超時（目標={target:.0f}µs，當前={motor.current_pos:.0f}µs）", "warn")
                return False
            time.sleep(0.04)
        return False

    def stamping_cycle_sequence():
        state['pickup_active'] = True
        add_history("🚀 衝壓自動化取放料循環開始 (14 步標準週期)", "success")
        try:
            st_x = x_stepper
            st_z = z_stepper or y_stepper
            pos_x_a = float(state['x_min'])                                  # a 點 (歸零)
            pos_x_b = float((state['x_min'] + state['x_max']) / 2.0)        # b 點 (中間模具位)
            pos_x_c = float(state['x_max'])                                  # c 點 (右極限)
            pos_z_d = float(state.get('z_max', state['y_max']))              # d 點 (向上歸零/安全高度)
            pos_z_e = float(state.get('z_min', state['y_min']))              # e 點 (向下極限/取放深度)
            pos_z_half = float((pos_z_d + pos_z_e) / 2.0)                    # 下去一半 (50%)

            # 步驟 1: X 軸回到 a 點
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'home1', 'desc': '1. X軸回到a點(歸零)'})
            add_history("步驟 1/14: X 軸前往 a 點 (歸零點)...", "info")
            set_servo_pulse('x', pos_x_a)
            if not _wait_motor_reach(st_x, pos_x_a): return
            time.sleep(0.2)

            # 步驟 2: Z 軸下去 (到達 e 點)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'move-mold1', 'desc': '2. Z軸下降至e點'})
            add_history("步驟 2/14: Z 軸下降至 e 點 (向下極限取料面)...", "info")
            set_servo_pulse('z', pos_z_e)
            if not _wait_motor_reach(st_z, pos_z_e): return
            time.sleep(0.2)

            # 步驟 3: 圓形電磁鐵通電吸料
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'suck', 'desc': '3. 電磁鐵通電吸料'})
            add_history("步驟 3/14: 圓形電磁鐵通電，強磁力吸附金屬工件", "info")
            set_digital_output('magnet', True)
            time.sleep(0.4)

            # 步驟 4: Z 軸上去 (到達 d 點)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-mold', 'desc': '4. Z軸上升至d點'})
            add_history("步驟 4/14: Z 軸上升至 d 點 (安全高度)...", "info")
            set_servo_pulse('z', pos_z_d)
            if not _wait_motor_reach(st_z, pos_z_d): return
            time.sleep(0.2)

            # 步驟 5: 手臂霍爾感測金屬是否存在 (安全防呆檢查)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-suck', 'desc': '5. 霍爾感測金屬檢測'})
            add_history("步驟 5/14: 讀取手臂霍爾感測器，驗證金屬是否存在...", "info")
            read_limit_switches()
            if not state.get('hall_arm_metal'):
                add_history("❌ 警報：手臂霍爾未檢測到金屬工件！取料失敗，緊急中斷！", "error")
                set_digital_output('magnet', False)
                state['error'] = True
                state['error_msg'] = "手臂霍爾未檢測到金屬工件，自動循環中止"
                return
            add_history("✔ 霍爾感測確認：金屬工件吸附正常！", "success")
            time.sleep(0.2)

            # 步驟 6: X 軸到 c 點 (向右極限/放料點)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'move-mold2', 'desc': '6. X軸前往c點'})
            add_history("步驟 6/14: X 軸移動至 c 點 (向右極限放料點)...", "info")
            set_servo_pulse('x', pos_x_c)
            if not _wait_motor_reach(st_x, pos_x_c): return
            time.sleep(0.2)

            # 步驟 7: Z 軸下去 (到達 e 點)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'drop', 'desc': '7. Z軸下降至e點'})
            add_history("步驟 7/14: Z 軸下降至 e 點 (放料深度)...", "info")
            set_servo_pulse('z', pos_z_e)
            if not _wait_motor_reach(st_z, pos_z_e): return
            time.sleep(0.2)

            # 步驟 8: 圓形電磁鐵關電 (釋放工件)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-drop', 'desc': '8. 電磁鐵關電釋放'})
            add_history("步驟 8/14: 圓形電磁鐵斷電，工件已釋放", "info")
            set_digital_output('magnet', False)
            time.sleep(0.4)

            # 步驟 9: Z 軸上去 (到達 d 點)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'home2', 'desc': '9. Z軸上升至d點'})
            add_history("步驟 9/14: Z 軸上升至 d 點 (安全高度)...", "info")
            set_servo_pulse('z', pos_z_d)
            if not _wait_motor_reach(st_z, pos_z_d): return
            time.sleep(0.2)

            # 步驟 10: X 軸到 b 點 (中間模具工位接近開關)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'move-b', 'desc': '10. X軸前往b點'})
            add_history("步驟 10/14: X 軸移動至 b 點 (中間電感接近開關工位)...", "info")
            set_servo_pulse('x', pos_x_b)
            if not _wait_motor_reach(st_x, pos_x_b): return
            time.sleep(0.2)

            # 步驟 11: Z 軸下去一半 (50% 避讓高度)
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'half-down', 'desc': '11. Z軸下去一半'})
            add_history("步驟 11/14: Z 軸下降至 50% 中間避讓高度...", "info")
            set_servo_pulse('z', pos_z_half)
            if not _wait_motor_reach(st_z, pos_z_half): return
            time.sleep(0.2)

            # 步驟 12: 等模具衝壓完
            if not state['system_run']: return
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
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'up-again', 'desc': '13. Z軸上去d點'})
            add_history("步驟 13/14: Z 軸上升至 d 點 (安全高度)...", "info")
            set_servo_pulse('z', pos_z_d)
            if not _wait_motor_reach(st_z, pos_z_d): return
            time.sleep(0.2)

            # 步驟 14: X 軸回到 a 點
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'cycle-done', 'desc': '14. X軸回a點完成'})
            add_history("步驟 14/14: X 軸回到 a 點 (原點)...", "info")
            set_servo_pulse('x', pos_x_a)
            if not _wait_motor_reach(st_x, pos_x_a): return

            add_history("🎉 恭喜！一個完整衝壓循環 (Cycle) 順利完成！", "success")
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
    return jsonify({'ok': True, 'msg': '自動衝壓搬運循環啟動'})

@app.route('/api/set_limits', methods=['POST'])
def api_set_limits():
    """設定馬達行程範圍（支援 mm 與 us）、速度與校準參數"""
    data = request.get_json(silent=True) or {}
    
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

    # 支援 mm 與 µs 設定 X 軸（完全解除 2500µs 限制，最高可設 200,000µs）
    if 'x_min_mm' in data:
        mm = float(data['x_min_mm'])
        state['x_min'] = max(0, min(SYSTEM_PULSE_MAX, int(mm * state['x_scale'])))
    elif 'x_min' in data:
        state['x_min'] = max(0, min(SYSTEM_PULSE_MAX, int(data['x_min'])))

    if 'x_max_mm' in data:
        mm = float(data['x_max_mm'])
        state['x_max'] = max(state['x_min'] + 1, min(SYSTEM_PULSE_MAX, int(state['x_min'] + mm * state['x_scale'])))
    elif 'x_max' in data:
        state['x_max'] = max(state['x_min'] + 1, min(SYSTEM_PULSE_MAX, int(data['x_max'])))

    # 支援 mm 與 µs 設定 Z/Y 軸
    z_min_in = data.get('z_min_mm', data.get('y_min_mm'))
    if z_min_in is not None:
        mm = float(z_min_in)
        scale = state.get('z_scale', state['y_scale'])
        state['z_min'] = state['y_min'] = max(0, min(SYSTEM_PULSE_MAX, int(mm * scale)))
    elif 'z_min' in data or 'y_min' in data:
        val = data.get('z_min', data.get('y_min'))
        state['z_min'] = state['y_min'] = max(0, min(SYSTEM_PULSE_MAX, int(val)))

    z_max_in = data.get('z_max_mm', data.get('y_max_mm'))
    if z_max_in is not None:
        mm = float(z_max_in)
        scale = state.get('z_scale', state['y_scale'])
        state['z_max'] = state['y_max'] = max(state['z_min'] + 1, min(SYSTEM_PULSE_MAX, int(state['z_min'] + mm * scale)))
    elif 'z_max' in data or 'y_max' in data:
        val = data.get('z_max', data.get('y_max'))
        state['z_max'] = state['y_max'] = max(state['z_min'] + 1, min(SYSTEM_PULSE_MAX, int(val)))

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
        st_z = z_stepper or y_stepper
        if st_z: st_z.max_speed_hz = spd_val
        
    z_acc = data.get('z_accel_steps', data.get('y_accel_steps'))
    if z_acc is not None:
        acc_val = float(z_acc)
        state['z_accel_steps'] = state['y_accel_steps'] = acc_val
        st_z = z_stepper or y_stepper
        if st_z: st_z.accel_steps = acc_val
        
    if 'homing_speed_hz' in data:
        state['homing_speed_hz'] = max(100.0, min(2000.0, float(data['homing_speed_hz'])))
    
    state['x_min_mm'] = 0.0
    state['x_max_mm'] = round((state['x_max'] - state['x_min']) / max(0.001, state['x_scale']), 1)
    state['z_min_mm'] = state['y_min_mm'] = 0.0
    state['z_max_mm'] = state['y_max_mm'] = round((state['z_max'] - state['z_min']) / max(0.001, state.get('z_scale', state['y_scale'])), 1)

    add_history(f"更新行程範圍與參數: X({state['x_min']}~{state['x_max']}µs, 0~{state['x_max_mm']}mm, 比例:{state['x_scale']}), Z/Y({state['z_min']}~{state['z_max']}µs, 0~{state['z_max_mm']}mm, 比例:{state.get('z_scale', state['y_scale'])})", "success")
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
    
    if action == 'start':
        if macro_running:
            return jsonify({'ok': False, 'msg': '巨集已在運行中'})
        macros = load_macros()
        commands = macros.get(name)
        if not commands:
            return jsonify({'ok': False, 'msg': '找不到該巨集'})
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
            'MAGNET': {'pin': PIN['MAGNET'], 'state': state['magnet'], 'desc': '圓形電磁鐵強吸力吸盤 (DC 12V/24V)'},
            'VACUUM': {'pin': PIN['VACUUM'], 'state': state['vacuum']},
            'Z_DOWN': {'pin': PIN.get('Z_DOWN', 25), 'state': state['z_down']},
        },
        'inductive_sensors': {
            'sensor_model': 'HN-1805-E1 方形 24V NPN 常開',
            'X_A': {'pin': PIN['LIM_X_A'], 'name': 'X軸a點(左歸零)', 'triggered': state.get('lim_x_a')},
            'X_B': {'pin': PIN['LIM_X_B'], 'name': 'X軸b點(中間工位)', 'triggered': state.get('lim_x_b')},
            'X_C': {'pin': PIN['LIM_X_C'], 'name': 'X軸c點(向右極限)', 'triggered': state.get('lim_x_c')},
            'Z_D': {'pin': PIN['LIM_Z_D'], 'name': 'Z軸d點(向上歸零)', 'triggered': state.get('lim_z_d')},
            'Z_E': {'pin': PIN['LIM_Z_E'], 'name': 'Z軸e點(向下極限)', 'triggered': state.get('lim_z_e')},
        },
        'hall_sensors': {
            'ARM_METAL': {'pin': PIN['HALL_ARM_METAL'], 'name': '手臂金屬檢測霍爾', 'triggered': state.get('hall_arm_metal')},
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
    """從 GitHub API 取得最新版本資訊（不需要驗證）"""
    try:
        import urllib.request, json as _json
        api_url = f"https://api.github.com/repos/{GITHUB_REPO}/commits/{GITHUB_BRANCH}"
        req = urllib.request.Request(api_url, headers={'User-Agent': 'ServoController/1.0'})
        with urllib.request.urlopen(req, timeout=8) as resp:
            data = _json.loads(resp.read().decode())
        
        remote_sha    = data['sha'][:7]
        remote_msg    = data['commit']['message'].split('\n')[0][:80]
        remote_author = data['commit']['author']['name']
        remote_date   = data['commit']['author']['date'][:10]
        
        # 取得本地 commit SHA
        try:
            repo_dir = get_git_repo_dir()
            local_sha = subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'],
                capture_output=True, text=True,
                cwd=repo_dir
            ).stdout.strip()
        except Exception:
            local_sha = 'unknown'
        
        up_to_date = local_sha != '' and (remote_sha.startswith(local_sha[:7]) or local_sha == remote_sha)
        
        return jsonify({
            'ok': True,
            'local_sha': local_sha,
            'remote_sha': remote_sha,
            'remote_msg': remote_msg,
            'remote_author': remote_author,
            'remote_date': remote_date,
            'up_to_date': up_to_date,
            'repo': GITHUB_REPO,
            'branch': GITHUB_BRANCH
        })
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'無法連線至 GitHub: {str(e)}'})

@app.route('/api/update/pull', methods=['POST'])
def api_update_pull():
    """執行 git pull 更新程式（需要驗證，會自動重啟服務）"""
    data  = request.get_json(silent=True) or {}
    token = data.get('token', '')
    restart = data.get('restart', True)

    if not is_valid_site_token(token):
        return jsonify({'ok': False, 'msg': '未授權，請先登入'})

    repo_dir = get_git_repo_dir()
    add_history("系統更新：開始執行 git pull", "warn")

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
    """WebSocket 移動命令（低延遲，支援 mm 累加目標 100+100）"""
    axis = data.get('axis', 'x')
    delta_mm = data.get('delta_mm')
    delta_us = data.get('delta')

    scale = state['x_scale'] if axis == 'x' else state['y_scale']
    if delta_mm is not None:
        delta = float(delta_mm) * float(scale)
    else:
        delta = float(delta_us or 0)

    if state['system_run']:
        if axis == 'x':
            if state['lim_left'] and delta < 0:
                return
            if state['lim_right'] and delta > 0:
                return
            if x_stepper:
                x_stepper.add_target_delta(delta, state['x_min'], state['x_max'])
        elif axis in ('y', 'y1', 'y2'):
            if state['lim_down'] and delta < 0:
                return
            if state['lim_up'] and delta > 0:
                return
            if y_stepper:
                y_stepper.add_target_delta(delta, state['y_min'], state['y_max'])

# ── 主程式 ────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("=" * 60)
    print("  樹梅派步進馬達控制系統 (已升級)")
    print("  Raspberry Pi Stepper Motor Controller")
    print(f"  模式: {state['gpio_mode']}")
    print("=" * 60)

    # 初始化 GPIO
    init_gpio()

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
