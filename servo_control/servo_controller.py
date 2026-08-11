#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
樹梅派伺服馬達控制系統
Raspberry Pi Servo Motor Controller
=====================================
X 軸: 1 顆伺服馬達 (GPIO 12)
Y1 軸: 1 顆伺服馬達 (GPIO 13)
Y2 軸: 1 顆伺服馬達 (GPIO 18)
Z 軸 (上下): GPIO 繼電器 (GPIO 25)
吸盤 VACUUM: GPIO 繼電器 (GPIO 24)
限位開關: GPIO 17(左), 27(右), 22(上), 23(下)
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

# ── GPIO 引腳定義 (BCM 編號 - 升級為步進馬達配置) ──────────────────
PIN = {
    'X_PUL':      12,   # X 軸脈衝 (Row 3 - IO12)
    'X_DIR':      16,   # X 軸方向 (Row 2 - IO16)
    'X_ENA':      20,   # X 軸致能 (Row 3 - IO20)
    'Y_PUL':      13,   # Y 軸脈衝 (Row 4 - IO13)
    'Y_DIR':      19,   # Y 軸方向 (Row 3 - IO19)
    'Y_ENA':      26,   # Y 軸致能 (Row 4 - IO26)
    'VACUUM':     24,   # 吸盤繼電器 (Row 2 - IO24)
    'Z_DOWN':     25,   # 手臂下降繼電器 (Row 2 - IO25)
    'LIM_LEFT':   17,   # 限位開關-左 (Row 1 - IO17)
    'LIM_RIGHT':  27,   # 限位開關-右 (Row 4 - IO27)
    'LIM_UP':     22,   # 限位開關-上 (Row 3 - IO22)
    'LIM_DOWN':   23,   # 限位開關-下 (Row 3 - IO23)
}

# ── 步進馬達虛擬 PWM 規格 ──────────────────────────────────────────
# 保留原伺服的 500-2500us 概念，映射到步進馬達行程
PWM_FREQ     = 50       
PULSE_MIN    = 500      
PULSE_MAX    = 2500     
PULSE_CENTER = 1500     

def pulse_to_duty(pulse_us):
    """將脈衝寬度(µs)轉換成對應的 duty cycle，用於網頁顯示"""
    period_us = 1_000_000 / PWM_FREQ   # = 20000 µs
    return (pulse_us / period_us) * 100.0

# ── 系統狀態 ──────────────────────────────────────────────────────
state = {
    # 馬達當前位置 (對應 500~2500)
    'x_pulse':  PULSE_CENTER,
    'y1_pulse': PULSE_CENTER,
    'y2_pulse': PULSE_CENTER,

    # 轉換成百分比 0~100 (0=LEFT/DOWN, 100=RIGHT/UP)
    'x_pct':  50.0,
    'y_pct':  50.0,

    # 數位輸出
    'vacuum':   False,
    'z_down':   False,

    # 數位輸入（限位）
    'lim_left':  False,
    'lim_right': False,
    'lim_up':    False,
    'lim_down':  False,

    # 系統狀態
    'system_run':    False,
    'arm_down':      False,
    'pickup_active': False,
    'error':         False,
    'error_msg':     '',

    # 範圍與物理比例設定
    'x_min': PULSE_MIN,
    'x_max': PULSE_MAX,
    'y_min': PULSE_MIN,
    'y_max': PULSE_MAX,
    'x_scale': 10.0,    # 10 µs 對應 1 mm (500~2500 行程 = 0~200 mm)
    'y_scale': 10.0,

    # 速度與加減速規劃
    'x_max_speed': 2000.0,
    'x_accel_steps': 100.0,
    'y_max_speed': 2000.0,
    'y_accel_steps': 100.0,

    # 致能狀態
    'motor_locked': True,

    # Y 軸同步模式
    'y_sync': True,

    # 歸零執行狀態
    'is_homing': False,

    # 最後更新時間
    'timestamp': '',
    'gpio_mode': 'Raspberry Pi (Real)' if IS_RASPBERRY_PI else 'Simulation',
    # 安全設定
    'lock_timeout_min': 30,
}

homing_lock = threading.Lock()


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
        
        # 速度規劃參數
        # ⚠️ 注意：標準 Linux kernel 下 Python time.sleep() 精度約為 1~10ms，
        #    在真實 Pi 上超過 ~500Hz 將導致脈衝不穩。正式生產建議改用 pigpio 硬體 DMA 脈衝。
        self.max_speed_hz = 2000.0
        self.min_speed_hz = 200.0
        self.accel_steps = 100.0
        self.current_speed_hz = 200.0
        
        self.enabled = True
        self.running = True
        # [修正1] 歸零時暫停背景執行緒以避免競態條件 (Race Condition)
        self.pause_for_homing = False
        
        if IS_RASPBERRY_PI:
            GPIO.setup(self.pul_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.ena_pin, GPIO.OUT, initial=GPIO.LOW) # 低電位使能

        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_target(self, target):
        self.target_pos = max(500.0, min(2500.0, float(target)))

    def set_position(self, pos):
        self.current_pos = float(pos)
        self.target_pos = float(pos)

    def set_enable(self, value):
        self.enabled = value
        if IS_RASPBERRY_PI:
            # 共陰極接線下，ENA- 接地，ENA+ (GPIO) 輸出 LOW 鎖定馬達，輸出 HIGH 釋放馬達手推
            GPIO.output(self.ena_pin, GPIO.LOW if value else GPIO.HIGH)

    def halt(self):
        """[修正1] 立即停止：將目標位置設為當前位置，並重置速度"""
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
                
                # 限位開關安全防護
                if diff < 0 and self.name == 'x' and state['lim_left']:
                    time.sleep(0.01)
                    continue
                if diff > 0 and self.name == 'x' and state['lim_right']:
                    time.sleep(0.01)
                    continue
                if diff < 0 and self.name == 'y' and state['lim_down']:
                    time.sleep(0.01)
                    continue
                if diff > 0 and self.name == 'y' and state['lim_up']:
                    time.sleep(0.01)
                    continue

                if not IS_RASPBERRY_PI:
                    # 模擬模式：平滑移動
                    time.sleep(1.0 / self.current_speed_hz)
                    if diff > 0:
                        self.current_pos += step_val
                    else:
                        self.current_pos -= step_val
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
y_stepper = None

def init_gpio():
    """初始化 GPIO 引腳與步進馬達控制器"""
    global x_stepper, y_stepper
    
    # 建立/初始化步進馬達背景控制器
    if x_stepper is None:
        x_stepper = StepperMotor('x', PIN['X_PUL'], PIN['X_DIR'], PIN['X_ENA'])
    else:
        x_stepper.set_position(PULSE_CENTER)
        
    if y_stepper is None:
        y_stepper = StepperMotor('y', PIN['Y_PUL'], PIN['Y_DIR'], PIN['Y_ENA'])
    else:
        y_stepper.set_position(PULSE_CENTER)

    if not IS_RASPBERRY_PI:
        return

    # 數位輸出
    for pin_name in ['VACUUM', 'Z_DOWN']:
        GPIO.setup(PIN[pin_name], GPIO.OUT, initial=GPIO.LOW)

    # 數位輸入（限位開關）
    for pin_name in ['LIM_LEFT', 'LIM_RIGHT', 'LIM_UP', 'LIM_DOWN']:
        GPIO.setup(PIN[pin_name], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        try:
            GPIO.remove_event_detect(PIN[pin_name])
        except Exception:
            pass
        try:
            GPIO.add_event_detect(
                PIN[pin_name], GPIO.BOTH,
                callback=lambda ch: read_limit_switches(),
                bouncetime=50
            )
        except Exception as e:
            print(f"[GPIO] Warning: add_event_detect failed for {pin_name} ({e}), polling fallback active.")

    print("[GPIO] Initialized stepper motor system successfully")

def cleanup_gpio():
    """清理 GPIO"""
    global x_stepper, y_stepper
    if x_stepper:
        x_stepper.stop()
    if y_stepper:
        y_stepper.stop()
    if not IS_RASPBERRY_PI:
        return
    GPIO.cleanup()

def set_servo_pulse(axis, pulse_us):
    """設定馬達目標位置"""
    pulse_us = max(PULSE_MIN, min(PULSE_MAX, pulse_us))

    if axis == 'x':
        if x_stepper:
            x_stepper.set_target(pulse_us)

    elif axis in ('y', 'y1', 'y2'):
        # 由於 Y1/Y2 同步或單一 Y 步進馬達，我們統一設定 y_stepper 的目標
        if y_stepper:
            y_stepper.set_target(pulse_us)

def set_digital_output(name, value):
    """設定數位輸出"""
    if name == 'vacuum':
        state['vacuum'] = value
        if IS_RASPBERRY_PI:
            GPIO.output(PIN['VACUUM'], GPIO.HIGH if value else GPIO.LOW)

    elif name == 'z_down':
        state['z_down'] = value
        state['arm_down'] = value
        if IS_RASPBERRY_PI:
            GPIO.output(PIN['Z_DOWN'], GPIO.HIGH if value else GPIO.LOW)

def read_limit_switches():
    """讀取所有限位開關"""
    global x_stepper, y_stepper
    if IS_RASPBERRY_PI:
        state['lim_left']  = not GPIO.input(PIN['LIM_LEFT'])   # PULLUP: 低電位=觸發
        state['lim_right'] = not GPIO.input(PIN['LIM_RIGHT'])
        state['lim_up']    = not GPIO.input(PIN['LIM_UP'])
        state['lim_down']  = not GPIO.input(PIN['LIM_DOWN'])
    else:
        # 模擬模式：根據位置自動模擬限位
        if x_stepper:
            state['lim_left']  = x_stepper.current_pos <= state['x_min'] + 10
            state['lim_right'] = x_stepper.current_pos >= state['x_max'] - 10
        if y_stepper:
            state['lim_down']  = y_stepper.current_pos <= state['y_min'] + 10
            state['lim_up']    = y_stepper.current_pos >= state['y_max'] - 10

def _home_single_axis(axis_name):
    """單軸高速高精度歸零處理（含微秒脈衝控制與即時位置廣播）"""
    global x_stepper, y_stepper
    stepper = x_stepper if axis_name == 'x' else y_stepper
    if not stepper:
        return

    lim_pin = PIN['LIM_LEFT'] if axis_name == 'x' else PIN['LIM_DOWN']
    state_lim_key = 'lim_left' if axis_name == 'x' else 'lim_down'
    min_pos = state['x_min'] if axis_name == 'x' else state['y_min']
    
    if IS_RASPBERRY_PI:
        stepper.pause_for_homing = True
        time.sleep(0.02)  # 等待背景迴圈進入暫停狀態
        try:
            # 致能並設定負方向
            GPIO.output(stepper.ena_pin, GPIO.LOW)
            GPIO.output(stepper.dir_pin, GPIO.LOW)
            
            # 高速逼近限位開關 (2500 Hz: 每步 400µs)
            target_hz = 2500.0
            step_interval = 1.0 / target_hz
            steps = 0
            max_steps = 12000  # 安全最大脈衝數
            step_val = 1.0 / stepper.steps_per_unit  # 1 步對應 Position unit
            
            while steps < max_steps:
                if not GPIO.input(lim_pin):  # 低電位觸發 (PUD_UP)
                    state[state_lim_key] = True
                    break
                
                t_step_start = time.perf_counter()
                
                # 發送步進脈衝 (HIGH -> 4us -> LOW)
                GPIO.output(stepper.pul_pin, GPIO.HIGH)
                t_pulse = time.perf_counter() + 0.000004
                while time.perf_counter() < t_pulse:
                    pass
                GPIO.output(stepper.pul_pin, GPIO.LOW)
                
                steps += 1
                stepper.current_pos = max(min_pos, stepper.current_pos - step_val)
                
                # 先 sleep 釋放 GIL（保留最後 200µs 做忙等精準補正），避免長期佔用 GIL 凍結 SocketIO
                t_rem = (t_step_start + step_interval) - time.perf_counter()
                if t_rem > 0.0002:
                    time.sleep(t_rem - 0.0002)
                while time.perf_counter() < (t_step_start + step_interval):
                    pass
            
            if steps >= max_steps:
                add_history(f"{axis_name.upper()} 軸歸零警告: 超時未觸發現位開關", "warn")
                print(f"[HOME] Warning: {axis_name.upper()} axis homing step limit reached")
            else:
                add_history(f"{axis_name.upper()} 軸觸發現位開關", "info")

            # 退開限位開關 (Back-off 250 步 @ 1000 Hz ~0.25s)
            GPIO.output(stepper.dir_pin, GPIO.HIGH)  # 正方向向右/向上
            backoff_interval = 1.0 / 1000.0
            for _ in range(250):
                t_b_start = time.perf_counter()
                GPIO.output(stepper.pul_pin, GPIO.HIGH)
                t_pulse = time.perf_counter() + 0.000004
                while time.perf_counter() < t_pulse:
                    pass
                GPIO.output(stepper.pul_pin, GPIO.LOW)
                
                # 先 sleep 釋放 GIL（保留最後 200µs 做忙等精準補正）
                t_rem = (t_b_start + backoff_interval) - time.perf_counter()
                if t_rem > 0.0002:
                    time.sleep(t_rem - 0.0002)
                while time.perf_counter() < (t_b_start + backoff_interval):
                    pass
                    
            stepper.set_position(min_pos)
        finally:
            stepper.pause_for_homing = False
    else:
        # 模擬模式：瞬間回原點
        stepper.set_position(min_pos)
        time.sleep(0.1)

    stepper.set_target(min_pos)
    add_history(f"{axis_name.upper()} 軸歸零完成 (已回原點 0%)", "success")


def home_axis(axis='all'):
    """馬達歸零（回原點）
    [修復重點]
    1. 支援雙軸並行 (Parallel) 歸零，總耗時縮短為原本的半數以下 (< 2 秒)。
    2. 使用高精度脈衝定時與實時位置更新，大幅消除卡頓與介面凍結。
    3. 加入 homing_lock 重複觸發保護。
    """
    global homing_lock
    if not homing_lock.acquire(blocking=False):
        add_history("已有歸零程序執行中，無視重複請求", "warn")
        return
        
    state['is_homing'] = True
    try:
        if axis == 'all':
            print("[HOME] Starting parallel all-axis homing...")
            tx = threading.Thread(target=_home_single_axis, args=('x',), daemon=True)
            ty = threading.Thread(target=_home_single_axis, args=('y',), daemon=True)
            tx.start()
            ty.start()
            tx.join()
            ty.join()
        elif axis in ('x', 'y'):
            print(f"[HOME] Starting {axis.upper()} axis homing...")
            _home_single_axis(axis)
    finally:
        state['is_homing'] = False
        homing_lock.release()

# ── 背景廣播執行緒 ────────────────────────────────────────────────
broadcast_lock = threading.Lock()

def broadcast_state():
    """每 100ms 廣播狀態到所有客戶端"""
    global x_stepper, y_stepper
    sim_t = 0
    while True:
        try:
            # 同步實時馬達位置與百分比至系統狀態中
            if x_stepper:
                state['x_pulse'] = int(x_stepper.current_pos)
                state['x_pct'] = max(0.0, min(100.0, (x_stepper.current_pos - state['x_min']) / max(1, state['x_max'] - state['x_min']) * 100.0))
            if y_stepper:
                state['y1_pulse'] = int(y_stepper.current_pos)
                state['y2_pulse'] = int(y_stepper.current_pos)
                state['y_pct'] = max(0.0, min(100.0, (y_stepper.current_pos - state['y_min']) / max(1, state['y_max'] - state['y_min']) * 100.0))

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
    data = request.get_json(silent=True) or {}
    axis   = data.get('axis', 'x')       # 'x', 'y1', 'y2', 'y'
    delta  = int(data.get('delta', 0))   # µs 增量
    target = data.get('target')           # 直接指定 µs

    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動，請按 START'})

    try:
        if axis == 'x':
            new_pulse = state['x_pulse'] + delta if target is None else int(target)
            diff = new_pulse - state['x_pulse']
            # 限位保護
            if state['lim_left'] and diff < 0:
                return jsonify({'ok': False, 'msg': 'X 軸左側限位'})
            if state['lim_right'] and diff > 0:
                return jsonify({'ok': False, 'msg': 'X 軸右側限位'})
            new_pulse = max(state['x_min'], min(state['x_max'], new_pulse))
            set_servo_pulse('x', new_pulse)

        elif axis in ('y', 'y1', 'y2'):
            new_pulse = state['y1_pulse'] + delta if target is None else int(target)
            diff = new_pulse - state['y1_pulse']
            if state['lim_up']   and diff > 0:
                return jsonify({'ok': False, 'msg': 'Y 軸上側限位'})
            if state['lim_down'] and diff < 0:
                return jsonify({'ok': False, 'msg': 'Y 軸下側限位'})
            new_pulse = max(state['y_min'], min(state['y_max'], new_pulse))
            set_servo_pulse('y1', new_pulse)
            if state['y_sync'] or axis == 'y':
                set_servo_pulse('y2', new_pulse)
            elif axis == 'y2':
                set_servo_pulse('y2', new_pulse)

        return jsonify({'ok': True, 'x_pulse': state['x_pulse'],
                        'y1_pulse': state['y1_pulse'], 'y2_pulse': state['y2_pulse']})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)})

@app.route('/api/set_position', methods=['POST'])
def api_set_position():
    """透過百分比 0~100 設定馬達位置"""
    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動，請按 START'})

    data = request.get_json(silent=True) or {}
    axis = data.get('axis', 'x')
    pct  = float(data.get('pct', 50))

    if axis == 'x':
        pulse = state['x_min'] + (state['x_max'] - state['x_min']) * pct / 100
        diff = pulse - state['x_pulse']
        if state['lim_left'] and diff < 0:
            return jsonify({'ok': False, 'msg': 'X 軸左側限位'})
        if state['lim_right'] and diff > 0:
            return jsonify({'ok': False, 'msg': 'X 軸右側限位'})
        set_servo_pulse('x', int(pulse))
    elif axis in ('y', 'y1'):
        pulse = state['y_min'] + (state['y_max'] - state['y_min']) * pct / 100
        diff = pulse - state['y1_pulse']
        if state['lim_up'] and diff > 0:
            return jsonify({'ok': False, 'msg': 'Y 軸上側限位'})
        if state['lim_down'] and diff < 0:
            return jsonify({'ok': False, 'msg': 'Y 軸下側限位'})
        set_servo_pulse('y1', int(pulse))
        if state['y_sync']:
            set_servo_pulse('y2', int(pulse))
    elif axis == 'y2':
        pulse = state['y_min'] + (state['y_max'] - state['y_min']) * pct / 100
        diff = pulse - state['y2_pulse']
        if state['lim_up'] and diff > 0:
            return jsonify({'ok': False, 'msg': 'Y 軸上側限位'})
        if state['lim_down'] and diff < 0:
            return jsonify({'ok': False, 'msg': 'Y 軸下側限位'})
        set_servo_pulse('y2', int(pulse))

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
                    
                    while macro_running and not macro_paused:
                        if axis == 'x' and x_stepper:
                            if abs(x_stepper.current_pos - target) < 5:
                                break
                        elif axis in ('y', 'y1', 'y2') and y_stepper:
                            if abs(y_stepper.current_pos - target) < 5:
                                break
                        else:
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
                
        elif op == 'VACUUM':
            if len(parts) >= 2:
                val = parts[1].upper() == 'ON'
                set_digital_output('vacuum', val)
                add_history(f"巨集指令: 吸盤 {'開啟' if val else '關閉'}", "info")
                time.sleep(0.5)
            else:
                add_history("巨集錯誤: VACUUM 指令參數不足", "error")
                
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
            # home_axis() 內部 join() 確保完成後才返回，不需要額外 sleep
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
    # [修正2] E-STOP：立即將步進馬達目標位置設為當前位置，強制煞車並解除歸零暫停
    if x_stepper:
        x_stepper.pause_for_homing = False
        x_stepper.halt()
    if y_stepper:
        y_stepper.pause_for_homing = False
        y_stepper.halt()
    set_digital_output('vacuum', False)
    set_digital_output('z_down', False)
    add_history("系統緊急停止（重置數位輸出＋強制馬達煞車）", "warn")
    return jsonify({'ok': True})

@app.route('/api/home', methods=['POST'])
def api_home():
    if state.get('is_homing', False):
        return jsonify({'ok': False, 'msg': '歸零流程執行中，請稍候...'})
    data = request.get_json(silent=True) or {}
    axis = data.get('axis', 'all')
    add_history(f"手動點擊馬達歸零 ({axis.upper()})", "info")
    t = threading.Thread(target=home_axis, args=(axis,), daemon=True)
    t.start()
    return jsonify({'ok': True, 'msg': f'歸零中: {axis}'})

@app.route('/api/vacuum', methods=['POST'])
def api_vacuum():
    data = request.get_json(silent=True) or {}
    value = bool(data.get('on', False))
    set_digital_output('vacuum', value)
    add_history(f"手動切換吸盤: {'開啟' if value else '關閉'}", "info")
    return jsonify({'ok': True, 'vacuum': state['vacuum']})

@app.route('/api/z_down', methods=['POST'])
def api_z_down():
    data = request.get_json(silent=True) or {}
    value = bool(data.get('on', False))
    set_digital_output('z_down', value)
    state['arm_down'] = value
    add_history(f"手動切換手臂: {'下降' if value else '上升'}", "info")
    return jsonify({'ok': True, 'z_down': state['z_down']})

@app.route('/api/pickup', methods=['POST'])
def api_pickup():
    """自動取件流程"""
    if not state['system_run']:
        return jsonify({'ok': False, 'msg': '系統未啟動'})

    def _wait_motor_reach(motor, target, tolerance=10, timeout=10.0):
        """[修正3] 等待步進馬達抵達目標位置，並支援 system_run / timeout 中斷"""
        start = time.time()
        while state['pickup_active'] and state['system_run']:
            if motor is None or abs(motor.current_pos - target) < tolerance:
                return True
            if time.time() - start > timeout:
                add_history(f"自動取件警告：馬達到位超時（目標={target}µs，當前={motor.current_pos:.0f}µs）", "warn")
                return False
            time.sleep(0.05)
        return False  # 被 STOP 中斷

    def pickup_sequence():
        state['pickup_active'] = True
        add_history("自動取件程序開始", "success")
        try:
            # 1. 系統初始化
            socketio.emit('pickup_step', {'step': 'init'})
            # [修正3] 每步前先確認系統仍在運行，遇到 STOP 立刻跳出
            if not state['system_run']: return
            time.sleep(0.3)

            # 2. 回原點
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'home1'})
            home_axis('all')
            # 歸零後位置是 x_min（500），等待到達 x_min 而非 PULSE_CENTER
            _wait_motor_reach(x_stepper, state['x_min'], timeout=15.0)
            time.sleep(0.3)

            # 3. 確認模具
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-mold'})
            time.sleep(0.5)

            # 4. 移動到模具一（[修正3] 到位輪詢取代固定 sleep）
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'move-mold1'})
            x_pickup = state['x_max'] - 200
            set_servo_pulse('x', x_pickup)
            add_history(f"移動至取料點: {x_pickup}µs", "info")
            if not _wait_motor_reach(x_stepper, x_pickup): return

            # 5. 手臂下降與磁吸工件
            if not state['system_run']: return
            set_digital_output('z_down', True)
            add_history("手臂下降中", "info")
            time.sleep(0.8)  # 繼電器動作時間固定等待

            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'suck'})
            set_digital_output('vacuum', True)
            add_history("吸盤開啟並吸附工件", "info")
            time.sleep(0.5)

            # 6. 確認已磁吸？
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-suck'})
            time.sleep(0.5)

            # 7. 手臂上升與移動到模具二
            if not state['system_run']: return
            set_digital_output('z_down', False)
            add_history("吸附成功，手臂上升", "info")
            time.sleep(0.5)

            # [修正3] 到位輪詢
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'move-mold2'})
            x_place = state['x_min'] + 200
            set_servo_pulse('x', x_place)
            add_history(f"移動至放料點: {x_place}µs", "info")
            if not _wait_motor_reach(x_stepper, x_place): return

            # 8. 手臂下降與放下工件
            if not state['system_run']: return
            set_digital_output('z_down', True)
            add_history("到達放料點，手臂下降", "info")
            time.sleep(0.8)

            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'drop'})
            set_digital_output('vacuum', False)
            add_history("吸盤關閉，工件已釋放", "info")
            time.sleep(0.5)

            # 9. 確認已放下？
            if not state['system_run']: return
            socketio.emit('pickup_step', {'step': 'check-drop'})
            time.sleep(0.5)

            # 10. 回原點
            if not state['system_run']: return
            set_digital_output('z_down', False)
            time.sleep(0.3)

            socketio.emit('pickup_step', {'step': 'home2'})
            set_servo_pulse('x', PULSE_CENTER)
            _wait_motor_reach(x_stepper, PULSE_CENTER)
            add_history("自動取件程序完成，回到起點", "success")
        except Exception as e:
            state['error'] = True
            state['error_msg'] = str(e)
            add_history(f"自動取件異常: {e}", "error")
        finally:
            state['pickup_active'] = False
            socketio.emit('pickup_step', {'step': 'idle'})

    t = threading.Thread(target=pickup_sequence, daemon=True)
    t.start()
    return jsonify({'ok': True, 'msg': '自動取件流程啟動'})

@app.route('/api/set_limits', methods=['POST'])
def api_set_limits():
    """設定馬達行程範圍與速度、校準參數"""
    data = request.get_json(silent=True) or {}
    if 'x_min' in data: state['x_min'] = max(PULSE_MIN, int(data['x_min']))
    if 'x_max' in data: state['x_max'] = min(PULSE_MAX, int(data['x_max']))
    if 'y_min' in data: state['y_min'] = max(PULSE_MIN, int(data['y_min']))
    if 'y_max' in data: state['y_max'] = min(PULSE_MAX, int(data['y_max']))
    if 'y_sync' in data: state['y_sync'] = bool(data['y_sync'])
    
    if 'x_max_speed' in data:
        state['x_max_speed'] = float(data['x_max_speed'])
        if x_stepper: x_stepper.max_speed_hz = float(data['x_max_speed'])
    if 'x_accel_steps' in data:
        state['x_accel_steps'] = float(data['x_accel_steps'])
        if x_stepper: x_stepper.accel_steps = float(data['x_accel_steps'])
    if 'y_max_speed' in data:
        state['y_max_speed'] = float(data['y_max_speed'])
        if y_stepper: y_stepper.max_speed_hz = float(data['y_max_speed'])
    if 'y_accel_steps' in data:
        state['y_accel_steps'] = float(data['y_accel_steps'])
        if y_stepper: y_stepper.accel_steps = float(data['y_accel_steps'])
        
    if 'x_scale' in data: state['x_scale'] = float(data['x_scale'])
    if 'y_scale' in data: state['y_scale'] = float(data['y_scale'])
    
    add_history("更新系統與馬達參數設定", "success")
    return jsonify({'ok': True, 'limits': {
        'x_min': state['x_min'], 'x_max': state['x_max'],
        'y_min': state['y_min'], 'y_max': state['y_max'],
        'y_sync': state['y_sync'],
        'x_max_speed': state['x_max_speed'], 'x_accel_steps': state['x_accel_steps'],
        'y_max_speed': state['y_max_speed'], 'y_accel_steps': state['y_accel_steps'],
        'x_scale': state['x_scale'], 'y_scale': state['y_scale']
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
    """馬達診斷資訊"""
    diag = {
        'gpio_mode': state['gpio_mode'],
        'is_raspberry_pi': IS_RASPBERRY_PI,
        'pwm_freq': PWM_FREQ,
        'motors': {
            'X':  {'pin': PIN['X_PUL'],  'pulse': state['x_pulse'],  'duty': round(pulse_to_duty(state['x_pulse']), 3)},
            'Y1': {'pin': PIN['Y_PUL'], 'pulse': state['y1_pulse'], 'duty': round(pulse_to_duty(state['y1_pulse']), 3)},
            'Y2': {'pin': PIN['Y_PUL'], 'pulse': state['y2_pulse'], 'duty': round(pulse_to_duty(state['y2_pulse']), 3)},
        },
        'digital_outputs': {
            'VACUUM': {'pin': PIN['VACUUM'], 'state': state['vacuum']},
            'Z_DOWN': {'pin': PIN['Z_DOWN'], 'state': state['z_down']},
        },
        'limit_switches': {
            'LEFT':  {'pin': PIN['LIM_LEFT'],  'triggered': state['lim_left']},
            'RIGHT': {'pin': PIN['LIM_RIGHT'], 'triggered': state['lim_right']},
            'UP':    {'pin': PIN['LIM_UP'],    'triggered': state['lim_up']},
            'DOWN':  {'pin': PIN['LIM_DOWN'],  'triggered': state['lim_down']},
        },
        'pwm_signal_info': {
            'frequency_hz': PWM_FREQ,
            'period_ms': round(1000 / PWM_FREQ, 2),
            'x_pulse_us':  state['x_pulse'],
            'y1_pulse_us': state['y1_pulse'],
            'y2_pulse_us': state['y2_pulse'],
            'x_duty_pct':  round(pulse_to_duty(state['x_pulse']), 3),
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

@app.route('/api/terminal/kill', methods=['POST'])
def api_terminal_kill():
    """中止目前執行中的終端機指令"""
    data = request.get_json(silent=True) or {}
    token = data.get('token', '')
    if not is_valid_token(token):
        return jsonify({'ok': False, 'msg': '未授權'})
    add_history("使用者中止終端機指令", "warn")
    return jsonify({'ok': True, 'msg': '已發送中止訊號'})

# ── 系統更新 API ──────────────────────────────────────────────────
GITHUB_REPO = "Nono0325/Nice"  # GitHub 倉庫名稱
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
            local_sha = subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'],
                capture_output=True, text=True,
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            ).stdout.strip()
        except Exception:
            local_sha = 'unknown'
        
        up_to_date = local_sha != '' and remote_sha.startswith(local_sha[:7]) or local_sha == remote_sha
        
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

    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
            try:
                stdout, stderr = proc.communicate(input=user_pwd + '\n', timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.communicate()
                return jsonify({'ok': False, 'output': f'❌ 指令執行超時（超過 {timeout_sec} 秒）\n提示：長時間指令請改用 SSH 執行'})

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
            res = subprocess.run(
                cmd, shell=True, capture_output=True, text=True,
                timeout=timeout_sec, cwd=cwd, env=env
            )
            stdout = res.stdout or ''
            stderr = res.stderr or ''
            output = stdout
            if stderr.strip():
                output += f'\n[STDERR]\n{stderr}'
            if not output.strip():
                output = '(指令執行完成，無輸出內容)'
            return jsonify({'ok': res.returncode == 0, 'returncode': res.returncode,
                            'output': output, 'cwd': cwd})

    except subprocess.TimeoutExpired:
        return jsonify({'ok': False, 'output': f'❌ 指令執行超時（超過 {timeout_sec} 秒）'})
    except Exception as e:
        return jsonify({'ok': False, 'output': f'❌ 執行例外: {str(e)}'}) 

@socketio.on('cmd_move')
def on_cmd_move(data):
    """WebSocket 移動命令（低延遲）
    [修正4] 新增 y2 分支，避免傳入 y2 時被默默忽略。
    ⚠️ 注意：目前後端只有一個 y_stepper 實體，y1/y2 同步至相同硬體軸。
       若未來需要真正獨立驅動 Y1/Y2，需新增第二個 StepperMotor 實體。
    """
    axis  = data.get('axis', 'x')
    delta = int(data.get('delta', 0))
    if state['system_run']:
        if axis == 'x':
            if state['lim_left'] and delta < 0:
                return
            if state['lim_right'] and delta > 0:
                return
            new_pulse = max(state['x_min'], min(state['x_max'], state['x_pulse'] + delta))
            set_servo_pulse('x', new_pulse)
        elif axis in ('y', 'y1', 'y2'):  # [修正4] 加入 y2 分支
            if state['lim_down'] and delta < 0:
                return
            if state['lim_up'] and delta > 0:
                return
            new_pulse = max(state['y_min'], min(state['y_max'], state['y1_pulse'] + delta))
            set_servo_pulse('y1', new_pulse)
            # y_sync 開啟時或明確指定 y/y2 時同步 y2
            if state['y_sync'] or axis in ('y', 'y2'):
                set_servo_pulse('y2', new_pulse)

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
