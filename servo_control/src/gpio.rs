#[cfg(target_os = "linux")]
use rppal::gpio::{Gpio, InputPin, OutputPin};
#[cfg(target_os = "linux")]
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::state::{AppContext, PULSE_CENTER};

/// 高精度微秒延遲：短延遲使用 spin_loop 避免 Linux 排程器 60~80µs 喚醒延遲，長延遲結合 sleep + spin
#[inline]
pub fn delay_micros(us: u64) {
    if us == 0 {
        return;
    }
    let start = Instant::now();
    let target = Duration::from_micros(us);
    if us > 250 {
        thread::sleep(Duration::from_micros(us - 150));
    }
    while start.elapsed() < target {
        std::hint::spin_loop();
    }
}

pub struct HardwareGpio {
    pub is_raspberry_pi: bool,
    #[cfg(target_os = "linux")]
    outputs: Mutex<HashMap<u8, OutputPin>>,
    #[cfg(target_os = "linux")]
    inputs: Mutex<HashMap<u8, InputPin>>,
}

impl HardwareGpio {
    pub fn new() -> Self {
        #[cfg(target_os = "linux")]
        {
            if let Ok(gpio) = Gpio::new() {
                let mut outputs = HashMap::new();
                let mut inputs = HashMap::new();

                // 步進馬達輸出腳位: X(12,16,20), Z(13,19,26) + 4組電磁鐵(24,18,14,15)
                for pin_num in [12u8, 16, 20, 13, 19, 26, 24, 18, 14, 15] {
                    if let Ok(p) = gpio.get(pin_num) {
                        let mut out = p.into_output_low();
                        out.set_low();
                        outputs.insert(pin_num, out);
                    }
                }

                // 數位輸入腳位 (內部上拉 PullUp): 接近開關(17,25,27,22,23) + 8顆霍爾(6,7,8,9,10,11,4,21) + 衝壓訊號(5)
                for pin_num in [17u8, 25, 27, 22, 23, 6, 7, 8, 9, 10, 11, 4, 21, 5] {
                    if let Ok(p) = gpio.get(pin_num) {
                        let inp = p.into_input_pullup();
                        inputs.insert(pin_num, inp);
                    }
                }

                println!("[GPIO] Running on Raspberry Pi - Real Hardware GPIO mode (rppal)");
                return Self {
                    is_raspberry_pi: true,
                    outputs: Mutex::new(outputs),
                    inputs: Mutex::new(inputs),
                };
            }
        }

        println!("[GPIO] Running in Simulation mode (Rust Cross-Platform)");
        Self {
            is_raspberry_pi: false,
            #[cfg(target_os = "linux")]
            outputs: Mutex::new(HashMap::new()),
            #[cfg(target_os = "linux")]
            inputs: Mutex::new(HashMap::new()),
        }
    }

    #[inline]
    pub fn write_pin(&self, pin: u8, high: bool) {
        #[cfg(target_os = "linux")]
        if self.is_raspberry_pi {
            if let Ok(mut map) = self.outputs.lock() {
                if let Some(out) = map.get_mut(&pin) {
                    if high {
                        out.set_high();
                    } else {
                        out.set_low();
                    }
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (pin, high);
        }
    }

    #[inline]
    pub fn read_pin(&self, pin: u8) -> bool {
        #[cfg(target_os = "linux")]
        if self.is_raspberry_pi {
            if let Ok(map) = self.inputs.lock() {
                if let Some(inp) = map.get(&pin) {
                    return inp.is_high();
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = pin;
        }
        true
    }

    /// 支援自適應/雙極性消抖檢測：
    /// 如果 active_high 為 true 檢測高電平觸發；如果為 false 檢測低電平觸發
    #[inline]
    pub fn is_pin_active_debounced(&self, pin: u8, active_high: bool) -> bool {
        let v1 = self.read_pin(pin);
        if active_high {
            if !v1 {
                return false;
            }
            delay_micros(6);
            self.read_pin(pin)
        } else {
            if v1 {
                return false;
            }
            delay_micros(6);
            !self.read_pin(pin)
        }
    }

    /// 工業抗干擾數位濾波消抖讀取 (Active-low NPN/光耦觸發為 LOW)
    #[inline]
    pub fn is_pin_active_low_debounced(&self, pin: u8) -> bool {
        self.is_pin_active_debounced(pin, false)
    }

    /// 智慧電平跳變檢測：比對當前電平與初始基準電平，一旦發生狀態反轉立即判定觸發
    #[inline]
    pub fn is_pin_flipped(&self, pin: u8, baseline: bool) -> bool {
        let v = self.read_pin(pin);
        if v != baseline {
            delay_micros(6);
            self.read_pin(pin) != baseline
        } else {
            false
        }
    }
}

struct MotorInner {
    current_pos: f64,
    target_pos: f64,
    steps_per_unit: f64,
    max_speed_hz: f64,
    min_speed_hz: f64,
    accel_steps: f64,
    current_speed_hz: f64,
}

pub struct StepperMotor {
    pub name: String,
    pub pul_pin: u8,
    pub dir_pin: u8,
    pub ena_pin: u8,
    inner: Mutex<MotorInner>,
    pub enabled: AtomicBool,
    pub running: AtomicBool,
    pub pause_for_homing: AtomicBool,
    gpio: Arc<HardwareGpio>,
}

impl StepperMotor {
    pub fn new(name: &str, pul_pin: u8, dir_pin: u8, ena_pin: u8, gpio: Arc<HardwareGpio>) -> Arc<Self> {
        // 低電位使能 (LOW = enabled)
        gpio.write_pin(ena_pin, false);
        Arc::new(Self {
            name: name.to_string(),
            pul_pin,
            dir_pin,
            ena_pin,
            inner: Mutex::new(MotorInner {
                current_pos: 0.0,
                target_pos: 0.0,
                steps_per_unit: 320.0, // 57BYG-112 1600細分 / 5mm導程 = 320 steps/mm
                max_speed_hz: 16000.0, // 16000 Hz = 50 mm/s (大扭矩高速運轉)
                min_speed_hz: 400.0,
                accel_steps: 600.0,
                current_speed_hz: 400.0,
            }),
            enabled: AtomicBool::new(true),
            running: AtomicBool::new(true),
            pause_for_homing: AtomicBool::new(false),
            gpio,
        })
    }

    pub fn steps_per_unit(&self) -> f64 {
        self.inner.lock().unwrap().steps_per_unit
    }

    pub fn set_steps_per_unit(&self, spu: f64) {
        let mut g = self.inner.lock().unwrap();
        g.steps_per_unit = spu.max(0.1);
    }

    pub fn current_pos(&self) -> f64 {
        self.inner.lock().unwrap().current_pos
    }

    pub fn target_pos(&self) -> f64 {
        self.inner.lock().unwrap().target_pos
    }

    pub fn set_target(&self, target: f64, min_us: f64, max_us: f64) -> f64 {
        let mut g = self.inner.lock().unwrap();
        g.target_pos = target.clamp(min_us, max_us);
        g.target_pos
    }

    pub fn add_target_delta(&self, delta_us: f64, min_us: f64, max_us: f64) -> f64 {
        let mut g = self.inner.lock().unwrap();
        g.target_pos = (g.target_pos + delta_us).clamp(min_us, max_us);
        g.target_pos
    }

    pub fn set_position(&self, pos: f64) {
        let mut g = self.inner.lock().unwrap();
        g.current_pos = pos;
        g.target_pos = pos;
    }

    pub fn set_speed_params(&self, max_speed_hz: f64, accel_steps: f64) {
        let mut g = self.inner.lock().unwrap();
        g.max_speed_hz = max_speed_hz.max(100.0);
        g.accel_steps = accel_steps.max(1.0);
    }

    pub fn set_enable(&self, locked: bool) {
        self.enabled.store(locked, Ordering::Relaxed);
        // 低電位使能
        self.gpio.write_pin(self.ena_pin, !locked);
    }

    pub fn halt(&self) {
        let mut g = self.inner.lock().unwrap();
        g.target_pos = g.current_pos;
        g.current_speed_hz = g.min_speed_hz;
    }

    pub fn start_thread(motor: Arc<Self>, ctx: Arc<AppContext>) {
        thread::spawn(move || {
            while motor.running.load(Ordering::Relaxed) {
                if motor.pause_for_homing.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_millis(5));
                    continue;
                }
                if !motor.enabled.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_millis(10));
                    continue;
                }

                let (diff, dist_steps, speed_hz, batch_steps, spu) = {
                    let mut g = motor.inner.lock().unwrap();
                    let spu = g.steps_per_unit;
                    let diff = g.target_pos - g.current_pos;
                    let dist_steps = diff.abs() * spu;
                    if dist_steps >= 1.0 {
                        let batch = (dist_steps.floor() as usize).clamp(1, 40);
                        let accel = g.accel_steps.max(1.0);
                        let next_speed = if dist_steps < accel {
                            g.min_speed_hz
                                + (g.max_speed_hz - g.min_speed_hz) * (dist_steps / accel)
                        } else {
                            (g.current_speed_hz
                                + (g.max_speed_hz - g.min_speed_hz) * (batch as f64 / accel))
                                .min(g.max_speed_hz)
                        };
                        g.current_speed_hz = next_speed.clamp(g.min_speed_hz, g.max_speed_hz);
                        (diff, dist_steps, g.current_speed_hz, batch, spu)
                    } else {
                        g.current_speed_hz = g.min_speed_hz;
                        (0.0, 0.0, g.min_speed_hz, 0, spu)
                    }
                };

                if dist_steps < 1.0 || batch_steps == 0 {
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }

                // 限位與霍爾開關硬體防撞即時保護 (具備微秒級雙重採樣消抖，濾除電磁雜訊)
                if motor.gpio.is_raspberry_pi {
                    let active_high = ctx.state.read().unwrap().hall_active_high;
                    if motor.name == "x" {
                        // X 軸原點/向後限位 (GPIO 17 LIM_X_A 或 霍爾感測器 6, 7, 10, 4)
                        let lim_a_hit = motor.gpio.is_pin_active_low_debounced(17)
                            || motor.gpio.is_pin_active_debounced(6, active_high)
                            || motor.gpio.is_pin_active_debounced(7, active_high)
                            || motor.gpio.is_pin_active_debounced(10, active_high)
                            || motor.gpio.is_pin_active_debounced(4, active_high);
                        if diff < 0.0 && lim_a_hit {
                            motor.halt();
                            // 觸發原點立即自動校準歸零坐標點
                            motor.set_position(0.0);
                            {
                                let mut s = ctx.state.write().unwrap();
                                s.lim_x_a = true;
                                s.lim_left = true;
                                s.x_pulse = 0;
                                s.x_mm = 0.0;
                                s.x_pct = 0.0;
                                s.x_homed = true;
                            }
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                        // X 軸正向極限 (GPIO 27 LIM_X_C)
                        if diff > 0.0 && motor.gpio.is_pin_active_low_debounced(27) {
                            motor.halt();
                            {
                                let mut s = ctx.state.write().unwrap();
                                s.lim_x_c = true;
                                s.lim_right = true;
                            }
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    } else {
                        // Z 軸原點/安全位置 (GPIO 22 LIM_Z_D 或 霍爾感測器 8, 9, 6, 7, 11, 21)
                        let lim_origin_hit = motor.gpio.is_pin_active_low_debounced(22)
                            || motor.gpio.is_pin_active_debounced(8, active_high)
                            || motor.gpio.is_pin_active_debounced(9, active_high)
                            || motor.gpio.is_pin_active_debounced(6, active_high)
                            || motor.gpio.is_pin_active_debounced(7, active_high)
                            || motor.gpio.is_pin_active_debounced(11, active_high)
                            || motor.gpio.is_pin_active_debounced(21, active_high);
                        if diff < 0.0 && lim_origin_hit {
                            motor.halt();
                            // 觸發原點立即自動校準歸零坐標點
                            motor.set_position(0.0);
                            {
                                let mut s = ctx.state.write().unwrap();
                                s.lim_z_d = true;
                                s.lim_up = true;
                                s.z_pulse = 0;
                                s.z_mm = 0.0;
                                s.z_pct = 0.0;
                                s.y1_pulse = 0;
                                s.y2_pulse = 0;
                                s.y_mm = 0.0;
                                s.y_pct = 0.0;
                                s.z_homed = true;
                            }
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                        // Z 軸反向/下極限 (GPIO 23 LIM_Z_E)
                        if diff > 0.0 && motor.gpio.is_pin_active_low_debounced(23) {
                            motor.halt();
                            {
                                let mut s = ctx.state.write().unwrap();
                                s.lim_z_e = true;
                                s.lim_down = true;
                            }
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    }
                } else {
                    let s = ctx.state.read().unwrap();
                    if motor.name == "x" {
                        if diff < 0.0 && (s.lim_x_a || s.lim_left) {
                            drop(s);
                            motor.halt();
                            motor.set_position(0.0);
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                        if diff > 0.0 && (s.lim_x_c || s.lim_right) {
                            drop(s);
                            motor.halt();
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    } else {
                        if diff < 0.0 && (s.lim_z_d || s.lim_up || s.lim_z_e || s.lim_down) {
                            drop(s);
                            motor.halt();
                            motor.set_position(0.0);
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    }
                }

                if !motor.gpio.is_raspberry_pi {
                    // 模擬模式：高速平滑步進運動
                    thread::sleep(Duration::from_millis(10));
                    let mut g = motor.inner.lock().unwrap();
                    let rem = g.target_pos - g.current_pos;
                    let step_chunk = rem.abs().min((speed_hz / spu) * 0.01);
                    if rem > 0.0 {
                        g.current_pos += step_chunk;
                    } else {
                        g.current_pos -= step_chunk;
                    }
                    continue;
                }

                // 實際樹莓派硬體閉迴路步進輸出（微秒級精準脈衝 + 即時中斷與原點校準歸零）
                let step_val = 1.0 / spu;
                let dir_positive = diff > 0.0;
                motor.gpio.write_pin(motor.dir_pin, dir_positive);
                let step_delay_us = ((1_000_000.0 / speed_hz) as u64).saturating_sub(4).max(4);

                let mut actual_steps = 0usize;
                let mut hit_limit_stop = false;
                for _ in 0..batch_steps {
                    if motor.pause_for_homing.load(Ordering::Relaxed)
                        || !motor.enabled.load(Ordering::Relaxed)
                    {
                        break;
                    }

                    // 步進前直接硬體讀取感測器，加入二重取樣消抖，防止電磁干擾誤煞停
                    if !dir_positive {
                        // 往後/原點方向
                        let ah = ctx.state.read().unwrap().hall_active_high;
                        if motor.name == "x" {
                            if motor.gpio.is_pin_active_low_debounced(17)
                                || motor.gpio.is_pin_active_debounced(6, ah)
                                || motor.gpio.is_pin_active_debounced(7, ah)
                                || motor.gpio.is_pin_active_debounced(10, ah)
                                || motor.gpio.is_pin_active_debounced(4, ah)
                            {
                                hit_limit_stop = true;
                                break;
                            }
                        } else {
                            if motor.gpio.is_pin_active_low_debounced(22)
                                || motor.gpio.is_pin_active_debounced(8, ah)
                                || motor.gpio.is_pin_active_debounced(9, ah)
                                || motor.gpio.is_pin_active_debounced(6, ah)
                                || motor.gpio.is_pin_active_debounced(7, ah)
                                || motor.gpio.is_pin_active_debounced(11, ah)
                                || motor.gpio.is_pin_active_debounced(21, ah)
                            {
                                hit_limit_stop = true;
                                break;
                            }
                        }
                    } else {
                        // 正向/極限方向
                        if motor.name == "x" {
                            if motor.gpio.is_pin_active_low_debounced(27) {
                                hit_limit_stop = true;
                                break;
                            }
                        } else {
                            if motor.gpio.is_pin_active_low_debounced(23) {
                                hit_limit_stop = true;
                                break;
                            }
                        }
                    }

                    motor.gpio.write_pin(motor.pul_pin, true);
                    delay_micros(4);
                    motor.gpio.write_pin(motor.pul_pin, false);
                    delay_micros(step_delay_us);
                    actual_steps += 1;
                }

                if hit_limit_stop {
                    motor.halt();
                    // 觸發原點立即自動歸零
                    if !dir_positive {
                        motor.set_position(0.0);
                        let mut s = ctx.state.write().unwrap();
                        if motor.name == "x" {
                            s.lim_x_a = true;
                            s.lim_left = true;
                            s.x_pulse = 0;
                            s.x_mm = 0.0;
                            s.x_pct = 0.0;
                            s.x_homed = true;
                        } else {
                            s.lim_z_d = true;
                            s.lim_up = true;
                            s.z_pulse = 0;
                            s.z_mm = 0.0;
                            s.z_pct = 0.0;
                            s.y1_pulse = 0;
                            s.y2_pulse = 0;
                            s.y_mm = 0.0;
                            s.y_pct = 0.0;
                            s.z_homed = true;
                        }
                    }
                } else if actual_steps > 0 {
                    let delta_pos = (actual_steps as f64) * step_val;
                    let mut g = motor.inner.lock().unwrap();
                    if dir_positive {
                        g.current_pos = (g.current_pos + delta_pos).min(g.target_pos.max(g.current_pos));
                    } else {
                        g.current_pos = (g.current_pos - delta_pos).max(g.target_pos.min(g.current_pos));
                    }
                }
            }
        });
    }
}

pub fn home_axis(ctx: Arc<AppContext>, axis: &str) -> bool {
    let lock_guard = match ctx.homing_lock.try_lock() {
        Ok(g) => g,
        Err(_) => {
            ctx.add_history("已有歸零程序執行中，無視重複請求", "warn");
            return false;
        }
    };

    ctx.homing_abort.store(false, Ordering::Relaxed);
    {
        ctx.state.write().unwrap().is_homing = true;
    }

    let ax = axis.to_lowercase();
    let mut target_axes = Vec::new();
    if ax == "all" || ax == "xy" || ax == "xz" {
        target_axes.push("x");
        target_axes.push("z");
    } else if ax == "x" {
        target_axes.push("x");
    } else if matches!(ax.as_str(), "z" | "y" | "y1" | "y2") {
        target_axes.push("z");
    }

    for &a in &target_axes {
        if a == "x" {
            ctx.x_stepper.pause_for_homing.store(true, Ordering::Relaxed);
        } else {
            ctx.z_stepper.pause_for_homing.store(true, Ordering::Relaxed);
        }
    }
    thread::sleep(Duration::from_millis(20));

    if ctx.is_raspberry_pi {
        execute_hardware_homing(&ctx, &target_axes);
    } else {
        execute_simulation_homing(&ctx, &target_axes);
    }

    ctx.read_limit_switches();

    for &a in &target_axes {
        if a == "x" {
            ctx.x_stepper.pause_for_homing.store(false, Ordering::Relaxed);
        } else {
            ctx.z_stepper.pause_for_homing.store(false, Ordering::Relaxed);
        }
    }
    {
        ctx.state.write().unwrap().is_homing = false;
    }
    drop(lock_guard);
    true
}

fn execute_hardware_homing(ctx: &Arc<AppContext>, target_axes: &[&str]) {
    struct HomingCfg {
        name: &'static str,
        stepper: Arc<StepperMotor>,
        pul_pin: u8,
        dir_pin: u8,
        lim_pin: u8,
        max_bound: f64,
        steps_per_mm: f64,
        step_val: f64,
        seek_high: bool,
        backoff_high: bool,
        triggered: bool,
        sensor_hit: bool,
    }

    let (x_max_mm, z_max_mm, homing_hz, retract_mm, latch_hz, x_spu, z_spu) = {
        let s = ctx.state.read().unwrap();
        (
            s.x_max_mm,
            s.z_max_mm,
            s.homing_speed_hz.clamp(300.0, 25000.0),
            s.homing_retract_mm.max(1.0),
            s.homing_latch_hz.clamp(80.0, 3000.0),
            s.x_steps_per_mm.max(0.1),
            s.z_steps_per_mm.max(0.1),
        )
    };

    let mut cfgs = Vec::new();
    for &ax in target_axes {
        if ax == "x" {
            ctx.gpio.write_pin(ctx.x_stepper.ena_pin, false);
            cfgs.push(HomingCfg {
                name: "x",
                stepper: Arc::clone(&ctx.x_stepper),
                pul_pin: ctx.x_stepper.pul_pin,
                dir_pin: ctx.x_stepper.dir_pin,
                lim_pin: 17, // LIM_X_A
                max_bound: x_max_mm,
                steps_per_mm: x_spu,
                step_val: 1.0 / x_spu,
                seek_high: false,
                backoff_high: true,
                triggered: false,
                sensor_hit: false,
            });
        } else {
            ctx.gpio.write_pin(ctx.z_stepper.ena_pin, false);
            cfgs.push(HomingCfg {
                name: "z",
                stepper: Arc::clone(&ctx.z_stepper),
                pul_pin: ctx.z_stepper.pul_pin,
                dir_pin: ctx.z_stepper.dir_pin,
                lim_pin: 22, // LIM_Z_D
                max_bound: z_max_mm,
                steps_per_mm: z_spu,
                step_val: 1.0 / z_spu,
                seek_high: false,
                backoff_high: true,
                triggered: false,
                sensor_hit: false,
            });
        }
    }

    let active_high = ctx.state.read().unwrap().hall_active_high;

    let check_sensor = |cfg: &HomingCfg| -> bool {
        let (hall_pins, lim_pin) = if cfg.name == "x" {
            // X 軸向左歸零：覆蓋 8 顆霍爾中所有可能接線，優先 6, 7, 10, 4
            (&[6u8, 7, 10, 4, 8, 9, 11, 21][..], cfg.lim_pin)
        } else {
            // Y 軸向上歸零：覆蓋 8 顆霍爾中所有可能接線，優先 8, 9, 6, 7, 11, 21
            (&[8u8, 9, 6, 7, 11, 21, 10, 4][..], cfg.lim_pin)
        };

        // 1. 檢測接近開關 (NPN 常開低電平)
        if ctx.gpio.is_pin_active_low_debounced(lim_pin) {
            return true;
        }

        // 2. 檢測霍爾模組：同時支援 Active-Low (低電平) 與 Active-High (高電平/光耦跳線輸出高)
        for &pin in hall_pins {
            if ctx.gpio.is_pin_active_debounced(pin, active_high) {
                return true;
            }
            // 雙重自適應保險：若預設低電平未觸發，但檢測到該腳位顯著翻轉為對立電平
            if active_high && ctx.gpio.is_pin_active_debounced(pin, false) {
                return true;
            }
        }
        false
    };

    // ── 階段 0: 若開關已被觸發（或已在原點），先向正方向安全退開 ──
    let backoff_hz = (homing_hz * 0.5).clamp(400.0, 3000.0);
    let backoff_us = ((1_000_000.0 / backoff_hz) as u64).saturating_sub(4).max(10);
    for cfg in &cfgs {
        if check_sensor(cfg) {
            ctx.add_history(
                format!("{} 軸感測器處於觸發狀態，執行預先安全退開...", cfg.name.to_uppercase()),
                "info",
            );
            ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);
            let mut pre_steps = 0;
            while check_sensor(cfg) && pre_steps < 80000 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    ctx.add_history("歸零程序被中止（緊急停止）", "warn");
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                delay_micros(4);
                ctx.gpio.write_pin(cfg.pul_pin, false);
                pre_steps += 1;
                delay_micros(backoff_us);
            }
            let clearance_steps = (retract_mm * cfg.steps_per_mm) as usize;
            for _ in 0..clearance_steps {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                delay_micros(4);
                ctx.gpio.write_pin(cfg.pul_pin, false);
                delay_micros(backoff_us);
            }
            cfg.stepper.set_position(retract_mm);
        }
    }

    // ── 階段 1: 快速粗搜 (Fast Seek) ──
    let seek_us = ((1_000_000.0 / homing_hz) as u64).saturating_sub(4).max(8);
    for cfg in &mut cfgs {
        ctx.gpio.write_pin(cfg.dir_pin, cfg.seek_high);
        cfg.triggered = false;
        cfg.sensor_hit = false;
    }

    for _ in 0..800_000 {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            ctx.add_history("歸零程序被中止（緊急停止）", "warn");
            return;
        }
        let mut active_indices = Vec::new();
        for (idx, cfg) in cfgs.iter_mut().enumerate() {
            if !cfg.triggered {
                if check_sensor(cfg) {
                    cfg.triggered = true;
                    cfg.sensor_hit = true;
                } else {
                    ctx.gpio.write_pin(cfg.pul_pin, true);
                    active_indices.push(idx);
                }
            }
        }
        if active_indices.is_empty() {
            break;
        }
        delay_micros(4);
        for idx in active_indices {
            let cfg = &cfgs[idx];
            ctx.gpio.write_pin(cfg.pul_pin, false);
            let cur = cfg.stepper.current_pos();
            cfg.stepper.set_position((cur - cfg.step_val).max(0.0));
        }
        delay_micros(seek_us);
    }

    // ── 階段 2: 安全回退 (Retract) 脫開感測器 ──
    for cfg in &cfgs {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            return;
        }
        ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);
        let retract_steps = ((retract_mm * cfg.steps_per_mm) as usize).max(200);
        for _ in 0..retract_steps {
            if ctx.homing_abort.load(Ordering::Relaxed) {
                return;
            }
            ctx.gpio.write_pin(cfg.pul_pin, true);
            delay_micros(4);
            ctx.gpio.write_pin(cfg.pul_pin, false);
            delay_micros(backoff_us);
        }
    }
    thread::sleep(Duration::from_millis(50));

    // ── 階段 3: 極慢速精準捕獲邊緣 (Slow Latch) ──
    let latch_us = ((1_000_000.0 / latch_hz) as u64).saturating_sub(4).max(20);
    for cfg in &mut cfgs {
        ctx.gpio.write_pin(cfg.dir_pin, cfg.seek_high);
        cfg.triggered = false;
    }

    for _ in 0..150_000 {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            return;
        }
        let mut active_indices = Vec::new();
        for (idx, cfg) in cfgs.iter_mut().enumerate() {
            if !cfg.triggered {
                if check_sensor(cfg) {
                    cfg.triggered = true;
                } else {
                    ctx.gpio.write_pin(cfg.pul_pin, true);
                    active_indices.push(idx);
                }
            }
        }
        if active_indices.is_empty() {
            break;
        }
        delay_micros(4);
        for idx in active_indices {
            let cfg = &cfgs[idx];
            ctx.gpio.write_pin(cfg.pul_pin, false);
        }
        delay_micros(latch_us);
    }

    // ── 階段 4: 物理原點嚴格鎖定 (0.00 mm) ──
    for cfg in &cfgs {
        cfg.stepper.set_position(0.0);
        cfg.stepper.set_target(0.0, 0.0, cfg.max_bound);
        {
            let mut s = ctx.state.write().unwrap();
            if cfg.name == "x" {
                s.lim_x_a = true;
                s.lim_left = true;
                s.x_pulse = 0;
                s.x_mm = 0.0;
                s.x_pct = 0.0;
                s.x_homed = true;
            } else {
                s.lim_z_d = true;
                s.lim_up = true;
                s.z_pulse = 0;
                s.z_mm = 0.0;
                s.z_pct = 0.0;
                s.y1_pulse = 0;
                s.y2_pulse = 0;
                s.y_mm = 0.0;
                s.y_pct = 0.0;
                s.z_homed = true;
            }
        }
        ctx.add_history(
            if cfg.name == "x" {
                "X 軸向左霍爾模組兩段式精密歸零完成 (物理原點鎖定於左側 a 點：0.0 mm / 0 µs)".to_string()
            } else {
                "Z 軸霍爾模組兩段式精密歸零完成 (物理原點鎖定於頂部 d 點：0.0 mm / 0 µs)".to_string()
            },
            "success",
        );
    }
}

fn execute_simulation_homing(ctx: &Arc<AppContext>, target_axes: &[&str]) {
    let start_x = ctx.x_stepper.current_pos();
    let start_z = ctx.z_stepper.current_pos();

    // 就算坐標系目前是 0，也先向正方向安全退開再重新逼近原點，保證隨時點擊歸零皆能完整執行
    let need_sim_backoff = start_x.abs() < 2.0 || start_z.abs() < 2.0;
    if need_sim_backoff {
        ctx.add_history("當前座標已在原點 (0.0)，執行預先安全退開再重新定標歸零...", "info");
        let backoff_steps = 10;
        for i in 1..=backoff_steps {
            if ctx.homing_abort.load(Ordering::Relaxed) {
                return;
            }
            let ratio = i as f64 / backoff_steps as f64;
            for &ax in target_axes {
                if ax == "x" {
                    ctx.x_stepper.set_position(10.0 * ratio);
                } else {
                    ctx.z_stepper.set_position(10.0 * ratio);
                }
            }
            thread::sleep(Duration::from_millis(15));
        }
        thread::sleep(Duration::from_millis(40));
    }

    let seek_start_x = ctx.x_stepper.current_pos();
    let seek_start_z = ctx.z_stepper.current_pos();

    let steps = 20;
    for i in 1..=steps {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            ctx.add_history("歸零程序被中止（緊急停止）", "warn");
            return;
        }
        let ratio = i as f64 / steps as f64;
        let ease = 1.0 - (1.0 - ratio) * (1.0 - ratio);
        for &ax in target_axes {
            if ax == "x" {
                let pos = seek_start_x * (1.0 - ease);
                ctx.x_stepper.set_position(pos);
            } else {
                let pos = seek_start_z * (1.0 - ease);
                ctx.z_stepper.set_position(pos);
            }
        }
        thread::sleep(Duration::from_millis(20));
    }

    for &ax in target_axes {
        if ax == "x" {
            ctx.x_stepper.set_position(0.0);
            ctx.x_stepper.set_target(0.0, 0.0, ctx.state.read().unwrap().x_max_mm);
            let mut s = ctx.state.write().unwrap();
            s.x_pulse = 0;
            s.x_mm = 0.0;
            s.x_pct = 0.0;
            s.x_homed = true;
            ctx.add_history("X 軸兩段式精密歸零完成 (坐標點已精準歸零：0.0 mm / 0 µs)", "success");
        } else {
            ctx.z_stepper.set_position(0.0);
            ctx.z_stepper.set_target(0.0, 0.0, ctx.state.read().unwrap().z_max_mm);
            let mut s = ctx.state.write().unwrap();
            s.z_pulse = 0;
            s.z_mm = 0.0;
            s.z_pct = 0.0;
            s.y1_pulse = 0;
            s.y2_pulse = 0;
            s.y_mm = 0.0;
            s.y_pct = 0.0;
            s.z_homed = true;
            ctx.add_history("Z 軸兩段式精密歸零完成 (坐標點已精準歸零：0.0 mm / 0 µs)", "success");
        }
    }
}


