#[cfg(target_os = "linux")]
use rppal::gpio::{Gpio, InputPin, OutputPin};
#[cfg(target_os = "linux")]
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::state::AppContext;

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

                // 數位輸入腳位 (內部上拉 PullUp): 接近開關(17,25, 2(c), 3(d), 23) + 8顆霍爾(6,7,8,9,10,11,4,21) + 衝壓訊號(5)
                for pin_num in [17u8, 25, 2, 3, 23, 6, 7, 8, 9, 10, 11, 4, 21, 5] {
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
                    let (hall_ah, xa_ah, xc_ah, zd_ah, ze_ah, en_xa, en_xc, en_zd, en_ze) = {
                        let s = ctx.state.read().unwrap();
                        (
                            s.hall_active_high,
                            s.lim_x_a_active_high,
                            s.lim_x_c_active_high,
                            s.lim_z_d_active_high,
                            s.lim_z_e_active_high,
                            s.lim_x_a_enabled,
                            s.lim_x_c_enabled,
                            s.lim_z_d_enabled,
                            s.lim_z_e_enabled,
                        )
                    };
                    if motor.name == "x" {
                        // X 軸向左 a 點原點保護 (GPIO 17 LIM_X_A 或 A 點霍爾模組 1..8 全覆蓋，若啟用)
                        let hall_hit = [6u8, 7, 8, 9, 10, 11, 4, 21]
                            .iter()
                            .any(|&p| motor.gpio.is_pin_active_debounced(p, hall_ah) || (!hall_ah && !motor.gpio.read_pin(p)));
                        let pin17_hit = motor.gpio.is_pin_active_debounced(17, xa_ah) || (!xa_ah && !motor.gpio.read_pin(17));
                        let s_hit = {
                            let s = ctx.state.read().unwrap();
                            s.lim_x_a || s.lim_left
                        };
                        let lim_a_hit = en_xa && (pin17_hit || hall_hit || s_hit);
                        if diff < 0.0 && lim_a_hit {
                            motor.halt();
                            // 觸發 a 點原點立即自動校準歸零坐標點
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
                        // X 軸向右 c 點極限 (僅檢測 GPIO 2 LIM_X_C 接近開關，若啟用且支援 NC/Active-HIGH)
                        let lim_c_hit = en_xc && motor.gpio.is_pin_active_debounced(2, xc_ah);
                        if diff > 0.0 && lim_c_hit {
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
                        // Z 軸 D 點原點/向上安全高度 (僅檢測 GPIO 3 LIM_Z_D 接近開關，若啟用且支援 NC/Active-HIGH)
                        let lim_origin_hit = en_zd && motor.gpio.is_pin_active_debounced(3, zd_ah);
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
                        // Z 軸 E 點反向/向下極限 (僅檢測 GPIO 23 LIM_Z_E 接近開關，若啟用)
                        let lim_e_hit = en_ze && motor.gpio.is_pin_active_debounced(23, ze_ah);
                        if diff > 0.0 && lim_e_hit {
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
                        if diff < 0.0 && (s.lim_z_d || s.lim_up) {
                            drop(s);
                            motor.halt();
                            motor.set_position(0.0);
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                        if diff > 0.0 && (s.lim_z_e || s.lim_down) {
                            drop(s);
                            motor.halt();
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
                        // 往後/原點方向 (向 a 點 或 向 d 點)
                        if motor.name == "x" {
                            // X 軸向左 A 點移動：檢測 A 點接近開關 (GPIO 17) 與 A 點霍爾模組 (1..8 全覆蓋)
                            let (hall_ah, xa_ah, en_xa) = {
                                let s = ctx.state.read().unwrap();
                                (s.hall_active_high, s.lim_x_a_active_high, s.lim_x_a_enabled)
                            };
                            if en_xa {
                                let hall_hit = [6u8, 7, 8, 9, 10, 11, 4, 21]
                                    .iter()
                                    .any(|&p| motor.gpio.is_pin_active_debounced(p, hall_ah) || (!hall_ah && !motor.gpio.read_pin(p)));
                                let pin17_hit = motor.gpio.is_pin_active_debounced(17, xa_ah) || (!xa_ah && !motor.gpio.read_pin(17));
                                let s_hit = {
                                    let s = ctx.state.read().unwrap();
                                    s.lim_x_a || s.lim_left
                                };
                                if pin17_hit || hall_hit || s_hit {
                                    hit_limit_stop = true;
                                    break;
                                }
                            }
                        } else {
                            // Z 軸向上 D 點移動：僅在啟用時檢測 D 點接近開關 (GPIO 3 - Pin 5)
                            let (zd_ah, en_zd) = {
                                let s = ctx.state.read().unwrap();
                                (s.lim_z_d_active_high, s.lim_z_d_enabled)
                            };
                            if en_zd && (motor.gpio.is_pin_active_debounced(3, zd_ah) || (!zd_ah && !motor.gpio.read_pin(3))) {
                                hit_limit_stop = true;
                                break;
                            }
                        }
                    } else {
                        // 正向/極限方向 (向 c 點 或 向 e 點)
                        if motor.name == "x" {
                            // X 軸向右 C 點移動：僅在啟用時檢測 C 點接近開關 (GPIO 2 - Pin 3)
                            let (xc_ah, en_xc) = {
                                let s = ctx.state.read().unwrap();
                                (s.lim_x_c_active_high, s.lim_x_c_enabled)
                            };
                            if en_xc && motor.gpio.is_pin_active_debounced(2, xc_ah) {
                                hit_limit_stop = true;
                                break;
                            }
                        } else {
                            // Z 軸向下 E 點移動：僅在啟用時檢測 E 點接近開關 (GPIO 23)
                            let (ze_ah, en_ze) = {
                                let s = ctx.state.read().unwrap();
                                (s.lim_z_e_active_high, s.lim_z_e_enabled)
                            };
                            if en_ze && motor.gpio.is_pin_active_debounced(23, ze_ah) {
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
                lim_pin: 17, // LIM_X_A (左側原點開關)
                max_bound: x_max_mm,
                steps_per_mm: x_spu,
                step_val: 1.0 / x_spu,
                seek_high: false,    // X 軸向左歸零：DIR 輸出 LOW (負方向朝 a 點)
                backoff_high: true,  // 向右安全退開：DIR 輸出 HIGH (正方向)
                triggered: false,
                sensor_hit: false,
            });
        } else {
            let zd_en = ctx.state.read().unwrap().lim_z_d_enabled;
            if !zd_en {
                ctx.add_history("Z 軸 d 點原點開關 (GPIO 3 - Pin 5) 標記為未接線/停用，直接以當前位置作為 Z 軸原點 (0.0 mm)", "info");
                ctx.z_stepper.set_position(0.0);
                {
                    let mut s = ctx.state.write().unwrap();
                    s.z_pulse = 0;
                    s.z_mm = 0.0;
                    s.z_pct = 0.0;
                    s.y1_pulse = 0;
                    s.y2_pulse = 0;
                    s.y_mm = 0.0;
                    s.y_pct = 0.0;
                    s.z_homed = true;
                }
            } else {
                ctx.gpio.write_pin(ctx.z_stepper.ena_pin, false);
                cfgs.push(HomingCfg {
                    name: "z",
                    stepper: Arc::clone(&ctx.z_stepper),
                    pul_pin: ctx.z_stepper.pul_pin,
                    dir_pin: ctx.z_stepper.dir_pin,
                    lim_pin: 3, // LIM_Z_D (GPIO 3 - Pin 5)
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
    }

    if cfgs.is_empty() {
        return;
    }

    let (active_high, xa_ah, zd_ah, en_xa, en_zd) = {
        let s = ctx.state.read().unwrap();
        (
            s.hall_active_high,
            s.lim_x_a_active_high,
            s.lim_z_d_active_high,
            s.lim_x_a_enabled,
            s.lim_z_d_enabled,
        )
    };

    let is_point_a_hit = |base_xa: Option<bool>, base_halls: Option<&[(u8, bool)]>| -> bool {
        if !en_xa {
            return false;
        }

        // 1. 接近開關 GPIO 17 檢測 (消抖、基準跳變、或常規 NPN 低電平)
        let pin17_val = ctx.gpio.read_pin(17);
        let pin17_active = ctx.gpio.is_pin_active_debounced(17, xa_ah);
        let pin17_flipped = if let Some(base) = base_xa {
            pin17_val != base
        } else {
            false
        };
        let pin17_low = !xa_ah && !pin17_val;

        if pin17_active || pin17_flipped || pin17_low {
            return true;
        }

        // 2. 霍爾模組 (覆蓋 G1~G4 所有可能接線腳位: 6, 7, 8, 9, 10, 11, 4, 21)
        for &pin in &[6u8, 7, 8, 9, 10, 11, 4, 21] {
            let p_val = ctx.gpio.read_pin(pin);
            if ctx.gpio.is_pin_active_debounced(pin, active_high) {
                return true;
            }
            if !active_high && !p_val {
                return true;
            }
        }

        // 3. 霍爾模組基準跳變檢測 (任何一顆霍爾腳位電平跳變即確認捕獲磁場)
        if let Some(bases) = base_halls {
            for &(pin, base) in bases {
                if ctx.gpio.read_pin(pin) != base {
                    return true;
                }
            }
        }

        // 4. 背景狀態檢測 (若取樣迴圈判定 lim_x_a 觸發)
        if let Ok(s) = ctx.state.read() {
            if s.lim_x_a_enabled && (s.lim_x_a || s.lim_left) {
                return true;
            }
        }

        false
    };

    let is_point_d_hit = |base_zd: Option<bool>| -> bool {
        if !en_zd {
            return false;
        }
        let pin3_val = ctx.gpio.read_pin(3);
        let pin3_active = ctx.gpio.is_pin_active_debounced(3, zd_ah);
        let pin3_flipped = if let Some(base) = base_zd {
            pin3_val != base
        } else {
            false
        };
        let pin3_low = !zd_ah && !pin3_val;
        if pin3_active || pin3_flipped || pin3_low {
            return true;
        }
        if let Ok(s) = ctx.state.read() {
            if s.lim_z_d_enabled && (s.lim_z_d || s.lim_up) {
                return true;
            }
        }
        false
    };

    let check_axis_sensor = |name: &str, base_xa: Option<bool>, base_halls: Option<&[(u8, bool)]>, base_zd: Option<bool>| -> bool {
        if name == "x" {
            is_point_a_hit(base_xa, base_halls)
        } else {
            is_point_d_hit(base_zd)
        }
    };

    // ── 階段 0: 若開關已被觸發（或已在原點），先向正方向安全退開 ──
    let backoff_hz = (homing_hz * 0.5).clamp(400.0, 3000.0);
    let backoff_us = ((1_000_000.0 / backoff_hz) as u64).saturating_sub(4).max(10);
    for cfg in &cfgs {
        if check_axis_sensor(cfg.name, None, None, None) {
            ctx.add_history(
                format!("{} 軸原點感測器處於觸發狀態，執行預先安全退開...", cfg.name.to_uppercase()),
                "info",
            );
            ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);
            let mut pre_steps = 0;
            while check_axis_sensor(cfg.name, None, None, None) && pre_steps < 3000 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    ctx.add_history("歸零程序被中止（緊急停止）", "warn");
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                delay_micros(4);
                ctx.gpio.write_pin(cfg.pul_pin, false);
                pre_steps += 1;
                if pre_steps % 50 == 0 {
                    ctx.read_limit_switches();
                }
                delay_micros(backoff_us);
            }
            let clearance_steps = ((retract_mm * cfg.steps_per_mm) as usize).max(300);
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
            thread::sleep(Duration::from_millis(50));
            ctx.read_limit_switches();
        }
    }

    // 採集 Phase 1 尋標前穩定乾淨的初始基準電平（動態邊緣跳變偵測保險）
    let base_xa = ctx.gpio.read_pin(17);
    let base_zd = ctx.gpio.read_pin(3);
    let base_halls: Vec<(u8, bool)> = [6u8, 7, 8, 9, 10, 11, 4, 21]
        .iter()
        .map(|&p| (p, ctx.gpio.read_pin(p)))
        .collect();

    // ── 階段 1: 快速粗搜 (Fast Seek) ──
    let seek_us = ((1_000_000.0 / homing_hz) as u64).saturating_sub(4).max(8);
    for cfg in &mut cfgs {
        ctx.gpio.write_pin(cfg.dir_pin, cfg.seek_high);
        cfg.triggered = false;
        cfg.sensor_hit = false;
    }

    let max_seek_steps = ((x_max_mm * x_spu * 1.5) as usize).clamp(10000, 300000);
    for step_cnt in 0..max_seek_steps {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            ctx.add_history("歸零程序被中止（緊急停止）", "warn");
            return;
        }
        if step_cnt % 50 == 0 {
            ctx.read_limit_switches();
        }

        let mut active_indices = Vec::new();
        for (idx, cfg) in cfgs.iter_mut().enumerate() {
            if !cfg.triggered {
                let hit = check_axis_sensor(cfg.name, Some(base_xa), Some(&base_halls), Some(base_zd));
                if hit {
                    cfg.triggered = true;
                    cfg.sensor_hit = true;
                } else {
                    ctx.gpio.write_pin(cfg.pul_pin, true);
                    active_indices.push(idx);
                }
            }
        }

        if active_indices.is_empty() {
            // 所有軸原點皆已觸發，立即煞停！
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

    // ── 階段 2: 智慧安全回退 (Retract) ──
    // 逐步退開直到感測器徹底脫開，再加退安全餘量
    for cfg in &cfgs {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            return;
        }
        ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);

        // 逐步退開直到感測器信號斷開
        let mut retract_steps = 0;
        let max_disengage = ((retract_mm * 3.0 * cfg.steps_per_mm) as usize).max(3000);
        while check_axis_sensor(cfg.name, None, None, None) && retract_steps < max_disengage {
            if ctx.homing_abort.load(Ordering::Relaxed) {
                return;
            }
            ctx.gpio.write_pin(cfg.pul_pin, true);
            delay_micros(4);
            ctx.gpio.write_pin(cfg.pul_pin, false);
            retract_steps += 1;
            if retract_steps % 50 == 0 {
                ctx.read_limit_switches();
            }
            delay_micros(backoff_us);
        }

        // 額外退開 retract_mm (預設 3.0 mm) 確保遠離感應遲滯區
        let clearance_steps = ((retract_mm * cfg.steps_per_mm) as usize).max(300);
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
    thread::sleep(Duration::from_millis(50));
    ctx.read_limit_switches();

    // 採集 Phase 3 慢速逼近前的全新基準電平
    let base_xa_p3 = ctx.gpio.read_pin(17);
    let base_zd_p3 = ctx.gpio.read_pin(3);
    let base_halls_p3: Vec<(u8, bool)> = [6u8, 7, 8, 9, 10, 11, 4, 21]
        .iter()
        .map(|&p| (p, ctx.gpio.read_pin(p)))
        .collect();

    // ── 階段 3: 極慢速精準捕獲邊緣 (Slow Latch) ──
    let latch_us = ((1_000_000.0 / latch_hz) as u64).saturating_sub(4).max(20);
    for cfg in &mut cfgs {
        ctx.gpio.write_pin(cfg.dir_pin, cfg.seek_high);
        cfg.triggered = false;
        cfg.sensor_hit = false;
    }

    let max_latch_steps = ((retract_mm * 2.5 * x_spu) as usize).max(3000);
    for step_cnt in 0..max_latch_steps {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            return;
        }
        if step_cnt % 50 == 0 {
            ctx.read_limit_switches();
        }

        let mut active_indices = Vec::new();
        for (idx, cfg) in cfgs.iter_mut().enumerate() {
            if !cfg.triggered {
                let hit = check_axis_sensor(cfg.name, Some(base_xa_p3), Some(&base_halls_p3), Some(base_zd_p3));
                if hit {
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
        delay_micros(latch_us);
    }

    // ── 階段 4: 物理原點嚴格鎖定 (0.00 mm) ──
    for cfg in &cfgs {
        cfg.stepper.halt();
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
                "X 軸高精準兩段式歸零成功 (物理原點鎖定於 a 點: 0.0 mm / 0 µs)".to_string()
            } else {
                "Z 軸高精準兩段式歸零成功 (物理原點鎖定於 d 點: 0.0 mm / 0 µs)".to_string()
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


