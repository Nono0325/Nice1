#[cfg(target_os = "linux")]
use rppal::gpio::{Gpio, InputPin, OutputPin};
#[cfg(target_os = "linux")]
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::state::{AppContext, PULSE_CENTER};

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
}

struct MotorInner {
    current_pos: f64,
    target_pos: f64,
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
    pub steps_per_unit: f64,
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
            steps_per_unit: 5.0,
            inner: Mutex::new(MotorInner {
                current_pos: PULSE_CENTER as f64,
                target_pos: PULSE_CENTER as f64,
                max_speed_hz: 2000.0,
                min_speed_hz: 200.0,
                accel_steps: 100.0,
                current_speed_hz: 200.0,
            }),
            enabled: AtomicBool::new(true),
            running: AtomicBool::new(true),
            pause_for_homing: AtomicBool::new(false),
            gpio,
        })
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
            let step_val = 1.0 / motor.steps_per_unit;
            while motor.running.load(Ordering::Relaxed) {
                if motor.pause_for_homing.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_millis(5));
                    continue;
                }
                if !motor.enabled.load(Ordering::Relaxed) {
                    thread::sleep(Duration::from_millis(10));
                    continue;
                }

                let (diff, dist_steps, speed_hz) = {
                    let mut g = motor.inner.lock().unwrap();
                    let diff = g.target_pos - g.current_pos;
                    let dist_steps = diff.abs() * motor.steps_per_unit;
                    if dist_steps >= 1.0 {
                        let next_speed = if dist_steps < g.accel_steps {
                            g.min_speed_hz
                                + (g.max_speed_hz - g.min_speed_hz) * (dist_steps / g.accel_steps)
                        } else {
                            (g.current_speed_hz
                                + (g.max_speed_hz - g.min_speed_hz) / g.accel_steps)
                                .min(g.max_speed_hz)
                        };
                        g.current_speed_hz = next_speed.clamp(g.min_speed_hz, g.max_speed_hz);
                        (diff, dist_steps, g.current_speed_hz)
                    } else {
                        g.current_speed_hz = g.min_speed_hz;
                        (0.0, 0.0, g.min_speed_hz)
                    }
                };

                if dist_steps < 1.0 {
                    thread::sleep(Duration::from_millis(4));
                    continue;
                }

                // 限位開關硬體防撞保護
                {
                    let s = ctx.state.read().unwrap();
                    if motor.name == "x" {
                        if diff < 0.0 && (s.lim_x_a || s.lim_left) {
                            drop(s);
                            motor.halt();
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
                        if diff < 0.0 && (s.lim_z_e || s.lim_down) {
                            drop(s);
                            motor.halt();
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                        if diff > 0.0 && (s.lim_z_d || s.lim_up) {
                            drop(s);
                            motor.halt();
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    }
                }

                if !motor.gpio.is_raspberry_pi {
                    // 模擬模式：50Hz 平滑步進運動
                    thread::sleep(Duration::from_millis(20));
                    let mut g = motor.inner.lock().unwrap();
                    let rem = g.target_pos - g.current_pos;
                    let step_chunk = rem.abs().min((speed_hz / motor.steps_per_unit) * 0.02);
                    if rem > 0.0 {
                        g.current_pos += step_chunk;
                    } else {
                        g.current_pos -= step_chunk;
                    }
                    continue;
                }

                // 實際樹莓派硬體步進輸出（Rust 無 GIL 鎖定，原生執行緒微秒級精準脈衝）
                motor.gpio.write_pin(motor.dir_pin, diff > 0.0);
                motor.gpio.write_pin(motor.pul_pin, true);
                thread::sleep(Duration::from_micros(6));
                motor.gpio.write_pin(motor.pul_pin, false);

                let step_delay_us = ((1_000_000.0 / speed_hz) as u64).saturating_sub(6).max(150);
                thread::sleep(Duration::from_micros(step_delay_us));

                {
                    let mut g = motor.inner.lock().unwrap();
                    if diff > 0.0 {
                        g.current_pos += step_val;
                    } else {
                        g.current_pos -= step_val;
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
        home_pos: f64,
        min_bound: f64,
        max_bound: f64,
        step_val: f64,
        step_sign: f64,
        seek_high: bool,
        backoff_high: bool,
        triggered: bool,
        sensor_hit: bool,
    }

    let (x_min, x_max, z_min, z_max, homing_hz) = {
        let s = ctx.state.read().unwrap();
        (
            s.x_min as f64,
            s.x_max as f64,
            s.z_min as f64,
            s.z_max as f64,
            s.homing_speed_hz.clamp(150.0, 800.0),
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
                home_pos: x_min,
                min_bound: x_min,
                max_bound: x_max,
                step_val: 1.0 / ctx.x_stepper.steps_per_unit,
                step_sign: -1.0,
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
                home_pos: z_max,
                min_bound: z_min,
                max_bound: z_max,
                step_val: 1.0 / ctx.z_stepper.steps_per_unit,
                step_sign: 1.0,
                seek_high: true,
                backoff_high: false,
                triggered: false,
                sensor_hit: false,
            });
        }
    }

    // 階段 A: 若已壓在接近開關上，先退開
    let pre_us = (1_000_000.0 / 250.0) as u64;
    for cfg in &cfgs {
        if !ctx.gpio.read_pin(cfg.lim_pin) {
            ctx.add_history(
                format!("{} 軸接近開關處於壓下狀態，執行預先退開...", cfg.name.to_uppercase()),
                "info",
            );
            ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);
            let mut pre_steps = 0;
            while !ctx.gpio.read_pin(cfg.lim_pin) && pre_steps < 3000 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    ctx.add_history("歸零程序被中止（緊急停止）", "warn");
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                thread::sleep(Duration::from_micros(6));
                ctx.gpio.write_pin(cfg.pul_pin, false);
                let pos = (cfg.stepper.current_pos() - cfg.step_sign * cfg.step_val)
                    .clamp(cfg.min_bound, cfg.max_bound);
                cfg.stepper.set_position(pos);
                pre_steps += 1;
                thread::sleep(Duration::from_micros(pre_us));
            }
            for _ in 0..50 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                thread::sleep(Duration::from_micros(6));
                ctx.gpio.write_pin(cfg.pul_pin, false);
                let pos = (cfg.stepper.current_pos() - cfg.step_sign * cfg.step_val)
                    .clamp(cfg.min_bound, cfg.max_bound);
                cfg.stepper.set_position(pos);
                thread::sleep(Duration::from_micros(pre_us));
            }
        }
    }

    // 階段 B: 平穩逼近原點（嚴格限制不超出設定之上下限 x_min / z_max）
    let seek_us = (1_000_000.0 / homing_hz) as u64;
    for cfg in &mut cfgs {
        ctx.gpio.write_pin(cfg.dir_pin, cfg.seek_high);
        cfg.triggered = false;
        cfg.sensor_hit = false;
    }

    for _ in 0..20000 {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            ctx.add_history("歸零程序被中止（緊急停止）", "warn");
            return;
        }
        let mut active_indices = Vec::new();
        for (idx, cfg) in cfgs.iter_mut().enumerate() {
            if !cfg.triggered {
                let cur_pos = cfg.stepper.current_pos();
                let pin_low = !ctx.gpio.read_pin(cfg.lim_pin);
                let reached_soft_limit = if cfg.step_sign > 0.0 {
                    cur_pos >= cfg.home_pos
                } else {
                    cur_pos <= cfg.home_pos
                };

                if pin_low || reached_soft_limit {
                    cfg.triggered = true;
                    cfg.sensor_hit = pin_low;
                    cfg.stepper.set_position(cfg.home_pos);
                    if pin_low {
                        let mut s = ctx.state.write().unwrap();
                        if cfg.name == "x" {
                            s.lim_x_a = true;
                            s.lim_left = true;
                        } else {
                            s.lim_z_d = true;
                            s.lim_up = true;
                        }
                        drop(s);
                        ctx.add_history(
                            format!("{} 軸平穩觸發現位接近開關", cfg.name.to_uppercase()),
                            "info",
                        );
                    } else {
                        ctx.add_history(
                            format!("{} 軸已到達設定歸零上限/原點座標 ({:.0} µs)", cfg.name.to_uppercase(), cfg.home_pos),
                            "info",
                        );
                    }
                } else {
                    ctx.gpio.write_pin(cfg.pul_pin, true);
                    active_indices.push(idx);
                }
            }
        }
        if active_indices.is_empty() {
            break;
        }
        thread::sleep(Duration::from_micros(6));
        for idx in active_indices {
            let cfg = &cfgs[idx];
            ctx.gpio.write_pin(cfg.pul_pin, false);
            let pos = (cfg.stepper.current_pos() + cfg.step_sign * cfg.step_val)
                .clamp(cfg.min_bound, cfg.max_bound);
            cfg.stepper.set_position(pos);
        }
        thread::sleep(Duration::from_micros(seek_us));
    }

    // 階段 C: 精準退開至開關釋放，並強制鎖定座標於 home_pos (絕不超過上限)
    for cfg in &mut cfgs {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            return;
        }
        if cfg.sensor_hit && !ctx.gpio.read_pin(cfg.lim_pin) {
            ctx.gpio.write_pin(cfg.dir_pin, cfg.backoff_high);
            let mut bo_steps = 0;
            while !ctx.gpio.read_pin(cfg.lim_pin) && bo_steps < 2000 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                thread::sleep(Duration::from_micros(6));
                ctx.gpio.write_pin(cfg.pul_pin, false);
                bo_steps += 1;
                thread::sleep(Duration::from_micros(pre_us));
            }
            for _ in 0..25 {
                if ctx.homing_abort.load(Ordering::Relaxed) {
                    return;
                }
                ctx.gpio.write_pin(cfg.pul_pin, true);
                thread::sleep(Duration::from_micros(6));
                ctx.gpio.write_pin(cfg.pul_pin, false);
                thread::sleep(Duration::from_micros(pre_us));
            }
        }
        cfg.stepper.set_position(cfg.home_pos);
        cfg.stepper.set_target(cfg.home_pos, cfg.min_bound, cfg.max_bound);
        {
            let mut s = ctx.state.write().unwrap();
            if cfg.name == "x" {
                s.lim_x_a = false;
                s.lim_left = false;
            } else {
                s.lim_z_d = false;
                s.lim_up = false;
            }
        }
        ctx.add_history(
            format!("{} 軸平穩歸零完成 (已精準校準原點 {:.0} µs)", cfg.name.to_uppercase(), cfg.home_pos),
            "success",
        );
    }
}

fn execute_simulation_homing(ctx: &Arc<AppContext>, target_axes: &[&str]) {
    let (x_min, z_max) = {
        let s = ctx.state.read().unwrap();
        (s.x_min as f64, s.z_max as f64)
    };
    let start_x = ctx.x_stepper.current_pos();
    let start_z = ctx.z_stepper.current_pos();

    let steps = 25;
    for i in 1..=steps {
        if ctx.homing_abort.load(Ordering::Relaxed) {
            ctx.add_history("歸零程序被中止（緊急停止）", "warn");
            return;
        }
        let ratio = i as f64 / steps as f64;
        let ease = 1.0 - (1.0 - ratio) * (1.0 - ratio);
        for &ax in target_axes {
            if ax == "x" {
                let pos = start_x + (x_min - start_x) * ease;
                ctx.x_stepper.set_position(pos);
            } else {
                let pos = start_z + (z_max - start_z) * ease;
                ctx.z_stepper.set_position(pos);
            }
        }
        thread::sleep(Duration::from_millis(25));
    }

    for &ax in target_axes {
        if ax == "x" {
            ctx.x_stepper.set_position(x_min);
            ctx.add_history("X 軸平穩歸零完成 (已回原點)", "success");
        } else {
            ctx.z_stepper.set_position(z_max);
            ctx.add_history("Z 軸平穩歸零完成 (已回原點)", "success");
        }
    }
}

