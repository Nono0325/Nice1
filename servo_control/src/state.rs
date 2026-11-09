use chrono::Local;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{HashMap, VecDeque};
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use tokio::sync::broadcast;

use crate::gpio::{HardwareGpio, StepperMotor};

pub const PWM_FREQ: f64 = 50.0;
pub const PULSE_MIN: i64 = 0;
pub const PULSE_MAX: i64 = 2500;
pub const PULSE_CENTER: i64 = 1250;
pub const SYSTEM_PULSE_MAX: i64 = 200_000;
pub const MAX_HISTORY: usize = 60;

pub fn pulse_to_duty(pulse_us: i64) -> f64 {
    let period_us = 1_000_000.0 / PWM_FREQ;
    ((pulse_us as f64 / period_us) * 100.0 * 1000.0).round() / 1000.0
}

pub fn pin_config_map() -> HashMap<String, u8> {
    let mut m = HashMap::new();
    // X 軸步進馬達
    m.insert("X_PUL".into(), 12);
    m.insert("X_DIR".into(), 16);
    m.insert("X_ENA".into(), 20);
    // Z 軸步進馬達
    m.insert("Z_PUL".into(), 13);
    m.insert("Z_DIR".into(), 19);
    m.insert("Z_ENA".into(), 26);
    m.insert("Y_PUL".into(), 13);
    m.insert("Y_DIR".into(), 19);
    m.insert("Y_ENA".into(), 26);
    // 4 組圓形電磁鐵 LSD-P25/20 + JQC-3FF-S-Z 繼電器
    m.insert("MAGNET_1".into(), 24);
    m.insert("MAGNET_2".into(), 18);
    m.insert("MAGNET_3".into(), 14);
    m.insert("MAGNET_4".into(), 15);
    m.insert("MAGNET".into(), 24);
    m.insert("VACUUM".into(), 24);
    m.insert("Z_DOWN".into(), 25);
    // 電感式接近開關 QN-1805-E1 (a, b, c, d, e)
    m.insert("LIM_X_A".into(), 17);
    m.insert("LIM_X_B".into(), 25);
    m.insert("LIM_X_C".into(), 27);
    m.insert("LIM_Z_D".into(), 22);
    m.insert("LIM_Z_E".into(), 23);
    m.insert("LIM_LEFT".into(), 17);
    m.insert("LIM_MID".into(), 25);
    m.insert("LIM_RIGHT".into(), 27);
    m.insert("LIM_UP".into(), 22);
    m.insert("LIM_DOWN".into(), 23);
    // 8 顆末端金屬檢測霍爾感測器 (兩個一組共 4 組，817 光耦隔離板)
    m.insert("HALL_1".into(), 6);
    m.insert("HALL_2".into(), 7);
    m.insert("HALL_3".into(), 8);
    m.insert("HALL_4".into(), 9);
    m.insert("HALL_5".into(), 10);
    m.insert("HALL_6".into(), 11);
    m.insert("HALL_7".into(), 4);
    m.insert("HALL_8".into(), 21);
    m.insert("HALL_ARM_METAL".into(), 6);
    m.insert("PRESS_DONE".into(), 5);
    m
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemState {
    pub x_pulse: i64,
    pub z_pulse: i64,
    pub y1_pulse: i64,
    pub y2_pulse: i64,

    pub x_mm: f64,
    pub z_mm: f64,
    pub y_mm: f64,

    pub x_pct: f64,
    pub z_pct: f64,
    pub y_pct: f64,

    pub magnet: bool,
    pub magnet_1: bool,
    pub magnet_2: bool,
    pub magnet_3: bool,
    pub magnet_4: bool,
    pub magnet_active_count: usize,
    pub vacuum: bool,
    pub z_down: bool,

    pub lim_x_a: bool,
    pub lim_x_b: bool,
    pub lim_x_c: bool,
    pub lim_z_d: bool,
    pub lim_z_e: bool,

    pub hall_1: bool,
    pub hall_2: bool,
    pub hall_3: bool,
    pub hall_4: bool,
    pub hall_5: bool,
    pub hall_6: bool,
    pub hall_7: bool,
    pub hall_8: bool,
    pub hall_group_1: bool,
    pub hall_group_2: bool,
    pub hall_group_3: bool,
    pub hall_group_4: bool,
    pub hall_active_count: usize,
    pub hall_group_count: usize,
    pub hall_arm_metal: bool,
    pub press_done: bool,

    pub lim_left: bool,
    pub lim_mid: bool,
    pub lim_right: bool,
    pub lim_up: bool,
    pub lim_down: bool,
    pub hall_metal: bool,

    pub system_run: bool,
    pub arm_down: bool,
    pub pickup_active: bool,
    pub error: bool,
    pub error_msg: String,

    pub x_min: i64,
    pub x_max: i64,
    pub z_min: i64,
    pub z_max: i64,
    pub y_min: i64,
    pub y_max: i64,
    pub x_min_mm: f64,
    pub x_max_mm: f64,
    pub z_min_mm: f64,
    pub z_max_mm: f64,
    pub y_min_mm: f64,
    pub y_max_mm: f64,
    pub x_scale: f64,
    pub z_scale: f64,
    pub y_scale: f64,

    pub x_max_speed: f64,
    pub x_accel_steps: f64,
    pub z_max_speed: f64,
    pub z_accel_steps: f64,
    pub y_max_speed: f64,
    pub y_accel_steps: f64,
    pub homing_speed_hz: f64,

    pub motor_locked: bool,
    pub y_sync: bool,
    pub z_sync: bool,
    pub is_homing: bool,
    pub x_homed: bool,
    pub z_homed: bool,

    pub continuous_mode: bool,
    pub cycle_count: u64,
    pub cycle_target: u64,
    pub cycle_interval_sec: f64,
    pub uptime_sec: u64,
    pub watchdog_active: bool,
    pub magnet_on_time: f64,

    pub timestamp: String,
    pub gpio_mode: String,
    pub lock_timeout_min: u64,
}

impl SystemState {
    pub fn new(is_pi: bool) -> Self {
        Self {
            x_pulse: PULSE_CENTER,
            z_pulse: PULSE_CENTER,
            y1_pulse: PULSE_CENTER,
            y2_pulse: PULSE_CENTER,
            x_mm: 100.0,
            z_mm: 100.0,
            y_mm: 100.0,
            x_pct: 50.0,
            z_pct: 50.0,
            y_pct: 50.0,
            magnet: false,
            magnet_1: false,
            magnet_2: false,
            magnet_3: false,
            magnet_4: false,
            magnet_active_count: 0,
            vacuum: false,
            z_down: false,
            lim_x_a: false,
            lim_x_b: false,
            lim_x_c: false,
            lim_z_d: false,
            lim_z_e: false,
            hall_1: false,
            hall_2: false,
            hall_3: false,
            hall_4: false,
            hall_5: false,
            hall_6: false,
            hall_7: false,
            hall_8: false,
            hall_group_1: false,
            hall_group_2: false,
            hall_group_3: false,
            hall_group_4: false,
            hall_active_count: 0,
            hall_group_count: 0,
            hall_arm_metal: false,
            press_done: false,
            lim_left: false,
            lim_mid: false,
            lim_right: false,
            lim_up: false,
            lim_down: false,
            hall_metal: false,
            system_run: false,
            arm_down: false,
            pickup_active: false,
            error: false,
            error_msg: String::new(),
            x_min: PULSE_MIN,
            x_max: PULSE_MAX,
            z_min: PULSE_MIN,
            z_max: PULSE_MAX,
            y_min: PULSE_MIN,
            y_max: PULSE_MAX,
            x_min_mm: 0.0,
            x_max_mm: 200.0,
            z_min_mm: 0.0,
            z_max_mm: 200.0,
            y_min_mm: 0.0,
            y_max_mm: 200.0,
            x_scale: 10.0,
            z_scale: 10.0,
            y_scale: 10.0,
            x_max_speed: 8000.0,
            x_accel_steps: 50.0,
            z_max_speed: 8000.0,
            z_accel_steps: 50.0,
            y_max_speed: 8000.0,
            y_accel_steps: 50.0,
            homing_speed_hz: 2500.0,
            motor_locked: true,
            y_sync: true,
            z_sync: true,
            is_homing: false,
            x_homed: false,
            z_homed: false,
            continuous_mode: false,
            cycle_count: 0,
            cycle_target: 0,
            cycle_interval_sec: 1.0,
            uptime_sec: 0,
            watchdog_active: false,
            magnet_on_time: 0.0,
            timestamp: String::new(),
            gpio_mode: if is_pi {
                "Raspberry Pi (Real - Rust)".to_string()
            } else {
                "Simulation (Rust)".to_string()
            },
            lock_timeout_min: 30,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PwmHistory {
    pub x: Vec<f64>,
    pub y1: Vec<f64>,
    pub y2: Vec<f64>,
    pub time: Vec<usize>,
}

impl Default for PwmHistory {
    fn default() -> Self {
        Self {
            x: vec![PULSE_CENTER as f64; MAX_HISTORY],
            y1: vec![PULSE_CENTER as f64; MAX_HISTORY],
            y2: vec![PULSE_CENTER as f64; MAX_HISTORY],
            time: (0..MAX_HISTORY).collect(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HistoryEntry {
    pub time: String,
    pub msg: String,
    #[serde(rename = "type")]
    pub log_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorLogEntry {
    pub id: u64,
    pub time: String,
    pub level: String,
    pub source: String,
    pub code: String,
    pub msg: String,
    pub detail: String,
    pub resolved: bool,
}

#[derive(Debug, Clone)]
pub struct SessionInfo {
    pub password: String,
    pub expires: f64,
    pub cwd: PathBuf,
}

pub struct AppContext {
    pub base_dir: PathBuf,
    pub is_raspberry_pi: bool,
    pub start_instant: Instant,
    pub state: RwLock<SystemState>,
    pub pwm_history: Mutex<PwmHistory>,
    pub pin_config: HashMap<String, u8>,
    pub gpio: Arc<HardwareGpio>,
    pub x_stepper: Arc<StepperMotor>,
    pub z_stepper: Arc<StepperMotor>,

    pub history_buffer: Mutex<VecDeque<HistoryEntry>>,
    pub history_dirty: AtomicBool,

    pub error_buffer: Mutex<VecDeque<ErrorLogEntry>>,
    pub error_seq: std::sync::atomic::AtomicU64,
    pub error_dirty: AtomicBool,

    pub homing_lock: Mutex<()>,
    pub homing_abort: AtomicBool,

    pub macro_running: AtomicBool,
    pub macro_paused: AtomicBool,

    pub sessions: Mutex<HashMap<String, SessionInfo>>,
    pub sim_password: Mutex<String>,
    pub active_term_pid: Mutex<Option<u32>>,

    pub ws_tx: broadcast::Sender<String>,
}

impl AppContext {
    pub fn new(base_dir: PathBuf) -> Arc<Self> {
        let gpio = Arc::new(HardwareGpio::new());
        let is_pi = gpio.is_raspberry_pi;
        let state = SystemState::new(is_pi);
        let (ws_tx, _) = broadcast::channel(256);

        let x_stepper = StepperMotor::new("x", 12, 16, 20, Arc::clone(&gpio));
        let z_stepper = StepperMotor::new("z", 13, 19, 26, Arc::clone(&gpio));

        let ctx = Arc::new(Self {
            base_dir,
            is_raspberry_pi: is_pi,
            start_instant: Instant::now(),
            state: RwLock::new(state),
            pwm_history: Mutex::new(PwmHistory::default()),
            pin_config: pin_config_map(),
            gpio,
            x_stepper,
            z_stepper,
            history_buffer: Mutex::new(VecDeque::with_capacity(250)),
            history_dirty: AtomicBool::new(false),
            error_buffer: Mutex::new(VecDeque::with_capacity(500)),
            error_seq: std::sync::atomic::AtomicU64::new(1),
            error_dirty: AtomicBool::new(false),
            homing_lock: Mutex::new(()),
            homing_abort: AtomicBool::new(false),
            macro_running: AtomicBool::new(false),
            macro_paused: AtomicBool::new(false),
            sessions: Mutex::new(HashMap::new()),
            sim_password: Mutex::new("Nice".to_string()),
            active_term_pid: Mutex::new(None),
            ws_tx,
        });

        ctx.load_limits_config();
        ctx.load_history_from_disk();
        ctx.load_errors_from_disk();
        StepperMotor::start_thread(Arc::clone(&ctx.x_stepper), Arc::clone(&ctx));
        StepperMotor::start_thread(Arc::clone(&ctx.z_stepper), Arc::clone(&ctx));

        ctx
    }

    pub fn now_unix_sec() -> f64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0)
    }

    pub fn emit_ws(&self, event: &str, data: Value) {
        let msg = json!({
            "event": event,
            "data": data
        });
        if let Ok(text) = serde_json::to_string(&msg) {
            let _ = self.ws_tx.send(text);
        }
    }

    pub fn classify_error_meta(msg: &str, level: &str) -> (String, String) {
        let m = msg.to_lowercase();
        if m.contains("電磁鐵") || m.contains("過熱") || m.contains("gpio") || m.contains("看門狗") {
            ("HARDWARE".to_string(), "ERR_HARDWARE_MAGNET".to_string())
        } else if m.contains("霍爾") || m.contains("金屬") || m.contains("接近開關") || m.contains("感測") {
            ("SENSOR".to_string(), "ERR_SENSOR_CHECK".to_string())
        } else if m.contains("歸零") || m.contains("限位") || m.contains("極限") || m.contains("馬達") || m.contains("移動") {
            ("MOTION".to_string(), "ERR_MOTION_LIMIT".to_string())
        } else if m.contains("巨集") || m.contains("積木") || m.contains("衝壓") || m.contains("g-code") || m.contains("gcode") {
            ("MACRO".to_string(), "ERR_MACRO_EXEC".to_string())
        } else if m.contains("密碼") || m.contains("登入") || m.contains("權限") || m.contains("驗證") {
            ("AUTH".to_string(), "ERR_AUTH_DENIED".to_string())
        } else if m.contains("更新") || m.contains("github") || m.contains("git") {
            ("OTA".to_string(), "ERR_OTA_UPDATE".to_string())
        } else if m.contains("前端") || m.contains("js") || m.contains("連線") || m.contains("websocket") {
            ("FRONTEND".to_string(), "ERR_FRONTEND_RUNTIME".to_string())
        } else if level == "WARN" {
            ("SYSTEM".to_string(), "WARN_SYSTEM_EVENT".to_string())
        } else {
            ("SYSTEM".to_string(), "ERR_SYSTEM_FAULT".to_string())
        }
    }

    pub fn snapshot_detail(&self) -> String {
        if let Ok(s) = self.state.read() {
            format!(
                "X={:.1}mm({}µs) Z={:.1}mm({}µs) | Run={} | M={}/4 | Hall={}/8(G={}/4)",
                s.x_mm, s.x_pulse, s.z_mm, s.z_pulse, s.system_run, s.magnet_active_count, s.hall_active_count, s.hall_group_count
            )
        } else {
            "Snapshot unavailable".to_string()
        }
    }

    pub fn add_error_log(
        &self,
        level: &str,
        source: Option<&str>,
        code: Option<&str>,
        msg: impl Into<String>,
        detail: Option<String>,
    ) -> ErrorLogEntry {
        let msg_str = msg.into();
        let lvl = level.to_uppercase();
        let (auto_src, auto_code) = Self::classify_error_meta(&msg_str, &lvl);
        let src_str = source.filter(|s| !s.is_empty()).unwrap_or(&auto_src).to_uppercase();
        let code_str = code.filter(|s| !s.is_empty()).unwrap_or(&auto_code).to_uppercase();
        let det_str = detail.filter(|d| !d.is_empty()).unwrap_or_else(|| self.snapshot_detail());
        let id = self.error_seq.fetch_add(1, Ordering::Relaxed);
        let entry = ErrorLogEntry {
            id,
            time: Local::now().format("%Y-%m-%d %H:%M:%S").to_string(),
            level: lvl,
            source: src_str,
            code: code_str,
            msg: msg_str,
            detail: det_str,
            resolved: false,
        };
        {
            let mut buf = self.error_buffer.lock().unwrap();
            if buf.len() >= 500 {
                buf.pop_front();
            }
            buf.push_back(entry.clone());
        }
        self.error_dirty.store(true, Ordering::Relaxed);
        if entry.level == "ERROR" || entry.level == "CRITICAL" {
            self.save_errors_to_disk();
        }
        self.emit_ws("new_error", serde_json::to_value(&entry).unwrap_or(Value::Null));
        entry
    }

    pub fn is_motion_busy(&self) -> Option<&'static str> {
        let s = self.state.read().unwrap();
        if s.is_homing {
            return Some("目前正在執行馬達歸零程序，請等待歸零完成");
        }
        if s.pickup_active {
            return Some("目前 14 步衝壓搬運自動循環正在運行中，請先停止循環");
        }
        drop(s);
        if self.macro_running.load(Ordering::Relaxed) {
            return Some("目前積木程式 / G-code 巨集正在執行中，請先停止巨集");
        }
        None
    }

    pub fn add_history(&self, msg: impl Into<String>, log_type: &str) {
        let msg_str = msg.into();
        let entry = HistoryEntry {
            time: Local::now().format("%Y-%m-%d %H:%M:%S").to_string(),
            msg: msg_str.clone(),
            log_type: log_type.to_string(),
        };
        {
            let mut buf = self.history_buffer.lock().unwrap();
            if buf.len() >= 250 {
                buf.pop_front();
            }
            buf.push_back(entry.clone());
        }
        self.history_dirty.store(true, Ordering::Relaxed);
        self.emit_ws("new_history", serde_json::to_value(&entry).unwrap_or(Value::Null));

        if log_type == "error" || log_type == "warn" {
            let lvl = if log_type == "error" { "ERROR" } else { "WARN" };
            self.add_error_log(lvl, None, None, msg_str, None);
        }

        if log_type == "error" {
            self.save_history_to_disk();
        }
    }

    pub fn history_file(&self) -> PathBuf {
        self.base_dir.join("history_log.json")
    }

    pub fn errors_file(&self) -> PathBuf {
        self.base_dir.join("error_log.json")
    }

    pub fn macros_file(&self) -> PathBuf {
        self.base_dir.join("macros.json")
    }

    pub fn limits_file(&self) -> PathBuf {
        self.base_dir.join("limits_config.json")
    }

    pub fn load_history_from_disk(&self) {
        let path = self.history_file();
        if let Ok(content) = fs::read_to_string(path) {
            if let Ok(entries) = serde_json::from_str::<Vec<HistoryEntry>>(&content) {
                let mut buf = self.history_buffer.lock().unwrap();
                buf.clear();
                let skip = entries.len().saturating_sub(250);
                for item in entries.into_iter().skip(skip) {
                    buf.push_back(item);
                }
            }
        }
    }

    pub fn load_errors_from_disk(&self) {
        let path = self.errors_file();
        let mut loaded = false;
        if let Ok(content) = fs::read_to_string(path) {
            if let Ok(entries) = serde_json::from_str::<Vec<ErrorLogEntry>>(&content) {
                let mut buf = self.error_buffer.lock().unwrap();
                buf.clear();
                let mut max_id = 0u64;
                let skip = entries.len().saturating_sub(500);
                for item in entries.into_iter().skip(skip) {
                    if item.id > max_id {
                        max_id = item.id;
                    }
                    buf.push_back(item);
                }
                self.error_seq.store(max_id + 1, Ordering::Relaxed);
                loaded = !buf.is_empty();
            }
        }
        // 若尚無獨立 error_log.json，從既有 history_buffer 匯入 error / warn 紀錄
        if !loaded {
            let hist_items: Vec<HistoryEntry> = {
                let h = self.history_buffer.lock().unwrap();
                h.iter()
                    .filter(|e| e.log_type == "error" || e.log_type == "warn")
                    .cloned()
                    .collect()
            };
            if !hist_items.is_empty() {
                let mut buf = self.error_buffer.lock().unwrap();
                for h in hist_items {
                    let lvl = if h.log_type == "error" { "ERROR" } else { "WARN" };
                    let (src, code) = Self::classify_error_meta(&h.msg, lvl);
                    let id = self.error_seq.fetch_add(1, Ordering::Relaxed);
                    buf.push_back(ErrorLogEntry {
                        id,
                        time: h.time,
                        level: lvl.to_string(),
                        source: src,
                        code,
                        msg: h.msg,
                        detail: "歷史日誌匯入紀錄".to_string(),
                        resolved: false,
                    });
                }
            }
        }
    }

    pub fn save_errors_to_disk(&self) {
        let data: Vec<ErrorLogEntry> = {
            let buf = self.error_buffer.lock().unwrap();
            buf.iter().cloned().collect()
        };
        self.error_dirty.store(false, Ordering::Relaxed);
        if let Ok(json_str) = serde_json::to_string_pretty(&data) {
            let _ = fs::write(self.errors_file(), json_str);
        }
    }

    pub fn clear_errors(&self) {
        {
            let mut buf = self.error_buffer.lock().unwrap();
            buf.clear();
        }
        if let Ok(mut s) = self.state.write() {
            s.error = false;
            s.error_msg.clear();
        }
        self.save_errors_to_disk();
    }

    pub fn resolve_errors(&self, target_id: Option<u64>) {
        {
            let mut buf = self.error_buffer.lock().unwrap();
            for item in buf.iter_mut() {
                if target_id.is_none() || target_id == Some(item.id) {
                    item.resolved = true;
                }
            }
        }
        if target_id.is_none() {
            if let Ok(mut s) = self.state.write() {
                s.error = false;
                s.error_msg.clear();
            }
        }
        self.save_errors_to_disk();
    }

    pub fn save_history_to_disk(&self) {
        let data: Vec<HistoryEntry> = {
            let buf = self.history_buffer.lock().unwrap();
            buf.iter().cloned().collect()
        };
        self.history_dirty.store(false, Ordering::Relaxed);
        if let Ok(json_str) = serde_json::to_string_pretty(&data) {
            let _ = fs::write(self.history_file(), json_str);
        }
    }

    pub fn clear_history(&self) {
        {
            let mut buf = self.history_buffer.lock().unwrap();
            buf.clear();
        }
        self.save_history_to_disk();
    }

    pub fn load_limits_config(&self) {
        let path = self.limits_file();
        if let Ok(content) = fs::read_to_string(path) {
            if let Ok(cfg) = serde_json::from_str::<Value>(&content) {
                let mut s = self.state.write().unwrap();
                if let Some(v) = cfg.get("x_min").and_then(|v| v.as_i64()) { s.x_min = v; }
                if let Some(v) = cfg.get("x_max").and_then(|v| v.as_i64()) { s.x_max = v; }
                if let Some(v) = cfg.get("z_min").and_then(|v| v.as_i64()) { s.z_min = v; s.y_min = v; }
                if let Some(v) = cfg.get("z_max").and_then(|v| v.as_i64()) { s.z_max = v; s.y_max = v; }
                if let Some(v) = cfg.get("x_min_mm").and_then(|v| v.as_f64()) { s.x_min_mm = v; }
                if let Some(v) = cfg.get("x_max_mm").and_then(|v| v.as_f64()) { s.x_max_mm = v; }
                if let Some(v) = cfg.get("z_min_mm").and_then(|v| v.as_f64()) { s.z_min_mm = v; s.y_min_mm = v; }
                if let Some(v) = cfg.get("z_max_mm").and_then(|v| v.as_f64()) { s.z_max_mm = v; s.y_max_mm = v; }
                if let Some(v) = cfg.get("x_scale").and_then(|v| v.as_f64()) { s.x_scale = v.max(0.001); }
                if let Some(v) = cfg.get("z_scale").and_then(|v| v.as_f64()) { s.z_scale = v.max(0.001); s.y_scale = s.z_scale; }
                if let Some(v) = cfg.get("x_max_speed").and_then(|v| v.as_f64()) {
                    s.x_max_speed = v;
                    self.x_stepper.set_speed_params(v, s.x_accel_steps);
                }
                if let Some(v) = cfg.get("x_accel_steps").and_then(|v| v.as_f64()) {
                    s.x_accel_steps = v;
                    self.x_stepper.set_speed_params(s.x_max_speed, v);
                }
                if let Some(v) = cfg.get("z_max_speed").and_then(|v| v.as_f64()) {
                    s.z_max_speed = v;
                    s.y_max_speed = v;
                    self.z_stepper.set_speed_params(v, s.z_accel_steps);
                }
                if let Some(v) = cfg.get("z_accel_steps").and_then(|v| v.as_f64()) {
                    s.z_accel_steps = v;
                    s.y_accel_steps = v;
                    self.z_stepper.set_speed_params(s.z_max_speed, v);
                }
                if let Some(v) = cfg.get("homing_speed_hz").and_then(|v| v.as_f64()) { s.homing_speed_hz = v; }
                if let Some(v) = cfg.get("y_sync").and_then(|v| v.as_bool()) { s.y_sync = v; s.z_sync = v; }
            }
        }
    }

    pub fn save_limits_config(&self) {
        let s = self.state.read().unwrap().clone();
        let cfg = json!({
            "x_min": s.x_min,
            "x_max": s.x_max,
            "z_min": s.z_min,
            "z_max": s.z_max,
            "y_min": s.y_min,
            "y_max": s.y_max,
            "x_min_mm": s.x_min_mm,
            "x_max_mm": s.x_max_mm,
            "z_min_mm": s.z_min_mm,
            "z_max_mm": s.z_max_mm,
            "y_min_mm": s.y_min_mm,
            "y_max_mm": s.y_max_mm,
            "x_scale": s.x_scale,
            "z_scale": s.z_scale,
            "y_scale": s.y_scale,
            "x_max_speed": s.x_max_speed,
            "x_accel_steps": s.x_accel_steps,
            "z_max_speed": s.z_max_speed,
            "z_accel_steps": s.z_accel_steps,
            "y_max_speed": s.y_max_speed,
            "y_accel_steps": s.y_accel_steps,
            "homing_speed_hz": s.homing_speed_hz,
            "y_sync": s.y_sync,
        });
        if let Ok(json_str) = serde_json::to_string_pretty(&cfg) {
            let _ = fs::write(self.limits_file(), json_str);
        }
    }

    pub fn load_macros(&self) -> HashMap<String, Vec<String>> {
        let path = self.macros_file();
        if let Ok(content) = fs::read_to_string(path) {
            if let Ok(map) = serde_json::from_str::<HashMap<String, Vec<String>>>(&content) {
                return map;
            }
        }
        let mut default_map = HashMap::new();
        default_map.insert(
            "自動搬運範例".to_string(),
            vec![
                "# 歸零與準備".into(),
                "GOTO Z_D".into(),
                "GOTO X_A".into(),
                "DELAY 300".into(),
                "GOTO Z_E".into(),
                "MAGNET ALL ON".into(),
                "DELAY 400".into(),
                "GOTO Z_D".into(),
                "CHECK METAL".into(),
                "GOTO X_C".into(),
                "GOTO Z_E".into(),
                "MAGNET ALL OFF".into(),
                "DELAY 300".into(),
                "GOTO Z_D".into(),
                "GOTO X_A".into(),
                "MSG 搬運任務已完成！".into(),
            ],
        );
        default_map
    }

    pub fn save_macros(&self, macros: &HashMap<String, Vec<String>>) {
        if let Ok(json_str) = serde_json::to_string_pretty(macros) {
            let _ = fs::write(self.macros_file(), json_str);
        }
    }

    pub fn set_servo_pulse(&self, axis: &str, pulse_us: f64) {
        let s = self.state.read().unwrap();
        match axis.to_lowercase().as_str() {
            "x" => {
                let clamped = pulse_us.clamp(s.x_min as f64, s.x_max as f64);
                self.x_stepper.set_target(clamped, s.x_min as f64, s.x_max as f64);
            }
            "z" | "y" | "y1" | "y2" => {
                let clamped = pulse_us.clamp(s.z_min as f64, s.z_max as f64);
                self.z_stepper.set_target(clamped, s.z_min as f64, s.z_max as f64);
            }
            _ => {}
        }
    }

    pub fn set_digital_output(&self, name: &str, value: bool) {
        let mut s = self.state.write().unwrap();
        match name {
            "magnet" | "vacuum" | "magnet_all" => {
                s.magnet = value;
                s.vacuum = value;
                s.magnet_1 = value;
                s.magnet_2 = value;
                s.magnet_3 = value;
                s.magnet_4 = value;
                s.magnet_active_count = if value { 4 } else { 0 };
                s.magnet_on_time = if value { Self::now_unix_sec() } else { 0.0 };
                for pin in [24u8, 18, 14, 15] {
                    self.gpio.write_pin(pin, value);
                }
            }
            "magnet_1" | "magnet_2" | "magnet_3" | "magnet_4" => {
                let idx = name.chars().last().unwrap_or('1').to_digit(10).unwrap_or(1) as usize;
                match idx {
                    1 => { s.magnet_1 = value; self.gpio.write_pin(24, value); }
                    2 => { s.magnet_2 = value; self.gpio.write_pin(18, value); }
                    3 => { s.magnet_3 = value; self.gpio.write_pin(14, value); }
                    4 => { s.magnet_4 = value; self.gpio.write_pin(15, value); }
                    _ => {}
                }
                let active_cnt = [s.magnet_1, s.magnet_2, s.magnet_3, s.magnet_4]
                    .iter()
                    .filter(|&&b| b)
                    .count();
                let any_on = active_cnt > 0;
                if any_on && !s.magnet {
                    s.magnet_on_time = Self::now_unix_sec();
                } else if !any_on {
                    s.magnet_on_time = 0.0;
                }
                s.magnet_active_count = active_cnt;
                s.magnet = any_on;
                s.vacuum = any_on;
            }
            "z_down" => {
                s.z_down = value;
                s.arm_down = value;
            }
            _ => {}
        }
    }

    pub fn read_limit_switches(&self) {
        let x_pos = self.x_stepper.current_pos();
        let z_pos = self.z_stepper.current_pos();

        if self.is_raspberry_pi {
            let pins = [
                17u8, 25, 27, 22, 23, // LIM_X_A, LIM_X_B, LIM_X_C, LIM_Z_D, LIM_Z_E
                6, 7, 8, 9, 10, 11, 4, 21, // HALL_1..8
                5, // PRESS_DONE
            ];
            let mut counts = [0u8; 14];
            for sample in 0..3 {
                for (i, &pin) in pins.iter().enumerate() {
                    // NPN 常開與 817 光耦在觸發時拉低為 LOW (false)
                    if !self.gpio.read_pin(pin) {
                        counts[i] += 1;
                    }
                }
                if sample < 2 {
                    std::thread::sleep(std::time::Duration::from_micros(80));
                }
            }
            let mut s = self.state.write().unwrap();
            s.lim_x_a = counts[0] >= 2;
            s.lim_x_b = counts[1] >= 2;
            s.lim_x_c = counts[2] >= 2;
            s.lim_z_d = counts[3] >= 2;
            s.lim_z_e = counts[4] >= 2;
            s.hall_1 = counts[5] >= 2;
            s.hall_2 = counts[6] >= 2;
            s.hall_3 = counts[7] >= 2;
            s.hall_4 = counts[8] >= 2;
            s.hall_5 = counts[9] >= 2;
            s.hall_6 = counts[10] >= 2;
            s.hall_7 = counts[11] >= 2;
            s.hall_8 = counts[12] >= 2;
            s.press_done = counts[13] >= 2;
            Self::sync_derived_sensors(&mut s);
        } else {
            let mut s = self.state.write().unwrap();
            s.lim_x_a = x_pos <= (s.x_min as f64) + 5.0;
            let x_mid = (s.x_min as f64 + s.x_max as f64) / 2.0;
            s.lim_x_b = (x_pos - x_mid).abs() <= 30.0;
            s.lim_x_c = x_pos >= (s.x_max as f64) - 5.0;
            s.lim_z_e = z_pos <= (s.z_min as f64) + 5.0;
            s.lim_z_d = z_pos >= (s.z_max as f64) - 5.0;

            s.hall_1 = s.magnet_1;
            s.hall_2 = s.magnet_1;
            s.hall_3 = s.magnet_2;
            s.hall_4 = s.magnet_2;
            s.hall_5 = s.magnet_3;
            s.hall_6 = s.magnet_3;
            s.hall_7 = s.magnet_4;
            s.hall_8 = s.magnet_4;
            s.press_done = true;
            Self::sync_derived_sensors(&mut s);
        }
    }

    fn sync_derived_sensors(s: &mut SystemState) {
        s.hall_group_1 = s.hall_1 && s.hall_2;
        s.hall_group_2 = s.hall_3 && s.hall_4;
        s.hall_group_3 = s.hall_5 && s.hall_6;
        s.hall_group_4 = s.hall_7 && s.hall_8;
        s.hall_active_count = [
            s.hall_1, s.hall_2, s.hall_3, s.hall_4, s.hall_5, s.hall_6, s.hall_7, s.hall_8,
        ]
        .iter()
        .filter(|&&b| b)
        .count();
        s.hall_group_count = [s.hall_group_1, s.hall_group_2, s.hall_group_3, s.hall_group_4]
            .iter()
            .filter(|&&b| b)
            .count();
        s.hall_arm_metal = s.hall_group_count > 0 || s.hall_active_count > 0;

        s.lim_left = s.lim_x_a;
        s.lim_mid = s.lim_x_b;
        s.lim_right = s.lim_x_c;
        s.lim_up = s.lim_z_d;
        s.lim_down = s.lim_z_e;
        s.hall_metal = s.hall_arm_metal;
    }

    pub fn build_full_status_json(&self, include_history_and_pins: bool) -> Value {
        let s = self.state.read().unwrap().clone();
        let mut val = serde_json::to_value(&s).unwrap_or_else(|_| json!({}));
        if include_history_and_pins {
            if let Some(obj) = val.as_object_mut() {
                let pwm = self.pwm_history.lock().unwrap().clone();
                obj.insert(
                    "pwm_history".to_string(),
                    json!({
                        "x": pwm.x,
                        "y1": pwm.y1,
                        "y2": pwm.y2,
                    }),
                );
                obj.insert(
                    "pin_config".to_string(),
                    serde_json::to_value(&self.pin_config).unwrap_or_else(|_| json!({})),
                );
            }
        }
        val
    }
}

