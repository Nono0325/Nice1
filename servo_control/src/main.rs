#![allow(dead_code)]

mod api;
mod automation;
mod gpio;
mod state;

use axum::{
    routing::{delete, get, post},
    Router,
};
use chrono::Local;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;
use tower_http::cors::CorsLayer;
use tower_http::services::ServeDir;

use crate::state::{AppContext, MAX_HISTORY};

fn resolve_base_dir() -> PathBuf {
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    if cwd.join("templates").is_dir() && cwd.join("static").is_dir() {
        return cwd;
    }
    if cwd.join("servo_control").join("templates").is_dir() {
        return cwd.join("servo_control");
    }
    if let Ok(exe) = std::env::current_exe() {
        let mut cur = exe.parent().map(|p| p.to_path_buf());
        while let Some(dir) = cur {
            if dir.join("templates").is_dir() && dir.join("static").is_dir() {
                return dir;
            }
            cur = dir.parent().map(|p| p.to_path_buf());
        }
    }
    cwd
}

fn spawn_broadcast_loop(ctx: Arc<AppContext>) {
    tokio::spawn(async move {
        let mut sim_t: f64 = 0.0;
        let mut tick: u64 = 0;

        loop {
            let x_pos = ctx.x_stepper.current_pos();
            let z_pos = ctx.z_stepper.current_pos();

            ctx.read_limit_switches();

            let mut magnet_overheat = false;
            let (x_pulse, z_pulse, is_pi, sys_run) = {
                let mut s = ctx.state.write().unwrap();

                let x_min_mm = s.x_min_mm;
                let x_max_mm = s.x_max_mm.max(x_min_mm + 1.0);
                let z_min_mm = s.z_min_mm;
                let z_max_mm = s.z_max_mm.max(z_min_mm + 1.0);

                let x_clamped_mm = x_pos.clamp(x_min_mm, x_max_mm);
                let z_clamped_mm = z_pos.clamp(z_min_mm, z_max_mm);

                s.x_mm = (x_clamped_mm * 10.0).round() / 10.0;
                s.x_pct = (((x_clamped_mm - x_min_mm) / (x_max_mm - x_min_mm)) * 100.0).clamp(0.0, 100.0);
                s.x_pulse = (x_clamped_mm * s.x_steps_per_mm).round() as i64;

                s.z_mm = (z_clamped_mm * 10.0).round() / 10.0;
                s.z_pct = (((z_clamped_mm - z_min_mm) / (z_max_mm - z_min_mm)) * 100.0).clamp(0.0, 100.0);
                s.z_pulse = (z_clamped_mm * s.z_steps_per_mm).round() as i64;

                // 若原點霍爾/限位處於觸發狀態或已完成歸零，強制定錨於 0 原點坐標並同步重設馬達坐標
                if s.lim_x_a || s.lim_left {
                    s.x_pulse = 0;
                    s.x_mm = 0.0;
                    s.x_pct = 0.0;
                    s.x_homed = true;
                    if x_pos > 0.05 && !s.is_homing {
                        ctx.x_stepper.set_position(0.0);
                    }
                }
                if s.lim_z_d || s.lim_up {
                    s.z_pulse = 0;
                    s.z_mm = 0.0;
                    s.z_pct = 0.0;
                    s.z_homed = true;
                    if z_pos > 0.05 && !s.is_homing {
                        ctx.z_stepper.set_position(0.0);
                    }
                }

                s.y1_pulse = s.z_pulse;
                s.y2_pulse = s.z_pulse;
                s.y_pct = s.z_pct;
                s.y_mm = s.z_mm;

                s.timestamp = Local::now().format("%H:%M:%S%.3f").to_string();
                s.uptime_sec = ctx.start_instant.elapsed().as_secs();

                if s.magnet && s.magnet_on_time > 0.0 {
                    if AppContext::now_unix_sec() - s.magnet_on_time > 45.0 {
                        magnet_overheat = true;
                    }
                }

                (s.x_pulse as f64, s.z_pulse as f64, ctx.is_raspberry_pi, s.system_run)
            };

            if magnet_overheat {
                ctx.set_digital_output("magnet", false);
                {
                    let mut s = ctx.state.write().unwrap();
                    s.error = true;
                    s.error_msg = "電磁鐵連續通電超過 45 秒，強制斷電保護".to_string();
                }
                ctx.add_history(
                    "⚠️ 安全保護：電磁鐵連續吸附超過 45 秒，強制斷電防線圈過熱！",
                    "warn",
                );
            }

            let noise = if !is_pi && sys_run {
                (sim_t * 0.3).sin() * 3.0
            } else {
                0.0
            };
            sim_t += 0.1;

            {
                let mut pwm = ctx.pwm_history.lock().unwrap();
                pwm.x.push(((x_pulse + noise) * 10.0).round() / 10.0);
                pwm.y1.push(((z_pulse + noise * 0.8) * 10.0).round() / 10.0);
                pwm.y2.push(((z_pulse + noise * 0.6) * 10.0).round() / 10.0);
                if pwm.x.len() > MAX_HISTORY {
                    pwm.x.remove(0);
                    pwm.y1.remove(0);
                    pwm.y2.remove(0);
                }
            }

            let include_extra = tick % 3 == 0;
            let payload = ctx.build_full_status_json(include_extra);
            ctx.emit_ws("state_update", payload);

            tick = tick.wrapping_add(1);
            if tick % 1800 == 0 {
                if ctx.history_dirty.load(Ordering::Relaxed) {
                    ctx.save_history_to_disk();
                }
                if ctx.error_dirty.load(Ordering::Relaxed) {
                    ctx.save_errors_to_disk();
                }
            }

            tokio::time::sleep(Duration::from_millis(160)).await;
        }
    });
}

fn spawn_hardware_watchdog(ctx: Arc<AppContext>) {
    std::thread::spawn(move || {
        #[cfg(target_os = "linux")]
        {
            use std::io::Write;
            if let Ok(mut wd) = std::fs::OpenOptions::new().write(true).open("/dev/watchdog") {
                ctx.state.write().unwrap().watchdog_active = true;
                println!("[WATCHDOG] Raspberry Pi BCM2835 Hardware Watchdog activated (/dev/watchdog)");
                loop {
                    let _ = wd.write_all(b"1");
                    let _ = wd.flush();
                    std::thread::sleep(Duration::from_secs(3));
                }
            }
        }
        let _ = ctx;
    });
}

#[tokio::main]
async fn main() {
    let base_dir = resolve_base_dir();
    let ctx = AppContext::new(base_dir.clone());

    println!("============================================================");
    println!("  🦀 樹莓派步進馬達與電磁鐵控制系統 v{} (Rust 2.0 高效能架構)", crate::state::APP_VERSION);
    println!("  Raspberry Pi Stepper Motor Controller v{} — Powered by Rust", crate::state::APP_VERSION);
    println!("  版本: v{}", crate::state::APP_VERSION);
    println!("  模式: {}", ctx.state.read().unwrap().gpio_mode);
    println!("  資源目錄: {}", base_dir.display());
    println!("============================================================");

    spawn_hardware_watchdog(Arc::clone(&ctx));
    spawn_broadcast_loop(Arc::clone(&ctx));

    let static_dir = base_dir.join("static");

    let app = Router::new()
        .route("/", get(api::index_handler))
        .route("/ws", get(api::ws_handler))
        .route("/api/status", get(api::api_status))
        .route("/api/move", post(api::api_move))
        .route("/api/set_position", post(api::api_set_position))
        .route("/api/start", post(api::api_start))
        .route("/api/stop", post(api::api_stop))
        .route("/api/home", post(api::api_home))
        .route("/api/magnet", post(api::api_magnet))
        .route("/api/vacuum", post(api::api_magnet))
        .route("/api/z_down", post(api::api_z_down))
        .route("/api/stamping_cycle", post(api::api_pickup))
        .route("/api/pickup", post(api::api_pickup))
        .route("/api/continuous_mode", post(api::api_continuous_mode))
        .route("/api/reset_cycle_count", post(api::api_reset_cycle_count))
        .route("/api/set_limits", post(api::api_set_limits))
        .route("/api/settings", post(api::api_set_limits))
        .route("/api/set_motor_enable", post(api::api_set_motor_enable))
        .route(
            "/api/macros",
            get(api::api_get_macros)
                .post(api::api_save_macro)
                .delete(api::api_delete_macro),
        )
        .route("/api/macro/run", post(api::api_macro_run))
        .route(
            "/api/history",
            get(api::api_get_history).delete(api::api_delete_history),
        )
        .route(
            "/api/errors",
            get(api::api_get_errors)
                .post(api::api_post_errors)
                .delete(api::api_delete_errors),
        )
        .route("/api/errors/resolve", post(api::api_resolve_errors))
        .route("/api/diagnose", get(api::api_diagnose))
        .route("/api/auth/login", post(api::api_auth_login))
        .route("/api/auth/status", post(api::api_auth_status))
        .route("/api/auth/logout", post(api::api_auth_logout))
        .route("/api/settings/security", post(api::api_settings_security))
        .route("/api/terminal/auth", post(api::api_terminal_auth))
        .route("/api/terminal/logout", post(api::api_terminal_logout))
        .route(
            "/api/terminal/check_token",
            post(api::api_terminal_check_token),
        )
        .route("/api/terminal/exec", post(api::api_terminal_exec))
        .route("/api/terminal/kill", post(api::api_terminal_kill))
        .route("/api/system_power", post(api::api_system_power))
        .route(
            "/api/update/check",
            get(api::api_update_check).post(api::api_update_check),
        )
        .route("/api/update/pull", post(api::api_update_pull))
        .nest_service("/static", ServeDir::new(static_dir))
        .layer(CorsLayer::permissive())
        .with_state(ctx);

    let port = std::env::var("PORT")
        .ok()
        .and_then(|s| s.parse::<u16>().ok())
        .unwrap_or(5000);
    let addr = format!("0.0.0.0:{}", port);
    println!("[WEB] Rust Axum Server listening at http://{}", addr);

    let listener = tokio::net::TcpListener::bind(&addr)
        .await
        .expect("Failed to bind port 5000");
    axum::serve(listener, app)
        .await
        .expect("Server execution error");
}
