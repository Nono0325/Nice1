use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    http::{header, HeaderMap, StatusCode},
    response::{Html, IntoResponse, Json},
};
use futures_util::{SinkExt, StreamExt};
use rand::Rng;
use serde_json::{json, Value};
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crate::automation::{spawn_macro_runner, spawn_stamping_cycle};
use crate::gpio::home_axis;
use crate::state::{
    pulse_to_duty, AppContext, HistoryEntry, SessionInfo, PULSE_MIN, PWM_FREQ, SYSTEM_PULSE_MAX,
};

fn no_cache_headers() -> HeaderMap {
    let mut headers = HeaderMap::new();
    headers.insert(
        header::CACHE_CONTROL,
        "no-cache, no-store, must-revalidate".parse().unwrap(),
    );
    headers.insert(header::PRAGMA, "no-cache".parse().unwrap());
    headers.insert(header::EXPIRES, "0".parse().unwrap());
    headers
}

pub async fn index_handler(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    let index_path = ctx.base_dir.join("templates").join("index.html");
    let html = std::fs::read_to_string(index_path)
        .unwrap_or_else(|_| "<h1>Servo Controller UI Missing</h1>".to_string());
    (no_cache_headers(), Html(html))
}

pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(ctx): State<Arc<AppContext>>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, ctx))
}

async fn handle_socket(socket: WebSocket, ctx: Arc<AppContext>) {
    let (mut sender, mut receiver) = socket.split();
    let mut rx = ctx.ws_tx.subscribe();

    // 連線成功後立即推播一次完整狀態
    let init_payload = json!({
        "event": "state_update",
        "data": ctx.build_full_status_json(true)
    });
    if let Ok(txt) = serde_json::to_string(&init_payload) {
        let _ = sender.send(Message::Text(txt)).await;
    }

    let send_task = tokio::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            if sender.send(Message::Text(msg)).await.is_err() {
                break;
            }
        }
    });

    let ctx_recv = Arc::clone(&ctx);
    let recv_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = receiver.next().await {
            if let Message::Text(text) = msg {
                if let Ok(parsed) = serde_json::from_str::<Value>(&text) {
                    let ev = parsed.get("event").and_then(|v| v.as_str()).unwrap_or("");
                    let data = parsed.get("data").cloned().unwrap_or(parsed.clone());
                    if ev == "cmd_move" || parsed.get("axis").is_some() {
                        handle_ws_cmd_move(&ctx_recv, &data);
                    }
                }
            }
        }
    });

    tokio::select! {
        _ = send_task => {},
        _ = recv_task => {},
    }
}

fn handle_ws_cmd_move(ctx: &Arc<AppContext>, data: &Value) {
    let axis = data
        .get("axis")
        .and_then(|v| v.as_str())
        .unwrap_or("x")
        .to_lowercase();
    let delta_mm = data.get("delta_mm").and_then(|v| v.as_f64());
    let delta_us = data.get("delta").and_then(|v| v.as_f64());

    let (sys_run, scale, x_min, x_max, z_min, z_max, lim_xa, lim_xc, lim_ze, lim_zd) = {
        let s = ctx.state.read().unwrap();
        (
            s.system_run,
            if axis == "x" { s.x_scale } else { s.z_scale },
            s.x_min as f64,
            s.x_max as f64,
            s.z_min as f64,
            s.z_max as f64,
            s.lim_x_a || s.lim_left,
            s.lim_x_c || s.lim_right,
            s.lim_z_e || s.lim_down,
            s.lim_z_d || s.lim_up,
        )
    };
    if !sys_run {
        return;
    }

    let delta = if let Some(mm) = delta_mm {
        mm * scale
    } else {
        delta_us.unwrap_or(0.0)
    };

    if axis == "x" {
        if (lim_xa && delta < 0.0) || (lim_xc && delta > 0.0) {
            return;
        }
        ctx.x_stepper.add_target_delta(delta, x_min, x_max);
    } else if matches!(axis.as_str(), "z" | "y" | "y1" | "y2") {
        if (lim_ze && delta < 0.0) || (lim_zd && delta > 0.0) {
            return;
        }
        ctx.z_stepper.add_target_delta(delta, z_min, z_max);
    }
}

pub async fn api_status(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    (no_cache_headers(), Json(ctx.build_full_status_json(true)))
}

pub async fn api_move(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let axis = data
        .get("axis")
        .and_then(|v| v.as_str())
        .unwrap_or("x")
        .to_lowercase();
    let delta_mm = data.get("delta_mm").and_then(|v| v.as_f64());
    let mut delta_us = data.get("delta").and_then(|v| v.as_f64());
    let target_mm = data.get("target_mm").and_then(|v| v.as_f64());
    let mut target_us = data.get("target").and_then(|v| v.as_f64());

    let (sys_run, scale, x_min, x_max, z_min, z_max, lim_xa, lim_xc, lim_ze, lim_zd) = {
        let s = ctx.state.read().unwrap();
        (
            s.system_run,
            if axis == "x" { s.x_scale } else { s.z_scale },
            s.x_min as f64,
            s.x_max as f64,
            s.z_min as f64,
            s.z_max as f64,
            s.lim_x_a || s.lim_left,
            s.lim_x_c || s.lim_right,
            s.lim_z_e || s.lim_down,
            s.lim_z_d || s.lim_up,
        )
    };

    if !sys_run {
        return Json(json!({ "ok": false, "msg": "系統未啟動，請按 START" }));
    }

    if let Some(mm) = delta_mm {
        delta_us = Some(mm * scale);
    }
    if let Some(mm) = target_mm {
        let base_min = if axis == "x" { x_min } else { z_min };
        target_us = Some(base_min + mm * scale);
    }

    if axis == "x" {
        if let Some(d) = delta_us {
            if lim_xa && d < 0.0 {
                return Json(json!({ "ok": false, "msg": "X 軸 a 點左側限位觸發" }));
            }
            if lim_xc && d > 0.0 {
                return Json(json!({ "ok": false, "msg": "X 軸 c 點右側限位觸發" }));
            }
            ctx.x_stepper.add_target_delta(d, x_min, x_max);
        } else if let Some(t) = target_us {
            ctx.x_stepper.set_target(t, x_min, x_max);
        }
    } else if matches!(axis.as_str(), "z" | "y" | "y1" | "y2") {
        if let Some(d) = delta_us {
            if lim_ze && d < 0.0 {
                return Json(json!({ "ok": false, "msg": "Z 軸 e 點下極限觸發" }));
            }
            if lim_zd && d > 0.0 {
                return Json(json!({ "ok": false, "msg": "Z 軸 d 點上歸零觸發" }));
            }
            ctx.z_stepper.add_target_delta(d, z_min, z_max);
        } else if let Some(t) = target_us {
            ctx.z_stepper.set_target(t, z_min, z_max);
        }
    }

    let s = ctx.state.read().unwrap();
    Json(json!({
        "ok": true,
        "x_pulse": s.x_pulse,
        "z_pulse": s.z_pulse,
        "y1_pulse": s.y1_pulse,
        "y2_pulse": s.y2_pulse
    }))
}

pub async fn api_set_position(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let axis = data
        .get("axis")
        .and_then(|v| v.as_str())
        .unwrap_or("x")
        .to_lowercase();
    let pct = data
        .get("pct")
        .and_then(|v| v.as_f64())
        .unwrap_or(50.0)
        .clamp(0.0, 100.0);

    let (sys_run, x_min, x_max, z_min, z_max, x_pulse, z_pulse, lim_xa, lim_xc, lim_zd, lim_ze) = {
        let s = ctx.state.read().unwrap();
        (
            s.system_run,
            s.x_min as f64,
            s.x_max as f64,
            s.z_min as f64,
            s.z_max as f64,
            s.x_pulse as f64,
            s.z_pulse as f64,
            s.lim_x_a || s.lim_left,
            s.lim_x_c || s.lim_right,
            s.lim_z_d || s.lim_up,
            s.lim_z_e || s.lim_down,
        )
    };
    if !sys_run {
        return Json(json!({ "ok": false, "msg": "系統未啟動，請按 START" }));
    }

    if axis == "x" {
        let pulse = x_min + (x_max - x_min) * pct / 100.0;
        let diff = pulse - x_pulse;
        if lim_xa && diff < 0.0 {
            return Json(json!({ "ok": false, "msg": "X 軸左側 a 點限位" }));
        }
        if lim_xc && diff > 0.0 {
            return Json(json!({ "ok": false, "msg": "X 軸右側 c 點限位" }));
        }
        ctx.set_servo_pulse("x", pulse);
    } else if matches!(axis.as_str(), "z" | "y" | "y1" | "y2") {
        let pulse = z_min + (z_max - z_min) * pct / 100.0;
        let diff = pulse - z_pulse;
        if lim_zd && diff > 0.0 {
            return Json(json!({ "ok": false, "msg": "Z 軸上側 d 點限位" }));
        }
        if lim_ze && diff < 0.0 {
            return Json(json!({ "ok": false, "msg": "Z 軸下側 e 點限位" }));
        }
        ctx.set_servo_pulse("z", pulse);
    }

    Json(json!({ "ok": true }))
}

pub async fn api_start(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    {
        let mut s = ctx.state.write().unwrap();
        s.system_run = true;
        s.error = false;
        s.error_msg.clear();
    }
    ctx.add_history("系統啟動", "success");
    Json(json!({ "ok": true }))
}

pub async fn api_stop(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    {
        let mut s = ctx.state.write().unwrap();
        s.system_run = false;
        s.pickup_active = false;
    }
    ctx.macro_running.store(false, Ordering::Relaxed);
    ctx.homing_abort.store(true, Ordering::Relaxed);
    ctx.x_stepper.pause_for_homing.store(false, Ordering::Relaxed);
    ctx.z_stepper.pause_for_homing.store(false, Ordering::Relaxed);
    ctx.x_stepper.halt();
    ctx.z_stepper.halt();
    ctx.set_digital_output("magnet", false);
    ctx.set_digital_output("z_down", false);
    ctx.add_history("系統緊急停止（斷開電磁鐵吸盤＋強制雙軸煞車）", "warn");
    Json(json!({ "ok": true }))
}

pub async fn api_home(
    State(ctx): State<Arc<AppContext>>,
    body: Option<Json<Value>>,
) -> impl IntoResponse {
    if ctx.state.read().unwrap().is_homing {
        return Json(json!({ "ok": false, "msg": "歸零流程執行中，請稍候..." }));
    }
    let axis = body
        .as_ref()
        .and_then(|b| b.get("axis"))
        .and_then(|v| v.as_str())
        .unwrap_or("all")
        .to_lowercase();

    ctx.add_history(format!("手動點擊馬達歸零 ({})", axis.to_uppercase()), "info");
    ctx.state.write().unwrap().is_homing = true;

    let ctx_clone = Arc::clone(&ctx);
    let axis_clone = axis.clone();
    thread::spawn(move || {
        home_axis(ctx_clone, &axis_clone);
    });

    Json(json!({ "ok": true, "msg": format!("歸零中: {}", axis) }))
}

pub async fn api_magnet(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let value = data.get("on").and_then(|v| v.as_bool()).unwrap_or(false);
    let group_opt = data
        .get("group")
        .and_then(|v| v.as_u64().or_else(|| v.as_str()?.parse::<u64>().ok()));

    if let Some(g) = group_opt.filter(|&g| (1..=4).contains(&g)) {
        ctx.set_digital_output(&format!("magnet_{}", g), value);
        ctx.add_history(
            format!(
                "手動切換第 {} 組電磁鐵 (M{}): {}",
                g,
                g,
                if value { "通電吸附 ON" } else { "斷電釋放 OFF" }
            ),
            "info",
        );
    } else {
        ctx.set_digital_output("magnet", value);
        ctx.add_history(
            format!(
                "手動切換全部 4 組電磁鐵 (M1~M4): {}",
                if value { "通電吸附 ON" } else { "斷電釋放 OFF" }
            ),
            "info",
        );
    }

    let s = ctx.state.read().unwrap();
    Json(json!({
        "ok": true,
        "magnet": s.magnet,
        "magnet_1": s.magnet_1,
        "magnet_2": s.magnet_2,
        "magnet_3": s.magnet_3,
        "magnet_4": s.magnet_4,
        "magnet_active_count": s.magnet_active_count,
        "vacuum": s.vacuum
    }))
}

pub async fn api_z_down(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let value = data.get("on").and_then(|v| v.as_bool()).unwrap_or(false);
    ctx.set_digital_output("z_down", value);
    ctx.add_history(
        format!("手動切換手臂: {}", if value { "下降" } else { "上升" }),
        "info",
    );
    let z_down = ctx.state.read().unwrap().z_down;
    Json(json!({ "ok": true, "z_down": z_down }))
}

pub async fn api_pickup(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    let (sys_run, pickup_act, cont) = {
        let s = ctx.state.read().unwrap();
        (s.system_run, s.pickup_active, s.continuous_mode)
    };
    if !sys_run {
        return Json(json!({ "ok": false, "msg": "系統未啟動，請先按下 START PB" }));
    }
    if pickup_act {
        return Json(json!({ "ok": false, "msg": "自動衝壓循環已在執行中" }));
    }

    spawn_stamping_cycle(Arc::clone(&ctx));
    Json(json!({
        "ok": true,
        "msg": "自動衝壓搬運循環啟動",
        "continuous_mode": cont
    }))
}

pub async fn api_continuous_mode(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let (cont, target, interval, count) = {
        let mut s = ctx.state.write().unwrap();
        if let Some(en) = data.get("enabled").and_then(|v| v.as_bool()) {
            s.continuous_mode = en;
        }
        if let Some(t) = data.get("target").and_then(|v| v.as_u64()) {
            s.cycle_target = t;
        }
        if let Some(iv) = data.get("interval").and_then(|v| v.as_f64()) {
            s.cycle_interval_sec = iv.clamp(0.1, 60.0);
        }
        (
            s.continuous_mode,
            s.cycle_target,
            s.cycle_interval_sec,
            s.cycle_count,
        )
    };
    let mode_text = if cont {
        "啟用連續生產模式"
    } else {
        "關閉連續生產模式"
    };
    ctx.add_history(
        format!("{} (目標={}次, 間隔={}s)", mode_text, target, interval),
        "info",
    );
    Json(json!({
        "ok": true,
        "continuous_mode": cont,
        "cycle_target": target,
        "cycle_interval_sec": interval,
        "cycle_count": count
    }))
}

pub async fn api_reset_cycle_count(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    ctx.state.write().unwrap().cycle_count = 0;
    ctx.add_history("生產累計計數器已歸零", "info");
    Json(json!({ "ok": true, "cycle_count": 0 }))
}

fn parse_f64(val: Option<&Value>) -> Option<f64> {
    val.and_then(|v| v.as_f64().or_else(|| v.as_str()?.parse::<f64>().ok()))
}

pub async fn api_set_limits(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    {
        let mut s = ctx.state.write().unwrap();

        if let Some(xs) = parse_f64(data.get("x_scale")) {
            s.x_scale = xs.max(0.001);
        }
        if let Some(zs) = parse_f64(data.get("z_scale").or_else(|| data.get("y_scale"))) {
            let sc = zs.max(0.001);
            s.z_scale = sc;
            s.y_scale = sc;
        }

        // X 軸設定 (支援 mm 與 µs)
        let x_min_mm_in = parse_f64(data.get("x_min_mm"));
        let x_max_mm_in = parse_f64(data.get("x_max_mm"));
        if x_min_mm_in.is_some() || x_max_mm_in.is_some() {
            let min_mm = x_min_mm_in.unwrap_or(s.x_min_mm);
            let mut max_mm = x_max_mm_in.unwrap_or(s.x_max_mm);
            if max_mm <= min_mm {
                max_mm = min_mm + 1.0;
            }
            s.x_min_mm = (min_mm * 10.0).round() / 10.0;
            s.x_max_mm = (max_mm * 10.0).round() / 10.0;
            s.x_min = ((PULSE_MIN as f64 + min_mm * s.x_scale).round() as i64)
                .clamp(0, SYSTEM_PULSE_MAX);
            s.x_max = ((PULSE_MIN as f64 + max_mm * s.x_scale).round() as i64)
                .clamp(s.x_min + 1, SYSTEM_PULSE_MAX);
        } else {
            let mut changed_us = false;
            if let Some(xmin) = parse_f64(data.get("x_min")) {
                s.x_min = (xmin.round() as i64).clamp(0, SYSTEM_PULSE_MAX);
                changed_us = true;
            }
            if let Some(xmax) = parse_f64(data.get("x_max")) {
                s.x_max = (xmax.round() as i64).clamp(s.x_min + 1, SYSTEM_PULSE_MAX);
                changed_us = true;
            }
            if changed_us {
                s.x_min_mm =
                    (((s.x_min - PULSE_MIN).max(0) as f64 / s.x_scale.max(0.001)) * 10.0).round()
                        / 10.0;
                s.x_max_mm = ((s.x_min_mm + (s.x_max - s.x_min) as f64 / s.x_scale.max(0.001))
                    * 10.0)
                    .round()
                    / 10.0;
            }
        }

        let clamped_x = ctx
            .x_stepper
            .current_pos()
            .clamp(s.x_min as f64, s.x_max as f64);
        if (clamped_x - ctx.x_stepper.current_pos()).abs() > 0.5 {
            ctx.x_stepper.set_position(clamped_x);
            ctx.x_stepper.set_target(clamped_x, s.x_min as f64, s.x_max as f64);
        }
        s.x_pulse = clamped_x as i64;

        // Z / Y 軸設定 (支援 mm 與 µs)
        let z_min_mm_in = parse_f64(data.get("z_min_mm").or_else(|| data.get("y_min_mm")));
        let z_max_mm_in = parse_f64(data.get("z_max_mm").or_else(|| data.get("y_max_mm")));
        if z_min_mm_in.is_some() || z_max_mm_in.is_some() {
            let min_mm = z_min_mm_in.unwrap_or(s.z_min_mm);
            let mut max_mm = z_max_mm_in.unwrap_or(s.z_max_mm);
            if max_mm <= min_mm {
                max_mm = min_mm + 1.0;
            }
            let r_min = (min_mm * 10.0).round() / 10.0;
            let r_max = (max_mm * 10.0).round() / 10.0;
            s.z_min_mm = r_min;
            s.y_min_mm = r_min;
            s.z_max_mm = r_max;
            s.y_max_mm = r_max;
            let zmin_us = ((PULSE_MIN as f64 + min_mm * s.z_scale).round() as i64)
                .clamp(0, SYSTEM_PULSE_MAX);
            let zmax_us = ((PULSE_MIN as f64 + max_mm * s.z_scale).round() as i64)
                .clamp(zmin_us + 1, SYSTEM_PULSE_MAX);
            s.z_min = zmin_us;
            s.y_min = zmin_us;
            s.z_max = zmax_us;
            s.y_max = zmax_us;
        } else {
            let mut changed_us = false;
            if let Some(zmin) = parse_f64(data.get("z_min").or_else(|| data.get("y_min"))) {
                let v = (zmin.round() as i64).clamp(0, SYSTEM_PULSE_MAX);
                s.z_min = v;
                s.y_min = v;
                changed_us = true;
            }
            if let Some(zmax) = parse_f64(data.get("z_max").or_else(|| data.get("y_max"))) {
                let v = (zmax.round() as i64).clamp(s.z_min + 1, SYSTEM_PULSE_MAX);
                s.z_max = v;
                s.y_max = v;
                changed_us = true;
            }
            if changed_us {
                let r_min =
                    (((s.z_min - PULSE_MIN).max(0) as f64 / s.z_scale.max(0.001)) * 10.0).round()
                        / 10.0;
                let r_max = ((r_min + (s.z_max - s.z_min) as f64 / s.z_scale.max(0.001)) * 10.0)
                    .round()
                    / 10.0;
                s.z_min_mm = r_min;
                s.y_min_mm = r_min;
                s.z_max_mm = r_max;
                s.y_max_mm = r_max;
            }
        }

        let clamped_z = ctx
            .z_stepper
            .current_pos()
            .clamp(s.z_min as f64, s.z_max as f64);
        if (clamped_z - ctx.z_stepper.current_pos()).abs() > 0.5 {
            ctx.z_stepper.set_position(clamped_z);
            ctx.z_stepper.set_target(clamped_z, s.z_min as f64, s.z_max as f64);
        }
        s.z_pulse = clamped_z as i64;
        s.y1_pulse = s.z_pulse;
        s.y2_pulse = s.z_pulse;

        if let Some(sync) = data
            .get("z_sync")
            .or_else(|| data.get("y_sync"))
            .and_then(|v| v.as_bool())
        {
            s.y_sync = sync;
            s.z_sync = sync;
        }

        if let Some(spd) = parse_f64(data.get("x_max_speed")) {
            s.x_max_speed = spd;
        }
        if let Some(acc) = parse_f64(data.get("x_accel_steps")) {
            s.x_accel_steps = acc;
        }
        ctx.x_stepper.set_speed_params(s.x_max_speed, s.x_accel_steps);

        if let Some(spd) = parse_f64(data.get("z_max_speed").or_else(|| data.get("y_max_speed"))) {
            s.z_max_speed = spd;
            s.y_max_speed = spd;
        }
        if let Some(acc) =
            parse_f64(data.get("z_accel_steps").or_else(|| data.get("y_accel_steps")))
        {
            s.z_accel_steps = acc;
            s.y_accel_steps = acc;
        }
        ctx.z_stepper.set_speed_params(s.z_max_speed, s.z_accel_steps);

        if let Some(hspd) = parse_f64(data.get("homing_speed_hz")) {
            s.homing_speed_hz = hspd.clamp(100.0, 2000.0);
        }

        if data.get("x_scale").is_some() && x_min_mm_in.is_none() && x_max_mm_in.is_none() {
            s.x_max_mm = ((s.x_min_mm + (s.x_max - s.x_min) as f64 / s.x_scale.max(0.001)) * 10.0)
                .round()
                / 10.0;
        }
        if (data.get("y_scale").is_some() || data.get("z_scale").is_some())
            && z_min_mm_in.is_none()
            && z_max_mm_in.is_none()
        {
            let zmax_mm = ((s.z_min_mm + (s.z_max - s.z_min) as f64 / s.z_scale.max(0.001)) * 10.0)
                .round()
                / 10.0;
            s.z_max_mm = zmax_mm;
            s.y_max_mm = zmax_mm;
        }
    }

    ctx.save_limits_config();
    let s = ctx.state.read().unwrap().clone();
    ctx.add_history(
        format!(
            "更新行程範圍與參數: X({}~{}µs, {}~{}mm, 比例:{}), Z/Y({}~{}µs, {}~{}mm, 比例:{})",
            s.x_min, s.x_max, s.x_min_mm, s.x_max_mm, s.x_scale, s.z_min, s.z_max, s.z_min_mm, s.z_max_mm, s.z_scale
        ),
        "success",
    );

    Json(json!({
        "ok": true,
        "limits": {
            "x_min": s.x_min, "x_max": s.x_max,
            "z_min": s.z_min, "z_max": s.z_max,
            "y_min": s.y_min, "y_max": s.y_max,
            "x_min_mm": s.x_min_mm, "x_max_mm": s.x_max_mm,
            "z_min_mm": s.z_min_mm, "z_max_mm": s.z_max_mm,
            "y_min_mm": s.y_min_mm, "y_max_mm": s.y_max_mm,
            "y_sync": s.y_sync,
            "x_max_speed": s.x_max_speed, "x_accel_steps": s.x_accel_steps,
            "z_max_speed": s.z_max_speed, "z_accel_steps": s.z_accel_steps,
            "y_max_speed": s.y_max_speed, "y_accel_steps": s.y_accel_steps,
            "homing_speed_hz": s.homing_speed_hz,
            "x_scale": s.x_scale, "z_scale": s.z_scale, "y_scale": s.y_scale
        }
    }))
}

pub async fn api_set_motor_enable(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let locked = data.get("locked").and_then(|v| v.as_bool()).unwrap_or(true);
    ctx.state.write().unwrap().motor_locked = locked;
    ctx.x_stepper.set_enable(locked);
    ctx.z_stepper.set_enable(locked);
    let msg = if locked {
        "馬達鎖定 (致能通電中)"
    } else {
        "馬達釋放 (斷電手推模式已啟用)"
    };
    ctx.add_history(msg, "warn");
    Json(json!({ "ok": true, "motor_locked": locked }))
}

pub async fn api_get_macros(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    Json(json!({ "ok": true, "macros": ctx.load_macros() }))
}

pub async fn api_save_macro(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let name = data.get("name").and_then(|v| v.as_str()).unwrap_or("");
    let cmds = data.get("commands").and_then(|v| v.as_array());
    if name.is_empty() || cmds.is_none() {
        return Json(json!({ "ok": false, "msg": "無效的參數" }));
    }
    let cmd_vec: Vec<String> = cmds
        .unwrap()
        .iter()
        .filter_map(|v| v.as_str().map(|s| s.to_string()))
        .collect();
    let mut macros = ctx.load_macros();
    macros.insert(name.to_string(), cmd_vec);
    ctx.save_macros(&macros);
    ctx.add_history(format!("儲存巨集腳本: {}", name), "success");
    Json(json!({ "ok": true }))
}

pub async fn api_delete_macro(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let name = data.get("name").and_then(|v| v.as_str()).unwrap_or("");
    let mut macros = ctx.load_macros();
    if macros.remove(name).is_some() {
        ctx.save_macros(&macros);
        ctx.add_history(format!("刪除巨集腳本: {}", name), "warn");
        return Json(json!({ "ok": true }));
    }
    Json(json!({ "ok": false, "msg": "找不到該巨集" }))
}

pub async fn api_macro_run(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let action = data.get("action").and_then(|v| v.as_str()).unwrap_or("");
    match action {
        "start" => {
            if ctx.macro_running.load(Ordering::Relaxed) {
                return Json(json!({ "ok": false, "msg": "巨集或積木程式已在運行中" }));
            }
            let direct = data
                .get("commands")
                .and_then(|v| v.as_array())
                .map(|arr| {
                    arr.iter()
                        .filter_map(|x| x.as_str().map(|s| s.to_string()))
                        .collect::<Vec<String>>()
                });
            let commands = if let Some(cmds) = direct.filter(|c| !c.is_empty()) {
                cmds
            } else {
                let name = data.get("name").and_then(|v| v.as_str()).unwrap_or("");
                ctx.load_macros().get(name).cloned().unwrap_or_default()
            };
            if commands.is_empty() {
                return Json(json!({ "ok": false, "msg": "找不到該巨集或積木指令為空" }));
            }
            spawn_macro_runner(Arc::clone(&ctx), commands);
            Json(json!({ "ok": true }))
        }
        "stop" => {
            ctx.macro_running.store(false, Ordering::Relaxed);
            ctx.macro_paused.store(false, Ordering::Relaxed);
            Json(json!({ "ok": true }))
        }
        "pause" => {
            ctx.macro_paused.store(true, Ordering::Relaxed);
            ctx.emit_ws("macro_status", json!({ "running": true, "paused": true }));
            Json(json!({ "ok": true }))
        }
        "resume" => {
            ctx.macro_paused.store(false, Ordering::Relaxed);
            ctx.emit_ws("macro_status", json!({ "running": true, "paused": false }));
            Json(json!({ "ok": true }))
        }
        _ => Json(json!({ "ok": false, "msg": "無效的指令" })),
    }
}

pub async fn api_get_history(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    let list: Vec<HistoryEntry> = ctx.history_buffer.lock().unwrap().iter().cloned().collect();
    Json(json!({ "ok": true, "history": list }))
}

pub async fn api_delete_history(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    ctx.clear_history();
    Json(json!({ "ok": true }))
}

pub async fn api_diagnose(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    let s = ctx.state.read().unwrap().clone();
    let diag = json!({
        "gpio_mode": s.gpio_mode,
        "is_raspberry_pi": ctx.is_raspberry_pi,
        "pwm_freq": PWM_FREQ,
        "motors": {
            "X": { "pin": 12, "pulse": s.x_pulse, "duty": pulse_to_duty(s.x_pulse) },
            "Z": { "pin": 13, "pulse": s.z_pulse, "duty": pulse_to_duty(s.z_pulse) },
            "Y1": { "pin": 13, "pulse": s.y1_pulse, "duty": pulse_to_duty(s.y1_pulse) },
            "Y2": { "pin": 13, "pulse": s.y2_pulse, "duty": pulse_to_duty(s.y2_pulse) },
        },
        "digital_outputs": {
            "MAGNET": {
                "pin": 24,
                "state": s.magnet,
                "active_count": s.magnet_active_count,
                "model": "LSD-P25/20 (4組)",
                "relay": "TONGLING JQC-3FF-S-Z 5VDC (光耦隔離繼電器 × 4組)",
                "desc": "4 組圓形電磁鐵 LSD-P25/20 + JQC-3FF-S-Z 繼電器"
            },
            "MAGNET_1": { "pin": 24, "state": s.magnet_1, "name": "第1組電磁鐵 M1 (對應霍爾G1)" },
            "MAGNET_2": { "pin": 18, "state": s.magnet_2, "name": "第2組電磁鐵 M2 (對應霍爾G2)" },
            "MAGNET_3": { "pin": 14, "state": s.magnet_3, "name": "第3組電磁鐵 M3 (對應霍爾G3)" },
            "MAGNET_4": { "pin": 15, "state": s.magnet_4, "name": "第4組電磁鐵 M4 (對應霍爾G4)" },
            "VACUUM": { "pin": 24, "state": s.vacuum },
            "Z_DOWN": { "pin": 25, "state": s.z_down },
        },
        "inductive_sensors": {
            "sensor_model": "QN-1805-E1 方形 DC10-30V 三線 NPN 常開 (檢測距離 5mm)",
            "X_A": { "pin": 17, "name": "X軸a點(左歸零)", "triggered": s.lim_x_a },
            "X_B": { "pin": 25, "name": "X軸b點(中間工位)", "triggered": s.lim_x_b },
            "X_C": { "pin": 27, "name": "X軸c點(向右極限)", "triggered": s.lim_x_c },
            "Z_D": { "pin": 22, "name": "Z軸d點(向上歸零)", "triggered": s.lim_z_d },
            "Z_E": { "pin": 23, "name": "Z軸e點(向下極限)", "triggered": s.lim_z_e },
        },
        "hall_sensors": {
            "optocoupler": "8路 817 光耦電壓隔離板 (PC817 電壓控制轉接模組)",
            "layout": "8 顆霍爾開關，兩個一組 (共 4 組)",
            "active_count": s.hall_active_count,
            "group_count": s.hall_group_count,
            "groups": {
                "G1": { "name": "第1組 (H1+H2)", "pins": [6, 7], "h1": s.hall_1, "h2": s.hall_2, "triggered": s.hall_group_1 },
                "G2": { "name": "第2組 (H3+H4)", "pins": [8, 9], "h3": s.hall_3, "h4": s.hall_4, "triggered": s.hall_group_2 },
                "G3": { "name": "第3組 (H5+H6)", "pins": [10, 11], "h5": s.hall_5, "h6": s.hall_6, "triggered": s.hall_group_3 },
                "G4": { "name": "第4組 (H7+H8)", "pins": [4, 21], "h7": s.hall_7, "h8": s.hall_8, "triggered": s.hall_group_4 },
            },
            "ARM_METAL": { "pin": 6, "name": "手臂金屬檢測總狀態 (8顆/4組)", "triggered": s.hall_arm_metal },
        },
        "limit_switches": {
            "LEFT": { "pin": 17, "triggered": s.lim_left },
            "MID": { "pin": 25, "triggered": s.lim_mid },
            "RIGHT": { "pin": 27, "triggered": s.lim_right },
            "UP": { "pin": 22, "triggered": s.lim_up },
            "DOWN": { "pin": 23, "triggered": s.lim_down },
        },
        "pwm_signal_info": {
            "frequency_hz": PWM_FREQ,
            "period_ms": 20.0,
            "x_pulse_us": s.x_pulse,
            "z_pulse_us": s.z_pulse,
            "y1_pulse_us": s.y1_pulse,
            "y2_pulse_us": s.y2_pulse,
            "x_duty_pct": pulse_to_duty(s.x_pulse),
            "z_duty_pct": pulse_to_duty(s.z_pulse),
            "y1_duty_pct": pulse_to_duty(s.y1_pulse),
            "y2_duty_pct": pulse_to_duty(s.y2_pulse),
        }
    });
    Json(diag)
}

// ── 認證與終端機管理 ────────────────────────────────────────────────────────

fn verify_password(ctx: &Arc<AppContext>, password: &str) -> bool {
    let pwd = password.trim();
    if pwd.is_empty() {
        return false;
    }
    let sim_pwd = ctx.sim_password.lock().unwrap().clone();
    if pwd == sim_pwd {
        return true;
    }
    #[cfg(unix)]
    {
        if let Ok(mut child) = Command::new("sudo")
            .args(["-k", "-S", "-v"])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
        {
            if let Some(mut stdin) = child.stdin.take() {
                let _ = writeln!(stdin, "{}", pwd);
            }
            if let Ok(status) = child.wait() {
                return status.success();
            }
        }
    }
    false
}

fn generate_token() -> String {
    let mut rng = rand::thread_rng();
    (0..64)
        .map(|_| format!("{:x}", rng.gen_range(0..16u8)))
        .collect()
}

fn create_session(ctx: &Arc<AppContext>, password: &str) -> String {
    let token = generate_token();
    let timeout_sec = (ctx.state.read().unwrap().lock_timeout_min * 60) as f64;
    let info = SessionInfo {
        password: password.trim().to_string(),
        expires: AppContext::now_unix_sec() + timeout_sec,
        cwd: ctx.base_dir.clone(),
    };
    ctx.sessions.lock().unwrap().insert(token.clone(), info);
    token
}

fn check_and_refresh_token(ctx: &Arc<AppContext>, token: &str) -> bool {
    if token.is_empty() {
        return false;
    }
    let now = AppContext::now_unix_sec();
    let timeout_sec = (ctx.state.read().unwrap().lock_timeout_min * 60) as f64;
    let mut map = ctx.sessions.lock().unwrap();
    map.retain(|_, v| v.expires > now);
    if let Some(sess) = map.get_mut(token) {
        sess.expires = now + timeout_sec;
        return true;
    }
    false
}

pub async fn api_auth_login(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let pwd = data.get("password").and_then(|v| v.as_str()).unwrap_or("");
    if verify_password(&ctx, pwd) {
        let token = create_session(&ctx, pwd);
        ctx.add_history("全站登入成功 (Rust Backend)", "success");
        Json(json!({ "ok": true, "token": token, "msg": "登入成功" }))
    } else {
        ctx.add_history("全站登入失敗（密碼錯誤）", "warn");
        Json(json!({ "ok": false, "msg": "密碼錯誤，拒絕登入" }))
    }
}

pub async fn api_auth_status(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    let valid = check_and_refresh_token(&ctx, token);
    Json(json!({ "ok": true, "valid": valid }))
}

pub async fn api_auth_logout(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    ctx.sessions.lock().unwrap().remove(token);
    ctx.add_history("使用者登出系統", "info");
    Json(json!({ "ok": true }))
}

pub async fn api_settings_security(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    if let Some(mins) = data.get("lock_timeout_min").and_then(|v| v.as_u64()) {
        ctx.state.write().unwrap().lock_timeout_min = mins.clamp(1, 480);
    }
    if let Some(new_pwd) = data.get("new_password").and_then(|v| v.as_str()) {
        let trimmed = new_pwd.trim();
        if trimmed.len() >= 4 {
            *ctx.sim_password.lock().unwrap() = trimmed.to_string();
            ctx.add_history("登入密碼已更新", "success");
        }
    }
    let mins = ctx.state.read().unwrap().lock_timeout_min;
    ctx.add_history(format!("安全設定已更新：自動鎖定 {} 分鐘", mins), "success");
    Json(json!({
        "ok": true,
        "lock_timeout_min": mins,
        "is_raspberry_pi": ctx.is_raspberry_pi
    }))
}

pub async fn api_terminal_auth(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let pwd = data.get("password").and_then(|v| v.as_str()).unwrap_or("");
    if verify_password(&ctx, pwd) {
        let token = create_session(&ctx, pwd);
        ctx.add_history("Web 終端機通過身份驗證解鎖成功", "success");
        Json(json!({ "ok": true, "token": token, "msg": "驗證成功" }))
    } else {
        ctx.add_history("Web 終端機身份驗證失敗 (密碼錯誤)", "warn");
        Json(json!({ "ok": false, "msg": "密碼錯誤，拒絕存取" }))
    }
}

pub async fn api_terminal_logout(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    ctx.sessions.lock().unwrap().remove(token);
    Json(json!({ "ok": true }))
}

pub async fn api_terminal_check_token(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    let valid = check_and_refresh_token(&ctx, token);
    Json(json!({ "ok": valid, "valid": valid }))
}

pub async fn api_terminal_exec(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    let cmd = data.get("cmd").and_then(|v| v.as_str()).unwrap_or("").trim();

    if !check_and_refresh_token(&ctx, token) {
        return Json(json!({
            "ok": false,
            "auth_required": true,
            "output": "🔒 權限不足：請先輸入密碼驗證解鎖終端機"
        }));
    }
    if cmd.is_empty() {
        return Json(json!({ "ok": false, "output": "空指令" }));
    }

    let (user_pwd, cwd) = {
        let map = ctx.sessions.lock().unwrap();
        let s = map.get(token).unwrap();
        (s.password.clone(), s.cwd.clone())
    };

    ctx.add_history(format!("終端機執行: {}", cmd), "info");

    if cmd == "cd" || cmd.starts_with("cd ") {
        let target_str = if cmd == "cd" {
            std::env::var("HOME")
                .or_else(|_| std::env::var("USERPROFILE"))
                .unwrap_or_else(|_| ".".to_string())
        } else {
            cmd[3..].trim().to_string()
        };
        let target_path = if PathBuf::from(&target_str).is_absolute() {
            PathBuf::from(&target_str)
        } else {
            cwd.join(&target_str)
        };
        if let Ok(canon) = target_path.canonicalize() {
            if canon.is_dir() {
                if let Some(sess) = ctx.sessions.lock().unwrap().get_mut(token) {
                    sess.cwd = canon.clone();
                }
                let disp = canon.to_string_lossy().to_string();
                return Json(json!({
                    "ok": true,
                    "returncode": 0,
                    "output": format!("[已切換目錄 → {}]", disp),
                    "cwd": disp
                }));
            }
        }
        return Json(json!({
            "ok": false,
            "returncode": 1,
            "output": format!("cd: {}: 找不到目錄", target_str),
            "cwd": cwd.to_string_lossy()
        }));
    }

    let is_sudo = cmd.starts_with("sudo ") && cfg!(unix);
    let output_res = if is_sudo {
        let inner = cmd[5..].trim();
        let child_res = Command::new("sh")
            .arg("-c")
            .arg(format!("sudo -k -S {}", inner))
            .current_dir(&cwd)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn();
        match child_res {
            Ok(mut child) => {
                *ctx.active_term_pid.lock().unwrap() = Some(child.id());
                if let Some(mut stdin) = child.stdin.take() {
                    let _ = writeln!(stdin, "{}", user_pwd);
                }
                let res = child.wait_with_output();
                *ctx.active_term_pid.lock().unwrap() = None;
                res
            }
            Err(e) => Err(e),
        }
    } else {
        #[cfg(windows)]
        let mut command = {
            let mut c = Command::new("powershell");
            c.args(["-NoProfile", "-Command", cmd]);
            c
        };
        #[cfg(not(windows))]
        let mut command = {
            let mut c = Command::new("sh");
            c.args(["-c", cmd]);
            c
        };
        command
            .current_dir(&cwd)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        match command.spawn() {
            Ok(child) => {
                *ctx.active_term_pid.lock().unwrap() = Some(child.id());
                let res = child.wait_with_output();
                *ctx.active_term_pid.lock().unwrap() = None;
                res
            }
            Err(e) => Err(e),
        }
    };

    match output_res {
        Ok(out) => {
            let stdout = String::from_utf8_lossy(&out.stdout).to_string();
            let stderr = String::from_utf8_lossy(&out.stderr)
                .lines()
                .filter(|l| {
                    let lower = l.to_lowercase();
                    !lower.starts_with("[sudo]") && !lower.contains("password for")
                })
                .collect::<Vec<&str>>()
                .join("\n");
            let mut combined = stdout;
            if !stderr.trim().is_empty() {
                combined.push_str("\n[STDERR]\n");
                combined.push_str(&stderr);
            }
            if combined.trim().is_empty() {
                combined = "(指令執行完成，無輸出內容)".to_string();
            }
            let code = out.status.code().unwrap_or(0);
            Json(json!({
                "ok": out.status.success(),
                "returncode": code,
                "output": combined,
                "cwd": cwd.to_string_lossy()
            }))
        }
        Err(e) => Json(json!({
            "ok": false,
            "output": format!("❌ 執行例外: {}", e)
        })),
    }
}

pub async fn api_terminal_kill(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    if !check_and_refresh_token(&ctx, token) {
        return Json(json!({ "ok": false, "msg": "未授權" }));
    }
    if let Some(pid) = *ctx.active_term_pid.lock().unwrap() {
        #[cfg(unix)]
        let _ = Command::new("kill").args(["-9", &pid.to_string()]).status();
        #[cfg(windows)]
        let _ = Command::new("taskkill")
            .args(["/F", "/PID", &pid.to_string()])
            .status();
        ctx.add_history("使用者終止終端機執行中指令", "warn");
        return Json(json!({ "ok": true, "msg": "已成功終止指令進程" }));
    }
    Json(json!({ "ok": true, "msg": "目前無執行中的指令進程" }))
}

pub async fn api_system_power(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let action = data.get("action").and_then(|v| v.as_str()).unwrap_or("");
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    let pwd = data.get("password").and_then(|v| v.as_str()).unwrap_or("");

    let authed = check_and_refresh_token(&ctx, token) || verify_password(&ctx, pwd);
    if !authed {
        return Json(json!({
            "ok": false,
            "auth_required": true,
            "msg": "權限不足：密碼驗證失敗"
        }));
    }

    let user_pwd = ctx
        .sessions
        .lock()
        .unwrap()
        .get(token)
        .map(|s| s.password.clone())
        .unwrap_or_else(|| pwd.to_string());

    if action == "shutdown" {
        ctx.add_history("觸發樹莓派系統關機 (Shutdown)", "warn");
        #[cfg(unix)]
        {
            thread::spawn(move || {
                thread::sleep(Duration::from_secs(1));
                if let Ok(mut child) = Command::new("sudo")
                    .args(["-S", "shutdown", "-h", "now"])
                    .stdin(Stdio::piped())
                    .spawn()
                {
                    if let Some(mut stdin) = child.stdin.take() {
                        let _ = writeln!(stdin, "{}", user_pwd);
                    }
                }
            });
            return Json(json!({ "ok": true, "msg": "樹莓派正在關機中..." }));
        }
        #[cfg(not(unix))]
        {
            let _ = user_pwd;
            return Json(json!({ "ok": true, "msg": "模擬模式：已觸發虛擬關機" }));
        }
    } else if action == "reboot" {
        ctx.add_history("觸發樹莓派系統重啟 (Reboot)", "warn");
        #[cfg(unix)]
        {
            thread::spawn(move || {
                thread::sleep(Duration::from_secs(1));
                if let Ok(mut child) = Command::new("sudo")
                    .args(["-S", "reboot"])
                    .stdin(Stdio::piped())
                    .spawn()
                {
                    if let Some(mut stdin) = child.stdin.take() {
                        let _ = writeln!(stdin, "{}", user_pwd);
                    }
                }
            });
            return Json(json!({ "ok": true, "msg": "樹莓派正在重新啟動中..." }));
        }
        #[cfg(not(unix))]
        {
            let _ = user_pwd;
            return Json(json!({ "ok": true, "msg": "模擬模式：已觸發虛擬重啟" }));
        }
    }

    Json(json!({ "ok": false, "msg": "無效的電源指令" }))
}

fn find_git_repo_dir(base_dir: &PathBuf) -> PathBuf {
    let mut cur = base_dir.clone();
    for _ in 0..4 {
        if cur.join(".git").is_dir() {
            return cur;
        }
        if let Some(parent) = cur.parent() {
            cur = parent.to_path_buf();
        } else {
            break;
        }
    }
    base_dir.clone()
}

pub async fn api_update_check(State(ctx): State<Arc<AppContext>>) -> impl IntoResponse {
    let repo_dir = find_git_repo_dir(&ctx.base_dir);
    let _ = Command::new("git")
        .args(["fetch", "origin", "main", "--quiet"])
        .current_dir(&repo_dir)
        .output();

    let local_sha = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .current_dir(&repo_dir)
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".to_string());

    let remote_info = Command::new("git")
        .args([
            "log",
            "-1",
            "--format=%h|%s|%an|%cs",
            "origin/main",
        ])
        .current_dir(&repo_dir)
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default();

    let parts: Vec<&str> = remote_info.splitn(4, '|').collect();
    let remote_sha = parts.first().copied().filter(|s| !s.is_empty()).unwrap_or(&local_sha);
    let remote_msg = parts.get(1).copied().unwrap_or("Rust Architecture Release");
    let remote_author = parts.get(2).copied().unwrap_or("Nono0325");
    let remote_date = parts.get(3).copied().unwrap_or("-");

    let behind_count: u32 = Command::new("git")
        .args(["rev-list", "--count", "HEAD..origin/main"])
        .current_dir(&repo_dir)
        .output()
        .ok()
        .and_then(|o| String::from_utf8_lossy(&o.stdout).trim().parse::<u32>().ok())
        .unwrap_or(0);

    let changelog: Vec<String> = Command::new("git")
        .args(["log", "--oneline", "-n", "5", "HEAD..origin/main"])
        .current_dir(&repo_dir)
        .output()
        .ok()
        .map(|o| {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .map(|l| l.trim().to_string())
                .filter(|l| !l.is_empty())
                .collect()
        })
        .unwrap_or_default();

    let up_to_date = !local_sha.is_empty()
        && (remote_sha.starts_with(&local_sha) || local_sha.starts_with(remote_sha))
        && behind_count == 0;

    Json(json!({
        "ok": true,
        "local_sha": local_sha,
        "remote_sha": remote_sha,
        "remote_msg": remote_msg,
        "remote_author": remote_author,
        "remote_date": remote_date,
        "behind_count": behind_count,
        "changelog": changelog,
        "up_to_date": up_to_date,
        "repo": "Nono0325/Nice1",
        "branch": "main"
    }))
}

pub async fn api_update_pull(
    State(ctx): State<Arc<AppContext>>,
    Json(data): Json<Value>,
) -> impl IntoResponse {
    let token = data.get("token").and_then(|v| v.as_str()).unwrap_or("");
    let restart = data.get("restart").and_then(|v| v.as_bool()).unwrap_or(true);
    let _ = check_and_refresh_token(&ctx, token);

    let repo_dir = find_git_repo_dir(&ctx.base_dir);
    ctx.add_history("系統自動更新：開始執行 git fetch + reset --hard 與編譯檢查", "warn");

    let fetch = Command::new("git")
        .args(["fetch", "origin", "main"])
        .current_dir(&repo_dir)
        .output();
    let pull = Command::new("git")
        .args(["reset", "--hard", "origin/main"])
        .current_dir(&repo_dir)
        .output();

    if let (Ok(f_out), Ok(p_out)) = (fetch, pull) {
        let output = format!(
            "{}\n{}",
            String::from_utf8_lossy(&f_out.stdout),
            String::from_utf8_lossy(&p_out.stdout)
        );
        let new_sha = Command::new("git")
            .args(["rev-parse", "--short", "HEAD"])
            .current_dir(&repo_dir)
            .output()
            .ok()
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
            .unwrap_or_else(|| "latest".to_string());

        // 若 Cargo.toml 存在則自動執行 cargo build --release
        let crate_dir = ctx.base_dir.clone();
        let _ = Command::new("cargo")
            .args(["build", "--release"])
            .current_dir(&crate_dir)
            .output();

        ctx.add_history(format!("系統更新成功：{}", new_sha), "success");

        #[cfg(unix)]
        if restart {
            let user_pwd = ctx
                .sessions
                .lock()
                .unwrap()
                .get(token)
                .map(|s| s.password.clone())
                .unwrap_or_default();
            thread::spawn(move || {
                thread::sleep(Duration::from_millis(1500));
                if let Ok(mut child) = Command::new("sudo")
                    .args(["-S", "systemctl", "restart", "servo-control"])
                    .stdin(Stdio::piped())
                    .spawn()
                {
                    if let Some(mut stdin) = child.stdin.take() {
                        let _ = writeln!(stdin, "{}", user_pwd);
                    }
                }
            });
            return (
                StatusCode::OK,
                Json(json!({
                    "ok": true,
                    "sha": new_sha,
                    "output": output,
                    "msg": format!("更新完成！正在重啟 Rust 服務... ({})", new_sha)
                })),
            );
        }

        let _ = restart;
        return (
            StatusCode::OK,
            Json(json!({
                "ok": true,
                "sha": new_sha,
                "output": output,
                "msg": format!("更新完成！({}) 請手動重啟服務生效。", new_sha)
            })),
        );
    }

    (
        StatusCode::OK,
        Json(json!({ "ok": false, "msg": "git pull 執行失敗" })),
    )
}
