use serde_json::json;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use crate::gpio::{home_axis, StepperMotor};
use crate::state::AppContext;

fn wait_motor_reach(
    ctx: &Arc<AppContext>,
    motor: &Arc<StepperMotor>,
    target: f64,
    tolerance: f64,
    timeout_sec: f64,
) -> bool {
    let start = Instant::now();
    loop {
        {
            let s = ctx.state.read().unwrap();
            if !s.pickup_active || !s.system_run {
                return false;
            }
        }
        let cur = motor.current_pos();
        if (cur - target).abs() <= tolerance {
            return true;
        }
        if start.elapsed().as_secs_f64() > timeout_sec {
            ctx.add_history(
                format!(
                    "到位警告：馬達到位超時（目標={:.0}µs，當前={:.0}µs）",
                    target, cur
                ),
                "warn",
            );
            return false;
        }
        thread::sleep(Duration::from_millis(40));
    }
}

pub fn spawn_stamping_cycle(ctx: Arc<AppContext>) {
    thread::spawn(move || {
        {
            let mut s = ctx.state.write().unwrap();
            s.pickup_active = true;
            let mode_str = if s.continuous_mode {
                "連續生產模式"
            } else {
                "單次循環模式"
            };
            let target = s.cycle_target;
            drop(s);
            ctx.add_history(
                format!(
                    "🚀 衝壓自動化取放料開始運轉 ({}，目標: {} 次)",
                    mode_str, target
                ),
                "success",
            );
        }

        loop {
            let (sys_run, pickup_act, pos_x_a, pos_x_b, pos_x_c, pos_z_d, pos_z_e, pos_z_half) = {
                let s = ctx.state.read().unwrap();
                let xa = s.x_min as f64;
                let xc = s.x_max as f64;
                let xb = (xa + xc) / 2.0;
                let zd = s.z_max as f64;
                let ze = s.z_min as f64;
                let zhalf = (zd + ze) / 2.0;
                (
                    s.system_run,
                    s.pickup_active,
                    xa,
                    xb,
                    xc,
                    zd,
                    ze,
                    zhalf,
                )
            };
            if !sys_run || !pickup_act {
                break;
            }

            // 步驟 1: X 軸回到 a 點
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "home1", "desc": "1. X軸回到a點(歸零)" }),
            );
            ctx.add_history("步驟 1/14: X 軸前往 a 點 (歸零點)...", "info");
            ctx.set_servo_pulse("x", pos_x_a);
            if !wait_motor_reach(&ctx, &ctx.x_stepper, pos_x_a, 15.0, 12.0) {
                break;
            }
            if ctx.state.read().unwrap().lim_x_a {
                ctx.x_stepper.set_position(pos_x_a);
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 2: Z 軸下去 (到達 e 點)
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "move-mold1", "desc": "2. Z軸下降至e點" }),
            );
            ctx.add_history("步驟 2/14: Z 軸下降至 e 點 (向下極限取料面)...", "info");
            ctx.set_servo_pulse("z", pos_z_e);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_e, 15.0, 12.0) {
                break;
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 3: 圓形電磁鐵通電吸料
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "suck", "desc": "3. 電磁鐵通電吸料" }),
            );
            ctx.add_history("步驟 3/14: 圓形電磁鐵通電，強磁力吸附金屬工件", "info");
            ctx.set_digital_output("magnet", true);
            thread::sleep(Duration::from_millis(400));

            // 步驟 4: Z 軸上去 (到達 d 點)
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "check-mold", "desc": "4. Z軸上升至d點" }),
            );
            ctx.add_history("步驟 4/14: Z 軸上升至 d 點 (安全高度)...", "info");
            ctx.set_servo_pulse("z", pos_z_d);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_d, 15.0, 12.0) {
                break;
            }
            if ctx.state.read().unwrap().lim_z_d {
                ctx.z_stepper.set_position(pos_z_d);
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 5: 末端 8 顆霍爾感測陣列 (4組) 金屬檢測
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "check-suck", "desc": "5. 8顆霍爾(4組)金屬檢測" }),
            );
            ctx.add_history(
                "步驟 5/14: 讀取末端 8 顆霍爾感測陣列 (4 組雙重確認)，驗證金屬是否吸附...",
                "info",
            );
            ctx.read_limit_switches();
            let (has_metal, act_cnt, grp_cnt) = {
                let s = ctx.state.read().unwrap();
                (s.hall_arm_metal, s.hall_active_count, s.hall_group_count)
            };
            if !has_metal {
                ctx.add_history(
                    "❌ 警報：末端 8 顆霍爾感測器 (G1~G4) 皆未檢測到金屬工件！取料失敗，緊急中斷！",
                    "error",
                );
                ctx.set_digital_output("magnet", false);
                {
                    let mut s = ctx.state.write().unwrap();
                    s.error = true;
                    s.error_msg =
                        "末端霍爾感測陣列 (8顆/4組) 未檢測到金屬工件，自動循環中止".to_string();
                    s.pickup_active = false;
                }
                ctx.emit_ws("pickup_step", json!({ "step": "idle" }));
                return;
            }
            ctx.add_history(
                format!(
                    "✔ 霍爾感測確認：金屬工件吸附正常 ({}/8 顆觸發，{}/4 組就緒)！",
                    act_cnt, grp_cnt
                ),
                "success",
            );
            thread::sleep(Duration::from_millis(200));

            // 步驟 6: X 軸到 c 點
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "move-mold2", "desc": "6. X軸前往c點" }),
            );
            ctx.add_history("步驟 6/14: X 軸移動至 c 點 (向右極限放料點)...", "info");
            ctx.set_servo_pulse("x", pos_x_c);
            if !wait_motor_reach(&ctx, &ctx.x_stepper, pos_x_c, 15.0, 12.0) {
                break;
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 7: Z 軸下去 (到達 e 點)
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "drop", "desc": "7. Z軸下降至e點" }),
            );
            ctx.add_history("步驟 7/14: Z 軸下降至 e 點 (放料深度)...", "info");
            ctx.set_servo_pulse("z", pos_z_e);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_e, 15.0, 12.0) {
                break;
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 8: 圓形電磁鐵關電釋放
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "check-drop", "desc": "8. 電磁鐵關電釋放" }),
            );
            ctx.add_history("步驟 8/14: 圓形電磁鐵斷電，工件已釋放", "info");
            ctx.set_digital_output("magnet", false);
            thread::sleep(Duration::from_millis(400));

            // 步驟 9: Z 軸上去 (到達 d 點)
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "home2", "desc": "9. Z軸上升至d點" }),
            );
            ctx.add_history("步驟 9/14: Z 軸上升至 d 點 (安全高度)...", "info");
            ctx.set_servo_pulse("z", pos_z_d);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_d, 15.0, 12.0) {
                break;
            }
            if ctx.state.read().unwrap().lim_z_d {
                ctx.z_stepper.set_position(pos_z_d);
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 10: X 軸到 b 點
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "move-b", "desc": "10. X軸前往b點" }),
            );
            ctx.add_history("步驟 10/14: X 軸移動至 b 點 (中間電感接近開關工位)...", "info");
            ctx.set_servo_pulse("x", pos_x_b);
            if !wait_motor_reach(&ctx, &ctx.x_stepper, pos_x_b, 15.0, 12.0) {
                break;
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 11: Z 軸下去一半
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "half-down", "desc": "11. Z軸下去一半" }),
            );
            ctx.add_history("步驟 11/14: Z 軸下降至 50% 中間避讓高度...", "info");
            ctx.set_servo_pulse("z", pos_z_half);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_half, 15.0, 12.0) {
                break;
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 12: 等模具衝壓完
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "wait-press", "desc": "12. 等待衝壓完畢" }),
            );
            ctx.add_history("步驟 12/14: 等候模具衝壓完畢訊號 (PRESS_DONE)...", "info");
            let t_press = Instant::now();
            loop {
                {
                    let s = ctx.state.read().unwrap();
                    if !s.pickup_active || !s.system_run {
                        break;
                    }
                }
                ctx.read_limit_switches();
                let done = ctx.state.read().unwrap().press_done;
                if done || (!ctx.is_raspberry_pi && t_press.elapsed().as_secs_f64() >= 1.5) {
                    break;
                }
                if t_press.elapsed().as_secs_f64() > 15.0 {
                    ctx.add_history("衝壓等待警告：模具訊號超時，自動繼續", "warn");
                    break;
                }
                thread::sleep(Duration::from_millis(100));
            }
            ctx.add_history("✔ 模具衝壓完成！", "success");
            thread::sleep(Duration::from_millis(200));

            // 步驟 13: Z 軸上去 (到達 d 點)
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "up-again", "desc": "13. Z軸上去d點" }),
            );
            ctx.add_history("步驟 13/14: Z 軸上升至 d 點 (安全高度)...", "info");
            ctx.set_servo_pulse("z", pos_z_d);
            if !wait_motor_reach(&ctx, &ctx.z_stepper, pos_z_d, 15.0, 12.0) {
                break;
            }
            if ctx.state.read().unwrap().lim_z_d {
                ctx.z_stepper.set_position(pos_z_d);
            }
            thread::sleep(Duration::from_millis(200));

            // 步驟 14: X 軸回到 a 點
            if !ctx.state.read().unwrap().system_run {
                break;
            }
            ctx.emit_ws(
                "pickup_step",
                json!({ "step": "cycle-done", "desc": "14. X軸回a點完成" }),
            );
            ctx.add_history("步驟 14/14: X 軸回到 a 點 (原點)...", "info");
            ctx.set_servo_pulse("x", pos_x_a);
            if !wait_motor_reach(&ctx, &ctx.x_stepper, pos_x_a, 15.0, 12.0) {
                break;
            }
            if ctx.state.read().unwrap().lim_x_a {
                ctx.x_stepper.set_position(pos_x_a);
            }

            let (count, target, cont, interval) = {
                let mut s = ctx.state.write().unwrap();
                s.cycle_count += 1;
                (
                    s.cycle_count,
                    s.cycle_target,
                    s.continuous_mode,
                    s.cycle_interval_sec.max(0.1),
                )
            };
            ctx.emit_ws(
                "cycle_update",
                json!({
                    "cycle_count": count,
                    "cycle_target": target,
                    "continuous_mode": cont
                }),
            );
            ctx.add_history(format!("🎉 衝壓循環第 {} 次完成！", count), "success");

            if !cont {
                break;
            }
            if target > 0 && count >= target {
                ctx.add_history(
                    format!(
                        "🏁 已達設定目標生產量 ({} 次)，連續循環自動安全停機！",
                        target
                    ),
                    "success",
                );
                break;
            }

            let wait_start = Instant::now();
            while wait_start.elapsed().as_secs_f64() < interval {
                let s = ctx.state.read().unwrap();
                if !s.pickup_active || !s.system_run {
                    break;
                }
                drop(s);
                thread::sleep(Duration::from_millis(50));
            }
        }

        let final_cnt = ctx.state.read().unwrap().cycle_count;
        ctx.add_history(
            format!("衝壓搬運流程就緒 (累計完成: {} 次)", final_cnt),
            "info",
        );
        ctx.set_digital_output("magnet", false);
        ctx.state.write().unwrap().pickup_active = false;
        ctx.emit_ws("pickup_step", json!({ "step": "idle" }));
    });
}

fn wait_macro_motor_target(ctx: &Arc<AppContext>, axis: &str, target_pulse: f64) {
    let is_x = axis == "x";
    let (min_l, max_l) = {
        let s = ctx.state.read().unwrap();
        if is_x {
            (s.x_min as f64, s.x_max as f64)
        } else {
            (s.z_min as f64, s.z_max as f64)
        }
    };
    let clamped = target_pulse.clamp(min_l, max_l);
    let motor = if is_x { &ctx.x_stepper } else { &ctx.z_stepper };
    let start = Instant::now();

    while ctx.macro_running.load(Ordering::Relaxed) && !ctx.macro_paused.load(Ordering::Relaxed) {
        let cur = motor.current_pos();
        if (cur - clamped).abs() < 10.0 {
            break;
        }
        {
            let s = ctx.state.read().unwrap();
            if is_x {
                if (clamped < cur && s.lim_x_a) || (clamped > cur && s.lim_x_c) {
                    break;
                }
            } else if (clamped < cur && s.lim_z_e) || (clamped > cur && s.lim_z_d) {
                break;
            }
        }
        if start.elapsed().as_secs_f64() > 15.0 {
            ctx.add_history(
                format!("巨集警告: 移動至 {:.0}µs 到位超時", target_pulse),
                "warn",
            );
            break;
        }
        thread::sleep(Duration::from_millis(45));
    }
}

pub fn spawn_macro_runner(ctx: Arc<AppContext>, commands: Vec<String>) {
    thread::spawn(move || {
        ctx.macro_running.store(true, Ordering::Relaxed);
        ctx.macro_paused.store(false, Ordering::Relaxed);
        ctx.state.write().unwrap().system_run = true;

        ctx.add_history("🧩 積木/巨集程式執行開始", "success");
        ctx.emit_ws(
            "macro_status",
            json!({ "running": true, "paused": false, "current_line": 0 }),
        );

        struct LoopFrame {
            start_idx: usize,
            remaining: usize,
            total: usize,
        }
        let mut loop_stack: Vec<LoopFrame> = Vec::new();
        let mut idx = 0usize;

        while idx < commands.len() && ctx.macro_running.load(Ordering::Relaxed) {
            if ctx.macro_paused.load(Ordering::Relaxed) {
                thread::sleep(Duration::from_millis(100));
                continue;
            }

            let cmd = commands[idx].trim().to_string();
            idx += 1;
            ctx.emit_ws(
                "macro_status",
                json!({ "running": true, "paused": false, "current_line": idx }),
            );

            if cmd.is_empty() || cmd.starts_with('#') || cmd.starts_with("//") {
                continue;
            }

            let parts: Vec<&str> = cmd.split_whitespace().collect();
            let op = parts[0].to_uppercase();

            match op.as_str() {
                "REPEAT" => {
                    let cnt = parts
                        .get(1)
                        .and_then(|s| s.parse::<usize>().ok())
                        .unwrap_or(2)
                        .clamp(1, 999);
                    loop_stack.push(LoopFrame {
                        start_idx: idx,
                        remaining: cnt.saturating_sub(1),
                        total: cnt,
                    });
                    ctx.add_history(format!("🔁 積木迴圈開始: 重複執行 {} 次", cnt), "info");
                }
                "END" | "END_REPEAT" => {
                    if let Some(top) = loop_stack.last_mut() {
                        if top.remaining > 0 {
                            top.remaining -= 1;
                            let done_cnt = top.total - top.remaining;
                            let next_idx = top.start_idx;
                            let total = top.total;
                            ctx.add_history(
                                format!("🔁 迴圈第 {}/{} 次開始...", done_cnt, total),
                                "info",
                            );
                            idx = next_idx;
                        } else {
                            loop_stack.pop();
                            ctx.add_history("🔁 積木迴圈結束", "info");
                        }
                    }
                }
                "MOVE" => {
                    if parts.len() >= 3 {
                        let axis = parts[1].to_lowercase();
                        let val_str = parts[2].to_lowercase();
                        if let Some(mm_str) = val_str.strip_suffix("mm") {
                            if let Ok(mm_val) = mm_str.parse::<f64>() {
                                let (scale, base_min) = {
                                    let s = ctx.state.read().unwrap();
                                    if axis == "x" {
                                        (s.x_scale, s.x_min as f64)
                                    } else {
                                        (s.z_scale, s.z_min as f64)
                                    }
                                };
                                let target = base_min + mm_val * scale;
                                ctx.set_servo_pulse(&axis, target);
                                ctx.add_history(
                                    format!(
                                        "巨集指令: 移動 {} 至 {:.1} mm ({:.0} µs)",
                                        axis.to_uppercase(),
                                        mm_val,
                                        target
                                    ),
                                    "info",
                                );
                                wait_macro_motor_target(&ctx, &axis, target);
                            }
                        } else if let Ok(target) = val_str.parse::<f64>() {
                            ctx.set_servo_pulse(&axis, target);
                            ctx.add_history(
                                format!("巨集指令: 移動 {} 至 {:.0} µs", axis.to_uppercase(), target),
                                "info",
                            );
                            wait_macro_motor_target(&ctx, &axis, target);
                        }
                    }
                }
                "MOVE_MM" => {
                    if parts.len() >= 3 {
                        let axis = parts[1].to_lowercase();
                        if let Ok(mm_val) = parts[2].parse::<f64>() {
                            let (scale, base_min) = {
                                let s = ctx.state.read().unwrap();
                                if axis == "x" {
                                    (s.x_scale, s.x_min as f64)
                                } else {
                                    (s.z_scale, s.z_min as f64)
                                }
                            };
                            let target = base_min + mm_val * scale;
                            ctx.set_servo_pulse(&axis, target);
                            ctx.add_history(
                                format!(
                                    "🧩 積木移動: {} 軸前往 {:.1} mm ({:.0} µs)",
                                    axis.to_uppercase(),
                                    mm_val,
                                    target
                                ),
                                "info",
                            );
                            wait_macro_motor_target(&ctx, &axis, target);
                        }
                    }
                }
                "GOTO" => {
                    if parts.len() >= 2 {
                        let pt = parts[1].to_uppercase();
                        let (x_min, x_max, z_min, z_max) = {
                            let s = ctx.state.read().unwrap();
                            (s.x_min as f64, s.x_max as f64, s.z_min as f64, s.z_max as f64)
                        };
                        let resolved = match pt.as_str() {
                            "X_A" => Some(("x", x_min, "X軸 a點 (0mm 歸零原點)")),
                            "X_B" => Some(("x", (x_min + x_max) / 2.0, "X軸 b點 (100mm 中間模具工位)")),
                            "X_C" => Some(("x", x_max, "X軸 c點 (200mm 右極限放料位)")),
                            "Z_D" => Some(("z", z_max, "Z軸 d點 (200mm 向上安全高度)")),
                            "Z_E" => Some(("z", z_min, "Z軸 e點 (0mm 向下取放高度)")),
                            "Z_HALF" => Some(("z", (z_min + z_max) / 2.0, "Z軸 50% (100mm 避讓高度)")),
                            _ => None,
                        };
                        if let Some((axis, target, label)) = resolved {
                            ctx.set_servo_pulse(axis, target);
                            ctx.add_history(format!("🧩 積木定位: 前往【{}】", label), "info");
                            wait_macro_motor_target(&ctx, axis, target);
                        }
                    }
                }
                "STEP" => {
                    if parts.len() >= 3 {
                        let axis = parts[1].to_lowercase();
                        if let Ok(delta_mm) = parts[2].parse::<f64>() {
                            let scale = {
                                let s = ctx.state.read().unwrap();
                                if axis == "x" { s.x_scale } else { s.z_scale }
                            };
                            let cur_pos = if axis == "x" {
                                ctx.x_stepper.current_pos()
                            } else {
                                ctx.z_stepper.current_pos()
                            };
                            let target = cur_pos + delta_mm * scale;
                            ctx.set_servo_pulse(&axis, target);
                            let dir_str = if delta_mm >= 0.0 {
                                format!("+{:.1}", delta_mm)
                            } else {
                                format!("{:.1}", delta_mm)
                            };
                            ctx.add_history(
                                format!("🧩 積木相對移動: {} 軸 {} mm", axis.to_uppercase(), dir_str),
                                "info",
                            );
                            wait_macro_motor_target(&ctx, &axis, target);
                        }
                    }
                }
                "DELAY" => {
                    if parts.len() >= 2 {
                        if let Ok(ms) = parts[1].parse::<f64>() {
                            ctx.add_history(format!("巨集指令: 延遲 {:.0} 毫秒", ms), "info");
                            let mut elapsed = 0.0;
                            let target_sec = ms / 1000.0;
                            while elapsed < target_sec && ctx.macro_running.load(Ordering::Relaxed) {
                                if !ctx.macro_paused.load(Ordering::Relaxed) {
                                    thread::sleep(Duration::from_millis(50));
                                    elapsed += 0.05;
                                } else {
                                    thread::sleep(Duration::from_millis(100));
                                }
                            }
                        }
                    }
                }
                "MAGNET" | "VACUUM" => {
                    if parts.len() >= 3 && matches!(parts[1], "1" | "2" | "3" | "4") {
                        let g = parts[1];
                        let val = parts[2].eq_ignore_ascii_case("ON");
                        ctx.set_digital_output(&format!("magnet_{}", g), val);
                        ctx.add_history(
                            format!(
                                "巨集指令: 第 {} 組電磁鐵 (M{}) {}",
                                g,
                                g,
                                if val { "通電 ON" } else { "斷電 OFF" }
                            ),
                            "info",
                        );
                        thread::sleep(Duration::from_millis(400));
                    } else if parts.len() >= 3 && parts[1].eq_ignore_ascii_case("ALL") {
                        let val = parts[2].eq_ignore_ascii_case("ON");
                        ctx.set_digital_output("magnet", val);
                        ctx.add_history(
                            format!(
                                "巨集指令: 4組電磁鐵吸盤 (M1~M4) {}",
                                if val { "全部通電 ON" } else { "全部斷電 OFF" }
                            ),
                            "info",
                        );
                        thread::sleep(Duration::from_millis(400));
                    } else if parts.len() >= 2 {
                        let val = parts[1].eq_ignore_ascii_case("ON");
                        ctx.set_digital_output("magnet", val);
                        ctx.add_history(
                            format!(
                                "巨集指令: 4組電磁鐵吸盤 (M1~M4) {}",
                                if val { "全部通電 ON" } else { "全部斷電 OFF" }
                            ),
                            "info",
                        );
                        thread::sleep(Duration::from_millis(400));
                    }
                }
                "CHECK" => {
                    if parts.len() >= 2 && parts[1].eq_ignore_ascii_case("METAL") {
                        ctx.read_limit_switches();
                        let (ok, cnt) = {
                            let s = ctx.state.read().unwrap();
                            (s.hall_arm_metal, s.hall_active_count)
                        };
                        if !ok {
                            ctx.add_history(
                                "巨集錯誤: 末端霍爾感測陣列 (8顆/4組) 未檢測到金屬工件！",
                                "error",
                            );
                            ctx.set_digital_output("magnet", false);
                            break;
                        } else {
                            ctx.add_history(
                                format!(
                                    "巨集檢測: 末端霍爾感測金屬工件正常 ({}/8 顆觸發)！",
                                    cnt
                                ),
                                "success",
                            );
                        }
                    } else if parts.len() >= 3 && parts[1].eq_ignore_ascii_case("GROUP") {
                        let g = parts[2];
                        ctx.read_limit_switches();
                        let ok = {
                            let s = ctx.state.read().unwrap();
                            match g {
                                "1" => s.hall_group_1,
                                "2" => s.hall_group_2,
                                "3" => s.hall_group_3,
                                "4" => s.hall_group_4,
                                _ => false,
                            }
                        };
                        if !ok {
                            ctx.add_history(
                                format!("巨集錯誤: 第 {} 組霍爾開關 (G{}) 未檢測到金屬工件！", g, g),
                                "error",
                            );
                            ctx.set_digital_output("magnet", false);
                            break;
                        } else {
                            ctx.add_history(
                                format!("巨集檢測: 第 {} 組霍爾開關 (G{}) 金屬工件確認在位！", g, g),
                                "success",
                            );
                        }
                    }
                }
                "WAIT" => {
                    if parts.len() >= 2 && parts[1].eq_ignore_ascii_case("PRESS") {
                        ctx.add_history("巨集指令: 等待衝壓模具完成訊號...", "info");
                        let start = Instant::now();
                        while ctx.macro_running.load(Ordering::Relaxed)
                            && !ctx.macro_paused.load(Ordering::Relaxed)
                        {
                            ctx.read_limit_switches();
                            let done = ctx.state.read().unwrap().press_done;
                            if done || (!ctx.is_raspberry_pi && start.elapsed().as_secs_f64() >= 1.5)
                            {
                                break;
                            }
                            if start.elapsed().as_secs_f64() > 15.0 {
                                ctx.add_history("巨集警告: 等待衝壓完成超時", "warn");
                                break;
                            }
                            thread::sleep(Duration::from_millis(100));
                        }
                    }
                }
                "ARM" => {
                    if parts.len() >= 2 {
                        let val = parts[1].eq_ignore_ascii_case("DOWN");
                        ctx.set_digital_output("z_down", val);
                        ctx.add_history(
                            format!("巨集指令: 手臂 {}", if val { "下降" } else { "上升" }),
                            "info",
                        );
                        thread::sleep(Duration::from_millis(500));
                    }
                }
                "HOME" => {
                    let axis = parts.get(1).copied().unwrap_or("all");
                    ctx.add_history(format!("巨集指令: 馬達歸零 ({})", axis.to_uppercase()), "info");
                    home_axis(Arc::clone(&ctx), axis);
                }
                "MSG" => {
                    let msg = parts[1..].join(" ");
                    ctx.add_history(format!("巨集提示: {}", msg), "success");
                    ctx.emit_ws("macro_msg", json!({ "msg": msg }));
                }
                _ => {
                    ctx.add_history(format!("巨集錯誤: 未知指令 '{}'", op), "error");
                }
            }
        }

        ctx.macro_running.store(false, Ordering::Relaxed);
        ctx.emit_ws(
            "macro_status",
            json!({ "running": false, "paused": false, "current_line": 0 }),
        );
        ctx.add_history("🧩 積木/巨集程式執行結束", "success");
    });
}
