/**
 * 樹梅派伺服馬達控制系統 - 前端 JavaScript
 * Raspberry Pi Servo Motor Controller - Frontend App
 */

// ── WebSocket 連線 ──────────────────────────────────────────────
const socket = io({ transports: ['websocket', 'polling'] });

// ── 全域狀態 ──────────────────────────────────────────────────
let state = {
  x_pulse: 1500, y1_pulse: 1500, y2_pulse: 1500,
  x_pct: 50, y_pct: 50,
  vacuum: false, z_down: false,
  system_run: false, arm_down: false,
  pickup_active: false, error: false, error_msg: '',
  lim_left: false, lim_right: false, lim_up: false, lim_down: false,
  x_min: 500, x_max: 2500, y_min: 500, y_max: 2500,
  y_sync: true,
  x_scale: 10.0, y_scale: 10.0,
  x_max_speed: 2000.0, x_accel_steps: 100.0,
  y_max_speed: 2000.0, y_accel_steps: 100.0,
  motor_locked: true,
  timestamp: '',
  gpio_mode: '-'
};

let stepSize  = 50;       // 步進值 µs
let isTestRunning = false;
let testInterval = null;
let settingsDirty = false;  // 設定頁輸入臟標記（防止 WebSocket 覆蓋）

// ── 全站 Session ─────────────────────────────────────────────────
let siteToken = localStorage.getItem('siteToken') || '';
let siteAuthTimer = null;  // 自動鎖定計時器

// ── Chart.js 圖表 ────────────────────────────────────────────
let chartX = null, chartY = null;

function initCharts() {
  const labels = Array.from({ length: 60 }, (_, i) => '');
  const chartConfig = (labels, datasets) => ({
    type: 'line',
    data: { labels, datasets },
    options: {
      responsive: true,
      animation: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { display: false },
        y: {
          min: 400, max: 2600,
          grid: { color: 'rgba(42,58,85,0.4)', drawBorder: false },
          ticks: {
            color: '#475569', font: { family: "'JetBrains Mono'" , size: 10 },
            callback: v => v + 'µs'
          }
        }
      },
      elements: { point: { radius: 0 }, line: { tension: 0.3, borderWidth: 2 } }
    }
  });

  const ctxX = document.getElementById('chartX')?.getContext('2d');
  if (ctxX) {
    chartX = new Chart(ctxX, chartConfig(labels, [{
      label: 'X Motor', data: new Array(60).fill(1500),
      borderColor: '#3b82f6', backgroundColor: 'rgba(59,130,246,0.06)',
      fill: true
    }]));
  }

  const ctxY = document.getElementById('chartY')?.getContext('2d');
  if (ctxY) {
    chartY = new Chart(ctxY, chartConfig(labels, [
      { label: 'Y1 Motor', data: new Array(60).fill(1500), borderColor: '#10b981', backgroundColor: 'rgba(16,185,129,0.04)', fill: false },
      { label: 'Y2 Motor', data: new Array(60).fill(1500), borderColor: '#8b5cf6', backgroundColor: 'rgba(139,92,246,0.04)', fill: false }
    ]));
  }
}

// ── 量規繪製 ─────────────────────────────────────────────────
function drawGauge(canvasId, value, min, max, color) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const r  = cx - 12;
  const startAngle = Math.PI * 0.75;
  const endAngle   = Math.PI * 2.25;
  const pct = (value - min) / (max - min);
  const valAngle = startAngle + pct * (endAngle - startAngle);

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 背景弧
  ctx.beginPath();
  ctx.arc(cx, cy, r, startAngle, endAngle);
  ctx.strokeStyle = '#1a2235';
  ctx.lineWidth = 12;
  ctx.lineCap = 'round';
  ctx.stroke();

  // 刻度弧
  const grad = ctx.createLinearGradient(0, 0, canvas.width, 0);
  grad.addColorStop(0, '#ef4444');
  grad.addColorStop(0.5, color);
  grad.addColorStop(1, '#10b981');
  ctx.beginPath();
  ctx.arc(cx, cy, r, startAngle, valAngle);
  ctx.strokeStyle = grad;
  ctx.lineWidth = 12;
  ctx.lineCap = 'round';
  ctx.stroke();

  // 刻度標記
  for (let i = 0; i <= 10; i++) {
    const a = startAngle + (i / 10) * (endAngle - startAngle);
    const inner = r - 18, outer = r - 6;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
    ctx.lineTo(cx + Math.cos(a) * outer, cy + Math.sin(a) * outer);
    ctx.strokeStyle = i % 5 === 0 ? 'rgba(148,163,184,0.6)' : 'rgba(71,85,105,0.4)';
    ctx.lineWidth = i % 5 === 0 ? 2 : 1;
    ctx.stroke();
  }

  // 指針
  const needleLen = r - 22;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.cos(valAngle) * needleLen, cy + Math.sin(valAngle) * needleLen);
  ctx.strokeStyle = '#f8fafc';
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.stroke();

  // 中心圓
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();

  // 數值文字
  ctx.font = "bold 13px 'JetBrains Mono'";
  ctx.fillStyle = 'rgba(148,163,184,0.6)';
  ctx.textAlign = 'center';
  ctx.fillText(`${Math.round(pct * 100)}%`, cx, cy + r * 0.55);
}

// ── WebSocket 事件 ───────────────────────────────────────────
socket.on('connect', () => {
  console.log('[WS] Connected');
  setWsStatus(true);
  addLog('WebSocket 已連線', 'success');
  updateDiagWsTag(true);
});

socket.on('disconnect', () => {
  setWsStatus(false);
  addLog('WebSocket 已斷線', 'error');
  updateDiagWsTag(false);
});

socket.on('state_update', (data) => {
  state = { ...state, ...data };
  updateUI();
  if (typeof update3D === 'function') {
    update3D(state);
  }
});

socket.on('pickup_step', (data) => {
  const card = document.getElementById('flowchartCard');
  if (!card) return;
  
  if (data.step === 'idle') {
    // 流程結束，延遲隱藏流程圖
    setTimeout(() => {
      if (!state.pickup_active) {
        card.style.display = 'none';
      }
    }, 4000);
    return;
  }
  
  // 顯示流程圖
  card.style.display = 'block';
  
  const steps = ['init', 'home1', 'check-mold', 'move-mold1', 'suck', 'check-suck', 'move-mold2', 'drop', 'check-drop', 'home2'];
  const activeIdx = steps.indexOf(data.step);
  
  steps.forEach((step, idx) => {
    const el = document.getElementById(`step-${step}`);
    if (!el) return;
    
    if (idx < activeIdx) {
      el.className = 'flow-step completed';
    } else if (idx === activeIdx) {
      el.className = 'flow-step active';
    } else {
      el.className = 'flow-step';
    }
  });
});

function setWsStatus(online) {
  const dot  = document.getElementById('wsStatusDot');
  const text = document.getElementById('wsStatusText');
  if (dot)  dot.className = 'status-dot' + (online ? ' online' : '');
  if (text) text.textContent = online ? '已連線' : '已斷線';
}

function updateDiagWsTag(online) {
  const el = document.getElementById('ws-status-tag');
  if (!el) return;
  el.className = `tag ${online ? 'tag-green' : 'tag-red'}`;
  el.textContent = online ? '✔ 已連線' : '✘ 已斷線';
}

// ── UI 更新 ─────────────────────────────────────────────────
function updateUI() {
  const s = state;

  // 時鐘
  document.getElementById('systemTimestamp').textContent = s.timestamp || '--:--:--';
  document.getElementById('gpioModeText').textContent = s.gpio_mode || '-';

  // START / STOP 按鈕
  const btnStart = document.getElementById('btnStart');
  const btnStop  = document.getElementById('btnStop');
  const sysText  = document.getElementById('systemStatusText');
  if (s.system_run) {
    btnStart?.classList.add('active');
    btnStop?.classList.remove('active');
    if (sysText) { sysText.textContent = '▶ RUNNING'; sysText.style.color = 'var(--accent-green)'; }
  } else {
    btnStart?.classList.remove('active');
    btnStop?.classList.add('active');
    if (sysText) { sysText.textContent = '■ STOPPED'; sysText.style.color = 'var(--accent-red)'; }
  }

  // 指示燈
  setLamp('lamp-pickup',  s.pickup_active ? 'amber' : 'off');
  setLamp('lamp-armdown', s.arm_down      ? 'amber' : 'off');
  setLamp('lamp-vacuum',  s.vacuum        ? 'amber' : 'off');
  setLamp('lamp-limleft',  s.lim_left  ? 'red' : 'off');
  setLamp('lamp-limright', s.lim_right ? 'red' : 'off');
  setLamp('lamp-error',    s.error     ? 'red' : 'off');

  // 錯誤橫幅
  const errBanner = document.getElementById('errorBanner');
  if (errBanner) {
    if (s.error) {
      errBanner.classList.add('visible');
      document.getElementById('errorMsg').textContent = s.error_msg || '系統錯誤';
    } else {
      errBanner.classList.remove('visible');
    }
  }

  // X 軸位置條
  const xPct = Math.max(0, Math.min(100, s.x_pct || 0));
  setEl('xBar', el => el.style.width = `${xPct}%`);
  setEl('xPctDisplay', el => el.textContent = `${xPct.toFixed(1)}%`);
  setEl('xPulseDisplay', el => el.textContent = `${s.x_pulse} µs`);
  const xSlider = document.getElementById('xSlider');
  if (xSlider && document.activeElement !== xSlider) xSlider.value = xPct;

  // 真實物理距離 (mm)
  const xScale = s.x_scale || 10.0;
  const yScale = s.y_scale || 10.0;
  const xMm = (s.x_pulse - 500) / xScale;
  setEl('xMmDisplay', el => el.textContent = `${xMm.toFixed(1)} mm`);

  // Y 軸位置條
  const yPct = Math.max(0, Math.min(100, s.y_pct || 0));
  setEl('yBar', el => el.style.width = `${yPct}%`);
  setEl('yPctDisplay', el => el.textContent = `${yPct.toFixed(1)}%`);
  setEl('y1PulseDisplay', el => el.textContent = `${s.y1_pulse} µs`);
  setEl('y2PulseDisplay', el => el.textContent = `${s.y2_pulse} µs`);
  const yMm = (s.y1_pulse - 500) / yScale;
  setEl('yMmDisplay', el => el.textContent = `${yMm.toFixed(1)} mm`);
  
  const ySlider = document.getElementById('ySlider');
  if (ySlider && document.activeElement !== ySlider) ySlider.value = yPct;

  // 進階設定輸入值綁定
  setElValue('xMaxSpeedInput', s.x_max_speed || 2000);
  setElValue('xAccelStepsInput', s.x_accel_steps || 100);
  setElValue('xScaleInput', s.x_scale || 10);
  setElValue('yMaxSpeedInput', s.y_max_speed || 2000);
  setElValue('yAccelStepsInput', s.y_accel_steps || 100);
  setElValue('yScaleInput', s.y_scale || 10);

  // 馬達鎖定按鈕狀態
  const lockBtn = document.getElementById('btnMotorLock');
  if (lockBtn) lockBtn.className = 'btn-amber-toggle' + (s.motor_locked ? ' on' : '');

  // Arm Down / Vacuum 切換按鈕
  const zBtn = document.getElementById('btnZDown');
  if (zBtn) zBtn.className = `btn-amber-toggle${s.z_down ? ' on' : ''}`;
  const vacBtn = document.getElementById('btnVacuum');
  if (vacBtn) vacBtn.className = `btn-amber-toggle${s.vacuum ? ' on' : ''}`;

  // 監控 Tab - 量規
  drawGauge('gaugeX',  s.x_pulse,  s.x_min,  s.x_max,  '#3b82f6');
  drawGauge('gaugeY1', s.y1_pulse, s.y_min,  s.y_max,  '#10b981');
  drawGauge('gaugeY2', s.y2_pulse, s.y_min,  s.y_max,  '#8b5cf6');
  setEl('gaugeXVal',  el => el.textContent = `${s.x_pulse} µs`);
  setEl('gaugeY1Val', el => el.textContent = `${s.y1_pulse} µs`);
  setEl('gaugeY2Val', el => el.textContent = `${s.y2_pulse} µs`);

  // 監控 Tab - 數位 I/O
  updateTag('mon-vacuum', s.vacuum, '🟢 ON', '⚫ OFF', 'tag-green', 'tag-red');
  updateTag('mon-zdown',  s.z_down, '🟢 ON', '⚫ OFF', 'tag-green', 'tag-red');
  updateTag('mon-limleft',  s.lim_left,  '🔴 觸發', '✔ 正常', 'tag-red', 'tag-green');
  updateTag('mon-limright', s.lim_right, '🔴 觸發', '✔ 正常', 'tag-red', 'tag-green');
  updateTag('mon-limup',    s.lim_up,    '🔴 觸發', '✔ 正常', 'tag-red', 'tag-green');
  updateTag('mon-limdown',  s.lim_down,  '🔴 觸發', '✔ 正常', 'tag-red', 'tag-green');

  // 訊號檢測 Tab
  const duty = (µs) => ((µs / 20000) * 100).toFixed(3);
  setElText('sig-x-pulse', s.x_pulse);
  setElText('sig-y1-pulse', s.y1_pulse);
  setElText('sig-y2-pulse', s.y2_pulse);
  setElText('sig-x-duty',  duty(s.x_pulse));
  setElText('sig-y1-duty', duty(s.y1_pulse));
  setElText('sig-y2-duty', duty(s.y2_pulse));
  setElText('sig-x-pct',  `${xPct.toFixed(1)}%`);
  setElText('sig-y1-pct', `${yPct.toFixed(1)}%`);
  setElText('sig-y2-pct', `${yPct.toFixed(1)}%`);

  // 診斷 Tab
  setElText('diag-x-pulse', s.x_pulse);
  setElText('diag-y1-pulse', s.y1_pulse);
  setElText('diag-y2-pulse', s.y2_pulse);
  setElText('diag-x-duty', duty(s.x_pulse));
  setElText('diag-y1-duty', duty(s.y1_pulse));
  setElText('diag-y2-duty', duty(s.y2_pulse));
  setElText('diag-x-pos',  `${xPct.toFixed(1)}%`);
  setElText('diag-y1-pos', `${yPct.toFixed(1)}%`);
  setElText('diag-y2-pos', `${yPct.toFixed(1)}%`);

  const gpioTag = document.getElementById('diag-gpio-mode');
  if (gpioTag) {
    gpioTag.className = `tag ${s.gpio_mode.includes('Real') ? 'tag-green' : 'tag-amber'}`;
    gpioTag.textContent = s.gpio_mode;
  }

  // 範圍設定顯示
  setElText('curXMin', s.x_min);
  setElText('curXMax', s.x_max);
  setElText('curXRange', `${s.x_max - s.x_min} µs`);
  setElText('curYMin', s.y_min);
  setElText('curYMax', s.y_max);
  setElText('curYRange', `${s.y_max - s.y_min} µs`);

  // GPIO 模式橫幅（診斷頁）
  const modeTitle = document.getElementById('diagModeTitle');
  const modeDesc  = document.getElementById('diagModeDesc');
  if (modeTitle && s.gpio_mode) {
    if (s.gpio_mode.includes('Real')) {
      modeTitle.textContent = '🟢 Raspberry Pi Real GPIO Mode';
      modeTitle.style.color = 'var(--accent-green)';
      if (modeDesc) modeDesc.textContent = '運行在樹梅派硬體上 - 實際 GPIO 輸出已啟用';
    } else {
      modeTitle.textContent = '🟡 Simulation Mode (PC)';
      modeTitle.style.color = 'var(--accent-amber)';
      if (modeDesc) modeDesc.textContent = '非樹梅派環境 - 使用模擬 GPIO，部署到 Pi 後自動切換為實際模式';
    }
  }

  // Chart.js 更新
  if (s.pwm_history && chartX && chartY) {
    chartX.data.datasets[0].data = [...s.pwm_history.x];
    chartX.update('none');
    chartY.data.datasets[0].data = [...s.pwm_history.y1];
    chartY.data.datasets[1].data = [...s.pwm_history.y2];
    chartY.update('none');
  }

  // 位置 tag 更新
  updateMotorTag('tagX',  s.x_pulse,  s.x_min,  s.x_max);
  updateMotorTag('tagY1', s.y1_pulse, s.y_min,  s.y_max);
  updateMotorTag('tagY2', s.y2_pulse, s.y_min,  s.y_max);

  // 歸零中：禁用所有歸零按鈕防止重覆觸發
  const homing = !!s.is_homing;
  ['btnHomeX', 'btnHomeY', 'btnHomeAll'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.disabled = homing;
      el.style.opacity = homing ? '0.5' : '';
    }
  });
  if (homing) {
    // 小型歸零狀態指示
    const indicator = document.getElementById('homingIndicator');
    if (indicator) indicator.style.display = 'inline';
  } else {
    const indicator = document.getElementById('homingIndicator');
    if (indicator) indicator.style.display = 'none';
  }
}

function updateMotorTag(id, pulse, min, max) {
  const el = document.getElementById(id);
  if (!el) return;
  const pct = (pulse - min) / (max - min) * 100;
  if (pct < 5)       { el.className = 'tag tag-red';    el.textContent = 'MIN'; }
  else if (pct > 95) { el.className = 'tag tag-green';  el.textContent = 'MAX'; }
  else               { el.className = 'tag tag-blue';   el.textContent = 'RUNNING'; }
}

// ── 指示燈 ────────────────────────────────────────────────────
function setLamp(id, mode) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = `indicator-lamp ${mode}`;
}

// ── 工具函數 ─────────────────────────────────────────────────
function setEl(id, fn) { const el = document.getElementById(id); if (el) fn(el); }
function setElText(id, val) { const el = document.getElementById(id); if (el) el.textContent = val; }
function updateTag(id, active, activeText, inactiveText, activeClass, inactiveClass) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = `tag ${active ? activeClass : inactiveClass}`;
  el.textContent = active ? activeText : inactiveText;
}

// ── Tab 切換 ─────────────────────────────────────────────────
function switchTab(name) {
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
  document.getElementById(`tab-content-${name}`)?.classList.add('active');
  document.getElementById(`tab-${name}`)?.classList.add('active');

  if (name === 'diagnose') loadDiagnose();

  if (name === 'macros') {
    loadMacros();
  }

  if (name === 'gcode') {
    drawGcodePath();
  }

  if (name === 'simulation3d') {
    init3D();
    setTimeout(() => {
      if (typeof onWindowResize === 'function') onWindowResize();
      if (typeof update3D === 'function') update3D(state);
    }, 80);
  }

  if (name === 'terminal') {
    updateTerminalUI(!!terminalAuthToken);
  }
}

// ── 系統控制 ─────────────────────────────────────────────────
async function cmdStart() {
  const r = await apiPost('/api/start');
  if (r.ok) toast('系統已啟動', 'success');
  addLog('系統啟動', 'success');
}

async function cmdStop() {
  const r = await apiPost('/api/stop');
  if (r.ok) toast('系統已停止', 'info');
  addLog('系統停止', 'warn');
}

async function cmdHome(axis) {
  // 如果歸零已在執行中，拒絕重覆觸發
  if (state.is_homing) {
    toast('歸零執行中，請稍候...', 'warn');
    return;
  }
  const r = await apiPost('/api/home', { axis });
  if (r.ok) {
    toast(`${axis.toUpperCase()} 軸歸零中...`, 'info');
    addLog(`歸零: ${axis}`, 'info');
  } else if (r.msg) {
    toast(r.msg, 'warn');
  }
}

async function cmdPickup() {
  if (!state.system_run) { toast('請先啟動系統', 'error'); return; }
  const r = await apiPost('/api/pickup');
  if (r.ok) { toast('自動取件流程啟動', 'success'); addLog('自動取件流程啟動', 'success'); }
}

// ── 馬達移動 ─────────────────────────────────────────────────
function moveAxis(axis, dir) {
  if (!state.system_run) { toast('系統未啟動', 'error'); return; }
  const delta = dir * stepSize;
  socket.emit('cmd_move', { axis, delta });
}

function onXSlider(val) {
  if (!state.system_run) return;
  apiPost('/api/set_position', { axis: 'x', pct: parseFloat(val) });
}

function onYSlider(val) {
  if (!state.system_run) return;
  apiPost('/api/set_position', { axis: 'y', pct: parseFloat(val) });
}

// ── 步進值 ───────────────────────────────────────────────────
function setStep(size) {
  stepSize = size;
  document.querySelectorAll('#stepChipsX .step-chip').forEach(el => {
    el.classList.toggle('active', parseInt(el.textContent) === size);
  });
  const ci = document.getElementById('stepCustomInput');
  if (ci && document.activeElement !== ci) ci.value = size;
}

function setStepCustom(size) {
  if (!size || size < 1) return;
  stepSize = size;
  document.querySelectorAll('#stepChipsX .step-chip').forEach(el => el.classList.remove('active'));
}


// ── 數位輸出 ─────────────────────────────────────────────────
async function toggleVacuum() {
  const r = await apiPost('/api/vacuum', { on: !state.vacuum });
  if (r.ok) {
    const status = r.vacuum ? '吸盤 ON' : '吸盤 OFF';
    toast(status, r.vacuum ? 'success' : 'info');
    addLog(status, r.vacuum ? 'success' : 'info');
  }
}

async function toggleZDown() {
  const r = await apiPost('/api/z_down', { on: !state.z_down });
  if (r.ok) {
    const status = r.z_down ? '手臂下降' : '手臂上升';
    toast(status, r.z_down ? 'warn' : 'info');
    addLog(status, r.z_down ? 'warn' : 'info');
  }
}

function setYSync(val) {
  apiPost('/api/set_limits', { y_sync: val });
  document.getElementById('ySyncLabel').textContent = val ? 'ON' : 'OFF';
  document.getElementById('ySyncLabel').style.color = val ? 'var(--accent-green)' : 'var(--accent-red)';
  toast(`Y 軸同步: ${val ? 'ON' : 'OFF'}`, 'info');
}

// ── 範圍設定 ─────────────────────────────────────────────────
async function applyXLimits() {
  const xMin = parseInt(document.getElementById('xMinInput').value);
  const xMax = parseInt(document.getElementById('xMaxInput').value);
  if (xMin >= xMax) { toast('最小值必須小於最大值', 'error'); return; }
  const r = await apiPost('/api/set_limits', { x_min: xMin, x_max: xMax });
  if (r.ok) { toast(`X 軸範圍設定: ${xMin}~${xMax}µs`, 'success'); addLog(`X軸範圍: ${xMin}~${xMax}µs`, 'success'); }
}

async function applyYLimits() {
  const yMin = parseInt(document.getElementById('yMinInput').value);
  const yMax = parseInt(document.getElementById('yMaxInput').value);
  const sync = document.getElementById('ySyncSetting').checked;
  if (yMin >= yMax) { toast('最小值必須小於最大值', 'error'); return; }
  const r = await apiPost('/api/set_limits', { y_min: yMin, y_max: yMax, y_sync: sync });
  if (r.ok) { toast(`Y 軸範圍設定: ${yMin}~${yMax}µs`, 'success'); addLog(`Y軸範圍: ${yMin}~${yMax}µs`, 'success'); }
}

function resetXLimits() {
  document.getElementById('xMinInput').value = 500;
  document.getElementById('xMaxInput').value = 2500;
  document.getElementById('xMinSlider').value = 500;
  document.getElementById('xMaxSlider').value = 2500;
  applyXLimits();
}

function resetYLimits() {
  document.getElementById('yMinInput').value = 500;
  document.getElementById('yMaxInput').value = 2500;
  document.getElementById('yMinSlider').value = 500;
  document.getElementById('yMaxSlider').value = 2500;
  applyYLimits();
}

function clearError() {
  state.error = false;
  state.error_msg = '';
  document.getElementById('errorBanner')?.classList.remove('visible');
}

// ── 診斷 ─────────────────────────────────────────────────────
async function loadDiagnose() {
  try {
    const r = await fetch('/api/diagnose');
    const d = await r.json();
    addDiagLog(`GPIO: ${d.gpio_mode} | PWM: ${d.pwm_freq}Hz`, 'info');
    addDiagLog(`X: ${d.motors?.X?.pulse}µs | Y1: ${d.motors?.Y1?.pulse}µs | Y2: ${d.motors?.Y2?.pulse}µs`, 'info');
    addDiagLog(`VACUUM: ${d.digital_outputs?.VACUUM?.state ? 'ON' : 'OFF'} | Z_DOWN: ${d.digital_outputs?.Z_DOWN?.state ? 'ON' : 'OFF'}`, 'info');
  } catch (e) {
    addDiagLog('診斷資料載入失敗: ' + e.message, 'error');
  }
}

// ── 馬達測試 ─────────────────────────────────────────────────
async function testMotor(axis) {
  if (isTestRunning) return;
  if (!state.system_run) { toast('請先啟動系統', 'error'); return; }
  isTestRunning = true;
  document.getElementById('testStatus').innerHTML = `<span style="color:var(--accent-amber)">⚙️ 正在測試 ${axis.toUpperCase()} 軸...</span>`;
  addDiagLog(`馬達測試開始: ${axis.toUpperCase()} 軸`, 'warn');

  const pulses = [500, 800, 1100, 1500, 1900, 2200, 2500, 1500];
  for (const pulse of pulses) {
    if (!isTestRunning) break;
    await apiPost('/api/set_position', {
      axis: axis,
      pct: (pulse - 500) / 2000 * 100
    });
    addDiagLog(`${axis.toUpperCase()}: ${pulse}µs`, 'info');
    await sleep(600);
  }

  isTestRunning = false;
  document.getElementById('testStatus').innerHTML = `<span style="color:var(--accent-green)">✅ ${axis.toUpperCase()} 軸測試完成</span>`;
  addDiagLog(`馬達測試完成: ${axis.toUpperCase()} 軸`, 'success');
}

async function testVacuum() {
  if (!state.system_run) { toast('請先啟動系統', 'error'); return; }
  addDiagLog('吸盤測試: ON', 'warn');
  await apiPost('/api/vacuum', { on: true });
  await sleep(1000);
  await apiPost('/api/vacuum', { on: false });
  addDiagLog('吸盤測試: OFF', 'info');
  document.getElementById('testStatus').innerHTML = `<span style="color:var(--accent-green)">✅ VACUUM 測試完成</span>`;
}

function stopTest() {
  isTestRunning = false;
  if (testInterval) { clearInterval(testInterval); testInterval = null; }
  document.getElementById('testStatus').innerHTML = `<span style="color:var(--accent-red)">⏹ 測試已停止</span>`;
  addDiagLog('測試停止', 'warn');
}

// ── 日誌 ─────────────────────────────────────────────────────
function addLog(msg, type = 'info') {
  const panel = document.getElementById('logPanel');
  if (!panel) return;
  const time = new Date().toLocaleTimeString('zh-TW', { hour12: false });
  const colors = { info: 'var(--accent-cyan)', warn: 'var(--accent-amber)', error: 'var(--accent-red)', success: 'var(--accent-green)' };
  const icons  = { info: 'ℹ', warn: '⚠', error: '✘', success: '✔' };
  const line = document.createElement('div');
  line.className = 'log-line';
  line.innerHTML = `<span class="log-time">[${time}]</span> <span style="color:${colors[type] || 'var(--text-sec)'}">${icons[type] || '·'} ${msg}</span>`;
  panel.appendChild(line);
  panel.scrollTop = panel.scrollHeight;
  if (panel.children.length > 200) panel.removeChild(panel.firstChild);
}

function addDiagLog(msg, type = 'info') {
  const panel = document.getElementById('diagLog');
  if (!panel) return;
  const time = new Date().toLocaleTimeString('zh-TW', { hour12: false });
  const colors = { info: 'var(--accent-cyan)', warn: 'var(--accent-amber)', error: 'var(--accent-red)', success: 'var(--accent-green)' };
  const line = document.createElement('div');
  line.className = 'log-line';
  line.innerHTML = `<span class="log-time">[${time}]</span> <span style="color:${colors[type] || 'var(--text-sec)'}">${msg}</span>`;
  panel.appendChild(line);
  panel.scrollTop = panel.scrollHeight;
}

function clearLog() {
  const panel = document.getElementById('logPanel');
  if (panel) panel.innerHTML = '';
}

// ── Toast 通知 ───────────────────────────────────────────────
function toast(msg, type = 'info') {
  const container = document.getElementById('toastContainer');
  if (!container) return;
  const t = document.createElement('div');
  const icons = { success: '✔', error: '✘', info: 'ℹ', warn: '⚠' };
  t.className = `toast toast-${type === 'warn' ? 'info' : type}`;
  t.innerHTML = `<span>${icons[type] || 'ℹ'}</span>${msg}`;
  container.appendChild(t);
  setTimeout(() => t.remove(), 3200);
}

// ── API 工具 ─────────────────────────────────────────────────
async function apiPost(url, body = {}) {
  try {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    return await r.json();
  } catch (e) {
    addLog(`API 錯誤: ${url} - ${e.message}`, 'error');
    return { ok: false, msg: e.message };
  }
}

function setElValue(id, val) {
  const el = document.getElementById(id);
  // settingsDirty 時跳過覆蓋（防止 WebSocket 100ms 廣播覆蓋使用者正在修改的值）
  if (el && document.activeElement !== el && !settingsDirty) el.value = val;
}

// ── 進階設定套用 ────────────────────────────────────────────────
function markSettingsDirty() { settingsDirty = true; }
function clearSettingsDirty() { settingsDirty = false; }

async function applyAdvancedLimits() {
  const xMaxSpeed   = parseFloat(document.getElementById('xMaxSpeedInput').value);
  const xAccelSteps = parseFloat(document.getElementById('xAccelStepsInput').value);
  const xScale      = parseFloat(document.getElementById('xScaleInput').value);
  const yMaxSpeed   = parseFloat(document.getElementById('yMaxSpeedInput').value);
  const yAccelSteps = parseFloat(document.getElementById('yAccelStepsInput').value);
  const yScale      = parseFloat(document.getElementById('yScaleInput').value);
  const r = await apiPost('/api/set_limits', {
    x_max_speed: xMaxSpeed, x_accel_steps: xAccelSteps, x_scale: xScale,
    y_max_speed: yMaxSpeed, y_accel_steps: yAccelSteps, y_scale: yScale
  });
  if (r.ok) {
    toast('進階加減速與校準參數已套用！', 'success');
    clearSettingsDirty();
  }
}

// ── 外觀設定 ────────────────────────────────────────────────────
function previewFont() {
  const family = document.getElementById('fontFamilySelect')?.value || "'Inter',sans-serif";
  const size   = document.getElementById('fontSizeSlider')?.value || 14;
  const prev   = document.getElementById('fontPreviewText');
  if (prev) { prev.style.fontFamily = family; prev.style.fontSize = size + 'px'; }
}

function applyAppearance() {
  const family = document.getElementById('fontFamilySelect')?.value || "'Inter',sans-serif";
  const size   = parseInt(document.getElementById('fontSizeSlider')?.value || 14);
  document.body.style.fontFamily = family;
  document.body.style.fontSize   = size + 'px';
  localStorage.setItem('uiFontFamily', family);
  localStorage.setItem('uiFontSize',   size);
  toast('外觀設定已套用！', 'success');
}

function resetAppearance() {
  const def = "'Inter',sans-serif", sz = 14;
  const sel = document.getElementById('fontFamilySelect');
  const sld = document.getElementById('fontSizeSlider');
  const lbl = document.getElementById('fontSizeLabel');
  if (sel) sel.value = def;
  if (sld) sld.value = sz;
  if (lbl) lbl.textContent = sz;
  document.body.style.fontFamily = def;
  document.body.style.fontSize   = sz + 'px';
  localStorage.removeItem('uiFontFamily');
  localStorage.removeItem('uiFontSize');
  previewFont();
  toast('外觀已重置為預設', 'info');
}

function loadAppearanceFromStorage() {
  const family = localStorage.getItem('uiFontFamily');
  const size   = localStorage.getItem('uiFontSize');
  if (family) {
    document.body.style.fontFamily = family;
    const sel = document.getElementById('fontFamilySelect');
    if (sel) sel.value = family;
  }
  if (size) {
    document.body.style.fontSize = size + 'px';
    const sld = document.getElementById('fontSizeSlider');
    const lbl = document.getElementById('fontSizeLabel');
    if (sld) sld.value = size;
    if (lbl) lbl.textContent = size;
  }
  previewFont();
}

// ── 安全設定 ────────────────────────────────────────────────────
async function applySecuritySettings() {
  const timeout = parseInt(document.getElementById('lockTimeoutInput')?.value || 30);
  const newPwd  = document.getElementById('newPasswordInput')?.value || '';
  const body    = { lock_timeout_min: timeout };
  if (newPwd.trim().length >= 4) body.new_password = newPwd.trim();
  const r = await apiPost('/api/settings/security', body);
  if (r.ok) {
    toast(`安全設定已套用！自動鎖定：${r.lock_timeout_min} 分鐘`, 'success');
    document.getElementById('newPasswordInput').value = '';
    resetAutoLockTimer(r.lock_timeout_min);
    // 若是樹莓派環境，隱藏密碼修改區
    if (r.is_raspberry_pi) {
      const sec = document.getElementById('simPasswordSection');
      if (sec) sec.style.opacity = '0.4';
    }
  }
}

// ── 自動鎖定計時器 ──────────────────────────────────────────────
let _autoLockMs = 30 * 60 * 1000;

function startAutoLockTimer(minutes) {
  clearAutoLockTimer();
  if (minutes != null) _autoLockMs = minutes * 60 * 1000;
  siteAuthTimer = setTimeout(() => {
    toast('閒置自動鎖定，請重新登入', 'warn');
    siteLogout();
  }, _autoLockMs);
}

function resetAutoLockTimer(minutes) { startAutoLockTimer(minutes || null); }
function clearAutoLockTimer() { if (siteAuthTimer) { clearTimeout(siteAuthTimer); siteAuthTimer = null; } }

// 任何使用者互動都重置鎖定倒數
['click','keydown','mousemove','touchstart'].forEach(ev =>
  document.addEventListener(ev, () => { if (siteToken) resetAutoLockTimer(); }, { passive: true })
);

// ── 全站登入/登出 ───────────────────────────────────────────────
async function siteLogin(e) {
  if (e) e.preventDefault();
  const pwdEl = document.getElementById('siteLoginPassword');
  const errEl = document.getElementById('siteLoginError');
  const btn   = document.getElementById('siteLoginBtn');
  const pwd   = pwdEl?.value || '';
  if (!pwd) { if (errEl) errEl.textContent = '請輸入密碼'; return; }

  if (btn) { btn.textContent = '驗證中...'; btn.disabled = true; }
  const r = await apiPost('/api/auth/login', { password: pwd });
  if (btn) { btn.disabled = false; btn.textContent = '🔓 登入系統'; }

  if (r.ok && r.token) {
    siteToken = r.token;
    localStorage.setItem('siteToken', r.token);
    terminalAuthToken = r.token;
    sessionStorage.setItem('terminalToken', r.token);
    hideSiteLoginOverlay();
    startAutoLockTimer();
    const logoutBtn = document.getElementById('btnGlobalLogout');
    if (logoutBtn) logoutBtn.style.display = 'inline-block';
    toast('🔓 登入成功！', 'success');
    addLog('全站登入成功', 'success');
  } else {
    if (errEl) errEl.textContent = r.msg || '密碼錯誤，請重試';
    if (pwdEl) { pwdEl.value = ''; pwdEl.focus(); }
  }
}

async function siteLogout() {
  if (!confirm('確定要登出系統嗎？')) return;
  if (siteToken) await apiPost('/api/auth/logout', { token: siteToken });
  siteToken = ''; terminalAuthToken = '';
  localStorage.removeItem('siteToken');
  sessionStorage.removeItem('terminalToken');
  clearAutoLockTimer();
  showSiteLoginOverlay();
  const lb = document.getElementById('btnGlobalLogout');
  if (lb) lb.style.display = 'none';
  toast('🔒 已登出系統', 'info');
}

function showSiteLoginOverlay() {
  const el = document.getElementById('globalLoginOverlay');
  if (el) { el.style.display = 'flex'; el.style.opacity = '1'; }
  setTimeout(() => document.getElementById('siteLoginPassword')?.focus(), 120);
}

function hideSiteLoginOverlay() {
  const el = document.getElementById('globalLoginOverlay');
  if (!el) return;
  el.style.transition = 'opacity 0.4s';
  el.style.opacity = '0';
  setTimeout(() => { el.style.display = 'none'; el.style.removeProperty('opacity'); el.style.transition = ''; }, 420);
}

async function checkSiteAuthOnLoad() {
  const token = localStorage.getItem('siteToken');
  if (token) {
    const r = await apiPost('/api/auth/status', { token });
    if (r.ok && r.valid) {
      siteToken = token;
      terminalAuthToken = token;
      sessionStorage.setItem('terminalToken', token);
      hideSiteLoginOverlay();
      startAutoLockTimer();
      const lb = document.getElementById('btnGlobalLogout');
      if (lb) lb.style.display = 'inline-block';
      return;
    }
  }
  siteToken = '';
  localStorage.removeItem('siteToken');
  showSiteLoginOverlay();
}


// ── 馬達鎖定/致能開關 ───────────────────────────────────────────
async function toggleMotorLock() {
  const r = await apiPost('/api/set_motor_enable', { locked: !state.motor_locked });
  if (r.ok) {
    toast(r.motor_locked ? '馬達已通電鎖定' : '馬達已斷電釋放 (可手動推動)', r.motor_locked ? 'success' : 'warn');
  }
}

// ── 操作歷史紀錄 ───────────────────────────────────────────────
let historyList = [];

socket.on('new_history', (log) => {
  historyList.push(log);
  if (historyList.length > 200) historyList.shift();
  renderHistory();
});

async function loadHistory() {
  try {
    const r = await fetch('/api/history');
    const data = await r.json();
    if (data.ok) {
      historyList = data.history;
      renderHistory();
    }
  } catch (e) {
    console.error("載入歷史失敗: ", e);
  }
}

function renderHistory(filterText = "") {
  const tbody = document.getElementById('historyTableBody');
  if (!tbody) return;
  tbody.innerHTML = "";
  
  const typeColors = {
    success: 'var(--accent-green)',
    warn: 'var(--accent-amber)',
    error: 'var(--accent-red)',
    info: 'var(--accent-cyan)'
  };
  
  const filtered = historyList.filter(item => 
    item.msg.toLowerCase().includes(filterText.toLowerCase()) ||
    item.type.toLowerCase().includes(filterText.toLowerCase())
  );
  
  [...filtered].reverse().forEach(item => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td style="font-family:'JetBrains Mono';font-size:11px;color:var(--text-dim)">${item.time}</td>
      <td><span class="tag" style="background:${typeColors[item.type] || 'gray'};color:#000;font-weight:bold;padding:2px 6px;border-radius:4px;font-size:10px">${item.type.toUpperCase()}</span></td>
      <td style="color:var(--text-main);font-size:12px">${item.msg}</td>
    `;
    tbody.appendChild(tr);
  });
}

function filterHistory() {
  const text = document.getElementById('historySearch').value;
  renderHistory(text);
}

async function clearHistory() {
  if (confirm("確定要清空所有歷史紀錄嗎？")) {
    const res = await fetch('/api/history', { method: 'DELETE' });
    const d = await res.json();
    if (d.ok) {
      historyList = [];
      renderHistory();
      toast('歷史紀錄已清除', 'info');
    }
  }
}

// ── 動作巨集編輯與控制 ───────────────────────────────────────────
let macros = {};
let currentMacroName = "";

async function loadMacros() {
  try {
    const r = await fetch('/api/macros');
    const data = await r.json();
    if (data.ok) {
      macros = data.macros;
      populateMacroList();
    }
  } catch (e) {
    console.error("載入巨集失敗: ", e);
  }
}

function populateMacroList() {
  const sel = document.getElementById('macroSelect');
  if (!sel) return;
  const prevVal = sel.value;
  sel.innerHTML = "";
  
  Object.keys(macros).forEach(name => {
    const opt = document.createElement('option');
    opt.value = name;
    opt.textContent = name;
    sel.appendChild(opt);
  });
  
  if (prevVal && macros[prevVal]) {
    sel.value = prevVal;
  } else if (sel.options.length > 0) {
    sel.value = sel.options[0].value;
  }
  onMacroSelect(sel.value);
}

function onMacroSelect(name) {
  currentMacroName = name;
  const editor = document.getElementById('macroEditor');
  if (editor && macros[name]) {
    editor.value = macros[name].join('\n');
  }
}

async function saveCurrentMacro() {
  if (!currentMacroName) {
    toast("請先選擇或新增一個巨集", "error");
    return;
  }
  const text = document.getElementById('macroEditor').value;
  const commands = text.split('\n');
  const r = await apiPost('/api/macros', { name: currentMacroName, commands });
  if (r.ok) {
    macros[currentMacroName] = commands;
    toast(`巨集「${currentMacroName}」儲存成功！`, "success");
  }
}

async function createMacro() {
  const name = document.getElementById('newMacroName').value.trim();
  if (!name) {
    toast("請輸入巨集名稱", "error");
    return;
  }
  if (macros[name]) {
    toast("巨集名稱已存在", "error");
    return;
  }
  const commands = ["# 新巨集腳本", "HOME", "DELAY 500"];
  const r = await apiPost('/api/macros', { name, commands });
  if (r.ok) {
    macros[name] = commands;
    populateMacroList();
    document.getElementById('macroSelect').value = name;
    onMacroSelect(name);
    document.getElementById('newMacroName').value = "";
    toast(`巨集「${name}」新增成功`, "success");
  }
}

async function deleteMacro() {
  if (!currentMacroName) return;
  if (confirm(`確定要刪除巨集「${currentMacroName}」嗎？`)) {
    const r = await fetch('/api/macros', {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name: currentMacroName })
    });
    const data = await r.json();
    if (data.ok) {
      delete macros[currentMacroName];
      currentMacroName = "";
      populateMacroList();
      toast("巨集已刪除", "info");
    }
  }
}

async function runMacro() {
  if (!currentMacroName) return;
  const r = await apiPost('/api/macro/run', { action: 'start', name: currentMacroName });
  if (r.ok) {
    toast("巨集已啟動執行...", "success");
  } else {
    toast(r.msg, "error");
  }
}

async function pauseMacro() {
  const btn = document.getElementById('btnPauseMacro');
  const action = btn.textContent.includes("暫停") ? "pause" : "resume";
  const r = await apiPost('/api/macro/run', { action });
  if (r.ok) {
    if (action === "pause") {
      btn.textContent = "▶ 繼續";
      btn.className = "btn btn-success";
    } else {
      btn.textContent = "⏸ 暫停";
      btn.className = "btn btn-amber";
    }
  }
}

async function stopMacro() {
  const r = await apiPost('/api/macro/run', { action: 'stop' });
  if (r.ok) {
    toast("已發送停止指令", "warn");
  }
}

socket.on('macro_status', (data) => {
  const statusText = document.getElementById('macroStatusText');
  const lineText = document.getElementById('macroCurrentLine');
  
  const runBtn = document.getElementById('btnRunMacro');
  const pauseBtn = document.getElementById('btnPauseMacro');
  const stopBtn = document.getElementById('btnStopMacro');
  
  if (statusText) {
    if (data.running) {
      statusText.textContent = data.paused ? "PAUSED" : "RUNNING";
      statusText.style.color = data.paused ? "var(--accent-amber)" : "var(--accent-green)";
    } else {
      statusText.textContent = "IDLE";
      statusText.style.color = "var(--accent-cyan)";
    }
  }
  
  if (lineText && data.current_line !== undefined) {
    lineText.textContent = data.current_line;
  }
  
  if (runBtn) runBtn.disabled = data.running;
  if (pauseBtn) {
    pauseBtn.disabled = !data.running;
    if (!data.running) {
      pauseBtn.textContent = "⏸ 暫停";
      pauseBtn.className = "btn btn-amber";
    } else if (data.paused) {
      pauseBtn.textContent = "▶ 繼續";
      pauseBtn.className = "btn btn-success";
    } else {
      pauseBtn.textContent = "⏸ 暫停";
      pauseBtn.className = "btn btn-amber";
    }
  }
  if (stopBtn) stopBtn.disabled = !data.running;
});

socket.on('macro_msg', (data) => {
  toast(`[巨集提示] ${data.msg}`, 'success');
});

// ── G-code 解析與圖形繪製 ────────────────────────────────────────
let gcodeLines = [];
let gcodeRunning = false;
let gcodeIndex = 0;

function loadGcodeFile(event) {
  const file = event.target.files[0];
  if (!file) return;
  
  document.getElementById('gcodeFileName').textContent = file.name;
  
  const reader = new FileReader();
  reader.onload = function(e) {
    const text = e.target.result;
    gcodeLines = text.split('\n');
    document.getElementById('gcodeTotalLines').textContent = gcodeLines.length;
    document.getElementById('gcodeProgress').textContent = `0 / ${gcodeLines.length}`;
    document.getElementById('btnRunGcode').disabled = false;
    
    drawGcodePath();
  };
  reader.readAsText(file);
}

function drawGcodePath() {
  const canvas = document.getElementById('gcodeCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  // 繪製格線
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 50; i < 300; i += 50) {
    ctx.moveTo(i, 0); ctx.lineTo(i, 300);
    ctx.moveTo(0, i); ctx.lineTo(300, i);
  }
  ctx.stroke();
  
  // 繪製軌跡
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 2;
  ctx.beginPath();
  
  let lastX = 0, lastY = 0;
  let first = true;
  
  gcodeLines.forEach(line => {
    line = line.split(';')[0].split('(')[0].trim();
    if (!line) return;
    const parts = line.split(/\s+/);
    const cmd = parts[0].toUpperCase();
    
    if (cmd === 'G0' || cmd === 'G1') {
      let newX = lastX, newY = lastY;
      parts.forEach(part => {
        if (part.startsWith('X') || part.startsWith('x')) {
          newX = parseFloat(part.substring(1));
        } else if (part.startsWith('Y') || part.startsWith('y')) {
          newY = parseFloat(part.substring(1));
        }
      });
      
      const canvasX = (newX / 200.0) * 300;
      const canvasY = 300 - (newY / 200.0) * 300;
      
      if (first) {
        ctx.moveTo(canvasX, canvasY);
        first = false;
      } else {
        ctx.lineTo(canvasX, canvasY);
      }
      
      lastX = newX;
      lastY = newY;
    }
  });
  ctx.stroke();
  
  // 繪製起點紅點
  ctx.fillStyle = '#ef4444';
  ctx.beginPath();
  ctx.arc(0, 300, 5, 0, Math.PI * 2);
  ctx.fill();
}

async function runGcode() {
  if (gcodeLines.length === 0 || gcodeRunning) return;
  
  gcodeRunning = true;
  document.getElementById('btnRunGcode').disabled = true;
  document.getElementById('btnStopGcode').disabled = false;
  
  addLog("G-code 執行開始", "success");
  
  let lastX = 0, lastY = 0;
  
  for (gcodeIndex = 0; gcodeIndex < gcodeLines.length; gcodeIndex++) {
    if (!gcodeRunning) break;
    
    document.getElementById('gcodeProgress').textContent = `${gcodeIndex + 1} / ${gcodeLines.length}`;
    
    let line = gcodeLines[gcodeIndex].split(';')[0].split('(')[0].trim();
    if (!line) continue;
    
    const parts = line.split(/\s+/);
    const cmd = parts[0].toUpperCase();
    
    if (cmd === 'G0' || cmd === 'G1') {
      let targetX = lastX;
      let targetY = lastY;
      let hasMove = false;
      
      parts.forEach(part => {
        if (part.startsWith('X') || part.startsWith('x')) {
          targetX = parseFloat(part.substring(1));
          hasMove = true;
        } else if (part.startsWith('Y') || part.startsWith('y')) {
          targetY = parseFloat(part.substring(1));
          hasMove = true;
        }
      });
      
      if (hasMove) {
        const xScale = state.x_scale || 10.0;
        const yScale = state.y_scale || 10.0;
        const pulseX = 500 + targetX * xScale;
        const pulseY = 500 + targetY * yScale;
        
        await apiPost('/api/set_position', { axis: 'x', pct: (pulseX - state.x_min) / (state.x_max - state.x_min) * 100 });
        await apiPost('/api/set_position', { axis: 'y', pct: (pulseY - state.y_min) / (state.y_max - state.y_min) * 100 });
        
        while (gcodeRunning) {
          const diffX = Math.abs(state.x_pulse - pulseX);
          const diffY = Math.abs(state.y1_pulse - pulseY);
          if (diffX < 10 && diffY < 10) {
            break;
          }
          await sleep(100);
        }
        
        lastX = targetX;
        lastY = targetY;
      }
    } else if (cmd === 'G28') {
      await apiPost('/api/home', { axis: 'all' });
      // 輪詢等待歸零完成，最多等 15 秒
      for (let i = 0; i < 150 && gcodeRunning; i++) {
        await sleep(100);
        if (!state.is_homing) break;
      }
      lastX = 0;
      lastY = 0;
    } else if (cmd === 'M106') {
      await apiPost('/api/vacuum', { on: true });
      await sleep(500);
    } else if (cmd === 'M107') {
      await apiPost('/api/vacuum', { on: false });
      await sleep(500);
    }
  }
  
  gcodeRunning = false;
  document.getElementById('btnRunGcode').disabled = false;
  document.getElementById('btnStopGcode').disabled = true;
  addLog("G-code 執行結束", "success");
  toast("G-code 執行完成！", "success");
}

function stopGcode() {
  gcodeRunning = false;
  document.getElementById('btnRunGcode').disabled = false;
  document.getElementById('btnStopGcode').disabled = true;
  addLog("G-code 執行已被使用者終止", "warn");
}

// ── G-code 文字編輯器 ────────────────────────────────────────────
function gcodeInsert(text) {
  const editor = document.getElementById('gcodeDirectEditor');
  if (!editor) return;
  const start = editor.selectionStart;
  const before = editor.value.substring(0, start);
  const after  = editor.value.substring(editor.selectionEnd);
  const needNL = before.length > 0 && !before.endsWith('\n');
  editor.value = before + (needNL ? '\n' : '') + text + '\n' + after;
  editor.focus();
  editor.selectionStart = editor.selectionEnd = before.length + (needNL ? 1 : 0) + text.length + 1;
}

function drawGcodePathFromText() {
  const text = document.getElementById('gcodeDirectEditor')?.value || '';
  gcodeLines = text.split('\n');
  drawGcodePath();
  toast('路徑預覽已更新', 'info');
}

async function runGcodeFromText() {
  const text = document.getElementById('gcodeDirectEditor')?.value || '';
  if (!text.trim()) { toast('編輯器為空，請先輸入 G-code', 'error'); return; }
  gcodeLines = text.split('\n');
  const nameEl  = document.getElementById('gcodeFileName');
  const totalEl = document.getElementById('gcodeTotalLines');
  const progEl  = document.getElementById('gcodeProgress');
  if (nameEl)  nameEl.textContent  = '編輯器直接輸入';
  if (totalEl) totalEl.textContent = gcodeLines.length;
  if (progEl)  progEl.textContent  = `0 / ${gcodeLines.length}`;
  const runBtn = document.getElementById('btnRunGcode');
  if (runBtn) runBtn.disabled = false;
  drawGcodePath();
  await runGcode();
}

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }



// ── Web 系統終端機與資安驗證 ─────────────────────────────────────
let terminalAuthToken = sessionStorage.getItem('terminalToken') || "";
let terminalCmdHistory = [];
let terminalHistoryIdx = -1;

async function unlockTerminal(e) {
  if (e) e.preventDefault();
  const pwdInput = document.getElementById('terminalAuthPassword');
  const password = pwdInput ? pwdInput.value : '';
  if (!password) {
    toast('請輸入系統密碼', 'error');
    return;
  }

  toast('正在驗證系統身分與權限...', 'info');
  const res = await apiPost('/api/terminal/auth', { password });
  if (res.ok && res.token) {
    terminalAuthToken = res.token;
    sessionStorage.setItem('terminalToken', res.token);
    toast('🔓 身分驗證成功，終端機已解鎖！', 'success');
    updateTerminalUI(true);
    submitTerminalCmd('sudo systemctl status servo-control');
  } else {
    toast(`❌ 驗證失敗: ${res.msg || '密碼錯誤'}`, 'error');
    if (pwdInput) pwdInput.value = '';
  }
}

async function lockTerminal() {
  if (terminalAuthToken) {
    await apiPost('/api/terminal/logout', { token: terminalAuthToken });
  }
  terminalAuthToken = "";
  sessionStorage.removeItem('terminalToken');
  updateTerminalUI(false);
  toast('🔒 終端機已鎖定', 'info');
}

function updateTerminalUI(authenticated) {
  const authBox = document.getElementById('terminalAuthContainer');
  const mainBox = document.getElementById('terminalMainContainer');
  if (authenticated) {
    if (authBox) authBox.style.display = 'none';
    if (mainBox) mainBox.style.display = 'block';
    setTimeout(() => document.getElementById('terminalInput')?.focus(), 50);
  } else {
    if (authBox) authBox.style.display = 'block';
    if (mainBox) mainBox.style.display = 'none';
  }
}

async function cmdSystemPower(action) {
  const actionText = action === 'reboot' ? '重新啟動' : '關閉電源';
  
  let pwd = "";
  if (!terminalAuthToken) {
    pwd = prompt(`⚠️ 確定要將樹梅派系統【${actionText}】嗎？\n請輸入 Linux 系統權限密碼 (預設: Nice) 以驗證身分：`);
    if (pwd === null) return;
    if (!pwd.trim()) {
      toast("需輸入密碼方可執行系統電源指令", "error");
      return;
    }
  } else {
    if (!confirm(`⚠️ 確定要將樹梅派系統【${actionText}】嗎？`)) return;
  }

  toast(`正在發送 ${actionText} 指令...`, 'warn');
  addLog(`發送系統 ${actionText} 指令`, 'warn');

  const r = await apiPost('/api/system_power', {
    action: action,
    token: terminalAuthToken,
    password: pwd
  });

  if (r.ok) {
    toast(`樹梅派正在${actionText}，網頁連線將於幾秒後中斷...`, 'info');
  } else {
    toast(`執行失敗: ${r.msg || '密碼驗證失敗'}`, 'error');
  }
}

async function submitTerminalCmd(cmdText = null) {
  if (!terminalAuthToken) {
    updateTerminalUI(false);
    return;
  }

  const inputEl = document.getElementById('terminalInput');
  const outputEl = document.getElementById('terminalOutput');
  const screenEl = document.getElementById('terminalScreen');
  if (!outputEl || !inputEl) return;

  const cmd = cmdText !== null ? cmdText : inputEl.value.trim();
  if (!cmd) return;

  if (cmdText === null) {
    inputEl.value = '';
    terminalCmdHistory.push(cmd);
    terminalHistoryIdx = terminalCmdHistory.length;
  }

  const timeStr = new Date().toLocaleTimeString('zh-TW', { hour12: false });
  const cmdLine = document.createElement('div');
  cmdLine.style.color = '#38bdf8';
  cmdLine.style.marginTop = '8px';
  cmdLine.innerHTML = `<span style="color:var(--accent-cyan)">[${timeStr}]</span> <span style="color:var(--accent-green)">nice-pi4@pi:~$</span> <strong>${escapeHtml(cmd)}</strong>`;
  outputEl.appendChild(cmdLine);

  const execLine = document.createElement('div');
  execLine.style.color = 'var(--text-dim)';
  execLine.textContent = '⚡ 執行中...';
  outputEl.appendChild(execLine);
  screenEl.scrollTop = screenEl.scrollHeight;

  const res = await apiPost('/api/terminal/exec', { cmd, token: terminalAuthToken });
  execLine.remove();

  if (res.auth_required) {
    toast('🔒 身分驗證已過期，請重新解鎖', 'warn');
    lockTerminal();
    return;
  }

  const outBlock = document.createElement('pre');
  outBlock.style.whiteSpace = 'pre-wrap';
  outBlock.style.wordBreak = 'break-all';
  outBlock.style.fontFamily = "'JetBrains Mono', monospace";
  outBlock.style.margin = '4px 0 12px 0';
  outBlock.style.padding = '8px 12px';
  outBlock.style.borderRadius = '4px';
  outBlock.style.background = res.ok ? 'rgba(15,23,42,0.6)' : 'rgba(127,29,29,0.2)';
  outBlock.style.borderLeft = `3px solid ${res.ok ? 'var(--accent-green)' : 'var(--accent-red)'}`;
  outBlock.style.color = res.ok ? '#e2e8f0' : '#fca5a5';
  outBlock.textContent = res.output || '(無輸出內容)';

  outputEl.appendChild(outBlock);
  screenEl.scrollTop = screenEl.scrollHeight;
}

function quickTerminalCmd(cmd) {
  const inputEl = document.getElementById('terminalInput');
  if (inputEl) inputEl.value = cmd;
  submitTerminalCmd(cmd);
}

function clearTerminal() {
  const outputEl = document.getElementById('terminalOutput');
  if (outputEl) outputEl.innerHTML = '';
}

function onTerminalKey(e) {
  if (e.key === 'Enter') {
    e.preventDefault();
    submitTerminalCmd();
  } else if (e.key === 'ArrowUp') {
    if (terminalCmdHistory.length > 0 && terminalHistoryIdx > 0) {
      terminalHistoryIdx--;
      e.target.value = terminalCmdHistory[terminalHistoryIdx];
    }
  } else if (e.key === 'ArrowDown') {
    if (terminalHistoryIdx < terminalCmdHistory.length - 1) {
      terminalHistoryIdx++;
      e.target.value = terminalCmdHistory[terminalHistoryIdx];
    } else {
      terminalHistoryIdx = terminalCmdHistory.length;
      e.target.value = '';
    }
  }
}

function escapeHtml(str) {
  return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

// ── GitHub 系統更新 ───────────────────────────────────────────────
async function checkUpdate() {
  const btn     = document.getElementById('btnCheckUpdate');
  const pullBtn = document.getElementById('btnPullUpdate');
  const badge   = document.getElementById('updateStatusBadge');
  const localEl  = document.getElementById('updateLocalSha');
  const remoteEl = document.getElementById('updateRemoteSha');
  const msgEl    = document.getElementById('updateRemoteMsg');
  const metaEl   = document.getElementById('updateRemoteMeta');

  if (btn) { btn.textContent = '🔍 檢查中...'; btn.disabled = true; }
  if (badge) { badge.textContent = '連線中'; badge.style.background = 'rgba(59,130,246,0.2)'; badge.style.color = 'var(--accent-cyan)'; }

  try {
    const r = await fetch('/api/update/check');
    const data = await r.json();

    if (!data.ok) {
      if (badge) { badge.textContent = '❌ 無法連線'; badge.style.background = 'rgba(239,68,68,0.2)'; badge.style.color = 'var(--accent-red)'; }
      toast(`無法連線至 GitHub: ${data.msg}`, 'error');
      return;
    }

    if (localEl)  localEl.textContent  = data.local_sha  || '-';
    if (remoteEl) remoteEl.textContent = data.remote_sha || '-';
    if (msgEl)    msgEl.textContent    = `💬 ${data.remote_msg || ''}`;
    if (metaEl)   metaEl.textContent   = `👤 ${data.remote_author} · 📅 ${data.remote_date} · 🌿 ${data.branch}`;

    if (data.up_to_date) {
      if (badge) { badge.textContent = '✅ 已是最新'; badge.style.background = 'rgba(34,197,94,0.2)'; badge.style.color = 'var(--accent-green)'; }
      if (pullBtn) pullBtn.disabled = true;
      toast('已是最新版本！', 'success');
    } else {
      if (badge) { badge.textContent = '🆙 有新版本'; badge.style.background = 'rgba(251,191,36,0.2)'; badge.style.color = 'var(--accent-amber)'; }
      if (pullBtn) pullBtn.disabled = false;
      toast(`有新版本可更新！(${data.local_sha} → ${data.remote_sha})`, 'warn');
    }
  } catch (e) {
    toast(`檢查失敗: ${e.message}`, 'error');
  } finally {
    if (btn) { btn.textContent = '🔍 檢查更新'; btn.disabled = false; }
  }
}

async function pullUpdate() {
  if (!siteToken) { toast('請先登入系統', 'error'); return; }
  if (!confirm('⚠️ 確定要更新系統嗎？\n\n本地未提交的修改將被覆蓋，更新後服務可能短暫中斷。')) return;

  const btn    = document.getElementById('btnPullUpdate');
  const outEl  = document.getElementById('updateOutputPanel');
  const badge  = document.getElementById('updateStatusBadge');
  const autoRestart = document.getElementById('updateAutoRestart')?.checked ?? true;

  if (btn) { btn.textContent = '⬇️ 更新中...'; btn.disabled = true; }
  if (badge) { badge.textContent = '⏳ 更新中'; badge.style.background = 'rgba(59,130,246,0.2)'; badge.style.color = 'var(--accent-cyan)'; }
  if (outEl) { outEl.style.display = 'block'; outEl.textContent = '⏳ 正在執行 git fetch + reset --hard，請稍候...\n'; }

  toast('開始更新，請稍候...', 'warn');

  try {
    const r = await apiPost('/api/update/pull', { token: siteToken, restart: autoRestart });

    if (outEl) outEl.textContent = r.output || r.msg || '(無輸出)';

    if (r.ok) {
      if (badge) { badge.textContent = '✅ 更新完成'; badge.style.background = 'rgba(34,197,94,0.2)'; badge.style.color = 'var(--accent-green)'; }
      const localEl = document.getElementById('updateLocalSha');
      if (localEl && r.sha) localEl.textContent = r.sha;
      toast(r.msg || '更新成功！', 'success');
      addLog(`系統更新完成 → ${r.sha || ''}`, 'success');
      if (btn) { btn.textContent = '✅ 已更新'; btn.disabled = true; }
    } else {
      if (badge) { badge.textContent = '❌ 更新失敗'; badge.style.background = 'rgba(239,68,68,0.2)'; badge.style.color = 'var(--accent-red)'; }
      toast(`更新失敗: ${r.msg}`, 'error');
      if (btn) { btn.textContent = '⬇️ 下載並更新'; btn.disabled = false; }
    }
  } catch (e) {
    toast(`更新例外: ${e.message}`, 'error');
    if (btn) { btn.textContent = '⬇️ 下載並更新'; btn.disabled = false; }
  }
}



// ── 時鐘 ─────────────────────────────────────────────────────
function updateClock() {
  const el = document.getElementById('clockDisplay');
  if (el) el.textContent = new Date().toLocaleTimeString('zh-TW', { hour12: false });
}

// ── 鍵盤快捷鍵 ───────────────────────────────────────────────
document.addEventListener('keydown', (e) => {
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName)) return;
  if (!state.system_run) return;
  switch (e.key) {
    case 'ArrowLeft':  e.preventDefault(); moveAxis('x', -1); break;
    case 'ArrowRight': e.preventDefault(); moveAxis('x',  1); break;
    case 'ArrowUp':    e.preventDefault(); moveAxis('y',  1); break;
    case 'ArrowDown':  e.preventDefault(); moveAxis('y', -1); break;
    case 'v': case 'V': toggleVacuum(); break;
    case 'z': case 'Z': toggleZDown(); break;
    case 'h': case 'H': cmdHome('all'); break;
  }
});

async function checkTerminalAuthOnLoad() {
  const token = sessionStorage.getItem('terminalToken');
  if (token) {
    const res = await apiPost('/api/terminal/check_token', { token });
    if (res.ok && res.valid) {
      terminalAuthToken = token;
      updateTerminalUI(true);
      return;
    }
  }
  terminalAuthToken = "";
  sessionStorage.removeItem('terminalToken');
  updateTerminalUI(false);
}

// ── 初始化 ───────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  initCharts();
  setInterval(updateClock, 1000);
  updateClock();
  loadHistory();
  loadMacros();
  drawGcodePath();
  loadAppearanceFromStorage();

  // 設定頁輸入框監聽—標記 dirty，防止 WebSocket 覆蓋
  ['xMaxSpeedInput','xAccelStepsInput','xScaleInput',
   'yMaxSpeedInput','yAccelStepsInput','yScaleInput'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('input', markSettingsDirty);
  });

  // 終端機驗證密碼框 Enter 鍵
  const pwdInput = document.getElementById('terminalAuthPassword');
  if (pwdInput) {
    pwdInput.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); unlockTerminal(e); }
    });
  }

  addLog('系統介面載入完成', 'success');
  addLog('使用 ← → ↑ ↓ 方向鍵快速控制馬達', 'info');
  addLog('V=吸盤切換 | Z=手臂切換 | H=全部歸零', 'info');

  // 全站認證檢查（最後執行，可能會顯示登入遮罩）
  checkSiteAuthOnLoad();
});

