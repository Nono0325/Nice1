/**
 * 3D 實時機械模擬器 - Three.js Engine
 */

let scene, camera, renderer, controls;
let gantryX, sliderY, zArm, suctionTip, workpiece;
let gridHelper;
let isInitialized = false;

// 儲存目前運動狀態的平滑目標
let targetX = 0; // mapped to Three.js coordinates
let targetZ = 0; // mapped to Three.js coordinates
let targetArmY = 1.8; // Z-axis arm vertical height
let isVacuumActive = false;
let isGrabbed = false;

function init3D() {
  const container = document.getElementById('canvas3d-container');
  if (!container || isInitialized) return;

  // 1. 建立場景與渲染器
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x030712);
  scene.fog = new THREE.FogExp2(0x030712, 0.015);

  camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 1000);
  camera.position.set(18, 14, 22);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // 2. 建立 OrbitControls 視角控制器
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI / 2 - 0.05; // 限制不能看到地板下方
  controls.minDistance = 5;
  controls.maxDistance = 50;

  // 3. 建立光源
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(15, 25, 10);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 1024;
  dirLight.shadow.mapSize.height = 1024;
  scene.add(dirLight);

  const pointLightX = new THREE.PointLight(0x00f0ff, 1.0, 10);
  pointLightX.position.set(0, 5, 0);
  scene.add(pointLightX);

  // 4. 建立底座與格線網格
  gridHelper = new THREE.GridHelper(30, 30, 0x1e293b, 0x0f172a);
  gridHelper.position.y = 0.01;
  scene.add(gridHelper);

  const baseGeo = new THREE.BoxGeometry(26, 0.4, 26);
  const baseMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.8 });
  const baseMesh = new THREE.Mesh(baseGeo, baseMat);
  baseMesh.position.y = -0.2;
  baseMesh.receiveShadow = true;
  scene.add(baseMesh);

  // 繪製工作站固定點
  // 取件點座台
  createPlatform(-8.0, -2.0, 0.4, 0x1e293b);
  // 放件點座台
  createPlatform(8.0, 2.0, 0.4, 0x1e293b);

  // 5. 建立機械手臂的各個運動節點 (X軸導體 -> Y軸滑塊 -> Z軸吸盤)
  
  // 5.1. X軸 龍門導軌 (Gantry)
  const gantryXGeo = new THREE.BoxGeometry(2.0, 1.0, 24.0);
  const gantryXMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.2 });
  gantryX = new THREE.Mesh(gantryXGeo, gantryXMat);
  gantryX.position.set(0, 3.5, 0);
  gantryX.castShadow = true;
  scene.add(gantryX);

  // X軸 左右雙軌滑桿
  const railGeo = new THREE.CylinderGeometry(0.12, 0.12, 25.8, 16);
  const railMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.9, roughness: 0.1 });
  const rail1 = new THREE.Mesh(railGeo, railMat);
  rail1.rotation.x = Math.PI / 2;
  rail1.position.set(-6, 3.8, 0);
  scene.add(rail1);
  const rail2 = rail1.clone();
  rail2.position.set(6, 3.8, 0);
  scene.add(rail2);

  // 5.2. Y軸 滑台滑塊 (Slider)
  const sliderYGeo = new THREE.BoxGeometry(2.4, 1.2, 2.2);
  const sliderYMat = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.4 });
  sliderY = new THREE.Mesh(sliderYGeo, sliderYMat);
  sliderY.position.set(0, 0.4, 0); // 這是相對於 gantryX 的 local position
  sliderY.castShadow = true;
  gantryX.add(sliderY); // Y 滑塊是 X 龍門的子元件！

  // 5.3. Z軸 直立氣壓缸/吸嘴桿 (Z Actuator)
  const zArmGeo = new THREE.CylinderGeometry(0.15, 0.15, 3.2, 16);
  const zArmMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.8, roughness: 0.1 });
  zArm = new THREE.Mesh(zArmGeo, zArmMat);
  zArm.position.set(0, 1.8, 0); // 相對於 sliderY 的 local position
  zArm.castShadow = true;
  sliderY.add(zArm);

  // 5.4. 吸嘴末端圓形強力電磁鐵 (Circular Electromagnet DC 5V/6V/12V/24V)
  const cupGeo = new THREE.CylinderGeometry(0.55, 0.55, 0.35, 32);
  const cupMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.95, roughness: 0.15 });
  const cup = new THREE.Mesh(cupGeo, cupMat);
  cup.position.y = -1.6; // 位於 Z-arm 最底部
  zArm.add(cup);

  // 5.5. 電磁鐵通電/磁力指示光圈
  const ringGeo = new THREE.TorusGeometry(0.42, 0.05, 8, 24);
  const ledMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
  suctionTip = new THREE.Mesh(ringGeo, ledMat);
  suctionTip.rotation.x = Math.PI / 2;
  suctionTip.position.y = -1.78;
  zArm.add(suctionTip);

  // 6. 建立金屬工件 (Workpiece - 工業金屬衝壓件)
  const workpieceGeo = new THREE.BoxGeometry(1.4, 0.4, 1.4);
  const workpieceMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.8, roughness: 0.2 });
  workpiece = new THREE.Mesh(workpieceGeo, workpieceMat);
  workpiece.position.set(-8.0, 0.6, -2.0); // 初始放在取件點 A
  workpiece.castShadow = true;
  scene.add(workpiece);

  // 7. 註冊視窗縮放事件
  window.addEventListener('resize', onWindowResize);

  isInitialized = true;
  animate();

  document.getElementById('sim3d-status').textContent = "READY";
}

function createPlatform(x, z, h, color) {
  const geo = new THREE.BoxGeometry(2.4, h, 2.4);
  const mat = new THREE.MeshStandardMaterial({ color: color, roughness: 0.8 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(x, h/2, z);
  mesh.receiveShadow = true;
  mesh.castShadow = true;
  scene.add(mesh);
}

function onWindowResize() {
  const container = document.getElementById('canvas3d-container');
  if (!container || !renderer || !camera) return;
  const w = container.clientWidth || 800;
  const h = container.clientHeight || 600;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

function reset3DView() {
  if (controls) {
    controls.reset();
    camera.position.set(18, 14, 22);
  }
}

// ── 步進馬達坐報映射更新 ──────────────────────────────────────────
function update3D(stateData) {
  if (!isInitialized) return;

  // 坐標映射規則：
  // 1. X 軸行程範圍 500 ~ 2500 us -> 映射至 Three.js X 軸範圍 -8.0 到 +8.0 單位
  const pulseX = stateData.x_pulse || 1500;
  targetX = -8.0 + ((pulseX - 500) / 2000.0) * 16.0;

  // 2. Y 軸行程保留相容 (或固定於中間)
  const pulseY = stateData.y1_pulse || 1500;
  targetZ = -6.0 + ((pulseY - 500) / 2000.0) * 12.0;

  // 3. Z-axis arm 上升/下降映射 (500us=最下E點/0.3, 2500us=頂部D點/1.8)
  const pulseZ = stateData.z_pulse !== undefined ? stateData.z_pulse : (stateData.y1_pulse || 1500);
  if (stateData.z_pulse !== undefined) {
    targetArmY = 0.3 + ((pulseZ - 500) / 2000.0) * 1.5;
  } else {
    targetArmY = stateData.arm_down ? 0.3 : 1.8;
  }

  // 4. 圓形直流電磁鐵致能狀態
  isVacuumActive = !!(stateData.magnet !== undefined ? stateData.magnet : stateData.vacuum);
  
  // 更新 HMI 的狀態文字
  const statusEl = document.getElementById('sim3d-status');
  if (statusEl) {
    if (stateData.pickup_active) {
      statusEl.textContent = "AUTO SEQUENCE RUNNING";
      statusEl.style.color = "var(--accent-amber)";
    } else if (stateData.system_run) {
      statusEl.textContent = "LIVE MONITORED";
      statusEl.style.color = "var(--accent-green)";
    } else {
      statusEl.textContent = "SYSTEM STANDBY";
      statusEl.style.color = "var(--accent-cyan)";
    }
  }
}

// ── 渲染迴圈與物理模擬 ────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);

  if (!isInitialized) return;

  // 1. 平滑運動插值 (Lerp) 讓虛擬馬達動得非常滑順
  gantryX.position.x += (targetX - gantryX.position.x) * 0.15;
  sliderY.position.z += (targetZ - sliderY.position.z) * 0.15;
  zArm.position.y += (targetArmY - zArm.position.y) * 0.15;

  // 2. 獲取吸嘴 LED 世界坐標
  const ledWorldPos = new THREE.Vector3();
  suctionTip.getWorldPosition(ledWorldPos);

  // 3. 偵測吸附狀態與工件動作
  if (isVacuumActive) {
    // 當吸嘴十分接近工件時，且真空開啟，即視為被抓取
    const distToWorkpiece = ledWorldPos.distanceTo(workpiece.position);
    if (distToWorkpiece < 1.0) {
      isGrabbed = true;
      suctionTip.material.color.setHex(0x10b981); // 吸住時亮綠燈
    }
  } else {
    isGrabbed = false;
    suctionTip.material.color.setHex(0xef4444); // 未吸住時亮紅燈
  }

  // 4. 工件物理位置跟隨或重力掉落
  if (isGrabbed) {
    // 牢牢貼在吸盤下方
    workpiece.position.copy(ledWorldPos);
    workpiece.position.y -= 0.4; // 偏移出吸盤體積
  } else {
    // 當沒被吸住且高於桌面時，以簡易重力掉落到工作台/地板高度 (Y=0.4)
    if (workpiece.position.y > 0.4) {
      workpiece.position.y -= 0.2; // 掉落速度
      if (workpiece.position.y < 0.4) workpiece.position.y = 0.4;
    }
  }

  // 5. 更新視角控制器
  controls.update();

  // 6. 渲染場景
  renderer.render(scene, camera);
}
