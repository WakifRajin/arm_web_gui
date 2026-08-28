// app.js -- Team Interplanetar rover arm dashboard.
// Talks to the FastAPI backend over one websocket (/ws) for live
// telemetry + commands, and REST (/api/*) for a couple of one-shot
// reads. No build step -- plain JS on purpose so it runs unmodified
// on whatever machine sits next to the arm.

const state = {
  ws: null,
  config: null,
  joints: {},        // name -> latest status dict
  discovered: [],
  selectedJoint: null,
  jointOrder: [],
};

const LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR"];

function $(id) { return document.getElementById(id); }

// ------------------------------------------------------------- toasts ----
function showToast(message, kind = "info") {
  const stack = $("toastStack");
  const el = document.createElement("div");
  el.className = `toast ${kind}`;
  el.textContent = message;
  stack.appendChild(el);
  setTimeout(() => {
    el.classList.add("fade-out");
    setTimeout(() => el.remove(), 260);
  }, 3600);
}

// -------------------------------------------------------------- tabs ----
function initTabs() {
  document.querySelectorAll(".tab-btn").forEach(btn => {
    btn.addEventListener("click", () => switchTab(btn.dataset.tab));
  });
}
function switchTab(name) {
  document.querySelectorAll(".tab-btn").forEach(b => b.classList.toggle("active", b.dataset.tab === name));
  document.querySelectorAll(".tabpanel").forEach(p => p.classList.toggle("active", p.id === `tab-${name}`));
  if (name === "canbus") refreshCanStatus();
}

// ---------------------------------------------------------------- WS ----
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  state.ws = ws;

  ws.onopen = () => {
    $("wsDot").className = "dot dot-green";
    $("wsText").textContent = "connected";
  };
  ws.onclose = () => {
    $("wsDot").className = "dot dot-red";
    $("wsText").textContent = "disconnected -- retrying...";
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => handleMessage(JSON.parse(evt.data));
}

function send(cmd) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(cmd));
  }
}

function handleMessage(msg) {
  switch (msg.type) {
    case "config":
      state.config = msg.config;
      state.jointOrder = msg.config.joints.map(j => j.name);
      buildStaticJointUI();
      populateWristConfigUI();
      $("canIfaceName").textContent = msg.config.can_interface || "can0";
      $("canBitrate").textContent = msg.config.can_bitrate ? `${msg.config.can_bitrate} bps` : "--";
      break;
    case "status":
      state.joints[msg.name] = msg;
      renderJointCard(msg);
      if (msg.name === state.selectedJoint) renderControlPanel(msg);
      updateBusBanner();
      break;
    case "discovered":
      state.discovered = msg.devices;
      renderDiscovery();
      break;
    case "wrist_status":
      renderWristStatus(msg);
      break;
    case "info":
      appendLog({ ts: Date.now() / 1000, level: "INFO", message: msg.text, joint: null });
      break;
    case "log":
      appendLog(msg);
      break;
    case "estop_ack":
      appendLog({ ts: Date.now() / 1000, level: "WARNING", message: "E-STOP acknowledged by worker.", joint: null });
      showToast("E-STOP acknowledged -- all torque disabled.", "warning");
      break;
  }
}

// ------------------------------------------------------- bus watchdog UI --
function updateBusBanner() {
  const names = Object.keys(state.joints);
  if (!names.length) return;
  const allDown = names.every(n => state.joints[n].connected === false);
  $("busBanner").classList.toggle("hidden", !allDown);
}

// ---------------------------------------------------------- dashboard ----
function buildStaticJointUI() {
  const grid = $("jointGrid");
  grid.innerHTML = "";
  const batchBody = $("batchTableBody");
  batchBody.innerHTML = "";
  const jointSelect = $("jointSelect");
  jointSelect.innerHTML = "";
  const logJointFilter = $("logJointFilter");
  logJointFilter.innerHTML = '<option value="">All</option>';

  for (const j of state.config.joints) {
    const card = document.createElement("div");
    card.className = "joint-card disconnected";
    card.id = `card-${j.name}`;
    card.innerHTML = `
      <div class="jc-head">
        <span class="jc-name">${j.name}</span>
        <span class="jc-addr">ID ${j.addr}</span>
      </div>
      <div class="jc-angle" id="angle-${j.name}">--<small> deg</small></div>
      <div class="jc-mini-gauge"><div class="jc-mini-gauge-fill" id="mgauge-${j.name}" style="left:50%;"></div></div>
      <div class="jc-row"><span>target</span><span id="target-${j.name}">--</span></div>
      <div class="jc-row"><span>velocity</span><span id="vel-${j.name}">--</span></div>
      <div class="jc-row"><span>current</span><span id="current-${j.name}">--</span></div>
      <div class="jc-row"><span>voltage</span><span id="voltage-${j.name}">--</span></div>
      <div class="jc-row"><span>temp</span><span id="temp-${j.name}">--</span></div>
      <div class="jc-footer">
        <span class="jc-fault ok" id="fault-${j.name}">--</span>
        <span class="jc-torque off" id="torque-${j.name}">OFF</span>
      </div>
    `;
    card.addEventListener("click", () => { selectJoint(j.name); switchTab("control"); });
    grid.appendChild(card);

    const row = document.createElement("tr");
    row.innerHTML = `
      <td><input type="checkbox" class="batch-cb" value="${j.name}"></td>
      <td>${j.name} <span class="hint">(ID ${j.addr})</span></td>
      <td><input type="number" step="0.1" class="batch-angle" data-joint="${j.name}" value="0"></td>
    `;
    const cb = row.querySelector(".batch-cb");
    const updateRowState = () => row.classList.toggle("batch-row-unchecked", !cb.checked);
    cb.addEventListener("change", updateRowState);
    updateRowState();
    batchBody.appendChild(row);

    const opt = document.createElement("option");
    opt.value = j.name; opt.textContent = `${j.name} (ID ${j.addr})`;
    jointSelect.appendChild(opt);

    const opt2 = document.createElement("option");
    opt2.value = j.name; opt2.textContent = j.name;
    logJointFilter.appendChild(opt2);
  }

  if (!state.selectedJoint && state.config.joints.length) {
    selectJoint(state.config.joints[0].name);
  }
}

function renderJointCard(s) {
  const card = $(`card-${s.name}`);
  if (!card) return;
  card.className = "joint-card " + (s.connected ? "connected" : "disconnected");
  $(`angle-${s.name}`).innerHTML = s.connected ? `${s.angle.toFixed(2)}<small> deg</small>` : `--<small> deg</small>`;
  $(`target-${s.name}`).textContent = s.target_angle != null ? s.target_angle.toFixed(2) : "--";
  $(`vel-${s.name}`).textContent = s.cmd_velocity_deg_s != null ? `${s.cmd_velocity_deg_s.toFixed(1)} deg/s` : "--";
  $(`current-${s.name}`).textContent = s.connected ? `${s.current.toFixed(2)} A` : "--";
  $(`voltage-${s.name}`).textContent = s.voltage != null ? `${s.voltage.toFixed(2)} V` : "--";
  $(`temp-${s.name}`).textContent = s.temperature != null ? `${s.temperature} C` : "--";
  const fault = $(`fault-${s.name}`);
  fault.textContent = s.fault_text || "--";
  fault.className = "jc-fault " + (s.connected && s.fault_text === "OK" ? "ok" : "bad");
  const torque = $(`torque-${s.name}`);
  torque.textContent = s.enabled ? "ON" : "OFF";
  torque.className = "jc-torque " + (s.enabled ? "on" : "off");
  const mgauge = $(`mgauge-${s.name}`);
  if (mgauge && s.connected && s.min_deg != null && s.max_deg != null) {
    const pct = clampPct((s.angle - s.min_deg) / (s.max_deg - s.min_deg) * 100);
    mgauge.style.left = `${pct}%`;
  }
}

function clampPct(v) { return Math.max(0, Math.min(100, v)); }

// ------------------------------------------------------- control panel ----
function selectJoint(name) {
  state.selectedJoint = name;
  $("jointSelect").value = name;
  const jcfg = state.config.joints.find(j => j.name === name);
  $("limMin").value = jcfg.min_deg;
  $("limMax").value = jcfg.max_deg;
  $("limSpeed").value = jcfg.max_speed_rpm;
  $("limAccel").value = jcfg.max_accel_rpm_s;
  $("limCurrent").value = jcfg.max_current_a;
  $("posKp").value = jcfg.pos_kp ?? 0;
  $("posKi").value = jcfg.pos_ki ?? 0;
  $("velKp").value = jcfg.vel_kp ?? 0;
  $("velKi").value = jcfg.vel_ki ?? 0;
  $("dirSelect").value = jcfg.direction ?? 1;
  $("angleSlider").min = jcfg.min_deg;
  $("angleSlider").max = jcfg.max_deg;
  $("gaugeMin").textContent = `${jcfg.min_deg}°`;
  $("gaugeMax").textContent = `${jcfg.max_deg}°`;
  if (state.joints[name]) renderControlPanel(state.joints[name]);
}

function renderControlPanel(s) {
  if (s.name !== state.selectedJoint) return;
  if (document.activeElement !== $("angleInput") && document.activeElement !== $("angleSlider")) {
    if (s.target_angle != null) {
      $("angleInput").value = s.target_angle.toFixed(2);
      $("angleSlider").value = s.target_angle;
    }
  }

  const badge = $("selJointBadge");
  badge.textContent = s.enabled ? "torque ON" : "torque OFF";
  badge.className = "badge " + (s.enabled ? "badge-on" : "badge-dim");

  if (s.direction != null && document.activeElement !== $("dirSelect")) {
    $("dirSelect").value = s.direction;
  }

  if (s.min_deg != null && s.max_deg != null && s.connected) {
    const range = s.max_deg - s.min_deg || 1;
    const curPct = clampPct((s.angle - s.min_deg) / range * 100);
    $("gaugeCurrent").style.left = `${curPct}%`;
    $("gaugeFill").style.width = `${curPct}%`;
    $("gaugeCurrent").classList.remove("hidden");
    if (s.target_angle != null) {
      const tgtPct = clampPct((s.target_angle - s.min_deg) / range * 100);
      $("gaugeTarget").style.left = `${tgtPct}%`;
      $("gaugeTarget").classList.remove("hidden");
    }
    const vel = s.cmd_velocity_deg_s != null ? s.cmd_velocity_deg_s.toFixed(1) : "0.0";
    $("gaugeReadout").innerHTML = `${s.angle.toFixed(2)} deg &nbsp;|&nbsp; ${vel} deg/s`;
  } else {
    $("gaugeCurrent").classList.add("hidden");
    $("gaugeTarget").classList.add("hidden");
    $("gaugeReadout").textContent = "-- deg  |  -- deg/s";
  }
}

$("jointSelect").addEventListener("change", (e) => selectJoint(e.target.value));

function submitGoTo() {
  const deg = parseFloat($("angleInput").value);
  if (Number.isNaN(deg)) return;
  send({ type: "go_to_angle", joint: state.selectedJoint, deg, ramped: $("rampedToggle").checked });
}
$("btnGo").addEventListener("click", submitGoTo);
$("angleInput").addEventListener("keydown", (e) => { if (e.key === "Enter") submitGoTo(); });
$("angleSlider").addEventListener("input", (e) => { $("angleInput").value = e.target.value; });
$("angleSlider").addEventListener("change", (e) => {
  send({ type: "go_to_angle", joint: state.selectedJoint, deg: parseFloat(e.target.value), ramped: $("rampedToggle").checked });
});

$("btnEnable").addEventListener("click", () => send({ type: "enable", joint: state.selectedJoint }));
$("btnDisable").addEventListener("click", () => send({ type: "disable", joint: state.selectedJoint }));
$("btnClearFault").addEventListener("click", () => send({ type: "clear_fault", joint: state.selectedJoint }));
$("btnSetHome").addEventListener("click", () => {
  if (confirm(`Set HOME here for '${state.selectedJoint}'?\n\nTorque should be OFF and the joint physically at your intended zero pose. This is stored permanently in the motor.`)) {
    send({ type: "set_home", joint: state.selectedJoint });
  }
});

$("btnSaveLimits").addEventListener("click", () => {
  const name = state.selectedJoint;
  const min_deg = parseFloat($("limMin").value);
  const max_deg = parseFloat($("limMax").value);
  const max_speed_rpm = parseFloat($("limSpeed").value);
  const max_accel_rpm_s = parseFloat($("limAccel").value);
  const max_current_a = parseFloat($("limCurrent").value);
  if ([min_deg, max_deg, max_speed_rpm, max_accel_rpm_s, max_current_a].some(Number.isNaN)) {
    showToast("Limits/speed: enter numbers in every field.", "error"); return;
  }
  if (min_deg >= max_deg) { showToast("Min angle must be less than max angle.", "error"); return; }
  if (max_speed_rpm <= 0 || max_accel_rpm_s <= 0 || max_current_a <= 0) {
    showToast("Speed, accel, and current must be greater than zero.", "error"); return;
  }
  send({ type: "set_limits", joint: name, min_deg, max_deg });
  send({ type: "set_speed_limits", joint: name, max_speed_rpm, max_accel_rpm_s, max_current_a });
  const jcfg = state.config.joints.find(j => j.name === name);
  Object.assign(jcfg, { min_deg, max_deg, max_speed_rpm, max_accel_rpm_s, max_current_a });
  $("angleSlider").min = min_deg; $("angleSlider").max = max_deg;
  $("gaugeMin").textContent = `${min_deg}°`; $("gaugeMax").textContent = `${max_deg}°`;
  showToast(`${name}: limits & speed saved.`, "success");
});

$("btnSaveDirection").addEventListener("click", () => {
  const name = state.selectedJoint;
  const direction = parseInt($("dirSelect").value, 10);
  const jcfg = state.config.joints.find(j => j.name === name);
  const current = jcfg.direction ?? 1;
  if (direction === current) {
    showToast(`${name}: direction is already ${direction === 1 ? "normal" : "reversed"}.`, "info");
    return;
  }
  const s = state.joints[name];
  if (s && s.enabled) {
    showToast(`${name}: disable torque before reversing direction.`, "error");
    return;
  }
  if (!confirm(`Reverse direction for '${name}' to ${direction === 1 ? "Normal" : "Reversed"}?\n\nTorque must stay OFF until this is applied. Re-enable afterward so the target re-seeds under the new convention.`)) {
    return;
  }
  send({ type: "set_direction", joint: name, direction });
  jcfg.direction = direction;
  showToast(`${name}: direction set to ${direction === 1 ? "normal" : "reversed"} -- check Event Log to confirm the motor accepted it.`, "info");
});

$("btnSaveGains").addEventListener("click", () => {
  const name = state.selectedJoint;
  const pos_kp = parseFloat($("posKp").value);
  const pos_ki = parseFloat($("posKi").value);
  const vel_kp = parseFloat($("velKp").value);
  const vel_ki = parseFloat($("velKi").value);
  if ([pos_kp, pos_ki, vel_kp, vel_ki].some(Number.isNaN)) {
    showToast("Gains: enter numbers in every field.", "error"); return;
  }
  if ([pos_kp, pos_ki, vel_kp, vel_ki].some(v => v < 0)) {
    showToast("Gains can't be negative.", "error"); return;
  }
  send({ type: "set_position_gains", joint: name, kp: pos_kp, ki: pos_ki });
  send({ type: "set_velocity_gains", joint: name, kp: vel_kp, ki: vel_ki });
  const jcfg = state.config.joints.find(j => j.name === name);
  Object.assign(jcfg, { pos_kp, pos_ki, vel_kp, vel_ki });
  showToast(`${name}: gains sent -- check Event Log to confirm the driver accepted them.`, "info");
});

// jog: hold to move, release/leave to stop
function bindJog(btnId, direction) {
  const btn = $(btnId);
  const start = () => send({ type: "jog_start", joint: state.selectedJoint, direction });
  const stop = () => send({ type: "jog_stop", joint: state.selectedJoint });
  btn.addEventListener("mousedown", start);
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); start(); });
  ["mouseup", "mouseleave", "touchend", "touchcancel"].forEach(evt => btn.addEventListener(evt, stop));
}
bindJog("jogNeg", -1);
bindJog("jogPos", 1);

// ------------------------------------------------------------- wrist ----
function populateWristConfigUI() {
  const wd = state.config.wrist_diff || {};
  const motorA = $("wristMotorA");
  const motorB = $("wristMotorB");
  motorA.innerHTML = "";
  motorB.innerHTML = "";
  for (const j of state.config.joints) {
    const optA = document.createElement("option");
    optA.value = j.name; optA.textContent = j.name;
    motorA.appendChild(optA);
    const optB = document.createElement("option");
    optB.value = j.name; optB.textContent = j.name;
    motorB.appendChild(optB);
  }
  $("wristEnabled").checked = !!wd.enabled;
  if (wd.motor_a) motorA.value = wd.motor_a;
  if (wd.motor_b) motorB.value = wd.motor_b;
  $("wristPitchSignA").value = String(wd.pitch_sign_a ?? 1);
  $("wristRollSignA").value = String(wd.roll_sign_a ?? 1);
  $("wristPitchSignB").value = String(wd.pitch_sign_b ?? 1);
  $("wristRollSignB").value = String(wd.roll_sign_b ?? -1);
  $("wristMixRatio").value = wd.mix_ratio ?? 1.0;
  $("wristSwapPitchRoll").checked = !!wd.swap_pitch_roll;
  $("wristPitchMin").value = wd.pitch_min_deg ?? -90;
  $("wristPitchMax").value = wd.pitch_max_deg ?? 90;
  $("wristRollMin").value = wd.roll_min_deg ?? -180;
  $("wristRollMax").value = wd.roll_max_deg ?? 180;
}

function renderWristStatus(msg) {
  $("wristCurPitch").textContent = msg.pitch != null ? `${msg.pitch.toFixed(2)}°` : "--";
  $("wristCurRoll").textContent = msg.roll != null ? `${msg.roll.toFixed(2)}°` : "--";
  $("wristTgtPitch").textContent = msg.target_pitch != null ? `${msg.target_pitch.toFixed(2)}°` : "--";
  $("wristTgtRoll").textContent = msg.target_roll != null ? `${msg.target_roll.toFixed(2)}°` : "--";
}

$("btnWristGo").addEventListener("click", () => {
  const pitch_deg = parseFloat($("wristPitchInput").value);
  const roll_deg = parseFloat($("wristRollInput").value);
  if (Number.isNaN(pitch_deg) || Number.isNaN(roll_deg)) {
    showToast("Wrist: enter numbers for both pitch and roll.", "error"); return;
  }
  if (!state.config.wrist_diff || !state.config.wrist_diff.enabled) {
    showToast("Wrist: differential wrist isn't enabled -- save the Mixing Config first.", "error"); return;
  }
  send({ type: "wrist_go", pitch_deg, roll_deg, ramped: $("wristRampedToggle").checked });
});

$("btnWristSaveConfig").addEventListener("click", () => {
  const enabled = $("wristEnabled").checked;
  const motor_a = $("wristMotorA").value;
  const motor_b = $("wristMotorB").value;
  const pitch_sign_a = parseInt($("wristPitchSignA").value, 10);
  const roll_sign_a = parseInt($("wristRollSignA").value, 10);
  const pitch_sign_b = parseInt($("wristPitchSignB").value, 10);
  const roll_sign_b = parseInt($("wristRollSignB").value, 10);
  const mix_ratio = parseFloat($("wristMixRatio").value);
  const swap_pitch_roll = $("wristSwapPitchRoll").checked;
  const pitch_min_deg = parseFloat($("wristPitchMin").value);
  const pitch_max_deg = parseFloat($("wristPitchMax").value);
  const roll_min_deg = parseFloat($("wristRollMin").value);
  const roll_max_deg = parseFloat($("wristRollMax").value);
  if (motor_a === motor_b) { showToast("Wrist: Motor A and Motor B must be different joints.", "error"); return; }
  if (Number.isNaN(mix_ratio) || mix_ratio === 0) { showToast("Wrist: mix ratio must be a non-zero number.", "error"); return; }
  if ([pitch_min_deg, pitch_max_deg, roll_min_deg, roll_max_deg].some(Number.isNaN)) {
    showToast("Wrist: enter numbers for all pitch/roll limits.", "error"); return;
  }
  if (pitch_min_deg >= pitch_max_deg) { showToast("Wrist: pitch min must be less than pitch max.", "error"); return; }
  if (roll_min_deg >= roll_max_deg) { showToast("Wrist: roll min must be less than roll max.", "error"); return; }
  const wrist_diff = {
    enabled, motor_a, motor_b, pitch_sign_a, roll_sign_a, pitch_sign_b, roll_sign_b, mix_ratio,
    swap_pitch_roll, pitch_min_deg, pitch_max_deg, roll_min_deg, roll_max_deg,
  };
  send({ type: "set_wrist_diff_config", wrist_diff });
  state.config.wrist_diff = wrist_diff;
  showToast("Wrist config sent -- check Event Log to confirm it was accepted.", "info");
});

// ------------------------------------------------------------- batch ----
function selectedBatch() {
  return Array.from(document.querySelectorAll(".batch-cb:checked")).map(cb => cb.value);
}
$("batchEnable").addEventListener("click", () => {
  const joints = selectedBatch();
  if (joints.length) send({ type: "enable_batch", joints });
});
$("batchDisable").addEventListener("click", () => {
  const joints = selectedBatch();
  if (joints.length) send({ type: "disable_batch", joints });
});
$("batchClearFault").addEventListener("click", () => {
  const joints = selectedBatch();
  if (joints.length) send({ type: "clear_fault_batch", joints });
});
$("batchSelectAll").addEventListener("click", () => {
  document.querySelectorAll(".batch-cb").forEach(cb => { cb.checked = true; cb.dispatchEvent(new Event("change")); });
});
$("batchSelectNone").addEventListener("click", () => {
  document.querySelectorAll(".batch-cb").forEach(cb => { cb.checked = false; cb.dispatchEvent(new Event("change")); });
});
$("batchSetHome").addEventListener("click", () => {
  const joints = selectedBatch();
  if (!joints.length) { showToast("Batch: check at least one joint.", "error"); return; }
  if (!confirm(`Set HOME here for ${joints.length} joint(s): ${joints.join(", ")}?\n\nEach joint should be OFF torque and physically at its intended zero pose. This is a PERMANENT write to each motor.`)) return;
  send({ type: "set_home_batch", joints });
});
$("batchFill").addEventListener("click", () => {
  const deg = parseFloat($("batchAngle").value);
  if (Number.isNaN(deg)) return;
  document.querySelectorAll(".batch-cb:checked").forEach(cb => {
    const input = document.querySelector(`.batch-angle[data-joint="${cb.value}"]`);
    if (input) input.value = deg;
  });
});
$("batchGo").addEventListener("click", () => {
  const joints = selectedBatch();
  if (!joints.length) { showToast("Batch: check at least one joint.", "error"); return; }
  const targets = {};
  const bad = [];
  joints.forEach(n => {
    const input = document.querySelector(`.batch-angle[data-joint="${n}"]`);
    const deg = parseFloat(input ? input.value : NaN);
    if (Number.isNaN(deg)) { bad.push(n); return; }
    targets[n] = deg;
  });
  if (bad.length) { showToast(`Batch: enter a valid angle for ${bad.join(", ")}.`, "error"); return; }
  send({ type: "go_to_angle_batch", targets });
});

// ---------------------------------------------------------- discovery ----
function renderDiscovery() {
  const tbody = document.querySelector("#discoveryTable tbody");
  tbody.innerHTML = "";
  $("discoveryEmpty").classList.toggle("hidden", state.discovered.length > 0);
  for (const d of state.discovered) {
    const tr = document.createElement("tr");
    const age = ((Date.now() / 1000) - d.last_seen).toFixed(0);
    tr.innerHTML = `
      <td>${d.addr}</td>
      <td>${d.protocol != null ? "0x" + d.protocol.toString(16).toUpperCase() : "?"}</td>
      <td>${d.app_fw != null ? d.app_fw : "?"}</td>
      <td>${age}s ago</td>
      <td><button class="btn btn-secondary" data-addr="${d.addr}">Add as joint</button></td>
    `;
    tr.querySelector("button").addEventListener("click", () => openAddJointForm(d.addr));
    tbody.appendChild(tr);
  }
}

function openAddJointForm(addr) {
  $("newJointAddr").value = addr;
  $("addJointForm").classList.remove("hidden");
}
$("btnCancelAddJoint").addEventListener("click", () => $("addJointForm").classList.add("hidden"));
$("btnAddJoint").addEventListener("click", () => {
  const joint_def = {
    name: $("newJointName").value.trim(),
    addr: parseInt($("newJointAddr").value, 10),
    direction: parseInt($("newJointDir").value, 10),
    min_deg: parseFloat($("newJointMin").value),
    max_deg: parseFloat($("newJointMax").value),
    max_speed_rpm: parseFloat($("newJointSpeed").value),
    max_accel_rpm_s: parseFloat($("newJointAccel").value),
    max_current_a: parseFloat($("newJointCurrent").value),
  };
  if (!joint_def.name) { showToast("Name is required.", "error"); return; }
  send({ type: "add_joint", joint_def });
  $("addJointForm").classList.add("hidden");
  showToast(`Joint '${joint_def.name}' added.`, "success");
});

$("btnScan").addEventListener("click", () => fetch("/api/scan", { method: "POST" }));
$("btnReconnect").addEventListener("click", () => { send({ type: "reconnect" }); showToast("Reconnect requested.", "info"); });

// ------------------------------------------------------------- E-STOP ----
$("btnEstop").addEventListener("click", () => {
  send({ type: "estop" });
  fetch("/api/estop", { method: "POST" }); // belt-and-braces in case the ws happened to be mid-reconnect
});

// -------------------------------------------------------- CAN diagnostics --
async function refreshCanStatus() {
  try {
    const r = await fetch("/api/can/status");
    const d = await r.json();
    if (!d.ok) {
      $("canState").textContent = "error"; $("canState").style.color = "var(--err)";
      $("canBusOff").textContent = "--"; $("canErrPassive").textContent = "--"; $("canErrCounts").textContent = "-- / --";
      showToast(`CAN status: ${d.error}`, "error");
      return;
    }
    $("canState").textContent = d.state || "?";
    $("canState").style.color = d.state === "UP" ? "var(--ok)" : "var(--err)";
    $("canBusOff").textContent = d.bus_off ? "YES" : "no";
    $("canErrPassive").textContent = d.error_passive ? "YES" : "no";
    $("canErrCounts").textContent = `${d.rx_errors ?? "--"} / ${d.tx_errors ?? "--"}`;
  } catch (e) {
    showToast(`CAN status request failed: ${e}`, "error");
  }
}
$("btnCanRefresh").addEventListener("click", refreshCanStatus);

$("btnCanUp").addEventListener("click", async () => {
  try {
    const r = await fetch("/api/can/up", { method: "POST" });
    const d = await r.json();
    showToast(d.ok ? d.message : `CAN up failed: ${d.error}`, d.ok ? "success" : "error");
    refreshCanStatus();
  } catch (e) { showToast(`CAN up failed: ${e}`, "error"); }
});

$("btnCanDown").addEventListener("click", async () => {
  if (!confirm("Bring the CAN link DOWN?\n\nThis first triggers E-STOP, then cuts CAN traffic at the OS level entirely. Nothing will move or respond until you bring it back up.")) return;
  try {
    const r = await fetch("/api/can/down", { method: "POST" });
    const d = await r.json();
    showToast(d.ok ? d.message : `CAN down failed: ${d.error}`, d.ok ? "warning" : "error");
    refreshCanStatus();
  } catch (e) { showToast(`CAN down failed: ${e}`, "error"); }
});

$("btnCandump").addEventListener("click", async () => {
  const duration = $("candumpDuration").value;
  const out = $("candumpOutput");
  out.textContent = `Capturing for ${duration}s...`;
  try {
    const r = await fetch(`/api/can/candump?duration=${duration}`);
    const d = await r.json();
    if (!d.ok) { out.textContent = `Error: ${d.error}`; return; }
    out.textContent = d.lines.length ? d.lines.join("\n") : "No frames captured -- bus may be idle or down.";
  } catch (e) {
    out.textContent = `Request failed: ${e}`;
  }
});

// --------------------------------------------------------------- log ----
function appendLog(entry) {
  const levelOk = LEVELS.indexOf(entry.level) >= LEVELS.indexOf($("logLevelFilter").value);
  const jointFilter = $("logJointFilter").value;
  const jointOk = !jointFilter || entry.joint === jointFilter;
  const view = $("logView");
  const line = document.createElement("div");
  line.className = `log-line ${entry.level}`;
  line.dataset.level = entry.level;
  line.dataset.joint = entry.joint || "";
  const ts = new Date(entry.ts * 1000).toLocaleTimeString();
  line.innerHTML = `<span class="ts">${ts}</span><span class="lvl">${entry.level}</span>${entry.joint ? `<span class="jt">[${entry.joint}]</span>` : ""}${escapeHtml(entry.message)}`;
  line.style.display = (levelOk && jointOk) ? "" : "none";
  view.appendChild(line);
  while (view.children.length > 1000) view.removeChild(view.firstChild);
  view.scrollTop = view.scrollHeight;
  if (entry.level === "ERROR") showToast(entry.message, "error");
}

function escapeHtml(s) {
  return s.replace(/[&<>]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
}

function reapplyLogFilter() {
  const levelMin = LEVELS.indexOf($("logLevelFilter").value);
  const jointFilter = $("logJointFilter").value;
  for (const line of $("logView").children) {
    const ok = LEVELS.indexOf(line.dataset.level) >= levelMin && (!jointFilter || line.dataset.joint === jointFilter);
    line.style.display = ok ? "" : "none";
  }
}
$("logLevelFilter").addEventListener("change", reapplyLogFilter);
$("logJointFilter").addEventListener("change", reapplyLogFilter);
$("btnClearLogView").addEventListener("click", () => { $("logView").innerHTML = ""; });

initTabs();
connect();
