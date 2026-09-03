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
  wristPitchActual: null,
  wristRollActual: null,
  gamepad: { index: null, enabled: false, prevButtons: [] },
  vizView: "front",  // cockpit Arm Position view: front | top | wrist
  sim: {
    enabled: false,
    initialized: false,
    angles: { base: 0, shoulder: 0, elbow: 0, gripper: 0 },
    wristPitch: 0,
    wristRoll: 0,
  },
  simPrevButtons: [],
  gamepadBindings: null,   // populated by loadGamepadBindings() below
  listeningFor: null,      // action id currently being captured in Settings
  listenBaseline: null,
  listenBtnEl: null,
  listenStarted: 0,
};

// Standard joint names from gui_joints.default.json -- the Cockpit tab
// (visualization, quick controls, gamepad mapping) is written against
// these. Everything checks hasJoint() first, so renaming/removing a
// joint via the Discovery tab degrades gracefully instead of throwing.
const JOINT_BASE = "base", JOINT_SHOULDER = "shoulder", JOINT_ELBOW = "elbow",
      JOINT_WRIST_L = "wrist_l", JOINT_WRIST_R = "wrist_r", JOINT_GRIPPER = "gripper";
function hasJoint(name) { return !!(state.config && state.config.joints.some(j => j.name === name)); }
function getJointCfg(name) {
  return (state.config && state.config.joints.find(j => j.name === name)) || { min_deg: -90, max_deg: 90 };
}
function normRange(val, lo, hi, outLo, outHi) {
  if (val == null || lo == null || hi == null || hi === lo) return (outLo + outHi) / 2;
  const t = Math.max(0, Math.min(1, (val - lo) / (hi - lo)));
  return outLo + t * (outHi - outLo);
}

// Customizable jog step, ported from diff_wrist.py's JOG_STEPS_DEG /
// bracket-key cycling -- the old jog was a single fixed-speed hold with
// no way to dial in a precise increment, which is the "jog is confusing
// and non-customizable" complaint. One shared step applies to whichever
// joint/axis is currently selected (arm joint on the Control tab, or
// pitch/roll on the Wrist tab).
const JOG_STEPS = [0.1, 0.5, 1, 2, 5, 10, 30];
let jogStep = 1;
let jogStepIndex = JOG_STEPS.indexOf(1);

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
  if (name === "simulation") renderSimViz();
  if (name !== "settings" && state.listeningFor) cancelListening();
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
      buildCockpitGrid();
      buildGamepadLegend();
      renderArmViz();
      initSimDefaults();
      renderSimViz();
      $("canIfaceName").textContent = msg.config.can_interface || "can0";
      $("canBitrate").textContent = msg.config.can_bitrate ? `${msg.config.can_bitrate} bps` : "--";
      break;
    case "status":
      state.joints[msg.name] = msg;
      renderJointCard(msg);
      if (msg.name === state.selectedJoint) renderControlPanel(msg);
      updateBusBanner();
      updateCockpitCard(msg);
      updateWizardState();
      maybeRenderViz();
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

function buildCockpitGrid() {
  const grid = $("cockpitJointGrid");
  if (!grid || !state.config) return;
  grid.innerHTML = "";
  for (const j of state.config.joints) {
    const card = document.createElement("div");
    card.className = "cc-card";
    card.id = `cc-${j.name}`;
    card.innerHTML = `
      <div class="cc-head">
        <span class="dot dot-red" id="cc-dot-${j.name}"></span>
        <span class="cc-name">${j.name}</span>
        <span class="cc-angle" id="cc-angle-${j.name}">--°</span>
      </div>
      <div class="cc-torque-row">
        <button class="btn btn-primary cc-torque-btn" id="cc-torque-${j.name}">Enable</button>
      </div>
      <div class="jog-buttons">
        <button class="btn btn-jog" id="cc-jogneg-${j.name}" title="Hold to jog">&minus;</button>
        <button class="btn btn-jog" id="cc-jogpos-${j.name}" title="Hold to jog">+</button>
      </div>
    `;
    grid.appendChild(card);
    bindJogGeneric(`cc-jogneg-${j.name}`, j.name, -1);
    bindJogGeneric(`cc-jogpos-${j.name}`, j.name, 1);
    $(`cc-torque-${j.name}`).addEventListener("click", () => toggleJointTorque(j.name));
  }
}

function bindJogGeneric(btnId, jointName, direction) {
  const btn = $(btnId);
  if (!btn) return;
  const start = () => send({ type: "jog_start", joint: jointName, direction });
  const stop = () => send({ type: "jog_stop", joint: jointName });
  btn.addEventListener("mousedown", start);
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); start(); });
  ["mouseup", "mouseleave", "touchend", "touchcancel"].forEach(evt => btn.addEventListener(evt, stop));
}

function toggleJointTorque(name) {
  if (!hasJoint(name)) return;
  const s = state.joints[name];
  send({ type: s && s.enabled ? "disable" : "enable", joint: name });
}

function updateCockpitCard(s) {
  const dot = $(`cc-dot-${s.name}`);
  if (dot) dot.className = "dot " + (s.connected ? "dot-green" : "dot-red");
  const angle = $(`cc-angle-${s.name}`);
  if (angle) angle.textContent = s.connected ? `${s.angle.toFixed(1)}°` : "--°";
  const torqueBtn = $(`cc-torque-${s.name}`);
  if (torqueBtn) {
    torqueBtn.textContent = s.enabled ? "Disable" : "Enable";
    torqueBtn.className = "btn cc-torque-btn " + (s.enabled ? "btn-secondary" : "btn-primary");
  }
}

// ----------------------------------------------------------- wizard ----
function updateWizardState() {
  if (!state.config || !state.config.joints.length) return;
  const names = state.config.joints.map(j => j.name);
  const allConnected = names.every(n => state.joints[n] && state.joints[n].connected);
  const anyConnected = names.some(n => state.joints[n] && state.joints[n].connected);
  const allEnabled = names.every(n => state.joints[n] && state.joints[n].enabled);

  $("wizConnText").textContent = allConnected
    ? `All ${names.length} joints responding.`
    : anyConnected
      ? `Some joints responding -- check wiring/power for the rest.`
      : `Waiting for motors to respond…`;
  $("wizStep1").classList.toggle("wizard-step-done", allConnected);

  if (allEnabled) {
    $("wizardBody").classList.add("hidden");
    $("wizardCollapsed").classList.remove("hidden");
  } else {
    $("wizardBody").classList.remove("hidden");
    $("wizardCollapsed").classList.add("hidden");
  }
}

$("btnWizHome").addEventListener("click", () => {
  if (!state.config) return;
  const names = state.config.joints.map(j => j.name);
  if (!confirm(`Set HOME here for ALL ${names.length} joints: ${names.join(", ")}?\n\nEach joint should be OFF torque and physically at its intended zero pose. This is a PERMANENT write to each motor.`)) return;
  send({ type: "set_home_batch", joints: names });
  showToast("Home position set for all joints.", "success");
});
$("btnWizSkipHome").addEventListener("click", () => {
  showToast("Skipped homing -- using each motor's last stored zero.", "info");
});
$("btnWizEnable").addEventListener("click", () => {
  if (!state.config) return;
  const names = state.config.joints.map(j => j.name);
  send({ type: "enable_batch", joints: names });
  showToast("Enabling torque on all joints…", "info");
});
$("btnWizardToggle").addEventListener("click", () => {
  $("wizardBody").classList.toggle("hidden");
});
$("btnWizardReopen").addEventListener("click", () => {
  $("wizardBody").classList.remove("hidden");
  $("wizardCollapsed").classList.add("hidden");
});

// ------------------------------------------------------ arm visualization --
// Schematic, not a literal kinematic replica: each link's on-screen angle
// is the joint's live angle normalized into its own configured min/max
// range, mapped to a fixed on-screen sweep. That keeps the picture legible
// regardless of a joint's real (and possibly asymmetric, e.g. 0..165 deg)
// travel, and needs no hardware-verified sign convention to be useful --
// it's there so an operator can tell "mostly folded" from "reaching out",
// not to reproduce true arm geometry.
const VIZ = { upper: 66, fore: 74, wrist: 22, grip: 20, originX: 110, originY: 210 };

function polarPt(origin, angleDeg, length) {
  const rad = (angleDeg - 90) * Math.PI / 180;
  return { x: origin.x + length * Math.cos(rad), y: origin.y + length * Math.sin(rad) };
}

let lastVizRender = 0;
function maybeRenderViz() {
  const now = performance.now();
  if (now - lastVizRender < 60) return;
  lastVizRender = now;
  renderArmViz();
}

// computeArmPose() turns either live joint telemetry or the local
// simulation state into the same normalized-angle "pose" shape that all
// three views draw from. Angles are normalized into each joint's own
// configured min/max (see VIZ comment above) -- true for both live and
// simulated poses, since the sim is clamped to the same joint configs.
function computeArmPose(useSim) {
  const shCfg = getJointCfg(JOINT_SHOULDER), elCfg = getJointCfg(JOINT_ELBOW), gripCfg = getJointCfg(JOINT_GRIPPER);
  const wd = (state.config && state.config.wrist_diff) || {};
  const pitchMin = wd.pitch_min_deg ?? -90, pitchMax = wd.pitch_max_deg ?? 90;

  let shRaw, elRaw, baseRaw, gripRaw, wristPRaw, wristRRaw, stale, wristKnown;

  if (useSim) {
    shRaw = state.sim.angles.shoulder;
    elRaw = state.sim.angles.elbow;
    baseRaw = state.sim.angles.base;
    gripRaw = state.sim.angles.gripper;
    wristPRaw = state.sim.wristPitch;
    wristRRaw = state.sim.wristRoll;
    wristKnown = true;
    stale = false;
  } else {
    const sh = state.joints[JOINT_SHOULDER], el = state.joints[JOINT_ELBOW];
    const base = state.joints[JOINT_BASE], grip = state.joints[JOINT_GRIPPER];
    shRaw = (sh && sh.connected) ? sh.angle : null;
    elRaw = (el && el.connected) ? el.angle : null;
    baseRaw = (base && base.connected) ? base.angle : null;
    gripRaw = (grip && grip.connected) ? grip.angle : null;
    wristKnown = !!(wd.enabled && state.wristPitchActual != null);
    wristPRaw = wristKnown ? state.wristPitchActual : null;
    wristRRaw = (wd.enabled && state.wristRollActual != null) ? state.wristRollActual : null;
    stale = !(sh && sh.connected && el && el.connected);
  }

  const shAngle = shRaw != null ? normRange(shRaw, shCfg.min_deg, shCfg.max_deg, -75, 75) : 0;
  const elAngle = elRaw != null ? normRange(elRaw, elCfg.min_deg, elCfg.max_deg, -75, 75) : 0;
  const wristPitchAngle = wristPRaw != null ? normRange(wristPRaw, pitchMin, pitchMax, -60, 60) : 0;
  const gripFrac = (gripRaw != null && gripCfg.max_deg !== gripCfg.min_deg)
    ? Math.max(0, Math.min(1, (gripRaw - gripCfg.min_deg) / (gripCfg.max_deg - gripCfg.min_deg))) : 0;

  return {
    shAngle, elAngle, wristPitchAngle, gripFrac, stale, wristKnown,
    baseRollDeg: baseRaw, wristRollDeg: wristRRaw,
    baseRaw, shRaw, elRaw, gripRaw, wristPRaw, wristRRaw,
  };
}

function renderArmViz() {
  const svg = $("armSvg");
  if (!svg || !state.config) return;
  const pose = computeArmPose(false);
  drawView(svg, pose, state.vizView || "front");
  writeReadouts("viz", pose);
}

function renderSimViz() {
  const front = $("simSvgFront"), top = $("simSvgTop"), wrist = $("simSvgWrist");
  if (!front || !state.config) return;
  const pose = computeArmPose(true);
  drawView(front, pose, "front");
  drawView(top, pose, "top");
  drawView(wrist, pose, "wrist");
  writeReadouts("sim", pose);
}

function writeReadouts(prefix, pose) {
  const grip = $(`${prefix}GripReadout`);
  const gripKnown = pose.gripRaw != null || prefix === "sim";
  $(`${prefix}BaseReadout`).textContent = pose.baseRaw != null ? `${pose.baseRaw.toFixed(1)}°` : "--";
  $(`${prefix}ShoulderReadout`).textContent = pose.shRaw != null ? `${pose.shRaw.toFixed(1)}°` : "--";
  $(`${prefix}ElbowReadout`).textContent = pose.elRaw != null ? `${pose.elRaw.toFixed(1)}°` : "--";
  $(`${prefix}WristPReadout`).textContent = pose.wristKnown && pose.wristPRaw != null ? `${pose.wristPRaw.toFixed(1)}°` : (prefix === "sim" ? `${pose.wristPRaw.toFixed(1)}°` : "n/a");
  $(`${prefix}WristRReadout`).textContent = pose.wristKnown && pose.wristRRaw != null ? `${pose.wristRRaw.toFixed(1)}°` : (prefix === "sim" ? `${pose.wristRRaw.toFixed(1)}°` : "n/a");
  if (grip) grip.textContent = gripKnown ? `${Math.round(pose.gripFrac * 100)}% open` : "--";
}

function drawView(svgEl, pose, view) {
  if (view === "top") drawTopView(svgEl, pose);
  else if (view === "wrist") drawWristView(svgEl, pose);
  else drawFrontView(svgEl, pose);
}

// Front elevation -- the original single view. Shoulder/elbow/wrist-pitch
// unfold as a 2D chain; base roll only spins the turret icon (it isn't
// meant to reposition the chain -- see VIZ comment); wrist roll spins the
// gripper jaws in place.
function drawFrontView(svgEl, pose) {
  const { shAngle, elAngle, wristPitchAngle, gripFrac, stale } = pose;
  const baseRollDeg = pose.baseRollDeg, wristRollDeg = pose.wristRollDeg;
  const origin = { x: VIZ.originX, y: VIZ.originY };
  let cum = shAngle;
  const p1 = polarPt(origin, cum, VIZ.upper);
  cum += elAngle;
  const p2 = polarPt(p1, cum, VIZ.fore);
  cum += wristPitchAngle;
  const p3 = polarPt(p2, cum, VIZ.wrist);
  const jawSpread = 8 + gripFrac * 22;
  const jawTipL = polarPt(p3, cum - jawSpread, VIZ.grip);
  const jawTipR = polarPt(p3, cum + jawSpread, VIZ.grip);

  const turretRotate = baseRollDeg != null ? baseRollDeg : 0;
  const rollRotate = wristRollDeg != null ? wristRollDeg : 0;

  svgEl.setAttribute("viewBox", "0 0 260 240");
  svgEl.innerHTML = `
    <g class="${stale ? "viz-stale" : ""}">
      <ellipse cx="${origin.x}" cy="${origin.y + 6}" rx="30" ry="9" class="viz-base-shadow"/>
      <g transform="rotate(${turretRotate} ${origin.x} ${origin.y})">
        <rect x="${origin.x - 22}" y="${origin.y - 6}" width="44" height="12" rx="4" class="viz-turret"/>
        <line x1="${origin.x}" y1="${origin.y}" x2="${origin.x}" y2="${origin.y - 18}" class="viz-turret-mark"/>
      </g>
      <circle cx="${origin.x}" cy="${origin.y}" r="7" class="viz-pivot"/>
      <line x1="${origin.x}" y1="${origin.y}" x2="${p1.x}" y2="${p1.y}" class="viz-link"/>
      <circle cx="${p1.x}" cy="${p1.y}" r="6" class="viz-pivot"/>
      <line x1="${p1.x}" y1="${p1.y}" x2="${p2.x}" y2="${p2.y}" class="viz-link"/>
      <circle cx="${p2.x}" cy="${p2.y}" r="5" class="viz-pivot"/>
      <line x1="${p2.x}" y1="${p2.y}" x2="${p3.x}" y2="${p3.y}" class="viz-link viz-link-wrist"/>
      <g transform="rotate(${rollRotate} ${p3.x} ${p3.y})">
        <line x1="${p3.x}" y1="${p3.y}" x2="${jawTipL.x}" y2="${jawTipL.y}" class="viz-jaw"/>
        <line x1="${p3.x}" y1="${p3.y}" x2="${jawTipR.x}" y2="${jawTipR.y}" class="viz-jaw"/>
      </g>
      <circle cx="${p3.x}" cy="${p3.y}" r="4" class="viz-pivot"/>
    </g>
  `;
}

// Top (bird's-eye) view -- makes base rotation directly legible (the
// front view only spins a small turret icon) and shows the gripper spread
// + roll from above. "Reach" is a schematic fold indicator, not a true
// projected length -- same "proportional, not literal" approach as the
// front view.
function drawTopView(svgEl, pose) {
  const { shAngle, elAngle, gripFrac, stale } = pose;
  const baseRollDeg = pose.baseRollDeg, wristRollDeg = pose.wristRollDeg;
  const cx = 130, cy = 130;
  const heading = baseRollDeg != null ? baseRollDeg : 0;
  const foldAmount = Math.min(1, (Math.abs(shAngle) + Math.abs(elAngle)) / 150);
  const reach = 34 + (1 - foldAmount) * 74;
  const origin = { x: cx, y: cy };
  const tip = polarPt(origin, heading, reach);
  const jawSpread = 8 + gripFrac * 22;
  const rollRotate = wristRollDeg != null ? wristRollDeg : 0;
  const jawTipL = polarPt(tip, heading - jawSpread, VIZ.grip);
  const jawTipR = polarPt(tip, heading + jawSpread, VIZ.grip);

  svgEl.setAttribute("viewBox", "0 0 260 260");
  svgEl.innerHTML = `
    <g class="${stale ? "viz-stale" : ""}">
      <text x="${cx}" y="16" class="viz-top-label" text-anchor="middle">FRONT (0°) ↑</text>
      <circle cx="${cx}" cy="${cy}" r="96" class="viz-top-ring"/>
      <line x1="${cx}" y1="${cy - 96}" x2="${cx}" y2="${cy - 88}" class="viz-turret-mark"/>
      <circle cx="${cx}" cy="${cy}" r="16" class="viz-turret"/>
      <line x1="${cx}" y1="${cy}" x2="${tip.x}" y2="${tip.y}" class="viz-link"/>
      <circle cx="${cx}" cy="${cy}" r="5" class="viz-pivot"/>
      <g transform="rotate(${rollRotate} ${tip.x} ${tip.y})">
        <line x1="${tip.x}" y1="${tip.y}" x2="${jawTipL.x}" y2="${jawTipL.y}" class="viz-jaw"/>
        <line x1="${tip.x}" y1="${tip.y}" x2="${jawTipR.x}" y2="${jawTipR.y}" class="viz-jaw"/>
      </g>
      <circle cx="${tip.x}" cy="${tip.y}" r="4" class="viz-pivot"/>
    </g>
  `;
}

// Wrist/gripper close-up -- the dedicated spot to read roll and gripper
// state at a glance: a dashed roll dial with a marker line at the current
// wrist roll angle, plus jaw lines that open with the gripper fraction and
// spin with the marker (they share the tool axis).
function drawWristView(svgEl, pose) {
  const { wristPitchAngle, gripFrac, stale } = pose;
  const wristRollDeg = pose.wristRollDeg;
  const cx = 130, cy = 130, dialR = 62;
  const stubOrigin = { x: cx - 90, y: cy };
  const pitchVisual = Math.max(-60, Math.min(60, wristPitchAngle));
  const stubTip = polarPt(stubOrigin, 90 + pitchVisual, 60);
  const jawSpread = 10 + gripFrac * 26;
  const rollRotate = wristRollDeg != null ? wristRollDeg : 0;
  const jawTipL = polarPt(stubTip, 90 - jawSpread, 34);
  const jawTipR = polarPt(stubTip, 90 + jawSpread, 34);
  const marker = polarPt(stubTip, rollRotate, dialR * 0.6);

  svgEl.setAttribute("viewBox", "0 0 260 260");
  svgEl.innerHTML = `
    <g class="${stale ? "viz-stale" : ""}">
      <line x1="${stubOrigin.x - 24}" y1="${stubOrigin.y}" x2="${stubOrigin.x}" y2="${stubOrigin.y}" class="viz-link-wrist"/>
      <line x1="${stubOrigin.x}" y1="${stubOrigin.y}" x2="${stubTip.x}" y2="${stubTip.y}" class="viz-link viz-link-wrist"/>
      <circle cx="${stubTip.x}" cy="${stubTip.y}" r="${dialR}" class="viz-roll-ring"/>
      <line x1="${stubTip.x}" y1="${stubTip.y}" x2="${marker.x}" y2="${marker.y}" class="viz-roll-marker"/>
      <line x1="${stubTip.x}" y1="${stubTip.y}" x2="${jawTipL.x}" y2="${jawTipL.y}" class="viz-jaw"/>
      <line x1="${stubTip.x}" y1="${stubTip.y}" x2="${jawTipR.x}" y2="${jawTipR.y}" class="viz-jaw"/>
      <circle cx="${stubTip.x}" cy="${stubTip.y}" r="5" class="viz-pivot"/>
      <text x="${cx}" y="248" class="viz-top-label" text-anchor="middle">roll ${wristRollDeg != null ? wristRollDeg.toFixed(0) + "°" : "n/a"} · grip ${Math.round(gripFrac * 100)}%</text>
    </g>
  `;
}

// ------------------------------------------------------------ simulation --
function initSimDefaults() {
  if (state.sim.initialized) return;
  const gripCfg = getJointCfg(JOINT_GRIPPER);
  state.sim.angles.gripper = gripCfg.min_deg ?? 0;
  state.sim.initialized = true;
}

function resetSimulation() {
  const gripCfg = getJointCfg(JOINT_GRIPPER);
  state.sim.angles.base = 0;
  state.sim.angles.shoulder = 0;
  state.sim.angles.elbow = 0;
  state.sim.angles.gripper = gripCfg.min_deg ?? 0;
  state.sim.wristPitch = 0;
  state.sim.wristRoll = 0;
  renderSimViz();
  showToast("Simulated pose reset.", "info");
}
$("btnSimReset").addEventListener("click", resetSimulation);

$("simGamepadToggle").addEventListener("change", (e) => {
  state.sim.enabled = e.target.checked;
  lastSimTick = null;
  showToast(
    state.sim.enabled
      ? "Simulation: gamepad now drives the preview only -- no commands are sent to the arm."
      : "Simulation stopped.",
    "info"
  );
});

let lastSimTick = null;
function stepSimulation(gp) {
  const now = performance.now();
  if (lastSimTick == null) { lastSimTick = now; return; }
  const dt = Math.min(0.1, (now - lastSimTick) / 1000);
  lastSimTick = now;

  const shCfg = getJointCfg(JOINT_SHOULDER), elCfg = getJointCfg(JOINT_ELBOW),
        baseCfg = getJointCfg(JOINT_BASE), gripCfg = getJointCfg(JOINT_GRIPPER);
  const wd = (state.config && state.config.wrist_diff) || {};
  const pitchMin = wd.pitch_min_deg ?? -90, pitchMax = wd.pitch_max_deg ?? 90;
  const rollMin = wd.roll_min_deg ?? -180, rollMax = wd.roll_max_deg ?? 180;
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
  const rate = gpRateScale(gp);
  const AXIS_RATE = 90, HOLD_RATE = 60;

  state.sim.angles.base = clamp(
    state.sim.angles.base + applyDeadzone(bindingAxisValue(gp, "axisBase")) * rate * AXIS_RATE * dt,
    baseCfg.min_deg ?? -180, baseCfg.max_deg ?? 180);
  state.sim.angles.shoulder = clamp(
    state.sim.angles.shoulder + applyDeadzone(bindingAxisValue(gp, "axisShoulder")) * rate * AXIS_RATE * dt,
    shCfg.min_deg, shCfg.max_deg);
  state.sim.angles.elbow = clamp(
    state.sim.angles.elbow + applyDeadzone(bindingAxisValue(gp, "axisElbow")) * rate * AXIS_RATE * dt,
    elCfg.min_deg, elCfg.max_deg);
  state.sim.wristRoll = clamp(
    state.sim.wristRoll + applyDeadzone(bindingAxisValue(gp, "axisWristRoll")) * rate * AXIS_RATE * dt,
    rollMin, rollMax);

  if (bindingButtonPressed(gp, "wristPitchPos")) state.sim.wristPitch = clamp(state.sim.wristPitch + HOLD_RATE * dt, pitchMin, pitchMax);
  if (bindingButtonPressed(gp, "wristPitchNeg")) state.sim.wristPitch = clamp(state.sim.wristPitch - HOLD_RATE * dt, pitchMin, pitchMax);
  if (bindingButtonPressed(gp, "wristRollPos")) state.sim.wristRoll = clamp(state.sim.wristRoll + HOLD_RATE * dt, rollMin, rollMax);
  if (bindingButtonPressed(gp, "wristRollNeg")) state.sim.wristRoll = clamp(state.sim.wristRoll - HOLD_RATE * dt, rollMin, rollMax);
  if (bindingButtonPressed(gp, "gripperOpen")) state.sim.angles.gripper = clamp(state.sim.angles.gripper + HOLD_RATE * dt, gripCfg.min_deg, gripCfg.max_deg);
  if (bindingButtonPressed(gp, "gripperClose")) state.sim.angles.gripper = clamp(state.sim.angles.gripper - HOLD_RATE * dt, gripCfg.min_deg, gripCfg.max_deg);

  const prev = state.simPrevButtons;
  const pressed = gp.buttons.map(b => b.pressed);
  const resetIdx = state.gamepadBindings.resetArm.index;
  if (pressed[resetIdx] && !prev[resetIdx]) resetSimulation();
  state.simPrevButtons = pressed;

  renderSimViz();
}

// ---------------------------------------------------------- gamepad ----
// Standard Gamepad API mapping (Xbox controller on Chrome/Edge) by
// default, but every axis/button below is just a lookup into
// state.gamepadBindings -- rebindable from the Settings tab, and used
// identically by the Cockpit's live gamepad control and the Simulation
// tab's local preview. LT/RT are triggers (exposed as buttons with an
// analog `.value`); everything else is read via `.pressed`.
const GP_DEADZONE = 0.15;

const DEFAULT_GAMEPAD_BINDINGS = {
  axisBase:       { type: "axis",   index: 0 },
  axisShoulder:   { type: "axis",   index: 1 },
  axisWristRoll:  { type: "axis",   index: 2 },
  axisElbow:      { type: "axis",   index: 3 },
  triggerSlow:    { type: "button", index: 6 },   // LT
  triggerBoost:   { type: "button", index: 7 },   // RT
  wristPitchNeg:  { type: "button", index: 4 },   // LB
  wristPitchPos:  { type: "button", index: 5 },   // RB
  wristRollNeg:   { type: "button", index: 14 },  // D-Pad Left
  wristRollPos:   { type: "button", index: 15 },  // D-Pad Right
  gripperOpen:    { type: "button", index: 12 },  // D-Pad Up
  gripperClose:   { type: "button", index: 13 },  // D-Pad Down
  gripperStop:    { type: "button", index: 10 },  // L3
  wristRollStop:  { type: "button", index: 11 },  // R3
  toggleBase:     { type: "button", index: 1 },   // B
  toggleShoulder: { type: "button", index: 2 },   // X
  toggleElbow:    { type: "button", index: 3 },   // Y
  toggleGripper:  { type: "button", index: 0 },   // A
  toggleWrist:    { type: "button", index: 8 },   // View / Back
  resetArm:       { type: "button", index: 9 },   // Menu / Start
};

const BINDING_LABELS = {
  axisBase: "Base roll (stick axis)",
  axisShoulder: "Shoulder (stick axis)",
  axisWristRoll: "Wrist roll jog (stick axis)",
  axisElbow: "Elbow (stick axis)",
  triggerSlow: "Jog rate precision-slow (trigger)",
  triggerBoost: "Jog rate boost (trigger)",
  wristPitchNeg: "Wrist pitch \u2212 (hold)",
  wristPitchPos: "Wrist pitch + (hold)",
  wristRollNeg: "Wrist roll \u2212 (hold)",
  wristRollPos: "Wrist roll + (hold)",
  gripperOpen: "Gripper open (hold)",
  gripperClose: "Gripper close (hold)",
  gripperStop: "Gripper stop",
  wristRollStop: "Wrist roll stop",
  toggleBase: "Toggle base torque",
  toggleShoulder: "Toggle shoulder torque",
  toggleElbow: "Toggle elbow torque",
  toggleGripper: "Toggle gripper torque",
  toggleWrist: "Toggle wrist torque",
  resetArm: "Reset arm (torque off, all joints) / Reset simulated pose",
};

const ACTION_ORDER = [
  "axisBase", "axisShoulder", "axisElbow", "axisWristRoll",
  "triggerBoost", "triggerSlow",
  "wristPitchNeg", "wristPitchPos", "wristRollNeg", "wristRollPos",
  "gripperOpen", "gripperClose", "gripperStop", "wristRollStop",
  "toggleBase", "toggleShoulder", "toggleElbow", "toggleGripper", "toggleWrist", "resetArm",
];

function loadGamepadBindings() {
  let stored = {};
  try { stored = JSON.parse(localStorage.getItem("armGamepadBindings") || "{}"); } catch (e) { stored = {}; }
  const merged = {};
  for (const id of ACTION_ORDER) {
    merged[id] = Object.assign({}, DEFAULT_GAMEPAD_BINDINGS[id], stored[id] || {});
  }
  return merged;
}
function saveGamepadBindings() {
  try { localStorage.setItem("armGamepadBindings", JSON.stringify(state.gamepadBindings)); } catch (e) { /* storage unavailable -- bindings still work for this session */ }
}
state.gamepadBindings = loadGamepadBindings();

function bindingText(b) {
  if (!b) return "--";
  return b.type === "axis" ? `Axis ${b.index}${b.invert ? " (inverted)" : ""}` : `Button ${b.index}`;
}
function bindingButtonPressed(gp, actionId) {
  const b = state.gamepadBindings[actionId];
  const btn = b && gp.buttons[b.index];
  return !!(btn && (btn.pressed || btn.value > 0.5));
}
function bindingTriggerValue(gp, actionId) {
  const b = state.gamepadBindings[actionId];
  const btn = b && gp.buttons[b.index];
  return btn ? btn.value : 0;
}
function bindingAxisValue(gp, actionId) {
  const b = state.gamepadBindings[actionId];
  if (!b) return 0;
  let v = gp.axes[b.index] || 0;
  if (b.invert) v = -v;
  return v;
}

function buildGamepadLegend() {
  const wrap = $("gamepadLegend");
  if (!wrap) return;
  wrap.innerHTML = "";
  for (const id of ACTION_ORDER) {
    const el = document.createElement("div");
    el.className = "gp-item";
    el.dataset.action = id;
    el.innerHTML = `<div class="gp-item-key">${bindingText(state.gamepadBindings[id])}</div><div class="gp-item-desc">${BINDING_LABELS[id]}</div>`;
    wrap.appendChild(el);
  }
}

function setupGamepadEvents() {
  window.addEventListener("gamepadconnected", (e) => {
    state.gamepad.index = e.gamepad.index;
    updateGamepadStatusUI(e.gamepad);
    showToast(`Gamepad connected: ${e.gamepad.id}`, "success");
  });
  window.addEventListener("gamepaddisconnected", (e) => {
    if (state.gamepad.index === e.gamepad.index) {
      state.gamepad.index = null;
      updateGamepadStatusUI(null);
      showToast("Gamepad disconnected.", "warning");
    }
  });
  const existing = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const gp of existing) {
    if (gp) { state.gamepad.index = gp.index; updateGamepadStatusUI(gp); break; }
  }
}

function updateGamepadStatusUI(gp) {
  for (const id of ["gamepadStatus", "simGamepadStatus"]) {
    const el = $(id);
    if (!el) continue;
    if (gp) {
      el.textContent = `Connected: ${gp.id}`;
      el.classList.add("gp-connected");
    } else {
      el.textContent = "No controller detected -- plug in a controller and press any button.";
      el.classList.remove("gp-connected");
    }
  }
}

$("gamepadEnableToggle").addEventListener("change", (e) => {
  state.gamepad.enabled = e.target.checked;
  if (state.gamepad.enabled) {
    showToast("Gamepad control enabled.", "info");
  } else {
    allJogStop();
    showToast("Gamepad control disabled.", "info");
  }
});

function allJogStop() {
  for (const n of [JOINT_BASE, JOINT_SHOULDER, JOINT_ELBOW, JOINT_WRIST_L, JOINT_WRIST_R, JOINT_GRIPPER]) {
    if (hasJoint(n)) send({ type: "jog_stop", joint: n });
  }
}

function applyDeadzone(v) { return Math.abs(v) < GP_DEADZONE ? 0 : v; }

function gpRateScale(gp) {
  const rt = bindingTriggerValue(gp, "triggerBoost");
  const lt = bindingTriggerValue(gp, "triggerSlow");
  return Math.max(0.08, Math.min(1, 0.35 + 0.65 * rt - 0.25 * lt));
}

function sendAnalogJog(name, value) {
  if (!hasJoint(name)) return;
  send({ type: "jog_analog", joint: name, value });
}

function wristAxisJogAnalog(axis, value) {
  const wd = state.config && state.config.wrist_diff;
  if (wd && wd.enabled) {
    const signA = axis === "pitch" ? wd.pitch_sign_a : wd.roll_sign_a;
    const signB = axis === "pitch" ? wd.pitch_sign_b : wd.roll_sign_b;
    sendAnalogJog(wd.motor_a, value * signA);
    sendAnalogJog(wd.motor_b, value * signB);
  } else {
    // Fallback when the differential wrist isn't configured/enabled: jog
    // one raw motor as a single-axis proxy rather than doing nothing.
    sendAnalogJog(axis === "pitch" ? JOINT_WRIST_L : JOINT_WRIST_R, value);
  }
}

function wristAxisJogStart(axis, direction) {
  const wd = state.config && state.config.wrist_diff;
  if (wd && wd.enabled) {
    const signA = axis === "pitch" ? wd.pitch_sign_a : wd.roll_sign_a;
    const signB = axis === "pitch" ? wd.pitch_sign_b : wd.roll_sign_b;
    if (hasJoint(wd.motor_a)) send({ type: "jog_start", joint: wd.motor_a, direction: direction * signA });
    if (hasJoint(wd.motor_b)) send({ type: "jog_start", joint: wd.motor_b, direction: direction * signB });
  } else {
    const proxy = axis === "pitch" ? JOINT_WRIST_L : JOINT_WRIST_R;
    if (hasJoint(proxy)) send({ type: "jog_start", joint: proxy, direction });
  }
}
function wristAxisJogStop(axis) {
  const wd = state.config && state.config.wrist_diff;
  if (wd && wd.enabled) {
    if (hasJoint(wd.motor_a)) send({ type: "jog_stop", joint: wd.motor_a });
    if (hasJoint(wd.motor_b)) send({ type: "jog_stop", joint: wd.motor_b });
  } else {
    const proxy = axis === "pitch" ? JOINT_WRIST_L : JOINT_WRIST_R;
    if (hasJoint(proxy)) send({ type: "jog_stop", joint: proxy });
  }
}

function toggleWristTorque() {
  const names = [JOINT_WRIST_L, JOINT_WRIST_R].filter(hasJoint);
  if (!names.length) return;
  const anyOn = names.some(n => state.joints[n] && state.joints[n].enabled);
  send({ type: anyOn ? "disable_batch" : "enable_batch", joints: names });
}

function resetArmFromGamepad() {
  send({ type: "disable_all" });
  showToast("Gamepad: Reset -- torque disabled on all joints.", "warning");
}

function handleGamepadAxes(gp) {
  const rate = gpRateScale(gp);
  sendAnalogJog(JOINT_BASE, applyDeadzone(bindingAxisValue(gp, "axisBase")) * rate);
  sendAnalogJog(JOINT_SHOULDER, applyDeadzone(bindingAxisValue(gp, "axisShoulder")) * rate);
  sendAnalogJog(JOINT_ELBOW, applyDeadzone(bindingAxisValue(gp, "axisElbow")) * rate);
  wristAxisJogAnalog("roll", applyDeadzone(bindingAxisValue(gp, "axisWristRoll")) * rate);
}

function handleGamepadButtons(gp) {
  const prev = state.gamepad.prevButtons;
  const pressed = gp.buttons.map(b => b.pressed);
  const idxOf = (actionId) => state.gamepadBindings[actionId].index;
  const jp = (actionId) => { const i = idxOf(actionId); return pressed[i] && !prev[i]; };
  const jr = (actionId) => { const i = idxOf(actionId); return !pressed[i] && prev[i]; };

  if (jp("toggleBase")) toggleJointTorque(JOINT_BASE);
  if (jp("toggleShoulder")) toggleJointTorque(JOINT_SHOULDER);
  if (jp("toggleElbow")) toggleJointTorque(JOINT_ELBOW);
  if (jp("toggleGripper")) toggleJointTorque(JOINT_GRIPPER);
  if (jp("toggleWrist")) toggleWristTorque();
  if (jp("resetArm")) resetArmFromGamepad();

  if (jp("gripperOpen")) send({ type: "jog_start", joint: JOINT_GRIPPER, direction: 1 });
  if (jr("gripperOpen")) send({ type: "jog_stop", joint: JOINT_GRIPPER });
  if (jp("gripperClose")) send({ type: "jog_start", joint: JOINT_GRIPPER, direction: -1 });
  if (jr("gripperClose")) send({ type: "jog_stop", joint: JOINT_GRIPPER });

  if (jp("wristRollPos")) wristAxisJogStart("roll", 1);
  if (jr("wristRollPos")) wristAxisJogStop("roll");
  if (jp("wristRollNeg")) wristAxisJogStart("roll", -1);
  if (jr("wristRollNeg")) wristAxisJogStop("roll");

  if (jp("wristPitchPos")) wristAxisJogStart("pitch", 1);
  if (jr("wristPitchPos")) wristAxisJogStop("pitch");
  if (jp("wristPitchNeg")) wristAxisJogStart("pitch", -1);
  if (jr("wristPitchNeg")) wristAxisJogStop("pitch");

  if (jp("gripperStop")) send({ type: "jog_stop", joint: JOINT_GRIPPER });
  if (jp("wristRollStop")) wristAxisJogStop("roll");

  state.gamepad.prevButtons = pressed;
}

function renderGamepadLegendHighlight(gp) {
  document.querySelectorAll("#gamepadLegend .gp-item").forEach(el => {
    const id = el.dataset.action;
    const b = state.gamepadBindings[id];
    if (!b) return;
    const active = b.type === "axis"
      ? Math.abs(applyDeadzone(bindingAxisValue(gp, id))) > 0
      : (bindingButtonPressed(gp, id) || bindingTriggerValue(gp, id) > 0.1);
    el.classList.toggle("active", active);
  });
}

// One RAF loop drives three independent consumers of the same physical
// gamepad: (1) live hardware jog, gated by the Cockpit "Enable gamepad
// control" checkbox; (2) the Simulation tab's local preview, gated by its
// own checkbox -- runs even with no motors connected since it never talks
// to the backend; (3) Settings-tab keybind capture, whichever is active.
// All three read through state.gamepadBindings so a rebind in Settings
// takes effect everywhere immediately.
let lastGpSend = 0;
function pollGamepad() {
  requestAnimationFrame(pollGamepad);
  const gp = state.gamepad.index != null ? navigator.getGamepads()[state.gamepad.index] : null;
  if (!gp) return;

  if (state.listeningFor) tryCaptureBinding(gp);

  if (state.gamepad.enabled) {
    handleGamepadButtons(gp);
    const now = performance.now();
    if (now - lastGpSend >= 40) {
      lastGpSend = now;
      handleGamepadAxes(gp);
      renderGamepadLegendHighlight(gp);
    }
  }

  if (state.sim.enabled) stepSimulation(gp);
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

// jog: hold to move continuously (soft-ramped, existing behaviour)
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

// jog: customizable-step nudge -- one tap moves exactly `jogStep` degrees,
// ported from diff_wrist.py's incremental keyboard jog (JOG_STEPS_DEG,
// cycled with [ / ]) so precise positioning doesn't depend on how long a
// mouse button was held.
function buildJogStepChips() {
  const wrap = $("jogStepChips");
  if (!wrap) return;
  wrap.innerHTML = "";
  for (const step of JOG_STEPS) {
    const chip = document.createElement("button");
    chip.type = "button";
    chip.className = "jog-step-chip" + (step === jogStep ? " active" : "");
    chip.textContent = step < 1 ? step.toFixed(1) : `${step}`;
    chip.addEventListener("click", () => setJogStep(step));
    wrap.appendChild(chip);
  }
}
function setJogStep(step) {
  jogStep = Math.max(0.01, step);
  const idx = JOG_STEPS.indexOf(jogStep);
  jogStepIndex = idx >= 0 ? idx : jogStepIndex;
  buildJogStepChips();
  if ($("jogStepCustom") && document.activeElement !== $("jogStepCustom")) {
    $("jogStepCustom").value = jogStep;
  }
}
function cycleJogStep(dir) {
  jogStepIndex = Math.max(0, Math.min(JOG_STEPS.length - 1, jogStepIndex + dir));
  setJogStep(JOG_STEPS[jogStepIndex]);
}
$("jogStepCustom") && $("jogStepCustom").addEventListener("change", (e) => {
  const v = parseFloat(e.target.value);
  if (!Number.isNaN(v) && v > 0) setJogStep(v);
});
function nudge(direction) {
  if (!state.selectedJoint) return;
  send({ type: "jog_nudge", joint: state.selectedJoint, direction, step_deg: jogStep });
}
$("jogNudgeNeg") && $("jogNudgeNeg").addEventListener("click", () => nudge(-1));
$("jogNudgePos") && $("jogNudgePos").addEventListener("click", () => nudge(1));
buildJogStepChips();
setJogStep(jogStep);

// keyboard shortcuts (Control tab only, and never while typing into a
// field) -- ported from diff_wrist.py's key bindings: hold arrows to jog,
// [ ] to cycle step, , . to nudge by the current step, X/Esc to E-STOP.
document.addEventListener("keydown", (e) => {
  const tag = (e.target.tagName || "").toLowerCase();
  if (tag === "input" || tag === "select" || tag === "textarea") return;
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === "escape" && state.listeningFor) { cancelListening(); return; }
  if (k === "x" || k === "escape") {
    send({ type: "estop" });
    fetch("/api/estop", { method: "POST" });
    return;
  }
  const onControl = document.getElementById("tab-control").classList.contains("active");
  if (!onControl || !state.selectedJoint) return;
  if (k === "[") { cycleJogStep(-1); return; }
  if (k === "]") { cycleJogStep(1); return; }
  if (k === ",") { nudge(-1); return; }
  if (k === ".") { nudge(1); return; }
  if (e.key === "ArrowLeft") { send({ type: "jog_start", joint: state.selectedJoint, direction: -1 }); e.preventDefault(); return; }
  if (e.key === "ArrowRight") { send({ type: "jog_start", joint: state.selectedJoint, direction: 1 }); e.preventDefault(); return; }
});
document.addEventListener("keyup", (e) => {
  if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
    if (state.selectedJoint) send({ type: "jog_stop", joint: state.selectedJoint });
  }
});

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
  state.wristTargetPitch = msg.target_pitch;
  state.wristTargetRoll = msg.target_roll;
  state.wristPitchActual = msg.pitch;
  state.wristRollActual = msg.roll;
  maybeRenderViz();
}

// Wrist nudge: same customizable jog step as the Control tab, applied to
// whichever axis was pressed, sent as a ramped wrist_go relative to the
// last known target (falls back to 0 if the wrist hasn't reported yet).
function nudgeWrist(axis, direction) {
  if (!state.config || !state.config.wrist_diff || !state.config.wrist_diff.enabled) {
    showToast("Wrist: differential wrist isn't enabled -- save the Mixing Config first.", "error");
    return;
  }
  const pitch_deg = (state.wristTargetPitch ?? 0) + (axis === "pitch" ? direction * jogStep : 0);
  const roll_deg = (state.wristTargetRoll ?? 0) + (axis === "roll" ? direction * jogStep : 0);
  send({ type: "wrist_go", pitch_deg, roll_deg, ramped: $("wristRampedToggle").checked });
}
$("wristPitchNudgeNeg") && $("wristPitchNudgeNeg").addEventListener("click", () => nudgeWrist("pitch", -1));
$("wristPitchNudgePos") && $("wristPitchNudgePos").addEventListener("click", () => nudgeWrist("pitch", 1));
$("wristRollNudgeNeg") && $("wristRollNudgeNeg").addEventListener("click", () => nudgeWrist("roll", -1));
$("wristRollNudgePos") && $("wristRollNudgePos").addEventListener("click", () => nudgeWrist("roll", 1));


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

// --------------------------------------------------- settings: keybinds --
function renderKeybindTable() {
  const tbody = $("keybindTableBody");
  if (!tbody) return;
  tbody.innerHTML = "";
  for (const id of ACTION_ORDER) {
    const b = state.gamepadBindings[id];
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${BINDING_LABELS[id]}</td>
      <td class="mono-val" id="kb-current-${id}">${bindingText(b)}</td>
      <td>${b.type === "axis" ? `<label class="chk"><input type="checkbox" class="kb-invert" data-action="${id}" ${b.invert ? "checked" : ""}> Invert</label>` : ""}</td>
      <td><button class="btn btn-secondary kb-listen" data-action="${id}">Listen</button></td>
    `;
    tbody.appendChild(tr);
  }
  tbody.querySelectorAll(".kb-listen").forEach(btn => {
    btn.addEventListener("click", () => startListening(btn.dataset.action, btn));
  });
  tbody.querySelectorAll(".kb-invert").forEach(cb => {
    cb.addEventListener("change", (e) => {
      state.gamepadBindings[e.target.dataset.action].invert = e.target.checked;
      saveGamepadBindings();
    });
  });
}

function startListening(actionId, btnEl) {
  if (state.gamepad.index == null) {
    showToast("Connect a gamepad first -- press any button on it.", "error");
    return;
  }
  if (state.listeningFor === actionId) { cancelListening(); return; }
  cancelListening();
  const gp = navigator.getGamepads()[state.gamepad.index];
  state.listeningFor = actionId;
  state.listenBaseline = {
    axes: gp ? gp.axes.slice() : [],
    buttons: gp ? gp.buttons.map(b => b.pressed) : [],
  };
  state.listenBtnEl = btnEl;
  state.listenStarted = performance.now();
  btnEl.textContent = "Listening… (Esc to cancel)";
  btnEl.classList.add("kb-listening");
}

function cancelListening() {
  if (state.listenBtnEl) {
    state.listenBtnEl.textContent = "Listen";
    state.listenBtnEl.classList.remove("kb-listening");
  }
  state.listeningFor = null;
  state.listenBaseline = null;
  state.listenBtnEl = null;
}

function tryCaptureBinding(gp) {
  if (!state.listeningFor || !state.listenBaseline) return;
  if (performance.now() - state.listenStarted > 12000) {
    cancelListening();
    showToast("Keybind: timed out waiting for input.", "warning");
    return;
  }
  const actionId = state.listeningFor;
  const type = state.gamepadBindings[actionId].type;
  if (type === "axis") {
    for (let i = 0; i < gp.axes.length; i++) {
      const base = state.listenBaseline.axes[i] || 0;
      if (Math.abs(gp.axes[i] - base) > 0.5) {
        state.gamepadBindings[actionId].index = i;
        finishListening(actionId, `Axis ${i}`);
        return;
      }
    }
  } else {
    for (let i = 0; i < gp.buttons.length; i++) {
      const wasPressed = state.listenBaseline.buttons[i];
      const isPressed = gp.buttons[i] && (gp.buttons[i].pressed || gp.buttons[i].value > 0.5);
      if (isPressed && !wasPressed) {
        state.gamepadBindings[actionId].index = i;
        finishListening(actionId, `Button ${i}`);
        return;
      }
    }
  }
}

function finishListening(actionId, text) {
  saveGamepadBindings();
  cancelListening();
  renderKeybindTable();
  buildGamepadLegend();
  showToast(`${BINDING_LABELS[actionId]} bound to ${text}.`, "success");
}

$("btnKeybindReset").addEventListener("click", () => {
  if (!confirm("Reset all gamepad keybinds to their defaults?")) return;
  state.gamepadBindings = JSON.parse(JSON.stringify(DEFAULT_GAMEPAD_BINDINGS));
  saveGamepadBindings();
  renderKeybindTable();
  buildGamepadLegend();
  showToast("Gamepad keybinds reset to defaults.", "info");
});

renderKeybindTable();

// ------------------------------------------------- cockpit view switcher --
document.querySelectorAll("#vizViewTabs .viz-view-btn").forEach(btn => {
  btn.addEventListener("click", () => {
    state.vizView = btn.dataset.view;
    document.querySelectorAll("#vizViewTabs .viz-view-btn").forEach(b => b.classList.toggle("active", b === btn));
    renderArmViz();
  });
});

initTabs();
setupGamepadEvents();
pollGamepad();
connect();

// ------------------------------------------------------ nav rail / shell --
// Collapsible rail (desktop) + slide-in drawer (mobile). Purely presentational
// -- doesn't touch any websocket/command logic above.
(function initShell() {
  const shell = document.getElementById("appShell");
  const railToggle = document.getElementById("railToggle");
  const mobileBtn = document.getElementById("mobileNavBtn");
  const scrim = document.getElementById("railScrim");
  if (!shell) return;

  const RAIL_KEY = "armRailCollapsed";
  try {
    if (localStorage.getItem(RAIL_KEY) === "1") shell.classList.add("rail-collapsed");
  } catch (e) { /* storage unavailable */ }

  if (railToggle) {
    railToggle.addEventListener("click", () => {
      shell.classList.toggle("rail-collapsed");
      try { localStorage.setItem(RAIL_KEY, shell.classList.contains("rail-collapsed") ? "1" : "0"); } catch (e) {}
    });
  }
  function openNav() { shell.classList.add("nav-open"); }
  function closeNav() { shell.classList.remove("nav-open"); }
  if (mobileBtn) mobileBtn.addEventListener("click", openNav);
  if (scrim) scrim.addEventListener("click", closeNav);
  document.querySelectorAll("#tabbar .tab-btn").forEach(btn => {
    btn.addEventListener("click", closeNav);
  });
})();
