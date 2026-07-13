// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Thin browser shell: keyboard in, rendering + HUD out, telemetry/commands to the
// bridge. ALL vehicle behavior lives in src/vehicle.js (shared with tests and the
// HILS runner). Wall clock only decides HOW MANY fixed steps run — never physics.

import {
  createVehicle, vehicleStep, vehicleCommand, vehicleFault, vehicleClearFault,
  vehicleServoFault, vehicleClearServoFault, vehicleTelemetry,
} from './vehicle.js';
import { MODES, MODE_NAMES } from './autopilot.js';
import { airData } from './physics.js';
import { ekfReport } from './estimator.js';
import { batteryOutputs } from './battery.js';
import { startTelemetry } from './telemetry.js';
import { startMissionLink } from './missionLink.js';
import { createWorld } from './scene.js';
import { createEngineering } from './engineering.js';
import { SCENARIOS, runScenario } from './hils.js';

const THREE = window.THREE;
const DT = 1 / 60; // s — fixed physics timestep
const THROTTLE_RATE = 0.5; // full-range throttle travel per second (dt-integrated)

// --- Sim state -------------------------------------------------------------
let veh = createVehicle(); // boots DISARMED at the runway threshold
let throttle = 0; // pilot's throttle setting (MANUAL mode)
let manual = false; // once __advance is used, wall-clock stepping stops (test/HILS mode)

function applyCommand(cmd) {
  veh = vehicleCommand(veh, cmd);
}

const keys = new Set();
window.addEventListener('keydown', (e) => {
  // e.code (physical key), never e.key — IME layouts report e.key as 'Process'.
  keys.add(e.code);
  if (e.code === 'KeyR') reset();
  if (e.code === 'KeyT') applyCommand({ type: 'takeoff', alt: 60 });
  if (e.code === 'Space') applyCommand({ type: 'arm', value: veh.armed ? 0 : 1 });
  if (e.code === 'KeyM') applyCommand({ type: 'mode', custom: MODES.MANUAL });
  if (e.code.startsWith('Arrow') || e.code === 'Space') e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

function readControls() {
  return {
    pitch: (keys.has('ArrowUp') ? 1 : 0) - (keys.has('ArrowDown') ? 1 : 0),
    roll: (keys.has('ArrowRight') ? 1 : 0) - (keys.has('ArrowLeft') ? 1 : 0),
    yaw: (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0),
    throttle,
  };
}

function stepSim(dt) {
  const dThr = ((keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0)) * THROTTLE_RATE * dt;
  throttle = Math.min(1, Math.max(0, throttle + dThr));
  veh = vehicleStep(veh, dt, readControls());
  if (Math.round(veh.simTime / dt) % 6 === 0) eng.record(); // 10 Hz chart samples
}

function reset() {
  veh = createVehicle();
  throttle = 0;
  keys.clear();
}

// --- Deterministic test/HILS surface ----------------------------------------
window.__advance = (seconds, dt = DT) => {
  manual = true;
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) stepSim(dt);
  return window.__state();
};
window.__reset = () => reset();
window.__state = () => JSON.stringify({ throttle, ...veh });
window.__command = (cmd) => applyCommand(cmd); // same path the GCS uses
window.injectFault = (sensor, type, opts) => { veh = vehicleFault(veh, sensor, type, opts); };
window.clearFault = (sensor) => { veh = vehicleClearFault(veh, sensor); };
window.injectServoFault = (ch, type, opts) => { veh = vehicleServoFault(veh, ch, type, opts); };
window.clearServoFault = (ch) => { veh = vehicleClearServoFault(veh, ch); };
// Scenario bench: __hils.list() / __hils.run('name' | {custom scenario}).
// Runs on a FRESH vehicle — the live sim is untouched.
window.__hils = {
  list: () => SCENARIOS.map((s) => s.name),
  run: (sc) => runScenario(typeof sc === 'string' ? SCENARIOS.find((x) => x.name === sc) : sc),
  runAll: () => SCENARIOS.map((s) => runScenario(s)),
};

// --- Scene (built in src/scene.js; render-only) --------------------------------
const world = createWorld(THREE);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);
addEventListener('resize', () => {
  world.camera.aspect = innerWidth / innerHeight;
  world.camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

const hud = document.getElementById('hud');
const DEG = 180 / Math.PI;

const eng = createEngineering({
  getData: () => {
    const ad = airData(veh.state.quat, veh.state.vel, veh.windWorld);
    return {
      state: veh.state, est: veh.est, att: veh.att, windWorld: veh.windWorld,
      va: ad.Va, alpha: ad.alpha, beta: ad.beta,
      faults: veh.readings?.faults ?? {}, servoFaults: veh.servoFaults,
      ekf: ekfReport(veh.est, veh.readings),
      batt: batteryOutputs(veh.battery, veh.state.act.dt),
    };
  },
  injectFault: (s, t) => window.injectFault(s, t),
  clearFault: (s) => window.clearFault(s),
  injectServoFault: (ch, t) => window.injectServoFault(ch, t),
  clearServoFault: (ch) => window.clearServoFault(ch),
});
window.__eng = eng; // DOM-gate hook for the browser check

function render() {
  const { state, ap, armed, simTime } = veh;
  world.update(state, simTime);

  const ad = airData(state.quat, state.vel, veh.windWorld);
  const wp = ap.mission ? ` · WP ${Math.min(ap.mission.idx + 1, ap.mission.targets.length)}/${ap.mission.targets.length}` : '';
  const modeLabel = (MODE_NAMES[ap.mode] ?? ap.mode) + (ap.landing ? '·LAND' : '') + wp;
  hud.textContent =
    `VA  ${ad.Va.toFixed(1).padStart(5)} m/s   α ${(ad.alpha * DEG).toFixed(1).padStart(5)}°  β ${(ad.beta * DEG).toFixed(1).padStart(5)}°` +
    `\nALT ${state.pos[1].toFixed(1).padStart(5)} m     VS ${state.vel[1].toFixed(1).padStart(5)} m/s` +
    `\nTHR ${(state.act.dt * 100).toFixed(0).padStart(4)} %    AIL ${(state.act.da * DEG).toFixed(0).padStart(4)}°  ELV ${(state.act.de * DEG).toFixed(0).padStart(4)}°  RUD ${(state.act.dr * DEG).toFixed(0).padStart(4)}°` +
    `\nT+  ${simTime.toFixed(2).padStart(7)} s${manual ? '  [manual]' : ''}` +
    `\n${armed ? 'ARMED' : 'DISARMED'} · ${modeLabel}`;

  eng.render();
  renderer.render(world.scene, world.camera);
}

// --- Fixed-step loop: accumulate wall time, step in exact DT increments ------
let acc = 0;
let last = null;
function frame(tMs) {
  requestAnimationFrame(frame);
  if (!manual && last !== null) {
    acc += Math.min((tMs - last) / 1000, 0.25); // clamp: don't spiral after a stall
    while (acc >= DT) {
      stepSim(DT);
      acc -= DT;
    }
  }
  last = tMs;
  render();
}
requestAnimationFrame(frame);

// Feed the GCS bridge if (and only if) this page is served by it; commands ride
// back on the same probe result.
startTelemetry(() => vehicleTelemetry(veh)).then((on) => {
  if (on) {
    startMissionLink(applyCommand);
    console.log('telemetry → bridge: on, command link: on');
  }
});

window.__ready = true;
