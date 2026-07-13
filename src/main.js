// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Wiring + the deterministic fixed-step loop. All sim time comes from DT-sized steps;
// wall clock only decides HOW MANY steps to run — never feeds the physics.

import { stepAircraft, initialState } from './physics.js';
import { startTelemetry, telemetryFrom, eulerFromQuat, headingDeg } from './telemetry.js';
import { MODES, MODE_NAMES, apStep, manualControls } from './autopilot.js';
import { startMissionLink } from './missionLink.js';
import { toLocalTargets } from './missions.js';
import { geodeticToLocal } from './telemetry.js';
import { defaultParams, clampParam } from './params.js';
import { createSensors, stepSensors, injectFault, clearFault } from './sensors.js';
import {
  createEstimator, stepEstimator, ekfReport, createAttEstimator, stepAttEstimator,
} from './estimator.js';
import { createBattery, stepBattery, batteryOutputs } from './battery.js';
import { airData } from './physics.js';
import { createWorld } from './scene.js';
import { createWind, stepWind } from './wind.js';
import { createEngineering } from './engineering.js';

const THREE = window.THREE;
const DT = 1 / 60; // s — fixed physics timestep
const THROTTLE_RATE = 0.5; // full-range throttle travel per second (dt-integrated)

// --- Sim state -------------------------------------------------------------
let state = initialState();
let throttle = 0.65; // pilot's throttle setting (MANUAL mode)
let simTime = 0;
let manual = false; // once __advance is used, wall-clock stepping stops (test/HILS mode)

// The sim is authoritative for arm/mode; the GCS only requests changes.
let armed = true; // boots armed + airborne so the standalone sim flies immediately
let ap = {
  mode: MODES.MANUAL, landing: false, targetAlt: 120, targetHeading: 0,
  guided: null, mission: null,
};
let lastControls = { aileron: 0, elevator: 0, rudder: 0, throttle };
let lastReached = -1; // last mission seq completed (edge → MISSION_ITEM_REACHED)
let params = defaultParams(); // live-tunable via PARAM_SET; persists across resets
const SENSOR_SEED = 1;
let sensors = createSensors(SENSOR_SEED);
let readings = null; // latest sensor sweep (feeds telemetry)
let lastGps = null; // held through dropouts, like a real receiver's last fix
let est = createEstimator(state);
let att = createAttEstimator(state);
let battery = createBattery();
const WIND_SEED = 2;
let wind = createWind(WIND_SEED);
let windWorld = [0, 0, 0];

function setMode(m) {
  const e = eulerFromQuat(state.quat);
  // Capture "here" as the hold target; the mission survives mode changes.
  ap = {
    mode: m, landing: false,
    targetAlt: Math.max(40, state.pos[1]),
    targetHeading: headingDeg(e.yaw),
    guided: null,
    mission: ap.mission,
  };
}

function applyCommand(cmd) {
  switch (cmd.type) {
    case 'arm': armed = cmd.value === 1; break;
    case 'mode': setMode(cmd.custom >>> 0); break;
    case 'takeoff': armed = true; setMode(MODES.TAKEOFF); ap.targetAlt = cmd.alt; break;
    case 'land': ap = { ...ap, landing: true }; break;
    case 'rtl': setMode(MODES.RTL); break;
    case 'mission':
      ap = { ...ap, mission: { targets: toLocalTargets(cmd.items), idx: 0 } };
      lastReached = -1;
      break;
    case 'goto': {
      const [x, , z] = geodeticToLocal({ lat: cmd.lat, lon: cmd.lon });
      setMode(MODES.GUIDED);
      ap = { ...ap, guided: { x, z, alt: Math.max(40, cmd.alt > 1 ? cmd.alt : state.pos[1]) } };
      break;
    }
    case 'param': {
      const v = clampParam(cmd.id, cmd.value);
      if (v !== null) params = { ...params, [cmd.id]: v };
      break;
    }
  }
}

const keys = new Set();
window.addEventListener('keydown', (e) => {
  // e.code (physical key), never e.key — IME layouts report e.key as 'Process'.
  keys.add(e.code);
  if (e.code === 'KeyR') reset();
  if (e.code.startsWith('Arrow')) e.preventDefault();
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

  // HILS closure: everything in the control path is ESTIMATED — nav position/
  // velocity, Mahony attitude, bias-corrected rates, pitot airspeed. The only
  // truth left is the weight-on-wheels discrete (a real switch on real gear).
  const rateEst = readings?.gyro
    ? readings.gyro.map((v, i) => v - att.bias[i])
    : readings ? [0, 0, 0] : state.omega; // gyro dropout: rate damping goes blind
  const nav = {
    ...state, pos: est.pos, vel: est.vel, quat: att.quat, omega: rateEst,
    wow: state.pos[1] <= 0.5,
  };

  let controls;
  if (ap.mode === MODES.MANUAL && !ap.landing) {
    controls = manualControls(nav, readControls()); // stick → surfaces, SAS on est rates
  } else {
    const r = apStep(nav, ap, params, readings?.pitot?.[0] ?? null);
    ap = r.ap;
    controls = r.controls;
    if (r.disarm) armed = false;
    if (r.reached.length) lastReached = r.reached[r.reached.length - 1];
  }
  if (!armed) controls = { ...controls, throttle: 0 }; // DISARM cuts the engine

  lastControls = controls;
  const w = stepWind(wind, state, params, dt);
  wind = w.wind;
  windWorld = w.windWorld;
  state = stepAircraft(state, controls, dt, windWorld);
  const sw = stepSensors(sensors, state, params, windWorld);
  sensors = sw.sensors;
  readings = sw.readings;
  if (readings.gps) lastGps = readings.gps;
  att = stepAttEstimator(att, readings, dt);
  est = stepEstimator(est, readings, dt);
  battery = stepBattery(battery, state.act.dt, dt); // actual actuator throttle
  simTime += dt;
  if (Math.round(simTime / dt) % 6 === 0) eng.record(); // 10 Hz chart samples
}

function reset() {
  state = initialState();
  throttle = 0.65;
  simTime = 0;
  armed = true;
  ap = {
    mode: MODES.MANUAL, landing: false, targetAlt: 120, targetHeading: 0,
    guided: null, mission: null,
  };
  lastControls = { aileron: 0, elevator: 0, rudder: 0, throttle };
  lastReached = -1;
  sensors = createSensors(SENSOR_SEED);
  readings = null;
  lastGps = null;
  est = createEstimator(state);
  att = createAttEstimator(state);
  battery = createBattery();
  wind = createWind(WIND_SEED);
  windWorld = [0, 0, 0];
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
window.__state = () => JSON.stringify({ simTime, throttle, armed, ap, params, sensors, est, att, battery, wind, ...state });
window.__command = (cmd) => applyCommand(cmd); // same path the GCS uses, for tests/HILS
window.injectFault = (sensor, type, opts) => { sensors = injectFault(sensors, sensor, type, opts); };
window.clearFault = (sensor) => { sensors = clearFault(sensors, sensor); };

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
    const ad = airData(state.quat, state.vel, windWorld);
    return {
      state, est, att, windWorld, va: ad.Va, alpha: ad.alpha, beta: ad.beta,
      faults: readings?.faults ?? {}, ekf: ekfReport(est, readings),
      batt: batteryOutputs(battery, state.act.dt),
    };
  },
  injectFault: (s, t) => window.injectFault(s, t),
  clearFault: (s) => window.clearFault(s),
});
window.__eng = eng; // DOM-gate hook for the browser check

function render() {
  world.update(state, simTime);

  const ad = airData(state.quat, state.vel, windWorld);
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
startTelemetry(() =>
  telemetryFrom(state, state.act.dt, simTime, {
    armed, customMode: ap.mode,
    missionSeq: ap.mission ? Math.min(ap.mission.idx, ap.mission.targets.length - 1) : -1,
    missionReached: lastReached,
    gps: lastGps, gpsDropout: readings ? !readings.gps : false,
    baroAlt: readings?.baro?.[0], health: readings?.health, faults: readings?.faults,
    est, ekf: ekfReport(est, readings),
    va: airData(state.quat, state.vel, windWorld).Va,
    attQuat: att.quat,
    omega: readings?.gyro ? readings.gyro.map((v, i) => v - att.bias[i]) : state.omega,
    ...batteryOutputs(battery, state.act.dt),
  })
).then((on) => {
  if (on) {
    startMissionLink(applyCommand);
    console.log('telemetry → bridge: on, command link: on');
  }
});

window.__ready = true;
