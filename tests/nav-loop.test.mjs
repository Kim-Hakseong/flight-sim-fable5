// M11 gate: the FULL HILS loop — physics → sensors → estimator → autopilot →
// surfaces — flying on ESTIMATED nav + pitot airspeed, truth only for attitude
// and the weight-on-wheels discrete.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepAircraft, initialState } from '../src/physics.js';
import { MODES, apStep } from '../src/autopilot.js';
import { createSensors, stepSensors, injectFault, clearFault } from '../src/sensors.js';
import {
  createEstimator, stepEstimator, createAttEstimator, stepAttEstimator,
} from '../src/estimator.js';
import { createWind, stepWind } from '../src/wind.js';
import { toLocalTargets } from '../src/missions.js';
import { HOME } from '../src/telemetry.js';
import { defaultParams } from '../src/params.js';

const DT = 1 / 60;

function createLoop({ seed = 21, params = {} } = {}) {
  const P = { ...defaultParams(), ...params };
  return {
    P,
    state: initialState(),
    sensors: createSensors(seed),
    est: createEstimator(initialState()),
    att: createAttEstimator(initialState()),
    wind: createWind(seed + 1),
    readings: null,
  };
}

// One full closed-loop step, mirroring src/main.js stepSim exactly: the control
// path sees ONLY estimated state (nav, attitude, corrected rates, pitot) + WoW.
function stepLoop(L, ap) {
  const rateEst = L.readings?.gyro
    ? L.readings.gyro.map((v, i) => v - L.att.bias[i])
    : L.readings ? [0, 0, 0] : L.state.omega;
  const nav = {
    ...L.state, pos: L.est.pos, vel: L.est.vel, quat: L.att.quat, omega: rateEst,
    wow: L.state.pos[1] <= 0.5,
  };
  const r = apStep(nav, ap, L.P, L.readings?.pitot?.[0] ?? null);
  const w = stepWind(L.wind, L.state, L.P, DT);
  L.wind = w.wind;
  L.state = stepAircraft(L.state, r.controls, DT, w.windWorld);
  const sw = stepSensors(L.sensors, L.state, L.P, w.windWorld);
  L.sensors = sw.sensors;
  L.readings = sw.readings;
  L.att = stepAttEstimator(L.att, L.readings, DT);
  L.est = stepEstimator(L.est, L.readings, DT);
  return r;
}

const PLAN = toLocalTargets([
  { seq: 0, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon, alt: 120, param1: 0, param2: 0 },
  { seq: 1, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon + 0.014, alt: 150, param1: 0, param2: 0 },
  { seq: 2, command: 16, frame: 3, lat: HOME.lat, lon: HOME.lon + 0.014, alt: 100, param1: 0, param2: 0 },
]);

test('estimated nav: a 3-waypoint mission completes in light turbulence', () => {
  const L = createLoop();
  let ap = {
    mode: MODES.AUTO, landing: false, targetAlt: 120, targetHeading: 0,
    guided: null, mission: { targets: PLAN, idx: 0 },
  };
  const reached = [];
  for (let i = 0; i < 300 * 60 && ap.mission.idx < 3; i++) {
    const r = stepLoop(L, ap);
    ap = r.ap;
    reached.push(...r.reached);
  }
  assert.deepEqual(reached, [0, 1, 2]);
  assert.ok(L.state.pos[1] > 60, `still flying at ${L.state.pos[1]} m`);
});

test('estimated nav: survives a 12 s GPS dropout mid-leg and still finishes', () => {
  const L = createLoop();
  let ap = {
    mode: MODES.AUTO, landing: false, targetAlt: 120, targetHeading: 0,
    guided: null, mission: { targets: PLAN, idx: 0 },
  };
  const reached = [];
  let maxCoastErr = 0;
  for (let i = 0; i < 300 * 60 && ap.mission.idx < 3; i++) {
    if (i === 20 * 60) L.sensors = injectFault(L.sensors, 'gps', 'dropout');
    if (i === 32 * 60) L.sensors = clearFault(L.sensors, 'gps');
    const r = stepLoop(L, ap);
    ap = r.ap;
    reached.push(...r.reached);
    if (i >= 20 * 60 && i < 32 * 60) {
      maxCoastErr = Math.max(maxCoastErr,
        Math.hypot(L.est.pos[0] - L.state.pos[0], L.est.pos[2] - L.state.pos[2]));
    }
  }
  assert.deepEqual(reached, [0, 1, 2], 'mission must still complete');
  assert.ok(maxCoastErr < 80, `estimate drifted ${maxCoastErr} m while coasting`);
});

test('estimated nav: RTL lands and disarms on the WoW discrete', () => {
  const L = createLoop();
  L.state = { ...L.state, pos: [0, 120, -1500] };
  L.est = createEstimator(L.state);
  let ap = { mode: MODES.RTL, landing: false, targetAlt: 120, targetHeading: 0, guided: null, mission: null };
  let disarmed = false;
  for (let i = 0; i < 400 * 60 && !disarmed; i++) {
    const r = stepLoop(L, ap);
    ap = r.ap;
    disarmed = r.disarm;
  }
  assert.ok(disarmed, 'disarmed after touchdown');
  assert.equal(L.state.pos[1], 0, 'actually on the ground (truth)');
});

test('determinism: the full closed loop is bit-identical across reruns', () => {
  const fly = () => {
    const L = createLoop();
    let ap = { mode: MODES.GUIDED, landing: false, targetAlt: 140, targetHeading: 45, guided: null, mission: null };
    for (let i = 0; i < 30 * 60; i++) {
      const r = stepLoop(L, ap);
      ap = r.ap;
    }
    return JSON.stringify({ s: L.state, est: L.est, att: L.att, ap });
  };
  assert.equal(fly(), fly());
});
