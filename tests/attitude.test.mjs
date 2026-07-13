// M12 gate: Mahony attitude estimator — convergence, gyro-bias absorption,
// bounded error through turns, and fault behavior. Closed loops fly on the
// ESTIMATED attitude (same wiring as src/main.js).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  stepAircraft, initialState, quatMultiply, TRIM, AC, G,
} from '../src/physics.js';
import { createAttEstimator, stepAttEstimator, createEstimator, stepEstimator } from '../src/estimator.js';
import { createSensors, stepSensors, injectFault, specificForce } from '../src/sensors.js';
import { holdControls } from '../src/autopilot.js';
import { eulerFromQuat, headingDeg } from '../src/telemetry.js';
import { defaultParams } from '../src/params.js';

const DT = 1 / 60;
const P = defaultParams();
const TRIM_CMD = { aileron: 0, elevator: TRIM.de / AC.maxDef, rudder: 0, throttle: TRIM.dt };

const qErrDeg = (a, b) => {
  const d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, d)) * (180 / Math.PI);
};
const rollQ = (deg) => {
  const h = (deg / 2) * (Math.PI / 180);
  return [0, 0, -Math.sin(h), Math.cos(h)]; // roll right = rotation about −Z (nose)
};

// Open-loop trim flight; the estimator watches the (faultable) sensors.
function flyTrim(seconds, { seed = 21, mutate = (s) => s, attInit = null } = {}) {
  let s = initialState();
  let sns = mutate(createSensors(seed));
  let att = attInit ?? createAttEstimator(s);
  let firstGood = null;
  for (let i = 0; i < seconds * 60; i++) {
    s = stepAircraft(s, TRIM_CMD, DT);
    const sw = stepSensors(sns, s, P);
    sns = sw.sensors;
    att = stepAttEstimator(att, sw.readings, DT);
    if (firstGood === null && qErrDeg(att.quat, s.quat) < 2) firstGood = i / 60;
  }
  return { s, att, firstGood };
}

test('accelerometer model: level trim reads ≈ +g along body-up', () => {
  const f = specificForce(initialState());
  assert.ok(Math.abs(Math.hypot(...f) - G) < 0.5, `|f| = ${Math.hypot(...f)}`);
  assert.ok(f[1] > 9.2, `body-up component = ${f[1]}`);
});

test('converges from a 20° attitude error within 10 s of trim flight', () => {
  const s0 = initialState();
  const { s, att, firstGood } = flyTrim(30, {
    attInit: { quat: quatMultiply(s0.quat, rollQ(20)), bias: [0, 0, 0], lpErr: 0 },
  });
  assert.ok(firstGood !== null && firstGood < 10, `converged at t = ${firstGood}`);
  assert.ok(qErrDeg(att.quat, s.quat) < 1.5, `final err ${qErrDeg(att.quat, s.quat)}°`);
});

test('absorbs an injected gyro bias into the bias estimate', () => {
  const { s, att } = flyTrim(120, {
    seed: 5,
    mutate: (sns) => injectFault(sns, 'gyro', 'bias', { bias: 0.05 }),
  });
  for (const b of att.bias) {
    assert.ok(Math.abs(b - 0.05) < 0.015, `bias est ${att.bias} (want ≈0.05)`);
  }
  assert.ok(qErrDeg(att.quat, s.quat) < 2, `att err ${qErrDeg(att.quat, s.quat)}°`);
});

test('closed loop on estimated attitude: 90° turn stays bounded, re-converges', () => {
  let s = initialState();
  let sns = createSensors(21);
  let att = createAttEstimator(s);
  let est = createEstimator(s);
  let readings = null;
  let maxErrTurn = 0;
  for (let i = 0; i < 60 * 60; i++) {
    const rateEst = readings?.gyro ? readings.gyro.map((v, j) => v - att.bias[j]) : s.omega;
    const nav = { ...s, pos: est.pos, vel: est.vel, quat: att.quat, omega: rateEst };
    const va = readings?.pitot?.[0] ?? null;
    s = stepAircraft(s, holdControls(nav, 120, 90, null, undefined, va), DT);
    const sw = stepSensors(sns, s, P);
    sns = sw.sensors;
    readings = sw.readings;
    att = stepAttEstimator(att, readings, DT);
    est = stepEstimator(est, readings, DT);
    maxErrTurn = Math.max(maxErrTurn, qErrDeg(att.quat, s.quat));
  }
  const e = eulerFromQuat(s.quat);
  assert.ok(Math.abs(headingDeg(e.yaw) - 90) < 6, `true hdg ended at ${headingDeg(e.yaw)}`);
  assert.ok(maxErrTurn < 8, `att err peaked at ${maxErrTurn}° in the turn`);
  assert.ok(qErrDeg(att.quat, s.quat) < 3, `post-turn err ${qErrDeg(att.quat, s.quat)}°`);
});

test('mag bias fault drags the heading estimate off (visible failure mode)', () => {
  const { s, att } = flyTrim(60, {
    seed: 7,
    mutate: (sns) => injectFault(sns, 'mag', 'bias', { bias: 30 }),
  });
  const eEst = eulerFromQuat(att.quat);
  const eTrue = eulerFromQuat(s.quat);
  const hdgErr = Math.abs(headingDeg(eEst.yaw) - headingDeg(eTrue.yaw));
  assert.ok(hdgErr > 15, `heading estimate must be pulled off (got ${hdgErr}°)`);
});

test('gyro dropout: attitude degrades gracefully, no blow-up', () => {
  const { s, att } = flyTrim(20, {
    seed: 9,
    mutate: (sns) => injectFault(sns, 'gyro', 'dropout'),
  });
  const err = qErrDeg(att.quat, s.quat);
  assert.ok(Number.isFinite(err) && err < 25, `err ${err}° (accel/mag alone must roughly hold it)`);
});
