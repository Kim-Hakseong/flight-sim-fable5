import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createEstimator, stepEstimator, ekfReport, EKF } from '../src/estimator.js';
import { createBattery, stepBattery, batteryOutputs, batteryCurrentA } from '../src/battery.js';
import { createSensors, stepSensors, injectFault } from '../src/sensors.js';
import { stepAircraft, initialState, TRIM, AC } from '../src/physics.js';
import { defaultParams } from '../src/params.js';

const DT = 1 / 60;
const P = defaultParams();
const CONTROLS = { aileron: 0, elevator: TRIM.de / AC.maxDef, rudder: 0, throttle: TRIM.dt };

function flyWithNav(seconds, mutateSensors = (s) => s, seed = 21) {
  let state = initialState();
  let sns = mutateSensors(createSensors(seed));
  let est = createEstimator(state);
  let readings = null;
  for (let i = 0; i < seconds * 60; i++) {
    state = stepAircraft(state, CONTROLS, DT);
    const sw = stepSensors(sns, state, P);
    sns = sw.sensors;
    readings = sw.readings;
    est = stepEstimator(est, readings, DT);
  }
  return { state, est, readings };
}

test('estimator: tracks the true position with clean sensors', () => {
  const { state, est } = flyWithNav(30);
  const errH = Math.hypot(est.pos[0] - state.pos[0], est.pos[2] - state.pos[2]);
  assert.ok(errH < 5, `horizontal error ${errH} m`);
  assert.ok(Math.abs(est.pos[1] - state.pos[1]) < 3, `vertical error ${est.pos[1] - state.pos[1]} m`);
  const r = ekfReport(est, { mag: [0] });
  assert.ok(r.pos_horiz_variance < 0.5, `clean variance ${r.pos_horiz_variance}`);
  assert.ok(r.flags & EKF.POS_HORIZ_ABS && r.flags & EKF.POS_VERT_ABS);
});

test('estimator FDE: a big GPS bias is gated out — the estimate does not jump', () => {
  // Fly clean 10 s, then hit the GPS with a 500 m bias for 5 s.
  let state = initialState();
  let sns = createSensors(21);
  let est = createEstimator(state);
  for (let i = 0; i < 10 * 60; i++) {
    state = stepAircraft(state, CONTROLS, DT);
    const sw = stepSensors(sns, state, P);
    sns = sw.sensors;
    est = stepEstimator(est, sw.readings, DT);
  }
  sns = injectFault(sns, 'gps', 'bias', { bias: 500 });
  let maxErr = 0;
  let readings = null;
  for (let i = 0; i < 5 * 60; i++) {
    state = stepAircraft(state, CONTROLS, DT);
    const sw = stepSensors(sns, state, P);
    sns = sw.sensors;
    readings = sw.readings;
    est = stepEstimator(est, readings, DT);
    maxErr = Math.max(maxErr, Math.hypot(est.pos[0] - state.pos[0], est.pos[2] - state.pos[2]));
  }
  assert.ok(maxErr < 100, `estimate chased the biased GPS: err ${maxErr} m`);
  const r = ekfReport(est, readings);
  assert.ok(r.pos_horiz_variance > 0.5, `variance must grow while rejecting (${r.pos_horiz_variance})`);
  assert.equal(r.flags & EKF.POS_HORIZ_ABS, 0, 'abs-position flag must drop while unaided');
});

test('estimator: GPS dropout coasts, then recovers on restore', () => {
  const { est, readings } = flyWithNav(20, (s) => injectFault(s, 'gps', 'dropout'));
  assert.ok(est.gpsAge > 19, 'never aided');
  const r = ekfReport(est, readings);
  assert.equal(r.flags & EKF.POS_HORIZ_ABS, 0);
  assert.ok(r.pos_horiz_variance >= 1, `variance ${r.pos_horiz_variance}`);
});

test('battery: deterministic dt-integrated drain, sane wire values', () => {
  let a = createBattery();
  let b = createBattery();
  for (let i = 0; i < 60 * 60; i++) {
    a = stepBattery(a, 0.7, DT);
    b = stepBattery(b, 0.7, DT);
  }
  assert.equal(a.soc, b.soc, 'bit-identical across reruns');
  const drained = 1 - a.soc;
  const expected = (batteryCurrentA(0.7) * 60) / 3600 / 5; // 1 min at cruise
  assert.ok(Math.abs(drained - expected) < 1e-9, `drain ${drained} vs ${expected}`);

  const out = batteryOutputs(a, 0.7);
  assert.ok(out.battMv > 10500 && out.battMv < 12600, `voltage ${out.battMv}`);
  assert.ok(out.battPct >= 96 && out.battPct <= 99, `pct ${out.battPct}`);
  assert.equal(out.battCa, Math.round(batteryCurrentA(0.7) * 100));

  let dead = createBattery();
  for (let i = 0; i < 4 * 3600 * 10; i++) dead = stepBattery(dead, 1, 0.1);
  assert.equal(dead.soc, 0, 'clamps at empty');
});
