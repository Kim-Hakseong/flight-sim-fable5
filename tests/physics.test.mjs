import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  G, AC, TRIM, stepAircraft, initialState, airData, airDensity, forcesMoments,
  quatMultiply, quatNormalize, quatRotate, quatIntegrate, toFRD, fromFRD,
} from '../src/physics.js';

const DT = 1 / 60;
const trimCmd = () => ({
  aileron: 0, elevator: TRIM.de / AC.maxDef, rudder: 0, throttle: TRIM.dt,
});

function run(steps, cmdsAt) {
  let s = initialState();
  for (let i = 0; i < steps; i++) s = stepAircraft(s, cmdsAt(i), DT);
  return s;
}

test('determinism: identical inputs produce bit-identical state', () => {
  const cmds = (i) => ({
    aileron: i > 120 ? 0.3 : 0, elevator: TRIM.de / AC.maxDef + (i > 240 ? -0.2 : 0),
    rudder: i > 360 ? 0.1 : 0, throttle: 0.7,
  });
  assert.equal(JSON.stringify(run(600, cmds)), JSON.stringify(run(600, cmds)));
});

test('purity: stepAircraft does not mutate its input state', () => {
  const s = initialState();
  [s.pos, s.vel, s.quat, s.omega, s.act, s].forEach(Object.freeze);
  const before = JSON.stringify(s);
  stepAircraft(s, { aileron: 1, elevator: 1, rudder: 1, throttle: 1 }, DT);
  assert.equal(JSON.stringify(s), before);
});

test('trim: the solved trim actually holds level flight for 60 s', () => {
  const s = run(60 * 60, trimCmd);
  const ad = airData(s.quat, s.vel);
  assert.ok(Math.abs(s.pos[1] - 120) < 2, `alt drifted to ${s.pos[1]}`);
  assert.ok(Math.abs(ad.Va - TRIM.Va) < 0.5, `Va drifted to ${ad.Va}`);
  assert.ok(Math.abs(ad.beta) < 1e-6, `beta = ${ad.beta}`);
});

test('gravity: idle power at rest → free fall near −g·t', () => {
  let s = { ...initialState(), vel: [0, 0, 0], act: { da: 0, de: 0, dr: 0, dt: 0 } };
  for (let i = 0; i < 60; i++) {
    s = stepAircraft(s, { aileron: 0, elevator: 0, rudder: 0, throttle: 0 }, DT);
  }
  assert.ok(s.vel[1] < -9 && s.vel[1] > -G - 0.2, `vy after 1 s = ${s.vel[1]}`);
});

test('surface signs: aileron+ rolls right, elevator− pitches up, rudder− yaws right', () => {
  const right = quatRotate(run(60, () => ({ ...trimCmd(), aileron: 0.4 })).quat, [1, 0, 0]);
  assert.ok(right[1] < -0.05, `right wing y after aileron+ = ${right[1]}`);

  const upNose = quatRotate(run(60, () => ({ ...trimCmd(), elevator: TRIM.de / AC.maxDef - 0.3 })).quat, [0, 0, -1]);
  const trimNose = quatRotate(run(60, trimCmd).quat, [0, 0, -1]);
  assert.ok(upNose[1] > trimNose[1] + 0.02, `nose y ${upNose[1]} vs trim ${trimNose[1]}`);

  const yawNose = quatRotate(run(90, () => ({ ...trimCmd(), rudder: -0.4 })).quat, [0, 0, -1]);
  assert.ok(yawNose[0] > 0.02, `nose x after rudder− = ${yawNose[0]}`);
});

test('actuators: first-order lag and hard deflection limits', () => {
  let s = initialState();
  s = stepAircraft(s, { aileron: 5, elevator: 0, rudder: 0, throttle: 2 }, DT);
  assert.ok(s.act.da <= AC.maxDef + 1e-12 && s.act.dt <= 1);
  const first = s.act.da;
  for (let i = 0; i < 60; i++) s = stepAircraft(s, { aileron: 5, elevator: 0, rudder: 0, throttle: 1 }, DT);
  assert.ok(first < AC.maxDef * 0.5, 'lag: one step must not saturate');
  assert.ok(Math.abs(s.act.da - AC.maxDef) < 1e-6, 'converges to the limit');
});

test('air data: alpha+ when nose above velocity, beta+ for flow from the right', () => {
  const level = airData([0, 0, 0, 1], [0, -3, -30]);
  assert.ok(level.alpha > 0.09 && level.alpha < 0.11, `alpha = ${level.alpha}`);
  const side = airData([0, 0, 0, 1], [3, 0, -30]);
  assert.ok(side.beta > 0.09 && side.beta < 0.11, `beta = ${side.beta}`);
  assert.equal(airData([0, 0, 0, 1], [0, 0, -0.5]).alpha, 0, 'no aero angles when nearly still');
});

test('atmosphere: density falls with altitude (ISA)', () => {
  assert.ok(Math.abs(airDensity(0) - 1.225) < 1e-9);
  assert.ok(airDensity(1000) < airDensity(0) && airDensity(1000) > 1.0);
});

test('dynamic stability: dutch roll damps after a rudder pulse', () => {
  let s = initialState();
  for (let i = 0; i < 30 * 60; i++) {
    const c = i < 30 ? { ...trimCmd(), rudder: 0.4 } : trimCmd();
    s = stepAircraft(s, c, DT);
  }
  const ad = airData(s.quat, s.vel);
  assert.ok(Math.abs(ad.beta) < 0.01, `residual beta = ${ad.beta}`);
});

test('propulsion: thrust rises with throttle, falls with airspeed', () => {
  const at = (dt, Va) => forcesMoments([0, 0, 0, 1], [0, 0, -Va], [0, 0, 0], { da: 0, de: 0, dr: 0, dt }, 100);
  // Isolate thrust via Fx difference at the same flight condition.
  const dF = at(0.8, 30).F[0] - at(0.2, 30).F[0];
  assert.ok(dF > 50, `throttle authority ${dF} N`);
  const dV = at(0.6, 20).F[0] - at(0.6, 40).F[0];
  assert.ok(dV > 0, 'thrust must decay with airspeed');
});

test('frame conversion: FRD round-trip and rate mapping', () => {
  const eq = (a, b, what) => assert.ok(a.every((v, i) => v === b[i]), `${what}: ${a} ≠ ${b}`); // == : tolerates −0
  eq(fromFRD(toFRD([1, 2, 3])), [1, 2, 3], 'round-trip');
  eq(toFRD([0, 0, -1]), [1, 0, 0], 'nose (−Z ours) = FRD x');
  eq(toFRD([1, 0, 0]), [0, 1, 0], 'right wing = FRD y');
  eq(toFRD([0, 1, 0]), [0, 0, -1], 'top (+Y ours) = −FRD z (belly)');
});

test('quat math: identity ops and pitch-up integration', () => {
  assert.deepEqual(quatRotate([0, 0, 0, 1], [1, 2, 3]), [1, 2, 3]);
  assert.deepEqual(quatMultiply([0, 0, 0, 1], [0, 0, 0, 1]), [0, 0, 0, 1]);
  assert.deepEqual(quatNormalize([2, 0, 0, 0]), [1, 0, 0, 0]);
  let q = [0, 0, 0, 1];
  const steps = 1000;
  const dt = (Math.PI / 2) / steps;
  for (let i = 0; i < steps; i++) q = quatIntegrate(q, [1, 0, 0], dt);
  const nose = quatRotate(q, [0, 0, -1]);
  assert.ok(nose[1] > 0.99, `nose after 90° pitch-up = ${nose}`);
});
