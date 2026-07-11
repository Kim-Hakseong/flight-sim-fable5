import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  G, MASS, stepAircraft, initialState,
  quatMultiply, quatNormalize, quatRotate, quatIntegrate,
} from '../src/physics.js';

const DT = 1 / 60;

function run(steps, controlsAt) {
  let s = initialState();
  for (let i = 0; i < steps; i++) s = stepAircraft(s, controlsAt(i), DT);
  return s;
}

test('determinism: identical inputs produce bit-identical state', () => {
  const controls = (i) => ({
    pitch: i > 120 ? 0.4 : 0,
    roll: i > 240 ? -0.6 : 0,
    yaw: i > 360 ? 0.2 : 0,
    throttle: 0.8,
  });
  const a = run(600, controls);
  const b = run(600, controls);
  assert.equal(JSON.stringify(a), JSON.stringify(b));
});

test('purity: stepAircraft does not mutate its input state', () => {
  const s = initialState();
  Object.freeze(s);
  s.pos && [s.pos, s.vel, s.quat, s.omega].forEach(Object.freeze);
  const before = JSON.stringify(s);
  stepAircraft(s, { pitch: 1, roll: 1, yaw: 1, throttle: 1 }, DT);
  assert.equal(JSON.stringify(s), before);
});

test('gravity: no thrust, no airspeed → free fall near −g·t', () => {
  let s = { ...initialState(), vel: [0, 0, 0] };
  for (let i = 0; i < 60; i++) {
    s = stepAircraft(s, { pitch: 0, roll: 0, yaw: 0, throttle: 0 }, DT);
  }
  assert.ok(s.vel[1] < -9 && s.vel[1] > -G - 0.1, `vy after 1 s = ${s.vel[1]}`);
});

test('thrust: full throttle from rest accelerates along the nose (−Z)', () => {
  let s = { ...initialState(), vel: [0, 0, 0] };
  for (let i = 0; i < 60; i++) {
    s = stepAircraft(s, { pitch: 0, roll: 0, yaw: 0, throttle: 1 }, DT);
  }
  assert.ok(s.vel[2] < -5, `vz after 1 s = ${s.vel[2]}`); // ≈ −6000/1000·1 s minus drag
});

test('cruise: moderate throttle keeps it flying and climbing-capable', () => {
  const s = run(30 * 60, () => ({ pitch: 0, roll: 0, yaw: 0, throttle: 0.7 }));
  const spd = Math.hypot(...s.vel);
  assert.ok(s.pos[1] > 50, `alt after 30 s = ${s.pos[1]}`);
  assert.ok(spd > 30 && spd < 80, `speed after 30 s = ${spd}`);
});

test('ground: falling through y=0 clamps and kills sink rate', () => {
  let s = { ...initialState(), pos: [0, 0.5, 0], vel: [10, -20, -10] };
  for (let i = 0; i < 5; i++) {
    s = stepAircraft(s, { pitch: 0, roll: 0, yaw: 0, throttle: 0 }, DT);
  }
  assert.equal(s.pos[1], 0);
  assert.ok(s.vel[1] >= 0);
});

test('quaternion stays normalized through sustained rotation', () => {
  const s = run(1200, () => ({ pitch: 0.5, roll: 0.8, yaw: 0.3, throttle: 0.6 }));
  const n = Math.hypot(...s.quat);
  assert.ok(Math.abs(n - 1) < 1e-9, `|q| = ${n}`);
});

test('quat math: rotate identity is a no-op; multiply/normalize behave', () => {
  assert.deepEqual(quatRotate([0, 0, 0, 1], [1, 2, 3]), [1, 2, 3]);
  const id = quatMultiply([0, 0, 0, 1], [0, 0, 0, 1]);
  assert.deepEqual(id, [0, 0, 0, 1]);
  const n = quatNormalize([2, 0, 0, 0]);
  assert.deepEqual(n, [1, 0, 0, 0]);
});

test('quat integration: +X body rate pitches the nose (−Z) upward', () => {
  // 90° at 1 rad/s ⇒ π/2 s of integration; small steps keep it accurate.
  let q = [0, 0, 0, 1];
  const steps = 1000;
  const dt = (Math.PI / 2) / steps;
  for (let i = 0; i < steps; i++) q = quatIntegrate(q, [1, 0, 0], dt);
  const nose = quatRotate(q, [0, 0, -1]);
  assert.ok(nose[1] > 0.99, `nose after 90° pitch-up = ${nose}`); // now pointing +Y (up)
});

test('control signs: roll-right input banks right wing down', () => {
  // 0.5 s only — a longer full-stick roll wraps past 180° and inverts the sign.
  const s = run(30, () => ({ pitch: 0, roll: 1, yaw: 0, throttle: 0.7 }));
  const rightWing = quatRotate(s.quat, [1, 0, 0]);
  assert.ok(rightWing[1] < -0.1, `right wing tip y = ${rightWing[1]}`);
});

test('control signs: yaw-right input swings the nose toward +X', () => {
  const s = run(120, () => ({ pitch: 0, roll: 0, yaw: 1, throttle: 0.7 }));
  const nose = quatRotate(s.quat, [0, 0, -1]);
  assert.ok(nose[0] > 0.05, `nose x after yaw-right = ${nose[0]}`);
});
