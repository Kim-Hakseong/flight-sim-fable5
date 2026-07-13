// M13 gate: ground model + ground-roll takeoff.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepAircraft, groundState, airData, quatRotate, AC } from '../src/physics.js';
import { MODES, apStep, TAKEOFF_VR_MS } from '../src/autopilot.js';

const DT = 1 / 60;
const IDLE = { aileron: 0, elevator: 0, rudder: 0, throttle: 0 };
const AP0 = { landing: false, targetAlt: 60, targetHeading: 0, guided: null, mission: null };

test('ground rest: cold aircraft sits still and level indefinitely', () => {
  let s = groundState();
  for (let i = 0; i < 20 * 60; i++) s = stepAircraft(s, IDLE, DT);
  assert.ok(Math.hypot(...s.vel) < 1e-6, `crept to ${Math.hypot(...s.vel)} m/s`);
  assert.equal(s.pos[1], 0);
  const right = quatRotate(s.quat, [1, 0, 0]);
  assert.ok(Math.abs(right[1]) < 0.01, `roll leaked: right-wing y = ${right[1]}`);
});

test('static thrust is capped to the realistic value', () => {
  let s = { ...groundState(), act: { da: 0, de: 0, dr: 0, dt: 1 } };
  const before = [...s.vel];
  s = stepAircraft(s, { ...IDLE, throttle: 1 }, DT);
  const accel = Math.hypot(s.vel[0] - before[0], s.vel[2] - before[2]) / DT;
  assert.ok(accel < (AC.maxThrustN / AC.mass) + 0.5, `ground accel ${accel} m/s²`);
});

test('auto-brake: idle throttle stops a 25 m/s rollout within 150 m', () => {
  let s = { ...groundState(), vel: [0, 0, -25] };
  let dist = 0;
  let stopped = false;
  for (let i = 0; i < 60 * 60 && !stopped; i++) {
    const prev = s.pos;
    s = stepAircraft(s, IDLE, DT);
    dist += Math.hypot(s.pos[0] - prev[0], s.pos[2] - prev[2]);
    stopped = Math.hypot(s.vel[0], s.vel[2]) < 0.5;
  }
  assert.ok(stopped, 'never stopped');
  assert.ok(dist < 150, `rollout ${dist} m`);
});

test('ground-roll takeoff: accelerate, rotate at Vr, climb out, hold centerline', () => {
  let s = groundState();
  let ap = { ...AP0, mode: MODES.TAKEOFF };
  let rollDist = null;
  let vaAtLiftoff = null;
  let maxCenterlineDev = 0;
  for (let i = 0; i < 120 * 60; i++) {
    const nav = { ...s, wow: s.pos[1] <= 0.5 };
    const r = apStep(nav, ap, undefined, airData(s.quat, s.vel).Va);
    ap = r.ap;
    s = stepAircraft(s, r.controls, DT);
    if (s.pos[1] <= 0.5) maxCenterlineDev = Math.max(maxCenterlineDev, Math.abs(s.pos[0]));
    if (rollDist === null && s.pos[1] > 1) {
      rollDist = 350 - s.pos[2];
      vaAtLiftoff = airData(s.quat, s.vel).Va;
    }
  }
  assert.ok(rollDist !== null && rollDist < 400, `roll distance ${rollDist} m`);
  assert.ok(vaAtLiftoff >= TAKEOFF_VR_MS - 1, `lifted off at Va ${vaAtLiftoff}`);
  assert.ok(maxCenterlineDev < 10, `centerline deviation ${maxCenterlineDev} m`);
  assert.equal(ap.mode, MODES.GUIDED, 'hands off to GUIDED at altitude');
  assert.ok(Math.abs(s.pos[1] - 60) < 15, `alt ${s.pos[1]}`);
});
