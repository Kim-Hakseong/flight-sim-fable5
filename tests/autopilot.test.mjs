import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepAircraft, initialState, angleOfAttack, quatIntegrate } from '../src/physics.js';
import { MODES, apStep, holdControls, headingErrorDeg, bearingToDeg } from '../src/autopilot.js';
import { eulerFromQuat, headingDeg } from '../src/telemetry.js';

const DT = 1 / 60;

function flyHold(state, seconds, alt, hdg) {
  let s = state;
  for (let i = 0; i < seconds * 60; i++) s = stepAircraft(s, holdControls(s, alt, hdg), DT);
  return s;
}

test('headingErrorDeg: shortest signed turn, both ways across north', () => {
  assert.equal(headingErrorDeg(10, 350), 20);
  assert.equal(headingErrorDeg(350, 10), -20);
  assert.equal(headingErrorDeg(180, 0), -180); // antipodal: either sign is a valid turn
});

test('bearingToDeg: cardinal directions to home', () => {
  assert.equal(bearingToDeg([0, 0, 1000]), 0); // south of home → fly north
  assert.equal(bearingToDeg([0, 0, -1000]), 180); // north of home → fly south
  assert.equal(bearingToDeg([-1000, 0, 0]), 90); // west of home → fly east
});

test('angle of attack: positive when the nose is above the velocity vector', () => {
  // Pitch the nose up 10° while still flying level: AoA should read ≈ +10°.
  let q = [0, 0, 0, 1];
  const steps = 500;
  for (let i = 0; i < steps; i++) q = quatIntegrate(q, [(10 * Math.PI) / 180 / steps / DT, 0, 0], DT);
  const alpha = angleOfAttack(q, [0, 0, -40]);
  assert.ok(Math.abs(alpha - (10 * Math.PI) / 180) < 1e-3, `alpha = ${alpha}`);
  assert.equal(angleOfAttack([0, 0, 0, 1], [0, 0, -0.5]), 0, 'no aero angles when nearly still');
});

test('hold: altitude and heading converge from cruise', () => {
  const s = flyHold(initialState(), 60, 120, 0);
  const e = eulerFromQuat(s.quat);
  assert.ok(Math.abs(s.pos[1] - 120) < 10, `alt = ${s.pos[1]}`);
  assert.ok(Math.abs(headingErrorDeg(0, headingDeg(e.yaw))) < 3, `hdg = ${headingDeg(e.yaw)}`);
  const spd = Math.hypot(...s.vel);
  assert.ok(spd > 30 && spd < 60, `speed = ${spd}`);
});

test('hold: a 90° turn completes without losing the altitude band', () => {
  const s = flyHold(initialState(), 90, 120, 90);
  const e = eulerFromQuat(s.quat);
  assert.ok(Math.abs(headingErrorDeg(90, headingDeg(e.yaw))) < 5, `hdg = ${headingDeg(e.yaw)}`);
  assert.ok(Math.abs(s.pos[1] - 120) < 25, `alt = ${s.pos[1]}`);
});

test('takeoff: climbs to the target altitude then hands off to GUIDED hold', () => {
  let s = { ...initialState(), pos: [0, 5, 0], vel: [0, 0, -30] };
  let ap = { mode: MODES.TAKEOFF, landing: false, targetAlt: 80, targetHeading: 0 };
  for (let i = 0; i < 60 * 60; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    s = stepAircraft(s, r.controls, DT);
  }
  assert.equal(ap.mode, MODES.GUIDED, 'transitions to GUIDED at altitude');
  assert.ok(Math.abs(s.pos[1] - 80) < 15, `alt = ${s.pos[1]}`);
});

test('rtl: flies home from 2 km out, lands, and disarms on touchdown', () => {
  let s = { ...initialState(), pos: [0, 120, -2000] };
  let ap = { mode: MODES.RTL, landing: false, targetAlt: 120, targetHeading: 0 };
  let disarmed = false;
  for (let i = 0; i < 300 * 60 && !disarmed; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    disarmed = r.disarm;
    if (!disarmed) s = stepAircraft(s, r.controls, DT);
  }
  assert.ok(ap.landing, 'switched to landing near home');
  assert.ok(disarmed, 'disarmed after touchdown');
  assert.equal(s.pos[1], 0, 'on the ground');
  assert.ok(Math.hypot(s.pos[0], s.pos[2]) < 800, `came down ${Math.hypot(s.pos[0], s.pos[2])} m from home`);
});

test('determinism: a full RTL flight is bit-identical across reruns', () => {
  const fly = () => {
    let s = { ...initialState(), pos: [300, 120, -1500] };
    let ap = { mode: MODES.RTL, landing: false, targetAlt: 120, targetHeading: 0 };
    for (let i = 0; i < 60 * 60; i++) {
      const r = apStep(s, ap);
      ap = r.ap;
      s = stepAircraft(s, r.controls, DT);
    }
    return JSON.stringify({ s, ap });
  };
  assert.equal(fly(), fly());
});
