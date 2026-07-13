import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepAircraft, initialState, airData, TRIM, AC } from '../src/physics.js';
import {
  MODES, apStep, holdControls, manualControls, headingErrorDeg, bearingToDeg,
} from '../src/autopilot.js';
import { eulerFromQuat, headingDeg } from '../src/telemetry.js';

const DT = 1 / 60;
const AP0 = {
  landing: false, targetAlt: 120, targetHeading: 0, guided: null, mission: null,
};

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

test('hold: altitude, heading, and airspeed converge from cruise', () => {
  const s = flyHold(initialState(), 60, 120, 0);
  const e = eulerFromQuat(s.quat);
  const ad = airData(s.quat, s.vel);
  assert.ok(Math.abs(s.pos[1] - 120) < 5, `alt = ${s.pos[1]}`);
  assert.ok(Math.abs(headingErrorDeg(0, headingDeg(e.yaw))) < 3, `hdg = ${headingDeg(e.yaw)}`);
  assert.ok(Math.abs(ad.Va - 30) < 2, `Va = ${ad.Va}`);
});

test('hold: climb 80 m to a new altitude without losing airspeed', () => {
  const s = flyHold(initialState(), 60, 200, 0);
  const ad = airData(s.quat, s.vel);
  assert.ok(Math.abs(s.pos[1] - 200) < 5, `alt = ${s.pos[1]}`);
  assert.ok(ad.Va > 27, `Va sagged to ${ad.Va}`);
});

test('hold: a 90° turn is coordinated and keeps the altitude band', () => {
  let s = initialState();
  let minAlt = Infinity;
  let maxBeta = 0;
  for (let i = 0; i < 60 * 60; i++) {
    s = stepAircraft(s, holdControls(s, 120, 90), DT);
    minAlt = Math.min(minAlt, s.pos[1]);
    if (i > 5 * 60) maxBeta = Math.max(maxBeta, Math.abs(airData(s.quat, s.vel).beta));
  }
  const e = eulerFromQuat(s.quat);
  assert.ok(Math.abs(headingErrorDeg(90, headingDeg(e.yaw))) < 5, `hdg = ${headingDeg(e.yaw)}`);
  assert.ok(minAlt > 110, `alt sagged to ${minAlt}`);
  assert.ok(maxBeta < 0.02, `sideslip in turn = ${(maxBeta * 57.3).toFixed(2)}°`);
});

test('manual SAS: stick signs map to the right surfaces', () => {
  const s = initialState();
  const neutral = manualControls(s, { pitch: 0, roll: 0, yaw: 0, throttle: 0.6 });
  const rollR = manualControls(s, { pitch: 0, roll: 1, yaw: 0, throttle: 0.6 });
  const pitchUp = manualControls(s, { pitch: 1, roll: 0, yaw: 0, throttle: 0.6 });
  const yawR = manualControls(s, { pitch: 0, roll: 0, yaw: 1, throttle: 0.6 });
  assert.ok(rollR.aileron > neutral.aileron, 'roll-right stick → aileron+');
  assert.ok(pitchUp.elevator < neutral.elevator, 'pitch-up stick → elevator− (Cmde<0)');
  assert.ok(yawR.rudder < neutral.rudder, 'yaw-right stick → rudder− (Cndr<0)');
  assert.equal(neutral.throttle, 0.6);
});

test('takeoff: climbs to the target altitude then hands off to GUIDED hold', () => {
  let s = { ...initialState(), pos: [0, 5, 0], vel: [0, 0, -25] };
  let ap = { ...AP0, mode: MODES.TAKEOFF, targetAlt: 80 };
  for (let i = 0; i < 60 * 60; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    s = stepAircraft(s, r.controls, DT);
  }
  assert.equal(ap.mode, MODES.GUIDED, 'transitions to GUIDED at altitude');
  assert.ok(Math.abs(s.pos[1] - 80) < 15, `alt = ${s.pos[1]}`);
});

test('rtl: flies home from 2 km out and lands gently near home', () => {
  let s = { ...initialState(), pos: [0, 120, -2000] };
  let ap = { ...AP0, mode: MODES.RTL };
  let disarmed = false;
  let touchdownSink = null;
  let prevVy = 0;
  for (let i = 0; i < 400 * 60 && !disarmed; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    disarmed = r.disarm;
    if (!disarmed) {
      prevVy = s.vel[1];
      s = stepAircraft(s, r.controls, DT);
      if (s.pos[1] === 0 && touchdownSink === null) touchdownSink = prevVy;
    }
  }
  assert.ok(ap.landing, 'entered the landing phase');
  assert.ok(disarmed, 'disarmed after touchdown');
  assert.ok(touchdownSink > -3.5, `touchdown sink ${touchdownSink} m/s`);
  assert.ok(Math.hypot(s.pos[0], s.pos[2]) < 600, `stopped ${Math.hypot(s.pos[0], s.pos[2])} m from home`);
});

test('determinism: a full RTL flight is bit-identical across reruns', () => {
  const fly = () => {
    let s = { ...initialState(), pos: [300, 120, -1500] };
    let ap = { ...AP0, mode: MODES.RTL };
    for (let i = 0; i < 60 * 60; i++) {
      const r = apStep(s, ap);
      ap = r.ap;
      s = stepAircraft(s, r.controls, DT);
    }
    return JSON.stringify({ s, ap });
  };
  assert.equal(fly(), fly());
});
