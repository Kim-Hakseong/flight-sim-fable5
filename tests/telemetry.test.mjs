import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  HOME, localToGeodetic, eulerFromQuat, bodyRatesToFrd, headingDeg, telemetryFrom,
} from '../src/telemetry.js';
import { initialState, quatIntegrate } from '../src/physics.js';

const DEG = Math.PI / 180;

function quatFromRate(omega, angleRad, steps = 2000) {
  let q = [0, 0, 0, 1];
  const dt = angleRad / steps;
  for (let i = 0; i < steps; i++) q = quatIntegrate(q, omega, dt);
  return q;
}

test('geodetic: home maps to home; 1 km north ≈ +0.009° lat', () => {
  const atHome = localToGeodetic([0, 0, 0]);
  assert.equal(atHome.lat, HOME.lat);
  assert.equal(atHome.lon, HOME.lon);
  assert.equal(atHome.alt, HOME.alt);

  const north1km = localToGeodetic([0, 100, -1000]); // north = −Z
  assert.ok(Math.abs(north1km.lat - HOME.lat - 0.008993) < 1e-4, `lat=${north1km.lat}`);
  assert.equal(north1km.lon, HOME.lon);
  assert.equal(north1km.alt, HOME.alt + 100);

  const east1km = localToGeodetic([1000, 0, 0]);
  assert.ok(east1km.lon > HOME.lon, 'east must increase lon');
});

test('euler: identity attitude is level, heading north', () => {
  const e = eulerFromQuat([0, 0, 0, 1]);
  assert.ok(Math.abs(e.roll) < 1e-9 && Math.abs(e.pitch) < 1e-9 && Math.abs(e.yaw) < 1e-9);
  assert.equal(headingDeg(e.yaw), 0);
});

test('euler: signs — pitch-up, roll-right, yaw-right all come out positive', () => {
  const up30 = eulerFromQuat(quatFromRate([1, 0, 0], 30 * DEG));
  assert.ok(Math.abs(up30.pitch - 30 * DEG) < 1e-3, `pitch=${up30.pitch}`);

  const rollRight20 = eulerFromQuat(quatFromRate([0, 0, -1], 20 * DEG));
  assert.ok(Math.abs(rollRight20.roll - 20 * DEG) < 1e-3, `roll=${rollRight20.roll}`);

  const yawRight90 = eulerFromQuat(quatFromRate([0, -1, 0], 90 * DEG));
  assert.ok(Math.abs(yawRight90.yaw - 90 * DEG) < 1e-3, `yaw=${yawRight90.yaw}`);
  assert.ok(Math.abs(headingDeg(yawRight90.yaw) - 90) < 0.1); // nose east
});

test('body rates map to FRD roll/pitch/yaw speeds with correct signs', () => {
  const r = bodyRatesToFrd([0.1, -0.2, -0.3]);
  assert.equal(r.pitchspeed, 0.1); // +X body = pitch-up
  assert.equal(r.yawspeed, 0.2); // −Y body = yaw-right
  assert.equal(r.rollspeed, 0.3); // −Z body = roll-right
});

test('telemetryFrom: initial cruise state → sane wire-ready values', () => {
  const t = telemetryFrom(initialState(), 0.65, 12.5);
  assert.equal(t.timeBootMs, 12500);
  assert.ok(Math.abs(t.lat - HOME.lat) < 1e-9 && Math.abs(t.lon - HOME.lon) < 1e-9);
  assert.equal(t.relAlt, 120);
  assert.equal(t.alt, HOME.alt + 120);
  assert.ok(Math.abs(t.vn - 30) < 1e-9, 'flying north at 30 m/s'); // vel −Z ⇒ vn +
  assert.ok(t.ve === 0 && t.vd === 0); // == comparison: vd is −0 (negated 0)
  assert.ok(Math.abs(t.groundspeed - 30) < 1e-9);
  assert.equal(t.headingDeg, 0);
  assert.equal(t.throttlePct, 65);
});
