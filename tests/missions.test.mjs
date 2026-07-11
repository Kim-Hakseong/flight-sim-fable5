import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepAircraft, initialState } from '../src/physics.js';
import { MODES, apStep, LOITER_RADIUS_M } from '../src/autopilot.js';
import { toLocalTargets, missionStep, horizontalDistance, CMD, DEFAULT_ACCEPT_M } from '../src/missions.js';
import { HOME, localToGeodetic, geodeticToLocal } from '../src/telemetry.js';

const DT = 1 / 60;

test('geodetic round-trip: local → geo → local is metre-accurate', () => {
  for (const pos of [[1500, 80, -2200], [-300, 10, 400], [0, 0, 0]]) {
    const back = geodeticToLocal(localToGeodetic(pos));
    assert.ok(Math.hypot(back[0] - pos[0], back[2] - pos[2]) < 0.01, `xz ${back}`);
    assert.ok(Math.abs(back[1] - pos[1]) < 1e-9, `alt ${back[1]}`);
  }
});

test('toLocalTargets: frames, altitude floor, and acceptance radius', () => {
  const [rel, amsl, tight] = toLocalTargets([
    { seq: 0, command: 16, frame: 3, lat: HOME.lat, lon: HOME.lon, alt: 120, param1: 0, param2: 0 },
    { seq: 1, command: 16, frame: 0, lat: HOME.lat, lon: HOME.lon, alt: HOME.alt + 90, param1: 0, param2: 0 },
    { seq: 2, command: 16, frame: 3, lat: HOME.lat, lon: HOME.lon, alt: 5, param1: 0, param2: 25 },
  ]);
  assert.equal(rel.alt, 120); // frame 3: already relative
  assert.equal(amsl.alt, 90); // frame 0: AMSL minus home elevation
  assert.equal(tight.alt, 20); // floor: never command below 20 m
  assert.equal(rel.accept, DEFAULT_ACCEPT_M);
  assert.equal(tight.accept, 25); // NAV_WAYPOINT.param2
});

test('missionStep: advances through reached and unsupported items', () => {
  const targets = [
    { seq: 0, command: CMD.WAYPOINT, x: 0, z: -10, alt: 120, accept: 60 }, // already inside
    { seq: 1, command: 177, x: 0, z: 0, alt: 0, accept: 60 }, // DO_JUMP-ish: skip
    { seq: 2, command: CMD.WAYPOINT, x: 5000, z: 0, alt: 120, accept: 60 },
  ];
  const r = missionStep([0, 120, 0], { targets, idx: 0 });
  assert.deepEqual(r.reached, [0]);
  assert.equal(r.mission.idx, 2);
  assert.equal(r.action, 'fly');
  assert.equal(r.target.seq, 2);

  const done = missionStep([5000, 120, -20], { targets, idx: 2 });
  assert.equal(done.action, 'done');
  assert.deepEqual(done.reached, [2]);
});

test('missionStep: LAND and RTL items surface as actions', () => {
  const land = missionStep([0, 100, 0], {
    targets: [{ seq: 0, command: CMD.LAND, x: 900, z: 0, alt: 20, accept: 60 }], idx: 0,
  });
  assert.equal(land.action, 'land');
  const rtl = missionStep([0, 100, 0], {
    targets: [{ seq: 0, command: CMD.RTL, x: 0, z: 0, alt: 0, accept: 60 }], idx: 0,
  });
  assert.equal(rtl.action, 'rtl');
});

test('AUTO: flies a 3-waypoint plan end to end, reporting each reach', () => {
  const items = [
    { seq: 0, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon, alt: 120, param1: 0, param2: 0 },
    { seq: 1, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon + 0.014, alt: 150, param1: 0, param2: 0 },
    { seq: 2, command: 16, frame: 3, lat: HOME.lat, lon: HOME.lon + 0.014, alt: 100, param1: 0, param2: 0 },
  ];
  let s = initialState();
  let ap = {
    mode: MODES.AUTO, landing: false, targetAlt: 120, targetHeading: 0,
    guided: null, mission: { targets: toLocalTargets(items), idx: 0 },
  };
  const reached = [];
  for (let i = 0; i < 240 * 60 && ap.mission.idx < 3; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    reached.push(...r.reached);
    s = stepAircraft(s, r.controls, DT);
  }
  assert.deepEqual(reached, [0, 1, 2]);
  assert.ok(s.pos[1] > 60, `still flying at alt ${s.pos[1]}`);
});

test('GUIDED go-to: reaches the point and holds a bounded orbit', () => {
  let s = initialState();
  const t = { x: 2000, z: 0, alt: 140 };
  let ap = {
    mode: MODES.GUIDED, landing: false, targetAlt: 120, targetHeading: 0,
    guided: t, mission: null,
  };
  let arrived = null;
  let maxAfterSettle = 0;
  let minAlt = Infinity;
  for (let i = 0; i < 240 * 60; i++) {
    const r = apStep(s, ap);
    ap = r.ap;
    s = stepAircraft(s, r.controls, DT);
    const d = horizontalDistance(s.pos, t);
    if (arrived === null && d < 1.5 * LOITER_RADIUS_M) arrived = i / 60;
    else if (arrived !== null && i / 60 > arrived + 60) {
      maxAfterSettle = Math.max(maxAfterSettle, d);
      minAlt = Math.min(minAlt, s.pos[1]);
    }
  }
  assert.ok(arrived !== null && arrived < 90, `arrived at t=${arrived}`);
  assert.ok(maxAfterSettle < 2 * LOITER_RADIUS_M, `orbit drifted to ${maxAfterSettle} m`);
  assert.ok(minAlt > 100, `altitude sagged to ${minAlt} m in the orbit`);
});
