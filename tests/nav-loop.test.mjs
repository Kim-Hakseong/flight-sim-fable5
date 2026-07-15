// Full-stack gate on the SHARED vehicle implementation (src/vehicle.js): the
// control path sees only estimated state (nav, attitude, rates, pitot) + WoW.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  createVehicle, vehicleStep, vehicleCommand, vehicleFault, vehicleClearFault,
} from '../src/vehicle.js';
import { createEstimator, createAttEstimator } from '../src/estimator.js';
import { MODES } from '../src/autopilot.js';
import { HOME } from '../src/telemetry.js';

const DT = 1 / 60;

const PLAN_CMD = {
  type: 'mission',
  items: [
    { seq: 0, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon, alt: 120, param1: 0, param2: 0 },
    { seq: 1, command: 16, frame: 3, lat: HOME.lat + 0.012, lon: HOME.lon + 0.014, alt: 150, param1: 0, param2: 0 },
    { seq: 2, command: 16, frame: 3, lat: HOME.lat, lon: HOME.lon + 0.014, alt: 100, param1: 0, param2: 0 },
  ],
};

function airborneAuto() {
  let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22 });
  v = vehicleCommand(v, PLAN_CMD);
  return vehicleCommand(v, { type: 'mode', custom: MODES.AUTO });
}

test('estimated nav: a 3-waypoint mission completes in light turbulence', () => {
  let v = airborneAuto();
  for (let i = 0; i < 300 * 60 && v.ap.mission.idx < 3; i++) v = vehicleStep(v, DT);
  assert.equal(v.ap.mission.idx, 3, `stalled at waypoint ${v.ap.mission.idx}`);
  assert.equal(v.lastReached, 2);
  assert.ok(v.state.pos[1] > 60, `still flying at ${v.state.pos[1]} m`);
});

test('estimated nav: survives a 12 s GPS dropout mid-leg and still finishes', () => {
  let v = airborneAuto();
  let maxCoastErr = 0;
  for (let i = 0; i < 300 * 60 && v.ap.mission.idx < 3; i++) {
    if (i === 20 * 60) v = vehicleFault(v, 'gps', 'dropout');
    if (i === 32 * 60) v = vehicleClearFault(v, 'gps');
    v = vehicleStep(v, DT);
    if (i >= 20 * 60 && i < 32 * 60) {
      maxCoastErr = Math.max(maxCoastErr,
        Math.hypot(v.est.pos[0] - v.state.pos[0], v.est.pos[2] - v.state.pos[2]));
    }
  }
  assert.equal(v.ap.mission.idx, 3, 'mission must still complete');
  assert.ok(maxCoastErr < 80, `estimate drifted ${maxCoastErr} m while coasting`);
});

test('estimated nav: RTL lands and disarms on the WoW discrete', () => {
  let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22 });
  const state = { ...v.state, pos: [0, 120, -1500] };
  v = { ...v, state, est: createEstimator(state), att: createAttEstimator(state) };
  v = vehicleCommand(v, { type: 'rtl' });
  for (let i = 0; i < 400 * 60 && v.armed; i++) v = vehicleStep(v, DT);
  assert.ok(!v.armed, 'disarmed after touchdown');
  assert.equal(v.state.pos[1], 0, 'actually on the ground (truth)');
});

test('estimated nav: cold ground takeoff — the whole real-vehicle flow', () => {
  let v = createVehicle({ boot: 'ground', sensorSeed: 21, windSeed: 22 });
  v = vehicleCommand(v, { type: 'takeoff', alt: 60 });
  let lifted = false;
  for (let i = 0; i < 120 * 60; i++) {
    v = vehicleStep(v, DT);
    if (v.state.pos[1] > 1) lifted = true;
  }
  assert.ok(lifted, 'never left the ground');
  assert.equal(v.ap.mode, MODES.GUIDED, 'climb-out completed on estimated state');
  assert.ok(Math.abs(v.state.pos[1] - 60) < 20, `alt ${v.state.pos[1]}`);
});

test('estimated nav: AUTO started on the runway ground-rolls, lifts off, flies the mission', () => {
  let v = createVehicle({ boot: 'ground', sensorSeed: 21, windSeed: 22 });
  v = vehicleCommand(v, PLAN_CMD);
  v = vehicleCommand(v, { type: 'arm', value: 1 });
  v = vehicleCommand(v, { type: 'mode', custom: MODES.AUTO });
  let lifted = false;
  let maxYawRate = 0;
  for (let i = 0; i < 300 * 60 && v.lastReached < 0; i++) {
    v = vehicleStep(v, DT);
    if (v.state.pos[1] > 1) lifted = true;
    if (!lifted && i > 60) maxYawRate = Math.max(maxYawRate, Math.abs(v.state.omega[1]));
  }
  assert.ok(maxYawRate < 0.6, `ground-spins instead of rolling (yaw rate ${maxYawRate})`);
  assert.ok(lifted, 'never lifted off in AUTO');
  assert.ok(v.lastReached >= 0, 'never reached WP1');
});

test('determinism: the full closed loop is bit-identical across reruns', () => {
  const fly = () => {
    let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22 });
    v = vehicleCommand(v, { type: 'mode', custom: MODES.GUIDED });
    for (let i = 0; i < 30 * 60; i++) v = vehicleStep(v, DT);
    return JSON.stringify(v);
  };
  assert.equal(fly(), fly());
});

test('crash detection: a bad-attitude touchdown latches disarmed and stops', () => {
  let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22 });
  // slam it onto the runway inverted-ish with speed
  const bad = { ...v.state, pos: [0, 0.3, 0], vel: [20, -8, -20], quat: [0, 0, -0.7, 0.71], omega: [0, 0, 0] };
  v = { ...v, state: bad };
  for (let i = 0; i < 5 * 60; i++) v = vehicleStep(v, DT);
  assert.ok(v.crashed, 'crash not detected on a rolled-over touchdown');
  assert.ok(!v.armed, 'crashed vehicle still armed');
  assert.ok(v.state.act.dt < 0.02, 'engine not cut after crash');
  const restX = v.state.pos[0], restZ = v.state.pos[2];
  for (let i = 0; i < 10 * 60; i++) v = vehicleStep(v, DT);
  assert.ok(Math.hypot(v.state.pos[0] - restX, v.state.pos[2] - restZ) < 30, 'wreck skids away instead of stopping');
});
