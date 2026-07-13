// M17–M18 gates: wind estimation + crab compensation + GCS stick.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createVehicle, vehicleStep, vehicleCommand } from '../src/vehicle.js';
import { MODES } from '../src/autopilot.js';

const DT = 1 / 60;

function cruise(params, seconds, cmds = []) {
  let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22, params });
  v = vehicleCommand(v, { type: 'mode', custom: MODES.GUIDED });
  for (const c of cmds) v = vehicleCommand(v, c);
  const track = [];
  for (let i = 0; i < seconds * 60; i++) {
    v = vehicleStep(v, DT);
    if (i % 30 === 0) track.push([...v.state.pos]);
  }
  return { v, track };
}

test('wind estimator converges on the true steady wind', () => {
  const { v } = cruise({ WND_TRB: 0.5, WND_N_MS: -6, WND_E_MS: 4 }, 60);
  assert.ok(Math.abs(v.windEst.n - -6) < 1.5, `windN est ${v.windEst.n} (true −6)`);
  assert.ok(Math.abs(v.windEst.e - 4) < 1.5, `windE est ${v.windEst.e} (true 4)`);
});

test('crab compensation: a crosswind hold now tracks the course, not just heading', () => {
  // GUIDED hold captures course 0 (north) at boot; 8 m/s crosswind from the west.
  const { v, track } = cruise({ WND_TRB: 0, WND_E_MS: 8 }, 90);
  // Give the wind estimator 30 s to spin up, then measure eastward track drift rate.
  const late = track.filter((_, i) => i * 0.5 >= 30);
  const driftRate = (v.state.pos[0] - late[0][0]) / (late.length * 0.5);
  assert.ok(Math.abs(driftRate) < 2, `east drift ${driftRate} m/s with crab compensation`);
});

test('MANUAL_CONTROL stick: fresh GCS stick drives the surfaces, then expires', () => {
  let v = createVehicle({ boot: 'air', sensorSeed: 21, windSeed: 22 });
  v = { ...v, armed: true };
  v = vehicleCommand(v, { type: 'stick', pitch: 0, roll: 1, yaw: 0, throttle: 0.8 });
  for (let i = 0; i < 30; i++) v = vehicleStep(v, DT);
  assert.ok(v.state.act.da > 0.1, `roll stick → aileron (${v.state.act.da})`);
  assert.ok(v.state.act.dt > 0.4, 'throttle follows the stick');
  // 1 s freshness gate: with no new stick packets the vehicle reverts to neutral.
  for (let i = 0; i < 90; i++) v = vehicleStep(v, DT);
  assert.ok(Math.abs(v.state.act.da) < 0.05, `stale stick released the aileron (${v.state.act.da})`);
});
