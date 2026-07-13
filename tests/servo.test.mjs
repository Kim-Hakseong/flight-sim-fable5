// M16 gate: actuator fault mechanics + closed-loop survivability.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepActuators, stepAircraft, initialState, AC, TRIM } from '../src/physics.js';
import { createVehicle, vehicleStep, vehicleCommand, vehicleServoFault, vehicleClearServoFault } from '../src/vehicle.js';
import { MODES } from '../src/autopilot.js';

const DT = 1 / 60;
const CMDS = { aileron: 0.5, elevator: -0.3, rudder: 0.2, throttle: 0.8 };

test('jam: the servo holds its position and ignores commands', () => {
  let act = { da: 0.1, de: -0.05, dr: 0, dt: 0.6 };
  for (let i = 0; i < 120; i++) act = stepActuators(act, CMDS, DT, { da: { type: 'jam' } });
  assert.equal(act.da, 0.1, 'jammed channel frozen');
  assert.ok(Math.abs(act.de - -0.3 * AC.maxDef) < 1e-3, 'other channels track normally');
});

test('floating: the surface streams to aero-neutral regardless of command', () => {
  let act = { da: 0.3, de: -0.2, dr: 0.1, dt: 0.8 };
  for (let i = 0; i < 300; i++) {
    act = stepActuators(act, CMDS, DT, { de: { type: 'floating' }, dt: { type: 'floating' } });
  }
  assert.ok(Math.abs(act.de) < 1e-3, `elevator floats to 0 (${act.de})`);
  assert.ok(act.dt < 1e-3, 'floating throttle dies');
  assert.ok(Math.abs(act.da - 0.5 * AC.maxDef) < 1e-3, 'healthy channel unaffected');
});

test('slow: degraded slew reaches the target eventually, later than healthy', () => {
  const step = (faults) => {
    let act = { da: 0, de: 0, dr: 0, dt: 0 };
    act = stepActuators(act, CMDS, DT, faults);
    return act.da;
  };
  const healthy = step({});
  const slow = step({ da: { type: 'slow', factor: 8 } });
  assert.ok(slow > 0 && slow < healthy / 4, `slow first-step ${slow} vs healthy ${healthy}`);
});

test('faults thread through stepAircraft', () => {
  let s = initialState();
  const jammedAt = s.act.de;
  for (let i = 0; i < 60; i++) {
    s = stepAircraft(s, { ...CMDS, elevator: 0.5 }, DT, [0, 0, 0], { de: { type: 'jam' } });
  }
  assert.equal(s.act.de, jammedAt);
});

test('closed loop: aileron jam near trim is survivable (no crash, speed held)', () => {
  let v = createVehicle({ boot: 'air', sensorSeed: 17, windSeed: 18 });
  v = vehicleCommand(v, { type: 'mode', custom: MODES.GUIDED });
  for (let i = 0; i < 10 * 60; i++) v = vehicleStep(v, DT);
  v = vehicleServoFault(v, 'da', 'jam');
  let minAlt = Infinity;
  for (let i = 0; i < 60 * 60; i++) {
    v = vehicleStep(v, DT);
    minAlt = Math.min(minAlt, v.state.pos[1]);
  }
  assert.ok(minAlt > 60, `altitude sagged to ${minAlt} m with a jammed aileron`);
  v = vehicleClearServoFault(v, 'da');
  for (let i = 0; i < 20 * 60; i++) v = vehicleStep(v, DT);
  assert.ok(Math.abs(v.state.pos[1] - 120) < 30, `recovers control after clear (alt ${v.state.pos[1]})`);
});

test('telemetry: servo faults surface on the STATUSTEXT fault channel', async () => {
  const { vehicleTelemetry } = await import('../src/vehicle.js');
  let v = createVehicle({ boot: 'air' });
  v = vehicleStep(v, DT);
  v = vehicleServoFault(v, 'de', 'slow');
  const t = vehicleTelemetry(v);
  assert.equal(t.faults.servo_de, 'slow');
});
