// HILS scenario runner: declarative scenario in → deterministic closed-loop run →
// pass/fail report out. Shares the exact vehicle implementation with the live sim.
//
// Scenario: { name, description, boot: 'air'|'ground', duration s, seed,
//   params: {..overrides}, events: [{ t, command|fault|clear }],
//   checks: [
//     { name, type:'band',  signal, from, to, min?, max? }  — inside band during [from,to]
//     { name, type:'final', signal, min?, max? }            — at scenario end
//     { name, type:'reach', signal, min?, max?, from?, by } — happens at some t in [from, by]
//   ] }

import {
  createVehicle, vehicleStep, vehicleCommand, vehicleFault, vehicleClearFault,
} from './vehicle.js';
import { eulerFromQuat, headingDeg, HOME } from './telemetry.js';
import { airData } from './physics.js';

const DT = 1 / 60;

function sample(v) {
  const e = eulerFromQuat(v.state.quat);
  const qd = Math.abs(v.att.quat.reduce((s, x, i) => s + x * v.state.quat[i], 0));
  return {
    t: v.simTime,
    alt: v.state.pos[1],
    va: airData(v.state.quat, v.state.vel, v.windWorld).Va,
    gs: Math.hypot(v.state.vel[0], v.state.vel[2]),
    hdg: headingDeg(e.yaw),
    distHome: Math.hypot(v.state.pos[0], v.state.pos[2]),
    estErrH: Math.hypot(v.est.pos[0] - v.state.pos[0], v.est.pos[2] - v.state.pos[2]),
    attErrDeg: 2 * Math.acos(Math.min(1, qd)) * (180 / Math.PI),
    mode: v.ap.mode,
    armed: v.armed ? 1 : 0,
    reached: v.lastReached,
  };
}

function evalCheck(c, samples, finalS) {
  const inWin = c.type === 'band'
    ? samples.filter((s) => s.t >= (c.from ?? 0) && s.t <= (c.to ?? Infinity))
    : c.type === 'reach' ? samples.filter((s) => s.t >= (c.from ?? 0) && s.t <= c.by) : [finalS];
  const ok = (s) =>
    (c.min === undefined || s[c.signal] >= c.min) &&
    (c.max === undefined || s[c.signal] <= c.max);
  const pass = c.type === 'reach' ? inWin.some(ok) : inWin.length > 0 && inWin.every(ok);
  let worst = null;
  if (inWin.length) {
    const vals = inWin.map((s) => s[c.signal]);
    worst = c.min !== undefined && c.max === undefined ? Math.min(...vals)
      : c.max !== undefined && c.min === undefined ? Math.max(...vals)
        : [Math.min(...vals), Math.max(...vals)];
  }
  return { name: c.name, pass, signal: c.signal, worst };
}

export function runScenario(sc) {
  let v = createVehicle({
    boot: sc.boot ?? 'air',
    sensorSeed: sc.seed ?? 1,
    windSeed: (sc.seed ?? 1) + 1,
    params: sc.params ?? {},
  });
  const events = [...(sc.events ?? [])].sort((a, b) => a.t - b.t);
  let ei = 0;
  const samples = [];
  const steps = Math.round((sc.duration ?? 60) / DT);
  for (let i = 0; i < steps; i++) {
    while (ei < events.length && events[ei].t <= i * DT) {
      const ev = events[ei++];
      if (ev.command) v = vehicleCommand(v, ev.command);
      if (ev.fault) v = vehicleFault(v, ev.fault.sensor, ev.fault.type, ev.fault);
      if (ev.clear) v = vehicleClearFault(v, ev.clear);
    }
    v = vehicleStep(v, DT);
    if (i % 6 === 5) samples.push(sample(v)); // 10 Hz
  }
  const finalS = sample(v);
  const results = (sc.checks ?? []).map((c) => evalCheck(c, samples, finalS));
  return { name: sc.name, pass: results.every((r) => r.pass), results, final: finalS };
}

// --- Built-in bench library ------------------------------------------------------
const GUIDED_HOLD = { t: 0.5, command: { type: 'mode', custom: 15 } };

export const SCENARIOS = [
  {
    name: 'gps-dropout-recovery',
    description: '12 s GPS outage in a GUIDED hold: coast, keep flying, re-converge.',
    boot: 'air', duration: 60, seed: 21,
    events: [
      GUIDED_HOLD,
      { t: 15, fault: { sensor: 'gps', type: 'dropout' } },
      { t: 27, clear: 'gps' },
    ],
    checks: [
      { name: 'altitude held throughout', type: 'band', signal: 'alt', from: 5, to: 60, min: 95, max: 150 },
      { name: 'nav coast error bounded', type: 'band', signal: 'estErrH', from: 15, to: 27, max: 80 },
      { name: 'nav re-converges', type: 'final', signal: 'estErrH', max: 10 },
    ],
  },
  {
    name: 'gyro-bias-absorption',
    description: 'A 0.05 rad/s gyro bias appears mid-flight; the Mahony integral eats it.',
    boot: 'air', duration: 90, seed: 5,
    events: [
      GUIDED_HOLD,
      { t: 10, fault: { sensor: 'gyro', type: 'bias', bias: 0.05 } },
    ],
    checks: [
      { name: 'no crash', type: 'band', signal: 'alt', from: 5, to: 90, min: 60 },
      { name: 'attitude recovers', type: 'final', signal: 'attErrDeg', max: 4 },
    ],
  },
  {
    name: 'mag-fault-flyable',
    description: '30° compass bias: heading goes wrong but the vehicle stays healthy.',
    boot: 'air', duration: 60, seed: 7,
    events: [
      GUIDED_HOLD,
      { t: 10, fault: { sensor: 'mag', type: 'bias', bias: 30 } },
    ],
    checks: [
      { name: 'altitude held', type: 'band', signal: 'alt', from: 5, to: 60, min: 95, max: 150 },
      { name: 'airspeed held', type: 'band', signal: 'va', from: 5, to: 60, min: 24, max: 36 },
    ],
  },
  {
    name: 'heavy-turbulence-goto',
    description: 'WND_TRB 2.5 + 8 m/s crosswind: fly a 1.5 km go-to and hold the orbit.',
    boot: 'air', duration: 120, seed: 11,
    params: { WND_TRB: 2.5, WND_E_MS: 8 },
    events: [
      { t: 0.5, command: { type: 'goto', lat: HOME.lat + 0.0135, lon: HOME.lon, alt: 130 } },
    ],
    checks: [
      { name: 'altitude band', type: 'band', signal: 'alt', from: 10, to: 120, min: 90, max: 170 },
      { name: 'reaches the point', type: 'reach', signal: 'distHome', max: 2200, by: 120 },
      { name: 'orbit is bounded', type: 'band', signal: 'distHome', from: 80, to: 120, min: 900, max: 2100 },
    ],
  },
  {
    name: 'full-sortie',
    description: 'Cold runway boot: takeoff, 2-waypoint mission, RTL, land, auto-disarm.',
    boot: 'ground', duration: 360, seed: 21,
    events: [
      { t: 1, command: { type: 'takeoff', alt: 60 } },
      {
        t: 2,
        command: {
          type: 'mission',
          items: [
            { seq: 0, command: 16, frame: 3, lat: HOME.lat + 0.011, lon: HOME.lon, alt: 120, param1: 0, param2: 0 },
            { seq: 1, command: 16, frame: 3, lat: HOME.lat + 0.011, lon: HOME.lon + 0.012, alt: 120, param1: 0, param2: 0 },
          ],
        },
      },
      { t: 40, command: { type: 'mode', custom: 10 } }, // AUTO once airborne
      { t: 170, command: { type: 'rtl' } },
    ],
    checks: [
      { name: 'lifts off', type: 'reach', signal: 'alt', min: 30, by: 40 },
      { name: 'waypoints reached', type: 'reach', signal: 'reached', min: 1, by: 170 },
      { name: 'lands', type: 'reach', signal: 'alt', max: 0.1, from: 180, by: 360 },
      { name: 'auto-disarms', type: 'final', signal: 'armed', max: 0 },
      { name: 'stops near home', type: 'final', signal: 'distHome', max: 800 },
    ],
  },
];

export function runAll() {
  return SCENARIOS.map(runScenario);
}
