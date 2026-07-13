// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// The whole vehicle as one value: airframe state + arm/mode + params + sensors +
// estimators + wind + battery. ONE implementation shared by the browser sim
// (src/main.js), the node tests, and the HILS scenario runner — the sim loop and
// the bench can never drift apart. Pure: step/command return a fresh vehicle.

import { stepAircraft, groundState, initialState } from './physics.js';
import { MODES, apStep, manualControls } from './autopilot.js';
import { toLocalTargets } from './missions.js';
import { geodeticToLocal, telemetryFrom, eulerFromQuat, headingDeg } from './telemetry.js';
import { defaultParams, clampParam } from './params.js';
import { createSensors, stepSensors, injectFault, clearFault } from './sensors.js';
import {
  createEstimator, stepEstimator, createAttEstimator, stepAttEstimator, ekfReport,
} from './estimator.js';
import { createBattery, stepBattery, batteryOutputs } from './battery.js';
import { createWind, stepWind } from './wind.js';

const NEUTRAL_STICK = { pitch: 0, roll: 0, yaw: 0, throttle: 0 };

export function createVehicle({ boot = 'ground', sensorSeed = 1, windSeed = 2, params = {} } = {}) {
  const state = boot === 'air' ? initialState() : groundState();
  return {
    state,
    armed: boot === 'air', // ground boot is cold + disarmed, like a real vehicle
    simTime: 0,
    ap: {
      mode: MODES.MANUAL, landing: false, targetAlt: Math.max(40, state.pos[1]),
      targetHeading: 0, guided: null, mission: null,
    },
    params: { ...defaultParams(), ...params },
    sensors: createSensors(sensorSeed),
    readings: null,
    lastGps: null,
    est: createEstimator(state),
    att: createAttEstimator(state),
    battery: createBattery(),
    wind: createWind(windSeed),
    windWorld: [0, 0, 0],
    lastControls: { aileron: 0, elevator: 0, rudder: 0, throttle: 0 },
    lastReached: -1,
    servoFaults: {}, // channel (da/de/dr/dt) → {type: jam|floating|slow, factor?}
  };
}

// Mode changes capture "here" from the ESTIMATED state (nothing in the control
// path reads truth); the uploaded mission survives mode changes.
function setMode(v, m) {
  const e = eulerFromQuat(v.att.quat);
  return {
    ...v,
    ap: {
      mode: m, landing: false,
      targetAlt: Math.max(40, v.est.pos[1]),
      targetHeading: headingDeg(e.yaw),
      guided: null,
      mission: v.ap.mission,
    },
  };
}

// The single command path — GCS (SSE), keyboard, tests, and scenarios all land here.
export function vehicleCommand(v, cmd) {
  switch (cmd.type) {
    case 'arm': return { ...v, armed: cmd.value === 1 };
    case 'mode': return setMode(v, cmd.custom >>> 0);
    case 'takeoff': {
      const next = setMode({ ...v, armed: true }, MODES.TAKEOFF);
      return { ...next, ap: { ...next.ap, targetAlt: cmd.alt } };
    }
    case 'land': return { ...v, ap: { ...v.ap, landing: true } };
    case 'rtl': return setMode(v, MODES.RTL);
    case 'mission':
      return {
        ...v,
        ap: { ...v.ap, mission: { targets: toLocalTargets(cmd.items), idx: 0 } },
        lastReached: -1,
      };
    case 'goto': {
      const [x, , z] = geodeticToLocal({ lat: cmd.lat, lon: cmd.lon });
      const next = setMode(v, MODES.GUIDED);
      const alt = Math.max(40, cmd.alt > 1 ? cmd.alt : v.est.pos[1]);
      return { ...next, ap: { ...next.ap, guided: { x, z, alt } } };
    }
    case 'param': {
      const val = clampParam(cmd.id, cmd.value);
      if (val === null) return v;
      return { ...v, params: { ...v.params, [cmd.id]: val } };
    }
    default:
      return v;
  }
}

export function vehicleFault(v, sensor, type, opts) {
  return { ...v, sensors: injectFault(v.sensors, sensor, type, opts) };
}
export function vehicleClearFault(v, sensor) {
  return { ...v, sensors: clearFault(v.sensors, sensor) };
}

export function vehicleServoFault(v, channel, type, opts = {}) {
  if (!['da', 'de', 'dr', 'dt'].includes(channel)) return v;
  if (!['jam', 'floating', 'slow'].includes(type)) return v;
  return { ...v, servoFaults: { ...v.servoFaults, [channel]: { type, ...opts } } };
}
export function vehicleClearServoFault(v, channel) {
  const servoFaults = { ...v.servoFaults };
  delete servoFaults[channel];
  return { ...v, servoFaults };
}

// One fixed step. stick drives MANUAL mode only (null = hands off).
export function vehicleStep(v, dt, stick = null) {
  // Control path sees ONLY estimated state (+ the WoW discrete).
  const rateEst = v.readings?.gyro
    ? v.readings.gyro.map((x, i) => x - v.att.bias[i])
    : v.readings ? [0, 0, 0] : v.state.omega;
  const nav = {
    ...v.state, pos: v.est.pos, vel: v.est.vel, quat: v.att.quat, omega: rateEst,
    wow: v.state.pos[1] <= 0.5,
  };

  let { ap, armed, lastReached } = v;
  let controls;
  if (ap.mode === MODES.MANUAL && !ap.landing) {
    controls = manualControls(nav, stick ?? NEUTRAL_STICK);
  } else {
    const r = apStep(nav, ap, v.params, v.readings?.pitot?.[0] ?? null);
    ap = r.ap;
    controls = r.controls;
    if (r.disarm) armed = false;
    if (r.reached.length) lastReached = r.reached[r.reached.length - 1];
  }
  if (!armed) controls = { ...controls, throttle: 0 }; // DISARM cuts the engine

  const w = stepWind(v.wind, v.state, v.params, dt);
  const state = stepAircraft(v.state, controls, dt, w.windWorld, v.servoFaults);
  const sw = stepSensors(v.sensors, state, v.params, w.windWorld);
  return {
    ...v,
    state, armed, ap, lastReached,
    lastControls: controls,
    wind: w.wind, windWorld: w.windWorld,
    sensors: sw.sensors, readings: sw.readings,
    lastGps: sw.readings.gps ?? v.lastGps,
    att: stepAttEstimator(v.att, sw.readings, dt),
    est: stepEstimator(v.est, sw.readings, dt),
    battery: stepBattery(v.battery, state.act.dt, dt),
    simTime: v.simTime + dt,
  };
}

// Bridge-ready telemetry snapshot for the current vehicle.
export function vehicleTelemetry(v) {
  return telemetryFrom(v.state, v.state.act.dt, v.simTime, {
    armed: v.armed, customMode: v.ap.mode,
    missionSeq: v.ap.mission ? Math.min(v.ap.mission.idx, v.ap.mission.targets.length - 1) : -1,
    missionReached: v.lastReached,
    gps: v.lastGps, gpsDropout: v.readings ? !v.readings.gps : false,
    baroAlt: v.readings?.baro?.[0], health: v.readings?.health,
    // Servo faults ride the same edge-driven STATUSTEXT channel as sensor faults.
    faults: {
      ...v.readings?.faults,
      ...Object.fromEntries(Object.entries(v.servoFaults).map(([ch, f]) => [`servo_${ch}`, f.type])),
    },
    est: v.est, ekf: ekfReport(v.est, v.readings),
    va: v.readings?.pitot?.[0] ?? undefined,
    attQuat: v.att.quat,
    omega: v.readings?.gyro ? v.readings.gyro.map((x, i) => x - v.att.bias[i]) : v.state.omega,
    ...batteryOutputs(v.battery, v.state.act.dt),
  });
}
