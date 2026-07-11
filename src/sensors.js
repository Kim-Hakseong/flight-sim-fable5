// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Sensor error model + fault injection (HILS). Pure: stepSensors threads a seeded
// PRNG state through every draw — same seed, same flight, same readings, always.

import { gaussianNext } from './prng.js';
import { eulerFromQuat, headingDeg } from './telemetry.js';

export const SENSORS = ['gyro', 'accel', 'mag', 'baro', 'gps'];
// MAV_SYS_STATUS_SENSOR bits QGC colors in its health panel.
export const SENSOR_BITS = { gyro: 1, accel: 2, mag: 4, baro: 8, gps: 32 };
export const SENSORS_PRESENT = Object.values(SENSOR_BITS).reduce((a, b) => a | b, 0);
export const FAULT_TYPES = ['freeze', 'dropout', 'bias'];

export function createSensors(seed = 1) {
  return { rng: seed | 0, faults: {}, frozen: {} };
}

export function injectFault(sensors, sensor, type, opts = {}) {
  if (!SENSORS.includes(sensor) || !FAULT_TYPES.includes(type)) return sensors;
  return { ...sensors, faults: { ...sensors.faults, [sensor]: { type, ...opts } } };
}

export function clearFault(sensors, sensor) {
  const faults = { ...sensors.faults };
  delete faults[sensor];
  const frozen = { ...sensors.frozen };
  delete frozen[sensor];
  return { ...sensors, faults, frozen };
}

export function healthBits(faults) {
  let health = SENSORS_PRESENT;
  for (const s of Object.keys(faults)) health &= ~SENSOR_BITS[s];
  return health >>> 0;
}

// One sensor sweep. (sensors, state, P) → { sensors, readings }. Noise sigmas come
// from the shared param table; a faulted sensor freezes, drops out, or biases.
export function stepSensors(sensors, state, P) {
  let rng = sensors.rng;
  const draw = (sigma) => {
    const [g, next] = gaussianNext(rng);
    rng = next;
    return g * sigma;
  };
  const frozen = { ...sensors.frozen };

  // Each sensor ALWAYS draws its noise, fault or not — a fault must not shift the
  // PRNG stream of the other sensors (or determinism-with-faults gets untestable).
  const sample = (name, truth, sigma, biasDefault) => {
    const noisy = truth.map((v) => v + draw(sigma));
    const f = sensors.faults[name];
    if (!f) {
      delete frozen[name];
      return noisy;
    }
    if (f.type === 'dropout') return null;
    if (f.type === 'freeze') {
      if (!frozen[name]) frozen[name] = noisy;
      return frozen[name];
    }
    return noisy.map((v) => v + (f.bias ?? biasDefault)); // 'bias'
  };

  const e = eulerFromQuat(state.quat);
  const readings = {
    gps: sample('gps', [state.pos[0], state.pos[2], state.pos[1]], P.SNS_GPS_SGM_M, 50),
    baro: sample('baro', [state.pos[1]], P.SNS_BARO_SGM_M, 30),
    gyro: sample('gyro', state.omega, P.SNS_GYRO_SGM_R, 0.2),
    accel: sample('accel', [0, -9.81, 0], P.SNS_ACC_SGM_MS2, 2), // gravity ref (M6 refines)
    mag: sample('mag', [headingDeg(e.yaw)], 0.5, 45),
    health: healthBits(sensors.faults),
    faults: Object.fromEntries(Object.entries(sensors.faults).map(([k, v]) => [k, v.type])),
  };
  return { sensors: { ...sensors, rng, frozen }, readings };
}
