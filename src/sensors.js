// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Sensor error model + fault injection (HILS). Pure: stepSensors threads a seeded
// PRNG state through every draw — same seed, same flight, same readings, always.

import { gaussianNext } from './prng.js';
import { eulerFromQuat, headingDeg } from './telemetry.js';
import { airData, forcesMoments, quatRotate, quatConjugate, toFRD, fromFRD, AC, G } from './physics.js';

// What an accelerometer actually measures: specific force = (F_total − gravity)/m
// in the body frame — i.e. the aero+prop reaction. Level flight reads ≈ +g up;
// a coordinated turn reads "up" along BODY-up (the classic tilt-sensing trap).
export function specificForce(state, wind = [0, 0, 0]) {
  const { F } = forcesMoments(state.quat, state.vel, state.omega, state.act, state.pos[1], wind);
  const gB = toFRD(quatRotate(quatConjugate(state.quat), [0, -AC.mass * G, 0]));
  return fromFRD([
    (F[0] - gB[0]) / AC.mass,
    (F[1] - gB[1]) / AC.mass,
    (F[2] - gB[2]) / AC.mass,
  ]);
}

export const SENSORS = ['gyro', 'accel', 'mag', 'baro', 'pitot', 'gps'];
// MAV_SYS_STATUS_SENSOR bits QGC colors in its health panel (pitot = diff pressure).
export const SENSOR_BITS = { gyro: 1, accel: 2, mag: 4, baro: 8, pitot: 16, gps: 32 };
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

// One sensor sweep. (sensors, state, P, wind) → { sensors, readings }. Noise sigmas
// come from the shared param table; a faulted sensor freezes, drops out, or biases.
export function stepSensors(sensors, state, P, wind = [0, 0, 0]) {
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
    pitot: sample('pitot', [airData(state.quat, state.vel, wind).Va], P.SNS_PIT_SGM_MS, 8),
    gyro: sample('gyro', state.omega, P.SNS_GYRO_SGM_R, 0.2),
    accel: sample('accel', specificForce(state, wind), P.SNS_ACC_SGM_MS2, 2),
    mag: sample('mag', [headingDeg(e.yaw)], 0.5, 45),
    health: healthBits(sensors.faults),
    faults: Object.fromEntries(Object.entries(sensors.faults).map(([k, v]) => [k, v.type])),
  };
  return { sensors: { ...sensors, rng, frozen }, readings };
}
