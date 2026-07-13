// Shared parameter table — the single source for both the sim (live tuning) and
// the bridge (PARAM_* protocol). All params are MAV_PARAM_TYPE_REAL32. IDs are
// MAVLink param_ids: ≤ 16 chars.

export const PARAM_TYPE_REAL32 = 9;

export const PARAM_DEFS = [
  // Autopilot gains (used by src/autopilot.js — successive loop closure)
  { id: 'AP_HDG_P', def: 0.03, min: 0.005, max: 0.2 }, // rad bank per deg heading err
  { id: 'AP_BANK_MAX', def: 0.6, min: 0.15, max: 1.0 }, // rad
  { id: 'AP_ROLL_KP', def: 1.5, min: 0.2, max: 6 }, // aileron per rad bank err
  { id: 'AP_ROLL_KD', def: 0.15, min: 0, max: 1 }, // aileron per rad/s p
  { id: 'AP_PITCH_KP', def: 3, min: 0.5, max: 10 }, // elevator per rad pitch err
  { id: 'AP_PITCH_KD', def: 0.8, min: 0, max: 3 }, // elevator per rad/s q
  { id: 'AP_YAW_KD', def: 1.0, min: 0, max: 4 }, // rudder per rad/s yaw-rate err
  { id: 'AP_RUD_ROLL', def: 0.35, min: 0, max: 2 }, // rudder roll-assist (dihedral path)
  { id: 'AP_ALT_P', def: 0.25, min: 0.05, max: 1 }, // m/s climb per m alt err
  { id: 'AP_CLIMB_MAX', def: 5, min: 1, max: 10 }, // m/s
  { id: 'AP_SINK_MAX', def: 3.5, min: 1, max: 8 }, // m/s
  { id: 'AP_VA_TRIM', def: 30, min: 22, max: 40 }, // m/s airspeed hold target
  { id: 'AP_THR_KP', def: 0.04, min: 0, max: 0.2 }, // throttle per m/s speed err
  // Environment (consumed by src/wind.js)
  { id: 'WND_N_MS', def: 0, min: -20, max: 20 }, // steady wind TO north m/s
  { id: 'WND_E_MS', def: 0, min: -20, max: 20 }, // steady wind TO east m/s
  { id: 'WND_TRB', def: 1, min: 0, max: 3 }, // Dryden intensity scale (0 = calm)
  // Sensor error sigmas (consumed by the M5 sensor model)
  { id: 'SNS_GPS_SGM_M', def: 1.5, min: 0, max: 50 }, // m, horizontal
  { id: 'SNS_BARO_SGM_M', def: 0.4, min: 0, max: 20 }, // m
  { id: 'SNS_GYRO_SGM_R', def: 0.002, min: 0, max: 0.5 }, // rad/s
  { id: 'SNS_PIT_SGM_MS', def: 0.5, min: 0, max: 10 }, // m/s, pitot airspeed
  { id: 'SNS_ACC_SGM_MS2', def: 0.05, min: 0, max: 5 }, // m/s²
];

export function defaultParams() {
  return Object.fromEntries(PARAM_DEFS.map((p) => [p.id, p.def]));
}

// Clamp a set request into the param's range; null for unknown ids.
export function clampParam(id, value) {
  const d = PARAM_DEFS.find((p) => p.id === id);
  if (!d || !Number.isFinite(value)) return null;
  return Math.min(d.max, Math.max(d.min, value));
}
