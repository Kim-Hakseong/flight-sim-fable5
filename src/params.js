// Shared parameter table — the single source for both the sim (live tuning) and
// the bridge (PARAM_* protocol). All params are MAV_PARAM_TYPE_REAL32. IDs are
// MAVLink param_ids: ≤ 16 chars.

export const PARAM_TYPE_REAL32 = 9;

export const PARAM_DEFS = [
  // Autopilot gains (used by src/autopilot.js)
  { id: 'AP_HDG_P', def: 0.03, min: 0.005, max: 0.2 }, // rad bank per deg heading err
  { id: 'AP_BANK_MAX', def: 0.6, min: 0.15, max: 1.0 }, // rad
  { id: 'AP_ROLL_P', def: 2.5, min: 0.5, max: 8 }, // stick per rad roll err
  { id: 'AP_ALT_P', def: 0.25, min: 0.05, max: 1 }, // m/s climb per m alt err
  { id: 'AP_CLIMB_MAX', def: 6, min: 1, max: 12 }, // m/s
  { id: 'AP_SINK_MAX', def: 4, min: 1, max: 10 }, // m/s
  { id: 'AP_PITCH_P', def: 3, min: 0.5, max: 8 }, // stick per rad pitch err
  { id: 'AP_THR_CRUISE', def: 0.7, min: 0.2, max: 1 },
  // Sensor error sigmas (consumed by the M5 sensor model)
  { id: 'SNS_GPS_SGM_M', def: 1.5, min: 0, max: 50 }, // m, horizontal
  { id: 'SNS_BARO_SGM_M', def: 0.4, min: 0, max: 20 }, // m
  { id: 'SNS_GYRO_SGM_R', def: 0.002, min: 0, max: 0.5 }, // rad/s
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
