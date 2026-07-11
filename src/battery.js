// Deterministic battery: drain integrates on sim dt only (never wall clock).
// Current follows throttle², voltage tracks state-of-charge with a load sag.

export const BATT_CAPACITY_AH = 5;
export const BATT_V_FULL = 12.6;
export const BATT_V_EMPTY = 10.5;
export const BATT_I_BASE_A = 0.5; // avionics floor
export const BATT_I_MAX_A = 20; // extra at full throttle (≈ 29 min at cruise 0.7)

export function createBattery() {
  return { soc: 1 };
}

export function batteryCurrentA(throttle) {
  return BATT_I_BASE_A + BATT_I_MAX_A * throttle * throttle;
}

export function stepBattery(batt, throttle, dt) {
  const drawnAh = (batteryCurrentA(throttle) * dt) / 3600;
  return { soc: Math.max(0, batt.soc - drawnAh / BATT_CAPACITY_AH) };
}

// Wire-ready values for SYS_STATUS (mV, cA=10 mA units, %).
export function batteryOutputs(batt, throttle) {
  const amps = batteryCurrentA(throttle);
  const sag = 0.35 * (amps / (BATT_I_BASE_A + BATT_I_MAX_A));
  const volts = BATT_V_EMPTY + (BATT_V_FULL - BATT_V_EMPTY) * batt.soc - sag;
  return {
    battMv: Math.round(volts * 1000),
    battCa: Math.round(amps * 100),
    battPct: Math.round(batt.soc * 100),
  };
}
