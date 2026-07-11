// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Nav estimator: a gated position filter over the (faultable) GPS + baro with
// simple FDE — innovations outside the gate are REJECTED, the estimate coasts on
// its velocity, and the reported variance grows until QGC's EKF indicator flips.
// Pure and PRNG-free: same readings in, same estimate out.

export const R_GPS = 4; // m² measurement floor (≈ SNS_GPS_SGM 1.5–2 m)
export const R_BARO = 1; // m²
export const Q_H = 5; // m²/s horizontal process noise while coasting
export const Q_V = 2; // m²/s vertical
export const GATE_SIGMA = 5; // reject innovations beyond 5σ
export const FRESH_S = 1.5; // "absolute position" flag drops after this long unaided

// EKF_STATUS_FLAGS bits QGC reads.
export const EKF = {
  ATTITUDE: 1, VELOCITY_HORIZ: 2, VELOCITY_VERT: 4, POS_HORIZ_REL: 8,
  POS_HORIZ_ABS: 16, POS_VERT_ABS: 32, PRED_POS_HORIZ_REL: 256, PRED_POS_HORIZ_ABS: 512,
};

export function createEstimator(state) {
  return {
    pos: [...state.pos], vel: [...state.vel],
    varH: R_GPS, varV: R_BARO,
    lastGps: null, gpsAge: 0, baroAge: 0,
  };
}

export function stepEstimator(est, readings, dt) {
  // Predict: coast on the filtered velocity; uncertainty grows.
  const pos = [est.pos[0] + est.vel[0] * dt, est.pos[1] + est.vel[1] * dt, est.pos[2] + est.vel[2] * dt];
  const vel = [...est.vel];
  let varH = est.varH + Q_H * dt;
  let varV = est.varV + Q_V * dt;
  let { lastGps } = est;
  let gpsAge = est.gpsAge + dt;
  let baroAge = est.baroAge + dt;

  const gps = readings?.gps; // [x, z, relAlt] | null
  if (gps) {
    const ix = gps[0] - pos[0];
    const iz = gps[1] - pos[2];
    const gate = GATE_SIGMA * GATE_SIGMA * (varH + R_GPS);
    if (ix * ix + iz * iz <= gate) { // accepted: fuse + derive velocity
      const k = varH / (varH + R_GPS);
      pos[0] += k * ix;
      pos[2] += k * iz;
      if (lastGps) {
        vel[0] = 0.9 * vel[0] + 0.1 * ((gps[0] - lastGps[0]) / dt);
        vel[2] = 0.9 * vel[2] + 0.1 * ((gps[1] - lastGps[1]) / dt);
      }
      lastGps = gps;
      varH = Math.max(R_GPS, varH * (1 - k));
      gpsAge = 0;
    } // rejected (FDE): coast — varH keeps growing, age keeps counting
  }

  const baro = readings?.baro; // [relAlt] | null
  if (baro) {
    const iy = baro[0] - pos[1];
    if (iy * iy <= GATE_SIGMA * GATE_SIGMA * (varV + R_BARO)) {
      const k = varV / (varV + R_BARO);
      pos[1] += k * iy;
      vel[1] = 0.85 * vel[1] + 0.15 * (k * iy) / dt;
      varV = Math.max(R_BARO, varV * (1 - k));
      baroAge = 0;
    }
  }

  return { pos, vel, varH, varV, lastGps, gpsAge, baroAge };
}

// EKF_STATUS_REPORT fields. QGC colors variances: <0.5 good, <1 warn, ≥1 bad.
export function ekfReport(est, readings) {
  let flags = EKF.ATTITUDE | EKF.VELOCITY_HORIZ | EKF.VELOCITY_VERT | EKF.POS_HORIZ_REL |
    EKF.PRED_POS_HORIZ_REL | EKF.PRED_POS_HORIZ_ABS;
  if (est.gpsAge < FRESH_S) flags |= EKF.POS_HORIZ_ABS;
  if (est.baroAge < FRESH_S) flags |= EKF.POS_VERT_ABS;
  return {
    velocity_variance: Math.min(est.varH / 100, 10),
    pos_horiz_variance: Math.min(est.varH / 50, 10),
    pos_vert_variance: Math.min(est.varV / 25, 10),
    compass_variance: readings?.mag ? 0.05 : 1.5,
    terrain_alt_variance: 0,
    flags,
  };
}
