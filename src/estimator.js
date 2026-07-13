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

// Velocity comes from LONG-baseline position differences (a 1-step delta divides
// sensor noise by dt and is garbage: ±tens of m/s at 60 Hz); the estimate coasts
// on that velocity through dropouts/rejections.
export const VEL_BASE_H_S = 2; // s between horizontal velocity refreshes
export const VEL_BASE_V_S = 1; // s, vertical
export const REJECT_RESET_S = 8; // FDE backstop: re-anchor after this long rejecting

// --- Attitude: Mahony complementary filter + gyro-bias estimation ---------------
// Gyro integration, corrected toward the accelerometer's gravity direction (only
// when ‖f‖ ≈ g — turns/dynamics are gated out) and the magnetometer's heading.
// The integral of the correction IS the gyro-bias estimate.
import { G, quatIntegrate, quatRotate, quatConjugate } from './physics.js';

// Raw specific force in a turn points along BODY-up (1.15 g at 30° bank) — trusted
// naively it drags the estimate toward "wings level" while banked, and flown
// closed-loop that feedback spirals. Fix like real FCs: subtract the centripetal
// term ω × v (pitot speed along the nose) so the residual is gravity again; a
// norm band gates whatever the compensation misses.
export const ATT = { KP: 1.0, KI: 0.05, KMAG: 0.3, ACC_BAND: 0.15 };

export function createAttEstimator(state) {
  return { quat: [...state.quat], bias: [0, 0, 0], lpErr: 0 };
}

export function stepAttEstimator(att, readings, dt) {
  const q = att.quat;
  const e = [0, 0, 0]; // body-frame correction rate (pre-gain)

  const gyroRaw = readings?.gyro;
  const acc = readings?.accel; // specific force, our body axes
  if (acc) {
    // Centripetal compensation: a_c = ω × v, with v ≈ pitot airspeed along the nose.
    let fg = acc;
    if (gyroRaw) {
      const w = gyroRaw.map((v, i) => v - att.bias[i]);
      const vb = [0, 0, -(readings?.pitot?.[0] ?? 30)];
      fg = [
        acc[0] - (w[1] * vb[2] - w[2] * vb[1]),
        acc[1] - (w[2] * vb[0] - w[0] * vb[2]),
        acc[2] - (w[0] * vb[1] - w[1] * vb[0]),
      ];
    }
    const n = Math.hypot(...fg);
    if (n > 1 && Math.abs(n - G) / G < (gyroRaw ? ATT.ACC_BAND : 0.08)) {
      const um = [fg[0] / n, fg[1] / n, fg[2] / n]; // measured body-up
      const ue = quatRotate(quatConjugate(q), [0, 1, 0]); // estimated body-up
      e[0] += um[1] * ue[2] - um[2] * ue[1];
      e[1] += um[2] * ue[0] - um[0] * ue[2];
      e[2] += um[0] * ue[1] - um[1] * ue[0];
    }
  }

  let eMagScale = 0;
  const mag = readings?.mag; // [heading deg]
  if (mag) {
    const fwd = quatRotate(q, [0, 0, -1]);
    let dpsi = (mag[0] * Math.PI) / 180 - Math.atan2(fwd[0], -fwd[2]);
    dpsi = ((dpsi + 3 * Math.PI) % (2 * Math.PI)) - Math.PI;
    eMagScale = -dpsi; // heading + is yaw-right = −Y body rate (our frame)
  }
  const upB = quatRotate(quatConjugate(q), [0, 1, 0]);

  const bias = att.bias.map((b, i) => b - ATT.KI * (e[i] + upB[i] * eMagScale) * dt);
  const gyro = readings?.gyro; // null on dropout: coast on corrections alone
  const omega = [0, 1, 2].map((i) =>
    (gyro ? gyro[i] - bias[i] : 0) + ATT.KP * e[i] + ATT.KMAG * upB[i] * eMagScale
  );
  return {
    quat: quatIntegrate(q, omega, dt),
    bias,
    lpErr: 0.99 * att.lpErr + 0.01 * Math.hypot(...e),
  };
}

export function createEstimator(state) {
  return {
    pos: [...state.pos], vel: [...state.vel],
    varH: R_GPS, varV: R_BARO,
    gpsRef: null, gpsRefAge: 0, baroRef: null, baroRefAge: 0,
    gpsAge: 0, baroAge: 0, rejectS: 0,
  };
}

export function stepEstimator(est, readings, dt) {
  // Predict: coast on the filtered velocity; uncertainty grows.
  const pos = [est.pos[0] + est.vel[0] * dt, est.pos[1] + est.vel[1] * dt, est.pos[2] + est.vel[2] * dt];
  const vel = [...est.vel];
  let varH = est.varH + Q_H * dt;
  let varV = est.varV + Q_V * dt;
  let { gpsRef, baroRef } = est;
  let gpsRefAge = est.gpsRefAge + dt;
  let baroRefAge = est.baroRefAge + dt;
  let gpsAge = est.gpsAge + dt;
  let baroAge = est.baroAge + dt;
  let rejectS = est.rejectS;

  const gps = readings?.gps; // [x, z, relAlt] | null
  if (gps) {
    const ix = gps[0] - pos[0];
    const iz = gps[1] - pos[2];
    if (ix * ix + iz * iz <= GATE_SIGMA * GATE_SIGMA * (varH + R_GPS)) { // accepted
      const k = varH / (varH + R_GPS);
      pos[0] += k * ix;
      pos[2] += k * iz;
      if (gpsRef && gpsRefAge >= VEL_BASE_H_S) {
        vel[0] = 0.7 * vel[0] + 0.3 * ((gps[0] - gpsRef[0]) / gpsRefAge);
        vel[2] = 0.7 * vel[2] + 0.3 * ((gps[1] - gpsRef[1]) / gpsRefAge);
      }
      if (!gpsRef || gpsRefAge >= VEL_BASE_H_S) {
        gpsRef = gps;
        gpsRefAge = 0;
      }
      varH = Math.max(R_GPS, varH * (1 - k));
      gpsAge = 0;
      rejectS = 0;
    } else { // rejected (FDE): coast, but never wedge — re-anchor eventually
      rejectS += dt;
      if (rejectS > REJECT_RESET_S) {
        pos[0] = gps[0];
        pos[2] = gps[1];
        varH = 4 * R_GPS;
        gpsRef = gps;
        gpsRefAge = 0;
        gpsAge = 0;
        rejectS = 0;
      }
    }
  }

  const baro = readings?.baro; // [relAlt] | null
  if (baro) {
    const iy = baro[0] - pos[1];
    if (iy * iy <= GATE_SIGMA * GATE_SIGMA * (varV + R_BARO)) {
      const k = varV / (varV + R_BARO);
      pos[1] += k * iy;
      if (baroRef !== null && baroRefAge >= VEL_BASE_V_S) {
        vel[1] = 0.7 * vel[1] + 0.3 * ((baro[0] - baroRef) / baroRefAge);
      }
      if (baroRef === null || baroRefAge >= VEL_BASE_V_S) {
        baroRef = baro[0];
        baroRefAge = 0;
      }
      varV = Math.max(R_BARO, varV * (1 - k));
      baroAge = 0;
    }
  }

  return { pos, vel, varH, varV, gpsRef, gpsRefAge, baroRef, baroRefAge, gpsAge, baroAge, rejectS };
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
