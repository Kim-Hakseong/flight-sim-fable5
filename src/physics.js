// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Pure functions only — no THREE, no side effects, node-testable. Quats are [x, y, z, w].
//
// Flight model: full rigid-body 6-DOF (forces AND moments) with a stability-
// derivative aero model in the style of Beard & McLain "Small Unmanned Aircraft"
// (Aerosonde-class UAV), control surfaces behind first-order actuators, ISA
// atmosphere, and a propeller thrust model. Internally the aero math runs in
// standard FRD axes (x nose, y right wing, z belly) and converts at the boundary.

export const G = 9.81; // m/s²
export const RHO0 = 1.225; // kg/m³ sea level

// Airframe + stability derivatives (per rad; nondimensional rates use b/2Va, c/2Va).
export const AC = {
  mass: 13.5, Jx: 0.8244, Jy: 1.135, Jz: 1.759, // kg, kg·m² (Jxz ≈ 0.12 neglected)
  S: 0.55, b: 2.9, c: 0.19, // wing area m², span m, chord m
  // longitudinal
  CL0: 0.28, CLa: 3.45, CLde: 0.36,
  CD0: 0.03, Kind: 0.0231, // induced drag: CD = CD0 + Kind·CL² (1/(π·e·AR), AR≈15.3)
  Cm0: -0.02338, Cma: -0.38, Cmq: -3.6, Cmde: -0.5,
  // lateral-directional
  CYb: -0.98, CYdr: 0.19,
  Clb: -0.12, Clp: -0.26, Clr: 0.14, Clda: 0.13, Cldr: 0.008,
  Cnb: 0.25, Cnp: 0.022, Cnr: -0.35, Cnda: -0.011, Cndr: -0.069,
  // propulsion (T = ½ρ·Sprop·Cprop·((kMotor·δt)² − Va²)); the B&M model wildly
  // overestimates static thrust (~320 N ⇒ T/W 2.4), so cap it to a sane value.
  sProp: 0.2027, cProp: 1.0, kMotor: 50, maxThrustN: 60,
  // ground handling
  muRoll: 0.03, muBrake: 0.22, // rolling resistance; auto-brake at idle throttle
  // actuators
  maxDef: 0.44, actTau: 0.05, thrTau: 0.4, // ±25°, surface lag s, throttle lag s
  alphaClamp: 0.30, // rad — lift stops growing past ~17° (crude, bounded stall)
};

// --- Quaternions --------------------------------------------------------------
export function quatMultiply(a, b) {
  const [ax, ay, az, aw] = a, [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

export function quatConjugate(q) {
  return [-q[0], -q[1], -q[2], q[3]];
}

export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function quatRotate(q, v) {
  const [qx, qy, qz, qw] = q, [vx, vy, vz] = v;
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return [
    vx + qw * tx + qy * tz - qz * ty,
    vy + qw * ty + qz * tx - qx * tz,
    vz + qw * tz + qx * ty - qy * tx,
  ];
}

// q̇ = ½ q ⊗ (ω, 0) with ω the body-frame angular velocity; renormalize to fight drift.
export function quatIntegrate(q, omega, dt) {
  const h = 0.5 * dt;
  const dq = quatMultiply(q, [omega[0] * h, omega[1] * h, omega[2] * h, 0]);
  return quatNormalize([q[0] + dq[0], q[1] + dq[1], q[2] + dq[2], q[3] + dq[3]]);
}

// --- Frame conversion: our body axes ↔ aero FRD -------------------------------
// FRD x (nose) = −Z ours, y (right) = +X ours, z (belly) = −Y ours.
// The same mapping converts angular rates: [p, q, r] = toFRD([wx, wy, wz]).
export function toFRD(v) {
  return [-v[2], v[0], -v[1]];
}
export function fromFRD(v) {
  return [v[1], -v[2], -v[0]];
}

// --- Atmosphere + air data -----------------------------------------------------
export function airDensity(altM) {
  const h = Math.max(0, Math.min(altM, 11000));
  return RHO0 * Math.pow(1 - 2.2557e-5 * h, 4.2559); // ISA troposphere
}

// Va (m/s), alpha, beta (rad) from attitude + inertial velocity − wind (world frame).
export function airData(quat, vel, wind = [0, 0, 0]) {
  const rel = [vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2]];
  const [u, v, w] = toFRD(quatRotate(quatConjugate(quat), rel));
  const Va = Math.hypot(u, v, w);
  if (Va < 1) return { Va, alpha: 0, beta: 0, u, v, w };
  return { Va, alpha: Math.atan2(w, u), beta: Math.asin(Math.max(-1, Math.min(1, v / Va))), u, v, w };
}

// --- Forces + moments (FRD body axes) -------------------------------------------
// act: { da, de, dr (rad), dt (0..1) } — actual actuator positions, not commands.
export function forcesMoments(quat, vel, omega, act, altM, wind = [0, 0, 0]) {
  const { Va, alpha, beta } = airData(quat, vel, wind);
  const rho = airDensity(altM);
  const qbar = 0.5 * rho * Va * Va;
  const [p, q, r] = toFRD(omega);
  const A = AC;
  const bV = Va > 1 ? A.b / (2 * Va) : 0;
  const cV = Va > 1 ? A.c / (2 * Va) : 0;

  const aEff = Math.max(-A.alphaClamp, Math.min(A.alphaClamp, alpha));
  const CL = A.CL0 + A.CLa * aEff + A.CLde * act.de;
  const CD = A.CD0 + A.Kind * CL * CL;
  const lift = qbar * A.S * CL;
  const drag = qbar * A.S * CD;
  const fy = qbar * A.S * (A.CYb * beta + A.CYdr * act.dr);

  const thrust = Math.min(
    A.maxThrustN,
    0.5 * airDensity(altM) * A.sProp * A.cProp * ((A.kMotor * act.dt) ** 2 - Va * Va)
  );

  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const gBody = toFRD(quatRotate(quatConjugate(quat), [0, -A.mass * G, 0]));
  const F = [
    thrust - drag * ca + lift * sa + gBody[0],
    fy + gBody[1],
    -drag * sa - lift * ca + gBody[2],
  ];
  const M = [
    qbar * A.S * A.b * (A.Clb * beta + A.Clp * bV * p + A.Clr * bV * r + A.Clda * act.da + A.Cldr * act.dr),
    qbar * A.S * A.c * (A.Cm0 + A.Cma * aEff + A.Cmq * cV * q + A.Cmde * act.de),
    qbar * A.S * A.b * (A.Cnb * beta + A.Cnp * bV * p + A.Cnr * bV * r + A.Cnda * act.da + A.Cndr * act.dr),
  ];
  return { F, M, Va, alpha, beta };
}

// First-order actuators with deflection limits. cmds: {aileron, elevator, rudder
// ∈ [−1,1] of max deflection; throttle ∈ [0,1]}.
export function stepActuators(act, cmds, dt) {
  const clamp = (x, m) => Math.max(-m, Math.min(m, x));
  const kS = Math.min(dt / AC.actTau, 1);
  const kT = Math.min(dt / AC.thrTau, 1);
  const target = {
    da: clamp((cmds.aileron ?? 0) * AC.maxDef, AC.maxDef),
    de: clamp((cmds.elevator ?? 0) * AC.maxDef, AC.maxDef),
    dr: clamp((cmds.rudder ?? 0) * AC.maxDef, AC.maxDef),
    dt: Math.max(0, Math.min(1, cmds.throttle ?? 0)),
  };
  return {
    da: act.da + (target.da - act.da) * kS,
    de: act.de + (target.de - act.de) * kS,
    dr: act.dr + (target.dr - act.dr) * kS,
    dt: act.dt + (target.dt - act.dt) * kT,
  };
}

// One fixed step of the rigid-body state. Pure: returns a fresh state object.
export function stepAircraft(state, cmds, dt, wind = [0, 0, 0]) {
  const act = stepActuators(state.act, cmds, dt);
  const { F, M } = forcesMoments(state.quat, state.vel, state.omega, act, state.pos[1], wind);

  // Translation in the world frame (F already includes gravity).
  const fWorld = quatRotate(state.quat, fromFRD(F));
  const vel = [
    state.vel[0] + (fWorld[0] / AC.mass) * dt,
    state.vel[1] + (fWorld[1] / AC.mass) * dt,
    state.vel[2] + (fWorld[2] / AC.mass) * dt,
  ];
  const pos = [
    state.pos[0] + vel[0] * dt,
    state.pos[1] + vel[1] * dt,
    state.pos[2] + vel[2] * dt,
  ];

  // Rotation: Euler equations with a diagonal inertia tensor, in FRD.
  const [p, q, r] = toFRD(state.omega);
  const pDot = (M[0] + (AC.Jy - AC.Jz) * q * r) / AC.Jx;
  const qDot = (M[1] + (AC.Jz - AC.Jx) * p * r) / AC.Jy;
  const rDot = (M[2] + (AC.Jx - AC.Jy) * p * q) / AC.Jz;
  let omega = fromFRD([p + pDot * dt, q + qDot * dt, r + rDot * dt]);
  const quat = quatIntegrate(state.quat, omega, dt);

  if (pos[1] <= 0) {
    pos[1] = 0;
    vel[1] = Math.max(0, vel[1]);
    // Rolling resistance, plus auto-brake once the throttle is at idle.
    const gs = Math.hypot(vel[0], vel[2]);
    if (gs > 0) {
      const mu = AC.muRoll + (act.dt < 0.1 ? AC.muBrake : 0);
      const dec = Math.min(gs, mu * G * dt);
      vel[0] -= (vel[0] / gs) * dec;
      vel[2] -= (vel[2] / gs) * dec;
    }
    // Gear: springs hold roll/pitch level at rest, yaw is damped by the tires —
    // but pitch stays aero-controllable so the elevator can rotate at Vr.
    const [p, q, r] = toFRD(omega);
    const right = quatRotate(quat, [1, 0, 0]);
    const nose = quatRotate(quat, [0, 0, -1]);
    const roll = Math.asin(Math.max(-1, Math.min(1, -right[1])));
    const pitch = Math.asin(Math.max(-1, Math.min(1, nose[1])));
    const pNew = p * (1 - Math.min(6 * dt, 1)) - roll * 4 * dt;
    const qNew = pitch < -0.02 ? Math.max(q, 0) : q; // nose can't dig through the ground
    omega = fromFRD([pNew, qNew, r * (1 - Math.min(1.5 * dt, 1))]);
  }
  return { pos, vel, quat, omega, act };
}

// At the runway threshold (south end, z = +350), level, pointing north, cold.
export function groundState() {
  return {
    pos: [0, 0, 350],
    vel: [0, 0, 0],
    quat: [0, 0, 0, 1],
    omega: [0, 0, 0],
    act: { da: 0, de: 0, dr: 0, dt: 0 },
  };
}

// Boot near the level-flight trim at Va ≈ 30 m/s: nose above the (horizontal)
// velocity by the trim AoA, elevator + throttle at their trim settings.
export const TRIM = { Va: 30, alpha: 0.05566, de: -0.08906, dt: 0.62747 }; // Newton-solved

export function initialState() {
  const h = TRIM.alpha / 2;
  return {
    pos: [0, 120, 0],
    vel: [0, 0, -TRIM.Va],
    quat: quatNormalize([Math.sin(h), 0, 0, Math.cos(h)]), // pitched up by trim AoA
    omega: [0, 0, 0],
    act: { da: 0, de: TRIM.de, dr: 0, dt: TRIM.dt },
  };
}
