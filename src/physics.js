// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Pure functions only — no THREE, no side effects, node-testable. Quats are [x, y, z, w].

export const G = 9.81; // m/s²
export const MASS = 1000; // kg — small GA-class aircraft
export const MAX_THRUST = 6000; // N — full-throttle level top speed ≈ √(6000/1.9) ≈ 56 m/s
export const LIFT_CL0 = 5.0; // N/(m/s)² at zero AoA
export const LIFT_CLA = 25.0; // N/(m/s)² per rad of AoA — trim AoA ≈ 2.6° at 40 m/s
export const ALPHA_MAX = 0.35; // rad — soft stall: lift stops growing past ~20° AoA
export const DRAG_COEF = 1.9; // N/(m/s)² of total speed
export const LIFT_CAP = 1.8 * MASS * G; // N — saturate lift so dives can't produce silly g
export const MAX_RATE = { pitch: 1.0, roll: 1.8, yaw: 0.5 }; // rad/s at full stick
export const RATE_TAU = 0.25; // s — first-order lag, stick → body rate

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

// Rotate world-frame vector v by quaternion q (body → world when q is the attitude).
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

// Stick → body rates with first-order lag. In this frame (nose −Z, top +Y):
// pitch-up is +X rotation, yaw-right is −Y rotation, roll-right is −Z rotation.
export function stepRates(omega, controls, dt) {
  const target = [
    controls.pitch * MAX_RATE.pitch,
    -controls.yaw * MAX_RATE.yaw,
    -controls.roll * MAX_RATE.roll,
  ];
  const k = Math.min(dt / RATE_TAU, 1);
  return [
    omega[0] + (target[0] - omega[0]) * k,
    omega[1] + (target[1] - omega[1]) * k,
    omega[2] + (target[2] - omega[2]) * k,
  ];
}

// Angle of attack (rad): + when the nose points above the velocity vector.
export function angleOfAttack(quat, vel) {
  const vb = quatRotate(quatConjugate(quat), vel); // velocity in body frame
  const vFwd = -vb[2]; // along the nose
  if (vFwd < 1) return 0; // too slow for aero angles to mean anything
  return Math.atan2(-vb[1], vFwd);
}

// World-frame net force: thrust along the nose, AoA-dependent lift along body-up
// (capped), quadratic drag opposing velocity, gravity.
export function aeroForces(quat, vel, throttle) {
  const fwd = quatRotate(quat, [0, 0, -1]);
  const up = quatRotate(quat, [0, 1, 0]);
  const [vx, vy, vz] = vel;
  const speed = Math.hypot(vx, vy, vz);
  const vFwd = Math.max(0, vx * fwd[0] + vy * fwd[1] + vz * fwd[2]);
  const thrust = throttle * MAX_THRUST;
  const alpha = Math.max(-ALPHA_MAX, Math.min(ALPHA_MAX, angleOfAttack(quat, vel)));
  const cl = Math.max(0, LIFT_CL0 + LIFT_CLA * alpha);
  const lift = Math.min(cl * vFwd * vFwd, LIFT_CAP);
  const d = DRAG_COEF * speed; // −d·v ⇒ |drag| = DRAG_COEF·speed²
  return [
    thrust * fwd[0] + lift * up[0] - d * vx,
    thrust * fwd[1] + lift * up[1] - d * vy - MASS * G,
    thrust * fwd[2] + lift * up[2] - d * vz,
  ];
}

// One fixed step of the 6-DOF-ready state. Pure: returns a fresh state object.
// controls: { pitch, roll, yaw ∈ [−1,1], throttle ∈ [0,1] }
export function stepAircraft(state, controls, dt) {
  const omega = stepRates(state.omega, controls, dt);
  const quat = quatIntegrate(state.quat, omega, dt);
  const f = aeroForces(quat, state.vel, controls.throttle);
  const vel = [
    state.vel[0] + (f[0] / MASS) * dt,
    state.vel[1] + (f[1] / MASS) * dt,
    state.vel[2] + (f[2] / MASS) * dt,
  ];
  const pos = [
    state.pos[0] + vel[0] * dt,
    state.pos[1] + vel[1] * dt,
    state.pos[2] + vel[2] * dt,
  ];
  if (pos[1] <= 0) {
    // Crude ground for M0: clamp to the surface, kill sink, bleed speed off.
    pos[1] = 0;
    vel[1] = Math.max(0, vel[1]);
    const fr = Math.max(0, 1 - 2 * dt);
    vel[0] *= fr;
    vel[2] *= fr;
  }
  return { pos, vel, quat, omega };
}

// Airborne cruise toward −Z ("north"), wings level.
export function initialState() {
  return {
    pos: [0, 120, 0],
    vel: [0, 0, -40],
    quat: [0, 0, 0, 1],
    omega: [0, 0, 0],
  };
}
