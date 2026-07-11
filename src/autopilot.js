// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Guidance: pure functions from (state, ap) → stick controls. The physics has no
// weathervane moment yet, so turns are flown coordinated: bank tilts the lift and
// a computed yaw rate (g·tanφ/V) keeps the nose on the velocity vector.

import { G, MAX_RATE } from './physics.js';
import { eulerFromQuat, headingDeg } from './telemetry.js';

// ArduPlane custom_mode numbers — QGC shows these names for autopilot=3.
export const MODES = { MANUAL: 0, AUTO: 10, RTL: 11, LOITER: 12, TAKEOFF: 13, GUIDED: 15 };
export const MODE_NAMES = Object.fromEntries(Object.entries(MODES).map(([k, v]) => [v, k]));

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

// Shortest signed heading error in degrees [−180, 180).
export function headingErrorDeg(target, current) {
  return ((target - current + 540) % 360) - 180;
}

// Bearing (deg, 0=north) from the aircraft position to a local x/z point.
export function bearingToDeg(pos, to = [0, 0, 0]) {
  const east = to[0] - pos[0];
  const north = -(to[2] - pos[2]);
  return headingDeg(Math.atan2(east, north));
}

// Hold a target altitude + heading: cascade of P loops onto the rate-command stick.
export function holdControls(state, targetAlt, targetHeading, throttleBase = 0.7) {
  const e = eulerFromQuat(state.quat);
  const speed = Math.max(10, Math.hypot(...state.vel));

  const hdgErr = headingErrorDeg(targetHeading, headingDeg(e.yaw));
  const bankTarget = clamp(hdgErr * 0.03, -0.6, 0.6); // rad; full bank past ~20° error
  const roll = clamp((bankTarget - e.roll) * 2.5, -1, 1);
  // Coordinated turn: nose follows the velocity vector at ω = g·tanφ / V.
  const yaw = clamp((G * Math.tan(e.roll)) / speed / MAX_RATE.yaw, -1, 1);

  const climbTarget = clamp((targetAlt - state.pos[1]) * 0.25, -4, 6);
  const pitchTarget = clamp((climbTarget - state.vel[1]) * 0.05, -0.3, 0.35);
  const pitch = clamp((pitchTarget - e.pitch) * 3, -1, 1);

  const throttle = clamp(throttleBase + climbTarget * 0.04, 0.15, 1);
  return { pitch, roll, yaw, throttle };
}

// One guidance step. ap: { mode, landing, targetAlt, targetHeading }.
// Returns { controls, ap, disarm } — ap may transition (pure: fresh object).
export function apStep(state, ap) {
  let next = ap;

  if (ap.landing) {
    const c = holdControls(state, -50, ap.targetHeading, 0); // drive alt down
    const flare = state.pos[1] < 15 ? 0.6 : 1; // shallow the sink near the ground
    const controls = { ...c, throttle: 0, pitch: c.pitch * flare };
    const down = state.pos[1] <= 0.5 && Math.hypot(state.vel[0], state.vel[2]) < 3;
    return { controls, ap: next, disarm: down };
  }

  switch (ap.mode) {
    case MODES.TAKEOFF: {
      if (state.pos[1] >= ap.targetAlt - 2) {
        next = { ...ap, mode: MODES.GUIDED }; // climb-out done: hold here
        break;
      }
      const c = holdControls(state, ap.targetAlt, ap.targetHeading, 1);
      return { controls: { ...c, throttle: 1 }, ap, disarm: false };
    }
    case MODES.RTL: {
      const dist = Math.hypot(state.pos[0], state.pos[2]);
      if (dist < 150) {
        next = { ...ap, landing: true };
        break;
      }
      const controls = holdControls(state, Math.max(80, ap.targetAlt), bearingToDeg(state.pos));
      return { controls, ap, disarm: false };
    }
    default:
      break; // GUIDED / AUTO / LOITER: hold captured alt + heading (targets land in M3)
  }

  const controls = holdControls(state, next.targetAlt, next.targetHeading);
  return { controls, ap: next, disarm: false };
}
