// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Guidance + control by successive loop closure onto real control surfaces:
//   heading → bank cmd → aileron (+ roll-rate damping)
//   altitude → climb cmd → pitch cmd → elevator (+ q damping)
//   airspeed → throttle · coordinated-turn rudder (+ yaw damping)
// Surface sign map (from the aero derivatives): aileron+ rolls right,
// elevator− pitches up (Cmde < 0), rudder− yaws right (Cndr < 0).

import { G, AC, TRIM, toFRD, airData } from './physics.js';
import { eulerFromQuat, headingDeg } from './telemetry.js';
import { missionStep, horizontalDistance } from './missions.js';
import { defaultParams } from './params.js';

const DP = defaultParams();

// Must exceed the achievable turn radius (V²/g·tanφ ≈ 160 m at 30 m/s, 30° bank).
export const LOITER_RADIUS_M = 250;
export const TAKEOFF_VR_MS = 20; // rotate speed ≈ 1.15·Vstall (17.3 m/s)

// ArduPlane custom_mode numbers — QGC shows these names for autopilot=3.
export const MODES = { MANUAL: 0, AUTO: 10, RTL: 11, LOITER: 12, TAKEOFF: 13, GUIDED: 15 };
export const MODE_NAMES = Object.fromEntries(Object.entries(MODES).map(([k, v]) => [v, k]));

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
const DE_TRIM = TRIM.de / AC.maxDef; // trim elevator, in normalized command units

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

// Hold a target altitude + heading (+ airspeed). throttleOverride pins the
// throttle (landing glide = 0, takeoff = 1) instead of the airspeed loop.
// `va` is the MEASURED airspeed (pitot); without it the loop falls back to
// inertial speed — i.e. it can't feel the wind, like a UAV with no airspeed sensor.
export function holdControls(state, targetAlt, targetHeading, throttleOverride = null, P = DP, va = null) {
  const e = eulerFromQuat(state.quat);
  const [p, q, r] = toFRD(state.omega);
  const Va = va ?? airData(state.quat, state.vel).Va;
  const V = Math.max(Va, 10);

  // targetHeading is a COURSE over ground: with a wind estimate on the nav state,
  // crab into the crosswind so the TRACK (not the nose) follows it.
  let hdgCmd = targetHeading;
  if (state.windEst) {
    const cRad = (targetHeading * Math.PI) / 180;
    const wCross = state.windEst.e * Math.cos(cRad) - state.windEst.n * Math.sin(cRad);
    hdgCmd -= (Math.asin(clamp(wCross / V, -0.6, 0.6)) * 180) / Math.PI;
  }
  const hdgErr = headingErrorDeg(hdgCmd, headingDeg(e.yaw));
  const bankT = clamp(hdgErr * P.AP_HDG_P, -P.AP_BANK_MAX, P.AP_BANK_MAX);
  const aileron = clamp(P.AP_ROLL_KP * (bankT - e.roll) - P.AP_ROLL_KD * p, -1, 1);
  // Coordinated turn: track the turn's kinematic yaw rate, damp the rest — plus a
  // rudder roll-assist through the dihedral (yaw→β→Clβ) path. The assist is what
  // keeps the aircraft controllable when the aileron servo is jammed/floating.
  const rCmd = (G / V) * Math.tan(e.roll);
  const rudder = clamp(-P.AP_YAW_KD * (rCmd - r) - P.AP_RUD_ROLL * (bankT - e.roll), -1, 1);

  const climbT = clamp((targetAlt - state.pos[1]) * P.AP_ALT_P, -P.AP_SINK_MAX, P.AP_CLIMB_MAX);
  // Pitch target: flight-path feedforward (γ ≈ climb/Va) + trim AoA + rate feedback.
  const thetaT = clamp(climbT / V + TRIM.alpha + 0.03 * (climbT - state.vel[1]), -0.35, 0.4);
  const elevator = clamp(DE_TRIM - P.AP_PITCH_KP * (thetaT - e.pitch) + P.AP_PITCH_KD * q, -1, 1);

  const throttle = throttleOverride ??
    clamp(TRIM.dt + P.AP_THR_KP * (P.AP_VA_TRIM - Va) + 0.05 * climbT, 0, 1);
  return { aileron, elevator, rudder, throttle };
}

// MANUAL with stability augmentation: stick → surfaces plus p/q/r damping, so a
// keyboard can fly the bare airframe. stick: {pitch, roll, yaw ∈ [−1,1], throttle}.
export function manualControls(state, stick) {
  const [p, q, r] = toFRD(state.omega);
  return {
    aileron: clamp(stick.roll * 0.8 - 0.2 * p, -1, 1),
    elevator: clamp(DE_TRIM - 0.7 * stick.pitch + 0.5 * q, -1, 1),
    rudder: clamp(-0.6 * stick.yaw + 0.8 * r, -1, 1),
    throttle: stick.throttle,
  };
}

// Ground roll: full power, rudder steers the given heading, wings held level;
// rotate (elevator up) once the pitot reaches Vr. Used by TAKEOFF mode and by
// AUTO when a mission is started from the runway (real ArduPlane behavior —
// without this, AUTO on the ground just rudder-spins toward the first waypoint).
function groundRollControls(state, targetHeading, va) {
  const e = eulerFromQuat(state.quat);
  const [p, , r] = toFRD(state.omega);
  const hdgErr = headingErrorDeg(targetHeading, headingDeg(e.yaw));
  const speed = va ?? Math.hypot(state.vel[0], state.vel[2]);
  return {
    aileron: clamp(-1.5 * e.roll - 0.2 * p, -1, 1),
    elevator: speed >= TAKEOFF_VR_MS ? clamp(DE_TRIM - 0.45, -1, 1) : DE_TRIM,
    rudder: clamp(-0.04 * hdgErr + 0.8 * r, -1, 1),
    throttle: 1,
  };
}

// One guidance step. ap: { mode, landing, targetAlt, targetHeading,
// guided: {x,z,alt}|null, mission: {targets,idx}|null }.
// Returns { controls, ap, disarm, reached } — ap may transition (pure: fresh object).
export function apStep(state, ap, P = DP, va = null) {
  let next = ap;
  let stepReached = []; // waypoint seqs completed during this step
  const out = (controls, apOut = next, disarm = false, reached = stepReached) =>
    ({ controls, ap: apOut, disarm, reached });

  if (ap.landing) {
    // Powered approach AND powered flare — cutting power early bleeds Va and the
    // sink returns (mushing). Airspeed loop stays on; idle only in the last metres.
    // Commanded sink = (altT − y)·AP_ALT_P: approach −3.5, flare −1.5, touch −0.8.
    const y = state.pos[1];
    const altT = y < 5 ? y - 3.2 : y < 15 ? y - 6 : -50;
    const controls = holdControls(state, altT, ap.targetHeading, y < 2 ? 0 : null, P, va);
    // Touchdown = weight-on-wheels (a real discrete, state.wow) when provided —
    // estimated altitude is too noisy to declare "on the ground" by itself.
    const wow = state.wow ?? state.pos[1] <= 0.5;
    const down = wow && Math.hypot(state.vel[0], state.vel[2]) < 3;
    return out(controls, next, down);
  }

  switch (ap.mode) {
    case MODES.TAKEOFF: {
      if (state.pos[1] >= ap.targetAlt - 2) {
        next = { ...ap, mode: MODES.GUIDED }; // climb-out done: hold here
        break;
      }
      if (state.wow ?? state.pos[1] <= 0.5) {
        return out(groundRollControls(state, ap.targetHeading, va), ap);
      }
      return out(holdControls(state, ap.targetAlt, ap.targetHeading, 1, P, va), ap);
    }
    case MODES.RTL: {
      const dist = Math.hypot(state.pos[0], state.pos[2]);
      // Begin the descent early enough that touchdown lands near home:
      // approach + flare ground distance ≈ 11·alt (measured, Va 30 / sink profile).
      if (dist < Math.max(LOITER_RADIUS_M, 11 * state.pos[1])) {
        next = { ...ap, landing: true, targetHeading: bearingToDeg(state.pos) };
        break;
      }
      return out(holdControls(state, Math.max(80, ap.targetAlt), bearingToDeg(state.pos), null, P, va), ap);
    }
    case MODES.AUTO: {
      if (!ap.mission) break; // no plan yet: hold
      if ((state.wow ?? state.pos[1] <= 0.5) && Math.hypot(state.vel[0], state.vel[2]) < TAKEOFF_VR_MS + 5) {
        // Mission started from the runway: ground-roll takeoff straight ahead on
        // the heading captured at mode entry, then the waypoint logic takes over.
        return out(groundRollControls(state, ap.targetHeading, va), ap);
      }
      const ms = missionStep(state.pos, ap.mission);
      next = { ...ap, mission: ms.mission };
      stepReached = ms.reached;
      if (ms.action === 'land') {
        const brg = bearingToDeg(state.pos, [ms.target.x, 0, ms.target.z]);
        next = { ...next, landing: true, targetHeading: brg };
        break;
      }
      if (ms.action === 'rtl') {
        next = { ...next, mode: MODES.RTL };
        return out(holdControls(state, Math.max(80, ap.targetAlt), bearingToDeg(state.pos), null, P, va), next);
      }
      if (ms.action === 'done') break; // mission complete: hold here
      const brg = bearingToDeg(state.pos, [ms.target.x, 0, ms.target.z]);
      return out(holdControls(state, ms.target.alt, brg, null, P, va), next);
    }
    case MODES.GUIDED: {
      if (!ap.guided) break; // no target: hold
      const t = ap.guided;
      const dist = horizontalDistance(state.pos, t);
      // Far out: fly straight at it. Near: chase a carrot on the loiter circle —
      // the radial through the aircraft, swung ~80° ahead, scaled to the radius.
      let aim = [t.x, 0, t.z];
      if (dist < 2 * LOITER_RADIUS_M) {
        const dx = (state.pos[0] - t.x) / (dist || 1);
        const dz = (state.pos[2] - t.z) / (dist || 1);
        const a = (80 * Math.PI) / 180;
        const rx = dx * Math.cos(a) - dz * Math.sin(a);
        const rz = dx * Math.sin(a) + dz * Math.cos(a);
        aim = [t.x + rx * LOITER_RADIUS_M, 0, t.z + rz * LOITER_RADIUS_M];
      }
      return out(holdControls(state, t.alt, bearingToDeg(state.pos, aim), null, P, va), ap);
    }
    default:
      break; // MANUAL(landing)/LOITER: hold captured alt + heading
  }

  const controls = holdControls(state, next.targetAlt, next.targetHeading, null, P, va);
  return out(controls);
}
