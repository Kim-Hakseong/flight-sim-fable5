// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Sim → bridge telemetry. Pure geodetic/attitude helpers up top (node-tested);
// the posting loop is plain I/O and never touches sim state or sim time.

import { quatRotate } from './physics.js';

export const HOME = { lat: 37.4449, lon: 126.4656, alt: 7 }; // Incheon-ish, open water/fields
const R_EARTH = 6378137; // m, WGS-84 semi-major

// Local scene coords → geodetic around HOME. North = −Z, east = +X, up = +Y.
export function localToGeodetic(pos, home = HOME) {
  const north = -pos[2];
  const east = pos[0];
  const latRad = (home.lat * Math.PI) / 180;
  return {
    lat: home.lat + (north / R_EARTH) * (180 / Math.PI),
    lon: home.lon + (east / (R_EARTH * Math.cos(latRad))) * (180 / Math.PI),
    alt: home.alt + pos[1],
  };
}

// Geodetic → local scene coords (inverse of localToGeodetic). Returns [x, y, z].
export function geodeticToLocal(geo, home = HOME) {
  const latRad = (home.lat * Math.PI) / 180;
  const north = ((geo.lat - home.lat) * Math.PI / 180) * R_EARTH;
  const east = ((geo.lon - home.lon) * Math.PI / 180) * R_EARTH * Math.cos(latRad);
  return [east, (geo.alt ?? home.alt) - home.alt, -north];
}

// Attitude quat → aerospace euler (rad): roll right +, pitch up +, yaw 0=north, +east.
// Derived by mapping body FRD / world NED onto our frame (N=−z, E=+x, D=−y).
export function eulerFromQuat(quat) {
  const fwd = quatRotate(quat, [0, 0, -1]);
  const up = quatRotate(quat, [0, 1, 0]);
  const right = quatRotate(quat, [1, 0, 0]);
  return {
    roll: Math.atan2(-right[1], up[1]),
    pitch: Math.asin(Math.max(-1, Math.min(1, fwd[1]))),
    yaw: Math.atan2(fwd[0], -fwd[2]),
  };
}

// Body rates [wx, wy, wz] (about +X right, +Y top, +Z tail) → FRD roll/pitch/yaw rates.
export function bodyRatesToFrd(omega) {
  return { rollspeed: -omega[2], pitchspeed: omega[0], yawspeed: -omega[1] };
}

export function headingDeg(yawRad) {
  const deg = (yawRad * 180) / Math.PI;
  return (deg + 360) % 360;
}

// Full snapshot the bridge needs, from a sim state + throttle + sim time.
export function telemetryFrom(state, throttle, simTime, vehicle = { armed: true, customMode: 0 }) {
  // GLOBAL_POSITION_INT is the FUSED estimate: fed by the nav estimator when one
  // is running, so estimator behavior (coasting, gating) is what the GCS map shows.
  const navPos = vehicle.est?.pos ?? state.pos;
  const navVel = vehicle.est?.vel ?? state.vel;
  const geo = localToGeodetic(navPos);
  const e = eulerFromQuat(state.quat);
  const rates = bodyRatesToFrd(state.omega);
  const [vx, vy, vz] = navVel;
  const speed = Math.hypot(...state.vel); // air data stays true airspeed
  return {
    timeBootMs: Math.round(simTime * 1000),
    lat: geo.lat, lon: geo.lon, alt: geo.alt, relAlt: navPos[1],
    roll: e.roll, pitch: e.pitch, yaw: e.yaw, ...rates,
    vn: -vz, ve: vx, vd: -vy,
    headingDeg: headingDeg(e.yaw),
    airspeed: speed, groundspeed: Math.hypot(vx, vz), climb: vy,
    throttlePct: Math.round(throttle * 100),
    armed: vehicle.armed, customMode: vehicle.customMode,
    missionSeq: vehicle.missionSeq ?? -1, missionReached: vehicle.missionReached ?? -1,
    ekf: vehicle.ekf ?? null,
    battMv: vehicle.battMv ?? 12600, battCa: vehicle.battCa ?? -1, battPct: vehicle.battPct ?? -1,
    ...sensedFields(geo, state, vehicle),
  };
}

// GPS_RAW_INT / VFR_HUD are fed from the SENSED values (fault-visible in the GCS);
// GLOBAL_POSITION_INT above stays the fused/true estimate. gps: [x, z, relAlt]|null.
function sensedFields(trueGeo, state, vehicle) {
  const gps = vehicle.gps; // last good reading is the caller's job to hold
  const geo = gps ? localToGeodetic([gps[0], gps[2], gps[1]]) : trueGeo;
  const dropout = vehicle.gpsDropout === true;
  return {
    gpsLat: geo.lat, gpsLon: geo.lon, gpsAlt: geo.alt,
    gpsFix: dropout ? 1 : 3, gpsSats: dropout ? 0 : 12,
    baroAlt: (vehicle.baroAlt ?? state.pos[1]) + HOME.alt,
    health: vehicle.health ?? 47, // SENSORS_PRESENT (gyro|accel|mag|baro|gps)
    faults: vehicle.faults ?? {},
  };
}

// Post telemetry to the bridge at `hz` — only if this page is actually served by
// the bridge (probed via its x-flight-bridge header), so a static-only serve
// stays console-clean. Wall clock is fine here: this is I/O, not sim time.
export async function startTelemetry(getSnapshot, { hz = 10 } = {}) {
  try {
    // './' not '/': under a sub-path host (GitHub Pages) the domain root is 404.
    const res = await fetch('./', { method: 'HEAD' });
    if (!res.headers.get('x-flight-bridge')) return false;
  } catch {
    return false;
  }
  setInterval(() => {
    fetch('/telemetry', {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify(getSnapshot()),
    }).catch(() => {}); // bridge restarting is not the sim's problem
  }, 1000 / hz);
  return true;
}
