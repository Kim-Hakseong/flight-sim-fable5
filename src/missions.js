// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Mission sequencing — pure functions. Items arrive from the bridge as MAVLink
// mission items (lat/lon in deg, alt per frame); they are converted once to local
// scene targets, then the sequencer advances an index as waypoints are reached.

import { geodeticToLocal, HOME } from './telemetry.js';

export const DEFAULT_ACCEPT_M = 60;

// MAV_CMD we act on; anything else is skipped (sequenced through, not flown).
export const CMD = { WAYPOINT: 16, RTL: 20, LAND: 21, TAKEOFF: 22 };

// MAVLink items → local targets. frame 3/6 = alt relative to home, 0/2 = AMSL.
export function toLocalTargets(items) {
  return items.map((it) => {
    const relAlt = it.frame === 0 || it.frame === 2 ? it.alt - HOME.alt : it.alt;
    const [x, , z] = geodeticToLocal({ lat: it.lat, lon: it.lon, alt: HOME.alt });
    return {
      seq: it.seq, command: it.command,
      x, z, alt: Math.max(20, relAlt),
      accept: it.param2 > 0 ? it.param2 : DEFAULT_ACCEPT_M, // NAV_WAYPOINT.param2

    };
  });
}

export function horizontalDistance(pos, target) {
  return Math.hypot(target.x - pos[0], target.z - pos[2]);
}

// One sequencing step. mission: { targets, idx }. pos = [x, y(alt), z]. Pure —
// returns { mission, target, reached, action } where reached lists the seqs
// completed this step and action is 'takeoff' | 'fly' | 'land' | 'rtl' | 'done'.
export function missionStep(pos, mission) {
  const { targets } = mission;
  let idx = mission.idx;
  const reached = [];

  while (idx < targets.length) {
    const t = targets[idx];
    if (t.command === CMD.LAND) return { mission: { targets, idx }, target: t, reached, action: 'land' };
    if (t.command === CMD.RTL) return { mission: { targets, idx }, target: null, reached, action: 'rtl' };
    if (t.command === CMD.TAKEOFF) {
      // A takeoff item is NOT a position to fly to — its lat/lon is often 0/0
      // (a phantom point far away). It means "climb straight ahead to this
      // altitude, then continue". Reached once airborne at the target altitude.
      if (pos[1] >= t.alt - 5) { reached.push(t.seq); idx++; continue; }
      return { mission: { targets, idx }, target: t, reached, action: 'takeoff' };
    }
    if (t.command !== CMD.WAYPOINT) {
      idx++; // unsupported item: sequence through it so the mission can't stall
      continue;
    }
    if (horizontalDistance(pos, t) < t.accept) {
      reached.push(t.seq);
      idx++;
      continue;
    }
    return { mission: { targets, idx }, target: t, reached, action: 'fly' };
  }
  return { mission: { targets, idx }, target: null, reached, action: 'done' };
}
