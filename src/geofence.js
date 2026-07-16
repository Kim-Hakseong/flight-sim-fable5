// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Geofence — pure geometry. Fence items arrive from the bridge as MAVLink fence
// items (MAV_CMD 5000-5004, lat/lon in deg); convert once to local scene polygons
// and circles, then test the vehicle position each step for a breach. No side
// effects — buildFence/fenceBreach are pure and unit-tested.

import { geodeticToLocal, HOME } from './telemetry.js';

// MAV_CMD fence commands (a fence "mission" is a list of these).
export const FENCE_CMD = {
  RETURN_POINT: 5000,
  POLY_INCLUSION: 5001, // stay INSIDE this polygon (param1 = vertex count)
  POLY_EXCLUSION: 5002, // stay OUTSIDE this polygon
  CIRCLE_INCLUSION: 5003, // stay INSIDE this circle (param1 = radius m)
  CIRCLE_EXCLUSION: 5004, // stay OUTSIDE this circle
};

// Ray-casting point-in-polygon on the local x/z plane.
export function pointInPolygon(px, pz, verts) {
  let inside = false;
  for (let i = 0, j = verts.length - 1; i < verts.length; j = i++) {
    const [xi, zi] = verts[i];
    const [xj, zj] = verts[j];
    if ((zi > pz) !== (zj > pz) && px < ((xj - xi) * (pz - zi)) / (zj - zi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

// Fence items (lat/lon deg, command, param1) → local geometry. Polygon vertices
// are consecutive items sharing a command; param1 is that polygon's vertex count,
// so a run of `param1` vertices closes one polygon and the next starts a new one.
export function buildFence(items = []) {
  const polygons = []; // { inclusion, verts: [[x,z],…] }
  const circles = []; // { inclusion, x, z, r }
  let ret = null; // return point { x, z }
  let poly = null; // current polygon accumulator
  for (const it of items) {
    const [x, , z] = geodeticToLocal({ lat: it.lat, lon: it.lon, alt: HOME.alt });
    const cmd = it.command;
    if (cmd === FENCE_CMD.CIRCLE_INCLUSION || cmd === FENCE_CMD.CIRCLE_EXCLUSION) {
      circles.push({ inclusion: cmd === FENCE_CMD.CIRCLE_INCLUSION, x, z, r: Math.max(1, it.param1 || 0) });
      poly = null;
    } else if (cmd === FENCE_CMD.RETURN_POINT) {
      ret = { x, z };
      poly = null;
    } else if (cmd === FENCE_CMD.POLY_INCLUSION || cmd === FENCE_CMD.POLY_EXCLUSION) {
      const inclusion = cmd === FENCE_CMD.POLY_INCLUSION;
      const need = Math.max(3, Math.round(it.param1) || 3);
      if (!poly || poly.inclusion !== inclusion || poly.verts.length >= poly.need) {
        poly = { inclusion, need, verts: [] };
        polygons.push(poly);
      }
      poly.verts.push([x, z]);
    }
  }
  return {
    polygons: polygons.map((p) => ({ inclusion: p.inclusion, verts: p.verts })),
    circles, ret, count: items.length,
  };
}

// Test a position [x, y(alt), z] against the fence. Returns a short breach label
// (for the STATUSTEXT warning) or null when inside every boundary. altMax > 0
// enforces an altitude ceiling.
export function fenceBreach(pos, fence, altMax = 0) {
  const [x, y, z] = pos;
  if (altMax > 0 && y > altMax) return 'max altitude';
  if (!fence) return null;
  for (const c of fence.circles) {
    const d = Math.hypot(x - c.x, z - c.z);
    if (c.inclusion ? d > c.r : d < c.r) return c.inclusion ? 'circle boundary' : 'exclusion zone';
  }
  for (const p of fence.polygons) {
    if (p.verts.length < 3) continue;
    const inside = pointInPolygon(x, z, p.verts);
    if (p.inclusion ? !inside : inside) return p.inclusion ? 'polygon boundary' : 'exclusion zone';
  }
  return null;
}
