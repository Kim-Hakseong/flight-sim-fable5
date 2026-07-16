import { test } from 'node:test';
import assert from 'node:assert/strict';
import { buildFence, fenceBreach, pointInPolygon, FENCE_CMD } from '../src/geofence.js';
import { HOME } from '../src/telemetry.js';

test('pointInPolygon: inside / outside a unit square', () => {
  const sq = [[-10, -10], [10, -10], [10, 10], [-10, 10]];
  assert.equal(pointInPolygon(0, 0, sq), true);
  assert.equal(pointInPolygon(20, 0, sq), false);
  assert.equal(pointInPolygon(0, 20, sq), false);
  assert.equal(pointInPolygon(-9.9, 9.9, sq), true);
});

test('fenceBreach: circle inclusion — inside ok, outside breaches', () => {
  const fence = { circles: [{ inclusion: true, x: 0, z: 0, r: 100 }], polygons: [] };
  assert.equal(fenceBreach([50, 60, 0], fence), null);
  assert.equal(fenceBreach([150, 60, 0], fence), 'circle boundary');
});

test('fenceBreach: circle exclusion — inside breaches, outside ok', () => {
  const fence = { circles: [{ inclusion: false, x: 200, z: 0, r: 50 }], polygons: [] };
  assert.equal(fenceBreach([200, 60, 10], fence), 'exclusion zone');
  assert.equal(fenceBreach([0, 60, 0], fence), null);
});

test('fenceBreach: polygon inclusion / exclusion', () => {
  const inc = { circles: [], polygons: [{ inclusion: true, verts: [[-50, -50], [50, -50], [50, 50], [-50, 50]] }] };
  assert.equal(fenceBreach([0, 60, 0], inc), null);
  assert.equal(fenceBreach([80, 60, 0], inc), 'polygon boundary');
  const exc = { circles: [], polygons: [{ inclusion: false, verts: [[-50, -50], [50, -50], [50, 50], [-50, 50]] }] };
  assert.equal(fenceBreach([0, 60, 0], exc), 'exclusion zone');
  assert.equal(fenceBreach([80, 60, 0], exc), null);
});

test('fenceBreach: altitude ceiling (altMax) bites even with no geometry', () => {
  assert.equal(fenceBreach([0, 130, 0], null, 120), 'max altitude');
  assert.equal(fenceBreach([0, 110, 0], null, 120), null);
  assert.equal(fenceBreach([0, 130, 0], null, 0), null); // 0 = disabled
});

test('fenceBreach: a degenerate (<3 vertex) polygon is ignored, not a breach', () => {
  const fence = { circles: [], polygons: [{ inclusion: true, verts: [[0, 0], [10, 0]] }] };
  assert.equal(fenceBreach([999, 60, 999], fence), null);
});

test('buildFence: a circle-inclusion item → one circle, radius from param1', () => {
  const fence = buildFence([
    { command: FENCE_CMD.CIRCLE_INCLUSION, lat: HOME.lat, lon: HOME.lon, param1: 250 },
  ]);
  assert.equal(fence.circles.length, 1);
  assert.equal(fence.circles[0].inclusion, true);
  assert.equal(fence.circles[0].r, 250);
  // HOME maps to the local origin (± rounding).
  assert.ok(Math.hypot(fence.circles[0].x, fence.circles[0].z) < 1);
});

test('buildFence: consecutive vertices group into polygons by param1 count', () => {
  const v = (dLat, dLon) => ({ command: FENCE_CMD.POLY_INCLUSION, lat: HOME.lat + dLat, lon: HOME.lon + dLon, param1: 4 });
  const fence = buildFence([
    v(0.001, 0.001), v(0.001, -0.001), v(-0.001, -0.001), v(-0.001, 0.001), // polygon A (4)
    v(0.002, 0.002), v(0.002, 0.001), v(0.001, 0.001), v(0.001, 0.002), // polygon B (4)
  ]);
  assert.equal(fence.polygons.length, 2);
  assert.equal(fence.polygons[0].verts.length, 4);
  assert.equal(fence.polygons[1].verts.length, 4);
  assert.equal(fence.polygons[0].inclusion, true);
});

test('buildFence: exclusion polygon + return point are captured', () => {
  const fence = buildFence([
    { command: FENCE_CMD.RETURN_POINT, lat: HOME.lat, lon: HOME.lon, param1: 0 },
    { command: FENCE_CMD.POLY_EXCLUSION, lat: HOME.lat + 0.001, lon: HOME.lon, param1: 3 },
    { command: FENCE_CMD.POLY_EXCLUSION, lat: HOME.lat + 0.001, lon: HOME.lon + 0.001, param1: 3 },
    { command: FENCE_CMD.POLY_EXCLUSION, lat: HOME.lat, lon: HOME.lon + 0.001, param1: 3 },
  ]);
  assert.ok(fence.ret);
  assert.equal(fence.polygons.length, 1);
  assert.equal(fence.polygons[0].inclusion, false);
  assert.equal(fence.polygons[0].verts.length, 3);
});
