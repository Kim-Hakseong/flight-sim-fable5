import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createWind, stepWind, DRYDEN } from '../src/wind.js';
import { stepAircraft, initialState, airData } from '../src/physics.js';
import { holdControls, headingErrorDeg } from '../src/autopilot.js';
import { eulerFromQuat, headingDeg } from '../src/telemetry.js';
import { defaultParams } from '../src/params.js';

const DT = 1 / 60;
const P = defaultParams();

function flyWithWind(seconds, P2, seed = 2, targetAlt = 120, targetHdg = 0) {
  let s = initialState();
  let w = createWind(seed);
  let track = { minAlt: Infinity, maxAlt: -Infinity, minVa: Infinity, maxVa: -Infinity };
  let ww = [0, 0, 0];
  for (let i = 0; i < seconds * 60; i++) {
    const step = stepWind(w, s, P2, DT);
    w = step.wind;
    ww = step.windWorld;
    const va = airData(s.quat, s.vel, ww).Va;
    s = stepAircraft(s, holdControls(s, targetAlt, targetHdg, null, undefined, va), DT, ww);
    if (i > 10 * 60) {
      const va = airData(s.quat, s.vel, ww).Va;
      track.minAlt = Math.min(track.minAlt, s.pos[1]);
      track.maxAlt = Math.max(track.maxAlt, s.pos[1]);
      track.minVa = Math.min(track.minVa, va);
      track.maxVa = Math.max(track.maxVa, va);
    }
  }
  return { s, track, ww };
}

test('determinism: same seed → identical gust stream', () => {
  const run = () => {
    let w = createWind(9);
    const s = initialState();
    const out = [];
    for (let i = 0; i < 200; i++) {
      const r = stepWind(w, s, { ...P, WND_TRB: 2 }, DT);
      w = r.wind;
      out.push(r.windWorld);
    }
    return JSON.stringify(out);
  };
  assert.equal(run(), run());
});

test('calm: zero intensity + zero steady wind → windWorld ≈ 0', () => {
  let w = createWind(3);
  const s = initialState();
  const P2 = { ...P, WND_TRB: 0, WND_N_MS: 0, WND_E_MS: 0 };
  for (let i = 0; i < 300; i++) {
    const r = stepWind(w, s, P2, DT);
    w = r.wind;
    assert.ok(Math.hypot(...r.windWorld) < 1e-12, `gust leaked: ${r.windWorld}`);
  }
});

test('dryden: stationary gust sigma matches the model (u axis)', () => {
  let w = createWind(4);
  const s = initialState();
  const P2 = { ...P, WND_TRB: 1.5 };
  let sum = 0, sumSq = 0;
  const N = 40000;
  for (let i = 0; i < N; i++) {
    const r = stepWind(w, s, P2, DT);
    w = r.wind;
    sum += w.gust[0];
    sumSq += w.gust[0] ** 2;
  }
  const mean = sum / N;
  const sigma = Math.sqrt(sumSq / N - mean * mean);
  const want = DRYDEN.sigma[0] * P2.WND_TRB;
  assert.ok(Math.abs(sigma - want) / want < 0.15, `gust-u sigma ${sigma} vs ${want}`);
});

test('crosswind: heading holds, ground track crabs downwind', () => {
  const { s } = flyWithWind(60, { ...P, WND_TRB: 0, WND_E_MS: 8 });
  const e = eulerFromQuat(s.quat);
  assert.ok(Math.abs(headingErrorDeg(0, headingDeg(e.yaw))) < 4, `hdg = ${headingDeg(e.yaw)}`);
  assert.ok(s.pos[0] > 200, `east drift = ${s.pos[0]} m (wind must push the track)`);
});

test('headwind: airspeed is held, groundspeed drops', () => {
  const { s, ww } = flyWithWind(60, { ...P, WND_TRB: 0, WND_N_MS: -10 }); // wind TO south = headwind
  const va = airData(s.quat, s.vel, ww).Va;
  const gs = Math.hypot(s.vel[0], s.vel[2]);
  assert.ok(Math.abs(va - 30) < 2, `Va = ${va}`);
  assert.ok(gs < va - 6, `groundspeed ${gs} must be well below airspeed ${va}`);
});

test('turbulence: autopilot rides out heavy Dryden inside the altitude band', () => {
  const { track } = flyWithWind(60, { ...P, WND_TRB: 2 });
  assert.ok(track.minAlt > 110 && track.maxAlt < 130, `alt band ${track.minAlt}–${track.maxAlt}`);
  assert.ok(track.minVa > 24 && track.maxVa < 36, `Va band ${track.minVa}–${track.maxVa}`);
});
