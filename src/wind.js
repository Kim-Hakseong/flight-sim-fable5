// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Wind = steady N/E components (params, live-settable) + Dryden gusts: a seeded
// first-order Gauss–Markov process per body axis (Beard & McLain Table-4.1-style
// scale lengths / sigmas). Pure and deterministic: the PRNG state is threaded
// exactly like the sensors', and every step draws whether faulted^W scaled or not.

import { gaussianNext } from './prng.js';
import { quatRotate, fromFRD, airData } from './physics.js';

export const DRYDEN = { L: [200, 200, 50], sigma: [1.06, 1.06, 0.7] }; // u, v, w

export function createWind(seed = 2) {
  return { rng: seed | 0, gust: [0, 0, 0] }; // gust in body FRD [u, v, w] m/s
}

// One step. Returns { wind, windWorld } — windWorld is what the physics eats.
// Uses the previous step's no-wind Va as the frozen-turbulence transport speed.
export function stepWind(wind, state, P, dt) {
  const { Va } = airData(state.quat, state.vel);
  const V = Math.max(Va, 5);
  let rng = wind.rng;
  const gust = wind.gust.map((g, i) => {
    const [eta, next] = gaussianNext(rng);
    rng = next;
    const a = Math.min((V / DRYDEN.L[i]) * dt, 1);
    // Gauss–Markov: stationary sigma ≈ DRYDEN.sigma·WND_TRB for small a.
    return g * (1 - a) + DRYDEN.sigma[i] * P.WND_TRB * Math.sqrt(2 * a) * eta;
  });
  const gw = quatRotate(state.quat, fromFRD(gust));
  const windWorld = [
    P.WND_E_MS + gw[0], // east = +X
    gw[1],
    -P.WND_N_MS + gw[2], // north = −Z
  ];
  return { wind: { rng, gust }, windWorld };
}
