// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// RENDER-ONLY map imagery. Instead of overlaying separate tile planes (which
// z-fight the ground and flicker), this BAKES satellite/road tiles into a single
// canvas and SWAPS the existing ground plane's texture. One ground surface → no
// z-fighting is possible; the runway stays on top exactly as before. Aligned to
// the sim's own lat/lon frame (telemetry.HOME) so the sim shows the SAME place as
// the GCS map (Incheon). Never touches sim state or determinism — it only swaps a
// texture. Tiles come from a keyless public XYZ service (Esri); offline/blocked →
// the tiles just never paint and the ground stays green.

import { HOME } from './telemetry.js';

const R_EARTH = 6378137; // m, WGS-84 (matches telemetry.geodeticToLocal)
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const ZOOM = 15; // ~4 m/px imagery at this latitude
const RADIUS = 3; // tiles each way from home → a (2·RADIUS+1)² footprint (~6.6 km)
const GROUND_SIZE = 9000; // MUST match the ground plane in scene.js
const CANVAS = 2048; // baked-texture resolution (≈ tile native detail over 9 km)
const GREEN = '#4d6e3c'; // fill outside the imagery footprint (matches scene grass)

// Esri map services (CORS-enabled, keyless). Path order is /{z}/{y}/{x}.
const SOURCES = {
  satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile',
  road: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile',
};
const ORDER = ['off', 'satellite', 'road'];

function lonLatToTile(lon, lat, z) {
  const n = 2 ** z;
  return {
    x: Math.floor(((lon + 180) / 360) * n),
    y: Math.floor(((1 - Math.log(Math.tan(lat * D2R) + 1 / Math.cos(lat * D2R)) / Math.PI) / 2) * n),
  };
}
function tileBounds(x, y, z) {
  const n = 2 ** z;
  return {
    lonW: (x / n) * 360 - 180,
    lonE: ((x + 1) / n) * 360 - 180,
    latN: Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / n))) * R2D,
    latS: Math.atan(Math.sinh(Math.PI * (1 - (2 * (y + 1)) / n))) * R2D,
  };
}

export function createMapTiles(THREE, ground, home = HOME) {
  const cosLat = Math.cos(home.lat * D2R);
  const originalMap = ground.material.map; // the green grass texture, to restore
  const built = {}; // mode → baked CanvasTexture (lazy, network only when shown)
  let mode = 'off';

  // Bake one mode's tiles into a single north-up canvas covering the 9 km ground.
  function bake(m) {
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = CANVAS;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = GREEN;
    ctx.fillRect(0, 0, CANVAS, CANVAS); // outside-imagery filler
    const tex = new THREE.CanvasTexture(canvas);
    tex.encoding = THREE.sRGBEncoding;
    tex.anisotropy = 4;

    const center = lonLatToTile(home.lon, home.lat, ZOOM);
    for (let dx = -RADIUS; dx <= RADIUS; dx++) {
      for (let dy = -RADIUS; dy <= RADIUS; dy++) {
        const tx = center.x + dx;
        const ty = center.y + dy;
        const b = tileBounds(tx, ty, ZOOM);
        const cLon = (b.lonW + b.lonE) / 2;
        const cLat = (b.latN + b.latS) / 2;
        // tile centre → local metres (east +X, north = −Z), then → canvas pixels.
        // Canvas is north-up / east-right, matching the ground plane's UVs.
        const east = (cLon - home.lon) * D2R * R_EARTH * cosLat;
        const north = (cLat - home.lat) * D2R * R_EARTH;
        const wpx = ((b.lonE - b.lonW) * D2R * R_EARTH * cosLat) / GROUND_SIZE * CANVAS;
        const hpx = ((b.latN - b.latS) * D2R * R_EARTH) / GROUND_SIZE * CANVAS;
        const px = ((east + GROUND_SIZE / 2) / GROUND_SIZE) * CANVAS;
        const py = ((GROUND_SIZE / 2 - north) / GROUND_SIZE) * CANVAS;
        const img = new Image();
        img.crossOrigin = 'anonymous';
        img.onload = () => {
          ctx.drawImage(img, px - wpx / 2, py - hpx / 2, wpx + 1, hpx + 1); // +1 hides seams
          tex.needsUpdate = true;
        };
        img.onerror = () => {}; // offline/blocked tile: leave the green filler
        img.src = `${SOURCES[m]}/${ZOOM}/${ty}/${tx}`;
      }
    }
    return tex;
  }

  function apply() {
    if (mode === 'off') {
      ground.material.map = originalMap;
    } else {
      if (!built[mode]) built[mode] = bake(mode);
      ground.material.map = built[mode];
    }
    ground.material.needsUpdate = true;
  }

  // Cycle off → satellite → road → off; returns the new mode (for a UI hint).
  function cycle() {
    mode = ORDER[(ORDER.indexOf(mode) + 1) % ORDER.length];
    apply();
    return mode;
  }
  function setMode(m) { mode = ORDER.includes(m) ? m : 'off'; apply(); }

  return { cycle, setMode, getMode: () => mode };
}
