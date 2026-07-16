// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// RENDER-ONLY satellite/road map tiles textured onto the ground, aligned to the
// sim's own lat/lon frame (telemetry.HOME) so the sim shows the SAME place as the
// GCS map (Incheon). This never touches sim state or determinism — it only adds
// meshes to the scene and loads images. Tiles come from a public XYZ tile service
// (Esri, no API key). Offline / blocked → tiles simply never appear and the base
// green terrain shows through; nothing else changes.

import { HOME } from './telemetry.js';

const R_EARTH = 6378137; // m, WGS-84 (matches telemetry.geodeticToLocal)
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const ZOOM = 15; // ~0.95 km/tile at this latitude
const RADIUS = 3; // tiles each way from home → a (2·RADIUS+1)² grid (~6.6 km)

// Esri map services (CORS-enabled, keyless). Path order is /{z}/{y}/{x}.
const SOURCES = {
  satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile',
  road: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile',
};
const ORDER = ['off', 'satellite', 'road'];

// Web-Mercator (slippy-map) tile math.
function lonLatToTile(lon, lat, z) {
  const n = 2 ** z;
  const x = Math.floor(((lon + 180) / 360) * n);
  const y = Math.floor(((1 - Math.log(Math.tan(lat * D2R) + 1 / Math.cos(lat * D2R)) / Math.PI) / 2) * n);
  return { x, y };
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

export function createMapTiles(THREE, scene, home = HOME) {
  const group = new THREE.Group();
  group.visible = false;
  scene.add(group);

  const loader = new THREE.TextureLoader();
  loader.setCrossOrigin('anonymous');
  const cosLat = Math.cos(home.lat * D2R);
  const center = lonLatToTile(home.lon, home.lat, ZOOM);
  const tiles = [];

  for (let dx = -RADIUS; dx <= RADIUS; dx++) {
    for (let dy = -RADIUS; dy <= RADIUS; dy++) {
      const tx = center.x + dx;
      const ty = center.y + dy;
      const b = tileBounds(tx, ty, ZOOM);
      const cLon = (b.lonW + b.lonE) / 2;
      const cLat = (b.latN + b.latS) / 2;
      // Tile centre → local metres (same mapping as geodeticToLocal): +X east,
      // −Z north. A north-up, east-right tile image maps correctly onto a plane
      // rotated flat (its +Y → world −Z north, +X → east) with default flipY.
      const east = (cLon - home.lon) * D2R * R_EARTH * cosLat;
      const north = (cLat - home.lat) * D2R * R_EARTH;
      const w = (b.lonE - b.lonW) * D2R * R_EARTH * cosLat;
      const h = (b.latN - b.latS) * D2R * R_EARTH;
      const mat = new THREE.MeshLambertMaterial({ color: 0x8a8a8a }); // grey until loaded
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, h), mat);
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.set(east, 0.03, -north); // just above ground (0), below runway (0.04)
      mesh.receiveShadow = true;
      group.add(mesh);
      tiles.push({ mat, x: tx, y: ty, tex: { satellite: null, road: null } });
    }
  }

  let mode = 'off';
  function apply() {
    if (mode === 'off') { group.visible = false; return; }
    group.visible = true;
    for (const t of tiles) {
      if (!t.tex[mode]) {
        t.tex[mode] = loader.load(`${SOURCES[mode]}/${ZOOM}/${t.y}/${t.x}`);
        t.tex[mode].encoding = THREE.sRGBEncoding;
        t.tex[mode].anisotropy = 4;
      }
      t.mat.map = t.tex[mode];
      t.mat.color.setHex(0xffffff); // show the texture at full brightness
      t.mat.needsUpdate = true;
    }
  }

  // Cycle off → satellite → road → off; returns the new mode (for a UI hint).
  function cycle() {
    mode = ORDER[(ORDER.indexOf(mode) + 1) % ORDER.length];
    apply();
    return mode;
  }
  function setMode(m) { mode = ORDER.includes(m) ? m : 'off'; apply(); }

  return { group, cycle, setMode, getMode: () => mode };
}
