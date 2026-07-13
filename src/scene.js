// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// World + aircraft visuals. RENDER-ONLY: nothing here feeds back into the sim.
// Control surfaces and the prop are driven from the actuator states, so what the
// HILS bench commands is what you see. World detail is seeded (deterministic).

import { prngNext } from './prng.js';
import { AC } from './physics.js';

const WORLD_SEED = 7;

function groundTexture(THREE) {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#4d6e3c';
  g.fillRect(0, 0, 256, 256);
  let rng = WORLD_SEED;
  for (let i = 0; i < 1400; i++) {
    let v;
    [v, rng] = prngNext(rng);
    const x = v * 256;
    [v, rng] = prngNext(rng);
    const y = v * 256;
    [v, rng] = prngNext(rng);
    g.fillStyle = v < 0.5 ? 'rgba(38,58,28,0.35)' : 'rgba(96,130,70,0.30)';
    [v, rng] = prngNext(rng);
    const r = 2 + v * 7;
    g.fillRect(x, y, r, r);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(24, 24);
  return tex;
}

function buildRunway(THREE, scene) {
  const asphalt = new THREE.Mesh(
    new THREE.PlaneGeometry(30, 900),
    new THREE.MeshLambertMaterial({ color: 0x2e2e34 })
  );
  asphalt.rotation.x = -Math.PI / 2;
  asphalt.position.y = 0.04;
  asphalt.receiveShadow = true;
  scene.add(asphalt);
  const dash = new THREE.MeshLambertMaterial({ color: 0xd8d8d0 });
  for (let z = -420; z <= 420; z += 40) {
    const line = new THREE.Mesh(new THREE.PlaneGeometry(0.9, 18), dash);
    line.rotation.x = -Math.PI / 2;
    line.position.set(0, 0.05, z);
    scene.add(line);
  }
  for (const end of [-435, 435]) { // threshold bars
    for (let i = -3; i <= 3; i++) {
      if (i === 0) continue;
      const bar = new THREE.Mesh(new THREE.PlaneGeometry(2.2, 12), dash);
      bar.rotation.x = -Math.PI / 2;
      bar.position.set(i * 3.4, 0.05, end);
      scene.add(bar);
    }
  }
}

function scatterTrees(THREE, scene) {
  const trunkMat = new THREE.MeshLambertMaterial({ color: 0x5a4028 });
  const leafMat = new THREE.MeshLambertMaterial({ color: 0x2f5d2a });
  const leafGeo = new THREE.ConeGeometry(2.2, 6, 6);
  const trunkGeo = new THREE.CylinderGeometry(0.35, 0.5, 2.5, 5);
  let rng = WORLD_SEED + 1;
  for (let i = 0; i < 90; i++) {
    let a, b, s;
    [a, rng] = prngNext(rng);
    [b, rng] = prngNext(rng);
    [s, rng] = prngNext(rng);
    const x = (a - 0.5) * 5000;
    const z = (b - 0.5) * 5000;
    if (Math.abs(x) < 60 && Math.abs(z) < 520) continue; // keep the runway clear
    const tree = new THREE.Group();
    const trunk = new THREE.Mesh(trunkGeo, trunkMat);
    trunk.position.y = 1.25;
    const leaf = new THREE.Mesh(leafGeo, leafMat);
    leaf.position.y = 5.2;
    leaf.castShadow = true;
    tree.add(trunk, leaf);
    tree.scale.setScalar(0.8 + s * 1.4);
    tree.position.set(x, 0, z);
    scene.add(tree);
  }
  const hangar = new THREE.Mesh(
    new THREE.BoxGeometry(18, 6, 12),
    new THREE.MeshLambertMaterial({ color: 0x8a8f98 })
  );
  hangar.position.set(45, 3, -60);
  hangar.castShadow = true;
  scene.add(hangar);
}

// Procedural Aerosonde-ish UAV at true scale (b = 2.9 m). Hinged surfaces are
// separate meshes whose geometry is offset so rotation happens about the hinge.
function buildAircraft(THREE) {
  const group = new THREE.Group();
  const grey = new THREE.MeshLambertMaterial({ color: 0xdadde2 });
  const dark = new THREE.MeshLambertMaterial({ color: 0x3c414b });
  const red = new THREE.MeshLambertMaterial({ color: 0xc23b3b });

  const fuselage = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.12, 1.6, 10), grey);
  fuselage.rotation.x = Math.PI / 2;
  fuselage.position.z = 0.1;
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.35, 10), grey);
  nose.rotation.x = -Math.PI / 2;
  nose.position.z = -0.87;
  const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 1.1, 8), dark);
  boom.rotation.x = Math.PI / 2;
  boom.position.z = 1.4;

  const wing = new THREE.Mesh(new THREE.BoxGeometry(AC.b, 0.05, 0.34), grey);
  wing.position.set(0, 0.14, -0.1);
  const tipL = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.06, 0.34), red);
  tipL.position.set(-AC.b / 2 + 0.12, 0.14, -0.1);
  const tipR = tipL.clone();
  tipR.position.x = AC.b / 2 - 0.12;

  // Hinged surfaces: geometry translated so the mesh origin IS the hinge line.
  const surfGeo = (w, ch) => {
    const gm = new THREE.BoxGeometry(w, 0.03, ch);
    gm.translate(0, 0, ch / 2);
    return gm;
  };
  const ailL = new THREE.Mesh(surfGeo(0.75, 0.12), red);
  ailL.position.set(-AC.b / 2 + 0.55, 0.14, 0.07);
  const ailR = new THREE.Mesh(surfGeo(0.75, 0.12), red);
  ailR.position.set(AC.b / 2 - 0.55, 0.14, 0.07);

  const hstab = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.04, 0.22), grey);
  hstab.position.set(0, 0.05, 1.85);
  const elev = new THREE.Mesh(surfGeo(1.0, 0.11), red);
  elev.position.set(0, 0.05, 1.96);

  const fin = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.42, 0.26), grey);
  fin.position.set(0, 0.26, 1.85);
  const rudGeo = new THREE.BoxGeometry(0.03, 0.40, 0.12);
  rudGeo.translate(0, 0, 0.06);
  const rud = new THREE.Mesh(rudGeo, red);
  rud.position.set(0, 0.26, 1.98);

  const prop = new THREE.Group();
  for (const a of [0, Math.PI / 2]) {
    const blade = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.62, 0.02), dark);
    blade.rotation.z = a;
    prop.add(blade);
  }
  prop.position.z = -1.06;

  const gearMat = dark;
  for (const [x, z] of [[-0.35, -0.25], [0.35, -0.25]]) {
    const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.3, 6), gearMat);
    leg.position.set(x, -0.25, z);
    group.add(leg);
  }

  const meshes = [fuselage, nose, boom, wing, tipL, tipR, ailL, ailR, hstab, elev, fin, rud];
  meshes.forEach((m) => { m.castShadow = true; });
  group.add(...meshes, prop);
  return { group, ailL, ailR, elev, rud, prop };
}

export function createWorld(THREE) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x9cc0e8);
  scene.fog = new THREE.Fog(0x9cc0e8, 700, 4200);

  scene.add(new THREE.HemisphereLight(0xe8f0ff, 0x44502f, 0.75));
  const sun = new THREE.DirectionalLight(0xfff1d6, 1.0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 50;
  sun.shadow.camera.far = 600;
  const sc = sun.shadow.camera;
  sc.left = sc.bottom = -40;
  sc.right = sc.top = 40;
  scene.add(sun, sun.target);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(9000, 9000),
    new THREE.MeshLambertMaterial({ map: groundTexture(THREE) })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);
  buildRunway(THREE, scene);
  scatterTrees(THREE, scene);

  const aircraft = buildAircraft(THREE);
  scene.add(aircraft.group);

  const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 6000);
  camera.position.set(0, 123, 10);

  // Everything below is driven by SIM state only — deterministic in, visuals out.
  function update(state, simTime) {
    const g = aircraft.group;
    g.position.set(state.pos[0], state.pos[1] + 0.3, state.pos[2]); // gear height
    g.quaternion.set(state.quat[0], state.quat[1], state.quat[2], state.quat[3]);

    // Surfaces mirror the ACTUATORS (δ in rad): aileron+ = right TE up, left down.
    const k = 1.6; // visual exaggeration so deflections read at a glance
    aircraft.ailR.rotation.x = -state.act.da * k;
    aircraft.ailL.rotation.x = state.act.da * k;
    aircraft.elev.rotation.x = state.act.de * k;
    aircraft.rud.rotation.y = state.act.dr * k;
    aircraft.prop.rotation.z = simTime * (15 + 110 * state.act.dt);

    // Sun follows the aircraft so the shadow frustum stays tight.
    sun.position.set(state.pos[0] + 120, state.pos[1] + 260, state.pos[2] + 60);
    sun.target.position.set(state.pos[0], state.pos[1], state.pos[2]);

    // Chase camera: smoothed, world-up (no roll), looks a little ahead.
    const back = new THREE.Vector3(0, 1.8, 8).applyQuaternion(g.quaternion);
    const want = new THREE.Vector3().copy(g.position).add(back);
    if (camera.position.distanceTo(want) > 25) camera.position.copy(want); // teleport/reset
    else camera.position.lerp(want, 0.08);
    const ahead = new THREE.Vector3(0, 0, -8).applyQuaternion(g.quaternion).add(g.position);
    camera.lookAt(ahead);
  }

  return { scene, camera, update };
}
