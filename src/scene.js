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

// Procedural Aerosonde-class UAV at true scale (b = 2.9 m): sleek sensor-pod
// fuselage, high-AR tapered wing, twin tail booms, pusher prop. Surfaces mirror
// the actuators; the prop spin is a deterministic function of sim state.
function buildAircraft(THREE) {
  const group = new THREE.Group();
  const skin = new THREE.MeshPhongMaterial({ color: 0xe3e6ea, shininess: 70, specular: 0x445066 });
  const darkm = new THREE.MeshPhongMaterial({ color: 0x2e333c, shininess: 40 });
  const marker = new THREE.MeshPhongMaterial({ color: 0xd4491f, shininess: 50 });

  // Pod fuselage (lathe), nose at −Z; sensor turret ball under the chin.
  const prof = [[0, -0.85], [0.09, -0.7], [0.145, -0.42], [0.16, -0.05], [0.15, 0.35], [0.11, 0.62], [0.05, 0.8]];
  const lathe = new THREE.LatheGeometry(prof.map(([r, z]) => new THREE.Vector2(Math.max(r, 0.001), z)), 18);
  lathe.rotateX(-Math.PI / 2);
  const pod = new THREE.Mesh(lathe, skin);
  const turret = new THREE.Mesh(new THREE.SphereGeometry(0.085, 14, 10), darkm);
  turret.position.set(0, -0.13, -0.55);
  const pitot = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.3, 6), darkm);
  pitot.rotation.x = Math.PI / 2;
  pitot.position.set(0.06, 0.02, -0.95);
  const gps = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 0.03, 10), darkm);
  gps.position.set(0, 0.17, -0.1);

  // High-AR tapered wing (extruded planform), slight dihedral, orange tips.
  const half = (mirror) => {
    const sh = new THREE.Shape();
    sh.moveTo(0, 0.19);
    sh.lineTo(1.45, 0.03);
    sh.lineTo(1.45, -0.09);
    sh.lineTo(0, -0.16);
    sh.closePath();
    const g = new THREE.ExtrudeGeometry(sh, { depth: 0.045, bevelEnabled: false });
    g.rotateX(Math.PI / 2);
    const m = new THREE.Mesh(g, skin);
    m.position.set(0, 0.2, -0.15);
    m.rotation.z = mirror * 0.05; // dihedral
    m.scale.x = mirror;
    return m;
  };
  const wingL = half(-1);
  const wingR = half(1);
  const tipL = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.05, 0.13), marker);
  tipL.position.set(-1.43, 0.27, -0.16);
  const tipR = tipL.clone();
  tipR.position.x = 1.43;

  const hinged = (w, ch) => {
    const g = new THREE.BoxGeometry(w, 0.025, ch);
    g.translate(0, 0, ch / 2);
    return g;
  };
  const ailL = new THREE.Mesh(hinged(0.62, 0.09), marker);
  ailL.position.set(-1.05, 0.2, -0.02);
  const ailR = new THREE.Mesh(hinged(0.62, 0.09), marker);
  ailR.position.set(1.05, 0.2, -0.02);

  // Twin tail booms → stabilizer + elevator between twin fins with rudders.
  const booms = [];
  for (const x of [-0.42, 0.42]) {
    const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.018, 1.35, 8), darkm);
    boom.rotation.x = Math.PI / 2;
    boom.position.set(x, 0.16, 0.62);
    booms.push(boom);
  }
  const hstab = new THREE.Mesh(new THREE.BoxGeometry(0.88, 0.03, 0.17), skin);
  hstab.position.set(0, 0.16, 1.2);
  const elev = new THREE.Mesh(hinged(0.86, 0.09), marker);
  elev.position.set(0, 0.16, 1.29);
  const fins = [];
  const ruds = [];
  for (const x of [-0.42, 0.42]) {
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.34, 0.2), skin);
    fin.position.set(x, 0.33, 1.18);
    fins.push(fin);
    const rg = new THREE.BoxGeometry(0.02, 0.3, 0.07);
    rg.translate(0, 0, 0.035);
    const rud = new THREE.Mesh(rg, marker);
    rud.position.set(x, 0.33, 1.29);
    ruds.push(rud);
  }

  // Pusher prop at the pod tail.
  const prop = new THREE.Group();
  for (const a of [0, Math.PI / 2]) {
    const blade = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.42, 0.012), darkm);
    blade.rotation.z = a;
    prop.add(blade);
  }
  prop.position.set(0, 0, 0.84);

  for (const [x, z] of [[-0.16, -0.35], [0.16, -0.35], [0, 0.5]]) { // fixed gear
    const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.22, 6), darkm);
    leg.position.set(x, -0.2, z);
    group.add(leg);
  }

  const meshes = [pod, turret, pitot, gps, wingL, wingR, tipL, tipR, ailL, ailR, ...booms, hstab, elev, ...fins, ...ruds];
  meshes.forEach((m) => { m.castShadow = true; });
  group.add(...meshes, prop);
  return { group, ailL, ailR, elev, ruds, prop };
}

export function createWorld(THREE) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x86aed6);
  scene.fog = new THREE.Fog(0xb9cbdd, 900, 5200);

  // Gradient sky dome (cinematic horizon haze), render-only.
  const sky = new THREE.Mesh(
    new THREE.SphereGeometry(4500, 20, 12),
    new THREE.ShaderMaterial({
      side: THREE.BackSide, depthWrite: false, fog: false,
      uniforms: { top: { value: new THREE.Color(0x3f6fb5) }, bot: { value: new THREE.Color(0xd9e2ea) } },
      vertexShader: 'varying float h; void main(){ h = normalize(position).y; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }',
      fragmentShader: 'uniform vec3 top; uniform vec3 bot; varying float h; void main(){ gl_FragColor = vec4(mix(bot, top, clamp(h * 1.6, 0.0, 1.0)), 1.0); }',
    })
  );
  scene.add(sky);

  scene.add(new THREE.HemisphereLight(0xcfe0f5, 0x3d4634, 0.55));
  const sun = new THREE.DirectionalLight(0xffe3b8, 1.35);
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
    g.position.set(state.pos[0], state.pos[1] + 0.32, state.pos[2]); // gear height
    g.quaternion.set(state.quat[0], state.quat[1], state.quat[2], state.quat[3]);

    // Surfaces mirror the ACTUATORS (δ in rad): aileron+ = right TE up, left down.
    const k = 1.6; // visual exaggeration so deflections read at a glance
    aircraft.ailR.rotation.x = -state.act.da * k;
    aircraft.ailL.rotation.x = state.act.da * k;
    aircraft.elev.rotation.x = state.act.de * k;
    aircraft.ruds.forEach((r) => { r.rotation.y = state.act.dr * k; });
    aircraft.prop.rotation.z = simTime * (15 + 110 * state.act.dt); // pusher prop

    // Sun follows the aircraft so the shadow frustum stays tight.
    sun.position.set(state.pos[0] + 120, state.pos[1] + 260, state.pos[2] + 60);
    sun.target.position.set(state.pos[0], state.pos[1], state.pos[2]);

    // Chase camera: smoothed, world-up (no roll), looks a little ahead.
    const back = new THREE.Vector3(0, 1.6, 7.5).applyQuaternion(g.quaternion);
    const want = new THREE.Vector3().copy(g.position).add(back);
    if (camera.position.distanceTo(want) > 25) camera.position.copy(want); // teleport/reset
    else camera.position.lerp(want, 0.08);
    const ahead = new THREE.Vector3(0, 0, -8).applyQuaternion(g.quaternion).add(g.position);
    camera.lookAt(ahead);
  }

  return { scene, camera, update, ground };
}
