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

// Procedural F-16-style airframe (~15 m). Stabilators are all-moving (elevator),
// flaperons + rudder hinge, afterburner flame follows the throttle. RENDER-ONLY.
function buildAircraft(THREE) {
  const group = new THREE.Group();
  const skin = new THREE.MeshPhongMaterial({ color: 0x77808c, shininess: 55, specular: 0x333844 });
  const darkm = new THREE.MeshPhongMaterial({ color: 0x2c313a, shininess: 30 });
  const glass = new THREE.MeshPhongMaterial({ color: 0x2a2418, shininess: 120, specular: 0xccbb77, transparent: true, opacity: 0.92 });

  // Fuselage: lathe profile along the length (nose at −Z).
  const prof = [[0, -7.5], [0.16, -6.6], [0.34, -5.4], [0.52, -3.6], [0.62, -1.2],
    [0.66, 1.2], [0.60, 4.6], [0.46, 6.4], [0.40, 7.2]];
  const lathe = new THREE.LatheGeometry(prof.map(([r, z]) => new THREE.Vector2(Math.max(r, 0.001), z)), 20);
  lathe.rotateX(-Math.PI / 2);
  const fuselage = new THREE.Mesh(lathe, skin);
  const radome = new THREE.Mesh(new THREE.ConeGeometry(0.17, 1.0, 14), darkm);
  radome.rotation.x = -Math.PI / 2;
  radome.position.z = -7.0;
  const canopy = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), glass);
  canopy.scale.set(0.52, 0.55, 1.7);
  canopy.position.set(0, 0.55, -3.6);
  const intake = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.5, 2.6, 12), darkm);
  intake.rotation.x = Math.PI / 2;
  intake.position.set(0, -0.55, -0.6);

  const wingShape = (root, tip, span, sweep) => {
    const sh = new THREE.Shape();
    sh.moveTo(0, 0);
    sh.lineTo(span, -sweep);
    sh.lineTo(span, -sweep - tip);
    sh.lineTo(0, -root);
    sh.closePath();
    const g = new THREE.ExtrudeGeometry(sh, { depth: 0.09, bevelEnabled: false });
    g.rotateX(Math.PI / 2); // shape y → −Z (chord), extrude → up
    return g;
  };
  const wingL = new THREE.Mesh(wingShape(4.6, 1.1, 4.6, 3.4), skin);
  wingL.position.set(0, -0.05, -1.4);
  wingL.scale.x = -1;
  const wingR = new THREE.Mesh(wingShape(4.6, 1.1, 4.6, 3.4), skin);
  wingR.position.set(0, -0.05, -1.4);

  const hinged = (w, ch) => {
    const g = new THREE.BoxGeometry(w, 0.06, ch);
    g.translate(0, 0, ch / 2);
    return g;
  };
  const flapL = new THREE.Mesh(hinged(2.2, 0.5), darkm);
  flapL.position.set(-3.1, 0, 3.15);
  const flapR = new THREE.Mesh(hinged(2.2, 0.5), darkm);
  flapR.position.set(3.1, 0, 3.15);

  // All-moving stabilators: pivot at their leading edge.
  const stabGeo = wingShape(1.9, 0.7, 2.4, 1.5);
  stabGeo.translate(0, 0, 0); // pivot ~ LE already at z=0 of geometry
  const stabL = new THREE.Mesh(stabGeo, skin);
  stabL.position.set(0, 0.05, 5.4);
  stabL.scale.x = -1;
  const stabR = new THREE.Mesh(wingShape(1.9, 0.7, 2.4, 1.5), skin);
  stabR.position.set(0, 0.05, 5.4);

  const finShape = new THREE.Shape();
  finShape.moveTo(0, 0); finShape.lineTo(2.9, 1.9); finShape.lineTo(2.9, 2.9);
  finShape.lineTo(1.1, 0); finShape.closePath();
  const finGeo = new THREE.ExtrudeGeometry(finShape, { depth: 0.08, bevelEnabled: false });
  finGeo.rotateY(Math.PI / 2); // shape x → +Z (aft), y up
  const fin = new THREE.Mesh(finGeo, skin);
  fin.position.set(-0.04, 0.4, 3.4);
  const rudGeo = new THREE.BoxGeometry(0.06, 1.6, 0.55);
  rudGeo.translate(0, 0.8, 0.27);
  const rud = new THREE.Mesh(rudGeo, darkm);
  rud.position.set(0, 1.6, 6.4);

  // Afterburner: nozzle + throttle-driven flame (additive, deterministic flicker).
  const nozzle = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.34, 0.7, 14), darkm);
  nozzle.rotation.x = Math.PI / 2;
  nozzle.position.z = 7.4;
  const flameGeo = new THREE.ConeGeometry(0.3, 1, 12);
  flameGeo.translate(0, -0.5, 0);
  const flame = new THREE.Mesh(flameGeo, new THREE.MeshBasicMaterial({
    color: 0xff8a2a, transparent: true, opacity: 0.85, blending: THREE.AdditiveBlending, depthWrite: false,
  }));
  flame.rotation.x = -Math.PI / 2;
  flame.position.z = 7.7;

  const rails = [];
  for (const x of [-4.55, 4.55]) {
    const rail = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 2.6, 8), darkm);
    rail.rotation.x = Math.PI / 2;
    rail.position.set(x, 0, 1.9);
    rails.push(rail);
  }

  const meshes = [fuselage, radome, canopy, intake, wingL, wingR, flapL, flapR, stabL, stabR, fin, rud, nozzle, ...rails];
  meshes.forEach((m) => { m.castShadow = true; });
  group.add(...meshes, flame);
  return { group, flapL, flapR, stabL, stabR, rud, flame };
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
  sc.left = sc.bottom = -70;
  sc.right = sc.top = 70;
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
    g.position.set(state.pos[0], state.pos[1] + 1.5, state.pos[2]); // gear height
    g.quaternion.set(state.quat[0], state.quat[1], state.quat[2], state.quat[3]);

    // Surfaces mirror the ACTUATORS (δ in rad): aileron+ = right TE up, left down.
    const k = 1.6; // visual exaggeration so deflections read at a glance
    aircraft.flapR.rotation.x = -state.act.da * k;
    aircraft.flapL.rotation.x = state.act.da * k;
    aircraft.stabL.rotation.x = state.act.de * k; // all-moving stabilators
    aircraft.stabR.rotation.x = state.act.de * k;
    aircraft.rud.rotation.y = state.act.dr * k;
    // Afterburner: grows with throttle, deterministic flicker from sim time.
    const burn = Math.max(0, state.act.dt - 0.15);
    const flick = 1 + 0.18 * Math.sin(simTime * 47) * Math.sin(simTime * 31);
    aircraft.flame.scale.set(0.7 + burn, 0.7 + burn, (0.4 + 5.5 * burn * burn) * flick);
    aircraft.flame.material.opacity = Math.min(0.9, 0.25 + burn);

    // Sun follows the aircraft so the shadow frustum stays tight.
    sun.position.set(state.pos[0] + 120, state.pos[1] + 260, state.pos[2] + 60);
    sun.target.position.set(state.pos[0], state.pos[1], state.pos[2]);

    // Chase camera: smoothed, world-up (no roll), looks a little ahead.
    const back = new THREE.Vector3(0, 6, 30).applyQuaternion(g.quaternion);
    const want = new THREE.Vector3().copy(g.position).add(back);
    if (camera.position.distanceTo(want) > 80) camera.position.copy(want); // teleport/reset
    else camera.position.lerp(want, 0.08);
    const ahead = new THREE.Vector3(0, 0, -25).applyQuaternion(g.quaternion).add(g.position);
    camera.lookAt(ahead);
  }

  return { scene, camera, update };
}
