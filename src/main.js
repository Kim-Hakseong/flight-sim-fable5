// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Wiring + the deterministic fixed-step loop. All sim time comes from DT-sized steps;
// wall clock only decides HOW MANY steps to run — never feeds the physics.

import { stepAircraft, initialState } from './physics.js';

const THREE = window.THREE;
const DT = 1 / 60; // s — fixed physics timestep
const THROTTLE_RATE = 0.5; // full-range throttle travel per second (dt-integrated)

// --- Sim state -------------------------------------------------------------
let state = initialState();
let throttle = 0.65;
let simTime = 0;
let manual = false; // once __advance is used, wall-clock stepping stops (test/HILS mode)

const keys = new Set();
window.addEventListener('keydown', (e) => {
  // e.code (physical key), never e.key — IME layouts report e.key as 'Process'.
  keys.add(e.code);
  if (e.code === 'KeyR') reset();
  if (e.code.startsWith('Arrow')) e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());

function readControls() {
  return {
    pitch: (keys.has('ArrowUp') ? 1 : 0) - (keys.has('ArrowDown') ? 1 : 0),
    roll: (keys.has('ArrowRight') ? 1 : 0) - (keys.has('ArrowLeft') ? 1 : 0),
    yaw: (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0),
    throttle,
  };
}

function stepSim(dt) {
  const dThr = ((keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0)) * THROTTLE_RATE * dt;
  throttle = Math.min(1, Math.max(0, throttle + dThr));
  state = stepAircraft(state, readControls(), dt);
  simTime += dt;
}

function reset() {
  state = initialState();
  throttle = 0.65;
  simTime = 0;
  keys.clear();
}

// --- Deterministic test/HILS surface ----------------------------------------
window.__advance = (seconds, dt = DT) => {
  manual = true;
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) stepSim(dt);
  return window.__state();
};
window.__reset = () => reset();
window.__state = () => JSON.stringify({ simTime, throttle, ...state });

// --- Scene -------------------------------------------------------------------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x87b5e0);
scene.fog = new THREE.Fog(0x87b5e0, 600, 3500);

const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
document.body.appendChild(renderer.domElement);
addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

scene.add(new THREE.HemisphereLight(0xdfeaff, 0x3a4a2f, 0.9));
const sun = new THREE.DirectionalLight(0xfff2d8, 0.8);
sun.position.set(200, 400, 100);
scene.add(sun);

const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(8000, 8000),
  new THREE.MeshLambertMaterial({ color: 0x4a6b3a })
);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);
const grid = new THREE.GridHelper(4000, 80, 0x33502a, 0x3d5c31);
grid.position.y = 0.05;
scene.add(grid);

// Minimal aircraft: fuselage + wing + tail boxes (orientation must be readable).
const aircraft = new THREE.Group();
const mat = new THREE.MeshLambertMaterial({ color: 0xd8dbe0 });
const wingMat = new THREE.MeshLambertMaterial({ color: 0xc23b3b });
const fuselage = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.9, 6), mat);
const wing = new THREE.Mesh(new THREE.BoxGeometry(8, 0.15, 1.3), wingMat);
wing.position.set(0, 0.2, -0.3);
const tail = new THREE.Mesh(new THREE.BoxGeometry(3, 0.12, 0.8), wingMat);
tail.position.set(0, 0.3, 2.6);
const fin = new THREE.Mesh(new THREE.BoxGeometry(0.12, 1.1, 0.9), mat);
fin.position.set(0, 0.8, 2.6);
aircraft.add(fuselage, wing, tail, fin);
scene.add(aircraft);

const hud = document.getElementById('hud');

function render() {
  aircraft.position.set(state.pos[0], state.pos[1], state.pos[2]);
  aircraft.quaternion.set(state.quat[0], state.quat[1], state.quat[2], state.quat[3]);

  // Chase camera: behind and above the aircraft, world-up (no camera roll).
  const back = new THREE.Vector3(0, 3.5, 14).applyQuaternion(aircraft.quaternion);
  camera.position.copy(aircraft.position).add(back);
  camera.lookAt(aircraft.position);

  const spd = Math.hypot(state.vel[0], state.vel[1], state.vel[2]);
  hud.textContent =
    `SPD ${spd.toFixed(1).padStart(5)} m/s` +
    `\nALT ${state.pos[1].toFixed(1).padStart(5)} m` +
    `\nTHR ${(throttle * 100).toFixed(0).padStart(4)} %` +
    `\nT+  ${simTime.toFixed(2).padStart(6)} s${manual ? '  [manual]' : ''}`;

  renderer.render(scene, camera);
}

// --- Fixed-step loop: accumulate wall time, step in exact DT increments ------
let acc = 0;
let last = null;
function frame(tMs) {
  requestAnimationFrame(frame);
  if (!manual && last !== null) {
    acc += Math.min((tMs - last) / 1000, 0.25); // clamp: don't spiral after a stall
    while (acc >= DT) {
      stepSim(DT);
      acc -= DT;
    }
  }
  last = tMs;
  render();
}
requestAnimationFrame(frame);

window.__ready = true;
