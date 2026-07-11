// Coordinate frame (do not confuse): Three.js right-handed, +Y up, −Z forward.
// Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +, yaw-right +.
// Wiring + the deterministic fixed-step loop. All sim time comes from DT-sized steps;
// wall clock only decides HOW MANY steps to run — never feeds the physics.

import { stepAircraft, initialState } from './physics.js';
import { startTelemetry, telemetryFrom, eulerFromQuat, headingDeg } from './telemetry.js';
import { MODES, MODE_NAMES, apStep } from './autopilot.js';
import { startMissionLink } from './missionLink.js';
import { toLocalTargets } from './missions.js';
import { geodeticToLocal } from './telemetry.js';

const THREE = window.THREE;
const DT = 1 / 60; // s — fixed physics timestep
const THROTTLE_RATE = 0.5; // full-range throttle travel per second (dt-integrated)

// --- Sim state -------------------------------------------------------------
let state = initialState();
let throttle = 0.65; // pilot's throttle setting (MANUAL mode)
let simTime = 0;
let manual = false; // once __advance is used, wall-clock stepping stops (test/HILS mode)

// The sim is authoritative for arm/mode; the GCS only requests changes.
let armed = true; // boots armed + airborne so the standalone sim flies immediately
let ap = {
  mode: MODES.MANUAL, landing: false, targetAlt: 120, targetHeading: 0,
  guided: null, mission: null,
};
let lastControls = { pitch: 0, roll: 0, yaw: 0, throttle };
let lastReached = -1; // last mission seq completed (edge → MISSION_ITEM_REACHED)

function setMode(m) {
  const e = eulerFromQuat(state.quat);
  // Capture "here" as the hold target; the mission survives mode changes.
  ap = {
    mode: m, landing: false,
    targetAlt: Math.max(40, state.pos[1]),
    targetHeading: headingDeg(e.yaw),
    guided: null,
    mission: ap.mission,
  };
}

function applyCommand(cmd) {
  switch (cmd.type) {
    case 'arm': armed = cmd.value === 1; break;
    case 'mode': setMode(cmd.custom >>> 0); break;
    case 'takeoff': armed = true; setMode(MODES.TAKEOFF); ap.targetAlt = cmd.alt; break;
    case 'land': ap = { ...ap, landing: true }; break;
    case 'rtl': setMode(MODES.RTL); break;
    case 'mission':
      ap = { ...ap, mission: { targets: toLocalTargets(cmd.items), idx: 0 } };
      lastReached = -1;
      break;
    case 'goto': {
      const [x, , z] = geodeticToLocal({ lat: cmd.lat, lon: cmd.lon });
      setMode(MODES.GUIDED);
      ap = { ...ap, guided: { x, z, alt: Math.max(40, cmd.alt > 1 ? cmd.alt : state.pos[1]) } };
      break;
    }
  }
}

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

  let controls;
  if (ap.mode === MODES.MANUAL && !ap.landing) {
    controls = readControls();
  } else {
    const r = apStep(state, ap);
    ap = r.ap;
    controls = r.controls;
    if (r.disarm) armed = false;
    if (r.reached.length) lastReached = r.reached[r.reached.length - 1];
  }
  if (!armed) controls = { ...controls, throttle: 0 }; // DISARM cuts the engine

  lastControls = controls;
  state = stepAircraft(state, controls, dt);
  simTime += dt;
}

function reset() {
  state = initialState();
  throttle = 0.65;
  simTime = 0;
  armed = true;
  ap = {
    mode: MODES.MANUAL, landing: false, targetAlt: 120, targetHeading: 0,
    guided: null, mission: null,
  };
  lastControls = { pitch: 0, roll: 0, yaw: 0, throttle };
  lastReached = -1;
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
window.__state = () => JSON.stringify({ simTime, throttle, armed, ap, ...state });
window.__command = (cmd) => applyCommand(cmd); // same path the GCS uses, for tests/HILS

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
  const wp = ap.mission ? ` · WP ${Math.min(ap.mission.idx + 1, ap.mission.targets.length)}/${ap.mission.targets.length}` : '';
  const modeLabel = (MODE_NAMES[ap.mode] ?? ap.mode) + (ap.landing ? '·LAND' : '') + wp;
  hud.textContent =
    `SPD ${spd.toFixed(1).padStart(5)} m/s` +
    `\nALT ${state.pos[1].toFixed(1).padStart(5)} m` +
    `\nTHR ${(lastControls.throttle * 100).toFixed(0).padStart(4)} %` +
    `\nT+  ${simTime.toFixed(2).padStart(6)} s${manual ? '  [manual]' : ''}` +
    `\n${armed ? 'ARMED' : 'DISARMED'} · ${modeLabel}`;

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

// Feed the GCS bridge if (and only if) this page is served by it; commands ride
// back on the same probe result.
startTelemetry(() =>
  telemetryFrom(state, lastControls.throttle, simTime, {
    armed, customMode: ap.mode,
    missionSeq: ap.mission ? Math.min(ap.mission.idx, ap.mission.targets.length - 1) : -1,
    missionReached: lastReached,
  })
).then((on) => {
  if (on) {
    startMissionLink(applyCommand);
    console.log('telemetry → bridge: on, command link: on');
  }
});

window.__ready = true;
