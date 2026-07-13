// M20 gate: spawn the real plant process; THIS process is the external FC.
// The loop closes over UDP JSON lockstep: sensors in → control law → surfaces out.
// The controller reuses holdControls as its law, but builds its own nav state
// purely from the plant's SENSOR replies — no shared memory, no truth access.
import { spawn } from 'node:child_process';
import dgram from 'node:dgram';
import { fileURLToPath } from 'node:url';
import { holdControls } from '../src/autopilot.js';
import {
  createEstimator, stepEstimator, createAttEstimator, stepAttEstimator,
} from '../src/estimator.js';
import { initialState } from '../src/physics.js';

const PLANT = fileURLToPath(new URL('../bridge/plant.mjs', import.meta.url));
const DT = 1 / 60;

const sock = dgram.createSocket('udp4');
await new Promise((r) => sock.bind(0, '127.0.0.1', r));
const plant = spawn(process.execPath, [PLANT], {
  env: { ...process.env, PLANT_PORT: '0', GCS_PORT: '9' }, // GCS → discard port
  stdio: ['ignore', 'pipe', 'inherit'],
});
const plantPort = await new Promise((resolve, reject) => {
  const t = setTimeout(() => reject(new Error('plant start timeout')), 8000);
  plant.stdout.on('data', (d) => {
    const m = String(d).match(/plant up udp=(\d+)/);
    if (m) { clearTimeout(t); resolve(Number(m[1])); }
  });
});

const pending = new Map();
sock.on('message', (buf) => {
  const msg = JSON.parse(buf);
  const key = msg.type === 'ready' ? 'ready' : msg.seq;
  pending.get(key)?.(msg);
  pending.delete(key);
});
const send = (obj, key) =>
  new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error(`plant reply timeout (${key})`)), 4000);
    pending.set(key, (m) => { clearTimeout(t); resolve(m); });
    sock.send(JSON.stringify(obj), plantPort, '127.0.0.1');
  });

const failures = [];
const check = (cond, what) => {
  console.log(`${cond ? '✓' : '✗'} ${what}`);
  if (!cond) failures.push(what);
};

// External FC: its own estimators over the plant's sensor stream + holdControls.
async function flySortie(seconds) {
  await send({ type: 'reset', boot: 'air', seed: 21 }, 'ready');
  const s0 = initialState();
  let est = createEstimator(s0);
  let att = createAttEstimator(s0);
  let readings = null;
  let minAlt = Infinity;
  let lastAlt = 0;
  for (let i = 0; i < seconds * 60; i++) {
    const rateEst = readings?.gyro ? readings.gyro.map((x, j) => x - att.bias[j]) : [0, 0, 0];
    const nav = { pos: est.pos, vel: est.vel, quat: att.quat, omega: rateEst };
    const controls = holdControls(nav, 140, 90, null, undefined, readings?.pitot?.[0] ?? null);
    const out = await send({ type: 'ctl', seq: i, controls }, i);
    readings = out.readings;
    att = stepAttEstimator(att, readings, DT);
    est = stepEstimator(est, readings, DT);
    lastAlt = est.pos[1];
    if (i > 5 * 60) minAlt = Math.min(minAlt, est.pos[1]);
  }
  return { minAlt, lastAlt, est, att };
}

try {
  const t0 = Date.now();
  const run1 = await flySortie(30);
  check(run1.minAlt > 100, `external FC held altitude (min ${run1.minAlt.toFixed(1)} m)`);
  check(Math.abs(run1.lastAlt - 140) < 12, `climbed to the commanded 140 m (got ${run1.lastAlt.toFixed(1)})`);
  console.log(`  (30 s lockstep sortie in ${Date.now() - t0} ms)`);

  const run2 = await flySortie(30);
  check(
    JSON.stringify(run1.est) === JSON.stringify(run2.est),
    'lockstep is deterministic (identical estimator state across reruns)'
  );

  // Fault injection reaches the external FC through its sensor stream only.
  await send({ type: 'reset', boot: 'air', seed: 21 }, 'ready');
  sock.send(JSON.stringify({ type: 'fault', sensor: 'gps', faultType: 'dropout' }), plantPort, '127.0.0.1');
  const out = await send({ type: 'ctl', seq: 0, controls: { throttle: 0.6 } }, 0);
  check(out.readings.gps === null, 'injected GPS dropout visible in the plant sensor stream');
} catch (err) {
  console.error(`plant check error: ${err.message}`);
  failures.push(err.message);
} finally {
  plant.kill();
  sock.close();
}

console.log(failures.length ? `PLANT CHECK: FAIL (${failures.length})` : 'PLANT CHECK: PASS');
process.exit(failures.length ? 1 : 0);
