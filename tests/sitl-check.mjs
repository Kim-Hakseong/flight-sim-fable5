// M21 gate: drive the SITL JSON adapter exactly like ArduPilot does — binary
// servo packets in, JSON state lines out, lockstep with dedupe and restart.
import { spawn } from 'node:child_process';
import dgram from 'node:dgram';
import { fileURLToPath } from 'node:url';

const SITL = fileURLToPath(new URL('../bridge/sitl.mjs', import.meta.url));
const FRAME_RATE = 600;

const sock = dgram.createSocket('udp4');
await new Promise((r) => sock.bind(0, '127.0.0.1', r));
const proc = spawn(process.execPath, [SITL], {
  env: { ...process.env, SITL_PORT: '0', WND_TRB: '0' },
  stdio: ['ignore', 'pipe', 'inherit'],
});
const port = await new Promise((resolve, reject) => {
  const t = setTimeout(() => reject(new Error('sitl start timeout')), 8000);
  proc.stdout.on('data', (d) => {
    const m = String(d).match(/sitl-json up udp=(\d+)/);
    if (m) { clearTimeout(t); resolve(Number(m[1])); }
  });
});

let waiter = null;
sock.on('message', (buf) => waiter?.(JSON.parse(buf)));

function servoPacket(frameCount, pwm) {
  const buf = Buffer.alloc(40);
  buf.writeUInt16LE(18458, 0);
  buf.writeUInt16LE(FRAME_RATE, 2);
  buf.writeUInt32LE(frameCount, 4);
  const ch = [1500, 1500, 1000, 1500, ...Array(12).fill(1500)];
  Object.assign(ch, pwm);
  for (let i = 0; i < 16; i++) buf.writeUInt16LE(ch[i], 8 + i * 2);
  return buf;
}

const step = (frameCount, pwm) =>
  new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error(`no reply for frame ${frameCount}`)), 3000);
    waiter = (msg) => { clearTimeout(t); resolve(msg); };
    sock.send(servoPacket(frameCount, pwm), port, '127.0.0.1');
  });

const failures = [];
const check = (cond, what) => {
  console.log(`${cond ? '✓' : '✗'} ${what}`);
  if (!cond) failures.push(what);
};

try {
  // 1) idle frame: parked, level, accelerometer reads −g on FRD z.
  let s = await step(1, {});
  check(Math.abs(s.timestamp - 1 / FRAME_RATE) < 1e-9, 'lockstep timestamp = 1/frame_rate');
  check(Math.abs(s.imu.accel_body[2] + 9.81) < 0.3, `parked accel z ≈ −g (${s.imu.accel_body[2].toFixed(2)})`);
  check(Math.abs(s.position[2]) < 0.01 && Math.abs(s.attitude[0]) < 0.02, 'parked at origin, level');

  // 2) dedupe: same frame_count → same reply, no time advance.
  const dup = await step(1, {});
  check(dup.timestamp === s.timestamp, 'duplicate frame_count does not advance the sim');

  // 3) ground roll: full throttle accelerates; airspeed climbs.
  let frame = 1;
  for (let i = 0; i < 5 * FRAME_RATE; i++) s = await step(++frame, { 2: 2000 });
  check(s.airspeed > 15, `full throttle ground roll: Va ${s.airspeed.toFixed(1)} m/s after 5 s`);

  // 4) rotate: AP elevator+ (pwm high on ch2) must pitch the nose UP; then ease off.
  for (let i = 0; i < FRAME_RATE; i++) s = await step(++frame, { 1: 1650, 2: 2000 });
  check(s.attitude[1] > 0.03, `AP elevator+ pitches up (pitch ${(s.attitude[1] * 57.3).toFixed(1)}°)`);
  for (let i = 0; i < 2 * FRAME_RATE; i++) s = await step(++frame, { 1: 1620, 2: 1900 }); // climb out near trim elevator
  check(-s.position[2] > 1, `airborne (alt ${(-s.position[2]).toFixed(1)} m)`);

  // 5) aileron sign: AP aileron+ must produce a positive (right) roll RATE.
  for (let i = 0; i < FRAME_RATE / 4; i++) s = await step(++frame, { 0: 1700, 1: 1620, 2: 1900 });
  check(s.imu.gyro[0] > 0.05, `AP aileron+ rolls right (p = ${s.imu.gyro[0].toFixed(2)} rad/s)`);

  // 6) restart: frame_count reset → fresh sim.
  s = await step(1, {});
  check(s.timestamp < 0.01 && Math.abs(s.position[2]) < 0.01, 'frame_count reset → physics reset');
} catch (err) {
  console.error(`sitl check error: ${err.message}`);
  failures.push(err.message);
} finally {
  proc.kill();
  sock.close();
}

console.log(failures.length ? `SITL CHECK: FAIL (${failures.length})` : 'SITL CHECK: PASS');
process.exit(failures.length ? 1 : 0);
