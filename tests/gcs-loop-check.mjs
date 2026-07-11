// GCS integration gate (node-only, per PRD §6): spawn the real bridge, stand up a
// fake-GCS UDP socket, POST one telemetry frame, and assert the MAVLink packets
// that arrive are well-formed, CRC-valid, and carry the values we sent.
import { spawn } from 'node:child_process';
import dgram from 'node:dgram';
import { fileURLToPath } from 'node:url';
import { decode } from '../bridge/mavlink.mjs';

const BRIDGE = fileURLToPath(new URL('../bridge/server.mjs', import.meta.url));

// Fake GCS: a UDP socket on a random port; the bridge is pointed at it.
const gcs = dgram.createSocket('udp4');
await new Promise((r) => gcs.bind(0, '127.0.0.1', r));
const received = new Map(); // name → latest decoded message
gcs.on('message', (buf) => {
  const msg = decode(buf);
  if (msg) received.set(msg.name, msg);
});

const bridge = spawn(process.execPath, [BRIDGE], {
  env: { ...process.env, BRIDGE_HTTP_PORT: '0', GCS_PORT: String(gcs.address().port) },
  stdio: ['ignore', 'pipe', 'inherit'],
});
const httpPort = await new Promise((resolve, reject) => {
  const t = setTimeout(() => reject(new Error('bridge start timeout')), 8000);
  bridge.stdout.on('data', (d) => {
    const m = String(d).match(/bridge up http=(\d+)/);
    if (m) { clearTimeout(t); resolve(Number(m[1])); }
  });
  bridge.on('exit', () => reject(new Error('bridge exited early')));
});

const failures = [];
const check = (cond, what) => {
  console.log(`${cond ? '✓' : '✗'} ${what}`);
  if (!cond) failures.push(what);
};

async function waitFor(name, timeoutMs = 4000) {
  const deadline = Date.now() + timeoutMs;
  while (!received.has(name)) {
    if (Date.now() > deadline) return null;
    await new Promise((r) => setTimeout(r, 50));
  }
  return received.get(name);
}

try {
  // 1) Unprompted HEARTBEAT at 1 Hz.
  const hb = await waitFor('HEARTBEAT', 3000);
  check(!!hb, 'HEARTBEAT arrives within 3 s');
  check(hb?.crcOk === true, 'HEARTBEAT CRC valid');
  check(hb?.fields.type === 1 && hb?.fields.autopilot === 3, 'HEARTBEAT identifies a fixed-wing ArduPilot');

  // 2) One telemetry POST fans out to the four telemetry messages.
  const frame = {
    timeBootMs: 42000, lat: 37.4459, lon: 126.4696, alt: 127, relAlt: 120,
    roll: 0.05, pitch: -0.02, yaw: 1.5708, rollspeed: 0.01, pitchspeed: 0, yawspeed: 0.02,
    vn: 40, ve: 1.5, vd: -1.1, headingDeg: 90, airspeed: 40.5, groundspeed: 40,
    climb: 1.1, throttlePct: 65,
  };
  const res = await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(frame),
  });
  check(res.status === 204, 'POST /telemetry accepted (204)');

  const att = await waitFor('ATTITUDE');
  const gpi = await waitFor('GLOBAL_POSITION_INT');
  const hud = await waitFor('VFR_HUD');
  const gps = await waitFor('GPS_RAW_INT');
  check(!!att && att.crcOk, 'ATTITUDE arrives, CRC valid');
  check(!!gpi && gpi.crcOk, 'GLOBAL_POSITION_INT arrives, CRC valid');
  check(!!hud && hud.crcOk, 'VFR_HUD arrives, CRC valid');
  check(!!gps && gps.crcOk, 'GPS_RAW_INT arrives, CRC valid');

  check(Math.abs(att?.fields.yaw - 1.5708) < 1e-4, 'ATTITUDE.yaw round-trips');
  check(gpi?.fields.lat === Math.round(37.4459 * 1e7), 'GLOBAL_POSITION_INT.lat scaled to 1e7');
  check(gpi?.fields.relative_alt === 120000, 'GLOBAL_POSITION_INT.relative_alt in mm');
  check(gpi?.fields.vx === 4000, 'GLOBAL_POSITION_INT.vx (north) in cm/s');
  check(hud?.fields.throttle === 65, 'VFR_HUD.throttle in %');
  check(Math.abs(hud?.fields.airspeed - 40.5) < 1e-4, 'VFR_HUD.airspeed round-trips');
  check(gps?.fields.fix_type === 3, 'GPS_RAW_INT reports 3D fix');
  check(gps?.fields.time_usec === 42000000n, 'GPS_RAW_INT.time_usec in µs');

  // 3) The bridge serves the sim page itself (single command runs everything).
  const page = await fetch(`http://127.0.0.1:${httpPort}/`, { method: 'HEAD' });
  check(page.headers.get('x-flight-bridge') === '1', 'bridge serves the sim (x-flight-bridge header)');
} catch (err) {
  console.error(`gcs-loop-check error: ${err.message}`);
  failures.push(err.message);
} finally {
  bridge.kill();
  gcs.close();
}

console.log(failures.length ? `GCS LOOP CHECK: FAIL (${failures.length})` : 'GCS LOOP CHECK: PASS');
process.exit(failures.length ? 1 : 0);
