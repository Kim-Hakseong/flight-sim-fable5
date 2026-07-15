// GCS integration gate (node-only, per PRD §6): spawn the real bridge, stand up a
// fake-GCS UDP socket, POST one telemetry frame, and assert the MAVLink packets
// that arrive are well-formed, CRC-valid, and carry the values we sent.
import { spawn } from 'node:child_process';
import dgram from 'node:dgram';
import { fileURLToPath } from 'node:url';
import { decode, encode } from '../bridge/mavlink.mjs';
import { PARAM_DEFS } from '../src/params.js';
import { COMPAT_PARAMS } from '../bridge/compat-params.mjs';
const PARAM_TOTAL = PARAM_DEFS.length + COMPAT_PARAMS.size;

const BRIDGE = fileURLToPath(new URL('../bridge/server.mjs', import.meta.url));

// Fake GCS: a UDP socket on a random port; the bridge is pointed at it.
const gcs = dgram.createSocket('udp4');
await new Promise((r) => gcs.bind(0, '127.0.0.1', r));
const received = new Map(); // name → latest decoded message
let bridgeAddr = null; // learned from the bridge's own packets, like a real GCS
gcs.on('message', (buf, rinfo) => {
  bridgeAddr = rinfo;
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

  // 3) M2 command loop: GCS command → COMMAND_ACK back + SSE event to the sim.
  const sse = await fetch(`http://127.0.0.1:${httpPort}/commands`);
  check(sse.headers.get('content-type') === 'text/event-stream', 'GET /commands is an SSE stream');
  const reader = sse.body.getReader();
  const sseEvents = [];
  (async () => {
    const dec = new TextDecoder();
    let buf = '';
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += dec.decode(value, { stream: true });
      let i;
      while ((i = buf.indexOf('\n\n')) >= 0) {
        const chunk = buf.slice(0, i);
        buf = buf.slice(i + 2);
        const data = chunk.split('\n').find((l) => l.startsWith('data: '));
        if (data) sseEvents.push(JSON.parse(data.slice(6)));
      }
    }
  })().catch(() => {});
  const sseWait = async (pred, timeoutMs = 3000) => {
    const deadline = Date.now() + timeoutMs;
    for (;;) {
      const hit = sseEvents.find(pred);
      if (hit) return hit;
      if (Date.now() > deadline) return null;
      await new Promise((r) => setTimeout(r, 50));
    }
  };
  const sendToBridge = (name, fields) =>
    new Promise((r) => gcs.send(encode(name, fields), bridgeAddr.port, bridgeAddr.address, r));

  received.delete('COMMAND_ACK');
  await sendToBridge('COMMAND_LONG', {
    param1: 0, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0,
    command: 400, target_system: 1, target_component: 1, confirmation: 0,
  });
  const ack = await waitFor('COMMAND_ACK');
  check(ack?.crcOk && ack.fields.command === 400 && ack.fields.result === 0, 'DISARM → COMMAND_ACK(400, ACCEPTED)');
  const armEvt = await sseWait((e) => e.type === 'arm');
  check(armEvt?.value === 0, 'DISARM relayed to the sim over SSE');

  await sendToBridge('SET_MODE', { custom_mode: 15, target_system: 1, base_mode: 1 });
  const modeEvt = await sseWait((e) => e.type === 'mode');
  check(modeEvt?.custom === 15, 'SET_MODE(GUIDED) relayed to the sim over SSE');

  received.delete('COMMAND_ACK');
  await sendToBridge('COMMAND_LONG', { // newer QGC mode-change path (RTL button)
    param1: 81, param2: 11, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0,
    command: 176, target_system: 1, target_component: 1, confirmation: 0,
  });
  const dsmAck = await waitFor('COMMAND_ACK');
  check(dsmAck?.fields.command === 176 && dsmAck.fields.result === 0, 'DO_SET_MODE → ACK(ACCEPTED)');
  const dsmEvt = await sseWait((e) => e.type === 'mode' && e.custom === 11);
  check(dsmEvt?.custom === 11, 'DO_SET_MODE(RTL) relayed to the sim over SSE');
  // the bridge must reflect the new mode in a HEARTBEAT immediately (optimistic),
  // not wait for the sim→telemetry→heartbeat round-trip (QGC mode-change timeout).
  received.delete('HEARTBEAT');
  await sendToBridge('COMMAND_LONG', {
    param1: 81, param2: 10, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0,
    command: 176, target_system: 1, target_component: 1, confirmation: 0,
  });
  const hbMode = await waitFor('HEARTBEAT', 500); // must arrive well under the 1 Hz interval
  check(hbMode?.fields.custom_mode === 10, 'HEARTBEAT reflects DO_SET_MODE(AUTO) immediately');

  received.delete('COMMAND_ACK');
  await sendToBridge('COMMAND_LONG', {
    param1: 0, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0,
    command: 4242, target_system: 1, target_component: 1, confirmation: 0,
  });
  const nak = await waitFor('COMMAND_ACK');
  check(nak?.fields.result === 3, 'unknown command → COMMAND_ACK(UNSUPPORTED)');

  // 3b) M3 mission upload handshake: COUNT → REQUEST_INT×n → ITEM_INT×n → ACK.
  const wpItem = (seq, lat, lon, alt, command = 16) => ({
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: Math.round(lat * 1e7), y: Math.round(lon * 1e7), z: alt,
    seq, command, target_system: 1, target_component: 1, frame: 3, current: 0, autocontinue: 1,
  });
  const plan = [wpItem(0, 37.4569, 126.4796, 120), wpItem(1, 37.4569, 126.4936, 150)];

  received.delete('MISSION_REQUEST_INT');
  received.delete('MISSION_ACK');
  await sendToBridge('MISSION_COUNT', { count: 2, target_system: 1, target_component: 1 });
  let req = await waitFor('MISSION_REQUEST_INT');
  check(req?.fields.seq === 0, 'MISSION_COUNT → MISSION_REQUEST_INT(0)');
  received.delete('MISSION_REQUEST_INT');
  await sendToBridge('MISSION_ITEM_INT', plan[0]);
  req = await waitFor('MISSION_REQUEST_INT');
  check(req?.fields.seq === 1, 'item 0 accepted → MISSION_REQUEST_INT(1)');
  await sendToBridge('MISSION_ITEM_INT', plan[1]);
  const mAck = await waitFor('MISSION_ACK');
  check(mAck?.fields.type === 0, 'upload complete → MISSION_ACK(ACCEPTED)');
  const missionEvt = await sseWait((e) => e.type === 'mission');
  check(missionEvt?.items.length === 2 && Math.abs(missionEvt.items[1].lat - 37.4569) < 1e-6,
    'mission relayed to the sim over SSE (2 items, lat intact)');

  // Download it back, like QGC verifying an upload.
  received.delete('MISSION_COUNT');
  await sendToBridge('MISSION_REQUEST_LIST', { target_system: 1, target_component: 1 });
  const cnt = await waitFor('MISSION_COUNT');
  check(cnt?.fields.count === 2, 'MISSION_REQUEST_LIST → MISSION_COUNT(2)');
  received.delete('MISSION_ITEM_INT');
  await sendToBridge('MISSION_REQUEST_INT', { seq: 1, target_system: 1, target_component: 1 });
  const item = await waitFor('MISSION_ITEM_INT');
  check(item?.fields.seq === 1 && item.fields.x === plan[1].x, 'MISSION_REQUEST_INT(1) → stored item back');

  // 3c) GUIDED go-to via COMMAND_INT DO_REPOSITION.
  received.delete('COMMAND_ACK');
  await sendToBridge('COMMAND_INT', {
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: Math.round(37.46 * 1e7), y: Math.round(126.47 * 1e7), z: 140,
    command: 192, target_system: 1, target_component: 1, frame: 3, current: 0, autocontinue: 0,
  });
  const rAck = await waitFor('COMMAND_ACK');
  check(rAck?.fields.command === 192 && rAck.fields.result === 0, 'DO_REPOSITION → ACK(ACCEPTED)');
  const gotoEvt = await sseWait((e) => e.type === 'goto');
  check(gotoEvt && Math.abs(gotoEvt.lat - 37.46) < 1e-6 && gotoEvt.alt === 140, 'go-to relayed to the sim over SSE');

  // QGC ArduPilot "Go to location" sends a legacy MISSION_ITEM (id 39) with FLOAT
  // degree coords + current=2, and waits for a MISSION_ACK.
  received.delete('MISSION_ACK');
  await sendToBridge('MISSION_ITEM', {
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: 37.455, y: 126.472, z: 130,
    seq: 0, command: 16, target_system: 1, target_component: 1, frame: 6, current: 2, autocontinue: 1,
  });
  const gAck = await waitFor('MISSION_ACK');
  check(gAck?.fields.type === 0, 'guided MISSION_ITEM(current=2, float) → MISSION_ACK(ACCEPTED)');
  const gGoto = await sseWait((e) => e.type === 'goto' && Math.abs(e.lat - 37.455) < 1e-4);
  check(!!gGoto, 'guided MISSION_ITEM relayed as a go-to (float degrees)');

  // 3d) Mission progress: telemetry drives MISSION_CURRENT + MISSION_ITEM_REACHED.
  received.delete('MISSION_CURRENT');
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ...frame, missionSeq: 1, missionReached: 0 }),
  });
  const cur = await waitFor('MISSION_CURRENT');
  const reachedMsg = await waitFor('MISSION_ITEM_REACHED');
  check(cur?.fields.seq === 1, 'MISSION_CURRENT follows the sim');
  check(reachedMsg?.fields.seq === 0, 'MISSION_ITEM_REACHED fires on the reach edge');

  // 3e) M4 parameters: full list, targeted read, and a live (clamped) set.
  const paramValues = []; // params need every PARAM_VALUE, not just the latest
  const collectParams = (buf) => {
    const m = decode(buf);
    if (m?.name === 'PARAM_VALUE') paramValues.push(m.fields);
  };
  gcs.on('message', collectParams);
  await sendToBridge('PARAM_REQUEST_LIST', { target_system: 1, target_component: 1 });
  await new Promise((r) => setTimeout(r, 500));
  check(paramValues.length === PARAM_TOTAL, `PARAM_REQUEST_LIST → all ${PARAM_TOTAL} PARAM_VALUEs incl. QGC compat stubs (got ${paramValues.length})`);
  check(paramValues.every((p) => p.param_count === PARAM_TOTAL), 'PARAM_VALUE.param_count consistent');
  check(paramValues.some((p) => p.param_id === 'RCMAP_ROLL' && p.param_value === 1), 'QGC compat stub (RCMAP_ROLL) served');

  paramValues.length = 0;
  await sendToBridge('PARAM_REQUEST_READ', { param_index: -1, target_system: 1, target_component: 1, param_id: 'AP_VA_TRIM' });
  await new Promise((r) => setTimeout(r, 300));
  check(paramValues.some((p) => p.param_id === 'AP_VA_TRIM'), 'PARAM_REQUEST_READ by id answered');

  paramValues.length = 0;
  await sendToBridge('PARAM_SET', { param_value: 99, target_system: 1, target_component: 1, param_id: 'AP_VA_TRIM', param_type: 9 });
  await new Promise((r) => setTimeout(r, 300));
  const echoed = paramValues.find((p) => p.param_id === 'AP_VA_TRIM');
  check(Math.abs(echoed?.param_value - 40) < 1e-6, `PARAM_SET out-of-range clamps to max (echoed ${echoed?.param_value})`);
  const paramEvt = await sseWait((e) => e.type === 'param');
  check(paramEvt?.id === 'AP_VA_TRIM' && Math.abs(paramEvt.value - 40) < 1e-6, 'clamped param relayed to the sim over SSE');
  gcs.off('message', collectParams);

  // 3f) M5 faults: health bits + STATUSTEXT on the inject and clear edges.
  const texts = [];
  const collectTexts = (buf) => {
    const m = decode(buf);
    if (m?.name === 'STATUSTEXT') texts.push(m.fields);
  };
  gcs.on('message', collectTexts);
  received.delete('SYS_STATUS');
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ...frame, health: 63 & ~32, faults: { gps: 'bias' }, gpsFix: 3, gpsSats: 12 }),
  });
  let sys = await waitFor('SYS_STATUS');
  check((sys?.fields.onboard_control_sensors_health & 32) === 0, 'SYS_STATUS: GPS health bit drops on fault');
  await new Promise((r) => setTimeout(r, 200));
  check(texts.some((x) => x.severity === 4 && x.text.includes('GPS fault: bias')), 'STATUSTEXT warning on fault inject');

  received.delete('SYS_STATUS');
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ...frame, health: 63, faults: {} }),
  });
  sys = await waitFor('SYS_STATUS');
  check((sys?.fields.onboard_control_sensors_health & 32) === 32, 'SYS_STATUS: GPS healthy again on clear');
  await new Promise((r) => setTimeout(r, 200));
  check(texts.some((x) => x.severity === 6 && x.text.includes('GPS fault cleared')), 'STATUSTEXT info on fault clear');
  gcs.off('message', collectTexts);

  // 3g) M6 completeness: battery in SYS_STATUS, EKF report, lifecycle STATUSTEXT.
  const texts6 = [];
  const collect6 = (buf) => {
    const m = decode(buf);
    if (m?.name === 'STATUSTEXT') texts6.push(m.fields);
  };
  gcs.on('message', collect6);
  received.delete('SYS_STATUS');
  received.delete('EKF_STATUS_REPORT');
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, { // prime: armed, MANUAL
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ...frame, armed: true, customMode: 0 }),
  });
  await new Promise((r) => setTimeout(r, 150));
  received.delete('SYS_STATUS'); // don't read the priming frame's SYS_STATUS
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({
      ...frame, battMv: 11800, battCa: 1030, battPct: 72,
      ekf: { velocity_variance: 0.04, pos_horiz_variance: 0.08, pos_vert_variance: 0.05, compass_variance: 0.05, terrain_alt_variance: 0, flags: 831 },
      armed: false, customMode: 11, missionReached: 1,
    }),
  });
  const sys6 = await waitFor('SYS_STATUS');
  check(sys6?.fields.voltage_battery === 11800 && sys6.fields.battery_remaining === 72, 'SYS_STATUS carries the sim battery');
  const ekf6 = await waitFor('EKF_STATUS_REPORT');
  check(ekf6?.crcOk && ekf6.fields.flags === 831, 'EKF_STATUS_REPORT relayed with flags');
  await new Promise((r) => setTimeout(r, 200));
  check(texts6.some((x) => x.text === 'Disarming motors'), 'STATUSTEXT on arm edge');
  check(texts6.some((x) => x.text === 'Mode changed to RTL'), 'STATUSTEXT on mode edge');
  check(texts6.some((x) => x.text === 'Reached waypoint #1'), 'STATUSTEXT on waypoint reach');
  gcs.off('message', collect6);

  // 3h) M17–M19: WIND downlink, MANUAL_CONTROL → stick over SSE, v2 auto-upgrade.
  received.delete('WIND');
  await fetch(`http://127.0.0.1:${httpPort}/telemetry`, {
    method: 'POST', headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ...frame, windN: -6, windE: 0 }),
  });
  const windMsg = await waitFor('WIND');
  check(windMsg?.crcOk && Math.abs(windMsg.fields.direction - 0) < 1 && Math.abs(windMsg.fields.speed - 6) < 0.1,
    'WIND downlink (from-north 6 m/s)');

  await sendToBridge('MANUAL_CONTROL', { x: -500, y: 300, z: 800, r: 0, buttons: 0, target: 1 });
  const stickEvt = await sseWait((e) => e.type === 'stick');
  check(stickEvt && Math.abs(stickEvt.pitch - 0.5) < 1e-6 && Math.abs(stickEvt.throttle - 0.8) < 1e-6,
    'MANUAL_CONTROL relayed as a stick command');

  received.delete('HEARTBEAT');
  gcs.send(encode('HEARTBEAT', { custom_mode: 0, type: 6, autopilot: 8, base_mode: 0, system_status: 4, mavlink_version: 3 }, { v2: true }),
    bridgeAddr.port, bridgeAddr.address);
  const hb2 = await waitFor('HEARTBEAT');
  check(hb2?.v2 === true && hb2.crcOk, 'bridge auto-upgrades to v2 framing after hearing v2');

  // Reconnect: the bridge must replay the last command (monotonic seq dedupe).
  const sse2 = await fetch(`http://127.0.0.1:${httpPort}/commands`);
  const r2 = sse2.body.getReader();
  let replay = '';
  const rDeadline = Date.now() + 2000;
  while (!replay.includes('"type"') && Date.now() < rDeadline) {
    const { value, done } = await r2.read();
    if (done) break;
    replay += new TextDecoder().decode(value);
  }
  check(replay.includes('"type":"stick"'), 'last command replayed on SSE (re)connect');
  r2.cancel().catch(() => {});
  reader.cancel().catch(() => {});

  // 4) The bridge serves the sim page itself (single command runs everything).
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
