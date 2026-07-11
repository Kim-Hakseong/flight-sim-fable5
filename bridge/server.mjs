// GCS bridge: serves the static sim over HTTP (:8765), accepts telemetry POSTs
// from the browser, and relays MAVLink v1 over UDP to the GCS (QGC on :14550).
// Config via env: BRIDGE_HTTP_PORT, GCS_HOST, GCS_PORT.
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import dgram from 'node:dgram';
import { join, extname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { encode, decode } from './mavlink.mjs';
import { PARAM_DEFS, PARAM_TYPE_REAL32, defaultParams, clampParam } from '../src/params.js';

const ROOT = fileURLToPath(new URL('..', import.meta.url));
const HTTP_PORT = Number(process.env.BRIDGE_HTTP_PORT ?? 8765);
const GCS_HOST = process.env.GCS_HOST ?? '127.0.0.1';
const GCS_PORT = Number(process.env.GCS_PORT ?? 14550);
const SYSID = 1;

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json', '.md': 'text/markdown',
};

// --- MAVLink out --------------------------------------------------------------
const udp = dgram.createSocket('udp4');
let seq = 0;
// QGC's UDP link replies to whatever source last talked to it; remember the
// peer that actually sent us traffic and prefer it as the destination.
let gcsAddr = null;

function sendMsg(name, fields) {
  const pkt = encode(name, fields, { seq: seq++, sysid: SYSID, compid: 1 });
  const host = gcsAddr?.address ?? GCS_HOST;
  const port = gcsAddr?.port ?? GCS_PORT;
  udp.send(pkt, port, host);
}

// --- GCS → sim commands over SSE ------------------------------------------------
// Lessons baked in: no Nagle on the SSE socket, buffer the last command and
// replay it on (re)connect; the sim dedupes by the monotonic seq.
const sseClients = new Set();
let cmdSeq = 0;
let lastCmd = null;

function pushCommand(cmd) {
  lastCmd = { seq: cmdSeq++, ...cmd };
  const line = `data: ${JSON.stringify(lastCmd)}\n\n`;
  for (const res of sseClients) res.write(line);
  console.log(`bridge → sim: ${JSON.stringify(lastCmd)}`);
}

const MAV_RESULT_ACCEPTED = 0;
const MAV_RESULT_UNSUPPORTED = 3;

function handleCommandLong(f) {
  switch (f.command) {
    case 400: // MAV_CMD_COMPONENT_ARM_DISARM
      pushCommand({ type: 'arm', value: f.param1 >= 0.5 ? 1 : 0 });
      return MAV_RESULT_ACCEPTED;
    case 22: // MAV_CMD_NAV_TAKEOFF (param7 = altitude)
      pushCommand({ type: 'takeoff', alt: f.param7 > 1 ? f.param7 : 50 });
      return MAV_RESULT_ACCEPTED;
    case 21: // MAV_CMD_NAV_LAND
      pushCommand({ type: 'land' });
      return MAV_RESULT_ACCEPTED;
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
      pushCommand({ type: 'rtl' });
      return MAV_RESULT_ACCEPTED;
    case 192: // MAV_CMD_DO_REPOSITION (COMMAND_LONG form: params 5/6/7 = lat/lon/alt)
      pushCommand({ type: 'goto', lat: f.param5, lon: f.param6, alt: f.param7 });
      return MAV_RESULT_ACCEPTED;
    case 300: // MAV_CMD_MISSION_START → fly the uploaded plan
      pushCommand({ type: 'mode', custom: 10 }); // AUTO
      return MAV_RESULT_ACCEPTED;
    default:
      return MAV_RESULT_UNSUPPORTED;
  }
}

function handleCommandInt(f) {
  if (f.command === 192) { // DO_REPOSITION: x/y are lat/lon·1e7, z is alt
    pushCommand({ type: 'goto', lat: f.x / 1e7, lon: f.y / 1e7, alt: f.z });
    return MAV_RESULT_ACCEPTED;
  }
  if (f.command === 300) {
    pushCommand({ type: 'mode', custom: 10 });
    return MAV_RESULT_ACCEPTED;
  }
  return MAV_RESULT_UNSUPPORTED;
}

// --- Parameter protocol (shared table; the sim re-tunes live via SSE) ----------
const params = defaultParams();

function sendParamValue(id, index = PARAM_DEFS.findIndex((p) => p.id === id)) {
  sendMsg('PARAM_VALUE', {
    param_value: params[id], param_count: PARAM_DEFS.length, param_index: index,
    param_id: id, param_type: PARAM_TYPE_REAL32,
  });
}

function handleParamSet(f) {
  const clamped = clampParam(f.param_id, f.param_value);
  if (clamped === null) return; // unknown id: silence (QGC will time out its widget)
  params[f.param_id] = clamped;
  sendParamValue(f.param_id); // echo (possibly clamped) — closes QGC's set cycle
  pushCommand({ type: 'param', id: f.param_id, value: clamped });
}

// --- Mission protocol (upload: COUNT → REQUEST_INT×n → ACK; download: mirror) ---
let missionItems = []; // accepted plan, as raw MISSION_ITEM_INT fields
let upload = null; // { count, items, timer }

function requestNextItem() {
  if (!upload) return;
  sendMsg('MISSION_REQUEST_INT', {
    seq: upload.items.length, target_system: 255, target_component: 0,
  });
}

function startUpload(count) {
  if (upload) clearInterval(upload.timer);
  if (count === 0) {
    missionItems = [];
    sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0 });
    pushCommand({ type: 'mission', items: [] });
    return;
  }
  upload = { count, items: [], tries: 0, timer: null };
  upload.timer = setInterval(() => {
    if (++upload.tries > 8) { // give up: MAV_MISSION_ERROR
      clearInterval(upload.timer);
      upload = null;
      sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 1 });
      return;
    }
    requestNextItem();
  }, 700);
  requestNextItem();
}

function onMissionItem(f) {
  if (!upload || f.seq !== upload.items.length) return; // duplicate/stray
  upload.items.push(f);
  upload.tries = 0;
  if (upload.items.length < upload.count) {
    requestNextItem();
    return;
  }
  clearInterval(upload.timer);
  missionItems = upload.items;
  upload = null;
  sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0 });
  pushCommand({
    type: 'mission',
    items: missionItems.map((it) => ({
      seq: it.seq, command: it.command, frame: it.frame,
      lat: it.x / 1e7, lon: it.y / 1e7, alt: it.z,
      param1: it.param1, param2: it.param2,
    })),
  });
}

udp.on('message', (buf, rinfo) => {
  gcsAddr = rinfo;
  const msg = decode(buf);
  if (!msg || !msg.crcOk) {
    if (msg) console.log(`gcs → bridge: ${msg.name} (BAD CRC, dropped)`);
    return;
  }
  const f = msg.fields;
  switch (msg.name) {
    case 'COMMAND_LONG':
      sendMsg('COMMAND_ACK', { command: f.command, result: handleCommandLong(f) });
      break;
    case 'COMMAND_INT':
      sendMsg('COMMAND_ACK', { command: f.command, result: handleCommandInt(f) });
      break;
    case 'SET_MODE':
      pushCommand({ type: 'mode', custom: f.custom_mode });
      break;
    case 'MISSION_COUNT':
      startUpload(f.count);
      break;
    case 'MISSION_ITEM_INT':
      onMissionItem(f);
      break;
    case 'MISSION_REQUEST_LIST': // GCS downloads our plan back
      sendMsg('MISSION_COUNT', { count: missionItems.length, target_system: 255, target_component: 0 });
      break;
    case 'MISSION_REQUEST_INT':
    case 'MISSION_REQUEST': {
      const it = missionItems[f.seq];
      if (it) sendMsg('MISSION_ITEM_INT', it);
      break;
    }
    case 'PARAM_REQUEST_LIST':
      PARAM_DEFS.forEach((p, i) => sendParamValue(p.id, i));
      break;
    case 'PARAM_REQUEST_READ': {
      const byIndex = f.param_index >= 0 ? PARAM_DEFS[f.param_index]?.id : null;
      const id = byIndex ?? f.param_id;
      if (id in params) sendParamValue(id);
      break;
    }
    case 'PARAM_SET':
      handleParamSet(f);
      break;
    case 'MISSION_ACK': // GCS finished downloading
    case 'HEARTBEAT':
      break;
    default:
      console.log(`gcs → bridge: ${msg.name} (ignored)`);
  }
});

// The sim is authoritative for arm/mode: HEARTBEAT reflects the last telemetry.
let vehicle = { armed: true, customMode: 0 };
let lastReachedSent = -1; // MISSION_ITEM_REACHED fires on the rising edge only

setInterval(() => {
  sendMsg('HEARTBEAT', {
    custom_mode: vehicle.customMode, // ArduPlane numbering (sim-side MODES map)
    type: 1, // MAV_TYPE_FIXED_WING
    autopilot: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA — QGC then knows the mode map
    base_mode: 81 | (vehicle.armed ? 128 : 0), // CUSTOM_MODE|STABILIZE|MANUAL_INPUT (+ARMED)
    system_status: vehicle.armed ? 4 : 3, // ACTIVE : STANDBY
    mavlink_version: 3,
  });
}, 1000);

function relayTelemetry(t) {
  const ms = t.timeBootMs >>> 0;
  vehicle = { armed: !!t.armed, customMode: t.customMode >>> 0 };
  sendMsg('ATTITUDE', {
    time_boot_ms: ms, roll: t.roll, pitch: t.pitch, yaw: t.yaw,
    rollspeed: t.rollspeed, pitchspeed: t.pitchspeed, yawspeed: t.yawspeed,
  });
  sendMsg('GLOBAL_POSITION_INT', {
    time_boot_ms: ms,
    lat: Math.round(t.lat * 1e7), lon: Math.round(t.lon * 1e7),
    alt: Math.round(t.alt * 1000), relative_alt: Math.round(t.relAlt * 1000),
    vx: Math.round(t.vn * 100), vy: Math.round(t.ve * 100), vz: Math.round(t.vd * 100),
    hdg: Math.round(t.headingDeg * 100) % 36000,
  });
  sendMsg('VFR_HUD', {
    airspeed: t.airspeed, groundspeed: t.groundspeed, alt: t.alt, climb: t.climb,
    heading: Math.round(t.headingDeg), throttle: t.throttlePct,
  });
  sendMsg('GPS_RAW_INT', {
    time_usec: BigInt(ms) * 1000n,
    lat: Math.round(t.lat * 1e7), lon: Math.round(t.lon * 1e7),
    alt: Math.round(t.alt * 1000),
    eph: 80, epv: 120, vel: Math.round(t.groundspeed * 100),
    cog: Math.round(t.headingDeg * 100) % 36000,
    fix_type: 3, satellites_visible: 12,
  });
  if (t.missionSeq >= 0) sendMsg('MISSION_CURRENT', { seq: t.missionSeq });
  if (t.missionReached >= 0 && t.missionReached !== lastReachedSent) {
    lastReachedSent = t.missionReached;
    sendMsg('MISSION_ITEM_REACHED', { seq: t.missionReached });
  }
}

// --- HTTP: static sim + telemetry ingest ---------------------------------------
const server = createServer(async (req, res) => {
  res.setHeader('x-flight-bridge', '1');
  if (req.method === 'POST' && req.url === '/telemetry') {
    let body = '';
    req.on('data', (d) => (body += d));
    req.on('end', () => {
      try {
        relayTelemetry(JSON.parse(body));
        res.writeHead(204).end();
      } catch (err) {
        res.writeHead(400).end(String(err));
      }
    });
    return;
  }
  if (req.url === '/commands') {
    req.socket.setNoDelay(true);
    res.writeHead(200, {
      'content-type': 'text/event-stream',
      'cache-control': 'no-cache',
      connection: 'keep-alive',
    });
    res.write(':ok\n\n');
    if (lastCmd) res.write(`data: ${JSON.stringify(lastCmd)}\n\n`);
    sseClients.add(res);
    req.on('close', () => sseClients.delete(res));
    return;
  }
  if (req.url === '/api/ping') {
    res.writeHead(200, { 'content-type': 'application/json' });
    return res.end('{"ok":true}');
  }
  const path = req.url === '/' ? '/index.html' : req.url.split('?')[0];
  try {
    const body = await readFile(join(ROOT, path.slice(1)));
    res.writeHead(200, { 'content-type': MIME[extname(path)] || 'application/octet-stream' });
    res.end(req.method === 'HEAD' ? undefined : body);
  } catch {
    res.writeHead(404).end('not found');
  }
});

server.listen(HTTP_PORT, () => {
  console.log(`bridge up http=${server.address().port} gcs=${GCS_HOST}:${GCS_PORT}`);
});
