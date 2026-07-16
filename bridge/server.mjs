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
import { MODES, MODE_NAMES } from '../src/autopilot.js';
import { COMPAT_PARAMS, FENCE_FORWARDED } from './compat-params.mjs';

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
let gcsV2 = false; // reply in the framing the GCS last spoke

function sendMsg(name, fields) {
  const pkt = encode(name, fields, { seq: seq++, sysid: SYSID, compid: 1, v2: gcsV2 });
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
  if (cmd.type !== 'stick') console.log(`bridge → sim: ${JSON.stringify(lastCmd)}`);
}

const MAV_RESULT_ACCEPTED = 0;
const MAV_RESULT_UNSUPPORTED = 3;

function handleCommandLong(f) {
  switch (f.command) {
    case 400: // MAV_CMD_COMPONENT_ARM_DISARM
      relayArm(f.param1 >= 0.5);
      return MAV_RESULT_ACCEPTED;
    case 22: // MAV_CMD_NAV_TAKEOFF (param7 = altitude)
      // QGC's takeoff = arm, wait for the ARMED heartbeat, then this. Reflect arm
      // + TAKEOFF mode optimistically so QGC doesn't time out on either step.
      pushCommand({ type: 'takeoff', alt: f.param7 > 1 ? f.param7 : 50 });
      vehicle = { ...vehicle, armed: true, customMode: MODES.TAKEOFF };
      sendHeartbeat();
      return MAV_RESULT_ACCEPTED;
    case 21: // MAV_CMD_NAV_LAND
      pushCommand({ type: 'land' });
      return MAV_RESULT_ACCEPTED;
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
      pushCommand({ type: 'rtl' });
      return MAV_RESULT_ACCEPTED;
    case 176: // MAV_CMD_DO_SET_MODE — newer QGC sends mode changes this way
      // (param1 = base_mode flags, param2 = custom_mode when CUSTOM_MODE_ENABLED)
      relayMode(f.param2 >>> 0);
      return MAV_RESULT_ACCEPTED;
    case 193: // MAV_CMD_DO_PAUSE_CONTINUE — QGC pause/continue button
      // pause (param1=0) → LOITER hold-here; continue (param1=1) → resume AUTO
      relayMode(f.param1 < 0.5 ? 12 : 10);
      return MAV_RESULT_ACCEPTED;
    case 192: // MAV_CMD_DO_REPOSITION (COMMAND_LONG form: params 5/6/7 = lat/lon/alt)
      pushCommand({ type: 'goto', lat: f.param5, lon: f.param6, alt: f.param7 });
      return MAV_RESULT_ACCEPTED;
    case 300: // MAV_CMD_MISSION_START → fly the uploaded plan
      relayMode(10); // AUTO
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
    relayMode(10);
    return MAV_RESULT_ACCEPTED;
  }
  return MAV_RESULT_UNSUPPORTED;
}

// --- Parameter protocol (shared table; the sim re-tunes live via SSE) ----------
const params = defaultParams();
const compat = new Map(COMPAT_PARAMS); // QGC-facing stubs; never forwarded to the sim
const PARAM_TOTAL = PARAM_DEFS.length + compat.size;
const compatIndex = (id) => PARAM_DEFS.length + [...compat.keys()].indexOf(id);

function sendParamValue(id, index = PARAM_DEFS.findIndex((p) => p.id === id)) {
  const isCompat = compat.has(id);
  sendMsg('PARAM_VALUE', {
    param_value: isCompat ? compat.get(id) : params[id],
    param_count: PARAM_TOTAL,
    param_index: isCompat ? compatIndex(id) : index,
    param_id: id, param_type: PARAM_TYPE_REAL32,
  });
}

function handleParamSet(f) {
  if (compat.has(f.param_id)) { // QGC setup-page writes: accept + echo, keep local
    compat.set(f.param_id, f.param_value);
    sendParamValue(f.param_id);
    // A few fence params drive real enforcement — forward them to the sim.
    if (FENCE_FORWARDED.has(f.param_id)) {
      const altMax = (compat.get('FENCE_ENABLE') ? compat.get('FENCE_ALT_MAX') : 0) || 0;
      pushCommand({ type: 'fence', altMax });
    }
    return;
  }
  const clamped = clampParam(f.param_id, f.param_value);
  if (clamped === null) return; // unknown id: silence (QGC will time out its widget)
  params[f.param_id] = clamped;
  sendParamValue(f.param_id); // echo (possibly clamped) — closes QGC's set cycle
  pushCommand({ type: 'param', id: f.param_id, value: clamped });
}

// --- Mission + fence protocol (upload: COUNT → REQUEST_INT×n → ACK; download:
// mirror). mission_type 0 = mission (waypoints), 1 = fence (geofence geometry);
// the same handshake carries both, distinguished by the MISSION_COUNT mission_type.
const MT_MISSION = 0;
const MT_FENCE = 1;
let missionItems = []; // accepted plan, as raw MISSION_ITEM_INT fields
let fenceItems = []; // accepted fence, as raw MISSION_ITEM_INT fields
let upload = null; // { count, items, tries, timer, mtype }

function requestNextItem() {
  if (!upload) return;
  sendMsg('MISSION_REQUEST_INT', {
    seq: upload.items.length, target_system: 255, target_component: 0,
    mission_type: upload.mtype,
  });
}

function startUpload(count, mtype = MT_MISSION) {
  if (upload) clearInterval(upload.timer);
  if (count === 0) { // clear the mission or the fence
    if (mtype === MT_FENCE) { fenceItems = []; pushCommand({ type: 'fence', items: [] }); }
    else { missionItems = []; pushCommand({ type: 'mission', items: [] }); }
    sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0, mission_type: mtype });
    return;
  }
  upload = { count, items: [], tries: 0, timer: null, mtype };
  upload.timer = setInterval(() => {
    if (++upload.tries > 8) { // give up: MAV_MISSION_ERROR
      clearInterval(upload.timer);
      const mt = upload.mtype; upload = null;
      sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 1, mission_type: mt });
      return;
    }
    requestNextItem();
  }, 700);
  requestNextItem();
}

function finishUpload() {
  clearInterval(upload.timer);
  const { items, mtype } = upload;
  upload = null;
  sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0, mission_type: mtype });
  if (mtype === MT_FENCE) {
    fenceItems = items;
    pushCommand({
      type: 'fence',
      items: items.map((it) => ({
        command: it.command, lat: it.x / 1e7, lon: it.y / 1e7, param1: it.param1,
      })),
    });
    return;
  }
  missionItems = items;
  pushCommand({
    type: 'mission',
    items: items.map((it) => ({
      seq: it.seq, command: it.command, frame: it.frame,
      lat: it.x / 1e7, lon: it.y / 1e7, alt: it.z,
      param1: it.param1, param2: it.param2,
    })),
  });
}

function onMissionItem(f) {
  // A guided-mode go-to: QGC's "Go to location" can deliver the target as a
  // standalone MISSION_ITEM_INT with current == 2 (not through an upload
  // handshake or DO_REPOSITION). It expects a MISSION_ACK back — without it QGC
  // reports "vehicle does not respond to the Guided Mode Item".
  if (!upload && f.current === 2) {
    pushCommand({ type: 'goto', lat: f.x / 1e7, lon: f.y / 1e7, alt: f.z });
    sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0 });
    return;
  }
  if (!upload || f.seq !== upload.items.length) return; // duplicate/stray
  upload.items.push(f);
  upload.tries = 0;
  if (upload.items.length < upload.count) { requestNextItem(); return; }
  finishUpload();
}

udp.on('message', (buf, rinfo) => {
  gcsAddr = rinfo;
  const msg = decode(buf);
  if (!msg || !msg.crcOk) {
    if (msg) console.log(`gcs → bridge: ${msg.name} (BAD CRC, dropped)`);
    return;
  }
  gcsV2 = msg.v2;
  const f = msg.fields;
  switch (msg.name) {
    case 'MANUAL_CONTROL': // QGC virtual joystick: x fwd(+)=nose down, z 0..1000
      pushCommand({
        type: 'stick',
        pitch: -f.x / 1000, roll: f.y / 1000, yaw: f.r / 1000,
        throttle: Math.max(0, Math.min(1, f.z / 1000)),
      });
      break;
    case 'COMMAND_LONG':
      sendMsg('COMMAND_ACK', { command: f.command, result: handleCommandLong(f) });
      break;
    case 'COMMAND_INT':
      sendMsg('COMMAND_ACK', { command: f.command, result: handleCommandInt(f) });
      break;
    case 'SET_MODE':
      relayMode(f.custom_mode);
      break;
    case 'MISSION_COUNT':
      startUpload(f.count, f.mission_type >>> 0);
      break;
    case 'MISSION_ITEM_INT':
      onMissionItem(f);
      break;
    case 'MISSION_ITEM': // legacy float-coord item — QGC's ArduPilot guided go-to
      if (f.current === 2 || f.current === 3) {
        pushCommand({ type: 'goto', lat: f.x, lon: f.y, alt: f.z }); // x/y already degrees
        sendMsg('MISSION_ACK', { target_system: 255, target_component: 0, type: 0 });
      }
      break;
    case 'MISSION_REQUEST_LIST': { // GCS downloads our plan/fence back
      const mt = f.mission_type >>> 0;
      const list = mt === MT_FENCE ? fenceItems : missionItems;
      sendMsg('MISSION_COUNT', { count: list.length, target_system: 255, target_component: 0, mission_type: mt });
      break;
    }
    case 'MISSION_REQUEST_INT':
    case 'MISSION_REQUEST': {
      const mt = f.mission_type >>> 0;
      const it = (mt === MT_FENCE ? fenceItems : missionItems)[f.seq];
      if (it) sendMsg('MISSION_ITEM_INT', { ...it, mission_type: mt });
      break;
    }
    case 'PARAM_REQUEST_LIST':
      PARAM_DEFS.forEach((p, i) => sendParamValue(p.id, i));
      for (const id of compat.keys()) sendParamValue(id);
      break;
    case 'PARAM_REQUEST_READ': {
      let id = f.param_id;
      if (f.param_index >= 0) {
        id = f.param_index < PARAM_DEFS.length
          ? PARAM_DEFS[f.param_index]?.id
          : [...compat.keys()][f.param_index - PARAM_DEFS.length];
      }
      if (id && (id in params || compat.has(id))) sendParamValue(id);
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
let lastFaults = {}; // sensor → fault type; STATUSTEXT fires on every edge

// Lifecycle STATUSTEXT: announce arm/mode/waypoint transitions like a real AP.
let lifecycle = null; // { armed, customMode, missionReached }

function statusTextOnLifecycleEdges(t) {
  const now = { armed: !!t.armed, customMode: t.customMode >>> 0, missionReached: t.missionReached ?? -1 };
  if (lifecycle) {
    if (now.armed !== lifecycle.armed) {
      sendMsg('STATUSTEXT', { severity: 6, text: now.armed ? 'Arming motors' : 'Disarming motors' });
    }
    if (now.customMode !== lifecycle.customMode) {
      sendMsg('STATUSTEXT', { severity: 6, text: `Mode changed to ${MODE_NAMES[now.customMode] ?? now.customMode}` });
    }
    if (now.missionReached >= 0 && now.missionReached !== lifecycle.missionReached) {
      sendMsg('STATUSTEXT', { severity: 6, text: `Reached waypoint #${now.missionReached}` });
    }
  }
  lifecycle = now;
}

function statusTextOnFaultEdges(faults = {}) {
  for (const [sensor, type] of Object.entries(faults)) {
    if (lastFaults[sensor] !== type) {
      sendMsg('STATUSTEXT', {
        severity: sensor === 'crash' ? 2 : 4, // CRITICAL vs WARNING
        text: sensor === 'crash' ? 'CRASH DETECTED — vehicle disarmed' : `${sensor.toUpperCase()} fault: ${type}`,
      });
    }
  }
  for (const sensor of Object.keys(lastFaults)) {
    if (!(sensor in faults)) {
      sendMsg('STATUSTEXT', { severity: 6, text: `${sensor.toUpperCase()} fault cleared` }); // INFO
    }
  }
  lastFaults = faults;
}

function sendHeartbeat() {
  sendMsg('HEARTBEAT', {
    custom_mode: vehicle.customMode, // ArduPlane numbering (sim-side MODES map)
    type: 1, // MAV_TYPE_FIXED_WING
    autopilot: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA — QGC then knows the mode map
    base_mode: 81 | (vehicle.armed ? 128 : 0), // CUSTOM_MODE|STABILIZE|MANUAL_INPUT (+ARMED)
    system_status: vehicle.armed ? 4 : 3, // ACTIVE : STANDBY
    mavlink_version: 3,
  });
}
setInterval(sendHeartbeat, 1000);

// Relay a mode change to the sim AND reflect it in the HEARTBEAT immediately.
// Without the optimistic update, QGC waits on the command → SSE → sim → telemetry
// (10 Hz) → HEARTBEAT (1 Hz) round-trip (~1 s) and times out its mode-change
// verification ("failed to enter Auto mode"). The sim confirms via telemetry.
function relayMode(custom) {
  pushCommand({ type: 'mode', custom });
  vehicle = { ...vehicle, customMode: custom >>> 0 };
  sendHeartbeat();
}

// Relay ARM/DISARM to the sim AND reflect it in the HEARTBEAT's base_mode ARMED
// bit immediately. QGC's guided takeoff arms first, then waits for the vehicle to
// report ARMED before sending the takeoff — without the optimistic reflection that
// confirmation waits on the sim→telemetry→heartbeat round-trip and QGC gives up
// with "Vehicle failed to arm". The sim confirms (or corrects) via telemetry.
function relayArm(armed) {
  pushCommand({ type: 'arm', value: armed ? 1 : 0 });
  vehicle = { ...vehicle, armed };
  sendHeartbeat();
}

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
    airspeed: t.airspeed, groundspeed: t.groundspeed,
    alt: t.baroAlt ?? t.alt, climb: t.climb, // altimeter = the (faultable) baro
    heading: Math.round(t.headingDeg), throttle: t.throttlePct,
  });
  sendMsg('GPS_RAW_INT', {
    time_usec: BigInt(ms) * 1000n,
    lat: Math.round((t.gpsLat ?? t.lat) * 1e7), lon: Math.round((t.gpsLon ?? t.lon) * 1e7),
    alt: Math.round((t.gpsAlt ?? t.alt) * 1000),
    eph: 80, epv: 120, vel: Math.round(t.groundspeed * 100),
    cog: Math.round(t.headingDeg * 100) % 36000,
    fix_type: t.gpsFix ?? 3, satellites_visible: t.gpsSats ?? 12,
  });
  const health = (t.health ?? 63) >>> 0;
  sendMsg('SYS_STATUS', {
    onboard_control_sensors_present: 63, onboard_control_sensors_enabled: 63,
    onboard_control_sensors_health: health,
    load: 250, voltage_battery: t.battMv ?? 12600, current_battery: t.battCa ?? -1,
    drop_rate_comm: 0, errors_comm: 0,
    errors_count1: 0, errors_count2: 0, errors_count3: 0, errors_count4: 0,
    battery_remaining: t.battPct ?? -1,
  });
  if (t.ekf) sendMsg('EKF_STATUS_REPORT', t.ekf);
  const wSpd = Math.hypot(t.windN ?? 0, t.windE ?? 0);
  sendMsg('WIND', { // meteorological: direction the wind comes FROM
    direction: ((Math.atan2(-(t.windE ?? 0), -(t.windN ?? 0)) * 180) / Math.PI + 360) % 360,
    speed: wSpd, speed_z: 0,
  });
  statusTextOnFaultEdges(t.faults);
  statusTextOnLifecycleEdges(t);
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
