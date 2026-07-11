// GCS bridge: serves the static sim over HTTP (:8765), accepts telemetry POSTs
// from the browser, and relays MAVLink v1 over UDP to the GCS (QGC on :14550).
// Config via env: BRIDGE_HTTP_PORT, GCS_HOST, GCS_PORT.
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import dgram from 'node:dgram';
import { join, extname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { encode, decode } from './mavlink.mjs';

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
    default:
      return MAV_RESULT_UNSUPPORTED;
  }
}

udp.on('message', (buf, rinfo) => {
  gcsAddr = rinfo;
  const msg = decode(buf);
  if (!msg || !msg.crcOk) {
    if (msg) console.log(`gcs → bridge: ${msg.name} (BAD CRC, dropped)`);
    return;
  }
  if (msg.name === 'COMMAND_LONG') {
    const result = handleCommandLong(msg.fields);
    sendMsg('COMMAND_ACK', { command: msg.fields.command, result });
  } else if (msg.name === 'SET_MODE') {
    pushCommand({ type: 'mode', custom: msg.fields.custom_mode });
  } else if (msg.name !== 'HEARTBEAT') {
    console.log(`gcs → bridge: ${msg.name} (ignored)`);
  }
});

// The sim is authoritative for arm/mode: HEARTBEAT reflects the last telemetry.
let vehicle = { armed: true, customMode: 0 };

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
