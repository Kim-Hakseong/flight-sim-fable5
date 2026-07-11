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

udp.on('message', (buf, rinfo) => {
  gcsAddr = rinfo;
  const msg = decode(buf);
  if (msg && !msg.crcOk) console.log(`gcs → bridge: ${msg.name} (BAD CRC)`);
  else if (msg) console.log(`gcs → bridge: ${msg.name}`);
  // Command handling lands in M2.
});

setInterval(() => {
  sendMsg('HEARTBEAT', {
    custom_mode: 0, // MANUAL (ArduPlane numbering, wired for real in M2)
    type: 1, // MAV_TYPE_FIXED_WING
    autopilot: 3, // MAV_AUTOPILOT_ARDUPILOTMEGA — QGC then knows the mode map
    base_mode: 81, // CUSTOM_MODE | STABILIZE | MANUAL_INPUT, disarmed
    system_status: 4, // MAV_STATE_ACTIVE
    mavlink_version: 3,
  });
}, 1000);

function relayTelemetry(t) {
  const ms = t.timeBootMs >>> 0;
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
