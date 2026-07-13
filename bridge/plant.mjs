// Sim-as-plant: the vehicle hosted headlessly in node, with an EXTERNAL
// controller closing the loop over UDP JSON lockstep — actual HILS topology.
// The onboard autopilot is bypassed; the external FC gets exactly what a real
// FC gets (the faultable sensor readings + WoW) and commands raw surfaces.
//
//   controller → {type:'ctl', seq, controls:{aileron,elevator,rudder,throttle}}
//   plant      → {type:'out', seq, t, readings, wow}
//   controller → {type:'reset', boot:'air'|'ground', seed?}   → {type:'ready'}
//   controller → {type:'fault', sensor, faultType, ...opts}   → applied silently
//
// The sim advances ONLY on 'ctl' packets (the controller paces it): lockstep,
// deterministic, no wall clock anywhere. QGC telemetry keeps flowing (UDP 14550)
// so the whole thing stays observable from the GCS.

import dgram from 'node:dgram';
import {
  createVehicle, vehicleStep, vehicleFault, vehicleTelemetry,
} from '../src/vehicle.js';
import { encode } from './mavlink.mjs';

const DT = 1 / 60;
const PLANT_PORT = Number(process.env.PLANT_PORT ?? 9002);
const GCS_HOST = process.env.GCS_HOST ?? '127.0.0.1';
const GCS_PORT = Number(process.env.GCS_PORT ?? 14550);

let veh = { ...createVehicle({ boot: 'air' }), armed: true };
let stepCount = 0;

const gcs = dgram.createSocket('udp4');
let seq = 0;
function sendGcs(name, fields) {
  gcs.send(encode(name, fields, { seq: seq++, sysid: 1, compid: 1 }), GCS_PORT, GCS_HOST);
}

function telemetryOut() {
  const t = vehicleTelemetry(veh);
  sendGcs('ATTITUDE', {
    time_boot_ms: t.timeBootMs >>> 0, roll: t.roll, pitch: t.pitch, yaw: t.yaw,
    rollspeed: t.rollspeed, pitchspeed: t.pitchspeed, yawspeed: t.yawspeed,
  });
  sendGcs('GLOBAL_POSITION_INT', {
    time_boot_ms: t.timeBootMs >>> 0,
    lat: Math.round(t.lat * 1e7), lon: Math.round(t.lon * 1e7),
    alt: Math.round(t.alt * 1000), relative_alt: Math.round(t.relAlt * 1000),
    vx: Math.round(t.vn * 100), vy: Math.round(t.ve * 100), vz: Math.round(t.vd * 100),
    hdg: Math.round(t.headingDeg * 100) % 36000,
  });
  if (stepCount % 60 === 0) {
    sendGcs('HEARTBEAT', {
      custom_mode: 0, type: 1, autopilot: 3, base_mode: 81 | 128,
      system_status: 4, mavlink_version: 3,
    });
  }
}

const sock = dgram.createSocket('udp4');
sock.on('message', (buf, rinfo) => {
  let msg;
  try {
    msg = JSON.parse(buf);
  } catch {
    return;
  }
  const reply = (o) => sock.send(JSON.stringify(o), rinfo.port, rinfo.address);

  if (msg.type === 'reset') {
    veh = { ...createVehicle({ boot: msg.boot ?? 'air', sensorSeed: msg.seed ?? 1, windSeed: (msg.seed ?? 1) + 1 }), armed: true };
    stepCount = 0;
    reply({ type: 'ready', boot: msg.boot ?? 'air' });
    return;
  }
  if (msg.type === 'fault') {
    veh = vehicleFault(veh, msg.sensor, msg.faultType, msg.bias !== undefined ? { bias: msg.bias } : {});
    return;
  }
  if (msg.type === 'ctl') {
    veh = vehicleStep(veh, DT, null, msg.controls ?? {});
    stepCount++;
    if (stepCount % 6 === 0) telemetryOut(); // 10 Hz toward QGC
    reply({
      type: 'out', seq: msg.seq, t: veh.simTime,
      readings: veh.readings, wow: veh.state.pos[1] <= 0.5,
    });
  }
});

sock.bind(PLANT_PORT, () => {
  console.log(`plant up udp=${sock.address().port} gcs=${GCS_HOST}:${GCS_PORT} (lockstep, DT=1/60)`);
});
