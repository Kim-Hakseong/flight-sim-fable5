// ArduPilot SITL "JSON" backend adapter: run a REAL ArduPilot binary against our
// physics. ArduPilot (e.g. `sim_vehicle.py -v ArduPlane --model JSON`) sends a
// binary servo packet each loop and expects a one-line JSON state reply, in
// strict lockstep:
//
//   AP → { uint16 magic=18458, uint16 frame_rate, uint32 frame_count, uint16 pwm[16] } LE
//   us → {"timestamp":s,"imu":{"gyro":[p,q,r],"accel_body":[x,y,z]},
//         "position":[N,E,D],"attitude":[roll,pitch,yaw],"velocity":[N,E,D],"airspeed":Va}\n
//
// Truth is returned (FRD/NED, radians, meters) — ArduPilot layers its OWN sensor
// models on top. Same frame_count → resend (no step); frame_count going backwards
// → ArduPilot restarted → reset the physics. Wind via WND_N_MS/WND_E_MS/WND_TRB env.
//
// Servo mapping (ArduPlane defaults): 1 aileron, 2 elevator, 3 throttle, 4 rudder.
// Signs: AP elevator+ = nose up = our elevator−; AP rudder+ = yaw right = our rudder−.

import dgram from 'node:dgram';
import {
  stepAircraft, groundState, airData, toFRD, quatRotate, quatConjugate, G,
} from '../src/physics.js';
import { specificForce } from '../src/sensors.js';
import { eulerFromQuat } from '../src/telemetry.js';
import { createWind, stepWind } from '../src/wind.js';
import { defaultParams } from '../src/params.js';

const PORT = Number(process.env.SITL_PORT ?? 9002);
const MAGIC = 18458;
const params = {
  ...defaultParams(),
  WND_N_MS: Number(process.env.WND_N_MS ?? 0),
  WND_E_MS: Number(process.env.WND_E_MS ?? 0),
  WND_TRB: Number(process.env.WND_TRB ?? 0), // calm default: AP's EKF appreciates it
};

function boot() {
  return { state: { ...groundState(), pos: [0, 0, 0] }, wind: createWind(2), ww: [0, 0, 0], t: 0 };
}
let sim = boot();
let lastFrame = -1;
let lastReply = null;

function pwmNorm(p) { // 1000..2000 → −1..1 around 1500
  return Math.max(-1, Math.min(1, (p - 1500) / 500));
}

function stateJson() {
  const { state, ww, t } = sim;
  const e = eulerFromQuat(state.quat);
  const [p, q, r] = toFRD(state.omega);
  // Specific force in FRD (z ≈ −g at rest). Our ground contact is a kinematic
  // clamp (no modelled normal force), so on the ground substitute the support
  // reaction — otherwise ArduPilot's IMU would read free-fall while parked.
  const accOurs = state.pos[1] <= 0
    ? quatRotate(quatConjugate(state.quat), [0, G, 0])
    : specificForce(state, ww);
  const acc = toFRD(accOurs);
  return JSON.stringify({
    timestamp: t,
    imu: { gyro: [p, q, r], accel_body: acc },
    position: [-state.pos[2], state.pos[0], -state.pos[1]], // ours → NED
    attitude: [e.roll, e.pitch, e.yaw],
    velocity: [-state.vel[2], state.vel[0], -state.vel[1]],
    airspeed: airData(state.quat, state.vel, ww).Va,
  }) + '\n';
}

const sock = dgram.createSocket('udp4');
sock.on('message', (buf, rinfo) => {
  if (buf.length < 40 || buf.readUInt16LE(0) !== MAGIC) return;
  const frameRate = buf.readUInt16LE(2);
  const frameCount = buf.readUInt32LE(4);
  const pwm = [];
  for (let i = 0; i < 16; i++) pwm.push(buf.readUInt16LE(8 + i * 2));

  if (frameCount === lastFrame && lastReply) { // duplicate: resend, don't step
    sock.send(lastReply, rinfo.port, rinfo.address);
    return;
  }
  if (frameCount < lastFrame) { // ArduPilot restarted
    sim = boot();
  }
  lastFrame = frameCount;

  const dt = 1 / Math.max(50, Math.min(frameRate || 1200, 2000));
  const controls = {
    aileron: pwmNorm(pwm[0]),
    elevator: -pwmNorm(pwm[1]), // AP elevator+ = nose up; ours pitches down
    throttle: Math.max(0, Math.min(1, (pwm[2] - 1000) / 1000)),
    rudder: -pwmNorm(pwm[3]), // AP rudder+ = yaw right; ours yaws left
  };
  const w = stepWind(sim.wind, sim.state, params, dt);
  sim = {
    state: stepAircraft(sim.state, controls, dt, w.windWorld),
    wind: w.wind, ww: w.windWorld, t: sim.t + dt,
  };
  lastReply = stateJson();
  sock.send(lastReply, rinfo.port, rinfo.address);
});

sock.bind(PORT, () => {
  console.log(`sitl-json up udp=${sock.address().port} (ArduPilot --model JSON, lockstep)`);
});
