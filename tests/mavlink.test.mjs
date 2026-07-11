import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  MAGIC_V1, MESSAGES, crcX25, encode, decode, payloadLength,
} from '../bridge/mavlink.mjs';

test('crc: X.25 / CRC-16-MCRF4XX known vector ("123456789" → 0x6F91)', () => {
  const bytes = [...'123456789'].map((c) => c.charCodeAt(0));
  assert.equal(crcX25(bytes), 0x6f91);
});

test('payload lengths match the MAVLink v1 spec', () => {
  assert.equal(payloadLength(MESSAGES.HEARTBEAT), 9);
  assert.equal(payloadLength(MESSAGES.ATTITUDE), 28);
  assert.equal(payloadLength(MESSAGES.GLOBAL_POSITION_INT), 28);
  assert.equal(payloadLength(MESSAGES.VFR_HUD), 20);
  assert.equal(payloadLength(MESSAGES.GPS_RAW_INT), 30);
  assert.equal(payloadLength(MESSAGES.COMMAND_LONG), 33);
  assert.equal(payloadLength(MESSAGES.COMMAND_ACK), 3);
  assert.equal(payloadLength(MESSAGES.SET_MODE), 6);
  assert.equal(payloadLength(MESSAGES.MISSION_COUNT), 4);
  assert.equal(payloadLength(MESSAGES.MISSION_REQUEST_LIST), 2);
  assert.equal(payloadLength(MESSAGES.MISSION_REQUEST), 4);
  assert.equal(payloadLength(MESSAGES.MISSION_REQUEST_INT), 4);
  assert.equal(payloadLength(MESSAGES.MISSION_ITEM_INT), 37);
  assert.equal(payloadLength(MESSAGES.MISSION_ACK), 3);
  assert.equal(payloadLength(MESSAGES.MISSION_CURRENT), 2);
  assert.equal(payloadLength(MESSAGES.MISSION_ITEM_REACHED), 2);
  assert.equal(payloadLength(MESSAGES.COMMAND_INT), 35);
  assert.equal(payloadLength(MESSAGES.PARAM_REQUEST_READ), 20);
  assert.equal(payloadLength(MESSAGES.PARAM_REQUEST_LIST), 2);
  assert.equal(payloadLength(MESSAGES.PARAM_VALUE), 25);
  assert.equal(payloadLength(MESSAGES.PARAM_SET), 23);
  assert.equal(payloadLength(MESSAGES.SYS_STATUS), 31);
  assert.equal(payloadLength(MESSAGES.STATUSTEXT), 51);
});

test('statustext + sys_status round-trip (char50, health bits)', () => {
  const st = decode(encode('STATUSTEXT', { severity: 4, text: 'GPS fault: bias' }));
  assert.equal(st.crcOk, true);
  assert.equal(st.fields.severity, 4);
  assert.equal(st.fields.text, 'GPS fault: bias');

  const ss = decode(encode('SYS_STATUS', {
    onboard_control_sensors_present: 47, onboard_control_sensors_enabled: 47,
    onboard_control_sensors_health: 47 & ~32, // GPS unhealthy
    load: 250, voltage_battery: 12600, current_battery: -1,
    drop_rate_comm: 0, errors_comm: 0,
    errors_count1: 0, errors_count2: 0, errors_count3: 0, errors_count4: 0,
    battery_remaining: -1,
  }));
  assert.equal(ss.crcOk, true);
  assert.equal(ss.fields.onboard_control_sensors_health, 15);
  assert.equal(ss.fields.battery_remaining, -1);
});

test('char16: param ids pad with NULs and round-trip; 16-char ids fit exactly', () => {
  const short = decode(encode('PARAM_VALUE', {
    param_value: 0.7, param_count: 12, param_index: 7, param_id: 'AP_THR_CRUISE', param_type: 9,
  }));
  assert.equal(short.fields.param_id, 'AP_THR_CRUISE');
  assert.equal(short.crcOk, true);

  const exact = decode(encode('PARAM_SET', {
    param_value: 1, target_system: 1, target_component: 1,
    param_id: 'ABCDEFGHIJKLMNOP', param_type: 9, // exactly 16 — no NUL terminator on wire
  }));
  assert.equal(exact.fields.param_id, 'ABCDEFGHIJKLMNOP');
});

test('heartbeat frame: header layout and wire-order payload', () => {
  const pkt = encode('HEARTBEAT', {
    custom_mode: 0x04030201, type: 1, autopilot: 3,
    base_mode: 81, system_status: 4, mavlink_version: 3,
  }, { seq: 7, sysid: 1, compid: 1 });
  assert.equal(pkt.length, 6 + 9 + 2);
  assert.deepEqual([...pkt.slice(0, 6)], [MAGIC_V1, 9, 7, 1, 1, 0]);
  // custom_mode (uint32) must be FIRST in the payload — the v1 size reorder.
  assert.deepEqual([...pkt.slice(6, 10)], [0x01, 0x02, 0x03, 0x04]);
  assert.deepEqual([...pkt.slice(10, 15)], [1, 3, 81, 4, 3]);
});

test('round-trip: every M1 message encodes → decodes with crcOk', () => {
  const samples = {
    HEARTBEAT: { custom_mode: 10, type: 1, autopilot: 3, base_mode: 209, system_status: 4, mavlink_version: 3 },
    ATTITUDE: { time_boot_ms: 123456, roll: 0.1, pitch: -0.05, yaw: 1.57, rollspeed: 0.01, pitchspeed: -0.02, yawspeed: 0.005 },
    GLOBAL_POSITION_INT: { time_boot_ms: 123456, lat: 374449000, lon: 1264656000, alt: 127000, relative_alt: 120000, vx: 4000, vy: 12, vz: -110, hdg: 35999 },
    VFR_HUD: { airspeed: 41.5, groundspeed: 40.2, alt: 127, climb: 1.1, heading: 359, throttle: 65 },
    GPS_RAW_INT: { time_usec: 123456789n, lat: 374449000, lon: 1264656000, alt: 127000, eph: 80, epv: 120, vel: 4020, cog: 35900, fix_type: 3, satellites_visible: 12 },
    COMMAND_LONG: { param1: 1, param2: 0, param3: 0, param4: 0, param5: 0, param6: 0, param7: 50, command: 400, target_system: 1, target_component: 1, confirmation: 0 },
    COMMAND_ACK: { command: 400, result: 0 },
    SET_MODE: { custom_mode: 15, target_system: 1, base_mode: 1 },
    MISSION_COUNT: { count: 3, target_system: 1, target_component: 1 },
    MISSION_REQUEST_INT: { seq: 2, target_system: 1, target_component: 1 },
    MISSION_ITEM_INT: { param1: 0, param2: 60, param3: 0, param4: 0, x: 374569000, y: 1264796000, z: 120, seq: 1, command: 16, target_system: 1, target_component: 1, frame: 3, current: 0, autocontinue: 1 },
    MISSION_ACK: { target_system: 255, target_component: 0, type: 0 },
    MISSION_CURRENT: { seq: 2 },
    MISSION_ITEM_REACHED: { seq: 1 },
    COMMAND_INT: { param1: 0, param2: 0, param3: 0, param4: 0, x: 374569000, y: 1264796000, z: 140, command: 192, target_system: 1, target_component: 1, frame: 3, current: 0, autocontinue: 0 },
  };
  for (const [name, fields] of Object.entries(samples)) {
    const msg = decode(encode(name, fields, { seq: 42 }));
    assert.ok(msg, `${name}: decode returned null`);
    assert.equal(msg.name, name);
    assert.equal(msg.crcOk, true, `${name}: bad CRC`);
    assert.equal(msg.seq, 42);
    for (const [k, v] of Object.entries(fields)) {
      if (typeof v === 'bigint') assert.equal(msg.fields[k], v, `${name}.${k}`);
      else assert.ok(Math.abs(Number(msg.fields[k]) - v) < 1e-3, `${name}.${k}: ${msg.fields[k]} ≠ ${v}`);
    }
  }
});

test('decode: rejects a corrupted CRC and resyncs past garbage', () => {
  const pkt = encode('HEARTBEAT', { custom_mode: 0, type: 1, autopilot: 3, base_mode: 81, system_status: 4, mavlink_version: 3 });
  const bad = Uint8Array.from(pkt);
  bad[bad.length - 1] ^= 0xff;
  assert.equal(decode(bad).crcOk, false);

  const noisy = new Uint8Array([0x00, 0xfe, 0x03, ...pkt]);
  const msg = decode(noisy);
  assert.equal(msg.name, 'HEARTBEAT');
  assert.equal(msg.crcOk, true);
});

test('decode: returns null on truncated or unknown input', () => {
  assert.equal(decode(new Uint8Array([0xfe, 9, 0, 1])), null);
  assert.equal(decode(new Uint8Array(20).fill(0x55)), null);
});
