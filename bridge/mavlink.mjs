// MAVLink v1 encode/decode — no deps, shared by the bridge and node tests.
// Wire layout: 0xFE len seq sysid compid msgid payload crc_lo crc_hi.
// Field lists below are ALREADY in wire order (descending type size, stable),
// per the MAVLink v1 field-reorder rule. CRC is X.25 seeded over
// len..payload + the per-message crc_extra.

export const MAGIC_V1 = 0xfe;

const TYPES = {
  uint64: { size: 8, get: (v, o) => v.getBigUint64(o, true), set: (v, o, x) => v.setBigUint64(o, BigInt(x), true) },
  int64: { size: 8, get: (v, o) => v.getBigInt64(o, true), set: (v, o, x) => v.setBigInt64(o, BigInt(x), true) },
  double: { size: 8, get: (v, o) => v.getFloat64(o, true), set: (v, o, x) => v.setFloat64(o, x, true) },
  uint32: { size: 4, get: (v, o) => v.getUint32(o, true), set: (v, o, x) => v.setUint32(o, x >>> 0, true) },
  int32: { size: 4, get: (v, o) => v.getInt32(o, true), set: (v, o, x) => v.setInt32(o, x | 0, true) },
  float: { size: 4, get: (v, o) => v.getFloat32(o, true), set: (v, o, x) => v.setFloat32(o, x, true) },
  uint16: { size: 2, get: (v, o) => v.getUint16(o, true), set: (v, o, x) => v.setUint16(o, x & 0xffff, true) },
  int16: { size: 2, get: (v, o) => v.getInt16(o, true), set: (v, o, x) => v.setInt16(o, x, true) },
  uint8: { size: 1, get: (v, o) => v.getUint8(o), set: (v, o, x) => v.setUint8(o, x & 0xff) },
  int8: { size: 1, get: (v, o) => v.getInt8(o), set: (v, o, x) => v.setInt8(o, x) },
};

// Fixed-size NUL-padded ASCII strings (param_id, statustext); sort as 1-byte arrays.
for (const n of [16, 50]) {
  TYPES[`char${n}`] = {
    size: n,
    get: (v, o) => {
      let s = '';
      for (let i = 0; i < n; i++) {
        const c = v.getUint8(o + i);
        if (c === 0) break;
        s += String.fromCharCode(c);
      }
      return s;
    },
    set: (v, o, x) => {
      const s = String(x ?? '');
      for (let i = 0; i < n; i++) v.setUint8(o + i, i < s.length ? s.charCodeAt(i) & 0x7f : 0);
    },
  };
}

// name → { id, crcExtra, fields: [[fieldName, type], …] in wire order }
export const MESSAGES = {
  HEARTBEAT: {
    id: 0, crcExtra: 50,
    fields: [
      ['custom_mode', 'uint32'], ['type', 'uint8'], ['autopilot', 'uint8'],
      ['base_mode', 'uint8'], ['system_status', 'uint8'], ['mavlink_version', 'uint8'],
    ],
  },
  SYS_STATUS: {
    id: 1, crcExtra: 124,
    fields: [
      ['onboard_control_sensors_present', 'uint32'],
      ['onboard_control_sensors_enabled', 'uint32'],
      ['onboard_control_sensors_health', 'uint32'],
      ['load', 'uint16'], ['voltage_battery', 'uint16'], ['current_battery', 'int16'],
      ['drop_rate_comm', 'uint16'], ['errors_comm', 'uint16'],
      ['errors_count1', 'uint16'], ['errors_count2', 'uint16'],
      ['errors_count3', 'uint16'], ['errors_count4', 'uint16'],
      ['battery_remaining', 'int8'],
    ],
  },
  EKF_STATUS_REPORT: { // ardupilotmega dialect — QGC's EKF health widget
    id: 193, crcExtra: 71,
    fields: [
      ['velocity_variance', 'float'], ['pos_horiz_variance', 'float'],
      ['pos_vert_variance', 'float'], ['compass_variance', 'float'],
      ['terrain_alt_variance', 'float'], ['flags', 'uint16'],
    ],
  },
  STATUSTEXT: {
    id: 253, crcExtra: 83,
    fields: [['severity', 'uint8'], ['text', 'char50']],
  },
  SET_MODE: {
    id: 11, crcExtra: 89,
    fields: [
      ['custom_mode', 'uint32'], ['target_system', 'uint8'], ['base_mode', 'uint8'],
    ],
  },
  PARAM_REQUEST_READ: {
    id: 20, crcExtra: 214,
    fields: [
      ['param_index', 'int16'], ['target_system', 'uint8'], ['target_component', 'uint8'],
      ['param_id', 'char16'],
    ],
  },
  PARAM_REQUEST_LIST: {
    id: 21, crcExtra: 159,
    fields: [['target_system', 'uint8'], ['target_component', 'uint8']],
  },
  PARAM_VALUE: {
    id: 22, crcExtra: 220,
    fields: [
      ['param_value', 'float'], ['param_count', 'uint16'], ['param_index', 'uint16'],
      ['param_id', 'char16'], ['param_type', 'uint8'],
    ],
  },
  PARAM_SET: {
    id: 23, crcExtra: 168,
    fields: [
      ['param_value', 'float'], ['target_system', 'uint8'], ['target_component', 'uint8'],
      ['param_id', 'char16'], ['param_type', 'uint8'],
    ],
  },
  GPS_RAW_INT: {
    id: 24, crcExtra: 24,
    fields: [
      ['time_usec', 'uint64'], ['lat', 'int32'], ['lon', 'int32'], ['alt', 'int32'],
      ['eph', 'uint16'], ['epv', 'uint16'], ['vel', 'uint16'], ['cog', 'uint16'],
      ['fix_type', 'uint8'], ['satellites_visible', 'uint8'],
    ],
  },
  ATTITUDE: {
    id: 30, crcExtra: 39,
    fields: [
      ['time_boot_ms', 'uint32'], ['roll', 'float'], ['pitch', 'float'], ['yaw', 'float'],
      ['rollspeed', 'float'], ['pitchspeed', 'float'], ['yawspeed', 'float'],
    ],
  },
  GLOBAL_POSITION_INT: {
    id: 33, crcExtra: 104,
    fields: [
      ['time_boot_ms', 'uint32'], ['lat', 'int32'], ['lon', 'int32'], ['alt', 'int32'],
      ['relative_alt', 'int32'], ['vx', 'int16'], ['vy', 'int16'], ['vz', 'int16'],
      ['hdg', 'uint16'],
    ],
  },
  MISSION_ITEM: { // legacy float-coord item; QGC uses THIS for ArduPilot guided go-to
    id: 39, crcExtra: 254,
    fields: [
      ['param1', 'float'], ['param2', 'float'], ['param3', 'float'], ['param4', 'float'],
      ['x', 'float'], ['y', 'float'], ['z', 'float'], // x=lat, y=lon in DEGREES (not 1e7)
      ['seq', 'uint16'], ['command', 'uint16'],
      ['target_system', 'uint8'], ['target_component', 'uint8'], ['frame', 'uint8'],
      ['current', 'uint8'], ['autocontinue', 'uint8'],
    ],
  },
  MISSION_REQUEST: {
    id: 40, crcExtra: 230,
    fields: [['seq', 'uint16'], ['target_system', 'uint8'], ['target_component', 'uint8']],
  },
  MISSION_CURRENT: {
    id: 42, crcExtra: 28,
    fields: [['seq', 'uint16']],
  },
  MISSION_REQUEST_LIST: {
    id: 43, crcExtra: 132,
    fields: [['target_system', 'uint8'], ['target_component', 'uint8']],
  },
  MISSION_COUNT: {
    id: 44, crcExtra: 221,
    fields: [['count', 'uint16'], ['target_system', 'uint8'], ['target_component', 'uint8']],
  },
  MISSION_ITEM_REACHED: {
    id: 46, crcExtra: 11,
    fields: [['seq', 'uint16']],
  },
  MISSION_ACK: {
    id: 47, crcExtra: 153,
    fields: [['target_system', 'uint8'], ['target_component', 'uint8'], ['type', 'uint8']],
  },
  MISSION_REQUEST_INT: {
    id: 51, crcExtra: 196,
    fields: [['seq', 'uint16'], ['target_system', 'uint8'], ['target_component', 'uint8']],
  },
  MISSION_ITEM_INT: {
    id: 73, crcExtra: 38,
    fields: [
      ['param1', 'float'], ['param2', 'float'], ['param3', 'float'], ['param4', 'float'],
      ['x', 'int32'], ['y', 'int32'], ['z', 'float'],
      ['seq', 'uint16'], ['command', 'uint16'],
      ['target_system', 'uint8'], ['target_component', 'uint8'], ['frame', 'uint8'],
      ['current', 'uint8'], ['autocontinue', 'uint8'],
    ],
  },
  COMMAND_INT: {
    id: 75, crcExtra: 158,
    fields: [
      ['param1', 'float'], ['param2', 'float'], ['param3', 'float'], ['param4', 'float'],
      ['x', 'int32'], ['y', 'int32'], ['z', 'float'], ['command', 'uint16'],
      ['target_system', 'uint8'], ['target_component', 'uint8'], ['frame', 'uint8'],
      ['current', 'uint8'], ['autocontinue', 'uint8'],
    ],
  },
  MANUAL_CONTROL: { // GCS virtual joystick; axes −1000..1000, z 0..1000
    id: 69, crcExtra: 243,
    fields: [
      ['x', 'int16'], ['y', 'int16'], ['z', 'int16'], ['r', 'int16'],
      ['buttons', 'uint16'], ['target', 'uint8'],
    ],
  },
  WIND: { // ardupilotmega: QGC's wind indicator. direction = coming-FROM, deg
    id: 168, crcExtra: 1,
    fields: [['direction', 'float'], ['speed', 'float'], ['speed_z', 'float']],
  },
  COMMAND_LONG: {
    id: 76, crcExtra: 152,
    fields: [
      ['param1', 'float'], ['param2', 'float'], ['param3', 'float'], ['param4', 'float'],
      ['param5', 'float'], ['param6', 'float'], ['param7', 'float'],
      ['command', 'uint16'], ['target_system', 'uint8'], ['target_component', 'uint8'],
      ['confirmation', 'uint8'],
    ],
  },
  COMMAND_ACK: {
    id: 77, crcExtra: 143,
    fields: [['command', 'uint16'], ['result', 'uint8']],
  },
  VFR_HUD: {
    id: 74, crcExtra: 20,
    fields: [
      ['airspeed', 'float'], ['groundspeed', 'float'], ['alt', 'float'], ['climb', 'float'],
      ['heading', 'int16'], ['throttle', 'uint16'],
    ],
  },
};

export const MESSAGES_BY_ID = new Map(
  Object.entries(MESSAGES).map(([name, def]) => [def.id, { name, ...def }])
);

export function payloadLength(def) {
  return def.fields.reduce((n, [, t]) => n + TYPES[t].size, 0);
}

// X.25 / CRC-16-MCRF4XX, as specified by MAVLink.
export function crcAccumulate(byte, crc) {
  let tmp = (byte ^ (crc & 0xff)) & 0xff;
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

export function crcX25(bytes, crc = 0xffff) {
  for (const b of bytes) crc = crcAccumulate(b, crc);
  return crc;
}

export const MAGIC_V2 = 0xfd;

function packPayload(def, values) {
  const full = new Uint8Array(payloadLength(def));
  const view = new DataView(full.buffer);
  let o = 0;
  for (const [field, type] of def.fields) {
    TYPES[type].set(view, o, values[field] ?? 0);
    o += TYPES[type].size;
  }
  return full;
}

// v2 framing: 0xFD, 24-bit msgid, trailing zero bytes of the payload truncated.
export function encode(name, values, { seq = 0, sysid = 1, compid = 1, v2 = false } = {}) {
  const def = MESSAGES[name];
  if (!def) throw new Error(`unknown message: ${name}`);
  const payload = packPayload(def, values);
  if (!v2) {
    const len = payload.length;
    const buf = new Uint8Array(6 + len + 2);
    buf.set([MAGIC_V1, len, seq & 0xff, sysid, compid, def.id], 0);
    buf.set(payload, 6);
    let crc = crcX25(buf.subarray(1, 6 + len));
    crc = crcAccumulate(def.crcExtra, crc);
    new DataView(buf.buffer).setUint16(6 + len, crc, true);
    return buf;
  }
  let len = payload.length;
  while (len > 1 && payload[len - 1] === 0) len--; // v2 payload truncation
  const buf = new Uint8Array(10 + len + 2);
  buf.set([
    MAGIC_V2, len, 0, 0, seq & 0xff, sysid, compid,
    def.id & 0xff, (def.id >> 8) & 0xff, (def.id >> 16) & 0xff,
  ], 0);
  buf.set(payload.subarray(0, len), 10);
  let crc = crcX25(buf.subarray(1, 10 + len));
  crc = crcAccumulate(def.crcExtra, crc);
  new DataView(buf.buffer).setUint16(10 + len, crc, true);
  return buf;
}

function unpack(def, buf, start, len) {
  const full = new Uint8Array(payloadLength(def)); // zero-extend truncated v2 payloads
  full.set(buf.subarray(start, start + len));
  const view = new DataView(full.buffer);
  const fields = {};
  let o = 0;
  for (const [field, type] of def.fields) {
    fields[field] = TYPES[type].get(view, o);
    o += TYPES[type].size;
  }
  return fields;
}

// Decode the first well-formed v1 OR v2 packet found in buf.
// Returns { name, msgid, seq, sysid, compid, fields, crcOk, v2, bytes } or null.
export function decode(buf) {
  for (let i = 0; i + 8 <= buf.length; i++) {
    if (buf[i] === MAGIC_V1) {
      const len = buf[i + 1];
      const end = i + 6 + len + 2;
      if (end > buf.length) continue;
      const def = MESSAGES_BY_ID.get(buf[i + 5]);
      if (!def || payloadLength(def) !== len) continue;
      let crc = crcX25(buf.subarray(i + 1, i + 6 + len));
      crc = crcAccumulate(def.crcExtra, crc);
      const wire = buf[i + 6 + len] | (buf[i + 6 + len + 1] << 8);
      return {
        name: def.name, msgid: def.id, seq: buf[i + 2], sysid: buf[i + 3], compid: buf[i + 4],
        fields: unpack(def, buf, i + 6, len), crcOk: wire === crc, v2: false, bytes: end - i,
      };
    }
    if (buf[i] === MAGIC_V2) {
      const len = buf[i + 1];
      if (buf[i + 2] & 0x01) continue; // signed packets unsupported: skip
      const end = i + 10 + len + 2;
      if (end > buf.length) continue;
      const msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16);
      const def = MESSAGES_BY_ID.get(msgid);
      if (!def || len > payloadLength(def)) continue;
      let crc = crcX25(buf.subarray(i + 1, i + 10 + len));
      crc = crcAccumulate(def.crcExtra, crc);
      const wire = buf[i + 10 + len] | (buf[i + 10 + len + 1] << 8);
      return {
        name: def.name, msgid, seq: buf[i + 4], sysid: buf[i + 5], compid: buf[i + 6],
        fields: unpack(def, buf, i + 10, len), crcOk: wire === crc, v2: true, bytes: end - i,
      };
    }
  }
  return null;
}
