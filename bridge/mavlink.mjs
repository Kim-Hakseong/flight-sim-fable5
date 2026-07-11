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

// name → { id, crcExtra, fields: [[fieldName, type], …] in wire order }
export const MESSAGES = {
  HEARTBEAT: {
    id: 0, crcExtra: 50,
    fields: [
      ['custom_mode', 'uint32'], ['type', 'uint8'], ['autopilot', 'uint8'],
      ['base_mode', 'uint8'], ['system_status', 'uint8'], ['mavlink_version', 'uint8'],
    ],
  },
  SET_MODE: {
    id: 11, crcExtra: 89,
    fields: [
      ['custom_mode', 'uint32'], ['target_system', 'uint8'], ['base_mode', 'uint8'],
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

export function encode(name, values, { seq = 0, sysid = 1, compid = 1 } = {}) {
  const def = MESSAGES[name];
  if (!def) throw new Error(`unknown message: ${name}`);
  const len = payloadLength(def);
  const buf = new Uint8Array(6 + len + 2);
  const view = new DataView(buf.buffer);
  buf[0] = MAGIC_V1;
  buf[1] = len;
  buf[2] = seq & 0xff;
  buf[3] = sysid;
  buf[4] = compid;
  buf[5] = def.id;
  let o = 6;
  for (const [field, type] of def.fields) {
    TYPES[type].set(view, o, values[field] ?? 0);
    o += TYPES[type].size;
  }
  let crc = crcX25(buf.subarray(1, 6 + len));
  crc = crcAccumulate(def.crcExtra, crc);
  view.setUint16(6 + len, crc, true);
  return buf;
}

// Decode the first well-formed v1 packet found in buf.
// Returns { name, msgid, seq, sysid, compid, fields, crcOk, bytes } or null.
export function decode(buf) {
  for (let i = 0; i + 8 <= buf.length; i++) {
    if (buf[i] !== MAGIC_V1) continue;
    const len = buf[i + 1];
    const end = i + 6 + len + 2;
    if (end > buf.length) continue;
    const msgid = buf[i + 5];
    const def = MESSAGES_BY_ID.get(msgid);
    if (!def || payloadLength(def) !== len) continue;
    let crc = crcX25(buf.subarray(i + 1, i + 6 + len));
    crc = crcAccumulate(def.crcExtra, crc);
    const wire = buf[i + 6 + len] | (buf[i + 6 + len + 1] << 8);
    const view = new DataView(buf.buffer, buf.byteOffset + i + 6, len);
    const fields = {};
    let o = 0;
    for (const [field, type] of def.fields) {
      fields[field] = TYPES[type].get(view, o);
      o += TYPES[type].size;
    }
    return {
      name: def.name, msgid, seq: buf[i + 2], sysid: buf[i + 3], compid: buf[i + 4],
      fields, crcOk: wire === crc, bytes: end - i,
    };
  }
  return null;
}
