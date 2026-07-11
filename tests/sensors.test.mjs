import { test } from 'node:test';
import assert from 'node:assert/strict';
import { prngNext, gaussianNext } from '../src/prng.js';
import {
  createSensors, stepSensors, injectFault, clearFault, healthBits, SENSOR_BITS, SENSORS_PRESENT,
} from '../src/sensors.js';
import { defaultParams } from '../src/params.js';
import { initialState } from '../src/physics.js';

const P = defaultParams();

test('prng: seeded stream is deterministic and roughly standard-normal', () => {
  let a = 42, b = 42;
  for (let i = 0; i < 100; i++) {
    const [va, na] = prngNext(a);
    const [vb, nb] = prngNext(b);
    assert.equal(va, vb);
    a = na; b = nb;
  }
  let s = 7, sum = 0, sumSq = 0;
  const N = 20000;
  for (let i = 0; i < N; i++) {
    const [g, next] = gaussianNext(s);
    s = next; sum += g; sumSq += g * g;
  }
  const mean = sum / N;
  const sigma = Math.sqrt(sumSq / N - mean * mean);
  assert.ok(Math.abs(mean) < 0.03, `mean = ${mean}`);
  assert.ok(Math.abs(sigma - 1) < 0.03, `sigma = ${sigma}`);
});

test('sensors: same seed → identical reading streams', () => {
  const run = () => {
    let sns = createSensors(9);
    const out = [];
    for (let i = 0; i < 120; i++) {
      const r = stepSensors(sns, initialState(), P);
      sns = r.sensors;
      out.push(r.readings);
    }
    return JSON.stringify(out);
  };
  assert.equal(run(), run());
});

test('sensors: noise sigma of the GPS reading matches the param', () => {
  let sns = createSensors(3);
  const s = initialState();
  let sumSq = 0;
  const N = 3000;
  for (let i = 0; i < N; i++) {
    const r = stepSensors(sns, s, P);
    sns = r.sensors;
    sumSq += (r.readings.gps[0] - s.pos[0]) ** 2;
  }
  const sigma = Math.sqrt(sumSq / N);
  assert.ok(Math.abs(sigma - P.SNS_GPS_SGM_M) < 0.15, `gps sigma = ${sigma}`);
});

test('faults: freeze holds, dropout nulls, bias shifts — and clear restores', () => {
  const s = initialState();
  let sns = injectFault(createSensors(5), 'gps', 'freeze');
  const r1 = stepSensors(sns, s, P);
  const r2 = stepSensors(r1.sensors, { ...s, pos: [500, 200, -500] }, P);
  assert.deepEqual(r2.readings.gps, r1.readings.gps, 'freeze must hold the first faulted reading');

  sns = injectFault(createSensors(5), 'baro', 'dropout');
  assert.equal(stepSensors(sns, s, P).readings.baro, null);

  sns = injectFault(createSensors(5), 'gps', 'bias', { bias: 200 });
  const biased = stepSensors(sns, s, P).readings.gps;
  assert.ok(Math.abs(biased[0] - s.pos[0] - 200) < 10, `biased x = ${biased[0]}`);

  const cleared = stepSensors(clearFault(sns, 'gps'), s, P).readings;
  assert.ok(Math.abs(cleared.gps[0] - s.pos[0]) < 10, 'cleared gps reads true again');
  assert.equal(cleared.health, SENSORS_PRESENT);
});

test('faults: health bits drop exactly the faulted sensors', () => {
  assert.equal(healthBits({}), SENSORS_PRESENT);
  assert.equal(healthBits({ gps: { type: 'bias' } }), SENSORS_PRESENT & ~SENSOR_BITS.gps);
  assert.equal(
    healthBits({ gps: { type: 'dropout' }, baro: { type: 'freeze' } }),
    SENSORS_PRESENT & ~SENSOR_BITS.gps & ~SENSOR_BITS.baro
  );
});

test('faults: a GPS fault must not disturb the other sensors\' noise streams', () => {
  const s = initialState();
  const clean = [];
  let a = createSensors(11);
  let b = injectFault(createSensors(11), 'gps', 'dropout');
  for (let i = 0; i < 60; i++) {
    const ra = stepSensors(a, s, P);
    const rb = stepSensors(b, s, P);
    a = ra.sensors; b = rb.sensors;
    assert.deepEqual(rb.readings.gyro, ra.readings.gyro, `gyro diverged at step ${i}`);
    assert.deepEqual(rb.readings.baro, ra.readings.baro, `baro diverged at step ${i}`);
  }
});
