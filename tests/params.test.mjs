import { test } from 'node:test';
import assert from 'node:assert/strict';
import { PARAM_DEFS, defaultParams, clampParam } from '../src/params.js';
import { holdControls } from '../src/autopilot.js';
import { initialState } from '../src/physics.js';

test('table integrity: unique MAVLink-legal ids, defaults inside their range', () => {
  const ids = new Set();
  for (const p of PARAM_DEFS) {
    assert.ok(p.id.length >= 1 && p.id.length <= 16, `${p.id}: bad length`);
    assert.ok(!ids.has(p.id), `${p.id}: duplicate`);
    ids.add(p.id);
    assert.ok(p.min <= p.def && p.def <= p.max, `${p.id}: default outside range`);
  }
});

test('clampParam: clamps into range, rejects unknown ids and NaN', () => {
  assert.equal(clampParam('AP_VA_TRIM', 28), 28);
  assert.equal(clampParam('AP_VA_TRIM', 99), 40);
  assert.equal(clampParam('AP_VA_TRIM', 1), 22);
  assert.equal(clampParam('NOPE', 1), null);
  assert.equal(clampParam('AP_VA_TRIM', NaN), null);
});

test('a param change actually re-tunes the controller output', () => {
  const s = initialState();
  const soft = { ...defaultParams(), AP_BANK_MAX: 0.15, AP_HDG_P: 0.005, AP_ROLL_KP: 0.2 };
  const base = holdControls(s, 120, 90); // 90° right of current heading
  const tuned = holdControls(s, 120, 90, null, soft);
  assert.ok(Math.abs(tuned.aileron) < Math.abs(base.aileron),
    `soft gains must command less aileron (${tuned.aileron} vs ${base.aileron})`);
});
