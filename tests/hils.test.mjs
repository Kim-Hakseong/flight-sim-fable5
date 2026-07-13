// M14 gate: the HILS scenario runner itself.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { SCENARIOS, runScenario } from '../src/hils.js';

test('every built-in bench scenario passes', () => {
  for (const sc of SCENARIOS) {
    const rep = runScenario(sc);
    const bad = rep.results.filter((r) => !r.pass).map((r) => `${r.name} (${r.signal}=${r.worst})`);
    assert.ok(rep.pass, `${sc.name}: ${bad.join('; ')}`);
  }
});

test('reports are bit-identical across reruns (deterministic bench)', () => {
  const sc = SCENARIOS.find((s) => s.name === 'gps-dropout-recovery');
  assert.equal(JSON.stringify(runScenario(sc)), JSON.stringify(runScenario(sc)));
});

test('the runner is honest: an impossible check fails', () => {
  const rep = runScenario({
    name: 'impossible',
    boot: 'air', duration: 5, seed: 1,
    events: [{ t: 0.5, command: { type: 'mode', custom: 15 } }],
    checks: [{ name: 'teleports home', type: 'final', signal: 'distHome', max: 1 }],
  });
  assert.equal(rep.pass, false);
  assert.equal(rep.results[0].pass, false);
});

test('scenario events fire: a fault event degrades the tracked signal', () => {
  const clean = runScenario({
    name: 'clean', boot: 'air', duration: 30, seed: 21,
    events: [{ t: 0.5, command: { type: 'mode', custom: 15 } }],
    checks: [{ name: 'nav tight', type: 'final', signal: 'estErrH', max: 6 }],
  });
  const faulted = runScenario({
    name: 'faulted', boot: 'air', duration: 30, seed: 21,
    events: [
      { t: 0.5, command: { type: 'mode', custom: 15 } },
      { t: 10, fault: { sensor: 'gps', type: 'dropout' } },
    ],
    checks: [{ name: 'nav loose', type: 'final', signal: 'estErrH', min: 6 }],
  });
  assert.ok(clean.pass, 'clean run should track tightly');
  assert.ok(faulted.pass, 'faulted run should drift');
});
