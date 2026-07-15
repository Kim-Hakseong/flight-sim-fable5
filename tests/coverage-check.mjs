// Structural coverage gate for the JS REFERENCE MODEL modules (the golden source
// the C/FMU deployable is validated against). Uses node's built-in coverage
// (zero deps). Browser-only modules (main/scene/engineering/missionLink) are
// excluded — they are not model code and never run under `node --test`.
//
// Per-module line-coverage floors are asserted; a regression fails CI.
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { readdirSync } from 'node:fs';

const root = fileURLToPath(new URL('..', import.meta.url));
// Only *.test.mjs — the *-check.mjs / *-run.mjs scripts spawn servers and are
// not unit tests; running the whole dir would hang/fail.
const testFiles = readdirSync(new URL('.', import.meta.url))
  .filter((f) => f.endsWith('.test.mjs'))
  .map((f) => `tests/${f}`);

// module → minimum line coverage % (set from measured reality, not aspiration)
const FLOORS = {
  'physics.js': 100,
  'wind.js': 100,
  'estimator.js': 100,
  'sensors.js': 100,
  'autopilot.js': 100,
  'missions.js': 100,
  'params.js': 100,
  'battery.js': 100,
  'prng.js': 100,
  // telemetry.js pure math (geodetic/euler/telemetryFrom) is fully covered;
  // the shortfall is startTelemetry (a browser fetch loop, not model code).
  'telemetry.js': 85,
  'vehicle.js': 95, // a couple of browser-wiring branches unrun in node
};

const res = spawnSync('node', ['--test', '--experimental-test-coverage', ...testFiles], {
  cwd: root, encoding: 'utf8', maxBuffer: 64 * 1024 * 1024,
});
const out = res.stdout + res.stderr;
if (res.status !== 0 && !out.includes('# tests ')) {
  console.error('test run failed:\n', out.slice(-2000));
  process.exit(2);
}

// Parse the coverage table rows: "# src/physics.js | 100.00 | 88.89 | 100.00 | ..."
const rows = {};
for (const line of out.split('\n')) {
  const m = line.match(/([A-Za-z0-9_]+\.js)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)/);
  if (m) rows[m[1]] = { line: +m[2], branch: +m[3], func: +m[4] };
}

let failures = 0;
console.log('JS reference-model coverage (line % / branch % / func %):');
for (const [mod, floor] of Object.entries(FLOORS)) {
  const r = rows[mod];
  if (!r) { console.log(`  ✗ ${mod.padEnd(14)} — NOT MEASURED`); failures++; continue; }
  const ok = r.line >= floor;
  console.log(`  ${ok ? 'ok ' : '✗ '} ${mod.padEnd(14)} ${r.line.toFixed(1)} / ${r.branch.toFixed(1)} / ${r.func.toFixed(1)}  (line floor ${floor})`);
  if (!ok) failures++;
}
console.log(failures ? `JS COVERAGE: ${failures} below floor` : 'JS COVERAGE: PASS');
process.exit(failures ? 1 : 0);
