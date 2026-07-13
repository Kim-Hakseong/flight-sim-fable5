// HILS bench CLI: run all built-in scenarios (or one by name) and print a report.
//   node tests/hils-run.mjs [scenario-name]
import { SCENARIOS, runScenario } from '../src/hils.js';

const pick = process.argv[2];
const list = pick ? SCENARIOS.filter((s) => s.name === pick) : SCENARIOS;
if (!list.length) {
  console.error(`unknown scenario '${pick}' — available: ${SCENARIOS.map((s) => s.name).join(', ')}`);
  process.exit(2);
}

let failed = 0;
for (const sc of list) {
  const t0 = process.hrtime.bigint();
  const rep = runScenario(sc);
  const ms = Number(process.hrtime.bigint() - t0) / 1e6;
  console.log(`${rep.pass ? 'PASS' : 'FAIL'}  ${rep.name}  (${sc.duration}s sim in ${ms.toFixed(0)}ms)`);
  for (const r of rep.results) {
    const worst = Array.isArray(r.worst)
      ? r.worst.map((x) => x.toFixed(2)).join('…')
      : r.worst?.toFixed?.(2) ?? r.worst;
    console.log(`   ${r.pass ? '✓' : '✗'} ${r.name} [${r.signal} worst ${worst}]`);
  }
  if (!rep.pass) failed++;
}
console.log(failed ? `HILS BENCH: ${failed} FAILED` : 'HILS BENCH: ALL PASS');
process.exit(failed ? 1 : 0);
