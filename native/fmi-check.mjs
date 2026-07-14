// FMU verification: drive the BUILT FMU shared lib through the real fmi2 ABI
// (via the compiled fmi-driver) and compare its final state to a JS golden
// trajectory computed here. Proves the packaged, dlopen-loaded FMU — not just
// the C core — reproduces the reference within trajectory tolerance.
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { existsSync } from 'node:fs';
import { stepAircraft, initialState, groundState, TRIM, AC, airData } from '../src/physics.js';
import { createWind, stepWind } from '../src/wind.js';
import { defaultParams } from '../src/params.js';

const dir = fileURLToPath(new URL('.', import.meta.url));
const DT = 1 / 60;

// Build the driver (needs libdl on Linux; on macOS dlopen is in libc).
const soExt = process.platform === 'darwin' ? 'dylib' : 'so';
const soPath = `${dir}fmu-build/binaries/linux64/fdm-uav.so`;
// On macOS the Makefile still emits ".so"; dlopen loads it fine as a bundle.
if (!existsSync(soPath)) {
  console.error('build the FMU shared lib first: make -C native fmu-so');
  process.exit(1);
}
const driver = `${dir}fmi-driver`;
try {
  execFileSync('cc', ['-std=c99', '-O2', '-o', driver, `${dir}fmi-driver.c`], { stdio: 'pipe' });
} catch (e) {
  console.error('driver build failed:', e.stderr?.toString() || e.message);
  process.exit(1);
}

// JS golden trajectory (turbulence off, matching the driver's command law).
function jsGolden(scase, seconds) {
  const airborne = scase !== 'ground_roll';
  // Match the FMU's fdm_ground_state exactly (runway threshold pos[2] = 350),
  // not the golden generator's z=0 shortcut — otherwise a pure coordinate offset
  // masquerades as a physics mismatch.
  let s = airborne ? initialState() : groundState();
  let w = createWind(2);
  const P = { ...defaultParams(), WND_N_MS: 0, WND_E_MS: 0, WND_TRB: 0 };
  const TRIM_ELEV = TRIM.de / AC.maxDef;
  for (let i = 0; i < seconds * 60; i++) {
    const sw = stepWind(w, s, P, DT);
    w = sw.wind;
    const cmds = scase === 'ground_roll'
      ? { aileron: 0, elevator: 0, rudder: 0, throttle: 1 }
      : { aileron: 0, elevator: TRIM_ELEV, rudder: 0, throttle: TRIM.dt };
    s = stepAircraft(s, cmds, DT, sw.windWorld);
  }
  const ad = airData(s.quat, s.vel, [0, 0, 0]);
  return {
    posN: -s.pos[2], posE: s.pos[0], posD: -s.pos[1], va: ad.Va,
  };
}

const cases = [
  { name: 'trim_calm', seconds: 30, posTol: 1e-3 },
  { name: 'ground_roll', seconds: 8, posTol: 1e-3 },
];

let failures = 0;
for (const c of cases) {
  const out = JSON.parse(execFileSync(driver, [soPath, c.name, String(c.seconds)]).toString());
  const ref = jsGolden(c.name, c.seconds);
  const dPos = Math.max(
    Math.abs(out.posN - ref.posN), Math.abs(out.posE - ref.posE), Math.abs(out.posD - ref.posD)
  );
  const dVa = Math.abs(out.va - ref.va);
  const ok = dPos < c.posTol && dVa < 1e-2;
  console.log(`${ok ? 'ok ' : 'FAIL'} FMU ${c.name} ${c.seconds}s — pos Δ ${dPos.toExponential(2)} m, Va Δ ${dVa.toExponential(2)}`);
  if (!ok) { failures++; console.log(`   fmu=${JSON.stringify(out)}\n   ref=${JSON.stringify(ref)}`); }
}
console.log(failures ? `FMU CHECK: ${failures} FAILED` : 'FMU CHECK: PASS');
process.exit(failures ? 1 : 0);
