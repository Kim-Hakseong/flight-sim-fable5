# Log — flight-sim-fable5

> Append one entry per work loop, newest at the bottom. Format is fixed (CLAUDE.md §5).
> No log, no work. After each entry: commit + push (once a remote is configured).

---

## 2026-07-11 — M-1: project scaffold (spec only)

**Status**: GREEN
**Files changed**: CLAUDE.md, PRD.md, README.md, Log.md, .claude/settings.json, .gitignore
**Tests**: n/a (no code yet)
**Decisions**:
- Fresh, GCS-first rebuild seeded from the `../flight-sim2` design + its lessons. No code
  copied — the .md spec is the deliverable; the model builds from it.
- Stack fixed by PRD: vanilla ES modules + Three.js r128 CDN, Node bridge, no build step.
- Model set to `claude-fable-5` in `.claude/settings.json`.
**Next**:
- **M0 — Lean baseline**: scaffold (package.json, index.html, src/main.js), a flying box
  in a Three.js scene, fixed-step loop + deterministic `window.__advance`, keyboard
  control, first unit test + console-0 check. Ship GREEN, log, commit `M0: baseline`.
**Notes**:
- Read CLAUDE.md then PRD.md before writing any code. PRD wins on conflict.

## 2026-07-11 — M0: lean baseline

**Status**: GREEN
**Files changed**: package.json, index.html, src/physics.js, src/main.js, tests/physics.test.mjs, tests/browser-check.mjs
**Tests**: unit 11/11 pass · console 0 ✓ · gcs n/a (M1) · determinism ✓ (browser `__reset`+`__advance(10)` twice → bit-identical)
**Decisions**:
- Physics is deliberately simple for M0 (rate-command attitude, quadratic lift/drag, no stall/moments) — 6-DOF-ready state shape (pos/vel/quat/body-rates) so later milestones refine forces, not structure.
- No PRNG module yet: M0 has zero randomness; mulberry32 lands with the sensor model (M5) per "create modules as milestones require".
- Browser gate is raw CDP over Node's built-in WebSocket (zero deps); needs `--enable-unsafe-swiftshader` for headless WebGL.
- First `__advance` call permanently switches off wall-clock stepping (manual/HILS mode) so tests can't race the rAF loop.
**Next**:
- **M1 — Bridge + telemetry**: bridge/server.mjs + bridge/mavlink.mjs (HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, GPS_RAW_INT), sim→geodetic around a home point, gcs-loop-check.
**Notes**:
- Controls: ↑/↓ pitch, ←/→ roll, A/D yaw, W/S throttle, R reset — all via `e.code` (IME-safe).
- No git remote configured yet, so commit only (push resumes once a remote exists).

## 2026-07-11 — M1: bridge + telemetry

**Status**: GREEN
**Files changed**: bridge/mavlink.mjs, bridge/server.mjs, src/telemetry.js, src/main.js, tests/mavlink.test.mjs, tests/telemetry.test.mjs, tests/gcs-loop-check.mjs, package.json
**Tests**: unit 22/22 pass · console 0 ✓ · gcs-loop-check PASS (17/17) · determinism ✓
**Decisions**:
- MAVLink codec is table-driven (wire-order field specs + generic pack/unpack) so M2+ messages are one table entry + tests, not new code paths.
- HEARTBEAT claims MAV_AUTOPILOT_ARDUPILOTMEGA / fixed-wing so QGC uses the ArduPlane mode map — matches the M2 plan (MANUAL/AUTO/GUIDED custom modes).
- Telemetry POST only activates when the page is served by the bridge (probed via `x-flight-bridge` response header) — static serves stay console-clean, no 404 noise.
- Bridge remembers the GCS's actual UDP source address once it talks to us and replies there (QGC ephemeral-port pattern); defaults to 127.0.0.1:14550.
- Home point: 37.4449 N 126.4656 E (인천 부근), north = −Z, east = +X.
**Next**:
- **M2 — Command loop**: decode COMMAND_LONG/SET_MODE, ARM/DISARM + mode via SSE /commands → sim authoritative, COMMAND_ACK + base_mode reflect it.
**Notes**:
- QGroundControl is not installed on this machine — the node-level gcs-loop-check is the gate; install QGC and run `npm run bridge` to see the aircraft move on the map (auto-connects on UDP 14550).

## 2026-07-11 — M2: command loop

**Status**: GREEN
**Files changed**: bridge/mavlink.mjs, bridge/server.mjs, src/autopilot.js, src/missionLink.js, src/physics.js, src/telemetry.js, src/main.js, tests/autopilot.test.mjs, tests/mavlink.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 30/30 pass · console 0 ✓ · gcs-loop-check PASS (23/23, incl. ACK/SSE/replay) · determinism ✓ (browser + full-RTL-flight bit-identical test)
**Decisions**:
- Physics upgraded: lift is now AoA-dependent (CL0 + CLA·α, soft-stall clamp) — the M0 speed-only lift made pitch useless for climb control, so no autopilot could converge. Closed-loop-tested before committing.
- Turns are flown coordinated (yaw rate = g·tanφ/V) because the physics has no weathervane moment yet; the nose tracks the velocity vector.
- Mode numbers are ArduPlane's (MANUAL 0, AUTO 10, RTL 11, LOITER 12, TAKEOFF 13, GUIDED 15) since the heartbeat claims autopilot=3 — QGC shows proper mode names.
- ACK policy: bridge ACKs supported commands immediately (ACCEPTED) and forwards over SSE; the sim stays authoritative — arm/mode flow back via telemetry into HEARTBEAT (base_mode ARMED bit, system_status ACTIVE/STANDBY).
- NAV_LAND is a landing sub-state on top of any mode (ArduPlane has no plain LAND mode); touchdown + slow → auto-disarm.
- `window.__command(cmd)` added: tests/HILS drive the exact path GCS commands take.
**Next**:
- **M3 — GUIDED + missions**: mission upload handshake (MISSION_COUNT → REQUEST_INT → ITEM_INT → ACK), waypoint sequencing, DO_REPOSITION go-to, MISSION_CURRENT / MISSION_ITEM_REACHED.
**Notes**:
- QGC ARM/DISARM now cuts/restores the engine; TAKEOFF/LAND/RTL fly real (simple) profiles. Verified end-to-end at packet level; visual QGC pass still pending (QGC not installed here).

## 2026-07-11 — M3: GUIDED + missions

**Status**: GREEN
**Files changed**: bridge/mavlink.mjs, bridge/server.mjs, src/missions.js, src/autopilot.js, src/telemetry.js, src/main.js, tests/missions.test.mjs, tests/mavlink.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 36/36 pass · console 0 ✓ · gcs-loop-check PASS (33/33, incl. upload handshake, download-back, DO_REPOSITION, MISSION_CURRENT/ITEM_REACHED) · determinism ✓
**Decisions**:
- Mission protocol lives in the bridge (COUNT→REQUEST_INT→ITEM_INT→ACK with a 700 ms retry / 8-try give-up); the accepted plan ships to the sim as ONE SSE event — the sim never sees handshake timing, so it stays deterministic.
- Bridge also answers the download side (REQUEST_LIST / REQUEST(_INT)) from the stored plan, since QGC re-reads a plan to verify the upload.
- Loiter radius = 300 m: first attempt (150 m) was inside the achievable turn radius (V²/g·tanφ ≈ 280 m) — the orbit railed the bank, bled lift, and spiralled in. Guided loiter chases a carrot on the circle (radial swung 80° ahead), which settles to a stable ~230 m orbit.
- DO_REPOSITION accepted in both COMMAND_INT (QGC's form) and COMMAND_LONG; MAV_CMD_MISSION_START → AUTO.
- Unsupported mission items are sequenced through (not flown) so a plan never stalls; LAND/RTL items become the M2 landing/RTL behaviors.
**Next**:
- **M4 — Parameters**: PARAM_REQUEST_LIST / PARAM_REQUEST_READ / PARAM_SET for autopilot gains + sensor sigmas, shared param table, range clamping.
**Notes**:
- QGC flow now works end-to-end at packet level: upload a plan → slide-to-start (MISSION_START) → it flies waypoints and reports progress; "Go to location" reroutes into a stable orbit.

## 2026-07-11 — M4: parameters

**Status**: GREEN
**Files changed**: src/params.js, src/autopilot.js, src/main.js, bridge/mavlink.mjs, bridge/server.mjs, tests/params.test.mjs, tests/mavlink.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 40/40 pass · console 0 ✓ · gcs-loop-check PASS (38/38, incl. list/read/set/clamp) · determinism ✓
**Decisions**:
- src/params.js is the shared table (defs + defaults + clamp), imported by BOTH the browser sim and the node bridge — one source of truth, per PRD.
- 8 autopilot gains + 4 sensor sigmas (sigmas consumed in M5). All REAL32.
- Set flow: bridge clamps → echoes PARAM_VALUE (closes QGC's set cycle) → SSE 'param' → sim re-tunes live. Unknown ids are ignored (QGC times out its widget, matching real AP behavior).
- Params survive `__reset` (vehicles persist params); they're in `__state` so determinism checks see them.
- char16 codec type added for param_id (NUL-padded, 16-char exact fits, no terminator on wire).
**Next**:
- **M5 — HILS faults visible in the GCS**: sensor error model (scale/bias/noise/lag) + `injectFault` (freeze/dropout/bias), SYS_STATUS sensor-health bits + STATUSTEXT on fault edges.
**Notes**:
- In QGC's param screen the vehicle now lists AP_*/SNS_* params; editing AP_BANK_MAX etc. visibly changes turn behavior mid-flight.

## 2026-07-11 — M5: HILS faults visible in the GCS

**Status**: GREEN
**Files changed**: src/prng.js, src/sensors.js, src/telemetry.js, src/main.js, bridge/mavlink.mjs, bridge/server.mjs, tests/sensors.test.mjs, tests/mavlink.test.mjs, tests/gcs-loop-check.mjs, tests/browser-check.mjs
**Tests**: unit 47/47 pass · console 0 ✓ · gcs-loop-check PASS (42/42, incl. health bits + STATUSTEXT edges) · determinism ✓ (incl. NEW faulted-run reproducibility in browser)
**Decisions**:
- PRNG is pure-functional mulberry32 ([value, nextState] pairs) — sensor state threads it explicitly, so a faulted run replays bit-identically (checked in the browser gate).
- Every sensor draws its noise even while faulted, so injecting one fault cannot shift the other sensors' streams (unit-tested).
- GPS_RAW_INT + VFR_HUD altimeter feed from the SENSED values (fault-visible in QGC); GLOBAL_POSITION_INT stays the fused/true estimate until the M6 estimator.
- Fault → health bit drops immediately (sim self-reports; estimator-based FDE can refine in M6); dropout holds the last GPS fix with fix_type=1/sats=0 like a real receiver.
- `window.injectFault(sensor, type, opts)` / `window.clearFault(sensor)` — the HILS surface CLAUDE.md pins down.
**Next**:
- **M6 — Telemetry completeness**: deterministic battery drain in SYS_STATUS, EKF_STATUS_REPORT from estimator health, lifecycle STATUSTEXT (arm/mode/nav), mission progress rounding-out.
**Notes**:
- Demo: browser console → `injectFault('gps','bias',{bias:200})` → QGC map track jumps 200 m + GPS goes red + "GPS fault: bias" toast; `clearFault('gps')` recovers.

## 2026-07-11 — M6: telemetry completeness

**Status**: GREEN
**Files changed**: src/battery.js, src/estimator.js, src/telemetry.js, src/main.js, bridge/mavlink.mjs, bridge/server.mjs, tests/estimator.test.mjs, tests/mavlink.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 52/52 pass · console 0 ✓ · gcs-loop-check PASS (47/47) · determinism ✓ (battery/est in `__state`)
**Decisions**:
- Battery: I = base + max·throttle², dt-integrated only (constitution §0.5); voltage = SoC ramp − load sag; ≈ 29 min endurance at cruise. Disarmed → avionics-floor draw.
- Estimator: gated position filter over GPS+baro with 5σ innovation rejection (FDE). A 500 m GPS bias is rejected — the estimate coasts, variance grows, POS_HORIZ_ABS drops (unit-tested); QGC's EKF widget goes red on a nav fault.
- GLOBAL_POSITION_INT now carries the ESTIMATOR output (the map shows fused nav, incl. coasting during faults); GPS_RAW_INT/VFR_HUD stay raw-sensor; attitude remains truth until an INS lands.
- EKF_STATUS_REPORT is ardupilotmega id 193 (we already claim autopilot=3, so QGC listens).
- Lifecycle STATUSTEXT edges in the bridge: Arming/Disarming motors, "Mode changed to X", "Reached waypoint #N".
- Autopilot still flies TRUE state, not the estimate — flagged as the natural next refinement if full HILS closure is wanted.
**Next**:
- All PRD milestones (M0–M6) GREEN. Candidates: configure a git remote (auto-push per §0.8), visual QGC pass, autopilot-on-estimate, engineering console (src/engineering.js).
**Notes**:
- QGC now shows: depleting battery gauge + current, EKF health that reacts to `injectFault('gps',…)`, and toast messages for arm/mode/waypoint/fault events.

## 2026-07-11 — infra: GitHub remote + push

**Status**: GREEN
**Files changed**: Log.md (this entry)
**Tests**: n/a (no code change)
**Decisions**:
- Public repo `Kim-Hakseong/flight-sim-fable5` created via gh; `origin` wired, `main` pushed (M-1…M6 all up).
**Next**:
- Auto-push per §0.8 is now live: every loop commits AND pushes. Candidates from M6: visual QGC pass, autopilot-on-estimate, engineering console.
**Notes**:
- https://github.com/Kim-Hakseong/flight-sim-fable5

## 2026-07-13 — infra: GitHub Pages deploy

**Status**: GREEN
**Files changed**: src/telemetry.js, Log.md
**Tests**: unit 52/52 pass · console 0 ✓ · gcs-loop-check PASS · determinism ✓
**Decisions**:
- Pages enabled (main branch, root) → https://kim-hakseong.github.io/flight-sim-fable5/
- Bridge probe fetch('/') → fetch('./'): under a sub-path host the domain root 404s; relative keeps every serving mode (Pages / python / bridge) clean.
**Next**:
- Verify the live Pages build headlessly once it finishes.
**Notes**:
- Pages serves the STANDALONE sim (keyboard flight + HILS console surface). The MAVLink/QGC loop still needs the local bridge (`npm run bridge`) — UDP can't ride on Pages.

## 2026-07-13 — infra: live Pages verified

**Status**: GREEN
**Files changed**: tests/browser-check.mjs
**Tests**: unit 52/52 · browser check PASS locally AND against the live Pages URL (console 0, `__advance` + fault-run reproducible)
**Decisions**:
- browser-check gained a `PAGE_URL` override so the same gate smoke-tests the deployed site: `PAGE_URL=https://kim-hakseong.github.io/flight-sim-fable5/ npm run check:browser`.
**Next**:
- (idle) Candidates unchanged: visual QGC pass, autopilot-on-estimate, engineering console.
**Notes**:
- Live: https://kim-hakseong.github.io/flight-sim-fable5/

## 2026-07-13 — M7: high-fidelity flight model (Simulink/UAV-grade)

**Status**: GREEN
**Files changed**: PRD.md (M7/M8 added), src/physics.js (rewrite), src/autopilot.js (rewrite), src/params.js, src/main.js, tests/physics.test.mjs, tests/autopilot.test.mjs, tests/{params,telemetry,estimator}.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 54/54 pass · console 0 ✓ · gcs-loop-check PASS · determinism ✓
**Decisions**:
- Full rigid-body 6-DOF: forces AND moments, diagonal inertia (Jxz≈0.12 neglected), Beard & McLain-style stability derivatives for an Aerosonde-class 13.5 kg UAV; aero math runs in standard FRD axes with a boundary conversion (toFRD/fromFRD) to keep signs auditable.
- Control surfaces δa/δe/δr/δt behind first-order actuators (τ=0.05 s, ±25°, throttle τ=0.4 s); prop model T=½ρSprop((kΩδt)²−Va²), kMotor scaled to 50 for a sane cruise δt≈0.63; ISA density.
- Trim (α=3.19°, δe=−0.089, δt=0.627) Newton-solved offline, hardcoded + regression-tested; open-loop modes verified realistic (short-period/phugoid damp, dutch roll damps, spiral slowly diverges).
- Autopilot = successive loop closure onto surfaces (bank→aileron+p-damping, pitch→elevator+q-damping, coordinated-turn rudder tracking r=g·tanφ/V, alt→climb→θ, airspeed→throttle). Turns now fly at β≈0.
- MANUAL gets SAS damping (bare airframe + keyboard = unflyable). Param table: 12 AP gains/targets (KP/KD per axis, VA_TRIM…).
- Landing: powered approach AND powered flare (early throttle cut bled Va → mush, measured); two-stage sink profile −3.5/−1.5/−0.8, descent starts at ~11·alt from home. Touchdown sink −2.3 m/s, stops ~300 m from home, from any approach direction.
**Next**:
- **M8 — engineering visuals**: procedural UAV with moving control surfaces + prop, runway + seeded terrain, shadows, HUD (Va/α/β/surfaces), screenshot gate.
**Notes**:
- Loiter radius 250 m (turn radius at 30 m/s / 30° bank ≈ 160 m).

## 2026-07-13 — M8: engineering visuals

**Status**: GREEN
**Files changed**: src/scene.js (new), src/main.js, index.html-, tests/browser-check.mjs, .gitignore
**Tests**: unit 54/54 · console 0 ✓ · gcs PASS · determinism ✓ · screenshot artifact ✓ (tests/artifacts/sim.png, gitignored)
**Decisions**:
- Scene moved to src/scene.js, RENDER-ONLY: visuals are a pure function of sim state (prop spin = f(simTime, δt), not wall clock) — determinism untouched.
- Procedural true-scale (b=2.9 m) UAV: hinged ailerons/elevator/rudder are separate meshes rotating about their hinge lines, mirroring the ACTUATOR states (×1.6 visual gain) — the HILS bench can literally watch its commands move the surfaces.
- Runway (900 m, centerline + thresholds) at home aligned north; seeded tree scatter + ground texture from our PRNG (fixed WORLD_SEED) — same world every boot.
- Sun + PCFSoft shadows with a tight frustum that follows the aircraft; smoothed no-roll chase cam (snaps on teleport/reset).
- HUD is now an air-data strip: Va/α/β, VS, and live δa/δe/δr in degrees.
- browser-check captures a screenshot artifact per run (UI gate, CLAUDE.md §0.4).
**Next**:
- Candidates: engineering console (charts/state vector), autopilot-on-estimate, visual QGC pass.
**Notes**:
- Live after push: https://kim-hakseong.github.io/flight-sim-fable5/

## 2026-07-13 — M9: wind + Dryden turbulence

**Status**: GREEN
**Files changed**: PRD.md (M9–M11), src/wind.js (new), src/physics.js, src/autopilot.js, src/params.js, src/telemetry.js, src/main.js, tests/wind.test.mjs
**Tests**: unit 60/60 · console 0 ✓ · gcs PASS · determinism ✓
**Decisions**:
- Dryden as per-body-axis Gauss–Markov (B&M-style Lu/Lv/Lw = 200/200/50 m, σ = 1.06/1.06/0.7·WND_TRB), seeded + threaded like the sensor PRNG; stationary sigma unit-verified (±15%).
- Aero now runs on air-relative velocity; airspeed ≠ groundspeed. Steady wind (WND_N/E_MS) + intensity (WND_TRB, default 1 = light) are PARAMs — QGC can gust the vehicle live.
- holdControls gained a measured-airspeed input: found experimentally that without it the AP holds inertial speed and a 10 m/s headwind settles at Va 35.8 (realistic no-pitot behavior, but wrong target). Main feeds true Va for now; M11 swaps in a faultable pitot.
- Verified: crosswind holds heading while the track crabs (>200 m/min at 8 m/s), headwind holds Va 30 with groundspeed 20, heavy turbulence (WND_TRB 2) stays in a ±10 m alt band.
**Next**:
- **M10 — engineering console** (src/engineering.js).
**Notes**:
- Turbulence is on by default (light) — the live page now visibly "breathes".

## 2026-07-13 — M10: engineering console (HILS bench)

**Status**: GREEN
**Files changed**: src/engineering.js (new), src/main.js, index.html, tests/browser-check.mjs
**Tests**: unit 60/60 · console 0 ✓ · gcs PASS · determinism ✓ · DOM gate ✓ (3 charts, 20 bench buttons) · screenshot ✓
**Decisions**:
- KeyE toggles the panel: live state vector (pos/vel/pqr/surfaces/α/β/Va), estimator-vs-truth errors + variances + EKF flag bits, per-sensor fault bench (freeze/drop/bias/clear buttons driving the SAME window.injectFault surface as tests), wind + battery, and 30 s strip charts (alt truth-vs-estimate overlay, Va, est error).
- Chart samples are recorded on SIM steps at 10 Hz (deterministic data); drawing happens at render rate. Panel is render-only — physics untouched, `__advance` reproducibility unchanged.
- browser-check gained the DOM gate (panel exists/toggles/has charts+bench) and the screenshot artifact now ships with the console open.
**Next**:
- **M11 — autopilot flies the estimate** (+ pitot sensor).
**Notes**:
- Fault demo is now one click: E → gps [drop] → watch est errH chart climb while QGC's GPS goes red.

## 2026-07-13 — M11: autopilot flies the estimate (HILS loop closed)

**Status**: GREEN
**Files changed**: src/sensors.js (pitot), src/estimator.js (velocity + FDE rework), src/autopilot.js (WoW), src/main.js, src/params.js, src/telemetry.js, bridge/server.mjs, tests/nav-loop.test.mjs (new), tests/gcs-loop-check.mjs
**Tests**: unit 64/64 · console 0 ✓ · gcs PASS · determinism ✓ (full closed loop bit-identical)
**Decisions**:
- Guidance/alt/speed loops now consume the ESTIMATOR nav + a new faultable pitot (diff-pressure bit 16; SENSORS_PRESENT 47→63). Attitude/rates stay truth — an attitude estimator is future work, stated openly.
- Touchdown is a weight-on-wheels discrete from truth (est altitude too noisy to declare ground), like the real switch.
- Estimator rework, found via the new full-loop tests: (1) velocity from 1-step GPS deltas amplifies noise by 1/dt (±tens of m/s garbage) → replaced with 2 s/1 s long-baseline differences; (2) after a long outage the recovery innovation exceeded the gate and the filter wedged → FDE backstop re-anchors after 8 s of continuous rejection.
- New gate tests/nav-loop.test.mjs runs the WHOLE stack (wind→physics→sensors→estimator→AP): 3-wp mission in turbulence, 12 s GPS dropout mid-leg (coast err < 80 m, mission completes), RTL to WoW disarm, bit-exact rerun.
**Next**:
- M9–M11 done. Candidates: attitude estimator (gyro+accel+mag complementary/EKF), QGC visual pass, ground-roll takeoff from the runway.
**Notes**:
- The fault story is now end-to-end: inject GPS dropout → estimator coasts (EKF variance climbs in QGC + console chart) → the aircraft itself drifts → recovery re-converges. That's the HILS purpose of this whole build.

## 2026-07-13 — M12: attitude estimator (Mahony + gyro bias)

**Status**: GREEN
**Files changed**: PRD.md, src/estimator.js (attitude filter), src/sensors.js (true specific-force accel), src/main.js, src/telemetry.js, src/engineering.js, tests/attitude.test.mjs (new), tests/nav-loop.test.mjs
**Tests**: unit 70/70 · console 0 ✓ · gcs PASS · determinism ✓ (full estimated-state loop bit-identical)
**Decisions**:
- Accelerometer model upgraded from a gravity placeholder to TRUE specific force ((F_aero+prop)/m in body axes) — the turn-contamination failure mode is now physically real.
- Mahony filter: gyro integration + accel tilt correction + mag heading correction; the correction integral IS the gyro-bias estimate (converges to an injected 0.05 rad/s bias within ±0.015, unit-tested).
- Naive accel trust in turns fed back through the estimated-attitude control loop and spiralled (62° peak error, measured) → centripetal compensation ω×v with pitot speed, plus a ‖f‖≈g norm band (tight 8% band when gyro is out and compensation is impossible). Turn error now peaks < 8°, re-converges < 3°.
- Control path is now 100% estimated: nav pos/vel (M11), Mahony attitude, bias-corrected rates (SAS included), pitot Va. Truth remains ONLY in the WoW discrete. ATTITUDE downlink reports the estimated attitude — QGC shows what the vehicle believes, like a real one.
- Console NAV section: attitude error (deg) + live gyro-bias estimate.
**Next**:
- Candidates: ground-roll takeoff from the runway, QGC visual pass, EKF_STATUS attitude flag from lpErr.
**Notes**:
- Fault demos now cover all 6 sensors end-to-end: gyro bias → transient wobble then absorbed; mag bias 30° → the aircraft actually flies 30° off heading; gyro dropout → SAS goes blind + attitude coasts on accel/mag.
