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

## 2026-07-13 — M13: ground-roll takeoff

**Status**: GREEN
**Files changed**: PRD.md, src/physics.js (ground model, thrust cap, groundState), src/autopilot.js (ground-roll phase), src/main.js (ground boot, KeyT), index.html, tests/takeoff.test.mjs (new), tests/nav-loop.test.mjs
**Tests**: unit 75/75 · console 0 ✓ · gcs PASS · determinism ✓ · screenshot ✓ (parked on the runway)
**Decisions**:
- Ground model: rolling resistance μ=0.03 + auto-brake μ=0.22 at idle throttle; gear "springs" hold roll level and stop the nose digging in, while pitch stays aero-controllable for rotation. Rest state is exactly still (unit-tested).
- Prop static thrust capped at 60 N (B&M model gives ~320 N ⇒ T/W 2.4, silly for a ground roll); cruise/climb unaffected (needs ~11 N).
- TAKEOFF ground phase: full power, rudder centerline steering, wings level, rotate at Vr=20 m/s (pitot) → existing climb-out. Measured: liftoff after 73 m / 6.1 s, centerline dev 0.0 m, brake rollout 63 m.
- Sim now BOOTS DISARMED at the runway threshold (real vehicle lifecycle; supersedes M2's airborne-armed boot). KeyT = arm + auto-takeoff for keyboard users; `initialState()` (airborne trim) remains the test fixture.
- Full-stack gate: cold ground takeoff on 100% estimated state (nav-loop test).
**Next**:
- Candidates: QGC visual pass (needs local bridge + QGC), EKF_STATUS attitude flag, mission-item takeoff/land at specified runway points.
**Notes**:
- Real-vehicle GCS flow now works end to end: QGC ARM → TAKEOFF command → ground roll → rotate → climb → GUIDED/mission → RTL → land → WoW disarm.

## 2026-07-13 — UX: keyboard arm/manual keys

**Status**: GREEN
**Files changed**: src/main.js, index.html
**Tests**: unit 75/75 · console 0 ✓ · determinism ✓
**Decisions**:
- User feedback: W/S "didn't work" — the M13 disarmed boot cut the engine and the keyboard had no way to ARM. Space now toggles arm/disarm (same applyCommand path as the GCS), KeyM drops back to MANUAL from any AP mode.
**Next**:
- (idle)
**Notes**:
- Manual takeoff flow: Space(arm) → W hold → at ~20 m/s pull ↑.

## 2026-07-13 — M14+M15: HILS scenario runner + CI

**Status**: GREEN
**Files changed**: PRD.md (M14–M16), src/vehicle.js (new), src/hils.js (new), src/main.js (rewritten as a thin shell), tests/hils.test.mjs + tests/hils-run.mjs (new), tests/nav-loop.test.mjs (rewritten on vehicle.js), package.json, .github/workflows/ci.yml
**Tests**: unit 79/79 · console 0 ✓ · gcs PASS · determinism ✓ · HILS bench 5/5 PASS
**Decisions**:
- The whole vehicle extracted to src/vehicle.js — browser sim, node tests, and the scenario runner share ONE implementation (the nav-loop test previously hand-mirrored main.js; that duplication is gone). main.js is now a shell: keyboard, render, HUD, bridge I/O.
- Scenario format: {boot, seed, params, events:[{t, command|fault|clear}], checks:[band|final|reach]} → deterministic run → pass/fail report with worst-values. `reach` checks take a `from` window (found the hard way: "lands by t=320" passed on the PRE-takeoff ground samples).
- Built-in bench (5): gps-dropout-recovery, gyro-bias-absorption, mag-fault-flyable, heavy-turbulence-goto, full-sortie (cold boot → takeoff → mission → RTL → land → auto-disarm, 360 s sim in ~70 ms).
- Surfaces: `window.__hils.list/run/runAll` (fresh vehicle per run — live sim untouched), `npm run hils [name]` CLI.
- CI (GitHub Actions): unit + gcs-loop + HILS bench + headless-Chrome browser gate on every push/PR; screenshot uploaded as an artifact.
**Next**:
- **M16 — actuator faults** (servo jam/floating/slow).
**Notes**:
- CI's browser step uses the runner's preinstalled Chrome via CHROME_BIN.

## 2026-07-13 — M16: actuator faults + rudder roll-assist

**Status**: GREEN
**Files changed**: src/physics.js (stepActuators faults), src/vehicle.js, src/autopilot.js (AP_RUD_ROLL), src/params.js, src/hils.js (2 servo scenarios), src/main.js, src/engineering.js (servo bench), tests/servo.test.mjs (new)
**Tests**: unit 85/85 · console 0 ✓ · gcs PASS · HILS bench 7/7 · determinism ✓ · CI green on GitHub (all steps incl. headless-Chrome gate)
**Decisions**:
- Servo faults per channel (δa/δe/δr/δt): jam (hold position), floating (streams to aero-neutral; throttle dies), slow (τ × factor). Threaded physics→vehicle→console bench→STATUSTEXT (servo_da etc. on the same edge channel)→HILS events.
- Found via the bench: an aileron jammed at +1.3° (mid-turbulence-correction) is UNRECOVERABLE with the old AP — constant roll moment, spiral in 10 s. Real failure mode, real fix: rudder roll-assist through the dihedral path (yaw→β→Clβ), gain AP_RUD_ROLL=0.35 (QGC-tunable). Same jam now holds ≤16° roll and full altitude; assist also mildly helps normal turns.
- New scenarios: elevator-slow-survivable, aileron-jam-at-trim (both PASS with the assist).
**Next**:
- A→B→D done. Remaining candidates: wind estimation, QGC virtual joystick, MAVLink v2, sim-as-plant external FC interface.
**Notes**:
- The bench earned its keep on day one: it exposed a genuine unrecoverable failure mode and validated the control-law fix, all deterministic and CI-gated.

## 2026-07-13 — M17+M18+M19: wind estimation, GCS joystick, MAVLink v2

**Status**: GREEN
**Files changed**: PRD.md (M17–M20), src/estimator.js (wind est), src/autopilot.js (crab), src/vehicle.js, src/telemetry.js, bridge/mavlink.mjs (v2 framing, WIND, MANUAL_CONTROL), bridge/server.mjs, tests/wind-est.test.mjs (new), tests/mavlink.test.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 89/89 · gcs PASS · HILS bench 7/7 · console 0 ✓ · determinism ✓
**Decisions**:
- Wind estimate = nav ground-velocity − pitot·(estimated nose), τ=8 s low-pass, gated on airflow ≥ 8 m/s. Converges within ±1.5 m/s (unit-tested).
- Guidance headings are now COURSES: holdControls crabs into the estimated crosswind (asin(w_cross/Va)); an 8 m/s crosswind hold drifts < 2 m/s where it used to crab away at ~4 m/s. Old no-estimate behavior remains when nav state carries no windEst.
- WIND (ardupilotmega 168) downlinked — QGC shows the wind arrow from the vehicle's own estimate.
- MANUAL_CONTROL (QGC virtual joystick) → SSE 'stick' (log-quiet) → MANUAL mode, freshness-gated at 1 s with keyboard fallback; QGC x+ (stick fwd) maps to nose-down.
- MAVLink v2: 0xFD framing, 24-bit msgid, trailing-zero payload truncation (restored on decode), signed packets skipped; the bridge replies in whichever framing the GCS last spoke. v1 default stays.
**Next**:
- **M20 — sim-as-plant** (headless vehicle in node; external controller closes the loop).

## 2026-07-13 — M20: sim-as-plant (external FC over UDP lockstep)

**Status**: GREEN
**Files changed**: bridge/plant.mjs (new), src/vehicle.js (direct-controls option), tests/plant-check.mjs (new), package.json, .github/workflows/ci.yml
**Tests**: unit 89/89 · gcs PASS · HILS bench 7/7 · plant check PASS · console 0 ✓ · determinism ✓
**Decisions**:
- `npm run plant`: the vehicle hosted headlessly in node; an EXTERNAL controller closes the loop over UDP JSON lockstep — controls in, SENSOR READINGS out (never truth; WoW is the only discrete). The sim advances only on ctl packets: controller-paced, deterministic, no wall clock. reset/fault message types round out the bench protocol; QGC telemetry keeps flowing.
- vehicleStep gained a `direct` argument (raw surfaces, AP bypassed) — the plant is the same vehicle implementation as everything else, not a fork.
- plant-check is a REAL external FC: its own estimator instances fed only by the UDP sensor stream, holdControls as its law. It holds altitude, flies a 30 s lockstep sortie in ~100 ms, reruns bit-identically, and sees injected faults purely through its sensors.
- Fixed en route: fault opts spread let the wire field `type:'fault'` clobber the fault type (silent wrong-fault injection).
**Next**:
- Full backlog cleared (A–I). Natural follow-ons: ArduPilot SITL JSON adapter for the plant port, mission-item extensions, QGC visual pass.
**Notes**:
- The plant protocol is documented in bridge/plant.mjs header; an ArduPilot JSON-backend adapter would slot in front of it.

## 2026-07-13 — M21: ArduPilot SITL JSON adapter

**Status**: GREEN
**Files changed**: PRD.md, bridge/sitl.mjs (new), tests/sitl-check.mjs (new), package.json, .github/workflows/ci.yml
**Tests**: unit 89/89 · gcs PASS · HILS 7/7 · plant PASS · SITL check PASS (9/9) · browser PASS
**Decisions**:
- `npm run sitl` speaks ArduPilot's SITL JSON backend on UDP 9002: binary servo packet (magic 18458, frame_rate, frame_count, pwm[16]) in → one-line JSON truth state (timestamp/imu FRD/position NED/attitude/velocity/airspeed) out, strict lockstep. Duplicate frame_count = resend without stepping; frame_count going backwards = ArduPilot restart → physics reset.
- Truth (not our sensor model) is returned — ArduPilot layers its own sensor sim, per the JSON backend contract. Physics-only stepping (no estimator stack) since AP brings its own.
- On the ground the accelerometer substitutes the support reaction (−g on FRD z): our ground contact is a kinematic clamp with no modelled normal force, and without this AP's IMU would read free-fall while parked.
- Servo map (ArduPlane defaults 1-ail/2-elev/3-thr/4-rud); AP elevator+/rudder+ invert onto ours. Signs locked by the fake-AP driver: elevator+ → measured nose-up, aileron+ → measured p>0.
- Wind via env (WND_N_MS/WND_E_MS/WND_TRB), calm default.
**Next**:
- Real-binary smoke test: `sim_vehicle.py -v ArduPlane --model JSON` against `npm run sitl` (needs ArduPilot locally — user machine).
**Notes**:
- Usage: terminal 1 `npm run sitl`, terminal 2 ArduPlane `--model JSON`; QGC connects to ArduPilot itself (TCP 5760/UDP 14550), so the GCS shows a REAL autopilot flying our aero model.

## 2026-07-13 — M22: cinematic F-16 visuals

**Status**: GREEN
**Files changed**: PRD.md, src/scene.js (F-16 airframe, sky dome, lighting), src/main.js (ACES tone mapping)
**Tests**: unit 89/89 · browser PASS (console 0, determinism, DOM) · screenshot ✓
**Decisions**:
- Procedural F-16-style airframe at true ~15 m scale: lathe fuselage + radome + bubble canopy + intake, extruded swept wings/fin, hinged flaperons + rudder, ALL-MOVING stabilators driven by the elevator actuator, throttle-driven afterburner flame (additive, deterministic flicker from sim time). Chase camera pulled back to jet scale.
- Cinematic pass: ACES filmic tone mapping + sRGB out, gradient sky dome shader, warm key light, horizon haze fog.
- RENDER-ONLY by design: the flight model remains the validated small-UAV dynamics (a true F-16 model = future milestone); every gate stays green untouched.
**Next**:
- Optional: F-16-class flight model (mass/inertia/jet engine/coefficients) as a selectable airframe.

## 2026-07-13 — M22b: class-correct UAV visual (replaces F-16)

**Status**: GREEN
**Files changed**: src/scene.js
**Tests**: unit 89/89 unaffected (render-only) · browser PASS · screenshot ✓
**Decisions**:
- User call: match the visual to the PHYSICS class instead of an F-16 shell. New procedural Aerosonde-class UAV at true 2.9 m span: sensor-pod fuselage + chin turret + pitot/GPS fittings, high-AR tapered wing with dihedral + orange tip/surface markings, twin tail booms with twin fins, hinged ailerons/elevator/rudders, pusher prop. Cinematic pass (ACES, sky dome, warm light) retained; camera/gear back to small scale.
## 2026-07-14 — M23: portable C99 FDM core (the "yolk") + market research kickoff

**Status**: GREEN (research workflow running in background)
**Files changed**: PRD.md, native/{fdm.h,fdm.c,gen-golden.mjs,golden-check.c,Makefile,golden.h}, .github/workflows/ci.yml, .gitignore
**Tests**: golden cross-validation ALL PASS — point forces/moments worst 2.8e-14 rel; 30 s trim + 20 s doublets-in-turbulence + 8 s ground roll trajectories worst 2.3e-13 m · JS suite unaffected
**Decisions**:
- The deployable asset is a single dependency-free C99 file (native/fdm.c) mirroring src/physics.js + src/wind.js EXPRESSION-FOR-EXPRESSION (comment forbids "optimizing" the math) — 6-DOF, actuators incl. servo faults, ISA, mulberry32 Dryden, ground model.
- Validation architecture: the JS sim is the reference; gen-golden.mjs emits golden.h (point cases + 3 sampled trajectories incl. turbulence); golden-check.c must reproduce them. Tolerances: 1e-12 point-wise, 1e-3 m calm / 0.5 m turbulent trajectories (libm-vs-V8 ulp drift allowance) — measured 10+ orders better on this host.
- `make -C native golden so` gated in CI; libfdm.so builds -fPIC for the VeriStand Linux RT path (cross-compile via CC=<toolchain-gcc>).
- Market research (defense HILS model demand, per user's product focus) launched as a background deep-research workflow.
**Next**:
- M24: NI VeriStand Model Framework wrapper (Inports/Outports/Parameters from the shared table) + interface-spec autogen (참고자료 PDF 양식 준거). Research report → strategy doc.

## 2026-07-14 — M24: VeriStand model wrapper + interface spec autogen

**Status**: GREEN (market-research re-verification still running in background)
**Files changed**: PRD.md, native/{channels.json,nivs_model.c,ni_stub/ni_modelframework.h,gen-spec.mjs,INTERFACE.md,Makefile,fdm.h,fdm.c(+euler/rates helpers)}, .github/workflows/ci.yml
**Tests**: golden ALL PASS · `make nivs` compile-check (−Wall −Wextra −Werror) · spec generator verifies wrapper struct order against channels.json (fails the build on drift) · JS suite unaffected
**Decisions**:
- channels.json is the ONE source of truth: the NIVS wrapper structs must match it field-for-field (gen-spec.mjs gates this), and INTERFACE.md (참고자료 PDF 양식: 구성도 + 신호 테이블, 한글) regenerates from it.
- Channel map: Cmd_* 4ch + Env_* 3ch(wind/turb) + Flt_* 4ch(servo fault enums — HILS 벤치 셀링포인트를 Inport 스위치로) + Sim_Reset(라이징 에지, 1=지상/2=공중트림); Outports 20ch (NED pos/vel, euler, FRD rates, air data, actuator truth, WoW).
- ni_stub/ carries a minimal typed stub of ni_modelframework.h for CI honesty; deployment builds against NI's real SDK sources (command documented in the wrapper header).
- Scaling/calibration/bus protocol stay VeriStand-side per the reference workflow; the model speaks SI only.
**Next**:
- Research report lands → strategy doc. Then M25 (FMU 2.0 export) when ready.

## 2026-07-15 — Market research synthesized + M25: FMI 2.0 FMU export (primary delivery)

**Status**: GREEN
**Files changed**: MARKET.md (new), PRD.md, native/{fmi2_model.c,gen-fmu.mjs,fmi-driver.c,fmi-check.mjs,Makefile,.gitignore}, .github/workflows/ci.yml
**Tests**: golden ALL PASS · nivs compile-check · spec order-verified · **FMU CHECK PASS** (dlopen'd FMU driven through the real fmi2 ABI matches JS golden: trim 2.27e-13 m, ground-roll 0.0 m) · JS 89/89 · CI green
**Decisions**:
- Deep-research finished (2 runs; synthesis skipped on session limit, so synthesized directly from 3-0/2-0 confirmed claims → MARKET.md). Key findings: (a) UAV 6-DOF plant-model demand is real and confirmed (KAIST-한화시스템, KAI papers); (b) the reusable "plant-model catalog" layer is a genuine market gap — incumbents sell rigs/infra, not models; (c) **the original premise was WRONG (3-0 refuted): 리얼타임웨이브 uses its own RTNgine, not NI VeriStand** → don't bind the model to one runtime; (d) **FMU, not a bespoke .so, is VeriStand's official plant-model format (VeriStand 2019+ runs FMI 2.0 co-sim FMUs on PXI Linux RT) and also covers RTNgine/Simulink/SCADE.**
- So M25 makes FMI 2.0 Co-Simulation FMU the PRIMARY deliverable (.so demoted to secondary). fmi2_model.c wraps fdm.c through the standard fmi2 ABI (self-contained, no FMI SDK headers); gen-fmu.mjs emits modelDescription.xml from channels.json (same source of truth as the VeriStand wrapper + INTERFACE.md) and VERIFIES the C vref enum matches it. DoStep sub-steps to the fixed 1/60 s base rate so RT determinism is caller-independent.
- fmi-check.mjs is a REAL ABI test: fmi-driver.c dlopen's the built FMU and drives fmi2Instantiate→SetupExperiment→Init→(SetReal/DoStep loop)→GetReal, compared to a JS golden. `make -C native fmu` packages a spec-compliant fdm-uav.fmu; CI builds + uploads it as an artifact.
- Strategy (MARKET.md §6): first product = golden-VALIDATED small-UAV plant model + auto interface spec + CI traceability (the layer competitors don't productize). Priority customers: (1) HILS integrators (리얼타임웨이브) as an OEM model SUPPLIER via FMU, (2) UAV 체계개발 (KAI/한화시스템) reference-plant channel, (3) ADD/연구기관 independent-verification-model channel.
**Next**:
- Optional: FMI 3.0 variant; ADD-checklist doc mapping our CI evidence to DO-331-style traceability; re-verify the §5 unverified market items after limit reset.

## 2026-07-15 — DO-331 traceability mapping doc + milestone roll-up

**Status**: GREEN
**Files changed**: COMPLIANCE-DO331.md (new), Log.md
**Tests**: no code change (documentation) — full suite remains 89/89 unit · GCS · HILS 7 · plant · SITL · native golden/nivs/spec/fmu-check · browser, all CI-green
**Decisions**:
- Wrote COMPLIANCE-DO331.md: maps every project artifact to DO-331 (Model-Based Dev supplement) objectives. Explicitly a MAPPING, not a certification claim — §6 lists the honest gaps (DO-330 tool qual, structural coverage, verification independence, CM process, MISRA-C, DOORS-style req mgmt).
- Core content: model classification (Design/plant model; JS = reference, C/FMU = deployable, CI proves bit-equivalence); requirements hierarchy (PRD milestones = HLR, channels.json = interface reqs); a full requirements→model-element→verification traceability matrix mapping all 89 unit tests + every gate; MB.A-3..A-7 objective tables with E/P/G status; determinism + reference↔deployment equivalence evidence.
- This operationalizes the MARKET.md §6 differentiator ("검증된 모델 + 자동 트레이서빌리티") as a customer-facing baseline document for ADD/체계업체 cert integration.

**Project milestone roll-up (M0–M25, all GREEN, CI-gated):**
- M0–M6  GCS core: fixed-step deterministic sim; MAVLink v1 bridge+telemetry; ARM/mode/takeoff/land/RTL; missions+GUIDED; parameters; sensor-fault health (SYS_STATUS/STATUSTEXT); battery/EKF/lifecycle telemetry.
- M7–M13 Fidelity: Simulink/UAV-grade 6-DOF (moments+inertia, stability derivatives, actuators, SLC autopilot); engineering visuals; wind+Dryden turbulence; HILS engineering console; autopilot-flies-the-estimate (+pitot); Mahony attitude estimator (+true specific force); ground-roll takeoff.
- M14–M22 Bench+reach: shared vehicle module + HILS scenario runner/bench; GitHub Actions CI; servo faults + rudder roll-assist; onboard wind estimation+crab / QGC MANUAL_CONTROL joystick / MAVLink v2; sim-as-plant UDP lockstep; ArduPilot SITL JSON adapter; cinematic then class-correct UAV visuals.
- M23–M25 Product ("yolk"): portable C99 FDM core with JS-golden cross-validation (1e-13); NI VeriStand Model Framework wrapper + channels.json-driven interface spec autogen; **FMI 2.0 FMU export as primary delivery format** (market-driven pivot per MARKET.md, verified through the real fmi2 ABI).
- Deliverables now emitted+CI-gated: libfdm.so, fdm-uav.fmu, INTERFACE.md, VeriStand wrapper, MARKET.md strategy, COMPLIANCE-DO331.md.

**Next**:
- Optional: FMI 3.0 variant; close specific DO-331 gaps on demand (structural coverage tooling, MISRA-C static analysis) per a target DAL; re-verify MARKET.md §5 unverified items after limit reset; local QGC visual pass.
**Notes**:
- Log.md was already current through M25; this entry adds the DO-331 work plus a scannable M0–M25 roll-up so the 25-milestone history reads at a glance.

## 2026-07-15 — M26: structural coverage (closes DO-331 §6 gap #2, partial)

**Status**: GREEN
**Files changed**: native/{cov-driver.c,coverage.sh,Makefile,.gitignore}, tests/coverage-check.mjs, package.json, .github/workflows/ci.yml, COMPLIANCE-DO331.md
**Tests**: **C core fdm.c = 100% line (210/210)** · JS model modules = 100% line (physics/wind/estimator/sensors/autopilot/missions/params/battery/prng) · unit 89/89 · all prior gates green
**Decisions**:
- Closed the highest-value honest gap from COMPLIANCE-DO331.md §6 (#2 structural coverage) using only toolchain-native tools (gcc/clang gcov, node built-in --experimental-test-coverage) — no new deps, per the lean constitution.
- C deployable core: golden-check reached 93.3%; the shortfall was the wrapper-facing API (fdm_euler/fdm_rates_frd, exercised only via the FMU) + the null-env wind fallback. Added cov-driver.c (assertion-backed, not a line-toucher) to exercise them directly → combined 100%. coverage.sh is portable (gcc gcov / clang llvm-cov gcov), gates at 100%.
- JS reference model: tests/coverage-check.mjs runs node coverage over *.test.mjs only (the *-check/-run scripts spawn servers), asserts per-module line floors. Model math modules all 100%; telemetry/vehicle floors set to measured reality (85/95) with the shortfall documented as browser-only I/O, not model code — honest floors that still catch regressions.
- Wired: `npm run coverage`, `make -C native coverage`, both added to CI. COMPLIANCE-DO331.md updated: MB.A-5 gains a structural-coverage row (E), MB.A-7 line-coverage → E / MC-DC → P, §5 evidence + §6 gap #2 rewritten honestly (line covered+gated; MC/DC still needs a dedicated tool for DAL A).
**Next**:
- Remaining DO-331 gaps are the ones needing commercial tools/process (DO-330 tool qual, MC/DC via VectorCAST/LDRA, verification independence, formal CM) — close per a target DAL when a real cert engagement defines it.

## 2026-07-15 — M27: MC/DC condition coverage (closes DO-331 §6 gap #2 fully — measurement)

**Status**: GREEN
**Files changed**: native/{cov-driver.c(expanded),mcdc.sh,Makefile,.gitignore}, .github/workflows/ci.yml, COMPLIANCE-DO331.md
**Tests**: **fdm.c MC/DC = 98.7% condition coverage (75/76) — 100% of reachable conditions**; C line 100%; JS model modules 100% line; unit 89/89; all prior gates green
**Decisions**:
- Added real MC/DC measurement using gcc-14+ `-fcondition-coverage` + `gcov --conditions` — toolchain-native, no commercial tool (VectorCAST/LDRA not required for measurement).
- golden-check alone gave 73.7% conditions; expanded cov-driver.c (assertion-backed) to exercise every reachable boundary/fault decision the golden trajectories miss: altitude clamps, deflection/throttle rail clamps, servo faults (jam/floating/slow incl. default-factor), act_ch saturation (k>1), wind Gauss-Markov alpha clamp, null-env/no-wind paths, ground sink/friction/attitude clamps (both sides via a below-surface climb-out transient), and the euler asin guard via a denormalized quat → 98.7%.
- The single residual (L71 `quat_normalize` zero-norm guard) is provably unreachable defensive code — formally justified per DO-178C §6.4.4.3 and allowlisted in mcdc.sh; every other uncovered condition fails the build.
- mcdc.sh skips gracefully (exit 0) when no gcc>=14 is present (Apple clang has no -fcondition-coverage), so local macOS dev isn't blocked; CI installs gcc-14 and runs it for real (`make -C native mcdc CC=gcc-14`).
- COMPLIANCE-DO331.md updated honestly: MB.A-7 MC/DC → E* (measured+gated, but the * flags that gcc's tool is not DO-330-qualified, so no cert credit); §5 gains the MC/DC evidence + the L71 justification; §6 gap #2 rewritten — the remaining gap is tool QUALIFICATION (DO-330), not measurement.
**Next**:
- Remaining DO-331 gaps are process/qualification-bound (DO-330 tool qual, verification independence, formal CM, requirements-mgmt tooling) — close per a real cert engagement's target DAL.

## 2026-07-15 — docs: README rewrite (plant-model deliverables + verification)

**Status**: GREEN
**Files changed**: README.md
**Tests**: no code change (docs)
**Decisions**:
- README was badly stale ("M0 — not started" at M27). Full rewrite for BOTH audiences (business/customer + developer), facts only — no strategy or company names (those stay in MARKET.md, not linked from the public README).
- Structure: KR+EN overview → Deliverables table (FMU/.so/VeriStand wrapper/INTERFACE.md/C core) + model class + channel map → Verification & quality (golden cross-val, determinism, 100% line + 98.7% MC/DC, DO-331 mapping, CI gates) → simulator/HILS capabilities → run/build commands → layout → documents → an explicit scope-and-honesty section.
- All referenced files, npm scripts, make targets, and the 89-unit-test count verified against the repo before commit.

## 2026-07-15 — M28: HILS-delivery demo overlay

**Status**: GREEN
**Files changed**: PRD.md, src/hilspanel.js (new), src/main.js, index.html, tests/browser-check.mjs
**Tests**: unit 89/89 · browser PASS (DOM gate extended: badge + 8 scenario buttons + 32 channel rows) · determinism ✓ · render-only
**Decisions**:
- The demo now SHOWS the deliverable: (a) model-identity badge (fdm-uav · FMI 2.0/VeriStand/.so · 60 Hz fixed-step · deterministic · golden-validated), placed above the scenario bar (top-center collided with the HUD at narrow widths); (b) CHANNEL MONITOR (KeyC) rendering the exact delivered channel map — fetched from native/channels.json (the same single source of truth as the FMU/VeriStand wrappers), values computed with the wrappers' NED/FRD mapping, 10 Hz DOM updates; (c) one-click fault-scenario toggles (GPS/pitot dropout, gyro/mag bias, aileron jam, elevator slow, turb ×2.5, clear-all) driving the SAME injection surfaces as the tests — button state reads back from the vehicle so it can't lie.
- browser-check DOM gate extended to cover all three elements; channel monitor is opened before the screenshot so the CI artifact shows the delivery view.
**Next**:
- (idle) Backlog: DO-330-bound gaps per a real cert engagement; FMI 3.0 variant; QGC visual pass.

## 2026-07-15 — docs: README 한국어판

**Status**: GREEN
**Files changed**: README.md, Log.md
**Tests**: 문서만 변경
**Decisions**:
- README를 한국어 중심으로 재작성 (최상단에 영문 한 줄 요약만 유지). 구성은 동일: 개요 → 납품물 → 검증·품질 → 시뮬레이터 기능 → 실행·빌드 → 구조 → 문서 → 범위와 정직성.
- 사용자 지적 반영: "가짜 비행기" 표현 제거 → "조종면 명령에 실제 항공기처럼 응답하는 **모의 비행체 — 플랜트 모델**"로 교체.
- M28 산출물(채널 모니터 C키, hilspanel) 반영. 전략·회사명은 계속 미노출 (MARKET.md 비링크 유지).

## 2026-07-15 — docs: README 용어 정비 (한국어 설명 + 실무 영어 용어)

**Status**: GREEN
**Files changed**: README.md, Log.md
**Tests**: 문서만 변경
**Decisions**:
- 사용자 피드백: 방산·항공 실무에서 영어로 통용되는 용어의 억지 한글화가 오히려 거부감을 준다 → 원칙 확정: **설명 문장은 한국어, 기술 용어·소제목은 실무 영어 그대로**. (6자유도→6-DOF, 연속 루프 폐쇄→SLC, 안정성 증강→SAS, 고장 주입→fault injection, 고착/유동/저속→jam/floating/slow, 접지 스위치→WoW, 결정론적→deterministic, 락스텝→lockstep 등)
- 섹션 제목도 영어 관례로 (Deliverables / Verification & Quality / Simulator & HILS Capabilities / Run & Build / Scope & Honesty).

## 2026-07-15 — VeriStand 통합 절차서 + M29: airframe parameterization

**Status**: GREEN
**Files changed**: docs/VERISTAND-GUIDE.md (new), PRD.md, native/{fdm.h,fdm.c,fmi2_model.c,nivs_model.c,golden-check.c,cov-driver.c,fmi-driver.c,fmi-check.mjs,gen-fmu.mjs,gen-spec.mjs,mcdc.sh,channels.json,INTERFACE.md}, COMPLIANCE-DO331.md
**Tests**: golden **bit-identical after the refactor** (proof the math is untouched) · FMU parameter effect at the real ABI (SetReal mass×1.5 → −117 m/15 s) · line coverage 100% (215/215) · MC/DC 98.7% PASS · unit 89/89
**Decisions**:
- (③-1) docs/VERISTAND-GUIDE.md: 실물 리그 첫 탑재용 절차서 — 사전요건(VeriStand 2019+/Linux RT), FMU 확보(CI 아티팩트; macOS 빌드 금지 경고), Simulation Models 로드, 8단계 smoke test(부호 검증 포함), [확인점 1~5] 회신 양식. 실물 미탑재 사실을 문서 첫머리에 고지.
- (③-2, M29) 기체 상수 38개(#define)를 `fdm_coef` 구조체로 — ALL-double이라 wrapper가 flat double[]로 인덱싱(FMI vref 200+i). NULL→FDM_COEF_DEFAULT(정적 const 초기화). 공개 API에 coef 인자 추가, 전 C 호출부 갱신.
- 단일 진실 소스 확장: channels.json에 "parameters" 38종 → gen-fmu가 modelDescription.xml에 causality="parameter"로 방출하고 **fdm_coef 필드 순서를 fdm.h에서 파싱해 대조(드리프트=빌드 실패)**; INTERFACE.md에 §4 파라미터 표 자동 생성.
- 검증 전략: golden은 defaults에 고정(비트 동일 = 리팩터 무결성 증명), 파라미터 "효과"는 행동으로 검증(C API: cov-driver 질량↑→침하; 실 fmi2 ABI: fmi-driver 'heavy' 케이스). JS reference는 고정 계수 유지(문서화) — golden의 기준이므로.
- mcdc.sh 허용목록을 라인 번호→소스 패턴("n == 0.0") 기반으로 견고화 — 리팩터로 라인이 밀려도 정당화가 유지되고, 다른 미커버는 여전히 빌드 실패.
- NIVS wrapper는 기본 airframe 고정(파라미터 경로는 FMU) — 한계로 명시.
**Next**:
- 사용자 QGC 테스트 결과 대기; VeriStand 실물 [확인점] 회신 시 반영. Backlog: crash-detection latch, NIVS parameter table.

## 2026-07-15 — QGC 실테스트 피드백: Vehicle Setup 파라미터 팝업 해소

**Status**: GREEN
**Files changed**: bridge/compat-params.mjs (new), bridge/server.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 89/89 · gcs-loop-check PASS (73 params = 실파라미터 50 + compat 스텁 23종/35항목, RCMAP 스텁 서빙 검증)
**Decisions**:
- 사용자의 첫 QGC 실테스트에서 Vehicle Setup 진입 시 "펌웨어로부터 파라미터를 찾을 수 없습니다" 팝업 (RCMAP_*/COMPASS_*/FLTMODE*/ARMING_CHECK 등). 원인: HEARTBEAT가 ArduPilot을 선언(모드명 표시용 의도적 선택)하므로 QGC 설정 화면이 표준 ArduPilot 파라미터를 조회 — 비행/미션에는 무해하나 데모 품질 저하.
- 해소: bridge/compat-params.mjs — QGC 전용 호환 스텁(RCMAP 1..4, RC1..8 MIN/MAX/TRIM, FLTMODE1..6=우리 모드 번호, COMPASS/INS 캘리브레이션 0, BATT_MONITOR=4, ARMING_CHECK=1). **브리지에서만 응답·저장하고 심으로는 절대 전달 안 함** — 모델의 실제 파라미터 테이블(src/params.js)은 오염 없이 유지. param_count는 실+스텁 합계로 일관.
**Notes**:
- 사용자는 `npm run bridge` 재시작 + QGC 재연결로 팝업 소멸 확인 가능. Setup의 '라디오' 빨간 표시는 RC 캘리브레이션 미수행 표시로 남을 수 있으나 Arm/비행에는 영향 없음(심이 arming 권위).

## 2026-07-15 — QGC 실테스트 피드백 2: AUTO 지상 시작 시 제자리 회전 수정

**Status**: GREEN
**Files changed**: src/autopilot.js, tests/nav-loop.test.mjs
**Tests**: unit 90/90 (신규: AUTO runway start — ground-roll/이륙/WP1 도달 + yaw-rate 스핀 가드) · HILS 7/7 · gcs PASS
**Decisions**:
- 사용자 실테스트에서 발견: 지상에서 AUTO 진입 시 첫 WP 방향으로 러더만 꺾어 제자리 회전(실 ArduPlane은 미션 이륙을 수행). 원인: AUTO 분기에 지상활주 로직 부재.
- 수정: TAKEOFF의 지상활주를 groundRollControls() 헬퍼로 추출, AUTO에서 WoW+저속이면 동일 지상활주(진입 시 캡처된 활주로 헤딩 유지, Vr 로테이션) 후 웨이포인트 로직 인계.
- 부차 확인: T 키(TAKEOFF)는 설계상 상승 후 GUIDED-hold로 전환 — 미션 시작은 별도로 AUTO 선택 필요(실기와 동일). 사용자 안내로 해소.

## 2026-07-15 — QGC 실테스트 피드백 3: 주기 상태 제자리 회전(풍향계 스핀) 수정

**Status**: GREEN
**Files changed**: src/physics.js, native/fdm.c, native/golden.h(재생성)
**Tests**: idle 120 s 헤딩 드리프트 **0.00°** (수정 전 203°/30 s) · golden 재생성 ALL PASS (JS↔C 일치 1e-14) · line 100% (217/217) · MC/DC 98.8% PASS · unit 90/90 · HILS 7/7 · browser PASS
**Decisions**:
- 사용자 보고: 무조작 주기 상태에서 기체가 제자리 회전. 재현·규명: 기본 난류(×1) 돌풍이 주기 기체에 Va 1~2 m/s를 만들고, 그 때 β=±90°가 선형 Cnβ에 그대로 곱해져 가짜 풍향계 요 모멘트 발생(선형 안정미계수 모델의 유효범위 밖 외삽).
- 수정 2건 (JS 참조·C 코어 동일 적용, 골든 재생성): ① 극저속 aero fade — qbar에 clamp((Va−4)/3, 0, 1) 계수. 4 m/s 미만 공력 0, 7 m/s 이상 ×1.0(IEEE에서 비트 무영향 — 비행 영역 불변). ② 지상 타이어 요 마찰 — 지상속도가 낮을수록 요 감쇠 강화(1.5→7.5 /s), 주기 = 고정.
- 부수 효과: 정상풍 파라미터를 크게 주면(≥7 m/s) 주기 기체가 실제처럼 풍향계 거동 — 물리적으로 타당하여 유지.

## 2026-07-15 — QGC 실테스트 피드백 4: 미션 비행 3중 버그 수정 (이륙 아이템 · 이륙 상승 PIO · 가속 자세추정)

**Status**: GREEN
**Files changed**: src/missions.js, src/autopilot.js, src/estimator.js, tests/{missions,attitude,nav-loop}.test.mjs
**Tests**: unit 92/92 (신규: NAV_TAKEOFF 시퀀싱, 이륙 가속 자세추정 경계) · JS coverage 100% · HILS 7/7 · gcs PASS · browser PASS · (fdm.c 무변경 → native golden/MCDC 영향 없음)
**Decisions**: 사용자 QGC 미션 비행에서 "제자리 배회"를 재현·규명 → 3개 버그가 겹침:
1. **NAV_TAKEOFF 아이템(cmd 22)의 위경도가 0/0** → 좌표변환 시 12,000 km 밖 유령점. 이를 fly-to 웨이포인트로 취급해 이륙 후 그쪽으로 영원히 비행. 수정: TAKEOFF는 위경도 무시, "고도까지 상승 후 다음 아이템"으로 처리(action 'takeoff', 도달=고도−5m).
2. **지상 AUTO 시작 시 지상활주 판정을 지상속도(25 m/s) 기준**으로 해 과도 로테이션·porpoise. 수정: TAKEOFF 모드와 동일하게 WoW(접지) 기준.
3. **근본 원인 — 이륙 가속 중 자세추정기 발산**: 선형 이륙가속(0→30 m/s)이 비력 벡터를 기울여 가속도계가 "기수 +20°"로 오인(실제 −27°). 제어기가 그 47° 오차로 기수를 처박음. 원심 보상(ω×v)은 있으나 선형가속 보상 부재 → 가속 중 가속도계를 게이팅 차단하도록 ACC_BAND 0.15→0.05(정상비행 ‖f‖≈g는 통과, 협조선회는 원심보상으로 통과). 부수: 로테이션 완화(−0.45→−0.20) + 고정자세 climb-out 로직(고도루프 대신 피치 10° 유지, 목표−15m부터 고도홀드) 추가.
- 결과: QGC식 풀미션(이륙아이템+웨이포인트3+착륙패턴) E2E — 7초 이륙, 이륙아이템→WP1,2,3→착륙·disarm, 홈 561m. 이륙 자세오차 9~11°.

## 2026-07-15 — QGC 실테스트 피드백 5: 롤오버 나선 방지 + crash 감지 래치

**Status**: GREEN
**Files changed**: src/autopilot.js (bank protection), src/vehicle.js (crash latch), bridge/server.mjs (crash STATUSTEXT), tests/{autopilot,nav-loop}.test.mjs
**Tests**: unit 94/94 · JS coverage 100% · gcs PASS · HILS 7/7 · browser PASS · E2E 풀미션 정상완주(오탐 없음)
**Decisions**: 사용자 화면에서 롤 −145°(뒤집힘)로 지상에서 나뒹구는 상태. 다중 재현(촘촘 웨이포인트, 8분 loiter, 난류×3)으로는 현행 코드에서 롤오버 재현 불가 → 잠재 불안정 2개를 방어적으로 차단:
1. **협조선회 러더가 tan(roll)** 이라 뱅크 90° 근처에서 지령 폭발 → 나선 회복불가. 수정: rCmd의 tan 인자를 ±0.8 rad로 클램프 + |roll|>55°(0.95 rad) 시 heading 추종 포기하고 wings-level 우선. 검증: 인위적 60/75/90/110° 뱅크에서 1초 내 회복(추락 없음).
2. **crash 감지 래치**: WoW + |roll|>34° 또는 |pitch|>29° 접지(정상 착륙이 만들 수 없는 자세) → crashed 래치(disarm+중립, AUTO 지상활주 재시도 억제). 이전에 사용자가 물었던 "박고도 다시 이륙" 갭 해소. STATUSTEXT "CRASH DETECTED"(CRITICAL) + telemetry crash 플래그. reset로만 해제.
- 주의: 이 crashed 상태는 실기 GCS 관례대로 하드웨어 스위치 개념(진값 WoW+자세)으로만 트리거 — 제어경로의 추정치는 무관.
**Notes**:
- 사용자 화면이 stale일 가능성 큼(git pull + 브리지 완전 재시작 + R로 리셋 필요). 현행 코드는 전 재현에서 롤오버 없음.

## 2026-07-15 — QGC 실테스트 피드백 6: "Auto 모드 진입 실패" — 모드 확인 지연 해소

**Status**: GREEN
**Files changed**: bridge/server.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 94/94 · gcs PASS (신규: DO_SET_MODE 후 즉시 HEARTBEAT 반영 검증) · browser PASS
**Decisions**: 최신 코드로 crash/롤오버는 해소됨(사용자 화면: 활주로 정상 이륙). 새 증상 "미션 시작 불가: Auto 모드 진입 실패". 원인: HEARTBEAT의 customMode가 텔레메트리로만 갱신되어 QGC 명령→SSE→심→텔레메트리(10Hz)→HEARTBEAT(1Hz) 왕복 최대 ~1.1s 지연 → QGC의 모드전환 검증 타임아웃.
- 수정: relayMode() 헬퍼 — 모드 명령 릴레이 시 vehicle.customMode를 낙관적으로 즉시 반영하고 HEARTBEAT를 즉시 1발 전송. 심이 텔레메트리로 확정(불일치 시 다음 텔레메트리가 정정). SET_MODE/DO_SET_MODE(176)/DO_PAUSE_CONTINUE(193)/MISSION_START(300) 전 경로에 적용.
**Notes**: 사용자가 최신 코드로 정상 이륙 확인됨 — 이전 세션들 수정(이륙아이템·이륙PIO·가속자세·뱅크보호·crash)이 실물에서 효과. 남은 건 GCS 프로토콜 타이밍뿐이었음.

## 2026-07-15 — QGC 실테스트 피드백 7: RTL/Loiter 회귀 — crash 감지 오탐 수정

**Status**: GREEN
**Files changed**: src/vehicle.js
**Tests**: unit 94/94 · gcs PASS · HILS 7/7 · browser PASS · RTL 착륙 crash 오탐 0/9(전 시드) · 실제 전복은 여전히 래치
**Decisions**: 사용자 보고 "원래 되던 복귀(RTL)/Loiter가 안됨". 재현: 모델·브리지 레벨 RTL/Loiter는 정상이나, seed 21(라이브 기본값)에서 정상 RTL 착륙이 crash로 오탐 → 착륙 직후 disarm 래치로 "복귀 안됨"처럼 보임. 원인: crash 임계값(롤 34°/피치 29°)이 정상 착륙 범위(최종선회+난류로 롤 ~36°, 플레어 피치 ~25°) 안쪽. 수정: 임계를 롤 69°/피치 52°로 완화 — 정상·거친 착륙에는 절대 안 걸리고 명백한 전복/텀블만 래치. 검증: 9시드 RTL 착륙 오탐 0, 뒤집힌 접지는 여전히 crash.
**Notes**: 별도 fdm-uav-gcs 프로토타입(헤드리스 MAVLink 백엔드 + QGC 커스텀 레이어)은 병행 진행 중. 백엔드 gcs-check PASS.

## 2026-07-15 — QGC 실테스트 피드백 8: "Guided Mode Item 미응답" — 유도 go-to 미션아이템 경로 추가

**Status**: GREEN
**Files changed**: bridge/server.mjs, tests/gcs-loop-check.mjs (+ fdm-uav-gcs/backend 병행)
**Tests**: unit 94/94 · gcs PASS(신규: guided MISSION_ITEM_INT(current=2) ACK+goto) · backend gcs-check PASS
**Decisions**: 사용자 "미션 포인트 전송 실패: 기체가 미션 항목 통신 Guided Mode Item에 반응하지 않음". 모드는 이미 GUIDED(relayMode 정상). 원인: QGC "여기로 이동"이 목표점을 DO_REPOSITION이 아니라 **standalone MISSION_ITEM_INT with current==2(guided goto)** 로 보내고 MISSION_ACK를 기다리는데, 우리 onMissionItem은 업로드 핸드셰이크 중에만 반응(upload===null이면 무시) → 미응답. 수정: upload가 없고 current===2인 MISSION_ITEM_INT는 goto로 처리하고 MISSION_ACK(ACCEPTED) 회신. 브리지 + 헤드리스 백엔드 양쪽 적용.

## 2026-07-15 — QGC 실테스트 피드백 9: guided go-to는 MISSION_ITEM(39) float — QGC 소스로 정확히 규명

**Status**: GREEN
**Files changed**: bridge/mavlink.mjs(+MISSION_ITEM 39), bridge/server.mjs, tests/{gcs-loop-check,mavlink}.test.mjs (+ fdm-uav-gcs/backend 병행)
**Tests**: unit 94/94 · gcs PASS(신규: MISSION_ITEM current=2 float → ACK+goto) · HILS 7/7 · browser PASS · backend PASS
**Decisions**: 피드백8 수정(MISSION_ITEM_INT current=2) 후에도 "Guided Mode Item 미응답" 지속. QGC 소스(MissionManager::writeArduPilotGuidedMissionItem) 직접 확인: ArduPilot guided go-to는 **구형 MISSION_ITEM(id 39, float 도 단위 좌표), current= altChangeOnly?3:2** 를 encode해 전송하고 MISSION_ACK를 기다림. 우리는 MISSION_ITEM(39)을 메시지 테이블에 없어 디코드조차 안 함 → 완전 무시 → 미응답. 수정: mavlink.mjs에 MISSION_ITEM(39, crc_extra 254) 추가(encode/decode 라운드트립 검증), 브리지/백엔드에서 current 2·3이면 goto(lat=x, lon=y 도 단위 그대로) + MISSION_ACK. 교훈: MAVLink 상대 구현은 추측 말고 QGC 소스로 확인.

## 2026-07-16 — 린 리팩터: plant-model 납품 스코프 전면 제거, QGC 연동 시뮬로 재집중

**Status**: GREEN
**Files changed**: 삭제 — native/ 전체(fdm.c/.h, fmi2_model.c, nivs_model.c, channels.json, gen-*.mjs, golden/cov/mcdc, Makefile, ni_stub, INTERFACE.md, 빌드 산출물 .fmu/.so), bridge/plant.mjs, bridge/sitl.mjs, tests/plant-check.mjs, tests/sitl-check.mjs, COMPLIANCE-DO331.md, MARKET.md, docs/VERISTAND-GUIDE.md · 수정 — src/vehicle.js(sim-as-plant `direct` 훅 제거), src/hilspanel.js(배지·채널모니터 재구성), tests/coverage-check.mjs, .github/workflows/ci.yml, package.json, README.md, PRD.md
**Tests**: unit 94/94 · coverage PASS · gcs PASS · HILS 7/7 · browser PASS(console 0, badge·scenario 8·channel rows 32, __advance 재현) · determinism 유지
**Decisions**:
- 사용자 지시: 이 프로젝트는 "간결하게 QGC로 컨트롤 및 연동 시뮬레이션을 보여주는 화면" 역할만. C 컨버터/VeriStand .so/FMU 등 납품 모델은 별도(SCADE/Simulink)로 진행 → 여기서 전부 제거해 린하게.
- 제거 스코프: M20(sim-as-plant), M21(ArduPilot SITL), M23~M25(native C99·VeriStand·FMU), M29(FMI 파라미터화), 시장조사·DO-331 트레이서빌리티. PRD에 제거 사유 명시(재추가 금지).
- hilspanel: 배지에서 FMI/VeriStand/.so/golden 문구 제거 → "fdm-uav 6-DOF · 60 Hz fixed-step · deterministic". 채널 모니터는 삭제된 native/channels.json fetch를 인라인 CHANNELS 신호맵(12 inport + 20 outport)으로 대체 — 값은 매 프레임 vehicle에서 라이브 read. 유용한 HILS 신호 뷰라 기능은 보존.
- vehicleStep 시그니처에서 `direct`(외부 컨트롤러 주입) 인자 제거. crash 감지의 `&& !direct` / `else if (direct)` 분기 삭제.
- CI: native golden/nivs/spec/fmu/coverage/mcdc·gcc-14·plant-check·sitl-check·FMU 아티팩트 스텝 삭제. 유지: unit, JS coverage, gcs loop, HILS bench, browser gate, screenshot.
- package.json: plant/check:plant/sitl/check:sitl 스크립트 삭제.
**Next**:
- QGC 실테스트 계속(미션/guided go-to/RTL 실물 확인). 필요 시 프로토콜 미세조정.
**Notes**:
- 납품용 plant model은 별도 저장소(예: fdm-uav-gcs)에서 진행. 이 repo는 이제 QGC 컨트롤 + 시뮬레이션 화면에만 집중.
- vehicle.js 커버리지 floor 95 유지(현 97.5). `direct` 분기 제거로 커버리지 오히려 개선.

## 2026-07-16 — M30: 지오펜스(Geofence) — QGC Fence 업로드 + 심 위반 회피

**Status**: GREEN
**Files changed**: src/geofence.js(신규), src/vehicle.js, bridge/mavlink.mjs, bridge/server.mjs, bridge/compat-params.mjs, tests/{geofence.test.mjs(신규), mavlink.test.mjs, nav-loop.test.mjs, gcs-loop-check.mjs, coverage-check.mjs}, PRD.md, README.md
**Tests**: unit 109/109 · coverage PASS(geofence 100%, vehicle 97.8%) · gcs PASS(fence 업로드/다운로드 핸드셰이크) · HILS 7/7 · browser PASS
**Decisions**:
- 사용자 요청: "비행금지구역 추가해". QGC Fence 편집기는 지오펜스를 **미션 프로토콜 + mission_type=1**로 업로드 → 같은 COUNT→REQUEST→ITEM→ACK 핸드셰이크를 mission_type으로 분기.
- **MAVLink 확장 필드 지원 추가**: mission_type은 v2 전용 extension field. mavlink.mjs에 `def.ext` 개념 도입 — base 길이(crc_extra 대상, v1 정확일치)와 full 길이(base+ext, v2 상한) 분리. v1은 base만 인코딩(ext 미전송→0), v2는 full 인코딩 후 trailing-zero 절단. crc_extra는 base 필드만 대상이라 상수 불변. MISSION_COUNT/REQUEST_LIST/REQUEST_INT/REQUEST/ITEM_INT/ACK에 `ext: mission_type` 부여.
- **src/geofence.js(순수)**: buildFence(items→로컬 원/다각형), fenceBreach(pos, fence, altMax)→위반 라벨|null, pointInPolygon(ray-casting). 원/다각형 inclusion+exclusion + 고도 상한(FENCE_ALT_MAX). 폴리곤 정점은 연속 아이템을 param1 개수로 그룹핑.
- **vehicle.js 집행**: `fence`/`fenceAltMax`/`fenceBreach` 상태 + `fence` 커맨드(부분 갱신: items 재구성, altMax 설정, 빈 items=해제). vehicleStep가 **추정 위치**로 매 스텝 위반 검사 — 비행 중 위반 시 RTL로 회피(crash 래치와 달리 복귀 가능, 다시 안으로 들어오면 자동 해제). telemetry faults에 `fence` 키로 실어 기존 STATUSTEXT edge 머신이 경고 발화.
- **bridge/server.mjs**: startUpload/finishUpload에 mtype 도입. mission_type=1이면 fenceItems에 수집 후 `fence` SSE 커맨드 push. 다운로드(REQUEST_LIST/REQUEST_INT)도 mission_type 분기. FENCE_ALT_MAX/FENCE_ENABLE PARAM_SET은 심으로 forward(FENCE_FORWARDED).
- compat-params에 FENCE_* 스텁 10종 추가(QGC Fence 페이지 파라미터 조회 대응). 집행은 "업로드된 지오메트리가 있으면 ON" — FENCE_ENABLE은 altMax 게이팅에만 사용.
- 교훈 재확인: MAVLink 확장 필드는 v1에 없고 v2에서만 실림 — v1 정확길이 검사를 base로, v2 상한을 full로 분리해야 기존 v1 미션 경로가 안 깨짐.
**Next**:
- (별건 추천) AUTO 웨이포인트 유도를 L1(레그 추종+조기전환)으로 개선 — 현재 point-pursuit 오버슈트 루프 해소.
- (옵션) 3D 씬에 펜스 지오메트리 시각화(현재는 QGC에만 표시).
**Notes**:
- 사용자 QGC 실테스트에서 이륙/복귀/go-to 정상 확인됨. 미션 경로가 뱅글뱅글 도는 건 버그 아님(고정익 최소선회반경 ~134m + 단순 point-pursuit) — L1 개선은 별도 진행 대기.

## 2026-07-16 — docs: QGC 연동 테스트 & 초보자 사용 가이드 추가

**Status**: GREEN
**Files changed**: docs/QGC-GUIDE.md(신규), README.md
**Tests**: n/a (문서) · 기존 게이트 영향 없음
**Decisions**:
- 사용자 요청: 다른 윈도우 PC에서 git 받아 QGC로 테스트하는 가이드 + QGC/ArduPilot 개발자 사용법을 초보자용으로 아주 쉽게, git 포함.
- docs/QGC-GUIDE.md 작성(한국어, 초보자 대상): 30초 개요·그림 → 윈도우 설치(Node/QGC/Git, npm install 불필요 명시) → 실행 3단계 → QGC 화면 설명 → 기본 조작 실습(ARM/이륙/모드/go-to/미션/RTL/지오펜스/파라미터/고장주입) → 키보드 단축키 표 → 용어 사전 → 문제 해결 FAQ(방화벽·연결 등) → 개발자 메모(포트·핵심 파일·브리지 로그 읽기·테스트).
- 미션 경로 루프/착륙 기울기 등 사용자가 실제 물었던 것들을 FAQ에 "정상입니다"로 선반영.
- README Documents에 가이드 링크 최상단 배치.
**Next**:
- (대기) AUTO 유도 L1 개선, 지오펜스 3D 시각화(옵션).
**Notes**:
- 이 프로젝트는 외부 의존성 0 — 윈도우에서 Node.js만 있으면 `npm run bridge`로 바로 실행(npm install 불필요).

## 2026-07-16 — QGC 실테스트 피드백 10: "Vehicle failed to arm" — ARM 낙관적 반영

**Status**: GREEN
**Files changed**: bridge/server.mjs, tests/gcs-loop-check.mjs
**Tests**: unit 109/109 · gcs PASS(신규: HEARTBEAT가 ARM 즉시 반영) · HILS 7/7 · browser PASS
**Decisions**: 사용자 보고 "잘 되다 안된다" — QGC 이륙 시 간헐적 "Unable to start takeoff: Vehicle failed to arm". 원인: QGC 이륙은 ①ARM 명령 → ②HEARTBEAT의 ARMED 비트로 시동 확인 대기 → ③확인되면 takeoff 순서인데, ARM(400)에 낙관적 반영이 없어 시동 확인이 sim→telemetry(10Hz)→HEARTBEAT(1Hz) 왕복(~1s+)을 기다림 → QGC 타임아웃(간헐적). 모드 변경 때(relayMode)와 동일한 계열 버그. 수정: relayArm() 헬퍼 — ARM 명령 릴레이 시 vehicle.armed 즉시 반영 + HEARTBEAT 즉시 전송. NAV_TAKEOFF(22)도 armed+TAKEOFF 모드를 낙관적으로 반영. sim이 telemetry로 확정(불일치 시 정정).
**Notes**: 낙관적 반영 3종 완비 — 모드(relayMode)/시동(relayArm)/이륙. QGC의 명령→확인 타임아웃 계열 이슈 정리됨.

## 2026-07-16 — QGC 실테스트 피드백 11: 지오펜스 회귀 — FENCE_ALT_MAX 자동 적용 제거

**Status**: GREEN
**Files changed**: bridge/server.mjs, bridge/compat-params.mjs
**Tests**: unit 109/109 · gcs PASS(펜스 지오메트리 경로 유지) · HILS 7/7 · browser PASS
**Decisions**: 사용자 보고 "미션까지 잘 되다가 이제 go-to/RTL/이륙 다 안됨(더 나빠짐)". 원인: M30에서 FENCE_ENABLE/FENCE_ALT_MAX PARAM_SET을 심으로 forward하게 했는데, QGC Fence 페이지를 열거나 펜스를 만지면 QGC가 FENCE_ENABLE를 write → 브리지가 altMax=FENCE_ALT_MAX(기본 120m)를 심에 전달 → 고도 120m 초과 시 지오펜스 위반으로 자동 RTL 회피. 사용자 미션/go-to가 120~150m라 계속 복귀당해 "안 되는" 것처럼 보임. 수정: FENCE_* 파라미터를 순수 compat 스텁으로 되돌리고 자동 altMax forward 제거(FENCE_FORWARDED 삭제). 지오펜스는 이제 사용자가 **직접 그려 업로드한 지오메트리(원/다각형)** 만 집행 — 명시적 opt-in. FENCE_ENABLE 기본값도 ArduPlane과 동일하게 0으로.
**Notes**: 교훈 — 파라미터 조회/편집이 조용히 비행 동작을 바꾸면 안 됨. 고도 펜스는 사용자가 명시적으로 원할 때 별도 경로로. 브리지 재시작 + 브라우저 새로고침하면 잔여 펜스도 초기화됨.
