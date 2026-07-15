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
