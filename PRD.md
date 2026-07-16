# PRD — flight-sim-fable5

> The authority. **This wins over CLAUDE.md on any conflict.** Keep it current: if the
> plan changes, edit this file first, then build.

## 1. Vision

A **browser-based, deterministic 6-DOF flight simulator that flies under a Ground
Control Station (QGroundControl) over MAVLink.** Its one job is to exercise and
demonstrate the full GCS loop — telemetry up, commands and missions down, the vehicle
responds — precisely and with depth, and to serve as a HILS/engineering bench (inject
sensor faults, watch the estimator, tune gains live). It is NOT a game.

**Scope boundary.** This project is ONLY the QGC control + integrated-simulation
screen. A deliverable/productized plant model (native C core, FMU, VeriStand `.so`,
SCADE/Simulink port, DO-331 traceability) is explicitly OUT OF SCOPE here and lives
in a separate repository. Do not add it back to this project.

Deployed as a static site (GitHub Pages / any static host). The MAVLink bridge runs
locally (a Node process) because it speaks UDP to QGroundControl.

## 2. Principles (see CLAUDE.md §0 for the full constitution)

- GCS loop first; lean; deterministic; test-gated; log every loop.
- No game features. No build step. Vanilla ES modules + Three.js r128 from CDN.

## 3. Tech stack

- **Rendering:** Three.js r128, global `window.THREE` from a CDN `<script>`. No bundler.
- **Sim/logic:** vanilla JavaScript ES modules. Pure, unit-tested physics/nav.
- **Bridge:** Node.js (`.mjs`, no deps) — HTTP + Server-Sent Events on :8765, MAVLink
  v1 over UDP :14550 to QGroundControl. Serves the static sim too (single command).
- **Tests:** `node --test` for units; node scripts that spin the bridge + a fake-GCS
  UDP socket for GCS integration; headless Chrome (CDP) for browser/console checks.
- **Determinism:** a seeded PRNG (e.g. mulberry32) for all noise/world-gen; a fixed
  timestep loop; `window.__advance(seconds, dt)` drives the sim with no wall clock.

## 4. Milestones (GCS-first) — build in order, each ships GREEN

- **M0 — Lean baseline.** Project scaffold (package.json, .gitignore, index.html,
  `src/main.js`). A minimal aircraft (even a box) in a Three.js scene, a fixed-step
  6-DOF-ready loop, and a deterministic `window.__advance` path. Basic keyboard control
  (pitch/roll/yaw/throttle). First unit test + first console-0 check.
  *Verify:* page loads with 0 console errors; `__advance` is reproducible.

- **M1 — Bridge + telemetry.** Node bridge; `bridge/mavlink.mjs` encodes HEARTBEAT,
  ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, GPS_RAW_INT; sim POSTs telemetry → bridge
  relays to QGC over UDP. Sim local coords → geodetic (lat/lon/alt) around a home point.
  *Verify:* QGC shows the aircraft on the map and it moves; `gcs-loop-check` passes.

- **M2 — Command loop.** Decode COMMAND_LONG/SET_MODE; `ARM`/`DISARM`, `MODE`
  (MANUAL/AUTO/GUIDED), `NAV_TAKEOFF` / `NAV_LAND` / `RTL`; correct `COMMAND_ACK`.
  The sim is authoritative for mode/arm and the HEARTBEAT base_mode reflects it.
  *Verify:* QGC ARM/DISARM cuts the sim's engine; mode shows correctly.

- **M3 — GUIDED + missions.** Mission upload handshake (MISSION_COUNT → REQUEST_INT →
  ITEM_INT → ACK); autopilot flies the waypoints; GUIDED go-to (DO_REPOSITION) and
  loiter; position feedback + MISSION_CURRENT / MISSION_ITEM_REACHED close the loop.
  *Verify:* upload a plan in QGC, it flies; a "go here" reroutes it.

- **M4 — Parameters.** `PARAM_REQUEST_LIST` / `PARAM_REQUEST_READ` / `PARAM_SET` for
  key gains (autopilot) + sensor-noise sigmas, with range clamping. A shared param
  table used by both sim and bridge.
  *Verify:* QGC reads the params and a SET re-tunes the vehicle live.

- **M5 — HILS faults visible in the GCS.** A sensor error model (scale/bias/noise/lag)
  + fault injection (freeze/dropout/bias) via `injectFault`. Surface active faults over
  MAVLink: `SYS_STATUS` sensor-health bits (faulted sensor reads red in QGC) +
  `STATUSTEXT` notifications on each edge.
  *Verify:* inject a GPS fault → QGC shows GPS unhealthy + a warning toast.

- **M6 — Telemetry completeness.** Deterministic battery drain (real `SYS_STATUS`
  battery), `EKF_STATUS_REPORT` from the nav estimator's health, lifecycle `STATUSTEXT`
  (arm/mode/nav), mission progress. Rounds out what QGC displays.
  *Verify:* battery gauge depletes; EKF indicator reacts to a nav fault.

- **M7 — High-fidelity flight model (Simulink/UAV-grade).** Replace the rate-command
  kinematics with a full rigid-body 6-DOF: forces AND moments, inertia tensor,
  stability-derivative aero model (CL/CD/Cm/CY/Cl/Cn with α, β, p, q, r and control-
  surface terms — Beard & McLain small-UAV style), control surfaces (δa/δe/δr/δt)
  behind first-order actuators with deflection limits, ISA atmosphere (ρ(h)), and a
  propeller thrust model. Autopilot becomes successive-loop-closure (bank→aileron+
  roll damping, pitch→elevator+q damping, heading→bank, alt/speed→pitch/throttle);
  MANUAL mode gets SAS damping so it stays flyable. Param table exposes the new gains.
  *Verify:* trim exists near cruise; damped responses (unit-tested); all M2/M3
  behaviors (takeoff/land/RTL/mission/goto) still converge closed-loop.

- **M8 — Engineering visuals.** Procedural (no-asset) UAV model whose control
  surfaces and prop VISIBLY move with the actuator states (HILS: see the outputs),
  runway + seeded procedural terrain detail, sun light + shadows, smoothed chase
  camera, HUD with Va/α/β/surface deflections. Still Three.js r128 CDN, no bundler.
  *Verify:* console-0 + screenshot artifact; determinism untouched (render-only).

- **M9 — Wind + Dryden turbulence.** Steady wind (N/E, live-settable params) plus
  seeded Dryden gusts (Gauss-Markov per body axis, B&M Table-4.1-style Lu/Lv/Lw and
  sigmas, intensity param). Aero runs on air-relative velocity; airspeed ≠ groundspeed.
  Deterministic: gusts thread the same pure PRNG as the sensors.
  *Verify:* crosswind → heading holds, ground track crabs; turbulence → autopilot
  holds altitude band; zero-intensity + zero-wind reproduces the old trajectories.

- **M10 — Engineering console (HILS bench).** `src/engineering.js`: toggleable
  panel (KeyE) with the live state vector, estimator-vs-truth errors + variances,
  per-sensor health with fault inject/clear buttons, wind/battery readouts, and
  strip charts (alt truth-vs-estimate, Va, estimator error). Render-only.
  *Verify:* console-0, DOM gate (panel toggles, charts present), screenshot.

- **M11 — Autopilot flies the estimate.** Guidance/altitude/speed loops consume the
  estimator output (+ a new faultable pitot sensor for Va) instead of truth; attitude
  stays truth (no attitude estimator yet — future work). Touchdown detect is a
  weight-on-wheels switch (truth), like the real discrete. Closes the HILS loop:
  sensor fault → estimator reacts → the AIRCRAFT visibly feels it.
  *Verify:* mission completes on estimated nav, incl. across a 10 s GPS dropout.

- **M12 — Attitude estimator.** Mahony-style complementary filter + gyro-bias
  estimation over the (faultable) gyro/accel/mag: gyro integration corrected by
  accelerometer tilt (gated by ‖f‖≈g so turns don't corrupt it) and magnetometer
  heading. The accelerometer sensor model is upgraded to TRUE specific force
  (aero+prop reaction, body frame) so turn contamination is real. The autopilot,
  SAS damping, and the ATTITUDE downlink all consume the ESTIMATED attitude and
  bias-corrected rates — no truth left in the control path except WoW.
  *Verify:* converges from a large initial error; absorbs an injected gyro bias;
  bounded error through turns; full mission completes on 100% estimated state.

- **M13 — Ground-roll takeoff.** Real vehicle lifecycle from the runway: the sim
  boots DISARMED at the threshold (like a real vehicle; the old airborne boot
  stays available as a test fixture). Ground model: rolling resistance, auto-brake
  at idle throttle, gear "springs" that hold roll/pitch level at rest while leaving
  pitch free for rotation; prop static thrust capped to a realistic value. TAKEOFF
  gains a ground-roll phase — full throttle, rudder centerline steering, rotate at
  Vr (pitot) — then the existing climb-out. KeyT = arm + auto-takeoff.
  *Verify:* closed-loop (fully estimated state) arm → ground roll → rotate →
  climb-out → GUIDED, holding the centerline; brake rollout stops the aircraft.

- **M14 — HILS scenario runner (`__hils`).** Extract the whole vehicle (state +
  arm/mode + params + sensors + estimators + wind + battery) into `src/vehicle.js`
  — ONE implementation shared by the browser sim, the node tests, and the runner.
  `src/hils.js`: declarative scenarios ({boot, seed, params, events:[{t, command|
  fault|clear}], checks:[band/final/reach]}) run deterministically to a pass/fail
  report. Built-in scenario library; `window.__hils.run/list` in the browser;
  `node tests/hils-run.mjs` CLI bench.
  *Verify:* all built-in scenarios PASS, reports are bit-identical across reruns.

- **M15 — CI.** GitHub Actions on every push: unit tests, gcs-loop-check, the HILS
  scenario bench, and the headless-Chrome browser gate.
  *Verify:* the workflow runs green on GitHub.

- **M16 — Actuator faults.** Servo fault injection per channel (δa/δe/δr/δt):
  jam (hold position), floating (surface streams to neutral), slow (degraded slew).
  Wired through the vehicle, the engineering-console bench, STATUSTEXT edges, and
  HILS scenarios.
  *Verify:* mechanics unit-tested; aircraft remains recoverable (no crash) with an
  aileron jammed at trim; a servo scenario in the bench library.

- **M17 — Wind estimation.** Estimate the wind vector onboard (GPS ground velocity
  minus pitot·heading air vector, slow low-pass) and use it: guidance headings become
  COURSES with crab compensation; downlink the ardupilotmega WIND message so QGC
  shows it. *Verify:* estimate converges on the true steady wind; a crosswind leg's
  cross-track error shrinks vs uncompensated.

- **M18 — GCS manual control.** Decode MANUAL_CONTROL (QGC virtual joystick) and
  drive MANUAL mode with it (freshness-gated; keyboard is the fallback).
  *Verify:* packet round-trip + stick moves the surfaces through the whole path.

- **M19 — MAVLink v2.** Frame v2 (0xFD, 24-bit msgid, payload truncation) alongside
  v1: decode both always, reply in the version the GCS last spoke.
  *Verify:* v1/v2 round-trips, truncation restored, bridge auto-upgrades.

- **M22 — Engineering visuals refresh.** Keep the airframe visual honest to the
  underlying small-UAV dynamics with a clean procedural render pass (tone mapping,
  gradient sky dome, warm key light, moving control surfaces + prop). RENDER-ONLY:
  the flight model is untouched, determinism preserved.

- **M28 — HILS demo overlay.** The browser demo surfaces the sim's HILS character:
  (a) a model-identity badge (fdm-uav 6-DOF · 60 Hz fixed-step · deterministic),
  (b) a live CHANNEL MONITOR (KeyC) rendering the sim's I/O signal map (control/
  environment inports, state outports) in NED/FRD, read live from the vehicle, and
  (c) one-click fault-scenario toggles (GPS dropout, gyro/mag bias, pitot drop, servo
  jam/slow, heavy turbulence, clear-all) driving the same injection surfaces as tests.
  Render-only. *Verify:* console-0, DOM gate covers the panel, determinism untouched.

> **Removed from scope (2026-07-16).** Earlier drafts carried M20 (sim-as-plant
> external-controller lockstep), M21 (ArduPilot SITL adapter), and M23–M25 / M29
> (native C99 core, VeriStand wrapper, FMU export, FMI airframe parameterization),
> plus market-research and DO-331 traceability work. These productized-model
> milestones were pruned to keep this repo a lean QGC-control + simulation screen;
> that work will proceed separately (SCADE / Simulink). Do not re-add here.

## 5. Non-goals

Cockpit interiors, multiple maps, weather presets, multiplayer, AI traffic, game
scoring, drones. (These sank the predecessor; do not add them.)

## 6. Verification (definition of done, per milestone)

1. `npm test` green (units for every new physics/nav/MAVLink function).
2. The milestone's GCS integration check passes (node-only where possible).
3. `window.__advance` still reproducible (a determinism check that reruns and compares).
4. Browser console = 0 app errors.
5. A Log.md entry (CLAUDE.md §5) + a commit `M{n}: <task>` + push.

## 7. Known lessons from the predecessor (bake these in)

- Keyboard: key off **`e.code`** (physical key), not `e.key` — a Hangul/IME layout makes
  `e.key` arrive as `'Process'` and silently kills WASD/letter keys (arrows still work).
- SSE from the bridge: disable Nagle (`socket.setNoDelay(true)`) + buffer the last
  command and re-send on (re)connect; the sim dedupes by a monotonic sequence.
- Determinism: integrate battery/anything time-based on the sim `dt`, never wall clock;
  keep world-gen and sensor noise on their own seeded PRNGs so layouts stay identical.
- Headless-Chrome EventSource is occasionally a "zombie" (connects, receives nothing) —
  make the deterministic GCS gate node-only (bridge + fake-GCS UDP), not browser-SSE.
