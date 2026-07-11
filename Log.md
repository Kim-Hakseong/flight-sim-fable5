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
