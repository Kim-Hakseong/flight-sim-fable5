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
