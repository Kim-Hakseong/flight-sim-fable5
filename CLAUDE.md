# CLAUDE.md — flight-sim-fable5 Operating Manual

> Read first, every session. **PRD.md wins on any conflict.**
> This is a **from-scratch rebuild** of a lean, GCS-first flight simulator. A prior
> implementation (`../flight-sim2`) proved the design and the pitfalls; this repo
> bakes those lessons in from day 1. You MAY consult that repo for reference, but the
> goal is a clean rebuild you understand end-to-end — do not blind-copy it.

## 0. Constitution

1. **GCS is the north star.** The product is the Ground Control Station / MAVLink
   loop: telemetry up, commands + missions down, the vehicle responds. Prefer work
   that makes that loop more complete and correct over anything cosmetic. If a task
   doesn't serve the GCS loop or the sim core it depends on, push back or defer it.
2. **Don't drift into a game.** The prior project slid into cockpits, multiple maps,
   weather presets, scoring, multiplayer, AI traffic — then had to be stripped back.
   Do NOT add those. This is an engineering/HILS simulator, not a game.
3. **Don't ask — decide.** If ambiguous, pick the simplest working option, do it, and
   leave a one-line rationale in Log.md. Only ask when blocked by something genuinely
   the user's (external accounts, irreversible infra, a real product fork in the road).
4. **No merge without tests.** Physics/nav functions get unit tests; the GCS/command
   path gets an integration check; UI gets console-0 + a screenshot/DOM check.
5. **Deterministic stays deterministic.** The fixed-step sim must NEVER depend on wall
   clock or `Math.random()`/`Date.now()`. A seeded PRNG drives all "randomness". The
   `window.__advance(seconds)` path must stay bit-reproducible — it is the test + HILS
   surface. Battery drain, sensor noise, world generation: all seeded / dt-integrated.
6. **No log, no work.** Append to Log.md every loop (format in §5).
7. **Stay lean.** ES modules, no build step, Three.js from CDN. New dependencies need a
   PRD update first. No framework, no bundler, no TypeScript unless PRD says so.
8. **Auto Log + push.** After any edit/addition, update Log.md AND `git push` to the
   default branch without being asked (once a remote is configured). A push = a deploy.

## 1. Run / test

```bash
# Static sim only:
python3 -m http.server 8123          # → http://localhost:8123

# With the GCS bridge (serves the sim AND speaks MAVLink to QGC on UDP 14550):
npm run bridge                       # → http://localhost:8765  + MAVLink :14550
# Then start QGroundControl; it auto-connects to UDP 14550.

# Unit tests (pure physics/nav/MAVLink):
npm test                             # node --test tests/*.test.mjs

# Deterministic / GCS checks (node-only: spin the bridge + a fake-GCS UDP socket):
node tests/gcs-loop-check.mjs        # packet-level MAVLink round-trip
# Browser checks need a served page + headless Chrome over CDP.
```

## 2. Architecture map (target)

Build toward this shape (create modules as milestones require, not all at once):

- `src/physics.js` — 6-DOF aero/atmosphere (pure functions, unit-tested, ≤ 50 lines each).
- `src/sensors.js`, `src/estimator.js` — sensor error model + gated Kalman nav (FDE).
- `src/autopilot.js`, `src/missions.js` — guidance + mission sequencing.
- `src/telemetry.js` — sim → bridge telemetry.
- `src/missionLink.js` — bridge → sim commands/missions (SSE) → autopilot.
- `bridge/server.mjs`, `bridge/mavlink.mjs` — HTTP/SSE ↔ MAVLink v1 UDP to the GCS.
- `src/engineering.js` — HILS dev console (state vector, surfaces, faults, charts).
- `src/main.js` — wiring + the deterministic fixed-step loop.

## 3. Conventions

- ES modules, no build step. Three.js r128 (global `window.THREE` from CDN).
- **Coordinate frame (do not confuse):** Three.js right-handed, +Y up, −Z forward.
  Body frame: +X right wing, +Y top, −Z nose. Signs: pitch-up +, roll-right +,
  yaw-right +. Put this comment atop every physics/3D module.
- Physics functions are pure (no side effects) and ≤ 50 lines. Comments say *why*.
- Names: `camelCase`, constants `UPPER_SNAKE`, files `kebab-case.js` / single word.
- **MAVLink v1**: encode/decode with the per-message `crc_extra` and the descending-
  size field reorder. Every new message gets a unit test in `tests/mavlink.test.mjs`.

## 4. The GCS loop is the product

When adding a MAVLink message or command:
1. Decode/encode in `bridge/mavlink.mjs` (+ a unit test in `tests/mavlink.test.mjs`).
2. Relay in `bridge/server.mjs` (telemetry out) or via SSE `/commands` (commands in).
3. Wire the sim side (`telemetry.js` to emit, `missionLink.js`/autopilot to act).
4. Add an integration check (spin the bridge + a fake-GCS UDP socket, assert the
   round-trip). Verify against a real GCS (QGroundControl) when possible. Keep the
   deterministic tests green.

## 5. Log.md format (fixed)

```markdown
## YYYY-MM-DD — M{n}: <task>

**Status**: GREEN | RED | WIP
**Files changed**: ...
**Tests**: <unit pass/fail> · <console 0?> · <gcs check?> · <determinism check?>
**Decisions**:
- <one line>
**Next**:
- <next loop>
**Notes**:
- <context for the user>
```

## 6. Absolutely not

- ❌ Game features: cockpit interiors, multiple maps, weather presets, multiplayer,
  AI traffic, scoring, drones.
- ❌ Breaking determinism or the `window.__advance` / `__hils` / `injectFault` surface.
- ❌ Merging a physics/MAVLink change without a test.
- ❌ A build step, bundler, or framework (stay vanilla ES modules + CDN Three.js).
- ❌ Proceeding to the next loop without a Log.md entry (+ push once a remote exists).
