# flight-sim-fable5

A lean, **GCS-first** deterministic 6-DOF flight simulator that flies under a Ground
Control Station (QGroundControl) over **MAVLink** — an engineering / HILS bench, not a
game. A from-scratch rebuild (targeting the **Claude Fable 5** model in Claude Code) of
a design proven in `../flight-sim2`, with the hard-won lessons baked into the spec.

## Status

**M0 — not started.** Roadmap: M0 baseline → M1 telemetry → M2 commands → M3 GUIDED/
missions → M4 parameters → M5 HILS faults → M6 telemetry completeness. See `PRD.md`.

## Start here (for Claude Code / Fable 5)

1. Read `CLAUDE.md` (operating manual) then `PRD.md` (the spec — wins on conflict).
2. Build **M0** first (scaffold + a flying box + deterministic `window.__advance` +
   keyboard control + first test). Ship it GREEN, log it, commit `M0: baseline`.
3. Proceed milestone by milestone; never skip the tests or the Log.md entry.

## Run (once code exists)

```bash
python3 -m http.server 8123      # static sim → http://localhost:8123
npm run bridge                   # sim + MAVLink bridge → http://localhost:8765 (UDP :14550)
npm test                         # unit tests
```

Then open QGroundControl — it auto-connects to UDP 14550.

## Layout (target)

```
src/        physics, sensors, estimator, autopilot, missions, telemetry, missionLink, engineering, main
bridge/     server.mjs (HTTP/SSE ↔ UDP), mavlink.mjs (v1 encode/decode)
tests/      *.test.mjs (units) + gcs-*-check.mjs (integration)
```

## Model

This project is configured to use **Claude Fable 5** (`claude-fable-5`) via
`.claude/settings.json`. Confirm with `/model` inside Claude Code.
