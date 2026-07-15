# flight-sim-fable5

A deterministic **6-DOF small-UAV plant model** for Hardware-in-the-Loop
Simulation (HILS), delivered as an FMU / shared library and validated end-to-end
by an automated cross-check harness — plus a browser flight simulator and a full
MAVLink Ground-Control-Station loop built on the same model.

> **개요.** HILS에서 비행조종컴퓨터(제어기)를 시험하려면 "가짜 비행기" 역할을 하는
> **비행체 플랜트 모델**이 필요합니다. 이 저장소는 그 플랜트 모델을 — **검증된 상태로**,
> 여러 HILS 도구에 그대로 꽂을 수 있는 **표준 형식(FMI/FMU, 리눅스 실시간 `.so`,
> NI VeriStand 래퍼)** 으로 — 제공합니다. 같은 모델 위에 브라우저 비행 시뮬레이터와
> QGroundControl 연동(MAVLink) 루프가 함께 올라가 있습니다.

**Live demo:** <https://kim-hakseong.github.io/flight-sim-fable5/>
(brief note: the static page runs the standalone sim; the MAVLink bridge runs locally.)

---

## Deliverables — the plant model (납품물)

The physics core is a single dependency-free **C99** file (`native/fdm.c`) wrapped
into standard delivery formats. One source of truth (`native/channels.json`) drives
every wrapper's I/O map and the interface spec, so they cannot drift apart.

| Deliverable | 파일 | What it is |
|---|---|---|
| **FMI 2.0 FMU** (primary) | `native/fdm-uav.fmu` | Co-Simulation FMU — the standard model plug-in; loads in NI VeriStand, and other FMI-capable HILS/modeling tools |
| **Shared library** | `libfdm.so` | Position-independent `.so` for Linux Real-Time (PXIe-class) targets |
| **VeriStand wrapper** | `native/nivs_model.c` | NI VeriStand Model Framework entry points (`USER_Initialize` / `USER_TakeOneStep`) |
| **Interface spec** | `native/INTERFACE.md` | Auto-generated signal tables (Inports/Outports, ranges, units) + block diagram |
| **Portable core** | `native/fdm.c` / `fdm.h` | The math: 6-DOF rigid body, actuators (with faults), ISA atmosphere, Dryden wind |

**Model class.** Aerosonde-class small UAV (~13.5 kg), stability-derivative 6-DOF
(forces + moments, diagonal inertia; Beard & McLain–style coefficients), first-order
control-surface actuators with deflection limits, ISA density, seeded Dryden
turbulence, and a ground model (rolling resistance, brakes, gear). Cruise ≈ 30 m/s.

**Channel map** (SI units; scaling/bus protocol handled on the HILS side):
- **Inports (12):** aileron/elevator/rudder/throttle commands, steady wind N/E,
  Dryden intensity, per-channel servo-fault switches (jam/floating/slow), reset.
- **Outports (20):** position & velocity (NED), attitude (euler), body rates (FRD),
  air data (Va/α/β), actual actuator deflections, weight-on-wheels.

## Verification & quality — the differentiator (검증)

The delivered C model is not shipped on trust: a JavaScript **reference model** is the
golden source, and CI proves the C/FMU deployable reproduces it, every commit.

- **Golden cross-validation** — point-wise forces/moments match the reference to
  **~1e-14 relative**; full trajectories (trim, doublets-in-turbulence, ground roll)
  to **~1e-13 m**. The packaged FMU is driven through the real `fmi2` ABI and matched
  to the same golden.
- **Determinism** — identical inputs → bit-identical outputs (seeded PRNG; fixed
  1/60 s step; no wall-clock, no `Math.random`).
- **Structural coverage** — **100% line coverage** of the C core and of the JS model
  modules; **MC/DC condition coverage 98.7%** on the C core (100% of *reachable*
  conditions; the one residual is a justified, unreachable defensive guard), measured
  with `gcc -fcondition-coverage`.
- **Traceability** — requirements → model element → verification case, mapped to
  DO-331 objectives in `COMPLIANCE-DO331.md` (an honest mapping, not a certification
  claim; open gaps are listed).
- **Continuous integration** — every push runs: unit tests, JS model coverage, the
  GCS packet loop, the HILS scenario bench, the sim-as-plant lockstep check, the
  ArduPilot SITL check, the native golden/wrapper/FMU/coverage/MC-DC gates, and a
  headless-browser check. The FMU is uploaded as a build artifact.

## Simulator & HILS capabilities

Built on the same model, for demonstration, engineering, and integration testing:

- **6-DOF flight** with successive-loop-closure autopilot (bank/pitch/heading/
  altitude/airspeed), manual mode with stability augmentation, ground-roll takeoff.
- **Environment** — steady wind + seeded Dryden turbulence (airspeed ≠ groundspeed).
- **Fault injection** — 6 sensors (gyro/accel/mag/baro/pitot/GPS: freeze/dropout/bias)
  and 4 servos (jam/floating/slow), surfaced to the GCS and the engineering console.
- **Estimators** — gated position/velocity nav with fault detection; Mahony attitude
  estimator with gyro-bias estimation; onboard wind estimation. The control path flies
  on estimated state (weight-on-wheels is the only truth discrete).
- **GCS loop** — MAVLink **v1 & v2** to QGroundControl: telemetry, ARM/mode/takeoff/
  land/RTL, mission upload & flight, live parameters, sensor-health & STATUSTEXT,
  battery, EKF status, WIND, virtual joystick.
- **HILS bench** — declarative scenarios (fault at t, pass/fail checks) run
  deterministically; `window.__hils` in the browser, `npm run hils` on the CLI.
- **External-controller modes** — sim-as-plant UDP lockstep (an external flight
  controller closes the loop), and an ArduPilot SITL JSON backend adapter.
- **Engineering console** (`E` key) — live state vector, estimator-vs-truth, per-sensor
  and per-servo fault bench, wind/battery, strip charts.

## Run & build

```bash
# Browser simulator (standalone)
python3 -m http.server 8123        # → http://localhost:8123   (T takeoff · Space arm · E console)

# With the MAVLink bridge (serves the sim AND speaks to QGroundControl on UDP 14550)
npm run bridge                     # → http://localhost:8765

# Test / verification gates
npm test                           # unit tests
npm run coverage                   # JS reference-model structural coverage
npm run hils                       # HILS scenario bench
node tests/gcs-loop-check.mjs      # MAVLink packet round-trip (bridge + fake GCS)

# Native model: build + verify + package
make -C native golden              # cross-validate the C core against JS golden vectors
make -C native coverage            # C core line coverage (100%)
make -C native mcdc CC=gcc-14      # C core MC/DC condition coverage (needs gcc ≥ 14)
make -C native fmu                 # build + package native/fdm-uav.fmu
make -C native so                  # build libfdm.so
make -C native spec                # regenerate native/INTERFACE.md

# External-controller modes
npm run plant                      # sim-as-plant, UDP JSON lockstep for an external FC
npm run sitl                       # ArduPilot SITL JSON backend adapter (UDP 9002)
```

## Repository layout

```
src/          reference model + browser sim: physics, wind, sensors, estimator,
              autopilot, missions, telemetry, params, battery, vehicle (shared core),
              hils (scenario runner), engineering (HILS console), scene, main
bridge/       server.mjs (HTTP/SSE ↔ UDP), mavlink.mjs (v1/v2), plant.mjs, sitl.mjs
native/       fdm.c/.h (C99 core), fmi2_model.c (FMU), nivs_model.c (VeriStand),
              channels.json (single source of truth), gen-*.mjs (generators),
              golden-check.c / cov-driver.c, coverage.sh / mcdc.sh, Makefile
tests/        *.test.mjs (89 unit tests) + *-check.mjs / hils-run.mjs (integration gates)
.github/      CI workflow (all gates on every push)
```

## Documents

- `PRD.md` — the specification (authoritative), milestones M0–M27.
- `native/INTERFACE.md` — model interface spec (auto-generated).
- `COMPLIANCE-DO331.md` — DO-331 traceability mapping (with an honest gap list).
- `Log.md` — per-milestone development log.
- `CLAUDE.md` — engineering operating manual / conventions.

## Scope & honesty

This is a **small-UAV-class** flight model (not a high-performance jet); the visuals
are engineering visualization, not a game. The DO-331 document is a **traceability
mapping**, not a certification claim — remaining gaps (tool qualification, verification
independence, formal configuration management) are listed explicitly and are closed
per a target assurance level in an actual certification engagement.
