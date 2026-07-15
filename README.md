# flight-sim-fable5

> A deterministic 6-DOF small-UAV plant model for HILS, delivered as an FMU /
> shared library with an automated golden-validation harness — plus a browser
> simulator and a full MAVLink GCS loop on the same model.

**HILS(Hardware-in-the-Loop Simulation)용 deterministic 6-DOF 소형 UAV plant model**과,
같은 모델 위에 구축된 브라우저 시뮬레이터 · QGroundControl 연동(MAVLink) 루프입니다.

HILS에서 비행조종컴퓨터(FCC)를 시험하려면, 조종면 명령에 실제 항공기처럼 응답하는
**모의 비행체 — plant model**이 필요합니다. 이 저장소는 그 plant model을 **검증된
상태로**, 여러 HILS 장비에 그대로 통합할 수 있는 **표준 형식(FMI/FMU, Linux RT
`.so`, NI VeriStand wrapper)** 으로 제공합니다.

**Live demo:** <https://kim-hakseong.github.io/flight-sim-fable5/>
(정적 페이지는 단독 시뮬레이터로 동작하며, MAVLink bridge는 로컬에서 실행합니다.)

---

## Deliverables — plant model 납품물

물리 코어는 의존성 없는 **C99 단일 파일**(`native/fdm.c`)이며, 이를 표준 납품 형식으로
감쌌습니다. 하나의 소스(`native/channels.json`)가 모든 wrapper의 I/O 맵과 인터페이스
명세서를 생성하므로, **코드와 문서가 어긋날 수 없습니다.**

| Deliverable | 파일 | 설명 |
|---|---|---|
| **FMI 2.0 FMU** (1순위) | `native/fdm-uav.fmu` | Co-Simulation FMU — 표준 모델 플러그인. NI VeriStand 및 FMI 지원 HILS/모델링 도구에 로드 |
| **Shared library** | `libfdm.so` | Linux Real-Time(PXIe급) 타깃용 `.so` |
| **VeriStand wrapper** | `native/nivs_model.c` | NI VeriStand Model Framework 진입점(`USER_Initialize` / `USER_TakeOneStep`) |
| **Interface spec** | `native/INTERFACE.md` | 신호 테이블(Inport/Outport, 범위·단위) + 구성도 — 자동 생성 |
| **Portable C core** | `native/fdm.c` / `fdm.h` | 수학 본체: 6-DOF rigid body, actuator(fault 포함), ISA atmosphere, Dryden turbulence |

**Model class.** Aerosonde급 소형 UAV(약 13.5 kg). Stability-derivative 기반 6-DOF
(force + moment, diagonal inertia; Beard & McLain 계열 계수), deflection limit이 있는
1차 지연 control-surface actuator, ISA 밀도, seeded Dryden turbulence, ground model
(rolling resistance · brake · gear). 순항 약 30 m/s.

**Channel map** (SI 단위; scaling·bus protocol은 HILS 장비 측 처리):
- **Inport 12ch:** aileron/elevator/rudder/throttle 명령, steady wind N/E,
  turbulence intensity, 채널별 servo fault 스위치(jam/floating/slow), reset.
- **Outport 20ch:** position·velocity (NED), attitude (euler), body rates (FRD),
  air data (Va/α/β), 실제 actuator deflection, WoW (weight-on-wheels).

## Verification & Quality — 이 모델의 차별점

납품되는 C 모델은 "믿어 달라"는 방식이 아닙니다. JavaScript **reference model**이
golden 기준이며, C/FMU 배포본이 이를 재현함을 **CI가 매 커밋 자동 증명**합니다.

- **Golden cross-validation** — force/moment 점 단위 비교 오차 **상대 ~1e-14**;
  전체 trajectory(trim, 난류 속 doublet, ground roll) **~1e-13 m**. 패키징된 FMU도
  실제 `fmi2` ABI로 구동해 동일 golden과 대조.
- **Determinism** — 같은 입력이면 비트 단위로 같은 출력 (seeded PRNG, 고정 1/60 s
  step, wall-clock·`Math.random` 미사용).
- **Structural coverage** — C 코어와 JS 모델 모듈 **line coverage 100%**;
  C 코어 **MC/DC condition coverage 98.7%** (도달 가능한 condition은 100%; 잔여
  1건은 도달 불가로 정당화된 defensive code), `gcc -fcondition-coverage`로 계측.
- **Traceability** — 요구사항 → 모델 요소 → 검증 케이스를 DO-331 objective에 매핑
  (`COMPLIANCE-DO331.md`). 인증 주장이 아닌 정직한 매핑이며, 미비 항목을 명시.
- **CI** — 매 push마다 실행: unit test, JS 모델 coverage, GCS packet loop, HILS
  scenario bench, plant lockstep check, ArduPilot SITL check, native
  golden/wrapper/FMU/coverage/MC-DC gate, headless browser check. FMU는 빌드
  아티팩트로 업로드.

## Simulator & HILS Capabilities

같은 모델 위에 구축된 데모·엔지니어링·통합시험 환경:

- **6-DOF flight** — Successive Loop Closure(SLC) autopilot (bank/pitch/heading/
  altitude/airspeed hold), SAS 수동 조종, runway ground-roll takeoff.
- **Environment** — steady wind + seeded Dryden turbulence (airspeed ≠ groundspeed).
- **Fault injection** — 센서 6종(Gyro/Accel/Mag/Baro/Pitot/GPS: freeze/dropout/bias)
  + servo 4ch(jam/floating/slow). GCS와 화면 양쪽에 표시.
- **Estimator** — gated navigation filter (FDE 포함), Mahony attitude estimator
  (gyro bias 추정), onboard wind estimation. **제어 경로 전체가 estimated state로
  비행** (truth는 WoW discrete 하나뿐).
- **GCS loop** — MAVLink **v1·v2**로 QGroundControl 연동: telemetry, ARM/mode/
  takeoff/land/RTL, mission upload·비행, live parameter, sensor health·STATUSTEXT,
  battery, EKF status, WIND, virtual joystick.
- **HILS bench** — 선언적 scenario(시각별 fault 주입 + pass/fail 판정)를
  deterministic하게 재생. 브라우저에서 `window.__hils`, CLI에서 `npm run hils`.
- **External controller mode** — sim-as-plant(외부 FC가 UDP lockstep으로 루프 폐쇄),
  ArduPilot SITL JSON backend adapter.
- **On-screen tools** — Channel Monitor(`C` 키: 납품 channel map을 실시간 값으로
  표시), 원클릭 fault scenario 버튼, Engineering Console(`E` 키: state vector·
  추정 오차·fault bench·strip chart).

## Run & Build

```bash
# 브라우저 시뮬레이터 (단독)
python3 -m http.server 8123        # → http://localhost:8123  (T takeoff · Space arm · C channels · E console)

# MAVLink bridge 포함 (시뮬 서빙 + QGroundControl UDP 14550 연동)
npm run bridge                     # → http://localhost:8765

# 시험·검증 게이트
npm test                           # unit tests
npm run coverage                   # JS reference model structural coverage
npm run hils                       # HILS scenario bench
node tests/gcs-loop-check.mjs      # MAVLink packet round-trip (bridge + fake GCS)

# Native 모델: 빌드 + 검증 + 패키징
make -C native golden              # C 코어를 JS golden vector와 교차검증
make -C native coverage            # C 코어 line coverage (100%)
make -C native mcdc CC=gcc-14      # C 코어 MC/DC condition coverage (gcc ≥ 14 필요)
make -C native fmu                 # native/fdm-uav.fmu 빌드·패키징
make -C native so                  # libfdm.so 빌드
make -C native spec                # native/INTERFACE.md 재생성

# External controller mode
npm run plant                      # sim-as-plant: 외부 FC용 UDP JSON lockstep
npm run sitl                       # ArduPilot SITL JSON backend adapter (UDP 9002)
```

## Repository Layout

```
src/          reference model + 브라우저 시뮬: physics, wind, sensors, estimator,
              autopilot, missions, telemetry, params, battery, vehicle(공유 코어),
              hils(scenario runner), engineering(콘솔), hilspanel(channel monitor), scene, main
bridge/       server.mjs (HTTP/SSE ↔ UDP), mavlink.mjs (v1/v2), plant.mjs, sitl.mjs
native/       fdm.c/.h (C99 코어), fmi2_model.c (FMU), nivs_model.c (VeriStand),
              channels.json (single source of truth), gen-*.mjs (생성기),
              golden-check.c / cov-driver.c, coverage.sh / mcdc.sh, Makefile
tests/        *.test.mjs (unit 89건) + *-check.mjs / hils-run.mjs (통합 게이트)
.github/      CI workflow (매 push마다 전 게이트 실행)
```

## Documents

- `PRD.md` — 사양서(최상위 권위), 마일스톤 M0–M28.
- `native/INTERFACE.md` — 모델 인터페이스 명세 (자동 생성).
- `COMPLIANCE-DO331.md` — DO-331 traceability 매핑 (미비 항목 명시).
- `Log.md` — 마일스톤별 개발 로그.
- `CLAUDE.md` — 엔지니어링 운영 규범.

## Scope & Honesty

이 모델은 **소형 UAV급** flight model이며 고성능 제트기가 아닙니다. 화면은 게임이
아닌 엔지니어링 시각화입니다. `COMPLIANCE-DO331.md`는 **traceability 매핑**이지
인증 주장이 아닙니다 — 남은 항목(tool qualification, 검증 독립성, 정형 형상관리)은
문서에 명시되어 있으며, 실제 인증 사업에서 목표 DAL에 맞춰 해소합니다.
