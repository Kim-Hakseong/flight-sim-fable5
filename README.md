# flight-sim-fable5

> A deterministic 6-DOF small-UAV flight simulator in the browser, flown live from
> QGroundControl over a MAVLink bridge. GCS 연동 시뮬레이션을 보여주는 것이 이 프로젝트의
> 전부이자 핵심입니다 — 정교하고 깊이 있게.

**deterministic 6-DOF 소형 UAV 비행 시뮬레이터**와, 그 위에 얹은 **QGroundControl
연동(MAVLink) 루프**입니다. QGC에서 ARM·모드·이륙·착륙·복귀·미션·지점 이동을 명령하면
브라우저 시뮬레이터의 기체가 실제처럼 응답하고, 텔레메트리가 QGC로 되돌아옵니다.

**Live demo:** <https://kim-hakseong.github.io/flight-sim-fable5/>
(정적 페이지는 단독 시뮬레이터로 동작하며, MAVLink bridge는 로컬에서 실행합니다.)

> **Scope note.** 이 저장소는 오직 **QGC 컨트롤 + 연동 시뮬레이션 화면** 역할만 합니다.
> 별도의 납품용 plant model(FMU / VeriStand `.so` / SCADE·Simulink)은 이 프로젝트가
> 아니라 별도 저장소에서 진행합니다.

---

## The GCS loop is the product

QGroundControl을 UDP 14550으로 붙이면 아래 루프가 돕니다:

- **Telemetry up** — attitude, position(GPS), VFR HUD, body rates, air data,
  battery, EKF status, WIND, sensor health·STATUSTEXT.
- **Commands down** — ARM/DISARM, 모드 변경(Manual/Auto/Guided/RTL/Loiter),
  takeoff·land·RTL, live parameter, virtual joystick.
- **Missions down** — 미션 업로드·비행, "Go to location"(guided go-to), MISSION_START.
- **Geofence down** — QGC Fence 편집기로 그린 원형·다각형 금지/포함 구역과 고도 상한을
  업로드(mission_type=1). 위반 시 기체가 자동 RTL로 회피하고 STATUSTEXT로 경고.
- **Vehicle responds** — 명령이 autopilot을 통해 기체를 실제로 움직이고, 그 결과가
  다시 telemetry로 올라갑니다.

MAVLink **v1·v2**를 직접 인코드/디코드합니다(crc_extra + descending-size 재정렬).
새 메시지는 반드시 `tests/mavlink.test.mjs`에 단위 테스트가 붙습니다.

## Simulator capabilities

- **6-DOF flight** — Aerosonde급 소형 UAV(약 13.5 kg), stability-derivative 6-DOF
  (Beard & McLain 계열), 1차 지연 control-surface actuator, ISA 밀도, seeded Dryden
  turbulence, ground model(rolling resistance · brake · gear). 순항 약 30 m/s.
- **Autopilot** — Successive Loop Closure(SLC): bank/pitch/heading/altitude/airspeed
  hold, SAS 수동 조종, runway ground-roll takeoff.
- **Estimator** — gated navigation filter(FDE 포함), Mahony attitude estimator
  (gyro bias 추정), onboard wind estimation. **제어 경로 전체가 estimated state로
  비행** (truth는 WoW discrete 하나뿐).
- **Fault injection** — 센서 6종(Gyro/Accel/Mag/Baro/Pitot/GPS: freeze/dropout/bias)
  + servo 4ch(jam/floating/slow). GCS와 화면 양쪽에 표시.
- **Determinism** — 같은 입력이면 비트 단위로 같은 출력 (seeded PRNG, 고정 1/60 s
  step, wall-clock·`Math.random` 미사용). `window.__advance(seconds)`가 재현 가능한
  테스트·HILS 표면입니다.
- **On-screen tools** — Channel Monitor(`C` 키: 기체 I/O 신호를 실시간 값으로 표시),
  원클릭 fault scenario 버튼, Engineering Console(`E` 키: state vector · 추정 오차 ·
  fault bench · strip chart).

## Run

```bash
# 브라우저 시뮬레이터 (단독)
python3 -m http.server 8123        # → http://localhost:8123  (T takeoff · Space arm · C channels · E console)

# MAVLink bridge 포함 (시뮬 서빙 + QGroundControl UDP 14550 연동)
npm run bridge                     # → http://localhost:8765
# 이후 QGroundControl 실행 → UDP 14550으로 자동 연결

# 시험·검증 게이트
npm test                           # unit tests
npm run coverage                   # JS 모델 모듈 structural coverage
npm run hils                       # HILS scenario bench
node tests/gcs-loop-check.mjs      # MAVLink packet round-trip (bridge + fake GCS)
npm run check:browser             # headless Chrome: console-0 + determinism + DOM
```

## Repository layout

```
src/          physics, wind, sensors, estimator, autopilot, missions, geofence,
              telemetry, params, battery, vehicle(공유 코어), hils(scenario runner),
              engineering(콘솔), hilspanel(channel monitor), scene, main
bridge/       server.mjs (HTTP/SSE ↔ MAVLink UDP), mavlink.mjs (v1/v2),
              compat-params.mjs (QGC Vehicle Setup 호환 파라미터 스텁)
tests/        *.test.mjs (unit) + gcs-loop-check / hils-run / browser-check / coverage-check
.github/      CI workflow (매 push마다 전 게이트 실행)
```

## Documents

- `PRD.md` — 사양서(최상위 권위).
- `Log.md` — 마일스톤별 개발 로그.
- `CLAUDE.md` — 엔지니어링 운영 규범.

## Scope & honesty

이 모델은 **소형 UAV급** flight model이며 고성능 제트기가 아닙니다. 화면은 게임이
아닌 엔지니어링 시각화이며, 목적은 오직 **QGC 컨트롤 + 연동 시뮬레이션을 정확하게
보여주는 것**입니다.
