# flight-sim-fable5

> A deterministic 6-DOF small-UAV plant model for HILS, delivered as an FMU /
> shared library with an automated golden-validation harness — plus a browser
> simulator and a full MAVLink GCS loop on the same model. *(문서는 한국어 기준)*

**HILS(Hardware-in-the-Loop Simulation)용 결정론적 6자유도 소형 무인기 플랜트 모델**과,
같은 모델 위에 구축된 브라우저 시뮬레이터 · QGroundControl 연동(MAVLink) 루프입니다.

HILS에서 비행조종컴퓨터(제어기)를 시험하려면, 조종면 명령에 실제 항공기처럼 응답하는
**모의 비행체 — 플랜트 모델**이 필요합니다. 이 저장소는 그 플랜트 모델을 **검증된
상태로**, 여러 HILS 장비에 그대로 통합할 수 있는 **표준 형식(FMI/FMU, 리눅스 실시간
`.so`, NI VeriStand 래퍼)** 으로 제공합니다.

**라이브 데모:** <https://kim-hakseong.github.io/flight-sim-fable5/>
(정적 페이지는 단독 시뮬레이터로 동작하며, MAVLink 브리지는 로컬에서 실행합니다.)

---

## 납품물 — 플랜트 모델

물리 코어는 의존성 없는 **C99 단일 파일**(`native/fdm.c`)이며, 이를 표준 납품 형식으로
감쌌습니다. 하나의 소스(`native/channels.json`)가 모든 래퍼의 입출력 맵과 인터페이스
명세서를 생성하므로, **코드와 문서가 어긋날 수 없습니다.**

| 납품물 | 파일 | 설명 |
|---|---|---|
| **FMI 2.0 FMU** (1순위) | `native/fdm-uav.fmu` | Co-Simulation FMU — 표준 모델 플러그인. NI VeriStand 및 FMI 지원 HILS/모델링 도구에 로드 |
| **공유 라이브러리** | `libfdm.so` | 리눅스 실시간(PXIe급) 타깃용 `.so` |
| **VeriStand 래퍼** | `native/nivs_model.c` | NI VeriStand Model Framework 진입점(`USER_Initialize` / `USER_TakeOneStep`) |
| **인터페이스 명세서** | `native/INTERFACE.md` | 신호 테이블(Inport/Outport, 범위·단위) + 구성도 — 자동 생성 |
| **이식성 코어** | `native/fdm.c` / `fdm.h` | 수학 본체: 6자유도 강체, 액추에이터(고장 포함), ISA 대기, Dryden 난류 |

**모델 급.** Aerosonde급 소형 무인기(약 13.5 kg). 안정미계수 기반 6자유도(힘+모멘트,
대각 관성; Beard & McLain 계열 계수), 편각 제한이 있는 1차 지연 조종면 액추에이터,
ISA 밀도, 시드 기반 Dryden 난류, 지상 모델(구름저항·제동·착륙장치). 순항 약 30 m/s.

**채널 맵** (SI 단위; 스케일링·버스 프로토콜은 HILS 장비 측 처리):
- **Inport 12채널:** 에일러론/엘리베이터/러더/스로틀 명령, 정상풍 N/E, 난류 강도,
  채널별 서보 고장 스위치(고착/유동/저속), 리셋.
- **Outport 20채널:** 위치·속도(NED), 자세(오일러), 기체 각속도(FRD),
  에어데이터(Va/α/β), 실제 액추에이터 편각, 접지(WoW) 신호.

## 검증과 품질 — 이 모델의 차별점

납품되는 C 모델은 "믿어 달라"는 방식이 아닙니다. JavaScript **참조 모델**이 기준
(golden)이며, C/FMU 배포본이 이를 재현함을 **CI가 매 커밋 자동 증명**합니다.

- **골든 교차검증** — 힘·모멘트 점 단위 비교 오차 **상대 ~1e-14**; 전체 궤적(트림,
  난류 속 도블릿, 지상활주) **~1e-13 m**. 패키징된 FMU도 실제 `fmi2` ABI로 구동해
  동일 골든과 대조.
- **결정론** — 같은 입력이면 비트 단위로 같은 출력 (시드 PRNG, 고정 1/60초 스텝,
  벽시계·`Math.random` 미사용).
- **구조적 커버리지** — C 코어와 JS 모델 모듈 **라인 커버리지 100%**;
  C 코어 **MC/DC 조건 커버리지 98.7%** (도달 가능한 조건은 100%; 잔여 1건은 도달
  불가로 정당화된 방어 코드), `gcc -fcondition-coverage`로 계측.
- **트레이서빌리티** — 요구사항 → 모델 요소 → 검증 케이스를 DO-331 목표에 매핑
  (`COMPLIANCE-DO331.md`). 인증 주장이 아닌 정직한 매핑이며, 미비 항목을 명시.
- **지속 통합(CI)** — 매 push마다 실행: 유닛 테스트, JS 모델 커버리지, GCS 패킷 루프,
  HILS 시나리오 벤치, 플랜트 락스텝 체크, ArduPilot SITL 체크, 네이티브
  골든/래퍼/FMU/커버리지/MC-DC 게이트, 헤드리스 브라우저 체크. FMU는 빌드
  아티팩트로 업로드.

## 시뮬레이터·HILS 기능

같은 모델 위에 구축된 데모·엔지니어링·통합시험 환경:

- **6자유도 비행** — 연속 루프 폐쇄(SLC) 오토파일럿(뱅크/피치/헤딩/고도/속도),
  안정성 증강(SAS) 수동 조종, 활주로 지상활주 이륙.
- **환경** — 정상풍 + 시드 Dryden 난류 (대기속도 ≠ 지상속도).
- **고장 주입** — 센서 6종(자이로/가속도/지자기/기압/피토/GPS: 고정/두절/편차) +
  서보 4채널(고착/유동/저속). GCS와 화면 양쪽에 표시.
- **추정기** — 게이팅 항법 필터(FDE 포함), Mahony 자세 추정(자이로 바이어스 추정),
  온보드 바람 추정. **제어 경로 전체가 추정치로 비행** (진값은 접지 스위치 하나뿐).
- **GCS 루프** — MAVLink **v1·v2**로 QGroundControl 연동: 텔레메트리, ARM/모드/
  이륙/착륙/RTL, 미션 업로드·비행, 실시간 파라미터, 센서 헬스·STATUSTEXT, 배터리,
  EKF 상태, 바람, 가상 조이스틱.
- **HILS 벤치** — 선언적 시나리오(시각별 고장 주입 + 합격 판정)를 결정론적으로 재생.
  브라우저에서 `window.__hils`, CLI에서 `npm run hils`.
- **외부 제어기 모드** — 플랜트 모드(외부 비행제어기가 UDP 락스텝으로 루프 폐쇄),
  ArduPilot SITL JSON 백엔드 어댑터.
- **화면 도구** — 채널 모니터(`C` 키: 납품 채널 맵을 실시간 값으로 표시), 원클릭
  고장 시나리오 버튼, 엔지니어링 콘솔(`E` 키: 상태벡터·추정 오차·고장 벤치·차트).

## 실행·빌드

```bash
# 브라우저 시뮬레이터 (단독)
python3 -m http.server 8123        # → http://localhost:8123  (T 이륙 · Space 시동 · C 채널 · E 콘솔)

# MAVLink 브리지 포함 (시뮬 서빙 + QGroundControl UDP 14550 연동)
npm run bridge                     # → http://localhost:8765

# 시험·검증 게이트
npm test                           # 유닛 테스트
npm run coverage                   # JS 참조 모델 구조적 커버리지
npm run hils                       # HILS 시나리오 벤치
node tests/gcs-loop-check.mjs      # MAVLink 패킷 왕복 (브리지 + 모의 GCS)

# 네이티브 모델: 빌드 + 검증 + 패키징
make -C native golden              # C 코어를 JS 골든 벡터와 교차검증
make -C native coverage            # C 코어 라인 커버리지 (100%)
make -C native mcdc CC=gcc-14      # C 코어 MC/DC 조건 커버리지 (gcc ≥ 14 필요)
make -C native fmu                 # native/fdm-uav.fmu 빌드·패키징
make -C native so                  # libfdm.so 빌드
make -C native spec                # native/INTERFACE.md 재생성

# 외부 제어기 모드
npm run plant                      # 플랜트 모드: 외부 FC용 UDP JSON 락스텝
npm run sitl                       # ArduPilot SITL JSON 백엔드 어댑터 (UDP 9002)
```

## 저장소 구조

```
src/          참조 모델 + 브라우저 시뮬: physics, wind, sensors, estimator,
              autopilot, missions, telemetry, params, battery, vehicle(공유 코어),
              hils(시나리오 러너), engineering(콘솔), hilspanel(채널 모니터), scene, main
bridge/       server.mjs (HTTP/SSE ↔ UDP), mavlink.mjs (v1/v2), plant.mjs, sitl.mjs
native/       fdm.c/.h (C99 코어), fmi2_model.c (FMU), nivs_model.c (VeriStand),
              channels.json (단일 진실 소스), gen-*.mjs (생성기),
              golden-check.c / cov-driver.c, coverage.sh / mcdc.sh, Makefile
tests/        *.test.mjs (유닛 89건) + *-check.mjs / hils-run.mjs (통합 게이트)
.github/      CI 워크플로 (매 push마다 전 게이트 실행)
```

## 문서

- `PRD.md` — 사양서(최상위 권위), 마일스톤 M0–M28.
- `native/INTERFACE.md` — 모델 인터페이스 명세 (자동 생성).
- `COMPLIANCE-DO331.md` — DO-331 트레이서빌리티 매핑 (미비 항목 명시).
- `Log.md` — 마일스톤별 개발 로그.
- `CLAUDE.md` — 엔지니어링 운영 규범.

## 범위와 정직성

이 모델은 **소형 무인기급** 비행 모델이며 고성능 제트기가 아닙니다. 화면은 게임이
아닌 엔지니어링 시각화입니다. `COMPLIANCE-DO331.md`는 **트레이서빌리티 매핑**이지
인증 주장이 아닙니다 — 남은 항목(도구 적격성, 검증 독립성, 정형 형상관리)은 문서에
명시되어 있으며, 실제 인증 사업에서 목표 보증 등급에 맞춰 해소합니다.
