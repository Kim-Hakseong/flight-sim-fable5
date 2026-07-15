# DO-331 트레이서빌리티 매핑 — fdm-uav 플랜트 모델

> **문서 성격 (반드시 먼저 읽을 것).**
> 이 문서는 본 프로젝트의 개발·검증 산출물을 **DO-331(Model-Based Development and
> Verification Supplement to DO-178C/DO-278A)의 목표(objective)·개념에 *매핑*** 한 것이다.
> **인증(certification) 획득을 주장하지 않으며**, 툴 적격성(DO-330)·검증 독립성·
> 구성형상관리(CM) 프로세스·품질보증(SQA) 기록 등 실제 인증에 필요한 다수 요소는
> 아직 갖추지 않았다(§6 갭 참조). 목적은 두 가지다:
> 1. 고객(체계업체·ADD)이 본 모델을 인증 대상 체계에 통합할 때, **어떤 트레이서빌리티
>    근거가 이미 존재하고 무엇을 추가로 만들어야 하는지**를 한눈에 보이게 한다.
> 2. 본 모델의 차별점 — "검증된 모델 + 자동 트레이서빌리티" (MARKET.md §6) — 을
>    구체적 증거로 뒷받침한다.

## 1. 모델 분류 (DO-331 §MB.1.6)

| 항목 | 본 프로젝트 |
|---|---|
| 모델 종류 | **Design Model** (시뮬레이션/플랜트 모델). HILS 상에서 제어기(DUT)의 검증에 사용되는 항공기 동역학·환경 모델. |
| 모델링 표기 | (a) 실행 가능 참조 모델 = vanilla ES module JavaScript (`src/`), (b) 배포 모델 = 이식성 C99 (`native/fdm.c`) + FMI 2.0 FMU / NI VeriStand 래퍼 |
| 모델 표준 | `CLAUDE.md §3` (좌표계·순수함수·명명 규칙), `native/fdm.c` 헤더(연산 순서 보존 규칙) |
| 실행 의미 | 고정 스텝(1/60 s), 완전 결정론(시드 PRNG), 벽시계·`Math.random` 금지 (`CLAUDE.md §0.5`) |

**참조 모델 vs 배포 모델의 관계 = 본 트레이서빌리티의 핵심.** JS 참조 모델이 "골든
기준"이고, C/FMU 배포 모델은 골든 벡터로 교차검증되어 **두 표현이 비트 수준으로 동치임을
CI가 매 커밋 증명**한다(§4-C, §5). 이는 DO-331이 요구하는 "모델과 그로부터 생성된
산출물 간의 일관성" 근거에 직접 대응한다.

## 2. 요구사항 계층 (DO-331 §MB.5.1 트레이스 대상)

| 계층 | 본 프로젝트 산출물 | 위치 |
|---|---|---|
| 시스템/상위 요구 (HLR 등가) | 마일스톤 검증기준 (`*Verify:*` 절) | `PRD.md §4` (M0–M25) |
| 모델 인터페이스 요구 | 채널 명세 (Inport/Outport, 범위·단위) | `native/channels.json` → `native/INTERFACE.md`(자동생성) |
| 설계 모델 (Design Model) | 물리·항법·환경 모듈 | `src/*.js`, `native/fdm.c` |
| 구현 (Source/Object) | C99 코어 + FMU/VeriStand 래퍼 | `native/{fdm.c, fmi2_model.c, nivs_model.c}` |
| 검증 케이스·절차·결과 | 유닛/골든/통합 테스트 + CI 로그 | `tests/`, `native/*-check.*`, GitHub Actions |

## 3. 트레이서빌리티 매트릭스 (요구 → 모델요소 → 검증증거)

각 행: **요구(마일스톤)** → **모델 요소(모듈/함수)** → **검증 케이스(파일·건수)** →
**CI 게이트**. 유닛 테스트 총 **89건** 전수 매핑.

| ID | 요구 (모델 능력) | 모델 요소 | 검증 케이스 | CI 게이트 |
|---|---|---|---|---|
| R-DET | 결정론·고정스텝 재현성 | `main.js`/`vehicle.js` `__advance`; `prng.js` | `physics.test`(determinism), `browser-check`(재현+폴트재현) | Unit, Browser |
| R-6DOF | 강체 6-DOF 힘·모멘트·관성 | `physics.js` `stepAircraft/forcesMoments`; `fdm.c` | `physics.test`(12) | Unit, Native golden |
| R-AERO | 안정미계수 공력·트림 | `physics.js AC/TRIM`; `fdm.c` | `physics.test`(trim, dynamic modes) | Unit, Native golden |
| R-ACT | 액추에이터 1차지연·편각제한 | `physics.js stepActuators` | `physics.test`(actuators) | Unit |
| R-SVF | 서보 고장(jam/float/slow) | `physics.js stepActuators(faults)` | `servo.test`(6) | Unit |
| R-ATM | ISA 대기 밀도 | `physics.js airDensity`; `fdm.c` | `physics.test`(atmosphere) | Unit, Native golden |
| R-WIND | 정상풍 + Dryden 난류(시드) | `wind.js`; `fdm.c step_wind` | `wind.test`(6) | Unit, Native golden |
| R-GND | 지상 모델·제동·지상활주 | `physics.js` ground; `autopilot.js` takeoff | `takeoff.test`(4) | Unit |
| R-SNS | 센서 오차·폴트 주입 | `sensors.js` | `sensors.test`(6) | Unit |
| R-EST | 항법 추정기(위치/속도, FDE) | `estimator.js` | `estimator.test`(4) | Unit |
| R-ATT | 자세 추정(Mahony+bias) | `estimator.js` att | `attitude.test`(6) | Unit |
| R-NAV | 추정치 폐루프 비행 | `vehicle.js`+`autopilot.js` | `nav-loop.test`(5) | Unit |
| R-WEST | 온보드 바람 추정·크랩 | `estimator.js` windEst | `wind-est.test`(3) | Unit |
| R-GDN | 유도(SLC 오토파일럿·미션) | `autopilot.js`, `missions.js` | `autopilot.test`(9), `missions.test`(6) | Unit |
| R-GCS | MAVLink 텔레메트리·명령·미션·파라미터 | `bridge/mavlink.mjs`, `telemetry.js`, `params.js` | `mavlink.test`(10), `telemetry.test`(5), `params.test`(3) | Unit, GCS loop |
| R-HILS | 시나리오 벤치(폴트→판정) | `hils.js` | `hils.test`(4) + 벤치 7시나리오 | Unit, HILS bench |
| R-PLANT | 외부 FC 락스텝 플랜트 | `bridge/plant.mjs` | `plant-check`(외부 FC 통합) | Plant check |
| R-SITL | ArduPilot SITL JSON 어댑터 | `bridge/sitl.mjs` | `sitl-check`(9) | SITL check |
| R-PORT | C99 이식·JS 골든 교차검증 | `native/fdm.c` | `golden-check.c`(점 1e-12, 궤적) | Native golden |
| R-VS | VeriStand Model Framework 래퍼 | `native/nivs_model.c` | `make nivs`(컴파일) + `gen-spec` 순서검증 | Native nivs/spec |
| R-FMU | FMI 2.0 FMU 납품형식 | `native/fmi2_model.c` | `fmi-check`(실 ABI dlopen 구동 vs 골든) | Native fmu-check |

**단일 진실 소스 트레이스**: `channels.json` → (a) VeriStand 래퍼 구조체 순서,
(b) FMU `modelDescription.xml` vref, (c) `INTERFACE.md` 신호표 — 세 산출물이 **어긋나면
생성기가 빌드를 실패**시킨다(`gen-spec.mjs`/`gen-fmu.mjs`의 일치성 게이트). 이는
"인터페이스 데이터 항목의 일관성"에 대한 **자동화된 부적합 탐지** 근거다.

## 4. DO-331 목표 테이블 매핑 (MB.A-3 ~ A-7)

상태: **E** = 근거 존재(evidence) / **P** = 부분(partial) / **G** = 갭(gap, §6).

### MB.A-3 — 소프트웨어 요구사항 프로세스 출력 검증
| 목표(취지) | 상태 | 근거 |
|---|---|---|
| 요구가 정확·일관 | E | `PRD.md §4` `*Verify:*` 기준이 각 마일스톤별로 실행 가능한 판정으로 정의됨 |
| 요구가 상위 요구로 추적 가능 | E | §3 매트릭스(요구→모델→검증), `native/channels.json`→`INTERFACE.md` |
| 알고리즘 정확성 | E | 트림 뉴턴해·동적모드 검증(`physics.test`), 골든 교차검증 |

### MB.A-4 — 설계(모델) 프로세스 출력 검증
| 목표(취지) | 상태 | 근거 |
|---|---|---|
| 설계모델이 요구를 준수 | E | §3 전 요구가 모델 요소로 매핑되고 테스트로 커버됨 |
| 모델이 모델표준 준수 | P | 좌표계·순수함수·연산순서 규칙 문서화(`CLAUDE.md §3`, `fdm.c` 헤더). 자동 정적검사(lint) 미적용 |
| 모델 간 일관성(참조↔배포) | E | 골든 교차검증(`golden-check.c`), FMU 실 ABI 검증(`fmi-check.mjs`) |

### MB.A-5 — 코딩·통합 프로세스 출력 검증
| 목표(취지) | 상태 | 근거 |
|---|---|---|
| 소스가 설계모델을 준수 | E | C99 코어가 JS 참조 궤적을 1e-12(점)/궤적허용오차 내 재현 |
| 소스가 표준 준수 | P | `-std=c99 -Wall -Wextra -Werror` 무경고 컴파일. MISRA-C 등 코딩표준 미적용 |
| 통합 산출물 정합 | E | FMU를 dlopen하여 실제 fmi2 호출 시퀀스로 구동·대조 |
| **배포 코어 구조적 커버리지** | **E** | **`fdm.c` 라인 커버리지 100% (210/210)** — `golden-check` + `cov-driver` 결합, gcov 계측, 임계값 100% CI 게이트 (`make -C native coverage`) |

### MB.A-6 — 통합 산출물 시험(시뮬레이션 케이스/절차/결과)
| 목표(취지) | 상태 | 근거 |
|---|---|---|
| 요구 기반 시험 정상범위 | E | 89 유닛 + HILS 7시나리오(정상 소티·트림·선회·이착륙) |
| 요구 기반 시험 강건성 | E | 폴트 주입(센서 6종·서보 4종), 난류, 돌풍, GPS 두절·바이어스 시나리오 |
| 시험 결과가 기대와 일치 | E | 각 게이트가 pass/fail + worst-value 리포트 산출, CI 로그 보존 |

### MB.A-7 — 검증 프로세스 결과 검증(커버리지)
| 목표(취지) | 상태 | 근거 |
|---|---|---|
| 시험이 요구를 커버 | E | §3 매트릭스로 요구↔시험 양방향 추적 |
| 모델 구조적 커버리지 분석(라인) | E | **JS 참조 모델 모듈 라인 커버리지: physics/wind/estimator/sensors/autopilot/missions/params/battery/prng = 100%** (node 내장 커버리지, `npm run coverage` 게이트). **C 배포 코어 = 100%.** 라인 커버리지는 CI 강제 |
| 모델 구조적 커버리지 분석(MC/DC) | P | 분기(branch) 커버리지는 계측·표시되나(예: physics.js 88.9%) MC/DC 정형 판정은 미적용 (§6) |
| 시뮬레이션 사례 정확·완전 | E | 골든 벡터가 참조에서 결정론적으로 재생성됨(`gen-golden.mjs`) |

## 5. 모델 커버리지·결정론 증거

- **결정론(재현성)**: 동일 입력 → 비트 동일 출력. 브라우저 게이트가 `__advance` 및
  폴트 주입 실행의 재현성을, 유닛 테스트가 전 폐루프 소티의 비트 동일성을 검증.
- **참조↔배포 등치**: 점단위 힘/모멘트 최악 **2.8e-14 rel**, 30 s 트림 궤적 **2.3e-13 m**,
  난류 20 s 도블릿 **7.1e-14**, 지상활주 8 s **4.3e-14** (호스트 측정, 허용오차 대비 10+
  자릿수 여유). FMU 실 ABI 구동: 트림 **2.27e-13 m**, 지상활주 **0.0 m**.
- **기능 커버리지 범위**: 6-DOF 전 축, 트림·동적모드(단주기/장주기/더치롤/나선),
  전 비행단계(지상활주·이륙·순항·선회·미션·RTL·접근·플레어·접지), 전 센서·서보 폴트,
  환경(정상풍·Dryden), 추정기 정상·열화, GCS 프로토콜(v1/v2, 명령·미션·파라미터).
- **구조적(라인) 커버리지 — CI 강제**:
  - 배포 C 코어 `fdm.c` = **100% (210/210 라인)**, gcov 계측 (`make -C native coverage`).
  - JS 참조 모델 모듈 = **100% 라인** (physics·wind·estimator·sensors·autopilot·
    missions·params·battery·prng), node 내장 커버리지 (`npm run coverage`). telemetry·
    vehicle의 미커버는 브라우저 전용 I/O 경로(모델 코드 아님)로, 순수 수학은 전부 커버.

## 6. 갭 — 실제 인증 착수 시 추가로 필요한 것

정직성 원칙에 따라 **아직 없는 것**을 명시한다. 인증 프로젝트는 이를 별도 활동으로 수행:

1. **DO-330 툴 적격성**: 골든 생성기·일치성 게이트·FMU 빌드는 검증 툴이나, 적격성
   인정(TQL) 절차·자료 미비. (검증 자동화의 신뢰 근거로는 유효하나 credit 청구 불가)
2. **MC/DC 커버리지**: 라인 커버리지는 계측·CI 강제(배포 C 코어 100%, JS 모델 모듈 100%),
   분기 커버리지는 계측·표시. 그러나 DAL A가 요구하는 **MC/DC(Modified Condition/Decision)
   정형 판정**은 미적용 — 전용 커버리지 도구(예: VectorCAST, LDRA) 연동 필요.
3. **검증 독립성**: 개발/검증 담당 분리·독립 리뷰 기록 없음.
4. **구성형상관리·문제보고**: git 이력은 있으나 형상식별·기준선·CR/PR 추적 프로세스 미정립.
5. **코딩 표준 적합성**: MISRA-C 등 항공용 코딩표준 정적분석 미적용.
6. **요구사항 관리 도구**: PRD의 요구가 문서형. DOORS 등 정형 요구관리·양방향 추적 도구 미연동.
7. **모델 표준 준수 자동검증**: 모델 표준이 문서형(자동 강제 부분적).

## 7. 결론

본 모델은 **요구→모델→검증의 양방향 트레이서빌리티와 참조-배포 모델 등치 증거를
CI로 자동·상시 생산**한다는 점에서, 통상 SCADE/Simulink 납품 모델이 갖지 못한 검증
기반을 이미 확보하고 있다(MARKET.md §6의 차별점). §6 갭은 고객의 인증 등급(DAL)과
체계 요구에 맞추어 인증 프로젝트에서 채워 나갈 **명시된 로드맵**이며, 본 문서가 그
출발 기준선(baseline) 역할을 한다.

> 유지관리: 모델·테스트 변경 시 §3 매트릭스와 §4 상태를 갱신할 것. `native/channels.json`
> 변경은 자동으로 `INTERFACE.md`·FMU·VeriStand 래퍼 트레이스를 재검증한다.
