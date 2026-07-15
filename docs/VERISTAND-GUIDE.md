# fdm-uav — NI VeriStand 통합 절차서

> 대상: `fdm-uav.fmu`(FMI 2.0 Co-Simulation) 또는 VeriStand Model Framework `.so`를
> 실제 VeriStand 리그에 올리는 엔지니어.
> **정직 고지**: 본 모델은 CI에서 실제 `fmi2` ABI 구동까지 검증되었으나, **실물
> VeriStand 리그 탑재는 아직 1회도 수행 전**입니다. 본 절차서의 [확인점] 표시는
> 첫 탑재에서 결과를 기록해야 할 지점입니다 — 결과를 개발팀에 회신해 주세요.

## 0. 사전 요건

| 항목 | 요건 | 근거 |
|---|---|---|
| VeriStand | **2019 이상** (FMI 2.0 Co-Sim, Windows + PXI Linux RT 지원) | NI KB: "NI VeriStand 2019 and higher versions enable configuration and execution of FMI 2.0 Co-Simulation models on Windows and NI PXI Linux Real-Time systems." |
| RT 타깃 | PXI **Linux Real-Time** (FMU 내 바이너리가 `binaries/linux64/`) | FMU 패키징 구조 |
| 모델 base rate | **60 Hz** (1/60 s 고정 스텝) | 모델은 더 큰 통신 스텝을 받으면 내부에서 1/60 s로 sub-step하므로 rate 불일치에도 deterministic |

## 1. FMU 확보

두 경로 중 택일:

```bash
# (a) 저장소에서 직접 빌드 — Linux 호스트/컨테이너에서 (linux64 ELF .so가 필요)
make -C native fmu           # → native/fdm-uav.fmu

# (b) GitHub CI 아티팩트 — 매 push마다 Linux 러너가 빌드·검증한 FMU
#     GitHub → Actions → 최근 성공 run → Artifacts → "fdm-uav-fmu"
```

> **[확인점 1]** macOS에서 `make fmu` 하면 `.so`가 Mach-O가 되어 RT 타깃에서 로드
> 실패합니다. 반드시 (b) CI 아티팩트 또는 Linux 빌드를 사용할 것.

## 2. VeriStand 프로젝트에 모델 추가

1. System Explorer → 대상 Target → **Simulation Models** 우클릭 → 모델 추가에서
   `fdm-uav.fmu` 선택.
2. Execution: 모델 rate를 **60 Hz** 로 (또는 PCL rate의 정수 분주). Decimation 사용
   시에도 모델은 내부 sub-step으로 1/60 s를 유지합니다.
3. Deploy 후 Model Parameters/Channels 트리에 아래가 보여야 정상:
   - **Inports 12ch** — `Cmd_Aileron … Sim_Reset`
   - **Outports 20ch** — `Pos_N … WoW`
   (전체 신호 정의·단위·범위: `native/INTERFACE.md`)

> **[확인점 2]** FMU 로드 실패 시 로그의 문구를 기록해 주세요. 예상 원인 순서:
> ① linux64 바이너리 아님(→ §1), ② glibc 버전(러너는 Ubuntu 최신 — 필요시 타깃과
> 맞는 이미지에서 재빌드), ③ FMI 버전 설정.

## 3. 최소 기능 점검 (Smoke Test)

수동 채널 조작만으로 5분 내 검증:

| 순서 | 조작 | 기대 결과 |
|---|---|---|
| 1 | 배포 직후 | `WoW = 1`, `Pos_D ≈ 0` (활주로 위 정지, DISARM 개념 없음 — 모델은 명령을 그대로 따름) |
| 2 | `Sim_Reset` 0→**2** (rising edge) | 공중 trim으로 점프: `Pos_D ≈ −120`, `Vel_N ≈ +30`, `Air_Va ≈ 30`, `WoW = 0` |
| 3 | `Cmd_Elevator = −0.20`, `Cmd_Throttle = 0.63` 유지 | trim 부근 수평비행 유지 (`Att_Pitch ≈ +0.056 rad`) |
| 4 | `Cmd_Throttle` → 1.0 | `Air_Va` 증가 → 상승 (`Vel_D` 음수 방향) |
| 5 | `Cmd_Aileron` = +0.3 잠깐 | `Att_Roll` +로 증가(우측 bank), `Rate_P` 양수 스파이크 |
| 6 | `Flt_Aileron` = 1 (jam) 후 5 재시도 | `Act_Aileron`이 현재값에 고착, 명령 무반응 |
| 7 | `Env_Turb` = 2.0 | 자세·속도 채널이 난류로 요동 (seed 고정 → 같은 시퀀스 재현) |
| 8 | `Sim_Reset` 0→**1** | 지상 cold boot 복귀 (`WoW = 1`) |

> **[확인점 3]** 4~5단계에서 채널 부호가 표와 다르면 즉시 회신 — NED/FRD 매핑
> 문제이므로 채널 스케일로 덮지 말고 모델 쪽에서 수정해야 합니다.

## 3.5 Airframe 파라미터 (재타깃)

FMU는 **38개 airframe parameter**(mass/inertia, 공력계수 전체, prop, actuator,
ground; `native/INTERFACE.md` §4)를 노출합니다. VeriStand의 Model Parameters에서
값을 바꾸면 **재컴파일 없이** 다른 기체로 재타깃됩니다 — 예: 고객 기체의
Datcom/AVL 공력 DB. 기본값은 golden 검증된 Aerosonde급 세트입니다.

> **[확인점 3.5]** Model Parameters 트리에 38개 파라미터가 보이고, `mass`를
> 20.25로 올리면 trim 명령 유지 시 침하가 관측되어야 합니다 (CI에서 실 ABI로
> 동일 시험 통과: 15초에 −117 m).

## 4. 실시간 성능

모델 1스텝은 순수 산술 ~수백 FLOP 수준으로, 60 Hz PCL에서 CPU 부하는 무시 가능할
것으로 예상합니다(데스크톱에서 360초 시뮬 ≈ 70 ms).

> **[확인점 4]** VeriStand의 model execution time 채널에서 스텝당 소요시간을 기록해
> 주세요 (기대: << 1 ms).

## 5. 대안 경로 — Model Framework `.so`

FMU 경로에 문제가 있으면 VeriStand 전통 방식으로:

```bash
# 고객 툴체인(NI SDK 필요)에서:
gcc -std=c99 -O2 -fPIC -shared -I<NIVS_SDK> \
    native/nivs_model.c native/fdm.c <NIVS_SDK>/ni_modelframework.c \
    -o fdm-uav.so -lm
```

`nivs_model.c`는 동일한 channel map(같은 `channels.json`에서 생성·검증)을 노출합니다.
CI는 이 파일을 타입 스텁으로 컴파일 체크만 하므로, **실제 NI SDK 헤더로의 첫 빌드
결과도 [확인점 5]** 입니다.

## 6. 문제 발생 시 함께 보낼 것

- VeriStand 버전 / RT 타깃 OS·버전
- 로드/배포 로그의 오류 문구 원문
- §3 표에서 몇 번째 단계가 어떻게 다르게 나왔는지
