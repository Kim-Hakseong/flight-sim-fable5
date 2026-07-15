# fdm-uav — 모델 인터페이스 명세 (Model Interface Specification)

> AUTO-GENERATED from `native/channels.json` (`make -C native spec`). Do not edit.
>
> Aerosonde-class small-UAV integrated plant model (6-DOF + actuators + Dryden environment), golden-validated against the JS reference simulator.

## 1. 모델 인터페이스 구성도

```
                 ┌──────────────────────────────────┐
  제어기(DUT)     │        fdm-uav  플랜트 모델        │      VeriStand / GCS
  ───────────►  │  6-DOF 강체 + 액추에이터(고장주입)   │  ───────────►
  Cmd_* (4ch)    │  ISA 대기 + Dryden 난류(시드 결정론) │  Pos/Vel (NED)
  Flt_* (4ch)    │  지상 모델 (WoW)                   │  Att/Rate (FRD)
  Env_* (3ch)    │  base rate 60 Hz 고정 스텝       │  Air data · Act_* · WoW
                 └──────────────────────────────────┘
```

- 스케일링/캘리브레이션/버스 프로토콜(1553B 등)은 VeriStand 채널단 처리 전제 (모델은 SI 단위).
- 검증: JS 레퍼런스 대비 골든 교차검증 CI 게이트 (`make -C native golden`, 점단위 1e-12).

## 2. Inports (제어기/환경 → 모델)

| 신호 | 단위 | 범위 | 정보 |
|---|---|---|---|
| Cmd_Aileron | - | -1 ~ 1 | aileron command, fraction of ±25.2 deg |
| Cmd_Elevator | - | -1 ~ 1 | elevator command (+ = trailing edge down = nose down) |
| Cmd_Rudder | - | -1 ~ 1 | rudder command (− = nose right) |
| Cmd_Throttle | - | 0 ~ 1 | throttle command |
| Env_WindN | m/s | -20 ~ 20 | steady wind TO north |
| Env_WindE | m/s | -20 ~ 20 | steady wind TO east |
| Env_Turb | - | 0 ~ 3 | Dryden intensity scale (0 = calm) |
| Flt_Aileron | enum | 0 ~ 3 | servo fault: 0 none / 1 jam / 2 floating / 3 slow |
| Flt_Elevator | enum | 0 ~ 3 | servo fault: 0 none / 1 jam / 2 floating / 3 slow |
| Flt_Rudder | enum | 0 ~ 3 | servo fault: 0 none / 1 jam / 2 floating / 3 slow |
| Flt_Throttle | enum | 0 ~ 3 | servo fault: 0 none / 1 jam / 2 floating / 3 slow |
| Sim_Reset | bool | 0 ~ 2 | rising >0.5 re-initializes; 1 = ground boot, 2 = airborne trim |

## 3. Outports (모델 → 제어기/계측)

| 신호 | 단위 | 범위 | 정보 |
|---|---|---|---|
| Pos_N | m | — | position north of home |
| Pos_E | m | — | position east of home |
| Pos_D | m | — | position down (negative = altitude) |
| Vel_N | m/s | — | ground velocity north |
| Vel_E | m/s | — | ground velocity east |
| Vel_D | m/s | — | ground velocity down |
| Att_Roll | rad | — | roll, right + |
| Att_Pitch | rad | — | pitch, up + |
| Att_Yaw | rad | — | yaw, 0 = north, east + |
| Rate_P | rad/s | — | body roll rate (FRD) |
| Rate_Q | rad/s | — | body pitch rate (FRD) |
| Rate_R | rad/s | — | body yaw rate (FRD) |
| Air_Va | m/s | — | true airspeed |
| Air_Alpha | rad | — | angle of attack |
| Air_Beta | rad | — | sideslip |
| Act_Aileron | rad | — | actual aileron deflection |
| Act_Elevator | rad | — | actual elevator deflection |
| Act_Rudder | rad | — | actual rudder deflection |
| Act_Throttle | - | — | actual throttle |
| WoW | bool | — | weight on wheels discrete |

## 4. Parameters (airframe re-targeting, FMI vref 200+)

기체 재타깃용 파라미터 — 기본값은 golden 검증된 Aerosonde급 세트이며, FMU에서
재컴파일 없이 교체 가능(예: 고객 기체의 Datcom/AVL 공력 DB).

| 파라미터 | 단위 | 기본값 | 정보 |
|---|---|---|---|
| mass | kg | 13.5 | airframe mass |
| Jx | kg·m² | 0.8244 | roll inertia |
| Jy | kg·m² | 1.135 | pitch inertia |
| Jz | kg·m² | 1.759 | yaw inertia |
| wingS | m² | 0.55 | wing reference area |
| wingB | m | 2.9 | wing span |
| wingC | m | 0.19 | mean chord |
| CL0 | - | 0.28 | lift coeff at zero AoA |
| CLa | 1/rad | 3.45 | lift slope |
| CLde | 1/rad | 0.36 | lift per elevator |
| CD0 | - | 0.03 | parasitic drag |
| Kind | - | 0.0231 | induced drag factor |
| Cm0 | - | -0.02338 | pitch moment at zero AoA |
| Cma | 1/rad | -0.38 | pitch stiffness |
| Cmq | 1/rad | -3.6 | pitch damping |
| Cmde | 1/rad | -0.5 | elevator power |
| CYb | 1/rad | -0.98 | side force per sideslip |
| CYdr | 1/rad | 0.19 | side force per rudder |
| Clb | 1/rad | -0.12 | dihedral effect |
| Clp | 1/rad | -0.26 | roll damping |
| Clr | 1/rad | 0.14 | roll per yaw rate |
| Clda | 1/rad | 0.13 | aileron power |
| Cldr | 1/rad | 0.008 | roll per rudder |
| Cnb | 1/rad | 0.25 | weathercock stability |
| Cnp | 1/rad | 0.022 | yaw per roll rate |
| Cnr | 1/rad | -0.35 | yaw damping |
| Cnda | 1/rad | -0.011 | adverse yaw |
| Cndr | 1/rad | -0.069 | rudder power |
| sProp | m² | 0.2027 | prop disk area |
| cProp | - | 1 | prop efficiency coeff |
| kMotor | - | 50 | motor constant |
| maxThrustN | N | 60 | static thrust cap |
| muRoll | - | 0.03 | rolling resistance |
| muBrake | - | 0.22 | brake friction (idle throttle) |
| maxDef | rad | 0.44 | surface deflection limit |
| actTau | s | 0.05 | surface actuator time constant |
| thrTau | s | 0.4 | throttle time constant |
| alphaClamp | rad | 0.3 | lift-model AoA clamp |

## 5. 좌표계·규약

- 위치/속도: NED (North-East-Down), home 원점. 자세: 항공 오일러 (roll right +, pitch up +, yaw 0=북/동 +). 각속도: 기체 FRD.
- 결정론: 동일 입력 시퀀스 + 동일 리셋 → 비트 동일 출력 (난류는 시드 mulberry32 Gauss–Markov).
- Sim_Reset 라이징 에지: 1 = 지상 콜드 부팅(활주로), 2 = 공중 트림 (Va 30 m/s, 120 m).
