// Model interface specification generator — one source of truth (channels.json)
// feeds both the VeriStand wrapper (nivs_model.c structs, same order) and this
// document. Format follows the customer-facing spec style of the reference
// material (모델 인터페이스 구성도 + 신호 테이블):  node gen-spec.mjs
import { readFileSync, writeFileSync } from 'node:fs';

const ch = JSON.parse(readFileSync(new URL('./channels.json', import.meta.url)));

// Consistency gate: the wrapper must declare exactly these fields in this order.
const src = readFileSync(new URL('./nivs_model.c', import.meta.url), 'utf8');
for (const [list, name] of [[ch.inports, 'Inports'], [ch.outports, 'Outports']]) {
  const block = src.match(new RegExp(`typedef struct \\{([^}]*)\\} ${name};`, 's'))[1];
  const declared = [...block.matchAll(/\b([A-Z][A-Za-z0-9_]*)\b(?=[,;])/g)].map((m) => m[1]);
  const wanted = list.map((c2) => c2.name);
  if (JSON.stringify(declared) !== JSON.stringify(wanted)) {
    console.error(`MISMATCH in ${name}:\n  wrapper: ${declared.join(', ')}\n  json:    ${wanted.join(', ')}`);
    process.exit(1);
  }
}

const row = (c2) =>
  `| ${c2.name} | ${c2.unit} | ${c2.min !== undefined ? `${c2.min} ~ ${c2.max}` : '—'} | ${c2.desc} |`;

const md = `# ${ch.model} — 모델 인터페이스 명세 (Model Interface Specification)

> AUTO-GENERATED from \`native/channels.json\` (\`make -C native spec\`). Do not edit.
>
> ${ch.description}

## 1. 모델 인터페이스 구성도

\`\`\`
                 ┌──────────────────────────────────┐
  제어기(DUT)     │        fdm-uav  플랜트 모델        │      VeriStand / GCS
  ───────────►  │  6-DOF 강체 + 액추에이터(고장주입)   │  ───────────►
  Cmd_* (4ch)    │  ISA 대기 + Dryden 난류(시드 결정론) │  Pos/Vel (NED)
  Flt_* (4ch)    │  지상 모델 (WoW)                   │  Att/Rate (FRD)
  Env_* (3ch)    │  base rate ${ch.base_rate_hz} Hz 고정 스텝       │  Air data · Act_* · WoW
                 └──────────────────────────────────┘
\`\`\`

- 스케일링/캘리브레이션/버스 프로토콜(1553B 등)은 VeriStand 채널단 처리 전제 (모델은 SI 단위).
- 검증: JS 레퍼런스 대비 골든 교차검증 CI 게이트 (\`make -C native golden\`, 점단위 1e-12).

## 2. Inports (제어기/환경 → 모델)

| 신호 | 단위 | 범위 | 정보 |
|---|---|---|---|
${ch.inports.map(row).join('\n')}

## 3. Outports (모델 → 제어기/계측)

| 신호 | 단위 | 범위 | 정보 |
|---|---|---|---|
${ch.outports.map(row).join('\n')}

## 4. 좌표계·규약

- 위치/속도: NED (North-East-Down), home 원점. 자세: 항공 오일러 (roll right +, pitch up +, yaw 0=북/동 +). 각속도: 기체 FRD.
- 결정론: 동일 입력 시퀀스 + 동일 리셋 → 비트 동일 출력 (난류는 시드 mulberry32 Gauss–Markov).
- Sim_Reset 라이징 에지: 1 = 지상 콜드 부팅(활주로), 2 = 공중 트림 (Va ${30} m/s, 120 m).
`;
writeFileSync(new URL('./INTERFACE.md', import.meta.url), md);
console.log(`INTERFACE.md: ${ch.inports.length} inports, ${ch.outports.length} outports (wrapper order verified)`);
