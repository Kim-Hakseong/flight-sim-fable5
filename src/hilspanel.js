// HILS overlay: on-screen readout of the sim as a HILS plant. RENDER-ONLY.
//  - badge: model identity (name · base rate · determinism).
//  - channel monitor (KeyC): the sim's I/O signals — control/environment inports
//    and state outports — in the NED/FRD convention a HILS rig expects. The signal
//    list is inlined here (single source of truth); values are read live from the
//    vehicle each frame.
//  - scenario toggles: one-click fault injection through the SAME surfaces the
//    tests and the engineering console use (injectFault / servo faults / params).

import { airData, toFRD } from './physics.js';
import { eulerFromQuat } from './telemetry.js';

// Signal map (name + unit), grouped inports (controller/env → model) and outports
// (model → instrumentation). Names match the keys channelValues() produces.
const CHANNELS = {
  inports: [
    { name: 'Cmd_Aileron', unit: '' }, { name: 'Cmd_Elevator', unit: '' },
    { name: 'Cmd_Rudder', unit: '' }, { name: 'Cmd_Throttle', unit: '' },
    { name: 'Env_WindN', unit: 'm/s' }, { name: 'Env_WindE', unit: 'm/s' },
    { name: 'Env_Turb', unit: '×' },
    { name: 'Flt_Aileron', unit: '' }, { name: 'Flt_Elevator', unit: '' },
    { name: 'Flt_Rudder', unit: '' }, { name: 'Flt_Throttle', unit: '' },
    { name: 'Sim_Reset', unit: '' },
  ],
  outports: [
    { name: 'Pos_N', unit: 'm' }, { name: 'Pos_E', unit: 'm' }, { name: 'Pos_D', unit: 'm' },
    { name: 'Vel_N', unit: 'm/s' }, { name: 'Vel_E', unit: 'm/s' }, { name: 'Vel_D', unit: 'm/s' },
    { name: 'Att_Roll', unit: 'rad' }, { name: 'Att_Pitch', unit: 'rad' }, { name: 'Att_Yaw', unit: 'rad' },
    { name: 'Rate_P', unit: 'rad/s' }, { name: 'Rate_Q', unit: 'rad/s' }, { name: 'Rate_R', unit: 'rad/s' },
    { name: 'Air_Va', unit: 'm/s' }, { name: 'Air_Alpha', unit: 'rad' }, { name: 'Air_Beta', unit: 'rad' },
    { name: 'Act_Aileron', unit: '' }, { name: 'Act_Elevator', unit: '' },
    { name: 'Act_Rudder', unit: '' }, { name: 'Act_Throttle', unit: '' },
    { name: 'WoW', unit: '' },
  ],
};

const CSS = `
#hilsbadge { position: fixed; bottom: 64px; left: 50%; transform: translateX(-50%);
  z-index: 15; color: #d8e2f2; font: 11px/1.5 ui-monospace, Menlo, monospace;
  background: rgba(10,14,22,0.78); border: 1px solid #2b3345; border-radius: 6px;
  padding: 4px 14px; text-align: center; pointer-events: none; }
#hilsbadge b { color: #8fd3ff; }
#hilsbadge .ok { color: #9fe870; }
#chanmon { position: fixed; top: 150px; left: 10px; z-index: 15; width: 335px;
  color: #cfd6e4; font: 10.5px/1.4 ui-monospace, Menlo, monospace;
  background: rgba(10,14,22,0.82); border: 1px solid #2b3345; border-radius: 8px;
  padding: 8px 10px; display: none; max-height: calc(100vh - 260px); overflow-y: auto; }
#chanmon.open { display: block; }
#chanmon h3 { margin: 4px 0 2px; font-size: 10.5px; color: #8fd3ff; letter-spacing: 1px; }
#chanmon table { border-collapse: collapse; width: 100%; }
#chanmon td { padding: 0 4px; white-space: nowrap; }
#chanmon td.v { text-align: right; color: #9fe870; min-width: 72px; }
#chanmon td.u { color: #8a93a6; }
#scnbar { position: fixed; bottom: 34px; left: 50%; transform: translateX(-50%);
  z-index: 15; display: flex; gap: 5px; flex-wrap: wrap; justify-content: center;
  max-width: 92vw; }
#scnbar button { font: 10.5px ui-monospace, Menlo, monospace; color: #cfd6e4;
  background: rgba(20,26,40,0.85); border: 1px solid #39445e; border-radius: 5px;
  padding: 3px 9px; cursor: pointer; }
#scnbar button:hover { background: #35405c; }
#scnbar button.on { background: #5a2430; border-color: #a04050; color: #ffb0b0; }
#scnbar button.clr { border-color: #3f6b46; color: #9fe870; }
`;

// Fault-scenario toggles: apply()/clear() drive the public injection surfaces;
// active(veh) reads back the vehicle so button state can never lie.
const SCENARIOS = [
  { id: 'gps', label: 'GPS DROP', apply: (w) => w.injectFault('gps', 'dropout'), clear: (w) => w.clearFault('gps'), active: (v) => v.sensors.faults.gps },
  { id: 'gyro', label: 'GYRO BIAS', apply: (w) => w.injectFault('gyro', 'bias', { bias: 0.05 }), clear: (w) => w.clearFault('gyro'), active: (v) => v.sensors.faults.gyro },
  { id: 'mag', label: 'MAG 30°', apply: (w) => w.injectFault('mag', 'bias', { bias: 30 }), clear: (w) => w.clearFault('mag'), active: (v) => v.sensors.faults.mag },
  { id: 'pitot', label: 'PITOT DROP', apply: (w) => w.injectFault('pitot', 'dropout'), clear: (w) => w.clearFault('pitot'), active: (v) => v.sensors.faults.pitot },
  { id: 'sja', label: 'AIL JAM', apply: (w) => w.injectServoFault('da', 'jam'), clear: (w) => w.clearServoFault('da'), active: (v) => v.servoFaults.da },
  { id: 'ses', label: 'ELV SLOW', apply: (w) => w.injectServoFault('de', 'slow'), clear: (w) => w.clearServoFault('de'), active: (v) => v.servoFaults.de },
  {
    id: 'turb', label: 'TURB ×2.5',
    apply: (w) => w.__command({ type: 'param', id: 'WND_TRB', value: 2.5 }),
    clear: (w) => w.__command({ type: 'param', id: 'WND_TRB', value: 1 }),
    active: (v) => v.params.WND_TRB > 2,
  },
];

export function createHilsPanel({ getVehicle }) {
  const style = document.createElement('style');
  style.textContent = CSS;
  document.head.appendChild(style);

  const badge = document.createElement('div');
  badge.id = 'hilsbadge';
  badge.innerHTML =
    '<b>fdm-uav</b> 6-DOF flight model · 60 Hz fixed-step · ' +
    '<span class="ok">deterministic</span>';
  document.body.appendChild(badge);

  // --- channel monitor (built from the inlined CHANNELS signal map) -------------
  const mon = document.createElement('div');
  mon.id = 'chanmon';
  document.body.appendChild(mon);
  const rows = { in: [], out: [] }; // [{name, cell}]
  for (const [key, title, list] of [['in', 'INPORTS (제어기/환경 → 모델)', CHANNELS.inports], ['out', 'OUTPORTS (모델 → 계측)', CHANNELS.outports]]) {
    const h = document.createElement('h3');
    h.textContent = title;
    mon.appendChild(h);
    const tbl = document.createElement('table');
    for (const c of list) {
      const tr = document.createElement('tr');
      const cell = document.createElement('td');
      cell.className = 'v';
      tr.innerHTML = `<td>${c.name}</td>`;
      tr.appendChild(cell);
      tr.insertAdjacentHTML('beforeend', `<td class="u">${c.unit}</td>`);
      tbl.appendChild(tr);
      rows[key].push({ name: c.name, cell });
    }
    mon.appendChild(tbl);
  }

  // --- scenario toggle bar ------------------------------------------------------
  const bar = document.createElement('div');
  bar.id = 'scnbar';
  const btns = new Map();
  for (const sc of SCENARIOS) {
    const b = document.createElement('button');
    b.textContent = sc.label;
    b.onclick = () => (sc.active(getVehicle()) ? sc.clear(window) : sc.apply(window));
    bar.appendChild(b);
    btns.set(sc.id, b);
  }
  const clr = document.createElement('button');
  clr.className = 'clr';
  clr.textContent = 'CLEAR ALL';
  clr.onclick = () => SCENARIOS.forEach((sc) => sc.clear(window));
  bar.appendChild(clr);
  document.body.appendChild(bar);

  window.addEventListener('keydown', (e) => {
    if (e.code === 'KeyC') mon.classList.toggle('open');
  });

  // Channel values exactly as the wrappers map them (ours → NED / FRD).
  function channelValues(v) {
    const s = v.state;
    const e = eulerFromQuat(s.quat);
    const [p, q, r] = toFRD(s.omega);
    const ad = airData(s.quat, s.vel, v.windWorld);
    const F = v.servoFaults;
    const fnum = (f) => (!f ? 0 : f.type === 'jam' ? 1 : f.type === 'floating' ? 2 : 3);
    return {
      Cmd_Aileron: v.lastControls.aileron, Cmd_Elevator: v.lastControls.elevator,
      Cmd_Rudder: v.lastControls.rudder, Cmd_Throttle: v.lastControls.throttle,
      Env_WindN: v.params.WND_N_MS, Env_WindE: v.params.WND_E_MS, Env_Turb: v.params.WND_TRB,
      Flt_Aileron: fnum(F.da), Flt_Elevator: fnum(F.de), Flt_Rudder: fnum(F.dr), Flt_Throttle: fnum(F.dt),
      Sim_Reset: 0,
      Pos_N: -s.pos[2], Pos_E: s.pos[0], Pos_D: -s.pos[1],
      Vel_N: -s.vel[2], Vel_E: s.vel[0], Vel_D: -s.vel[1],
      Att_Roll: e.roll, Att_Pitch: e.pitch, Att_Yaw: e.yaw,
      Rate_P: p, Rate_Q: q, Rate_R: r,
      Air_Va: ad.Va, Air_Alpha: ad.alpha, Air_Beta: ad.beta,
      Act_Aileron: s.act.da, Act_Elevator: s.act.de, Act_Rudder: s.act.dr, Act_Throttle: s.act.dt,
      WoW: s.pos[1] <= 0.5 ? 1 : 0,
    };
  }

  let tick = 0;
  return {
    render() {
      const v = getVehicle();
      for (const sc of SCENARIOS) btns.get(sc.id).classList.toggle('on', !!sc.active(v));
      if (++tick % 6 !== 0) return; // 10 Hz DOM updates are plenty
      if (rows && mon.classList.contains('open')) {
        const vals = channelValues(v);
        for (const list of [rows.in, rows.out]) {
          for (const rrow of list) {
            const x = vals[rrow.name];
            rrow.cell.textContent = x === undefined ? '—' : (Math.abs(x) >= 1000 ? x.toFixed(1) : x.toFixed(3));
          }
        }
      }
    },
  };
}
