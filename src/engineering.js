// HILS engineering console: live state vector, estimator-vs-truth, sensor fault
// bench, wind/battery, strip charts. RENDER-ONLY — it reads sim state and calls
// the same injectFault/clearFault surface the tests use; it never mutates physics.
// Toggle with KeyE (e.code — IME-safe). Charts sample on SIM steps (deterministic
// data), drawn at render rate.

import { SENSORS } from './sensors.js';

const CSS = `
#eng { position: fixed; top: 10px; right: 10px; z-index: 20; width: 330px;
  color: #cfd6e4; font: 11px/1.45 ui-monospace, Menlo, monospace;
  background: rgba(10, 14, 22, 0.82); border: 1px solid #2b3345; border-radius: 8px;
  padding: 10px 12px; display: none; max-height: calc(100vh - 40px); overflow-y: auto; }
#eng.open { display: block; }
#eng h3 { margin: 8px 0 3px; font-size: 11px; color: #8fd3ff; letter-spacing: 1px; }
#eng h3:first-child { margin-top: 0; }
#eng pre { margin: 0; white-space: pre; }
#eng .flt button { margin: 0 3px 3px 0; font: 10px ui-monospace, Menlo, monospace;
  background: #232b3d; color: #cfd6e4; border: 1px solid #39445e; border-radius: 4px;
  padding: 1px 6px; cursor: pointer; }
#eng .flt button:hover { background: #35405c; }
#eng .flt .bad { color: #ff8c8c; }
#eng canvas { display: block; background: #0d1119; border: 1px solid #232b3d;
  border-radius: 4px; margin: 2px 0 6px; }
`;

const DEG = 180 / Math.PI;
const HIST = 300; // 30 s at 10 Hz

function fmtVec(v, d = 1) {
  return v.map((x) => x.toFixed(d).padStart(7)).join(' ');
}

export function createEngineering({ getData, injectFault, clearFault }) {
  const style = document.createElement('style');
  style.textContent = CSS;
  document.head.appendChild(style);

  const panel = document.createElement('div');
  panel.id = 'eng';
  document.body.appendChild(panel);

  const sections = {};
  for (const id of ['state', 'nav', 'sensors', 'env']) {
    const h = document.createElement('h3');
    h.textContent = { state: 'STATE VECTOR', nav: 'NAV / ESTIMATOR', sensors: 'SENSOR BENCH', env: 'ENV / BATTERY' }[id];
    const pre = document.createElement('pre');
    panel.appendChild(h);
    panel.appendChild(pre);
    sections[id] = pre;
  }
  // Fault buttons under the sensor section.
  const bench = document.createElement('div');
  bench.className = 'flt';
  sections.sensors.after(bench);
  for (const s of SENSORS) {
    const row = document.createElement('div');
    const label = document.createElement('span');
    label.textContent = s.padEnd(6);
    row.appendChild(label);
    for (const t of ['freeze', 'dropout', 'bias']) {
      const b = document.createElement('button');
      b.textContent = t.slice(0, 4);
      b.onclick = () => injectFault(s, t);
      row.appendChild(b);
    }
    const clr = document.createElement('button');
    clr.textContent = 'clear';
    clr.onclick = () => clearFault(s);
    row.appendChild(clr);
    bench.appendChild(row);
  }

  const chartsH = document.createElement('h3');
  chartsH.textContent = 'CHARTS (30 s)';
  panel.appendChild(chartsH);
  const charts = {};
  for (const id of ['alt', 'va', 'err']) {
    const c = document.createElement('canvas');
    c.width = 304;
    c.height = 64;
    panel.appendChild(c);
    charts[id] = c.getContext('2d');
  }

  const hist = []; // ring of {alt, estAlt, va, errH}

  function drawChart(g, label, series, colors, fixed = 1) {
    const W = 304, H = 64;
    g.clearRect(0, 0, W, H);
    const all = series.flat().filter(Number.isFinite);
    if (!all.length) return;
    let lo = Math.min(...all), hi = Math.max(...all);
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    const pad = (hi - lo) * 0.12;
    lo -= pad; hi += pad;
    series.forEach((data, si) => {
      g.strokeStyle = colors[si];
      g.beginPath();
      data.forEach((v, i) => {
        const x = (i / (HIST - 1)) * W;
        const y = H - ((v - lo) / (hi - lo)) * H;
        i === 0 ? g.moveTo(x, y) : g.lineTo(x, y);
      });
      g.stroke();
    });
    g.fillStyle = '#8fa0bd';
    g.font = '9px ui-monospace, Menlo, monospace';
    const last = series[0][series[0].length - 1];
    g.fillText(`${label} ${Number.isFinite(last) ? last.toFixed(fixed) : '—'}`, 5, 10);
    g.fillText(hi.toFixed(0), W - 24, 10);
    g.fillText(lo.toFixed(0), W - 24, H - 3);
  }

  const api = {
    open: false,
    toggle() {
      api.open = !api.open;
      panel.classList.toggle('open', api.open);
    },
    // Called from the SIM loop (fixed cadence → deterministic chart data).
    record() {
      const d = getData();
      hist.push({
        alt: d.state.pos[1], estAlt: d.est.pos[1],
        va: d.va,
        errH: Math.hypot(d.est.pos[0] - d.state.pos[0], d.est.pos[2] - d.state.pos[2]),
      });
      if (hist.length > HIST) hist.shift();
    },
    render() {
      if (!api.open) return;
      const d = getData();
      const [p, v, w] = [d.state.pos, d.state.vel, d.state.omega];
      const a = d.state.act;
      sections.state.textContent =
        `pos ${fmtVec(p)}\nvel ${fmtVec(v)}\npqr ${fmtVec(w.map((x) => x * DEG))} °/s` +
        `\nδa ${(a.da * DEG).toFixed(1)}°  δe ${(a.de * DEG).toFixed(1)}°  δr ${(a.dr * DEG).toFixed(1)}°  δt ${(a.dt * 100).toFixed(0)}%` +
        `\nVa ${d.va.toFixed(1)} m/s  α ${(d.alpha * DEG).toFixed(1)}°  β ${(d.beta * DEG).toFixed(1)}°`;
      const errH = Math.hypot(d.est.pos[0] - d.state.pos[0], d.est.pos[2] - d.state.pos[2]);
      const qd = Math.abs(d.att.quat.reduce((s2, v, i) => s2 + v * d.state.quat[i], 0));
      const attErr = 2 * Math.acos(Math.min(1, qd)) * DEG;
      sections.nav.textContent =
        `est err  H ${errH.toFixed(2)} m   V ${(d.est.pos[1] - d.state.pos[1]).toFixed(2)} m` +
        `\natt err  ${attErr.toFixed(2)}°   gyro bias ${fmtVec(d.att.bias.map((b) => b * DEG), 3)} °/s` +
        `\nvarH ${d.est.varH.toFixed(2)}  varV ${d.est.varV.toFixed(2)}  gpsAge ${d.est.gpsAge.toFixed(1)} s` +
        `\nekf flags 0b${(d.ekf.flags >>> 0).toString(2).padStart(10, '0')}`;
      sections.sensors.textContent = SENSORS
        .map((s) => `${s.padEnd(6)} ${d.faults[s] ? `FAULT:${d.faults[s]}` : 'ok'}`)
        .join('\n');
      bench.querySelectorAll('div').forEach((row, i) => {
        row.querySelector('span').className = d.faults[SENSORS[i]] ? 'bad' : '';
      });
      sections.env.textContent =
        `wind ${fmtVec(d.windWorld)} m/s\nbatt ${(d.batt.battMv / 1000).toFixed(2)} V  ` +
        `${(d.batt.battCa / 100).toFixed(1)} A  ${d.batt.battPct}%`;

      drawChart(charts.alt, 'alt t/e', [hist.map((h) => h.alt), hist.map((h) => h.estAlt)], ['#9fe870', '#8fd3ff']);
      drawChart(charts.va, 'Va', [hist.map((h) => h.va)], ['#ffd479']);
      drawChart(charts.err, 'est errH', [hist.map((h) => h.errH)], ['#ff8c8c'], 2);
    },
  };

  window.addEventListener('keydown', (e) => {
    if (e.code === 'KeyE') api.toggle();
  });
  return api;
}
