// Browser gate for M0+: serve the repo, drive headless Chrome over raw CDP
// (no deps — Node's global WebSocket), assert console-0 and that the
// window.__advance path is bit-reproducible in the browser.
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { spawn, execSync } from 'node:child_process';
import { mkdtempSync, rmSync, mkdirSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, extname } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = fileURLToPath(new URL('..', import.meta.url));
const CHROME =
  process.env.CHROME_BIN ||
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json',
};

// PAGE_URL overrides the local static server — e.g. smoke-test the live Pages
// deploy: PAGE_URL=https://kim-hakseong.github.io/flight-sim-fable5/ npm run check:browser
let server = null;
let pageUrl = process.env.PAGE_URL;
if (!pageUrl) {
  server = createServer(async (req, res) => {
    const path = req.url === '/' ? '/index.html' : req.url.split('?')[0];
    try {
      const body = await readFile(join(ROOT, path.slice(1)));
      res.writeHead(200, { 'content-type': MIME[extname(path)] || 'application/octet-stream' });
      res.end(body);
    } catch {
      res.writeHead(404).end('not found');
    }
  });
  await new Promise((r) => server.listen(0, '127.0.0.1', r));
  pageUrl = `http://127.0.0.1:${server.address().port}/`;
}

const profile = mkdtempSync(join(tmpdir(), 'fsim-chrome-'));
const chrome = spawn(CHROME, [
  '--headless', '--no-first-run', '--mute-audio',
  '--enable-unsafe-swiftshader', // allow software WebGL when headless has no GPU
  `--user-data-dir=${profile}`, '--remote-debugging-port=0', 'about:blank',
], { stdio: ['ignore', 'ignore', 'pipe'] });

const devtoolsPort = await new Promise((resolve, reject) => {
  let buf = '';
  const t = setTimeout(() => reject(new Error('Chrome DevTools endpoint timeout')), 15000);
  chrome.stderr.on('data', (d) => {
    buf += d;
    const m = buf.match(/DevTools listening on ws:\/\/127\.0\.0\.1:(\d+)\//);
    if (m) { clearTimeout(t); resolve(Number(m[1])); }
  });
  chrome.on('exit', () => reject(new Error('Chrome exited early')));
});

function cleanup() {
  chrome.kill();
  server?.close();
  try { rmSync(profile, { recursive: true, force: true }); } catch {}
}

let failed = false;
try {
  // Find the page target and attach.
  const list = await (await fetch(`http://127.0.0.1:${devtoolsPort}/json/list`)).json();
  const page = list.find((t) => t.type === 'page');
  const ws = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((r, j) => { ws.onopen = r; ws.onerror = j; });

  let msgId = 0;
  const pending = new Map();
  const consoleErrors = [];
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      msg.error ? reject(new Error(msg.error.message)) : resolve(msg.result);
    } else if (msg.method === 'Runtime.consoleAPICalled' && msg.params.type === 'error') {
      consoleErrors.push(msg.params.args.map((a) => a.value ?? a.description).join(' '));
    } else if (msg.method === 'Runtime.exceptionThrown') {
      const e = msg.params.exceptionDetails;
      consoleErrors.push(e.exception?.description || e.text);
    }
  };
  const send = (method, params = {}) =>
    new Promise((resolve, reject) => {
      const id = ++msgId;
      pending.set(id, { resolve, reject });
      ws.send(JSON.stringify({ id, method, params }));
    });
  const evaluate = async (expression) => {
    const r = await send('Runtime.evaluate', { expression, returnByValue: true });
    if (r.exceptionDetails) throw new Error(r.exceptionDetails.text);
    return r.result.value;
  };

  await send('Runtime.enable');
  await send('Page.enable');
  await send('Page.navigate', { url: pageUrl });

  // Wait for the app (and the CDN script) to be fully up.
  const deadline = Date.now() + 20000;
  while (!(await evaluate('window.__ready === true'))) {
    if (Date.now() > deadline) {
      const diag = await evaluate(
        'JSON.stringify({ href: location.href, ready: document.readyState, three: typeof window.THREE })'
      );
      consoleErrors.forEach((e) => console.error(`  console ✗ ${e}`));
      throw new Error(`page never became ready (window.__ready); diag=${diag}`);
    }
    await new Promise((r) => setTimeout(r, 250));
  }

  const det = await evaluate(`(() => {
    window.__reset(); window.__advance(10);
    const a = window.__state();
    window.__reset(); window.__advance(10);
    const b = window.__state();
    window.__reset(); window.injectFault('gps', 'bias', { bias: 100 }); window.__advance(5);
    const c = window.__state();
    window.__reset(); window.injectFault('gps', 'bias', { bias: 100 }); window.__advance(5);
    const d = window.__state();
    window.clearFault('gps'); window.__reset();
    return { match: a === b, faultMatch: c === d && c !== a, sample: a.slice(0, 120) };
  })()`);

  // Screenshot artifact (UI gate: CLAUDE.md §0.4). __reset first so the frame is canonical.
  await evaluate('window.__reset(), window.__advance(3), true');
  await new Promise((r) => setTimeout(r, 300)); // let a rAF render the new state
  const shot = await send('Page.captureScreenshot', { format: 'png' });
  const artDir = join(ROOT, 'tests', 'artifacts');
  mkdirSync(artDir, { recursive: true });
  const shotPath = join(artDir, 'sim.png');
  writeFileSync(shotPath, Buffer.from(shot.data, 'base64'));

  console.log(`page:        ${pageUrl}`);
  console.log(`screenshot:  ${shotPath}`);
  console.log(`console errors: ${consoleErrors.length}`);
  consoleErrors.forEach((e) => console.log(`  ✗ ${e}`));
  console.log(`__advance reproducible: ${det.match}`);
  console.log(`__advance + injectFault reproducible: ${det.faultMatch}`);
  console.log(`  state: ${det.sample}…`);

  if (consoleErrors.length > 0 || !det.match || !det.faultMatch) failed = true;
  ws.close();
} catch (err) {
  console.error(`browser check error: ${err.message}`);
  failed = true;
} finally {
  cleanup();
}

console.log(failed ? 'BROWSER CHECK: FAIL' : 'BROWSER CHECK: PASS');
process.exit(failed ? 1 : 0);
