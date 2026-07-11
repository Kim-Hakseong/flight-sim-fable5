// Bridge → sim command channel: SSE /commands → handler. Dedupes by the bridge's
// monotonic sequence (the bridge re-sends the last command on (re)connect).
// I/O only — command *application* happens in main.js between fixed steps.

export function startMissionLink(onCommand) {
  let lastSeq = -1;
  const es = new EventSource('/commands');
  es.onmessage = (ev) => {
    let cmd;
    try {
      cmd = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (typeof cmd.seq !== 'number' || cmd.seq <= lastSeq) return;
    lastSeq = cmd.seq;
    onCommand(cmd);
  };
  // EventSource auto-reconnects; the seq guard makes the replayed command a no-op.
  return es;
}
