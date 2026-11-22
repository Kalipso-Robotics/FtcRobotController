// Live driving: one Java sim process at a time, paced to the wall clock, steered from the browser (see SimLive.java for the Java side).
//   server -> Java (stdin):  "gp 1 lx ly rx ry lt rt buttons", "set Class.FIELD value" and "stop"
//   Java -> server (stdout): "#field/#robot/..." header lines, "#ready", then "#frame {json}" about every 20 ms, "#event s|text",
//                            "#telemetry {json}" and "#set ok|error ..." 
// This file starts no process itself: `launch` is given by the caller, so the tests can use a fake child.

import { parseRun } from './runfile.mjs';

/** Bit i of the buttons mask = BUTTON_NAMES[i]. Must match the bit table in SimLive.java. */
export const BUTTON_NAMES = ['a', 'b', 'x', 'y', 'left_bumper', 'right_bumper', 'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right',
  'back', 'start', 'guide', 'left_stick_button', 'right_stick_button'];

/** The stdin line for a browser input { lx, ly, rx, ry, lt, rt, buttons } (sticks -1..1, triggers 0..1, buttons a bitmask), or null if it is malformed. */
export function gamepadLine(input) {
  if (!input || typeof input !== 'object') return null;
  const nums = ['lx', 'ly', 'rx', 'ry', 'lt', 'rt'].map((k) => input[k] ?? 0);
  const buttons = input.buttons ?? 0;
  if (![...nums, buttons].every((v) => typeof v === 'number' && Number.isFinite(v))) return null;
  if (!Number.isInteger(buttons) || buttons < 0 || buttons >= 2 ** BUTTON_NAMES.length) return null;
  return `gp 1 ${nums.join(' ')} ${buttons}`;
}

/** The stdin line for a config change ("set SOTMConfig.ENABLED 1"): the bare class name, a field name, and a number or true/false. Null if malformed. */
export function setLine(name, value) {
  if (typeof name !== 'string' || !/^[A-Za-z]\w*\.[A-Za-z_]\w*$/.test(name)) return null;
  const v = typeof value === 'boolean' ? String(value) : typeof value === 'number' && Number.isFinite(value) ? String(value) : null;
  return v === null ? null : `set ${name} ${v}`;
}

export const IDLE_STOP_MS = 4000;   // no page watching for this long -> the session ends

/**
 * `launch()` resolves to a child-like object { stdin, stdout, stderr, kill(), on('close') }; `emit(name, data)` is how the session talks
 * to the pages ('state' { phase, message }, 'header' (the run-shaped geometry), 'frame' (the raw JSON text of one frame)).
 * Phases: idle -> starting -> running -> stopped | error. Only one session at a time.
 */
export function createLive({ launch, emit, opMode, killAfterMs = 3000 }) {
  let child = null;
  let phase = 'idle';
  let message = '';
  let header = null;          // the run-shaped object for scene3d, once the Java side said #ready
  let headerLines = [];
  let stopping = false;
  let stderrTail = '';
  let killTimer = null;
  let cancelled = false;      // stop() came while the build was still running

  const set = (next, text = '') => { phase = next; message = text; emit('state', { phase, message }); };
  const active = () => phase === 'starting' || phase === 'running';

  function onLine(line) {
    if (line.startsWith('#frame ')) { if (phase === 'running') emit('frame', line.slice(7)); return; }
    if (line.startsWith('#event ')) { if (phase === 'running') emit('event', line.slice(7)); return; }
    if (line.startsWith('#telemetry ')) { if (phase === 'running') emit('telemetry', line.slice(11)); return; }
    if (line.startsWith('#set ')) { emit('set', line.slice(5)); return; }
    if (line === '#ready') {
      header = parseRun(headerLines.join('\n'), opMode);
      emit('header', header);
      set('running');
    } else if (line.startsWith('#') && phase === 'starting') headerLines.push(line);
    else if (line.startsWith('#error ')) stderrTail += `${line.slice(7)}\n`;
  }

  const api = {
    state: () => ({ phase, message }),
    header: () => header,
    active,
    async start() {
      if (active()) throw Object.assign(new Error('A live session is already running.'), { code: 'busy' });
      headerLines = []; header = null; stopping = false; cancelled = false; stderrTail = '';
      set('starting', 'Building and starting the sim');
      try { child = await launch(); } catch (e) { set('error', String(e.message || e)); return; }
      const me = child;
      let buffer = '';
      me.stdout.on('data', (d) => {
        buffer += d;
        let nl;
        while ((nl = buffer.indexOf('\n')) >= 0) { onLine(buffer.slice(0, nl).replace(/\r$/, '')); buffer = buffer.slice(nl + 1); }
      });
      me.stderr.on('data', (d) => { stderrTail = (stderrTail + d).slice(-2000); });
      me.stdin.on('error', () => { /* the process is already gone; 'close' reports it */ });
      me.on('close', (code) => {
        if (child !== me) return;
        clearTimeout(killTimer);
        child = null;
        if (stopping) set('stopped', 'Stopped');
        else set('error', (stderrTail.trim().split('\n').filter((l) => !l.startsWith('WARNING')).join('\n') || `The sim process ended (exit ${code}).`));
      });
      if (cancelled) api.stop();
    },
    /** Asks Java to end ("stop" on stdin), then kills it if it has not gone after killAfterMs. Safe to call twice. */
    stop() {
      if (!child) { if (phase === 'starting') cancelled = true; return; }
      if (stopping) return;
      stopping = true;
      try { child.stdin.write('stop\n'); child.stdin.end(); } catch { /* already closed */ }
      const me = child;
      killTimer = setTimeout(() => me.kill('SIGKILL'), killAfterMs);
    },
    /** Changes a config value in the running sim; the answer comes back as a 'set' event. False if no running session or malformed. */
    set(name, value) {
      const line = setLine(name, value);
      if (!line || phase !== 'running' || !child) return false;
      try { child.stdin.write(`${line}\n`); } catch { return false; }
      return true;
    },
    /** Sends one gamepad state. False if there is no running session or the input is malformed. */
    input(body) {
      const line = gamepadLine(body);
      if (!line || phase !== 'running' || !child) return false;
      try { child.stdin.write(`${line}\n`); } catch { return false; }
      return true;
    },
  };
  return api;
}
