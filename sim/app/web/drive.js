// The #/drive page: drive the simulated robot live. The Java sim runs in real time (server.mjs /api/live); this page only sends the
// gamepad state (keyboard or a browser gamepad) and draws the frames it gets back. No robot logic lives here.
//   POST /api/live            start        DELETE /api/live   stop
//   POST /api/live/input      { lx, ly, rx, ry, lt, rt, buttons } or { set: "Class.FIELD", value }
//   GET  /api/live/events     SSE: state, header, frame, event, telemetry, set
// Leaving the page closes the event stream, and the server ends the session a few seconds later.

import * as THREE from 'three';
import { build3D } from './scene3d.js';
import { h } from './rail.js';

/** Bit i of `buttons` = BUTTON_BITS[i] (must match SimLive.java and live.mjs). */
const BIT = { a: 0, b: 1, x: 2, y: 3, lb: 4, rb: 5, up: 6, down: 7, left: 8, right: 9, back: 10, start: 11, guide: 12, ls: 13, rs: 14 };
const SEND_MS = 33;           // input is sent at most this often, and only when it changed
const DEADZONE = 0.08;

/** What the sim-only SimDriveTeleOp does with each control (its class comment is the source). */
const KEYS = [
  { keys: ['W', 'S'], what: 'Drive forward / back' },
  { keys: ['A', 'D'], what: 'Strafe left / right' },
  { keys: ['Q', 'E'], what: 'Turn left / right (arrow keys too)' },
  { keys: ['Shift'], what: 'Intake (hold)' },
  { keys: ['Space'], what: 'Shoot, one feed cycle' },
  { keys: ['T'], what: 'Switch target Cell A / B' },
  { keys: ['F'], what: 'Flywheel on / off' },
  { keys: ['M'], what: 'Shoot-on-the-move on / off' },
  { keys: ['Esc'], what: 'Stop the session' },
];
const PAD = ['Left stick: drive and strafe', 'Right stick X: turn', 'RT: intake · RB: shoot', 'X: target · Y: flywheel · A: SOTM'];
const TUNING = [   // UNMEASURED defaults live in the Java configs; the box shows the value the sim reports after a change
  { name: 'SOTMConfig.RELEASE_LATENCY_MS', label: 'Release latency', unit: 'ms' },
  { name: 'SOTMConfig.FLIGHT_TIME_A_MS', label: 'Flight time base', unit: 'ms' },
  { name: 'SOTMConfig.FLIGHT_TIME_B_MS_PER_MM', label: 'Flight time per mm', unit: 'ms/mm' },
  { name: 'SOTMConfig.MOTION_FIT_WINDOW_MS', label: 'Motion fit window', unit: 'ms' },
  { name: 'ShooterInterpolationConfig.rpsOffset', label: 'RPS offset', unit: 'rps' },
  { name: 'ShooterInterpolationConfig.hoodOffset', label: 'Hood offset', unit: '' },
];
const STATE_TEXT = { idle: 'Ready', starting: 'Connecting', running: 'Running', stopped: 'Stopped', error: 'Error', offline: 'Runner not reachable' };

const cssColor = (name) => new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888');
const post = (url, body) => fetch(url, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body ?? {}) }).then((r) => r.json().catch(() => ({}))).catch(() => ({ error: 'unreachable' }));

export function mountDrive(root) {
  let visible = false, es = null, phase = 'idle', message = '';
  let scene3d = null, renderer = null, balls = [], ballIndex = new Map(), tilts = [0, 0], following = true, frames = 0, fps = 0, fpsAt = 0;
  let last = null, lastSent = '', lastPose = null, speed = 0;
  const held = new Set(), latch = { buttons: 0 };
  let sendTimer = 0, raf = 0, shotCount = 0, buildGen = 0;

  // ---- DOM ----
  const dot = h('span', { class: 'dotc off' });
  const stateText = h('b', { text: STATE_TEXT.idle });
  const stateMsg = h('span', { class: 'dim dmsg' });
  const status = h('div', { class: 'dstatus', role: 'status', 'aria-live': 'polite' }, dot, stateText, stateMsg);
  const startBtn = h('button', { class: 'btn btn-p', onclick: () => start() }, 'Start driving');
  const stopBtn = h('button', { class: 'btn', onclick: () => stop() }, 'Stop ', h('span', { class: 'kbd', text: 'Esc' }));
  const followBtn = h('button', { class: 'btn', 'aria-pressed': 'true', onclick: () => { following = !following; if (scene3d) scene3d.follow(following); syncFollow(); } }, 'Follow robot');
  const resetBtn = h('button', { class: 'btn', onclick: () => scene3d && scene3d.resetCamera() }, 'Reset view');
  const canvas = h('canvas', { class: 'dcanvas', 'aria-label': 'Live 3D view of the robot' });
  const overlay = h('div', { class: 'doverlay' });
  const stage = h('div', { class: 'dstage' }, canvas, overlay,
    h('div', { class: 'dtools' }, followBtn, resetBtn));
  const telemetryBox = h('dl', { class: 'dtel' });
  const shotLog = h('ol', { class: 'dshots', 'aria-label': 'Shot log' });
  const shotEmpty = h('p', { class: 'dim dempty', text: 'No shots yet. Hold intake, drive into balls, then press Space.' });
  const tuneMsg = h('p', { class: 'dim dtunemsg', role: 'status', 'aria-live': 'polite' });
  const tuneInputs = new Map();
  const tuning = h('div', { class: 'dtune' },
    h('label', { class: 'switch dsotm' }, h('input', { type: 'checkbox', 'data-set': 'SOTMConfig.enabled', onchange: (e) => sendSet('SOTMConfig.enabled', e.target.checked) }), h('span', { class: 'track' }), h('span', { text: 'Shoot on the move' })),
    TUNING.map((t) => {
      const input = h('input', { class: 'numin m', type: 'number', step: 'any', placeholder: 'default', 'aria-label': `${t.label} ${t.unit}`, onchange: (e) => { const v = Number(e.target.value); if (e.target.value !== '' && Number.isFinite(v)) sendSet(t.name, v); } });
      tuneInputs.set(t.name, input);
      return h('label', { class: 'dtrow' }, h('span', { text: t.label }), input, h('span', { class: 'unit m dim', text: t.unit }));
    }),
    tuneMsg);
  const section = (title, ...kids) => h('section', { class: 'dsec' }, h('h2', { class: 'h', text: title }), ...kids);
  const legend = h('div', { class: 'dkeys' },
    KEYS.map((k) => h('div', { class: 'dkey' }, h('span', { class: 'dkeycaps' }, k.keys.map((c) => h('span', { class: 'kbd', text: c }))), h('span', { text: k.what }))),
    h('div', { class: 'dpad' }, h('b', { text: 'Gamepad' }), PAD.map((t) => h('span', { text: t }))));
  const panel = h('aside', { class: 'dpanel', 'aria-label': 'Driver panel' },
    h('div', { class: 'dhead' }, h('h1', { text: 'Drive' }), h('div', { class: 'dbtns' }, startBtn, stopBtn)),
    status,
    section('Telemetry', telemetryBox),
    section('Controls', legend),
    section('Tuning', tuning),
    section('Shots', shotEmpty, shotLog));
  root.replaceChildren(stage, panel);

  const setRow = (dl, key, value) => {
    let dd = dl.querySelector(`[data-k="${CSS.escape(key)}"]`);
    if (!dd) { dd = h('dd', { class: 'm', 'data-k': key }); dl.append(h('dt', { text: key }), dd); }
    dd.textContent = value;
  };
  function syncFollow() { followBtn.setAttribute('aria-pressed', String(following)); }

  function setPhase(next, text = '') {
    phase = next; message = text;
    stateText.textContent = STATE_TEXT[next] || next;
    stateMsg.textContent = next === 'idle' ? 'Press Start, then drive with the keyboard or a gamepad.' : next === 'starting' ? 'Compiling and starting the sim. This takes a few seconds.' : next === 'running' ? 'Real time. Click the page first if keys do nothing.' : next === 'stopped' ? 'The session ended.' : '';
    dot.className = `dotc ${next === 'running' ? 'ok' : next === 'starting' ? 'wait' : next === 'error' || next === 'offline' ? 'bad' : 'off'}`;
    status.dataset.phase = next;
    if ((next === 'error') && text) stateMsg.textContent = text;
    startBtn.disabled = next === 'starting' || next === 'running';
    stopBtn.disabled = !(next === 'starting' || next === 'running');
    for (const el of tuning.querySelectorAll('input')) el.disabled = next !== 'running';
    overlay.textContent = next === 'running' ? '' : next === 'starting' ? 'Connecting…' : next === 'error' ? 'The sim stopped with an error' : 'Press Start driving';
    overlay.hidden = next === 'running';
    overlay.classList.toggle('bad', next === 'error');
  }

  // ---- session ----
  async function start() {
    clearOutput();
    openEvents();
    const r = await post('/api/live');
    if (r.error) setPhase('error', r.error);
  }
  function stop() { fetch('/api/live', { method: 'DELETE' }).catch(() => {}); }
  function clearOutput() { shotLog.replaceChildren(); shotEmpty.hidden = false; telemetryBox.replaceChildren(); shotCount = 0; tuneMsg.textContent = ''; }

  function openEvents() {
    if (es) return;
    es = new EventSource('/api/live/events');
    const on = (name, fn) => es.addEventListener(name, (e) => { try { fn(e.data); } catch (err) { console.warn(`live ${name}`, err); } });
    on('state', (d) => { const s = JSON.parse(d); setPhase(s.phase, s.message); });
    on('header', (d) => buildScene(JSON.parse(d)));
    on('frame', onFrame);
    on('telemetry', (d) => { for (const [k, v] of Object.entries(JSON.parse(d))) if (k !== '_t') setRow(telemetryBox, k, v); });
    on('event', onEvent);
    on('set', (d) => onSetReply(d));
    es.onerror = () => { if (es && es.readyState === EventSource.CLOSED) setPhase('offline'); };
    es.onopen = () => { if (phase === 'offline') setPhase('idle'); };
  }
  function closeEvents() { if (es) es.close(); es = null; }

  // ---- scene ----
  async function buildScene(header) {
    const gen = ++buildGen;
    const run = { ...header, alliance: 'red', frames: [[0, 0, 0, 0, 0, 0, 0]], defenders: [], robotModel: null };
    try { const r = await (await fetch('/api/robot')).json(); run.robotModel = r.robot || null; } catch { /* the plain box is fine */ }
    if (gen !== buildGen || !visible) return;
    const colors = { accent: cssColor('--accent'), ghost: cssColor('--body'), red: cssColor('--red'), blue: cssColor('--blue') };
    if (scene3d) scene3d.dispose();
    scene3d = build3D(run, colors, canvas);
    scene3d.controls.enabled = true;
    scene3d.follow(following);
    balls = run.balls.info.map(() => [0, 0, -1000]);
    ballIndex = new Map(run.balls.info.map((b, i) => [b.id, i]));
    resize();
  }
  function onFrame(d) {
    const f = JSON.parse(d);
    last = f;
    for (const [id, x, y, z] of f.balls || []) { const i = ballIndex.get(id); if (i !== undefined) balls[i] = [x, y, z]; }
    tilts = f.hive || tilts;
    if (lastPose) { const dt = f.t - lastPose.t; if (dt > 0) speed = 0.8 * speed + 0.2 * Math.hypot(f.x - lastPose.x, f.y - lastPose.y) / dt; }
    lastPose = f;
    frames++;
    if (scene3d) scene3d.update({ x: f.x, y: f.y, theta: f.th, ox: f.ox ?? NaN, oy: f.oy ?? NaN, otheta: f.oth ?? 0 }, balls, tilts, 0, f.r2 ? { x: f.r2[0], y: f.r2[1], theta: f.r2[2] } : null);
  }
  function showPose() {
    if (!last) return;
    const now = performance.now();
    if (now - fpsAt > 1000) { fps = Math.round(frames * 1000 / (now - fpsAt)); frames = 0; fpsAt = now; }
    setRow(telemetryBox, 'x, y (mm)', `${last.x.toFixed(0)}, ${last.y.toFixed(0)}`);
    setRow(telemetryBox, 'heading', `${(last.th * 180 / Math.PI).toFixed(0)}°`);
    setRow(telemetryBox, 'speed', `${speed.toFixed(0)} mm/s`);
    setRow(telemetryBox, 'balls held', String(last.held));
    setRow(telemetryBox, 'frames/s', String(fps));
  }
  function loop() {
    if (!visible) return;
    raf = requestAnimationFrame(loop);
    if (scene3d && renderer) renderer.render(scene3d.scene, scene3d.camera);
    showPose();
  }
  function resize() {
    if (!renderer) return;
    const w = stage.clientWidth || 1, hh = stage.clientHeight || 1;
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setSize(w, hh, false);
    if (scene3d) scene3d.resize(w, hh);
  }
  new ResizeObserver(resize).observe(stage);

  // ---- events: shot log, tuning replies ----
  function onEvent(text) {
    const bar = text.indexOf('|');
    const t = Number(text.slice(0, bar)), body = text.slice(bar + 1);
    // world.mjs: "shot a pollen ball (45.6 RPS, 63°, 5.6 m/s)" when it leaves, then "result: shot 1 in|miss, long X mm, left Y mm, ..." when it lands
    const res = /^result: shot \d+ (in|miss), (.*)$/.exec(body);
    if (!res && !/^shot a\b/.test(body)) return;
    const result = res ? res[1] : '';
    shotEmpty.hidden = true;
    shotLog.prepend(h('li', { class: `dshot ${result}` }, h('span', { class: 'm dim', text: `${t.toFixed(1)}s` }), result ? h('span', { class: `pill ${result === 'in' ? 'pass' : 'miss'}`, text: result.toUpperCase() }) : null, h('span', { text: res ? res[2] : body.replace(/^shot /, '') })));
    while (shotLog.children.length > 60) shotLog.lastChild.remove();
  }
  function onSetReply(d) {
    tuneMsg.textContent = d.startsWith('ok') ? `Set ${d.slice(3)}` : `Not set: ${d.replace(/^error\s*/, '')}`;
    tuneMsg.classList.toggle('bad', !d.startsWith('ok'));
    const m = /^ok (\S+) (\S+)/.exec(d);
    if (m) { const input = tuneInputs.get(m[1]); if (input && document.activeElement !== input) input.value = m[2]; }
  }
  function sendSet(name, value) { post('/api/live/input', { set: name, value }).then((r) => { if (r.error) tuneMsg.textContent = r.error; }); }

  // ---- input ----
  const KEYMAP = { w: 'fwd', s: 'back', a: 'left', d: 'right', q: 'turnL', e: 'turnR', ArrowLeft: 'turnL', ArrowRight: 'turnR', ArrowUp: 'fwd', ArrowDown: 'back', Shift: 'intake', ' ': 'shoot', t: 'x', f: 'y', m: 'a' };
  const BUTTON_FOR = { shoot: BIT.rb, x: BIT.x, y: BIT.y, a: BIT.a };
  const keyName = (e) => (e.key.length === 1 ? e.key.toLowerCase() : e.key);
  function onKeyDown(e) {
    if (!visible || e.metaKey || e.ctrlKey || e.altKey) return;
    const tag = e.target.tagName;
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(tag) || (tag === 'BUTTON' && e.key === ' ')) return;   // typing, or Space pressing a focused button
    if (e.key === 'Escape') { stop(); return; }
    const k = KEYMAP[keyName(e)];
    if (!k) return;
    e.preventDefault();
    held.add(k);
    if (k in BUTTON_FOR) latch.buttons |= 1 << BUTTON_FOR[k];   // a tap shorter than one send still counts
  }
  function onKeyUp(e) { const k = KEYMAP[keyName(e)]; if (k) held.delete(k); }
  const axis = (neg, pos) => (held.has(pos) ? 1 : 0) - (held.has(neg) ? 1 : 0);
  const dz = (v) => (Math.abs(v) < DEADZONE ? 0 : v);

  function readInput() {
    let lx = axis('left', 'right'), ly = -axis('back', 'fwd'), rx = axis('turnL', 'turnR'), ry = 0;
    let lt = 0, rt = held.has('intake') ? 1 : 0, buttons = latch.buttons;
    latch.buttons = 0;
    for (const k of ['shoot', 'x', 'y', 'a']) if (held.has(k)) buttons |= 1 << BUTTON_FOR[k];
    const pad = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((p) => p && p.connected);
    if (pad) {
      const ax = (i) => dz(pad.axes[i] || 0), bt = (i) => (pad.buttons[i] ? pad.buttons[i].value : 0);
      const gx = ax(0), gy = ax(1), gr = ax(2), gry = ax(3);
      if (gx || gy || gr || gry) { lx = gx; ly = gy; rx = gr; ry = gry; }
      lt = Math.max(lt, bt(6)); rt = Math.max(rt, bt(7));
      [[0, BIT.a], [1, BIT.b], [2, BIT.x], [3, BIT.y], [4, BIT.lb], [5, BIT.rb], [8, BIT.back], [9, BIT.start], [10, BIT.ls], [11, BIT.rs], [12, BIT.up], [13, BIT.down], [14, BIT.left], [15, BIT.right], [16, BIT.guide]]
        .forEach(([i, b]) => { if (pad.buttons[i] && pad.buttons[i].pressed) buttons |= 1 << b; });
    }
    // two keys at once on a diagonal must not exceed full power
    const mag = Math.hypot(lx, ly);
    if (mag > 1) { lx /= mag; ly /= mag; }
    const r2 = (v) => Math.round(v * 100) / 100;
    return { lx: r2(lx), ly: r2(ly), rx: r2(rx), ry: r2(ry), lt: r2(lt), rt: r2(rt), buttons };
  }
  function tick() {
    if (phase !== 'running') { lastSent = ''; return; }
    const input = readInput();
    const key = JSON.stringify(input);
    if (key === lastSent) return;
    lastSent = key;
    post('/api/live/input', input);
  }
  window.addEventListener('blur', () => held.clear());

  setPhase('idle');
  return {
    show() {
      visible = true;
      if (!renderer) { renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true }); renderer.setClearColor(0x000000, 0); }
      document.addEventListener('keydown', onKeyDown);
      document.addEventListener('keyup', onKeyUp);
      sendTimer = setInterval(tick, SEND_MS);
      openEvents();
      fetch('/api/live').then((r) => r.json()).then((s) => { if (s.header && !scene3d) buildScene(s.header); }).catch(() => {});
      resize(); cancelAnimationFrame(raf); loop();
    },
    hide() {
      visible = false;
      document.removeEventListener('keydown', onKeyDown);
      document.removeEventListener('keyup', onKeyUp);
      clearInterval(sendTimer); held.clear(); cancelAnimationFrame(raf);
      closeEvents();      // no page watching: the server ends the session after a few seconds
    },
  };
}
