// The lane timeline under the workspace: transport, EVENTS, DECIDE, PARTNER and SCORE lanes with one playhead.
// The helpers at the top are pure (tested in ../timeline.test.mjs); mountTimeline() is the DOM.

import { icon } from './icons.js';

/** The kind of a recorded event, from its text. */
export function eventKind(text) {
  if (text.startsWith('shot')) return 'shot';
  if (text.includes('went into')) return 'in';
  if (text.startsWith('picked up')) return 'pickup';
  if (text.endsWith('tipped')) return 'tip';
  if (text.startsWith('robot touched')) return 'contact';
  return 'other';
}

/**
 * Merges events that would land less than minGapPx apart on a lane widthPx wide into one marker.
 * Returns [{ t, kinds: Set, events, label }] in time order. Events before START (t < 0) are left off.
 * A cluster of shots reads "3 shots" when every shot went in, otherwise "3 · 1 miss"; any other mix reads "N events".
 */
export function clusterMarkers(events, duration, widthPx, minGapPx = 6) {
  const pxPerSecond = widthPx / duration;
  const sorted = events.filter((e) => e.t >= 0 && e.t <= duration).sort((a, b) => a.t - b.t);
  const clusters = [];
  for (const e of sorted) {
    const last = clusters[clusters.length - 1];
    if (last && (e.t - last.events[last.events.length - 1].t) * pxPerSecond < minGapPx) last.events.push(e);
    else clusters.push({ t: e.t, events: [e] });
  }
  return clusters.map((c) => {
    const kinds = new Set(c.events.map((e) => eventKind(e.text)));
    return { t: c.t, kinds, events: c.events, label: clusterLabel(c.events, kinds) };
  });
}

function clusterLabel(events, kinds) {
  const shots = events.filter((e) => eventKind(e.text) === 'shot').length;
  const ins = events.filter((e) => eventKind(e.text) === 'in').length;
  const onlyShooting = [...kinds].every((k) => k === 'shot' || k === 'in');
  if (events.length === 1) {
    const text = events[0].text;
    const kind = [...kinds][0];
    return kind === 'shot' ? 'shot' : kind === 'in' ? 'in' : kind === 'pickup' ? 'pickup' : kind === 'tip' ? 'tip' : kind === 'contact' ? 'touch' : text;
  }
  if (onlyShooting && shots) {
    const miss = Math.max(0, shots - ins);
    return miss ? `${shots} · ${miss} miss` : `${shots} shots`;
  }
  return `${events.length} events`;
}

/** How a cluster looks: a count pill for shots, a dot or diamond for the rest. */
function clusterStyle(c) {
  const shots = c.events.filter((e) => eventKind(e.text) === 'shot').length;
  const ins = c.events.filter((e) => eventKind(e.text) === 'in').length;
  if (c.kinds.has('contact')) return { shape: c.events.length > 1 ? 'pill' : 'diamond', tone: 'warn' };
  if (shots) return { shape: shots > 1 || c.events.length > 1 ? 'pill' : 'pill', tone: ins >= shots ? 'pass' : ins ? 'warn' : 'fail' };
  if (c.kinds.has('tip')) return { shape: 'pill', tone: 'pass' };
  return { shape: c.events.length > 1 ? 'pill' : 'dot', tone: 'none' };
}

const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];
export const SHORTCUTS = [
  ['Space', 'Play or pause'], ['← →', 'One frame back or forward (Shift = ×10)'], ['Home / End', 'Start or end of the match'],
  ['[ ]', 'Slower or faster'], ['V', '2D or 3D'], ['F', 'Follow the robot (3D)'], ['R', 'Run again'],
  ['N / Shift+N', 'Next or previous problem'], ['D', 'Details drawer'], ['Esc', 'Close the drawer or go back to the summary'],
  ['/', 'Search parameters'], ['?', 'This list'],
];
const LANE_LABELS = { events: 'EVENTS', decide: 'DECIDE', partner: 'PARTNER', score: 'SCORE' };
const fmt2 = (v) => v.toFixed(2);

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function loadLanes() {
  try { return JSON.parse(localStorage.getItem('sim.lanes')) || {}; } catch { return {}; }
}
function saveLanes(v) {
  try { localStorage.setItem('sim.lanes', JSON.stringify(v)); } catch { /* private window: lanes just reset next time */ }
}

/**
 * Builds the timeline inside root. Callbacks: onSeek(t), onPlay(), onStep(dir, big), onSpeed(x).
 * Returns { setTime(t), setPlaying(bool), setRun(model), setStale(bool), setSpeed(x), openHelp() } where
 * model = { duration, events, decisions: [{move, from, to}], score, partner: {name, duration, closestAt, closestText} | null }.
 */
export function mountTimeline(root, { duration: initialDuration = 30, onSeek, onPlay, onStep, onSpeed }) {
  let duration = initialDuration, model = null, time = 0, trackW = 1, speedIndex = 2;
  const shown = { events: true, decide: true, partner: true, score: true, ...loadLanes() };

  // ---- header ----
  const back = el('button', 'tl-btn'); back.append(icon('back')); back.setAttribute('aria-label', 'Back one frame'); back.title = 'Back one frame (←)';
  const play = el('button', 'tl-btn tl-play'); play.append(icon('play')); play.setAttribute('aria-label', 'Play'); play.title = 'Play / pause (Space)';
  const fwd = el('button', 'tl-btn'); fwd.append(icon('fwd')); fwd.setAttribute('aria-label', 'Forward one frame'); fwd.title = 'Forward one frame (→)';
  const clock = el('span', 'tl-clock m'); const now = el('b', '', '0.00 s'); const total = el('span', 'tl-total', '/ 30.00');
  clock.append(now, total);
  const speedBtn = el('button', 'tl-text m', '1×'); speedBtn.title = 'Speed ([ and ])'; speedBtn.setAttribute('aria-label', 'Speed');
  const lanesBtn = el('button', 'tl-text', '+ Lanes'); lanesBtn.setAttribute('aria-haspopup', 'true');
  const helpBtn = el('button', 'tl-text', '?'); helpBtn.setAttribute('aria-label', 'Keyboard shortcuts');
  const head = el('div', 'tl-head'); head.append(back, play, fwd, clock, speedBtn, el('span', 'tl-spacer'), lanesBtn, helpBtn);

  // ---- lanes ----
  const laneBox = el('div', 'tl-lanes');
  const rows = {};
  for (const id of ['events', 'decide', 'partner', 'score']) {
    const row = el('div', `tl-row tl-${id}`);
    row.append(el('span', 'tl-label', LANE_LABELS[id]));
    const track = el('div', 'tl-track');
    row.append(track);
    laneBox.append(row);
    rows[id] = { row, track };
  }
  const playhead = el('div', 'tl-playhead');
  const range = el('input', 'tl-range'); range.type = 'range'; range.min = 0; range.max = duration; range.step = 0.001; range.value = 0; range.setAttribute('aria-label', 'Match time');
  const lanesMenu = el('div', 'tl-menu'); lanesMenu.hidden = true;
  const dialog = el('dialog', 'tl-help');
  dialog.append(el('h3', '', 'Keyboard shortcuts'));
  const dl = el('dl'); for (const [k, v] of SHORTCUTS) { const dt = el('dt'); dt.append(el('span', 'kbd m', k)); dl.append(dt, el('dd', '', v)); }
  dialog.append(dl);
  const close = el('button', 'btn', 'Close'); close.addEventListener('click', () => dialog.close()); dialog.append(close);
  root.replaceChildren(head, laneBox, range, lanesMenu, dialog);
  laneBox.append(playhead);

  // ---- behaviour ----
  back.addEventListener('click', () => onStep(-1, false));
  fwd.addEventListener('click', () => onStep(1, false));
  play.addEventListener('click', () => onPlay());
  speedBtn.addEventListener('click', () => { speedIndex = (speedIndex + 1) % SPEEDS.length; setSpeed(SPEEDS[speedIndex]); onSpeed(SPEEDS[speedIndex]); });
  helpBtn.addEventListener('click', () => dialog.showModal());
  range.addEventListener('input', () => onSeek(Number(range.value)));
  lanesBtn.addEventListener('click', () => { lanesMenu.hidden = !lanesMenu.hidden; });
  for (const id of ['events', 'decide', 'partner', 'score']) {
    const label = el('label'); const box = el('input'); box.type = 'checkbox'; box.checked = shown[id];
    box.addEventListener('change', () => { shown[id] = box.checked; saveLanes(shown); applyLaneVisibility(); });
    label.append(box, document.createTextNode(` ${LANE_LABELS[id]}`));
    lanesMenu.append(label);
  }
  document.addEventListener('pointerdown', (e) => { if (!lanesMenu.hidden && !lanesMenu.contains(e.target) && e.target !== lanesBtn) lanesMenu.hidden = true; });

  const timeAt = (clientX) => {
    const box = rows.events.track.getBoundingClientRect();
    return Math.min(duration, Math.max(0, ((clientX - box.left) / Math.max(1, box.width)) * duration));
  };
  let dragging = false;
  laneBox.addEventListener('pointerdown', (e) => {
    if (e.target.closest('.tl-marker')) return;
    dragging = true; laneBox.setPointerCapture(e.pointerId); onSeek(timeAt(e.clientX));
  });
  laneBox.addEventListener('pointermove', (e) => { if (dragging) onSeek(timeAt(e.clientX)); });
  const endDrag = () => { dragging = false; };
  laneBox.addEventListener('pointerup', endDrag); laneBox.addEventListener('pointercancel', endDrag);

  function applyLaneVisibility() {
    for (const id of Object.keys(rows)) {
      const hasData = id !== 'partner' || (model && model.partner);
      rows[id].row.hidden = !shown[id] || !hasData;
    }
  }

  // ---- drawing the lanes (only when the run or the width changes, never per frame) ----
  let decideEls = [], currentDecide = -2, scoreParts = null, lastPts = -1;

  function pct(t) { return `${(Math.min(duration, Math.max(0, t)) / duration) * 100}%`; }

  function drawEvents() {
    const track = rows.events.track; track.replaceChildren();
    if (!model) return;
    for (const c of clusterMarkers(model.events, duration, trackW)) {
      const { shape, tone } = clusterStyle(c);
      const m = el('button', `tl-marker ${shape} tone-${tone}`);
      m.style.left = pct(c.t);
      if (shape === 'pill') m.textContent = c.label;
      m.title = c.events.length === 1 ? `${c.events[0].text} · ${c.t.toFixed(2)} s` : `${c.events.length} events from ${c.t.toFixed(2)} s: ${c.events.map((e) => e.text).join('; ')}`;
      m.setAttribute('aria-label', m.title);
      m.addEventListener('click', () => onSeek(c.t));
      track.append(m);
    }
  }

  function drawDecide() {
    const track = rows.decide.track; track.replaceChildren(); decideEls = []; currentDecide = -2;
    if (!model) return;
    for (const d of model.decisions) {
      const seg = el('button', 'tl-seg m', d.move);
      seg.style.left = pct(d.from); seg.style.width = `${((d.to - d.from) / duration) * 100}%`;
      seg.title = `${d.move} · ${d.from.toFixed(1)}–${d.to.toFixed(1)} s`;
      seg.addEventListener('click', () => onSeek(d.from));
      track.append(seg); decideEls.push(seg);
    }
    if (!model.decisions.length) track.append(el('span', 'tl-none', 'This auto does not report decide()'));
  }

  function drawPartner() {
    const track = rows.partner.track; track.replaceChildren();
    if (!model || !model.partner) return;
    const p = model.partner;
    const bar = el('div', 'tl-partner-bar'); bar.style.width = `${(Math.min(p.duration, duration) / duration) * 100}%`;
    bar.append(el('span', 'm', p.name));
    track.append(bar);
    if (Number.isFinite(p.closestAt)) {
      const mark = el('i', 'tl-closest'); mark.style.left = pct(p.closestAt); mark.title = p.closestText || '';
      const text = el('span', 'tl-closest-text m', p.closestShort || 'closest'); text.style.left = pct(p.closestAt);
      track.append(mark, text);
    }
    const done = el('span', 'tl-done m', `done ${p.duration.toFixed(1)} s`); done.style.left = pct(Math.min(p.duration, duration));
    track.append(done);
  }

  function drawScore() {
    const track = rows.score.track; track.replaceChildren(); scoreParts = null; lastPts = -1;
    if (!model || !model.score) return;
    const s = model.score;
    const top = Math.max(s.final, 1);
    const pts = [[0, 0]]; let sum = 0;
    for (const i of s.items) { pts.push([i.t, sum]); sum += i.points; pts.push([i.t, sum]); }
    pts.push([duration, sum]);
    const X = (t) => (t / duration) * 1000, Y = (v) => 34 - (v / top) * 30;
    const d = pts.map(([t, v], k) => `${k ? 'L' : 'M'}${X(t).toFixed(1)} ${Y(v).toFixed(1)}`).join(' ');
    const NS = 'http://www.w3.org/2000/svg';
    const svg = document.createElementNS(NS, 'svg'); svg.setAttribute('viewBox', '0 0 1000 40'); svg.setAttribute('preserveAspectRatio', 'none'); svg.classList.add('tl-score-svg');
    svg.innerHTML = `<defs><clipPath id="tlPast"><rect x="0" y="0" width="0" height="40"/></clipPath><clipPath id="tlFuture"><rect x="0" y="0" width="1000" height="40"/></clipPath></defs>
      <path class="future" d="${d}" clip-path="url(#tlFuture)"/><path class="past" d="${d}" clip-path="url(#tlPast)"/>`;
    track.append(svg);
    for (const i of s.items) {
      const pill = el('span', `tl-pill m tone-${i.kind === 'tip' ? 'accent' : 'pass'}`, `+${i.points} ${i.kind.toUpperCase()}${i.kind === 'park' ? ` · ${i.t.toFixed(1)} s` : ''}`);
      if (i.est) pill.append(el('span', 'est', '~'));
      pill.style.left = pct(i.t); pill.title = i.label;
      track.append(pill);
    }
    const bubble = el('span', 'tl-total-bubble m', '0 pts'); track.append(bubble);
    scoreParts = { past: svg.querySelector('#tlPast rect'), bubble };
  }

  function measure() {
    trackW = rows.events.track.clientWidth || 1;
  }

  function drawAll() {
    measure();
    total.textContent = `/ ${fmt2(duration)}`;
    range.max = duration;
    drawEvents(); drawDecide(); drawPartner(); drawScore(); applyLaneVisibility();
    setTime(time, true);
  }

  new ResizeObserver(() => { const w = rows.events.track.clientWidth; if (w && Math.abs(w - trackW) > 1) { trackW = w; drawEvents(); setTime(time, true); } }).observe(laneBox);

  // ---- per frame: only transforms and a few text nodes ----
  function setTime(t, force = false) {
    time = t;
    playhead.style.transform = `translateX(${(t / duration) * trackW}px)`;
    const label = `${fmt2(t)} s`;
    if (now.textContent !== label) now.textContent = label;
    if (document.activeElement !== range) range.value = t;
    if (model) {
      let idx = -1;
      for (let i = 0; i < model.decisions.length; i++) if (model.decisions[i].from <= t) idx = i;
      if (idx !== currentDecide || force) {
        decideEls.forEach((e, i) => e.classList.toggle('now', i === idx));
        currentDecide = idx;
      }
      if (scoreParts) {
        scoreParts.past.setAttribute('width', (t / duration) * 1000);
        const pts = model.score.at(t);
        if (pts !== lastPts || force) {
          scoreParts.bubble.textContent = `${pts} pts`; lastPts = pts;
        }
        scoreParts.bubble.style.left = pct(t);
      }
    }
  }

  function setPlaying(isPlaying) {
    play.replaceChildren(icon(isPlaying ? 'pause' : 'play'));
    play.setAttribute('aria-label', isPlaying ? 'Pause' : 'Play');
  }
  function setSpeed(x) { speedIndex = Math.max(0, SPEEDS.indexOf(x)); speedBtn.textContent = `${x}×`; }
  function setRun(next) { model = next; duration = next.duration; drawAll(); }
  function setStale(stale) { laneBox.classList.toggle('old', stale); }

  setPlaying(false);
  return { setTime, setPlaying, setRun, setStale, setSpeed, openHelp: () => dialog.showModal(), isHelpOpen: () => dialog.open };
}
