// Draws the Stats page from an analysis (see stats.js). Plain DOM and SVG, no libraries.
// Every number on the page comes from the recorded run; this file only lays it out.

import { THRESHOLDS } from './stats.js';

const SVG = 'http://www.w3.org/2000/svg';
const reduceMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;

function h(tag, props = {}, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'text') el.textContent = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else if (k === 'style') el.style.cssText = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const kid of kids.flat()) if (kid !== null && kid !== undefined && kid !== false) el.append(kid);
  return el;
}

function svg(tag, props = {}, ...kids) {
  const el = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(props)) el.setAttribute(k, v);
  for (const kid of kids.flat()) if (kid) el.append(kid);
  return el;
}

const changed0 = (run) => Object.values(run.params || {}).filter((p) => p.used !== p.default).length;
const fmt = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '—');
const ICONS = { pass: '✓', warn: '!', fail: '×', info: 'i' };

/** Counts a number up from 0 when the page appears. Without motion it just shows the final number. */
function countUp(el, value, decimals) {
  if (!Number.isFinite(value) || reduceMotion()) { el.textContent = fmt(value, decimals); return; }
  const start = performance.now(), ms = 900;
  const tick = (now) => {
    const k = Math.min(1, (now - start) / ms);
    el.textContent = (value * (1 - Math.pow(1 - k, 3))).toFixed(decimals);
    if (k < 1) requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

function niceMax(v) {
  if (!(v > 0)) return 1;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  for (const m of [1, 2, 2.5, 5, 10]) if (v <= m * p) return m * p;
  return 10 * p;
}

/** A line chart over match time: hover for the value, click to open the replay at that moment. */
function lineChart({ title, hint, unit, series, duration, color, marks = [], bands = [], threshold = null, scale = 1, decimals = 0, onSeek }) {
  const W = 800, H = 190, L = 46, R = 12, T = 12, B = 26;
  const peak = Math.max(...series.map((p) => p[1] * scale), threshold ?? 0);
  const yMax = niceMax(peak * 1.05);
  const x = (t) => L + (t / duration) * (W - L - R);
  const y = (v) => T + (1 - Math.min(v, yMax) / yMax) * (H - T - B);

  const root = svg('svg', { viewBox: `0 0 ${W} ${H}`, class: 'chart', role: 'img', 'aria-label': `${title} over match time` });
  for (let i = 0; i <= 4; i++) {
    const v = (yMax * i) / 4;
    root.append(svg('line', { x1: L, x2: W - R, y1: y(v), y2: y(v), class: 'grid' }));
    const label = svg('text', { x: L - 8, y: y(v) + 4, class: 'tick', 'text-anchor': 'end' });
    label.textContent = fmt(v, yMax < 10 ? 1 : 0);
    root.append(label);
  }
  const step = duration > 20 ? 5 : duration > 8 ? 2 : 1;
  for (let t = 0; t <= duration + 1e-6; t += step) {
    const label = svg('text', { x: x(t), y: H - 7, class: 'tick', 'text-anchor': 'middle' });
    label.textContent = `${t}s`;
    root.append(label);
  }
  for (const b of bands) {
    root.append(svg('rect', { x: x(b.from), y: T, width: Math.max(1, x(b.to) - x(b.from)), height: H - T - B, class: 'band' }));
  }
  if (threshold !== null) {
    root.append(svg('line', { x1: L, x2: W - R, y1: y(threshold), y2: y(threshold), class: 'threshold' }));
    const label = svg('text', { x: W - R - 4, y: y(threshold) - 5, class: 'tick', 'text-anchor': 'end' });
    label.textContent = `fine under ${threshold}`;
    root.append(label);
  }
  for (const m of marks) root.append(svg('line', { x1: x(m.t), x2: x(m.t), y1: T, y2: H - B, class: 'mark' }));

  const points = series.map(([t, v]) => `${x(t).toFixed(1)},${y(v * scale).toFixed(1)}`);
  root.append(svg('polygon', { points: `${x(series[0][0])},${H - B} ${points.join(' ')} ${x(series[series.length - 1][0])},${H - B}`, class: 'area', style: `fill:${color}` }));
  let length = 0; // the real length of the line, so the draw-in animation does not depend on pathLength support
  for (let i = 1; i < series.length; i++) length += Math.hypot(x(series[i][0]) - x(series[i - 1][0]), y(series[i][1] * scale) - y(series[i - 1][1] * scale));
  root.append(svg('polyline', { points: points.join(' '), class: 'line', style: `stroke:${color};--len:${length.toFixed(1)}` }));
  const cursor = svg('line', { y1: T, y2: H - B, class: 'cursor', opacity: 0 });
  const dot = svg('circle', { r: 4, class: 'dot', opacity: 0, style: `fill:${color}` });
  root.append(cursor, dot);

  const tip = h('div', { class: 'tip', hidden: true });
  const wrap = h('div', { class: 'chartWrap' }, root, tip);
  const nearest = (t) => {
    let lo = 0, hi = series.length - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (series[mid][0] < t) lo = mid + 1; else hi = mid; }
    return series[lo];
  };
  const timeAt = (event) => {
    const box = root.getBoundingClientRect();
    const px = ((event.clientX - box.left) / box.width) * W;
    return Math.min(duration, Math.max(0, ((px - L) / (W - L - R)) * duration));
  };
  root.addEventListener('pointermove', (event) => {
    const t = timeAt(event);
    const [pt, pv] = nearest(t);
    const cx = x(pt);
    cursor.setAttribute('x1', cx); cursor.setAttribute('x2', cx); cursor.setAttribute('opacity', 1);
    dot.setAttribute('cx', cx); dot.setAttribute('cy', y(pv * scale)); dot.setAttribute('opacity', 1);
    tip.hidden = false;
    tip.textContent = `${fmt(pt, 2)} s · ${fmt(pv * scale, decimals)} ${unit}`;
    tip.style.left = `${(cx / W) * 100}%`;
  });
  root.addEventListener('pointerleave', () => { cursor.setAttribute('opacity', 0); dot.setAttribute('opacity', 0); tip.hidden = true; });
  root.addEventListener('click', (event) => onSeek(timeAt(event)));

  return h('section', { class: 'card chartCard' },
    h('div', { class: 'cardHead' }, h('h3', { text: title }), h('span', { class: 'dim', text: hint })), wrap);
}

const MOVE_COLORS = ['#2563eb', '#16a34a', '#d97706', '#9333ea', '#0891b2', '#db2777', '#65a30d', '#ea580c'];

/**
 * @param root     the element to fill
 * @param a        the analysis from stats.js
 * @param run      the run JSON
 * @param actions  { seek(seconds), replay(), rerun() }
 */
export function renderStats(root, a, run, actions) {
  const n = a.numbers;
  const verdictText = a.verdictText;
  const failed = a.checks.filter((c) => c.status === 'fail').length;
  const sub = a.verdict === 'clean'
    ? `All ${a.checks.length} checks passed.`
    : `${failed ? `${failed} failed, ` : ''}${a.needLook - failed} need${a.needLook - failed === 1 ? 's' : ''} a look. ${a.checks.length - a.needLook} of ${a.checks.length} passed.`;

  const hero = h('section', { class: 'card summary', 'data-verdict': a.verdict },
    h('div', { class: 'verdict' },
      h('span', { class: 'badge', text: a.verdict === 'clean' ? '✓' : a.verdict === 'failed' ? '×' : '!' }),
      h('div', {},
        h('p', { class: 'eyebrow', text: `${run.opMode} · ${run.alliance} alliance${changed0(run) ? ` · custom setup (${changed0(run)})` : ''}` }),
        h('h1', { text: verdictText }),
        h('p', { class: 'dim', text: sub }))),
    h('div', { class: 'actions' },
      h('button', { class: 'primary', onclick: actions.replay, text: 'Watch replay' }),
      h('button', { onclick: actions.rerun, text: 'Run again' })));

  const checks = h('section', { class: 'card' },
    h('div', { class: 'cardHead' }, h('h3', { text: 'Checks' }), h('span', { class: 'dim', text: 'Click one to see the moment in the replay' })),
    h('ul', { class: 'checks' }, a.checks.map((c) => h('li', {
      class: `check ${c.status}${c.seek !== null ? ' link' : ''}`,
      tabindex: c.seek !== null ? 0 : undefined,
      onclick: c.seek !== null ? () => actions.seek(c.seek) : undefined,
      onkeydown: c.seek !== null ? (e) => { if (e.key === 'Enter') actions.seek(c.seek); } : undefined,
    },
      h('span', { class: 'icon', text: ICONS[c.status] }),
      h('div', {}, h('strong', { text: c.label }), h('p', { text: c.detail }))))));

  const tiles = [
    ['Distance driven', n.distance / 1000, 2, 'm', `${fmt(n.rotationDeg, 0)}° of turning`],
    ['Moving time', n.movingTime, 1, 's', `of ${fmt(n.used, 1)} s · ${fmt(n.idleTotal, 1)} s standing still`],
    ['Speed while moving', n.avgMovingSpeed / 1000, 2, 'm/s', 'average'],
    ['Peak speed', n.peakSpeed / 1000, 2, 'm/s', `at ${fmt(n.peakSpeedAt, 1)} s`],
    ['Peak acceleration', n.peakAccel / 1000, 1, 'm/s²', `at ${fmt(n.peakAccelAt, 1)} s, impacts included`],
    ['Contacts', n.contacts, 0, '', n.setupContacts ? `plus ${n.setupContacts} before START` : 'during the match'],
    ['Odometry error', n.odoMax, 0, 'mm', `worst · ${fmt(n.odoFinal, 0)} mm at the end · ${fmt(n.odoMean, 0)} mm average`],
    ['Heading error', n.headingMax, 1, '°', `worst · ${fmt(n.headingFinal, 1)}° at the end`],
    ['Simulated speed', n.speedup, 0, '× real time', n.simMs ? `${fmt(n.simMs / 1000, 1)} s to simulate ${fmt(a.duration, 0)} s` : 'not measured'],
  ];
  if (a.hasBalls) {
    tiles.splice(5, 0,
      ['Balls picked up', n.pickups, 0, '', `${n.heldAtEnd} held at the end`],
      ['Shots fired', n.shots, 0, '', n.shots ? `${n.shotsIn} went into a Hive Cell` : 'none'],
      ['Hive tips', n.hiveTips, 0, '', n.hiveTips ? '20 points each in the manual' : 'none']);
  }
  const numberEls = [];
  const grid = h('section', { class: 'tiles' }, tiles.map(([label, value, decimals, unit, hint]) => {
    const valueEl = h('span', { class: 'value', text: '0' });
    numberEls.push([valueEl, value, decimals]);
    return h('div', { class: 'card tile' },
      h('span', { class: 'label', text: label }),
      h('div', {}, valueEl, h('span', { class: 'unit', text: unit })),
      h('span', { class: 'hint dim', text: hint }));
  }));

  const marks = a.contacts.map((c) => ({ t: c.t }));
  const speedChart = lineChart({ title: 'Speed', hint: 'Shaded = standing still', unit: 'm/s', series: a.series.speed, duration: a.duration,
    color: 'var(--accent)', marks, bands: a.idle, scale: 1 / 1000, decimals: 2, onSeek: actions.seek });
  const odoChart = a.series.odo.length
    ? lineChart({ title: 'Odometry error', hint: 'How far the code thinks it is from the real robot', unit: 'mm', series: a.series.odo, duration: a.duration,
      color: 'var(--ghost)', marks, threshold: THRESHOLDS.odoMmWarn, decimals: 0, onSeek: actions.seek })
    : null;

  const heldChart = a.hasBalls && a.series.held.length
    ? lineChart({ title: 'Balls held', hint: 'In the robot (the limit is 4)', unit: 'balls', series: a.series.held, duration: a.duration,
      color: 'var(--good)', marks, decimals: 0, onSeek: actions.seek })
    : null;

  let decisions = null;
  if (a.decisions.length) {
    const colorOf = {};
    Object.keys(a.decisionTime).forEach((m, i) => { colorOf[m] = MOVE_COLORS[i % MOVE_COLORS.length]; });
    decisions = h('section', { class: 'card' },
      h('div', { class: 'cardHead' }, h('h3', { text: 'What decide() chose' }), h('span', { class: 'dim', text: 'Click a stretch to jump to it' })),
      h('div', { class: 'timeline' }, a.decisions.map((d) => h('button', {
        class: 'seg', title: `${d.move}: ${fmt(d.from, 1)}–${fmt(d.to, 1)} s`,
        style: `flex:${Math.max(0.2, d.to - d.from)};background:${colorOf[d.move]}`, onclick: () => actions.seek(d.from) }))),
      h('ul', { class: 'legend' }, Object.entries(a.decisionTime).map(([move, seconds]) => h('li', {},
        h('i', { style: `background:${colorOf[move]}` }), h('span', { text: move }),
        h('span', { class: 'dim', text: `${fmt(seconds, 1)} s · ${fmt((seconds / a.duration) * 100, 0)}%` })))));
  }

  const contacts = h('section', { class: 'card' },
    h('div', { class: 'cardHead' }, h('h3', { text: 'Contacts' }), h('span', { class: 'dim', text: `${a.contacts.length} during the match` })),
    a.contacts.length + a.setupContacts.length === 0
      ? h('p', { class: 'dim', text: 'The robot touched nothing.' })
      : h('table', { class: 'table' }, h('tbody', {}, [...a.setupContacts, ...a.contacts].map((c) => h('tr', {
        class: c.t < 0 ? 'dimRow' : '', onclick: () => actions.seek(Math.max(0, c.t)) },
        h('td', { text: c.t < 0 ? 'before START' : `${fmt(c.t, 2)} s` }), h('td', { text: c.name }), h('td', { class: 'go', text: 'Show →' }))))));

  const r = run.field.robot;
  const changed = Object.values(run.params || {}).filter((p) => p.used !== p.default);
  const customRobot = ['robotFront', 'robotBack', 'robotWidth'].some((id) => run.params && run.params[id] && run.params[id].used !== run.params[id].default);
  const trust = h('section', { class: 'card' },
    h('div', { class: 'cardHead' }, h('h3', { text: 'How far to trust this run' }), h('span', { class: 'dim', text: 'These are guesses until measured on the real robot' })),
    h('ul', { class: 'plain' }, [
      changed.length ? `You changed: ${changed.map((p) => `${p.label} ${p.default} → ${p.used} ${p.unit}`).join(', ')}. Everything else is as in the code.` : null,
      customRobot
        ? `Robot body: front ${r.front}, back ${r.back}, width ${r.width} mm, as set on the Setup screen. It decides every contact above.`
        : `Robot body: front ${r.front}, back ${r.back}, width ${r.width} mm. Placeholder in RobotFootprint.java. It decides every contact above.`,
      'Hive leg lean: placeholder in FieldConfig.java. The legs are the lean-inflated boxes.',
      'Loop times: main 10 ms, odometry 2 ms (SimRunner). They set how often the code reads sensors and updates motors.',
      'Wheel response, robot mass and battery voltage are estimates (SimWorld, RapierWorld). The wheel speed per power comes from the code\'s own K_v.',
      a.hasBalls ? 'Ball mass, bounciness, the Hive tip weight, the shot speed per flywheel RPS, the hood angles and the ball exit height are estimates. Change them on the Setup screen.' : null,
      run.assumed && run.assumed.length
        ? `Assumed to work (the sim does not model these yet, so each call returned a harmless value): ${run.assumed.join('; ')}.`
        : 'Everything the code asked of the hardware was simulated.',
    ].filter(Boolean).map((t) => h('li', { text: t }))));

  const meta = run.meta || {};
  const foot = h('p', { class: 'footnote dim', text: [
    meta.runId ? `Run #${meta.runId}` : null, `${n.frames} frames`, meta.buildMs !== undefined ? `built in ${fmt(meta.buildMs / 1000, 1)} s` : null,
    n.simMs ? `simulated in ${fmt(n.simMs / 1000, 1)} s` : null, meta.finishedAt ? new Date(meta.finishedAt).toLocaleTimeString() : null,
  ].filter(Boolean).join(' · ') });

  const cards = [hero, checks, grid, speedChart, heldChart, odoChart, decisions, contacts, trust, foot].filter(Boolean);
  cards.forEach((c, i) => c.style.setProperty('--d', `${Math.min(i, 9) * 70}ms`));
  root.replaceChildren(...cards);
  numberEls.forEach(([el, value, decimals]) => countUp(el, value, decimals));
}
