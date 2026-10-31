// The summary rail on the left of the workspace (it replaces the old Stats page). Plain DOM and SVG.
// Every number comes from the recorded run; this file only lays it out. Views: summary, check, checks, numbers, trust.

import { THRESHOLDS, eventText, fmtLen, placeName, nextProblem } from './stats.js';
import { clusterMarkers, eventKind } from './timeline.js';
import { badge, icon } from './icons.js';

const SVG = 'http://www.w3.org/2000/svg';

export function h(tag, props = {}, ...kids) {
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

const fmt = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '—');
const STATUS_WORD = { pass: 'PASS', warn: 'NEEDS A LOOK', fail: 'FAIL', none: 'NOT CHECKED' };
const VERDICT_STATUS = { clean: 'pass', attention: 'warn', failed: 'fail' };

/** Numbers inside a sentence get the mono face. */
export function mono(text) {
  const out = [];
  let last = 0;
  for (const m of text.matchAll(/\d[\d.,]*(?:\s?(?:mm|ft\/s²|ft\/s|ft|in|s|°|pts?))?/g)) {
    if (m.index > last) out.push(text.slice(last, m.index));
    out.push(h('b', { class: 'm', text: m[0] }));
    last = m.index + m[0].length;
  }
  if (last < text.length) out.push(text.slice(last));
  return out;
}

export function niceMax(v) {
  if (!(v > 0)) return 1;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  for (const m of [1, 2, 2.5, 5, 10]) if (v <= m * p) return m * p;
  return 10 * p;
}

/** A line chart over match time: hover for the value, click to put the playhead there. */
export function lineChart({ title, hint, unit, series, duration, color, marks = [], bands = [], threshold = null, scale = 1, decimals = 0, onSeek }) {
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
  for (const b of bands) root.append(svg('rect', { x: x(b.from), y: T, width: Math.max(1, x(b.to) - x(b.from)), height: H - T - B, class: 'band' }));
  if (threshold !== null) {
    root.append(svg('line', { x1: L, x2: W - R, y1: y(threshold), y2: y(threshold), class: 'threshold' }));
    const label = svg('text', { x: W - R - 4, y: y(threshold) - 5, class: 'tick', 'text-anchor': 'end' });
    label.textContent = `fine under ${threshold}`;
    root.append(label);
  }
  for (const m of marks) root.append(svg('line', { x1: x(m.t), x2: x(m.t), y1: T, y2: H - B, class: 'mark' }));

  const points = series.map(([t, v]) => `${x(t).toFixed(1)},${y(v * scale).toFixed(1)}`);
  root.append(svg('polygon', { points: `${x(series[0][0])},${H - B} ${points.join(' ')} ${x(series[series.length - 1][0])},${H - B}`, class: 'area', style: `fill:${color}` }));
  root.append(svg('polyline', { points: points.join(' '), class: 'line', style: `stroke:${color}` }));
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
    const [pt, pv] = nearest(timeAt(event));
    const cx = x(pt);
    cursor.setAttribute('x1', cx); cursor.setAttribute('x2', cx); cursor.setAttribute('opacity', 1);
    dot.setAttribute('cx', cx); dot.setAttribute('cy', y(pv * scale)); dot.setAttribute('opacity', 1);
    tip.hidden = false;
    tip.textContent = `${fmt(pt, 2)} s · ${fmt(pv * scale, decimals)} ${unit}`;
    tip.style.left = `${(cx / W) * 100}%`;
  });
  root.addEventListener('pointerleave', () => { cursor.setAttribute('opacity', 0); dot.setAttribute('opacity', 0); tip.hidden = true; });
  root.addEventListener('click', (event) => onSeek(timeAt(event)));

  return h('section', { class: 'chartCard' }, h('div', { class: 'cardHead' }, h('span', { class: 'h', text: title }), h('span', { class: 'dim', text: hint })), wrap);
}

// ---------------------------------------------------------------------------------------------
// Pieces
// ---------------------------------------------------------------------------------------------

const estMark = () => h('span', { class: 'est', title: 'A guess, not measured', text: '~' });

function timeButton(t, actions) {
  return h('button', { class: 'timebtn m', text: `${t.toFixed(1)} s`, 'aria-label': `Jump to ${t.toFixed(1)} seconds`, onclick: (e) => { e.stopPropagation(); actions.seek(t); } });
}

function navRow({ iconName, title, sub, onclick, estimate }) {
  return h('button', { class: 'navrow', onclick },
    h('span', { class: 'ic' }, icon(iconName)),
    h('span', { class: 'txt' }, h('b', { text: title }), h('span', { class: 'sub' }, estimate ? estMark() : null, sub)),
    h('span', { class: 'go' }, icon('chevron')));
}

const BACK = (actions, label = 'Back to summary') => h('button', { class: 'back', onclick: () => actions.open('summary') }, '← ', label);

const CHANGE_ICON = { contacts: 'contact', shotsIn: 'shots', shots: 'shots', odometry: 'odometry', odoMax: 'odometry', heading: 'odometry', pickups: 'ball', score: 'checks', tips: 'hive', side: 'robot', partner: 'partner' };

function changeRow(d, actions) {
  const tone = d.kind === 'new' ? 'warn' : d.kind === 'fixed' ? 'pass' : 'none';
  let body;
  if (d.kind === 'changed') {
    const up = d.to > d.from;
    body = [d.label, ' ', h('b', { class: 'm', text: String(d.from) }), ' → ', h('b', { class: 'm', text: String(d.to) }),
      h('span', { class: `arrow ${d.better ? 'good' : 'bad'}`, 'aria-label': d.better ? 'better' : 'worse', text: up ? '↑' : '↓' })];
  } else body = [d.label];
  return h('div', { class: 'change', role: d.seek !== null && d.seek !== undefined ? 'button' : undefined, tabindex: d.seek != null ? 0 : undefined,
    onclick: d.seek != null ? () => actions.seek(d.seek) : undefined,
    onkeydown: d.seek != null ? (e) => { if (e.key === 'Enter') actions.seek(d.seek); } : undefined },
  h('span', { class: `ic tone-${tone}` }, icon(CHANGE_ICON[d.id] || 'checks')),
  h('span', { class: 'txt' }, ...body),
  d.seek != null ? timeButton(d.seek, actions) : null);
}

// ---------------------------------------------------------------------------------------------
// Views
// ---------------------------------------------------------------------------------------------

function summaryView(m, actions) {
  const { a, diff, score, est, prevId } = m;
  const status = VERDICT_STATUS[a.verdict];
  const crashed = a.checks.find((c) => c.id === 'ran').status === 'fail';
  const newCount = diff.filter((d) => d.kind === 'new').length;
  const compare = m.prevA
    ? [` · vs #${prevId}: `, h('b', { text: newCount ? `${newCount} new problem${newCount === 1 ? '' : 's'}` : 'no new problems' })]
    : [];
  const verdict = h('section', { class: 'verdictcard', 'data-status': status },
    h('div', { class: 'vhead' }, a.verdict === 'failed' && crashed ? badge('fail', 'blocked') : badge(status), h('span', { class: 'vword', text: a.verdictText.toUpperCase() })),
    h('p', {}, `${a.needLook} of ${a.checks.length} checks need a look`, ...compare));

  const changes = h('section', { class: 'block' },
    h('h3', { class: 'h', text: m.prevA ? `Changed since #${prevId}` : 'Changed since the last run' }),
    m.prevA
      ? (diff.length ? diff.map((d) => changeRow(d, actions)) : h('p', { class: 'dim pad', text: 'Nothing changed.' }))
      : h('p', { class: 'dim pad', text: 'First run, nothing to compare yet.' }));

  const n = a.numbers;
  const guesses = est.size;
  const more = h('section', { class: 'block' }, h('h3', { class: 'h', text: 'More' }),
    navRow({ iconName: 'checks', title: 'All checks', sub: `${a.checks.filter((c) => c.status === 'pass').length} pass · ${a.needLook} need a look`, onclick: () => actions.open('checks') }),
    m.run.partner && m.partnerCheck
      ? navRow({ iconName: 'partner', title: 'Partner R2', sub: `path file · closest ${fmtLen(a.numbers.partnerClosest)} at ${fmt(a.numbers.partnerClosestAt)} s`, onclick: () => actions.open('check', 'partner') })
      : null,
    navRow({ iconName: 'trust', title: 'How far to trust this', estimate: guesses > 0, sub: guesses ? ` ${m.guessCount} guesses affect ${m.guessAffects}` : 'Everything here is measured or read from your code', onclick: () => actions.open('trust') }),
    navRow({ iconName: 'numbers', title: 'Run numbers', sub: [a.hasBalls ? `${n.pickups} picked up · ${n.shots} shots · ` : '', `${fmtLen(n.distance)} driven`].join(''), onclick: () => actions.open('numbers') }));
  return [verdict, changes, more];
}

const THRESHOLD_LINE = (c) => {
  const T = THRESHOLDS;
  switch (c.id) {
    case 'odometry': return [`Pass: under ${T.odoMmWarn} mm`, `Fail: over ${T.odoMmFail} mm`];
    case 'heading': return [`Pass: under ${T.headingWarn}°`, `Fail: over ${T.headingFail}°`];
    case 'contacts': return ['Pass: 0 contacts', 'Needs a look: any'];
    case 'side': return ['Pass: body stays behind the midline', 'Fail: crosses it'];
    case 'idle': return [`Pass: under ${T.idleTailWarn} s still at the end`, `Needs a look: ${T.idleTailWarn} s or more`];
    case 'shots': return ['Pass: every shot lands', 'Fail: none land'];
    case 'tips': return ['Pass: a Hive tips', 'Needs a look: none tips'];
    case 'partner': return [`Pass: never closer than ${fmtLen(T.partnerWarnMm ?? 150)}`, 'Fail: touches R2'];
    case 'ran': return ['Pass: runs to the end with no error', 'Fail: error or no finish'];
    default: return ['Pass: no warnings', 'Needs a look: any warning'];
  }
};

function nearMoment(a, run, c, actions) {
  if (c.seek === null || c.seek === undefined) return null;
  const near = run.events.filter((e) => e.t >= 0 && Math.abs(e.t - c.seek) <= 0.5);
  if (!near.length) return null;
  const clusters = clusterMarkers(near, a.duration, 1200);
  const rowFor = (e) => h('div', { class: `evrow${Math.abs(e.t - c.seek) < 0.005 ? ' here' : ''}`, onclick: () => actions.seek(e.t) },
    h('span', { class: 'm t', text: e.t.toFixed(2) }), kindDot(eventKind(e.text)), h('span', { text: eventText(e.text) }));
  return h('section', { class: 'block' }, h('h3', { class: 'h', text: 'Near this moment' }),
    clusters.map((cl) => {
      if (cl.events.length === 1) return rowFor(cl.events[0]);
      const span = cl.events[cl.events.length - 1].t - cl.events[0].t;
      return h('div', { class: 'evgroup' }, h('div', { class: 'h', text: `${cl.events.length} events in ${span.toFixed(2)} s` }), cl.events.map(rowFor));
    }));
}

function kindDot(kind) {
  return h('span', { class: `kdot k-${kind}` });
}

function checkView(m, actions) {
  const { a, run } = m;
  const c = a.checks.find((x) => x.id === m.checkId) || a.checks[0];
  const [pass, fail] = THRESHOLD_LINE(c);
  const prev = nextProblem(a, c.seek ?? 0, -1), next = nextProblem(a, c.seek ?? 0, 1);
  const count = a.checks.filter((x) => x.status !== 'pass').length;
  const card = h('section', { class: 'verdictcard check', 'data-status': c.status },
    h('div', { class: 'vhead' }, badge(c.status), h('b', { class: 'ctitle', text: c.label }), h('span', { class: `sword s-${c.status}`, text: STATUS_WORD[c.status] })),
    h('p', {}, ...mono(c.detail)),
    h('p', { class: 'thresholds dim' }, h('span', { text: pass }), h('span', { text: fail })),
    h('div', { class: 'btnrow' },
      h('button', { class: 'btn', onclick: () => prev && actions.open('check', prev.id) }, '← Prev'),
      h('button', { class: 'btn', onclick: () => next && actions.open('check', next.id) }, count ? `Next problem` : 'No problems', h('span', { class: 'kbd', text: 'N' }))));
  return [BACK(actions), card, nearMoment(a, run, c, actions)];
}

function checksView(m, actions) {
  const { a, est } = m;
  return [BACK(actions), h('h3', { class: 'h pad', text: 'All checks' }),
    h('div', { class: 'list' }, a.checks.map((c) => h('button', { class: 'checkrow', onclick: () => { actions.open('check', c.id); if (c.seek != null) actions.seek(c.seek); } },
      badge(c.status),
      h('span', { class: 'txt' }, h('b', { text: c.label }), h('span', { class: 'sub', text: c.detail })),
      est.has(c.id) || (c.id === 'shots' && est.has('shots')) ? estMark() : null)))];
}

function numbersView(m, actions) {
  const { a, run } = m;
  const n = a.numbers;
  const row = (label, value, seek) => h('div', { class: `numrow${seek != null ? ' link' : ''}`, onclick: seek != null ? () => actions.seek(seek) : undefined }, h('span', { text: label }), h('b', { class: 'm', text: value }));
  const rows = [
    row('Distance driven', `${fmtLen(n.distance)} · ${fmt(n.rotationDeg, 0)}° turned`),
    row('Moving time', `${fmt(n.movingTime)} s of ${fmt(n.used)} s`),
    row('Speed while moving', `${fmt(n.avgMovingSpeed / 304.8)} ft/s`),
    row('Peak speed', `${fmt(n.peakSpeed / 304.8)} ft/s`, n.peakSpeedAt),
    row('Peak acceleration', `${fmt(n.peakAccel / 304.8)} ft/s²`, n.peakAccelAt),
    row('Contacts', `${n.contacts}${n.setupContacts ? ` + ${n.setupContacts} before START` : ''}`),
    row('Odometry error', `${fmt(n.odoMax, 0)} mm max · ${fmt(n.odoMean, 0)} mm avg`, n.odoMaxAt),
    row('Heading error', `${fmt(n.headingMax)}° max`, n.headingMaxAt),
  ];
  if (a.hasBalls) rows.push(row('Balls picked up', `${n.pickups} · ${n.heldAtEnd} held at the end`), row('Shots', `${n.shotsIn} of ${n.shots} landed`), row('Hive tips', String(n.hiveTips)));
  rows.push(row('Simulated speed', n.speedup ? `${fmt(n.speedup, 0)}× real time` : '—'));
  const marks = a.contacts.map((c) => ({ t: c.t }));
  const charts = [
    lineChart({ title: 'Speed', hint: 'Shaded = standing still', unit: 'ft/s', series: a.series.speed, duration: a.duration, color: 'var(--accent-text)', marks, bands: a.idle, scale: 1 / 304.8, decimals: 1, onSeek: actions.seek }),
    a.hasBalls && a.series.held.length ? lineChart({ title: 'Balls held', hint: 'The limit is 4', unit: 'balls', series: a.series.held, duration: a.duration, color: 'var(--pass)', marks, onSeek: actions.seek }) : null,
    a.series.odo.length ? lineChart({ title: 'Odometry error', hint: 'Code vs real robot', unit: 'mm', series: a.series.odo, duration: a.duration, color: 'var(--body)', marks, threshold: THRESHOLDS.odoMmWarn, onSeek: actions.seek }) : null,
  ];
  return [BACK(actions), h('h3', { class: 'h pad', text: 'Run numbers' }), h('div', { class: 'list' }, rows), ...charts];
}

const AFFECT_WORDS = { shots: 'Shots land in a Cell', tips: 'Tips a Hive', score: 'Score', contacts: 'Contacts', side: 'Stays on its side' };

function trustView(m) {
  const { run, paramDefs, actions } = m;
  const used = (def) => (run.params && run.params[def.id] ? run.params[def.id].used : null);
  const unmeasured = paramDefs.filter((d) => d.unmeasured);
  const r = run.field.robot;
  const lines = [
    `Robot body: front ${r.front}, back ${r.back}, width ${r.width} mm. ${run.params?.robotWidth && run.params.robotWidth.used !== run.params.robotWidth.default ? 'Changed in the sim.' : 'Read from RobotFootprint.java.'} It decides every contact.`,
    'Hive leg lean (FieldConfig placeholder): the legs are drawn dashed.',
    'Loop times: main 10 ms, odometry 2 ms (SimRunner).',
    'Wheel response, robot mass, battery voltage (SimWorld, RapierWorld).',
  ];
  return [BACK(actions), h('h3', { class: 'h pad', text: 'How far to trust this' }),
    h('div', { class: 'list' }, unmeasured.map((def) => {
      const v = used(def);
      return h('div', { class: 'trustrow' },
        h('div', { class: 'tr-top' }, estMark(), h('b', { text: def.label }), h('span', { class: 'm val', text: v === null ? '—' : `${v} ${def.unit}` })),
        h('div', { class: 'chips' }, (def.affects || []).map((id) => h('span', { class: 'chipx', text: AFFECT_WORDS[id] || id }))));
    })),
    unmeasured.length ? null : h('p', { class: 'dim pad', text: 'No unmeasured values in play.' }),
    h('h3', { class: 'h pad', text: 'Also assumed' }),
    h('ul', { class: 'plain' }, [...lines, ...(run.assumed || []).map((t) => `Assumed to work: ${t}`)].map((t) => h('li', { text: t })))];
}

// ---------------------------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------------------------

/**
 * One error card for every failure: compile error, crash during the run, runner offline.
 * opts: { title, where, message, context: [{n, text, caret}], detail, copyText, actions: [{label, onclick, primary}], footer, raw }
 */
export function errorCard(opts) {
  const lines = (opts.context || []).map((l) => h('div', { class: `srcline${l.caret ? ' err' : ''}` },
    h('span', { class: 'n m', text: String(l.n) }), h('span', { class: 'code m', text: l.text }), l.caret ? h('span', { class: 'caret m', text: `^${l.caret}` }) : null));
  return h('section', { class: 'errorcard', role: 'alert' },
    h('div', { class: 'vhead' }, badge('fail'), h('b', { class: 'etitle', text: opts.title })),
    opts.where ? h('p', { class: 'where m', text: opts.where }) : null,
    opts.message ? h('p', { class: 'emsg m', text: opts.message }) : null,
    lines.length ? h('div', { class: 'src' }, lines) : null,
    opts.raw ? h('pre', { class: 'raw m', text: opts.raw }) : null,
    opts.detail ? h('p', { class: 'edetail', text: opts.detail }) : null,
    h('div', { class: 'btnrow' }, (opts.actions || []).map((x) => h('button', { class: `btn${x.primary ? ' btn-p' : ''}`, onclick: x.onclick, text: x.label }))),
    opts.footer ? h('p', { class: 'dim small', text: opts.footer }) : null);
}

// ---------------------------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------------------------

/**
 * @param root     the #rail element
 * @param model    { a, run, prevA, prevId, diff, score, est: Set, view, checkId, paramDefs, problem?, oldRun?: {a, id, time} , guessCount, guessAffects }
 * @param actions  { seek(t), open(view, checkId?), rerun() }
 */
export function renderRail(root, model, actions) {
  const parts = [];
  if (model.problem) {
    parts.push(model.problem.card);
    if (model.a) {
      parts.push(h('div', { class: 'oldline' }, h('span', { class: 'tag old-tag', text: 'OLD' }), h('span', { class: 'dim', text: model.problem.oldText || '' })));
      const old = h('div', { class: 'old' }, ...summaryView(model, actions));
      parts.push(old);
    }
    root.replaceChildren(...parts);
    return;
  }
  if (!model.a) {
    root.replaceChildren(h('p', { class: 'dim pad', text: 'Waiting for the first run…' }));
    return;
  }
  const view = { summary: summaryView, check: checkView, checks: checksView, numbers: numbersView, trust: trustView }[model.view] || summaryView;
  root.replaceChildren(...view({ ...model, actions }, actions).filter(Boolean));
}

export { VERDICT_STATUS, STATUS_WORD, placeName };
