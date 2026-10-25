// The Parameters screen: the numbers you can change before a run. The helpers at the top are pure (../params.test.mjs).
// Values shown here are sim values; the server checks them again and Java applies them (SimParams.java).

import { h } from './rail.js';
import { badge, icon } from './icons.js';

const num = (v) => (Number.isInteger(v) ? v.toLocaleString('en-US') : String(Number(v.toFixed(4))));

/** Why this value cannot be run, in a sentence, or null when it is fine. */
export function paramProblem(def, value) {
  const unit = def.unit ? ` ${def.unit}` : '';
  if (typeof value !== 'number' || !Number.isFinite(value)) return `${def.label} needs a number. Runs are blocked until you fix it.`;
  if (value > def.max) return `${num(value)}${unit} is above the max (${num(def.max)}). Runs are blocked until you fix it.`;
  if (value < def.min) return `${num(value)}${unit} is below the min (${num(def.min)}). Runs are blocked until you fix it.`;
  return null;
}

/** The defs a filter and a search show. filter: all | changed | problems | unmeasured | code | <group name>. */
export function filterParams(defs, values, defaults, { filter = 'all', query = '' } = {}) {
  const q = query.trim().toLowerCase();
  return defs.filter((d) => {
    if (filter === 'changed' && !(values[d.id] !== defaults[d.id])) return false;
    if (filter === 'problems' && (d.type === 'choice' || !paramProblem(d, values[d.id]))) return false;
    if (filter === 'unmeasured' && !d.unmeasured) return false;
    if (filter === 'code' && !d.source) return false;
    if (!['all', 'changed', 'problems', 'unmeasured', 'code'].includes(filter) && d.group !== filter) return false;
    if (q && ![d.label, d.id, d.description, d.group].some((s) => (s || '').toLowerCase().includes(q))) return false;
    return true;
  });
}

const GROUP_INFO = {
  'Robot size': ['robot', 'millimetres from the robot’s centre'],
  Shooter: ['shooter', 'flywheel, hood and shot timing'],
  Balls: ['ball', 'mass, friction and bounce'],
  Hive: ['hive', 'tip weight and swing'],
  Intake: ['intake', 'how far the intake reaches'],
  'Partner robot': ['partner', 'R2 path file and speed'],
};
const AFFECT_WORDS = { shots: 'Shots land in a Cell', tips: 'Tips a Hive', score: 'Score', contacts: 'Contacts', side: 'Stays on its side' };

const srcTag = (file) => h('span', { class: 'src' }, icon('code'), file);
const estMark = () => h('span', { class: 'est', title: 'A guess, not measured', text: '~' });

/**
 * Builds the screen inside root.
 * ctx = { defs(), defaults(), values(), setValue(id, v), resetAll(), run(), back(), start(): {x,y,headingDeg}|null, field(): run.field|null,
 *         partnerGroup?: () => { count, subtitle, rows: Node[] } }
 * Returns { refresh(), focusSearch(), show() }.
 */
export function mountParams(root, ctx) {
  let filter = 'all', query = '', focusId = null;
  const rows = new Map();     // id -> { row, input, unitEl, reset, detail, track, ... }
  let navEls = {}, groupEls = new Map(), asideEls = {};

  const search = h('input', { class: 'search', type: 'search', placeholder: 'Search, e.g. flywheel', 'aria-label': 'Search parameters' });
  search.addEventListener('input', () => { query = search.value; build(); });
  const header = h('header', { class: 'phead' },
    h('a', { class: 'pback', href: '#/run', onclick: (e) => { e.preventDefault(); ctx.back(); } }, '← Workspace'),
    h('h1', { text: 'Parameters' }), h('span', { class: 'dim psub', id: 'paramsSub' }),
    h('span', { class: 'spacer' }),
    h('label', { class: 'searchbox' }, icon('numbers'), search, h('span', { class: 'kbd', text: '/' })));
  const nav = h('nav', { class: 'pnav', 'aria-label': 'Filters' });
  const main = h('div', { class: 'pmain' });
  const aside = h('aside', { class: 'paside' });
  root.replaceChildren(header, h('div', { class: 'pbody' }, nav, main, aside));

  const valueOf = (id) => ctx.values()[id];
  const changedIds = () => ctx.defs().filter((d) => valueOf(d.id) !== ctx.defaults()[d.id]);
  const problemIds = () => ctx.defs().filter((d) => d.type !== 'choice' && paramProblem(d, valueOf(d.id)));

  function navButton(id, iconEl, label, count) {
    const b = h('button', { class: 'navbtn', 'aria-pressed': String(filter === id), onclick: () => { filter = id; build(); } },
      iconEl, h('span', { class: 'lbl', text: label }), h('span', { class: 'cnt m', text: String(count) }));
    navEls[id] = b;
    return b;
  }

  function groupsOf() {
    const names = [...new Set(ctx.defs().map((d) => d.group))];
    const extra = ctx.partnerGroup ? ctx.partnerGroup() : null;
    return { names, extra };
  }

  function buildNav() {
    navEls = {};
    const defs = ctx.defs();
    const { names, extra } = groupsOf();
    const dot = h('span', { class: 'dotc' });
    nav.replaceChildren(
      navButton('all', h('span', { class: 'ico0' }), 'All', defs.length + (extra ? extra.count : 0)),
      navButton('changed', dot, 'Changed', changedIds().length),
      navButton('problems', badge('fail'), 'Problems', problemIds().length),
      navButton('unmeasured', estMark(), 'Unmeasured', defs.filter((d) => d.unmeasured).length),
      navButton('code', h('span', { class: 'src0' }, icon('code')), 'From your code', defs.filter((d) => d.source).length),
      h('h2', { class: 'h', text: 'Groups' }),
      ...names.map((g) => navButton(g, h('span', { class: 'ic' }, icon((GROUP_INFO[g] || ['params'])[0])), g, defs.filter((d) => d.group === g).length)),
      extra ? navButton('Partner robot', h('span', { class: 'ic' }, icon('partner')), 'Partner robot', extra.count) : null);
  }

  function inputFor(def) {
    if (def.type === 'choice') {
      const group = h('div', { class: 'choice', role: 'radiogroup', 'aria-label': def.label });
      const buttons = def.options.map((o) => {
        const b = h('button', { type: 'button', role: 'radio', onclick: () => { ctx.setValue(def.id, o.value); refresh(); } }, o.label);
        group.append(b);
        return { b, value: o.value };
      });
      return { el: group, sync: () => buttons.forEach(({ b, value }) => b.setAttribute('aria-checked', String(value === valueOf(def.id)))) };
    }
    const input = h('input', { class: 'numin m', type: 'number', step: def.step, inputmode: 'decimal', 'aria-label': `${def.label}${def.unit ? ` in ${def.unit}` : ''}` });
    input.addEventListener('input', () => {
      const v = input.value === '' ? NaN : Number(input.value);
      ctx.setValue(def.id, v);
      refresh(def.id);
    });
    input.addEventListener('focus', () => { focusId = def.id; refresh(); });
    input.addEventListener('blur', () => { if (focusId === def.id) { focusId = null; refresh(true); } });
    return { el: input, input, sync: (skip) => { if (!skip) input.value = Number.isNaN(valueOf(def.id)) ? '' : String(valueOf(def.id)); } };
  }

  function rangeBar(def) {
    const pct = (v) => `${Math.min(100, Math.max(0, ((v - def.min) / (def.max - def.min)) * 100))}%`;
    const dflt = ctx.defaults()[def.id];
    const bar = h('div', { class: 'rangebar' },
      h('div', { class: 'rtrack' }, h('i', { class: 'rdef', style: `left:${pct(dflt)}` }), h('i', { class: 'rnow' })),
      h('div', { class: 'rlabels m' }, h('span', { text: num(def.min) }), h('span', { class: 'rdeflbl', style: `left:${pct(dflt)}`, text: `default ${num(dflt)}` }), h('span', { text: `max ${num(def.max)}` })));
    return { el: bar, now: bar.querySelector('.rnow'), pct };
  }

  function buildRow(def) {
    const ctl = inputFor(def);
    const range = def.type === 'choice' ? null : rangeBar(def);
    const label = h('div', { class: 'plabel' }, h('b', { text: def.label }), def.source ? srcTag(def.source) : null, h('span', { class: 'psubl' }));
    const unitEl = h('span', { class: 'unit m', text: def.unit || '' });
    const est = h('span', { class: 'estcol' }, def.unmeasured ? estMark() : null);
    const reset = h('button', { class: 'resetbtn', type: 'button', 'aria-label': `Reset ${def.label.toLowerCase()} to ${ctx.defaults()[def.id]}`, onclick: () => { ctx.setValue(def.id, ctx.defaults()[def.id]); refresh(); } }, '↺');
    const problem = h('div', { class: 'problem', role: 'alert' }, badge('fail'), h('span'));
    const detail = h('div', { class: 'pdetail' },
      h('p', { class: 'pdesc', text: def.description }),
      range ? range.el : null, problem,
      def.affects && def.affects.length ? h('div', { class: 'affects' }, h('span', { text: 'Affects' }), def.affects.map((a) => h('span', { class: 'chipx', text: AFFECT_WORDS[a] || a }))) : null);
    const row = h('div', { class: 'prow', 'data-id': def.id }, h('div', { class: 'prow-main' }, label, ctl.el, unitEl, est, reset), detail);
    ctl.sync();
    rows.set(def.id, { def, row, ctl, label, reset, problem, range });
    return row;
  }

  function section(name, defsIn, extra) {
    const [iconName, subtitle] = GROUP_INFO[name] || ['params', ''];
    const file = defsIn.map((d) => d.source).find(Boolean);
    const stats = h('span', { class: 'gstats' });
    const head = h('header', { class: 'ghead' }, h('span', { class: 'ic' }, icon(iconName)),
      h('div', { class: 'gtitle' }, h('b', { text: name }), h('span', { class: 'dim', text: extra ? extra.subtitle : subtitle })), h('span', { class: 'spacer' }), file ? srcTag(file) : null, stats);
    groupEls.set(name, { stats, defs: defsIn });
    return h('section', { class: 'pgroup' }, head, defsIn.map(buildRow), extra ? extra.rows : null);
  }

  function build() {
    rows.clear(); groupEls = new Map();
    buildNav();
    const defs = filterParams(ctx.defs(), ctx.values(), ctx.defaults(), { filter, query });
    const names = [...new Set(defs.map((d) => d.group))];
    const { extra } = groupsOf();
    const parts = names.map((g) => section(g, defs.filter((d) => d.group === g)));
    const showPartner = extra && (filter === 'all' || filter === 'Partner robot') && !query;
    if (showPartner) parts.push(section('Partner robot', [], extra));
    if (!parts.length) parts.push(h('p', { class: 'dim pad empty', text: query ? `Nothing matches "${query}".` : 'Nothing here.' }));
    main.replaceChildren(...parts);
    buildAside();
    refresh(true);
  }

  // ---- the start-pose preview (reused from the old Setup screen) ----
  const SVGNS = 'http://www.w3.org/2000/svg';
  const svgEl = (tag, attrs = {}, parent = null) => { const n = document.createElementNS(SVGNS, tag); for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v); if (parent) parent.append(n); return n; };
  let preview = null;
  function drawPreview() {
    if (!preview) return;
    const v = ctx.values(), d = ctx.defaults();
    const size = { front: v.robotFront, back: v.robotBack, width: v.robotWidth };
    const code = { front: d.robotFront ?? size.front, back: d.robotBack ?? size.back, width: d.robotWidth ?? size.width };
    if (![size.front, size.back, size.width].every(Number.isFinite)) return;
    const maxFront = Math.max(size.front, code.front), maxBack = Math.max(size.back, code.back), maxWidth = Math.max(size.width, code.width);
    const scale = Math.min(250 / (maxFront + maxBack), 215 / maxWidth);
    const cx = 220, top = 165 - ((maxFront + maxBack) * scale) / 2, cy = top + maxFront * scale;
    const P = preview;
    const box = (r, s) => { r.setAttribute('x', cx - (s.width * scale) / 2); r.setAttribute('y', cy - s.front * scale); r.setAttribute('width', s.width * scale); r.setAttribute('height', (s.front + s.back) * scale); };
    box(P.ghost, code); box(P.body, size);
    const tipY = cy - size.front * scale, halfW = (size.width * scale) / 2;
    P.arrow.setAttribute('points', `${cx - 14},${tipY + 26} ${cx},${tipY + 10} ${cx + 14},${tipY + 26}`);
    P.centre.setAttribute('cx', cx); P.centre.setAttribute('cy', cy);
    const sideX = cx + Math.max(halfW, (code.width * scale) / 2) + 18;
    P.widthText.setAttribute('x', cx); P.widthText.setAttribute('y', Math.min(tipY, cy - code.front * scale) - 12);
    P.widthText.textContent = `${Math.round(size.width)} mm wide`;
    P.lenText.setAttribute('x', sideX); P.lenText.setAttribute('y', cy + 4);
    P.lenText.textContent = `front ${Math.round(size.front)} · back ${Math.round(size.back)}`;
    P.codeText.setAttribute('x', cx - halfW - 8); P.codeText.setAttribute('y', cy + 4);
    P.codeText.textContent = changedIds().some((c) => c.id.startsWith('robot')) ? 'code' : '';
  }

  function bodyCorners(pose, size) {
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta), hw = size.width / 2;
    return [[size.front, hw], [size.front, -hw], [-size.back, -hw], [-size.back, hw]].map(([lx, ly]) => [pose.x + lx * c - ly * s, pose.y + lx * s + ly * c]);
  }
  function overlapsBox(corners, theta, b) {
    const boxCorners = [[b.minX, b.minY], [b.maxX, b.minY], [b.maxX, b.maxY], [b.minX, b.maxY]];
    const axes = [[1, 0], [0, 1], [Math.cos(theta), Math.sin(theta)], [-Math.sin(theta), Math.cos(theta)]];
    const project = (pts, [ax, ay]) => { const v = pts.map(([x, y]) => x * ax + y * ay); return [Math.min(...v), Math.max(...v)]; };
    return axes.every((axis) => { const a = project(corners, axis), c = project(boxCorners, axis); return !(a[1] <= c[0] || c[1] <= a[0]); });
  }

  function startChecks() {
    const v = ctx.values(), start = ctx.start(), field = ctx.field();
    const size = { front: v.robotFront, back: v.robotBack, width: v.robotWidth };
    const mm = (x) => `${Math.round(x)} mm`;
    const list = [['none', `Body ${mm(size.front + size.back)} long × ${mm(size.width)} wide.`]];
    if (start && [size.front, size.back, size.width].every(Number.isFinite)) {
      const pose = { x: start.x, y: start.y, theta: (start.headingDeg * Math.PI) / 180 };
      const S = field ? field.size : 3657.6;
      const corners = bodyCorners(pose, size);
      let outside = 0, spare = Infinity;
      for (const [x, y] of corners) { outside = Math.max(outside, -x, x - S, -y, y - S); spare = Math.min(spare, x, S - x, y, S - y); }
      list.push(outside > 0.5 ? ['warn', `At the start pose the body is ${mm(outside)} inside the wall, so the robot gets pushed out and odometry starts that far off.`]
        : ['pass', `Fits inside the field at the start pose, with ${mm(spare)} to spare.`]);
      if (field) {
        const hit = field.obstacles.find((o) => overlapsBox(corners, pose.theta, o));
        list.push(hit ? ['warn', `At the start pose the body is inside ${hit.name}.`] : ['pass', 'Clear of every obstacle at the start pose.']);
      }
    }
    return list;
  }

  function buildAside() {
    const svg = svgEl('svg', { viewBox: '0 0 440 330', role: 'img', 'aria-label': 'The robot at its start pose' });
    preview = {
      ghost: svgEl('rect', { class: 'ghostBody', rx: 8 }, svg), body: svgEl('rect', { class: 'body', rx: 8 }, svg),
      arrow: svgEl('polyline', { class: 'arrow' }, svg), centre: svgEl('circle', { class: 'centre', r: 4 }, svg),
      widthText: svgEl('text', { class: 'dimText', 'text-anchor': 'middle' }, svg), lenText: svgEl('text', { class: 'dimText' }, svg),
      codeText: svgEl('text', { class: 'dimText', 'text-anchor': 'end' }, svg),
    };
    const checks = h('ul', { class: 'pchecks' });
    const summary = h('span', { class: 'psum' });
    const resetAll = h('button', { class: 'btn', onclick: () => { ctx.resetAll(); build(); } }, 'Reset all');
    const run = h('button', { class: 'btn btn-p big', onclick: () => ctx.run() }, 'Run');
    asideEls = { checks, summary, run, resetAll };
    aside.replaceChildren(
      h('h2', { class: 'h', text: 'Start pose preview' }), h('div', { class: 'previewbox' }, svg), checks,
      h('div', { class: 'plegend' },
        h('div', {}, estMark(), h('span', { text: 'Guess, not measured yet' })),
        h('div', {}, srcTag('File.java'), h('span', { text: 'Read from your TeamCode' })),
        h('div', {}, h('span', { class: 'dotc' }), h('span', { text: 'Changed in the sim' }))),
      h('span', { class: 'spacer' }),
      h('div', { class: 'afoot' }, summary, resetAll), run);
  }

  /** Updates what depends on the values without rebuilding the rows (so typing keeps its focus). */
  function refresh(skipInputId) {
    const defaults = ctx.defaults(), values = ctx.values();
    if (!defaults) return;
    const problems = problemIds();
    for (const [id, r] of rows) {
      const v = values[id];
      const changed = v !== defaults[id];
      const problem = r.def.type === 'choice' ? null : paramProblem(r.def, v);
      r.row.classList.toggle('changed', changed);
      r.row.classList.toggle('has-problem', Boolean(problem));
      r.row.classList.toggle('open', Boolean(problem) || focusId === id);
      r.reset.hidden = !changed;
      r.label.querySelector('.psubl').textContent = changed ? (r.def.source ? `sim override · code says ${defaults[id]}` : `changed · default ${defaults[id]}`) : '';
      r.problem.hidden = !problem;
      r.problem.querySelector('span').textContent = problem || '';
      if (r.ctl.input) { r.ctl.input.classList.toggle('bad', Boolean(problem)); r.ctl.sync(skipInputId === id || (skipInputId !== true && document.activeElement === r.ctl.input)); }
      else r.ctl.sync();
      if (r.range) {
        const clamped = Number.isFinite(v) ? Math.min(r.def.max, Math.max(r.def.min, v)) : r.def.min;
        r.range.now.style.left = r.range.pct(clamped);
        r.range.now.classList.toggle('bad', Boolean(problem));
      }
    }
    for (const [name, g] of groupEls) {
      const ch = g.defs.filter((d) => values[d.id] !== defaults[d.id]).length;
      const um = g.defs.filter((d) => d.unmeasured).length;
      const pr = g.defs.filter((d) => d.type !== 'choice' && paramProblem(d, values[d.id])).length;
      g.stats.replaceChildren(...[ch ? h('span', { class: 'chg', text: `${ch} changed` }) : null, um ? h('span', { text: `${um} unmeasured` }) : null, pr ? h('span', { class: 'prob', text: `${pr} problem${pr === 1 ? '' : 's'}` }) : null].filter(Boolean));
    }
    if (navEls.changed) {
      navEls.changed.querySelector('.cnt').textContent = String(changedIds().length);
      navEls.problems.querySelector('.cnt').textContent = String(problems.length);
      for (const [id, b] of Object.entries(navEls)) b.setAttribute('aria-pressed', String(filter === id));
    }
    root.querySelector('#paramsSub').textContent = `${ctx.defs().length} · sim values first, then your TeamCode config`;
    if (asideEls.checks) {
      asideEls.checks.replaceChildren(...startChecks().map(([s, t]) => h('li', {}, badge(s), h('span', { text: t }))));
      const nChanged = changedIds().length;
      asideEls.summary.replaceChildren(h('span', { class: 'chg', text: `${nChanged} changed` }), ' · ', h('span', { class: problems.length ? 'prob' : '', text: `${problems.length} problem${problems.length === 1 ? '' : 's'}` }));
      asideEls.run.textContent = problems.length ? `Fix ${problems.length} problem${problems.length === 1 ? '' : 's'} to run` : 'Run';
      asideEls.run.disabled = problems.length > 0;
      drawPreview();
    }
  }

  return {
    refresh, build,
    focusSearch() { search.focus(); search.select(); },
    show() { build(); },
    problems: () => problemIds().length,
  };
}
