// Turns one parsed run into a debrief.md and a path.png, and a batch of runs into summary.md. Pure: returns strings and a Buffer.
import { analyse, corners, rectGap, diffRuns, PARK_ZONE, THRESHOLDS } from '../app/web/stats.js';
import { Canvas } from './png.mjs';
import { aggregate } from './objective.mjs';
import { shotLines } from './explain.mjs';

export const spread = (xs) => ({ mean: xs.reduce((a, b) => a + b, 0) / xs.length, min: Math.min(...xs), max: Math.max(...xs) });

const f1 = (n) => n.toFixed(1);
const base = (move) => move.split(/[ :]/)[0];   // "COLLECT first: holding 2..." -> "COLLECT"
const KIND = (move) => (/^SHOOT/.test(base(move)) ? 'shoot' : /^(GET|COLLECT)/.test(base(move)) ? 'intake' : 'idle');
const BEFORE = (frames, t) => { let i = 0; while (i + 1 < frames.length && frames[i + 1][0] <= t) i++; return frames[i]; };

/** Per decision: seconds split drive/intake/shoot/idle and path efficiency. ponytail: "still" time is called intake/shoot by the decision's name, not by watching the mechanisms. */
function decisionRows(run, a) {
  const fr = run.frames;
  return a.decisions.map((d, i) => {
    const row = { n: i + 1, move: base(d.move), from: d.from, to: d.to, drive: 0, intake: 0, shoot: 0, idle: 0 };
    let len = 0;
    for (let k = 1; k < fr.length; k++) {
      if (fr[k][0] <= d.from || fr[k - 1][0] >= d.to) continue;
      const dt = fr[k][0] - fr[k - 1][0], step = Math.hypot(fr[k][1] - fr[k - 1][1], fr[k][2] - fr[k - 1][2]);
      len += step;
      const still = step / dt < THRESHOLDS.standingMmPerS && Math.abs(fr[k][3] - fr[k - 1][3]) / dt < THRESHOLDS.standingRadPerS;
      row[still ? KIND(d.move) : 'drive'] += dt;
    }
    const a0 = BEFORE(fr, d.from), a1 = BEFORE(fr, d.to);
    const chord = Math.hypot(a1[1] - a0[1], a1[2] - a0[2]);
    row.eff = chord >= 100 ? len / chord : null;
    return row;
  });
}

function wallGap(run) {
  let best = { gap: Infinity, t: 0 };
  for (const f of run.frames) for (const [x, y] of corners(f, run.field.robot)) {
    const g = Math.max(0, Math.min(x, y, run.field.size - x, run.field.size - y));
    if (g < best.gap) best = { gap: g, t: f[0] };
  }
  return best;
}

/** Floor balls (low, outside the Hive) at the end of the match, and how many sit within `near` mm of the R1 path. */
function looseBalls(run, near = 400) {
  const h = run.field.hive, fr = run.frames.filter((_, i) => i % 5 === 0);
  const ends = run.balls.tracks.filter((t) => t.length).map((t) => t[t.length - 1]);
  const floor = ends.filter(([, x, y, z]) => z < 100 && !(h && x >= h.minX && x <= h.maxX && y >= h.minY && y <= h.maxY));
  return { total: ends.length, near: floor.filter(([, x, y]) => fr.some((f) => Math.hypot(f[1] - x, f[2] - y) < near)).length };
}

/** Seconds left when the robot last entered the park zone for good, or null if it did not finish in it. */
function spareBeforePark(run, a) {
  const zone = [[PARK_ZONE.minX, PARK_ZONE.minY], [PARK_ZONE.maxX, PARK_ZONE.minY], [PARK_ZONE.maxX, PARK_ZONE.maxY], [PARK_ZONE.minX, PARK_ZONE.maxY]];
  const inZ = (f) => rectGap(corners(f, run.field.robot), zone) === 0;
  const fr = run.frames;
  if (!inZ(fr[fr.length - 1])) return null;
  let i = fr.length - 1;
  while (i > 0 && inZ(fr[i - 1])) i--;
  return a.duration - fr[i][0];
}

// Okabe-Ito colours picked to stay apart for colour-blind readers and to read on a pale field. Yellow is left out (invisible on pale).
const PALETTE = [[230, 159, 0], [0, 114, 178], [0, 158, 115], [213, 94, 0], [204, 121, 167], [86, 180, 233], [140, 90, 40], [75, 0, 146]];
const INK = [20, 20, 20];

/** The R1 path coloured by decision, R2 path, field, obstacles, park zone, markers and a legend. 0.25 px/mm, so a 3.66 m field is 914 px. */
function drawPath(run, a, rows, title) {
  const S = 0.25, M = 28, size = run.field.size, FW = Math.round(size * S), LEG = 330;
  const c = new Canvas(FW + 2 * M + LEG, FW + 2 * M, [255, 255, 255]);
  const X = (x) => M + x * S, Y = (y) => M + (size - y) * S;
  c.rect(M, M, FW, FW, [244, 242, 235]);
  for (let g = 500; g < size; g += 500) { c.rect(X(g), M, 1, FW, [222, 220, 212]); c.rect(M, Y(g), FW, 1, [222, 220, 212]); }
  for (let g = 500; g < size; g += 500) { c.text(X(g) - 5, M + FW + 6, String(g / 1000), [100, 100, 100], 2); c.text(4, Y(g) - 5, String(g / 1000), [100, 100, 100], 2); }
  c.rect(X(PARK_ZONE.minX), Y(PARK_ZONE.maxY), (PARK_ZONE.maxX - PARK_ZONE.minX) * S, (PARK_ZONE.maxY - PARK_ZONE.minY) * S, [190, 230, 190]);
  c.text(X(PARK_ZONE.minX) + 4, Y(PARK_ZONE.maxY) - 14, 'PARK', [30, 110, 30], 2);
  for (const o of run.field.obstacles) c.rect(X(o.minX), Y(o.maxY), (o.maxX - o.minX) * S, (o.maxY - o.minY) * S, [205, 205, 205]);
  for (const o of run.field.obstacles) c.outline(X(o.minX), Y(o.maxY), (o.maxX - o.minX) * S, (o.maxY - o.minY) * S, [130, 130, 130], 1);
  const h = run.field.hive;
  if (h) { c.outline(X(h.minX), Y(h.maxY), (h.maxX - h.minX) * S, (h.maxY - h.minY) * S, [160, 120, 60], 2); c.text(X(h.minX) + 6, Y(h.maxY) + 6, 'HIVE', [160, 120, 60], 2); }
  for (let y = 0; y < size; y += 60) c.rect(X(run.field.midline), Y(y) - 4, 2, 4, [150, 150, 150]);
  c.outline(M, M, FW, FW, INK, 2);

  const colour = (move) => PALETTE[[...new Set(rows.map((r) => r.move))].sort().indexOf(base(move)) % PALETTE.length];
  const at = (t) => BEFORE(run.frames, t);
  if (run.partner) {
    const pf = run.partner.frames;
    let run_ = 0;   // dashed: 14 px on, 10 px off, by distance travelled
    for (let i = 1; i < pf.length; i++) {
      const seg = Math.hypot(pf[i][1] - pf[i - 1][1], pf[i][2] - pf[i - 1][2]) * S;
      if (run_ % 24 < 14) c.line(X(pf[i - 1][1]), Y(pf[i - 1][2]), X(pf[i][1]), Y(pf[i][2]), 2, [110, 110, 110]);
      run_ += seg;
    }
    const e = pf[pf.length - 1];
    c.outline(X(e[1]) - 14, Y(e[2]) - 14, 28, 28, [120, 120, 120], 2); c.text(X(e[1]) - 6, Y(e[2]) - 5, 'R2', [80, 80, 80], 2);
  }
  const fr = run.frames;
  for (let i = 1; i < fr.length; i++) {
    const d = a.decisions.find((q) => fr[i][0] >= q.from && fr[i][0] < q.to) || a.decisions[a.decisions.length - 1];
    c.line(X(fr[i - 1][1]), Y(fr[i - 1][2]), X(fr[i][1]), Y(fr[i][2]), 4, d ? colour(d.move) : INK);
  }
  // robot footprint at start and end
  for (const f of [fr[0], fr[fr.length - 1]]) {
    const q = corners(f, run.field.robot);
    for (let i = 0; i < 4; i++) c.line(X(q[i][0]), Y(q[i][1]), X(q[(i + 1) % 4][0]), Y(q[(i + 1) % 4][1]), 2, INK);
  }
  c.text(X(fr[0][1]) + 8, Y(fr[0][2]) - 40, 'START', INK, 2);
  const mark = { pickup: (x, y) => { c.circle(x, y, 7, INK); c.circle(x, y, 5, [255, 255, 255]); },
    shot: (x, y) => c.rect(x - 6, y - 6, 12, 12, INK),
    contact: (x, y) => { c.line(x - 7, y - 7, x + 7, y + 7, 3, INK); c.line(x - 7, y + 7, x + 7, y - 7, 3, INK); } };
  const seen = new Set();   // one marker per place: a 4-ball volley is one shot marker
  const put = (kind, t) => { const f = at(t), x = X(f[1]), y = Y(f[2]), k = `${kind}${Math.round(x / 8)},${Math.round(y / 8)}`; if (!seen.has(k)) { seen.add(k); mark[kind](x, y); } };
  for (const e of run.events) if (e.t >= 0) { if (e.text.startsWith('picked up')) put('pickup', e.t); else if (e.text.startsWith('shot a')) put('shot', e.t); }
  for (const k of a.contacts) put('contact', k.t);
  // decision numbers where each decision starts, white disc so they read over any colour
  for (const r of rows) { const f = at(r.from); c.circle(X(f[1]), Y(f[2]) - 22, 9, [255, 255, 255]); c.text(X(f[1]) - (r.n > 9 ? 7 : 3), Y(f[2]) - 25, String(r.n), INK, 2); }

  // legend
  let y = M, x = FW + 2 * M;
  c.text(x, y, `${title} PATH`, INK, 3); y += 30;
  for (const r of rows) {
    c.rect(x, y, 22, 8, colour(r.move)); c.text(x + 30, y - 2, `${r.n} ${r.move} ${f1(r.from)}-${f1(r.to)}S`, INK, 2); y += 18;
  }
  y += 10;
  const key = [['pickup', 'PICKUP'], ['shot', 'SHOT'], ['contact', 'CONTACT']];
  for (const [k, label] of key) { mark[k](x + 11, y + 6); c.text(x + 30, y, label, INK, 2); y += 20; }
  c.line(x, y + 6, x + 10, y + 6, 2, [110, 110, 110]); c.line(x + 14, y + 6, x + 22, y + 6, 2, [110, 110, 110]); c.text(x + 30, y, 'R2 PATH (DASHED)', INK, 2); y += 20;
  c.text(x, y + 6, 'GRID 0.5 M. LABELS IN M', [100, 100, 100], 2); c.text(x, y + 26, 'DOTTED GREY LINE = MIDLINE', [100, 100, 100], 2);
  return c.encode();
}

/** { md, png } for one parsed run. opts: title, best (a parsed run to compare with). */
export function debrief(run, { title = run.opMode || 'run', best = null } = {}) {
  const a = analyse(run);
  const rows = decisionRows(run, a);
  const L = [`# Debrief: ${title}`, ''];
  const s = a.score;
  L.push(`Auto score: ${s.final}${s.items.length ? ` (${s.items.map((i) => `${i.label} +${i.points}${i.assumed ? ' (assumed)' : ''}`).join(', ')})` : ''}`, '');
  L.push('## Time per decision', '| # | decision | s | drive | intake | shoot | idle | path eff |', '|---|---|---|---|---|---|---|---|');
  for (const r of rows) L.push(`| ${r.n} | ${r.move} | ${f1(r.to - r.from)} | ${f1(r.drive)} | ${f1(r.intake)} | ${f1(r.shoot)} | ${f1(r.idle)} | ${r.eff === null ? '-' : r.eff.toFixed(2)} |`);
  L.push('', '## Spacing');
  L.push(`- Closest to R2: ${run.partner ? `${a.numbers.partnerClosest.toFixed(0)} mm at ${f1(a.numbers.partnerClosestAt)} s` : 'no partner in this run'}`);
  const w = wallGap(run);
  L.push(`- Closest to a wall: ${w.gap.toFixed(0)} mm at ${f1(w.t)} s`);
  const lb = looseBalls(run);
  L.push('', `## Loose balls at ${f1(a.duration)} s`, `- ${lb.near} floor ball(s) outside the Hive within 400 mm of the path (${lb.total} balls in the match)`);
  const spare = spareBeforePark(run, a);
  L.push('', '## Spare time', spare === null ? `- Not parked: no spare time to report. The robot was still for the last ${f1(a.numbers.idleTail)} s.` : `- ${f1(spare)} s between arriving in the park zone and the end`);
  if (a.shots.length) L.push('', '## Shots', ...shotLines(run).map((l) => `- ${l}`));
  L.push('', '## Changes vs best');
  const diff = best ? diffRuns(analyse(best), a) : null;
  if (!best) L.push('- no best run given');
  else if (!diff.length) L.push('- no change');
  else for (const d of diff) L.push(d.kind === 'changed' ? `- ${d.label}: ${d.from} -> ${d.to} (${d.better ? 'better' : 'worse'})` : `- ${d.label}`);
  return { md: L.join('\n') + '\n', png: drawPath(run, a, rows, title) };
}

/** summary.md for a batch. rows: [{ name, result: objective() | { error }, run? }]. */
export function summarize(rows, title = 'Batch') {
  const g = aggregate(rows.map((r) => r.result));
  const L = [`# ${title}`, '', `Runs: ${g.n}, errors: ${g.errors}, mean ${g.mean === null ? '-' : g.mean.toFixed(2)}, worst ${g.worst === null ? '-' : g.worst.toFixed(2)}`, '',
    '| run | value | points | contact pen | near-miss pen | note |', '|---|---|---|---|---|---|'];
  for (const r of rows) {
    const o = r.result;
    L.push(o.error ? `| ${r.name} | - | - | - | - | ERROR ${String(o.error).split('\n')[0]} |`
      : `| ${r.name} | ${o.value.toFixed(2)} | ${o.points} | ${o.penalties.contact} | ${o.penalties.nearMiss.toFixed(2)} | |`);
  }
  return L.join('\n') + '\n';
}
