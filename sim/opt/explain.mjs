#!/usr/bin/env node
// Plain-text shot-by-shot narrative of one run, for the AI loop and for humans.
// usage: node sim/opt/explain.mjs <run.csv> [--op <OpModeName>]
import fs from 'node:fs';
import { pathToFileURL } from 'node:url';
import { OPMODES, parseRun } from '../app/runfile.mjs';
import { analyse, annotateShots } from '../app/web/stats.js';

/** Java-side waits in a shot's window, from "[Actor] BLOCKED - Waiting on: [A, B]" and "STILL BLOCKED for 0.6s" log lines.
 *  ponytail: ms is the longest "STILL BLOCKED for" seen, else first BLOCKED line to release; fine for a narrative, not a profiler. */
export function javaWaits(shot, logs) {
  const from = (shot.waitStart ?? shot.t - 0.3) - 0.05;
  const waits = new Map();
  for (const l of logs || []) {
    if (l.t < from || l.t > shot.t + 0.05) continue;
    let m = /^\[([^\]]*)\] (?:STILL )?BLOCKED(?: for ([\d.]+)s)? - Waiting on: \[(.*)\]/.exec(l.msg);
    let skip = false;
    if (!m) { m = /^\[([^\]]*)\] Skipping action \[([^\]]*)\] - blocked by: \[(.*)\]/.exec(l.msg); skip = !!m; }
    if (!m) continue;
    const key = skip ? `${m[2]} blocked by ${m[3]}` : `${m[1]} waiting on ${m[3]}`;
    const w = waits.get(key) || { first: l.t, ms: 0 };
    // Skipping lines are throttled (~250 ms), so span first..last seen
    w.ms = Math.max(w.ms, skip ? (l.t - w.first) * 1000 : m[2] ? Number(m[2]) * 1000 : (shot.t - w.first) * 1000);
    waits.set(key, w);
  }
  // a Skipping line seen once (0 ms) is just the ActionSet's static dependency order, not a wait
  return [...waits].filter(([what, w]) => !/ blocked by /.test(what) || w.ms >= 1).map(([what, w]) => `${what} ${Math.round(Math.max(0, w.ms))} ms`);
}

const mm = (v) => (Number.isFinite(v) ? Math.round(v) : '?');

/** One narrative line per shot (empty for old runs). */
export function shotLines(run) {
  return annotateShots(run).map((s) => {
    const phys = Object.entries(s.gateMs || {}).filter(([, ms]) => ms > 0).sort((x, y) => y[1] - x[1]).map(([g, ms]) => `${g} ${Math.round(ms)} ms`);
    // after arrival only: before it the wait is just the drive
    const java = javaWaits(s.arrivedAt != null ? { ...s, waitStart: s.arrivedAt } : s, run.logs);
    const parts = [`physics over whole feed window: ${phys.join(', ') || 'none'}`];
    if (java.length) parts.push(`java: ${java.join('; ')}`);
    const delay = s.delayMs != null ? `DELAY ${Math.round(s.delayMs)} ms after arriving` : 'burst follow-up';
    let line = `shot ${s.n} @${s.t.toFixed(1)}s: ${delay}; feed window ${Math.round(s.waitMs)} ms (${parts.join('; ')})`;
    line += `; aimed ${s.aimed ?? '?'}${s.aimedUp === false ? ' (DOWN)' : ''}`;
    line += s.target ? `, decide wanted ${s.target}${s.down || s.mismatch ? ' ⚠' : ''}` : '';
    const r = s.result;
    if (!r) line += '; result unknown';
    else if (r.in) line += `; IN${r.dist != null ? `, dist ${mm(r.dist)} mm` : ''}${r.speed != null ? `, robot ${mm(r.speed)} mm/s` : ''}`;
    else line += `; MISS long ${mm(r.long)} mm left ${mm(r.left)} mm, dist ${mm(r.dist)} mm, robot ${mm(r.speed)} mm/s`;
    return line;
  });
}

export function explainRun(run, { title = run.opMode || 'run' } = {}) {
  const a = analyse(run);
  const shots = a.shots;
  const L = [`# ${title}`, `score ${a.score.final} · shots in ${shots.filter((s) => s.result && s.result.in).length}/${shots.length}`, ''];
  L.push('## Decisions');
  if (a.decisions.length) for (const d of a.decisions) L.push(`${d.from.toFixed(1)} s  ${d.move}`);
  else L.push('(this auto does not report decide())');
  L.push('', '## Shots');
  if (!shots.length) L.push('(no shot records in this run file)');
  L.push(...shotLines(run));
  return L.join('\n') + '\n';
}

if (import.meta.url === pathToFileURL(process.argv[1] || '').href) {
  const [file, ...rest] = process.argv.slice(2);
  if (!file) { console.error('usage: node sim/opt/explain.mjs <run.csv> [--op OpModeName]'); process.exit(2); }
  const name = rest[0] === '--op' ? rest[1] : null;
  const op = (name && OPMODES.find((o) => o.name === name)) || OPMODES.find((o) => file.includes(o.name)) || OPMODES[0];
  process.stdout.write(explainRun(parseRun(fs.readFileSync(file, 'utf8'), op), { title: file }));
}
