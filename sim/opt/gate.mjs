#!/usr/bin/env node
// Mechanical accept gate. usage: node gate.mjs <scores.json> <leaderboard.json|-> [eps] [tol]   exit 0 = pass
import fs from 'node:fs';
import { pathToFileURL } from 'node:url';

/** agg = scores.json aggregate; best = {mean, worst} or null (first run always passes unless a run errored). */
export function decide(agg, best, eps, tol) {
  if (agg.errors) return { ok: false, why: `${agg.errors} run(s) errored` };
  if (!best) return { ok: true, why: 'baseline' };
  if (!(agg.mean > best.mean + eps)) return { ok: false, why: `mean ${agg.mean} not > ${best.mean} + ${eps}` };
  if (!(agg.worst >= best.worst - tol)) return { ok: false, why: `worst ${agg.worst} < ${best.worst} - ${tol}` };
  return { ok: true, why: `mean ${agg.mean} > ${best.mean} + ${eps}, worst ${agg.worst} >= ${best.worst} - ${tol}` };
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const [s, l, eps = '0.5', tol = '2'] = process.argv.slice(2);
  const agg = JSON.parse(fs.readFileSync(s, 'utf8')).aggregate;
  const best = l !== '-' && fs.existsSync(l) ? JSON.parse(fs.readFileSync(l, 'utf8')).best : null;
  const r = decide(agg, best, Number(eps), Number(tol));
  console.log(`${r.ok ? 'PASS' : 'FAIL'}: ${r.why}`);
  process.exit(r.ok ? 0 : 1);
}
