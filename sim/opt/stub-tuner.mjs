#!/usr/bin/env node
// Stand-in for the LLM tuner/critic so loop.sh can be exercised without claude.
// usage: node stub-tuner.mjs <iteration> <params.json>   nudge one number by +-5%
//        node stub-tuner.mjs --critic                     always accept
import fs from 'node:fs';
import { pathToFileURL } from 'node:url';

const leaves = (o, pre = []) => Object.entries(o).flatMap(([k, v]) =>
  typeof v === 'number' ? [[...pre, k]] : v && typeof v === 'object' ? leaves(v, [...pre, k]) : []);

/** Copy of params with the (iteration mod count)th numeric value scaled by 1.05 (odd iterations) or 0.95 (even). */
export function nudge(params, iteration) {
  const out = structuredClone(params), all = leaves(out);
  if (!all.length) return out;
  const p = all[iteration % all.length], last = p.pop();
  const parent = p.reduce((o, k) => o[k], out);
  parent[last] *= iteration % 2 ? 1.05 : 0.95;
  return out;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  if (process.argv[2] === '--critic') console.log('Stub critic: no objection.\nVERDICT: ACCEPT');
  else {
    const [it, file] = [Number(process.argv[2]), process.argv[3]];
    fs.writeFileSync(file, JSON.stringify(nudge(JSON.parse(fs.readFileSync(file, 'utf8')), it), null, 2) + '\n');
  }
}
