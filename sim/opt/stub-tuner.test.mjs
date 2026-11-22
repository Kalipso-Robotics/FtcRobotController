import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { nudge } from './stub-tuner.mjs';

const P = { a: 100, name: 'x', nested: { b: 10, flag: true }, list: [1, 2] };
const diffs = (p, q, pre = '') => Object.keys(p).flatMap((k) =>
  typeof p[k] === 'object' && p[k] !== null ? diffs(p[k], q[k], `${pre}${k}.`) : p[k] === q[k] ? [] : [`${pre}${k}`]);

test('nudge changes exactly one numeric value by 5 percent', () => {
  for (let i = 1; i <= 8; i++) {
    const out = nudge(P, i);
    assert.equal(diffs(P, out).length, 1, `iteration ${i}`);
  }
  assert.ok(Math.abs(nudge({ a: 100 }, 1).a - 105) < 1e-9);
  assert.ok(Math.abs(nudge({ a: 100 }, 2).a - 95) < 1e-9);
});
test('nudge is deterministic and does not mutate its input', () => {
  assert.deepEqual(nudge(P, 3), nudge(P, 3));
  assert.equal(P.a, 100);
});
test('no numeric values: unchanged', () => assert.deepEqual(nudge({ s: 'x' }, 1), { s: 'x' }));
test('CLI edits the given file in place, and --critic accepts', () => {
  const f = path.join(fs.mkdtempSync(path.join(os.tmpdir(), 'st-')), 'p.json');
  fs.writeFileSync(f, JSON.stringify(P));
  const here = path.dirname(new URL(import.meta.url).pathname);
  execFileSync('node', [path.join(here, 'stub-tuner.mjs'), '2', f]);
  assert.equal(diffs(P, JSON.parse(fs.readFileSync(f, 'utf8'))).length, 1);
  assert.match(execFileSync('node', [path.join(here, 'stub-tuner.mjs'), '--critic']).toString(), /VERDICT: ACCEPT\s*$/);
});
