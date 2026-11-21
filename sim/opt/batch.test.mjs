import test from 'node:test';
import assert from 'node:assert/strict';
import { jitter, expandRuns, buildScores, parseArgs, opParams } from './batch.mjs';

test('jitter is a pure function of the seed and stays inside +-15 mm / +-1 deg', () => {
  assert.deepEqual(jitter(7), jitter(7));
  assert.notDeepEqual(jitter(7), jitter(8));
  for (let s = 1; s <= 200; s++) {
    const j = jitter(s);
    assert.ok(Math.abs(j.dx) <= 15 && Math.abs(j.dy) <= 15 && Math.abs(j.dTheta) <= Math.PI / 180 + 1e-12, JSON.stringify(j));
  }
});

test('expandRuns: seed x scenario x speed, directory names, jittered start', () => {
  const op = { start: { x: 1000, y: 2000, headingDeg: -90 }, scenarios: [{ partner: null, speeds: [1] }, { partner: 'p_far', speeds: [0.5, 1] }] };
  const runs = expandRuns(op, 2);
  assert.deepEqual(runs.map((r) => r.name), ['solo-s1', 'solo-s2', 'p_far@0.5-s1', 'p_far@0.5-s2', 'p_far@1-s1', 'p_far@1-s2']);
  assert.equal(runs[2].partner, 'p_far'); assert.equal(runs[2].speed, 0.5);
  assert.equal(runs[0].x, 1000 + jitter(1).dx);
  assert.equal(runs[1].theta, -Math.PI / 2 + jitter(2).dTheta);
  assert.deepEqual(expandRuns({ start: op.start }, 1).map((r) => r.name), ['solo-s1']);   // no scenarios: one solo run
});

test('buildScores: per-run objective or error, aggregate wired through, key order fixed', () => {
  const ok = (v) => ({ value: v, points: v, penalties: { contact: 0, nearMiss: 0, jerk: 0 } });
  const s = buildScores('X', [{ name: 'b-s1', result: ok(4) }, { name: 'a-s1', result: ok(2) }, { name: 'a-s2', result: { error: 'timeout' } }]);
  assert.deepEqual(Object.keys(s.runs), ['a-s1', 'a-s2', 'b-s1']);
  assert.deepEqual(s.aggregate, { mean: 3, worst: 2, n: 3, errors: 1 });
  assert.equal(s.runs['a-s2'].error, 'timeout');
});

test('parseArgs: defaults and flags', () => {
  const a = parseArgs(['--auto', 'RedDecideAuto', '--seeds', '3', '--out', 'o']);
  assert.equal(a.auto, 'RedDecideAuto'); assert.equal(a.seeds, 3); assert.equal(a.timeout, 120); assert.ok(a.jobs >= 1);
  assert.throws(() => parseArgs(['--seeds', '2']), /--auto/);
});

test('opParams: an object as it is, a string as the autoparams file it names, nothing as {}', () => {
  assert.deepEqual(opParams({ params: { SOTM: true } }), { SOTM: true });
  assert.deepEqual(opParams({}), {});
  assert.deepEqual(opParams({ params: 'TeamCode/src/main/resources/autoparams/RedDuoFullAuto.json' }), { PARTNER_MODE: 'NOT_SOLO', SOTM: false });
});
