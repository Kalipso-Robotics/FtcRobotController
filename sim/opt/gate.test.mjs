import test from 'node:test';
import assert from 'node:assert/strict';
import { decide } from './gate.mjs';

const best = { mean: 50, worst: 30 };
test('first run (no best) is accepted', () => assert.equal(decide({ mean: 1, worst: 0, errors: 0 }, null, 0.5, 2).ok, true));
test('needs mean > best + eps', () => {
  assert.equal(decide({ mean: 50.5, worst: 30, errors: 0 }, best, 0.5, 2).ok, false);
  assert.equal(decide({ mean: 50.6, worst: 30, errors: 0 }, best, 0.5, 2).ok, true);
});
test('worst may drop by tol but no more', () => {
  assert.equal(decide({ mean: 60, worst: 28, errors: 0 }, best, 0.5, 2).ok, true);
  assert.equal(decide({ mean: 60, worst: 27.9, errors: 0 }, best, 0.5, 2).ok, false);
});
test('any errored run rejects', () => assert.equal(decide({ mean: 99, worst: 99, errors: 1 }, best, 0.5, 2).ok, false));
