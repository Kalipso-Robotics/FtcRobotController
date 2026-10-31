import test from 'node:test';
import assert from 'node:assert/strict';
import { gapLabel, decideAt } from './web/details.js';

test('gap label only when odometry is known', () => {
  assert.equal(gapLabel({ x: 0, y: 0, theta: 0, ox: 38, oy: 0, otheta: 2.8 * Math.PI / 180 }, 30), 'gap 38 mm · 2.8°');
  assert.equal(gapLabel({ x: 0, y: 0, theta: 0, ox: NaN, oy: NaN, otheta: NaN }, 30), null);
});

test('a gap under the warn level has no label on the field, but the drawer (warn 0) still shows it', () => {
  const pose = { x: 0, y: 0, theta: 0, ox: 20, oy: 0, otheta: 0 };
  assert.equal(gapLabel(pose, 30), null);
  assert.equal(gapLabel(pose, 0), 'gap 20 mm · 0.0°');
});

test('decideAt reports the current move, since when, and the one before', () => {
  const decisions = [{ move: 'WAIT_TIP', from: 0, to: 9.9 }, { move: 'COLLECT_A', from: 9.9, to: 20 }];
  assert.deepEqual(decideAt(decisions, 12.4), { move: 'COLLECT_A', since: 9.9, was: 'WAIT_TIP' });
  assert.deepEqual(decideAt(decisions, 1), { move: 'WAIT_TIP', since: 0, was: null });
  assert.equal(decideAt([], 5), null);
});
