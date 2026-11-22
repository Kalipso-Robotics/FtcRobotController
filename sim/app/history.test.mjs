import test from 'node:test';
import assert from 'node:assert/strict';
import { createHistory } from './web/history.js';

test('undo and redo walk the snapshots; a new edit clears redo', () => {
  const h = createHistory();
  assert.equal(h.undo('c'), null);
  h.push('a'); h.push('b');
  assert.equal(h.undo('c'), 'b');
  assert.equal(h.undo('b'), 'a');
  assert.equal(h.undo('a'), null);
  assert.equal(h.redo('a'), 'b');
  assert.ok(h.canRedo());
  h.push('b2');
  assert.ok(!h.canRedo());
});

test('the stack is capped at 50, dropping the oldest', () => {
  const h = createHistory(50);
  for (let i = 0; i < 80; i++) h.push(String(i));
  let n = 0, cur = 'now', last;
  while ((last = h.undo(cur)) !== null) { cur = last; n++; }
  assert.equal(n, 50);
  assert.equal(cur, '30');
});
