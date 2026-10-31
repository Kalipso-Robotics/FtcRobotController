import test from 'node:test';
import assert from 'node:assert/strict';
import { clusterMarkers, eventKind } from './web/timeline.js';

test('events closer than 6 px merge into one pill with a count', () => {
  const ev = [{ t: 12.38, text: 'picked up a pollen ball (1 held)' }, { t: 12.40, text: 'robot touched wall-yMin' }, { t: 12.41, text: 'shot a pollen ball' }, { t: 20, text: 'red-hive tipped' }];
  const c = clusterMarkers(ev, 30, 1200);
  assert.equal(c.length, 2); assert.equal(c[0].events.length, 3); assert.ok(c[0].kinds.has('contact'));
  assert.equal(c[0].label, '3 events');
});

test('shot clusters count misses', () => {
  const ev = [0, 0.01, 0.02].flatMap((t, i) => [{ t, text: 'shot a pollen ball' }, ...(i < 2 ? [{ t: t + 0.005, text: "a pollen ball went into the red Hive's up Cell" }] : [])]);
  assert.equal(clusterMarkers(ev, 30, 1200)[0].label, '3 · 1 miss');
});

test('shots that all land read "3 shots"; events apart stay apart', () => {
  const ev = [0, 0.01, 0.02].flatMap((t) => [{ t, text: 'shot a pollen ball' }, { t: t + 0.005, text: "a pollen ball went into the red Hive's up Cell" }]);
  assert.equal(clusterMarkers(ev, 30, 1200)[0].label, '3 shots');
  assert.equal(clusterMarkers([{ t: 1, text: 'shot a pollen ball' }, { t: 2, text: 'shot a pollen ball' }], 30, 1200).length, 2);
});

test('events before START are left off the lane', () => {
  assert.equal(clusterMarkers([{ t: -2.9, text: 'robot touched wall-yMax' }], 30, 1200).length, 0);
});

test('eventKind sorts the recorded text', () => {
  assert.deepEqual(['shot a pollen ball (45 RPS)', 'picked up a pollen ball', 'red-hive tipped', 'robot touched x', "a pollen ball went into the red Hive's up Cell", 'something else'].map(eventKind),
    ['shot', 'pickup', 'tip', 'contact', 'in', 'other']);
});
