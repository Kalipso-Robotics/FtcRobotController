import test from 'node:test';
import assert from 'node:assert/strict';
import { chipFor } from './web/runState.js';

const run48 = { meta: { runId: 48, finishedAt: '2026-10-25T21:41:07.000Z' }, frames: [[0], [14.2]], error: null };
const fresh = { phase: 'done', runId: 48, watching: true, since: 1000, lastSaveAt: 0, error: null };

test('a finished, current run is fresh and not old', () => {
  const c = chipFor(fresh, run48, 5000, 'warn');
  assert.equal(c.status, 'warn'); assert.equal(c.title, 'Run #48'); assert.match(c.sub, /^fresh · \d\d:\d\d:\d\d$/); assert.equal(c.old, false);
});

test('a save while a run shows marks it old at once', () => {
  const c = chipFor({ phase: 'building', since: 1000, runId: 48, watching: true }, { meta: { runId: 48 } }, 7000);
  assert.equal(c.title, 'Compiling'); assert.equal(c.sub, '6 s'); assert.equal(c.old, true); assert.equal(c.status, 'busy');
});

test('simulating is busy too, and not old when there is nothing to be old', () => {
  const c = chipFor({ phase: 'running', since: 1000, runId: 0, watching: true }, null, 4000);
  assert.equal(c.title, 'Simulating'); assert.equal(c.sub, '3 s'); assert.equal(c.old, false);
});

test('run on save off and a newer save says out of date', () => {
  const c = chipFor({ ...fresh, watching: false, lastSaveAt: Date.parse('2026-10-25T21:50:00.000Z') }, run48, 0);
  assert.equal(c.title, 'Run #48 is out of date'); assert.equal(c.sub, 'press R'); assert.equal(c.old, true); assert.equal(c.status, 'none');
});

test('run on save off with nothing newer adds the extra chip', () => {
  const c = chipFor({ ...fresh, watching: false }, run48, 0, 'pass');
  assert.equal(c.title, 'Run #48'); assert.deepEqual([c.extra.title, c.extra.sub], ['Run on save off', 'press R']);
});

test('a compile error fails, keeps the old run and offers a retry', () => {
  const c = chipFor({ phase: 'compile-error', runId: 48, since: Date.parse('2026-10-25T21:44:19Z'), watching: true }, run48, 0);
  assert.equal(c.status, 'fail'); assert.equal(c.title, 'Build failed'); assert.match(c.sub, /showing old run #48$/); assert.equal(c.action, 'retry'); assert.equal(c.old, true);
});

test('a crash during the run names the last time it got to, and the partial run is not old', () => {
  const c = chipFor({ phase: 'run-error', runId: 49, watching: true }, { ...run48, meta: { runId: 49 }, error: 'boom' }, 0);
  assert.equal(c.status, 'fail'); assert.equal(c.title, 'Run failed'); assert.equal(c.sub, 'crashed at 14.2 s'); assert.equal(c.old, false);
});

test('the runner going away is a retry, not a crash', () => {
  const c = chipFor({ phase: 'offline', runId: 48 }, run48, 0);
  assert.equal(c.status, 'fail'); assert.equal(c.title, 'Runner disconnected'); assert.equal(c.action, 'retry');
});

test('before the first run there is nothing to show', () => {
  const c = chipFor({ phase: 'idle', runId: 0, watching: true }, null, 0);
  assert.equal(c.status, 'none'); assert.equal(c.title, 'No run yet');
});
