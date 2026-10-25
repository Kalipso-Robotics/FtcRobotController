import test from 'node:test';
import assert from 'node:assert/strict';
import { analyse, THRESHOLDS } from './web/stats.js';

/** A made-up run with known numbers: 1 s driving at 1000 mm/s along x, then standing still until 5 s. */
function run({ odoOffset = 0, events = [], warnings = [], error = null, finished = true, startX = 100, heading = 0, telemetry = [] } = {}) {
  const frames = [];
  for (let k = 0; k <= 250; k++) {
    const t = k * 0.02;
    const x = startX + 1000 * Math.min(t, 1);
    frames.push([t, x, 500, heading, x + odoOffset, 500, heading]);
  }
  return {
    frames, events, warnings, error, finished, used: 5, telemetry,
    field: { robot: { front: 300, back: 229, width: 457 }, midline: 1828.8 },
    meta: { simMs: 500 },
  };
}

test('distance, speed and standing still come out of the frames', () => {
  const a = analyse(run());
  assert.ok(Math.abs(a.numbers.distance - 1000) < 1, `distance ${a.numbers.distance}`);
  assert.ok(Math.abs(a.numbers.peakSpeed - 1000) < 5, `peak ${a.numbers.peakSpeed}`);
  assert.ok(Math.abs(a.numbers.movingTime - 1) < 0.15, `moving ${a.numbers.movingTime}`);
  assert.ok(Math.abs(a.numbers.idleTail - 4) < 0.15, `idle tail ${a.numbers.idleTail}`);
  assert.ok(Math.abs(a.numbers.speedup - 10) < 1e-9);
});

test('a run with nothing wrong is clean, except for the long stand-still at the end', () => {
  const a = analyse(run());
  assert.equal(a.checks.find((c) => c.id === 'idle').status, 'warn');
  assert.deepEqual(a.checks.filter((c) => c.id !== 'idle').map((c) => c.status), ['pass', 'pass', 'pass', 'pass', 'pass', 'pass']);
  assert.equal(a.verdict, 'attention');
});

test('odometry error is judged against the thresholds', () => {
  assert.equal(analyse(run({ odoOffset: THRESHOLDS.odoMmWarn - 5 })).checks.find((c) => c.id === 'odometry').status, 'pass');
  assert.equal(analyse(run({ odoOffset: THRESHOLDS.odoMmWarn + 5 })).checks.find((c) => c.id === 'odometry').status, 'warn');
  assert.equal(analyse(run({ odoOffset: THRESHOLDS.odoMmFail + 5 })).checks.find((c) => c.id === 'odometry').status, 'fail');
  assert.ok(Math.abs(analyse(run({ odoOffset: 50 })).numbers.odoMax - 50) < 1e-6);
});

test('contacts before START are setup problems, contacts after START are counted', () => {
  const a = analyse(run({ events: [{ t: -2.9, text: 'robot touched wall-yMax' }, { t: 0.4, text: 'robot touched hive-leg-low' }, { t: 0.9, text: 'robot touched hive-leg-low' }] }));
  assert.equal(a.numbers.contacts, 2);
  assert.equal(a.numbers.setupContacts, 1);
  assert.deepEqual(a.contactsByName, { 'hive-leg-low': 2 });
  assert.equal(a.checks.find((c) => c.id === 'contacts').seek, 0.4);
});

test('the body, not the centre, decides whether the robot crossed the midline', () => {
  // the centre ends at x = 1600, but the front (300 mm) reaches 1900, past the midline at 1828.8
  const a = analyse(run({ startX: 600 }));
  assert.equal(a.checks.find((c) => c.id === 'side').status, 'fail');
  assert.ok(Math.abs(a.numbers.maxX - 1900) < 1);
});

test('an error or an unfinished run fails, and the verdict follows', () => {
  assert.equal(analyse(run({ error: 'java.lang.NullPointerException\n at x' })).verdict, 'failed');
  assert.equal(analyse(run({ finished: false })).verdict, 'failed');
  assert.equal(analyse(run({ error: 'boom' })).verdictText, 'Run failed');
  assert.equal(analyse(run({ startX: 600 })).verdictText, 'Problems found'); // a failed check, but the auto itself ran
});

test('decide() choices become segments and totals', () => {
  const telemetry = [
    { t: 0, items: { 'decide says': 'GET_GARDEN' } },
    { t: 1, items: { 'decide says': 'GET_GARDEN', other: '1' } },
    { t: 2, items: { 'decide says': 'SHOOT_A' } },
    { t: 4, items: { 'decide says': 'PARK' } },
  ];
  const a = analyse(run({ telemetry }));
  assert.deepEqual(a.decisions.map((d) => [d.move, d.from, d.to]), [['GET_GARDEN', 0, 2], ['SHOOT_A', 2, 4], ['PARK', 4, 5]]);
  assert.equal(a.decisionTime.GET_GARDEN, 2);
});

test('ball runs: pickups, shots, shots that went in, tips and the held chart come from the events', () => {
  const base = run({ events: [
    { t: 1, text: 'picked up a pollen ball (1 held)' }, { t: 2, text: 'shot a pollen ball (45.6 RPS, 63°, 5.3 m/s)' },
    { t: 2.1, text: 'a pollen ball went into the red Hive\'s up Cell' }, { t: 2.2, text: 'shot a pollen ball (45.6 RPS, 63°, 5.3 m/s)' },
    { t: 3, text: 'red-hive tipped' }, { t: 3.5, text: 'robot touched wall-x0' },
  ] });
  base.balls = { info: [{ id: 0, type: 0, radius: 35 }], held: [[0, 4], [2, 3], [4, 0]] };
  const a = analyse(base);
  assert.equal(a.numbers.pickups, 1);
  assert.equal(a.numbers.shots, 2);
  assert.equal(a.numbers.shotsIn, 1);
  assert.equal(a.numbers.hiveTips, 1);
  assert.equal(a.numbers.heldAtEnd, 0);
  assert.equal(a.numbers.contacts, 1, 'a Hive tip or a pickup is not a contact');
  assert.equal(a.checks.find((c) => c.id === 'shots').status, 'warn');
  assert.equal(a.checks.find((c) => c.id === 'tips').status, 'pass');
  assert.ok(a.hasBalls);
});

test('a run with balls but nothing happening to them has no ball checks', () => {
  const base = run();
  base.balls = { info: [{ id: 0, type: 0, radius: 35 }], held: [] };
  const a = analyse(base);
  assert.equal(a.hasBalls, false);
  assert.equal(a.checks.find((c) => c.id === 'shots'), undefined);
});
