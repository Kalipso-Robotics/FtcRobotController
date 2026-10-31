import test from 'node:test';
import assert from 'node:assert/strict';
import { analyse, rectGap, THRESHOLDS, eventText, fmtLen, SCORING, score, diffRuns, estimatedChecks, placeName, problems, nextProblem } from './web/stats.js';

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

test('diff: a new contact is a new problem, fewer shots in is worse', () => {
  const prev = analyse(run()), cur = analyse(run({ events: [{ t: 12.4, text: 'robot touched wall-yMin' }] }));
  const d = diffRuns(prev, cur);
  assert.equal(d[0].kind, 'new'); assert.equal(d[0].id, 'contacts'); assert.equal(d[0].seek, 12.4);
  assert.equal(d[0].label, 'New: touched the top wall');
  assert.deepEqual(diffRuns(null, cur), []);
});

test('diff: a problem that went away is fixed, and a number that moved says better or worse', () => {
  const prev = analyse(run({ odoOffset: 50 })), cur = analyse(run({ odoOffset: 10 }));
  const d = diffRuns(prev, cur);
  assert.equal(d[0].kind, 'fixed'); assert.equal(d[0].label, 'Fixed: Odometry tracks the robot');
  const odo = d.find((x) => x.id === 'odoMax');
  assert.equal(odo.kind, 'changed'); assert.equal(odo.better, true); assert.equal(odo.from, 50); assert.equal(odo.to, 10);
});

test('score: leave + one red tip, tip is an estimate', () => {
  const r = run({ events: [{ t: 9.8, text: 'red-hive tipped' }] }); r.alliance = 'red';
  const s = score(r, analyse(r));
  assert.equal(s.final, SCORING.leave + SCORING.tip); assert.equal(s.est, true);
  assert.equal(s.at(5), SCORING.leave); assert.equal(s.at(10), SCORING.leave + SCORING.tip);
  assert.equal(s.items.find((i) => i.kind === 'leave').est, false);
});

test('score: a blue tip does not count for red, and a robot that never left scores nothing', () => {
  const r = run({ events: [{ t: 9.8, text: 'blue-hive tipped' }] }); r.alliance = 'red';
  assert.equal(score(r, analyse(r)).final, SCORING.leave);
  const still = run(); still.alliance = 'red'; still.frames.forEach((f) => { f[1] = 100; f[4] = 100; });
  assert.equal(score(still, analyse(still)).final, 0);
});

test('score: parking counts only when decide() ended on PARK and the robot is standing still', () => {
  const r = run({ telemetry: [{ t: 0, items: { 'decide says': 'GET_GARDEN' } }, { t: 2, items: { 'decide says': 'PARK' } }] }); r.alliance = 'red';
  const park = score(r, analyse(r)).items.find((i) => i.kind === 'park');
  assert.equal(park.points, SCORING.park); assert.equal(park.est, true);
  const moving = run({ telemetry: [{ t: 0, items: { 'decide says': 'PARK' } }] }); moving.alliance = 'red';
  moving.frames.forEach((f, i) => { f[1] = 100 + i * 20; });
  assert.equal(score(moving, analyse(moving)).items.some((i) => i.kind === 'park'), false);
});

test('nextProblem wraps from the last problem to the first', () => {
  const a = { checks: [{ id: 'x', status: 'warn', seek: 4 }, { id: 'y', status: 'pass', seek: 3 }, { id: 'z', status: 'warn', seek: 2 }] };
  assert.deepEqual(problems(a).map((c) => c.id), ['z', 'x']);
  assert.equal(nextProblem(a, 4.5, 1).seek, 2);
  assert.equal(nextProblem(a, 2.5, 1).seek, 4);
  assert.equal(nextProblem(a, 1, -1).seek, 4);
  assert.equal(nextProblem({ checks: [] }, 0, 1), null);
});

test('estimatedChecks follows affects of unmeasured params only', () => {
  assert.deepEqual([...estimatedChecks([{ id: 'a', unmeasured: true, affects: ['tips', 'score'] }, { id: 'b', affects: ['shots'] }])].sort(), ['score', 'tips']);
});

test('placeName turns wall and Flower ids into words', () => {
  assert.equal(placeName('wall-yMin'), 'the top wall');
  assert.equal(placeName('flower-C'), 'Flower C');
  assert.equal(placeName('hive-leg-low'), "the Hive's low leg");
  assert.equal(placeName('partner'), 'R2');
  assert.equal(placeName('mystery'), 'mystery');
});

test('eventText says what happened in plain words', () => {
  assert.equal(eventText('robot touched wall-yMin'), 'Touched the top wall');
  assert.equal(eventText('picked up a pollen ball (1 held)'), 'Picked up pollen (1 held)');
  assert.equal(eventText('red-hive tipped'), 'Red Hive tipped');
  assert.equal(eventText('something else'), 'something else');
});

test('lengths read in feet and inches', () => {
  assert.equal(fmtLen(10363), '34 ft');
  assert.equal(fmtLen(228.6), '9 in');
  assert.equal(fmtLen(1463), '4.8 ft');
});

/** R2 parked at (x, y) for the whole run, 400 mm wide and 400 mm long. */
const withPartner = (r, x, y) => ({ ...r, partner: { name: 'p', robot: { front: 200, back: 200, width: 400 }, frames: r.frames.map((f) => [f[0], x, y, 0]) } });

test('rectGap is 0 when overlapping and the edge distance when apart', () => {
  const sq = (x) => [[x, 0], [x + 100, 0], [x + 100, 100], [x, 100]];
  assert.equal(rectGap(sq(0), sq(50)), 0);
  assert.equal(rectGap(sq(0), sq(250)), 150);
});

test('touching the partner fails Avoids partner', () => {
  const a = analyse(withPartner(run({ events: [{ t: 18.1, text: 'robot touched partner' }] }), 3000, 3000));
  const c = a.checks.find((x) => x.id === 'partner');
  assert.equal(c.status, 'fail');
  assert.equal(c.seek, 18.1);
});

test('a close pass warns and reports the closest gap', () => {
  const a = analyse(withPartner(run(), 1100, 1028.5));
  const c = a.checks.find((x) => x.id === 'partner');
  assert.equal(c.status, 'warn');
  assert.ok(Math.abs(a.numbers.partnerClosest - 100) < 1, `closest ${a.numbers.partnerClosest}`);
});

test('a far partner passes', () => {
  assert.equal(analyse(withPartner(run(), 3000, 3000)).checks.find((x) => x.id === 'partner').status, 'pass');
});

test('no partner, no check', () => {
  assert.equal(analyse(run()).checks.some((c) => c.id === 'partner'), false);
});
