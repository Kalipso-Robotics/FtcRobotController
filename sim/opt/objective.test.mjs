import test from 'node:test';
import assert from 'node:assert/strict';
import { objective, aggregate, WEIGHTS, THRESHOLD_MM } from './objective.mjs';

/** 5 s run, robot driving +x along y=1500 (outside the loading zone). zig adds a 100 mm sideways wobble every 0.2 s. */
function run({ events = [], partnerAt = null, zig = false } = {}) {
  const frames = [];
  for (let k = 0; k <= 250; k++) {
    const t = k * 0.02, x = 100 + 400 * t, y = 1500 + (zig && Math.floor(t / 0.2) % 2 ? 100 : 0);
    frames.push([t, x, y, 0, x, y, 0]);
  }
  const robot = { front: 200, back: 200, width: 400 };
  const r = { alliance: 'red', frames, events, warnings: [], telemetry: [], error: null, finished: true, used: 5, field: { robot, midline: 1828.8 } };
  if (partnerAt) r.partner = { name: 'p', robot, frames: frames.map((f) => [f[0], partnerAt[0], partnerAt[1], 0]) };
  return r;
}

test('a clean run is worth its points: leave only', () => {
  const o = objective(run());
  assert.equal(o.points, 3); assert.equal(o.penalties.contact, 0); assert.equal(o.penalties.nearMiss, 0);
  assert.ok(o.penalties.jerk < 1e-6);
  assert.ok(Math.abs(o.value - 3) < 1e-6);
});

test('each partner contact costs WEIGHTS.contact, other contacts are free', () => {
  const ev = [{ t: 1, text: 'robot touched partner' }, { t: 2, text: 'robot touched partner' }, { t: 3, text: 'robot touched wall' }];
  assert.equal(objective(run({ events: ev })).penalties.contact, 2 * WEIGHTS.contact);
});

test('a near miss is penalised in proportion to how far inside the threshold it got; far partners are free', () => {
  const near = objective(run({ partnerAt: [300, 1500 + 400 + THRESHOLD_MM / 2] }));   // body edges THRESHOLD_MM/2 apart in y
  assert.ok(Math.abs(near.penalties.nearMiss - WEIGHTS.nearMiss / 2) < 1e-6, String(near.penalties.nearMiss));
  assert.equal(objective(run({ partnerAt: [300, 3000] })).penalties.nearMiss, 0);
});

test('a jerky path costs more than a straight one', () => {
  assert.ok(objective(run({ zig: true })).penalties.jerk > objective(run()).penalties.jerk + 1);
});

test('aggregate: mean and worst over finished runs, errors counted not averaged', () => {
  assert.deepEqual(aggregate([{ value: 10 }, { value: 4 }, { error: 'timeout' }]), { mean: 7, worst: 4, n: 3, errors: 1 });
  assert.deepEqual(aggregate([{ error: 'x' }]), { mean: null, worst: null, n: 1, errors: 1 });
});
