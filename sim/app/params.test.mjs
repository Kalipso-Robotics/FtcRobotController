import test from 'node:test';
import assert from 'node:assert/strict';
import { filterParams, paramProblem } from './web/params.js';

const defs = [
  { id: 'robotWidth', group: 'Robot size', label: 'Width', unit: 'mm', min: 150, max: 600, description: 'Side to side.', source: 'RobotFootprint.java' },
  { id: 'shotSpeedPerRps', group: 'Shooter', label: 'Ball speed per flywheel RPS', unit: 'm/s', min: 0.05, max: 0.25, description: 'How fast a ball leaves. Unmeasured.', unmeasured: true },
];
const defaults = { robotWidth: 457, shotSpeedPerRps: 0.1 };

test('changed filter shows only values that differ from the default', () => {
  const values = { robotWidth: 400, shotSpeedPerRps: 0.1 };
  assert.deepEqual(filterParams(defs, values, defaults, { filter: 'changed' }).map((d) => d.id), ['robotWidth']);
  assert.equal(filterParams(defs, defaults, defaults, { filter: 'changed' }).length, 0);
});

test('search matches label and description, case-insensitive', () => {
  assert.deepEqual(filterParams(defs, defaults, defaults, { filter: 'all', query: 'FLYWHEEL' }).map((d) => d.id), ['shotSpeedPerRps']);
  assert.deepEqual(filterParams(defs, defaults, defaults, { filter: 'all', query: 'side to' }).map((d) => d.id), ['robotWidth']);
});

test('unmeasured, from-code, group and problem filters', () => {
  assert.deepEqual(filterParams(defs, defaults, defaults, { filter: 'unmeasured' }).map((d) => d.id), ['shotSpeedPerRps']);
  assert.deepEqual(filterParams(defs, defaults, defaults, { filter: 'code' }).map((d) => d.id), ['robotWidth']);
  assert.deepEqual(filterParams(defs, defaults, defaults, { filter: 'Shooter' }).map((d) => d.id), ['shotSpeedPerRps']);
  assert.deepEqual(filterParams(defs, { robotWidth: 700, shotSpeedPerRps: 0.1 }, defaults, { filter: 'problems' }).map((d) => d.id), ['robotWidth']);
});

test('out of range gives a blocking problem with the unit', () => {
  assert.match(paramProblem({ label: 'Width', unit: 'mm', min: 150, max: 600 }, 700), /above the max \(600\)/);
  assert.match(paramProblem({ label: 'Width', unit: 'mm', min: 150, max: 600 }, 100), /below the min \(150\)/);
  assert.equal(paramProblem({ label: 'Width', unit: 'mm', min: 150, max: 600 }, 400), null);
  assert.match(paramProblem({ label: 'RPM', unit: 'RPM', min: 0, max: 6000 }, 6800), /^6,800 RPM is above the max \(6,000\)\. Runs are blocked until you fix it\.$/);
  assert.match(paramProblem({ label: 'Width', unit: 'mm', min: 1, max: 2 }, NaN), /number/);
});
