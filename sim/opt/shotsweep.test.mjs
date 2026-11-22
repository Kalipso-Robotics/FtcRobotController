import test from 'node:test';
import assert from 'node:assert/strict';
import { plan, widePlan, bearingTable, parseResult, fitLine } from './shotsweep.mjs';

test('the plan keeps every start inside the field and pairs each pass with SOTM off and on', () => {
  const runs = plan([500, 1000]);
  for (const r of runs) assert.ok(r.start.x > 300 && r.start.x < 3357 && r.start.y > 300 && r.start.y <= 3657.6 - 252, r.name);
  const passes = runs.filter((r) => r.kind === 'pass');
  assert.equal(passes.length, 2 * 4 * 2);
  assert.equal(passes.filter((r) => r.sotm).length, passes.length / 2);
});

test('parseResult reads in, miss and short lines and ignores others', () => {
  assert.deepEqual(parseResult('5.9|result: shot 1 in, long -10 mm, left 19 mm, flight 333 ms, dist 819 mm, robot 0 mm/s'),
    { in: true, long: -10, left: 19, flightMs: 333, dist: 819, robotMmS: 0 });
  assert.equal(parseResult('result: shot 2 miss, short: never reached the opening height, dist 2000 mm, robot 950 mm/s').long, null);
  assert.equal(parseResult('shot a pollen ball (45.6 RPS, 63°, 5.6 m/s)'), null);
});

test('fitLine recovers a line', () => {
  const f = fitLine([[1000, 530], [2000, 716], [3000, 902]]);
  assert.ok(Math.abs(f.a - 344) < 1e-6 && Math.abs(f.b - 0.186) < 1e-9);
});

import { tablePlan, standStart, pickCell, refineAxes, interp, diffVsCurrent, javaArray, CURRENT_TABLE } from './shotsweep.mjs';

const grid = (rpsList, hoodList, isIn) => rpsList.flatMap((rps) => hoodList.map((hood) => ({ rps, hood, in: isIn(rps, hood), long: isIn(rps, hood) ? 5 : 400, left: 5 })));

test('standStart fits the field, drops what cannot fit', () => {
  const s = standStart(900);
  assert.ok(Math.abs(Math.hypot(s.x - 1524, s.y - 2133.6) - 900) < 1e-6);
  const far = standStart(2000);
  assert.ok(far && far.x <= 3357 && far.y <= 3657.6 - 257);
  assert.ok(Math.abs(Math.hypot(far.x - 1524, far.y - 2133.6) - 2000) < 1e-6);
  assert.equal(standStart(4200), null);
});

test('tablePlan is distances x rps x hood, skipping unreachable distances', () => {
  const runs = tablePlan([900, 4200], [30, 34], [0.13, 0.2, 0.3]);
  assert.equal(runs.length, 6);
  assert.equal(new Set(runs.map((r) => r.name)).size, 6);
});

test('pickCell takes the middle of the largest hit region, away from misses', () => {
  const rpsL = [30, 34, 38, 42, 46, 50, 54], hoodL = [0.13, 0.17, 0.21, 0.25, 0.29];
  // big block rps 34..46 x hood 0.17..0.25 in, plus an isolated hit at rps 54
  const cells = grid(rpsL, hoodL, (r, h) => (r >= 34 && r <= 46 && h >= 0.17 && h <= 0.25) || (r === 54 && h === 0.13));
  const p = pickCell(cells, { rpsScale: 4, hoodScale: 0.04 });
  assert.ok(p.rps === 38 || p.rps === 42);   // 40 is not a grid line
  assert.ok(Math.abs(p.hood - 0.21) < 1e-9);
  assert.equal(p.region, 12);
  assert.equal(pickCell(grid([1, 2], [0.1], () => false)), null);
});

test('pickCell with no misses picks the smallest miss distance', () => {
  const cells = [{ rps: 40, hood: 0.2, in: true, long: 50, left: 0 }, { rps: 44, hood: 0.2, in: true, long: 5, left: 3 }];
  assert.equal(pickCell(cells).rps, 44);
});

test('refineAxes returns midpoints near the hit band only', () => {
  const rpsL = [30, 34, 38, 42, 46, 50, 54, 58], hoodL = [0.13, 0.17, 0.21];
  const cells = grid(rpsL, hoodL, (r) => r === 42 || r === 46);
  const a = refineAxes(cells, rpsL, hoodL);
  assert.deepEqual(a.rps, [40, 44, 48]);
  assert.deepEqual(a.hood, [0.15, 0.19]);
  assert.deepEqual(refineAxes([{ rps: 30, hood: 0.1, in: false, long: null, left: null }], rpsL, hoodL), { rps: [], hood: [] });
});

test('interp, diffVsCurrent and javaArray', () => {
  assert.deepEqual(interp(CURRENT_TABLE, 810), [45.65, 0.1284]);
  assert.deepEqual(interp(CURRENT_TABLE, 99999), [73.45, 0.295]);
  const d = diffVsCurrent([[1050, 44.65, 0.2]]);
  assert.equal(d[0].dRps, -1);
  assert.equal(d[0].dHood, -0.01);
  assert.match(javaArray([[1050, 44.65, 0.2]]), /1050, 44.65, 0.2,\n};/);
});

test('widePlan: standing grid inside the field, bearing 0 once, moving passes cross the line to the target', () => {
  const runs = widePlan([500, 1000]);
  const stand = runs.filter((r) => r.kind === 'stand');
  for (const r of runs) assert.ok(r.start.x >= 252 && r.start.x <= 3405 && r.start.y <= 3657.6 - 252 - 5, r.name);
  assert.equal(new Set(runs.map((r) => r.name)).size, runs.length);
  assert.deepEqual(stand.filter((r) => r.bearing === 0).map((r) => r.d), [1000]);   // only d=1000 fits straight back (y <= MAX_Y)
  assert.ok(stand.some((r) => r.name === 'wide-b55-L-1500'));
  assert.ok(stand.some((r) => r.name === 'wide-b55-R-1500'));
  const s = stand.find((r) => r.name === 'wide-b55-L-1500');
  assert.ok(Math.abs(s.start.x - (1524 + 1500 * Math.sin(55 * Math.PI / 180))) < 1e-6);
  assert.ok(Math.abs(s.start.y - (3657.6 - 1524 + 1500 * Math.cos(55 * Math.PI / 180))) < 1e-6);
  const moving = runs.filter((r) => r.kind === 'pass');
  assert.ok(moving.length > 0 && moving.every((r) => [45, 70].includes(r.bearing)));
  assert.deepEqual([...new Set(moving.map((r) => r.speed))].sort((a, b) => a - b), [500, 1000]);
  assert.ok(moving.some((r) => r.sotm) && moving.some((r) => !r.sotm));
  for (const r of moving) {   // velocity is perpendicular to the line from the release point to the target
    const dx = 1524 - r.release.x, dy = 3657.6 - 1524 - r.release.y;
    assert.ok(Math.abs(dx * r.v.x + dy * r.v.y) < 1e-6, r.name);
    assert.ok(Math.abs(Math.hypot(dx, dy) - 1500) < 1e-6, r.name);
  }
});

test('bearingTable buckets hit % by bearing for standing and moving separately', () => {
  const mk = (kind, bearing, i) => ({ kind, bearing, result: { in: i } });
  const t = bearingTable([mk('stand', 0, true), mk('stand', 0, false), mk('stand', 40, true), mk('pass', 70, false)]);
  assert.deepEqual(t.stand[0], { shots: 2, in: 1, pct: 50 });
  assert.deepEqual(t.stand[40], { shots: 1, in: 1, pct: 100 });
  assert.deepEqual(t.pass[70], { shots: 1, in: 0, pct: 0 });
});

import { enforceMonotonic } from './shotsweep.mjs';

test('enforceMonotonic lifts an rps dip to the previous row, leaves hood alone', () => {
  const rows = enforceMonotonic([{ d: 1050, rps: 58, hood: 0.2 }, { d: 1100, rps: 62, hood: 0.21 }, { d: 1200, rps: 57, hood: 0.22 }]);
  assert.deepEqual(rows.map((r) => r.rps), [58, 62, 62]);
  assert.deepEqual(rows.map((r) => r.hood), [0.2, 0.21, 0.22]);
});

test('standStart with a bearing puts the stand on that bearing, or null when off the field', () => {
  const s = standStart(1500, 40);
  assert.ok(Math.abs(Math.hypot(s.x - 1524, s.y - 2133.6) - 1500) < 1e-6);
  assert.ok(Math.abs(Math.atan2(s.x - 1524, s.y - 2133.6) * 180 / Math.PI - 40) < 1e-6);
  assert.equal(standStart(3000, 40), null);
});

import { onlyRuns } from './shotsweep.mjs';
test('onlyRuns keeps runs whose name matches the regex, all when empty', () => {
  const runs = [{ name: 'wide-b55-L-1500' }, { name: 'wide-b70-R-1000' }];
  assert.deepEqual(onlyRuns(runs, 'b70').map((r) => r.name), ['wide-b70-R-1000']);
  assert.equal(onlyRuns(runs, '').length, 2);
});

import { cellPoint, gridPoints, weakCheck, gridPlan, javaGrid } from './shotsweep.mjs';

test('cellPoint mirrors the forward axis for B and keeps side on +x', () => {
  const a = cellPoint('A', 900, 300), b = cellPoint('B', 900, 300);
  assert.deepEqual(a, { x: 1824, y: 2133.6 + 900 });
  assert.ok(Math.abs(b.y - (3657.6 - 2135 - 900)) < 1e-9 && b.x === 1824);
});

test('gridPoints drops off-field points and keeps the grid order', () => {
  const pts = gridPoints('A', [600, 3000], [0, 300, 3000]);
  assert.deepEqual(pts.map((p) => [p.forward, p.side]), [[600, 0], [600, 300]]);   // forward 3000 and side 3000 leave the field
  assert.ok(gridPoints('A', [600], [0]).every((p) => p.x === 1524 && Math.abs(p.y - 2733.6) < 1e-9));
});

test('gridPoints drops points inside a Hive leg', () => {
  assert.equal(gridPoints('A', [-600], [-300]).length, 0);   // beside the target, inside the Hive leg zone
});

test('weakCheck: the pick must go in at +-1 RPS and +-1.5 deg, neighbours past the hood range are skipped', () => {
  const pick = { rps: 50, hoodDeg: 70 };
  const nb = weakCheck(pick, 60, 70);
  assert.deepEqual(nb.map((n) => [n.rps, n.hoodDeg]), [[49, 70], [51, 70], [50, 68.5]]);
  assert.equal(weakCheck({ rps: 50, hoodDeg: 65 }, 60, 70).length, 4);
});

test('gridPlan names are unique per point, rps, hood and carry cell, deg and heading', () => {
  const pts = gridPoints('A', [600], [0, 300]);
  const runs = gridPlan(pts, [40, 44], [60, 65]);
  assert.equal(runs.length, 8);
  assert.equal(new Set(runs.map((r) => r.name)).size, 8);
  assert.ok(runs.every((r) => r.hoodDeg !== undefined && r.cell === 'A' && r.sotm === false));
});

test('javaGrid writes static final double arrays', () => {
  const t = javaGrid([600, 900], [0, 300], [[40, 41], [42, 43]], [[60, 65], [61, 66]]);
  assert.match(t, /static final double\[\] FORWARD_MM = \{600, 900\}/);
  assert.match(t, /static final double\[\]\[\] RPS = \{\n\s+\{40, 41\},\n\s+\{42, 43\}/);
  assert.match(t, /HOOD_DEG/);
});
