import test from 'node:test';
import assert from 'node:assert/strict';
import { debrief, spread } from './debrief.mjs';

/** 30 s: drive up x (shoot decision), drive down y (get decision), then sit still in PARK. R2 sits at (2000, 1000). */
function synthetic() {
  const frames = [];
  for (let k = 0; k <= 1500; k++) {
    const t = k * 0.02;
    const x = t <= 5 ? 500 + 200 * t : 1500;
    const y = t <= 5 ? 3000 : t <= 10 ? 3000 - 300 * (t - 5) : 1500;
    frames.push([t, x, y, 0, x, y, 0]);
  }
  const robot = { front: 200, back: 200, width: 400 };
  const tel = (t, move) => ({ t, items: { 'decide says': move } });
  return {
    opMode: 'Synth', alliance: 'red', frames, warnings: [], error: null, finished: true, used: 30, matchStart: 0,
    events: [{ t: 4, text: 'shot a pollen ball (1)' }, { t: 8, text: 'picked up a pollen ball (1 held)' }, { t: 9, text: 'robot touched wall-x0' }],
    telemetry: [tel(0, 'SHOOT_A'), tel(5, 'GET_FLOWER_B: far'), tel(10, 'PARK')],
    partner: { name: 'p', robot, frames: [[0, 2000, 1000, 0], [30, 2000, 1000, 0]] },
    balls: { info: [{ id: 0, type: 0, radius: 40 }, { id: 1, type: 0, radius: 40 }], tracks: [[[0, 1500, 1200, 20]], [[0, 3000, 300, 20]]], hive: [], held: [], params: {} },
    field: { size: 3657.6, robot, obstacles: [{ name: 'hive-leg', minX: 1200, minY: 1334, maxX: 2457, maxY: 2323 }], hive: { minX: 1200, minY: 1334, maxX: 2457, maxY: 2323 }, midline: 1828.8, points: [] },
  };
}

test('debrief md of a synthetic run', () => {
  const { md, png } = debrief(synthetic(), { title: 'Synth s1' });
  assert.equal(md, `# Debrief: Synth s1

Auto score: 3 (Left the start +3)

## Time per decision
| # | decision | s | drive | intake | shoot | idle | path eff |
|---|---|---|---|---|---|---|---|
| 1 | SHOOT_A | 5.0 | 5.0 | 0.0 | 0.0 | 0.0 | 1.00 |
| 2 | GET_FLOWER_B | 5.0 | 5.0 | 0.0 | 0.0 | 0.0 | 1.00 |
| 3 | PARK | 20.0 | 0.0 | 0.0 | 0.0 | 20.0 | - |

## Spacing
- Closest to R2: 141 mm at 10.0 s
- Closest to a wall: 300 mm at 0.0 s

## Loose balls at 30.0 s
- 1 floor ball(s) outside the Hive within 400 mm of the path (2 balls in the match)

## Spare time
- Not parked: no spare time to report. The robot was still for the last 20.0 s.

## Changes vs best
- no best run given
`);
  assert.deepEqual([...png.subarray(0, 4)], [137, 80, 78, 71]);
});

test('spread: mean/min/max of a list', () => assert.deepEqual(spread([1, 2, 6]), { mean: 3, min: 1, max: 6 }));

import { parseRun } from '../app/runfile.mjs';
import { FIXTURE } from './explain.test.mjs';
test('debrief adds a Shots section only when the run has shot records', () => {
  const run = parseRun(FIXTURE, { name: 'X', alliance: 'red', start: { x: 0, y: 0 } });
  assert.match(debrief(run).md, /## Shots\n- shot 3 @12\.4s/);
  assert.doesNotMatch(debrief(synthetic()).md, /## Shots/);
});
