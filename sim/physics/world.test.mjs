import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorld } from './world.mjs';

const K_V = 0.00036;
const K_OMEGA = Math.sqrt(232.414 ** 2 + 276.58 ** 2) / 2;
const TAU = 0.08;

function init(pose, boxes = []) {
  return {
    pose,
    robot: { frontMm: 200, backMm: 200, widthMm: 388.7, massKg: 12 },
    field: { sizeMm: 3657.6, boxes },
    drive: {
      kV: K_V, kOmegaMmPerRad: K_OMEGA, responseSeconds: TAU, brakeResponseSeconds: 0.03,
      trackWidthMm: 310.1446862, backDistanceMm: -91.19632916,
    },
  };
}

const NO_BRAKE = [false, false, false, false];
const seconds = (s) => Math.round(s * 1e9);

test('free drive matches the wheel-lag equation', async () => {
  const world = await createWorld(init([1000, 1800, 0]));
  const p = 0.5;
  const s = world.advanceTo(seconds(1), [p, p, p, p], NO_BRAKE);
  const maxSpeed = p / K_V;
  const expected = maxSpeed * (1 - TAU * (1 - Math.exp(-1 / TAU)));
  assert.ok(Math.abs(s.x - 1000 - expected) < 2, `moved ${s.x - 1000} mm, expected ${expected}`);
  assert.ok(Math.abs(s.y - 1800) < 0.05); // Rapier keeps positions in 32-bit floats
  // pods read the real motion: forward distance on left and right, none sideways
  assert.ok(Math.abs(s.leftPodMm - expected) < 2);
  assert.ok(Math.abs(s.backPodMm) < 0.05);
});

test('turning in place follows the wheel mix', async () => {
  const world = await createWorld(init([1800, 1800, 0]));
  const p = 0.3;
  const s = world.advanceTo(seconds(1), [p, -p, p, -p], NO_BRAKE); // FL, FR, BL, BR
  const w = (p / K_V) / K_OMEGA;
  const expected = w * (1 - TAU * (1 - Math.exp(-1 / TAU)));
  assert.ok(Math.abs(s.theta - expected) < 0.01, `turned ${s.theta} rad, expected ${expected}`);
  assert.ok(Math.hypot(s.x - 1800, s.y - 1800) < 1, 'turning in place should not travel');
});

test('a wall stops the robot, and the pods do not count the blocked distance', async () => {
  const world = await createWorld(init([3000, 1800, 0]));
  let s;
  for (let t = 1; t <= 4; t++) s = world.advanceTo(seconds(t), [1, 1, 1, 1], NO_BRAKE);
  // the robot is 400 mm long, so its centre stops 200 mm from the wall at x = 3657.6
  assert.ok(Math.abs(s.x - (3657.6 - 200)) < 5, `centre at ${s.x}`);
  assert.ok(s.leftPodMm < 600, `pods counted ${s.leftPodMm} mm but the robot only moved ${s.x - 3000}`);
});

test('an obstacle box blocks the robot', async () => {
  const box = { name: 'test-box', cx: 2000, cy: 1800, hw: 100, hh: 100 };
  const world = await createWorld(init([1000, 1800, 0], [box]));
  let s;
  for (let t = 1; t <= 4; t++) s = world.advanceTo(seconds(t), [1, 1, 1, 1], NO_BRAKE);
  assert.ok(Math.abs(s.x - (2000 - 100 - 200)) < 5, `centre at ${s.x}`);
});

test('a contact is reported once, by name, when it begins', async () => {
  const world = await createWorld(init([3000, 1800, 0]));
  const all = [];
  for (let t = 1; t <= 4; t++) all.push(...world.advanceTo(seconds(t), [1, 1, 1, 1], NO_BRAKE).events);
  assert.deepEqual(all.map((e) => e.name), ['wall-xMax']);
  assert.ok(all[0].nanos > 0 && all[0].nanos < seconds(2), `contact at ${all[0].nanos} ns`);
});

test('the robot footprint can sit off the odometry point', async () => {
  // front 300, back 100: driving forward into the wall at x = 3657.6 stops with the centre 300 mm away
  const asymmetric = init([3000, 1800, 0]);
  asymmetric.robot = { frontMm: 300, backMm: 100, widthMm: 388.7, massKg: 12 };
  const world = await createWorld(asymmetric);
  let s;
  for (let t = 1; t <= 4; t++) s = world.advanceTo(seconds(t), [1, 1, 1, 1], NO_BRAKE);
  assert.ok(Math.abs(s.x - (3657.6 - 300)) < 5, `centre at ${s.x}`);
});

test('the same run twice gives the same numbers', async () => {
  const run = async () => {
    const world = await createWorld(init([1500, 1500, 0.3]));
    let s;
    for (let t = 1; t <= 5; t++) s = world.advanceTo(seconds(t * 0.37), [0.9, 0.2, -0.4, 0.7], NO_BRAKE);
    return JSON.stringify(s);
  };
  assert.equal(await run(), await run());
});
