import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorld } from './world.mjs';

// The field as FieldConfig.java and the manual (Figure 10-2) describe it, in the Red frame.
const SIZE = 3657.6;
const HIVE_CENTRE = SIZE / 2;
const RED_X = HIVE_CENTRE - 323.75, BLUE_X = HIVE_CENTRE + 323.75; // the two Hives are 25.5 in apart (Figure 9-10)

function init({ withBalls = true, pose = [600, 600, 0] } = {}) {
  return {
    pose,
    robot: { frontMm: 300, backMm: 229, widthMm: 457, massKg: 12 },
    field: { sizeMm: SIZE, boxes: [] },
    drive: { kV: 0.00036, kOmegaMmPerRad: 180.7, responseSeconds: 0.08, brakeResponseSeconds: 0.03, trackWidthMm: 310.1, backDistanceMm: -91.2 },
    balls: withBalls ? {
      flowers: [
        { name: 'A', x: 225, y: 2500 }, { name: 'B', x: 1219, y: 157.6 },
        { name: 'C', x: SIZE - 225, y: SIZE - 2500 }, { name: 'D', x: SIZE - 1219, y: SIZE - 157.6 },
      ],
      gardens: [
        { name: 'red', x0: 35.56, y0: SIZE - 35.56, dx: 71.12, dy: 0 },
        { name: 'blue', x0: SIZE - 35.56, y0: 35.56, dx: -71.12, dy: 0 },
      ],
      hives: [
        { name: 'red-hive', x: RED_X, y: HIVE_CENTRE, upSide: 1, nectarType: 1 },
        { name: 'blue-hive', x: BLUE_X, y: HIVE_CENTRE, upSide: -1, nectarType: 2 },
      ],
    } : undefined,
  };
}

const deg = (rad) => (rad * 180) / Math.PI;
const typeCount = (w) => w.ballInfo().reduce((c, [, type]) => { c[type] = (c[type] || 0) + 1; return c; }, {});

test('the manual\'s starting balls are all there: 16 pollen in Flowers, 8 in Gardens, 3 nectar in each up Cell', async () => {
  const w = await createWorld(init());
  assert.deepEqual(typeCount(w), { 0: 24, 1: 3, 2: 3 });
});

test('the balls settle where the manual puts them and the Hives stay as they start', async () => {
  const w = await createWorld(init());
  const info = w.ballInfo();
  w.ballsMoved(); // first call reports everything
  const at = (id) => w.debugBall(id);
  // Flower A: balls 0-3, stacked on the lower ring around (225, 2500): bottom ball resting on the floor
  assert.ok(Math.abs(at(0).z - 35.56) < 6, `bottom Flower ball z ${at(0).z}`);
  assert.ok(Math.abs(at(0).x - 225) < 15 && Math.abs(at(0).y - 2500) < 15, 'bottom ball sits in the ring');
  assert.ok(at(3).z > 200, `top Flower ball is held up by the stack (z ${at(3).z})`);
  // Gardens: 16-23 on the floor along the wall
  for (let id = 16; id < 24; id++) assert.ok(Math.abs(at(id).z - 35.56) < 2, `garden ball ${id} z ${at(id).z}`);
  // Hives: red up on the +y side, blue up on the -y side, 30 degrees
  const [red, blue] = w.hiveAngles();
  assert.ok(Math.abs(deg(red) - 30) < 2, `red Hive angle ${deg(red)}`);
  assert.ok(Math.abs(deg(blue) + 30) < 2, `blue Hive angle ${deg(blue)}`);
  // the 3 red nectar are still inside the red up Cell: high above the floor, at the +y side
  const nectar = info.filter(([, type]) => type === 1).map(([id]) => at(id));
  for (const n of nectar) assert.ok(n.z > 1000 && n.y > HIVE_CENTRE + 200, `nectar at (${n.x.toFixed(0)}, ${n.y.toFixed(0)}, ${n.z.toFixed(0)})`);
});

/** A point inside the red up Cell, in world coordinates, for a ball placed at the cell-local (y, z) from the pivot. */
function cellPoint(localY, localZ, xOffset = 0, tiltDeg = 30) {
  const a = (tiltDeg * Math.PI) / 180;
  return [RED_X + xOffset, HIVE_CENTRE + localY * Math.cos(a) - localZ * Math.sin(a), 1116 + localY * Math.sin(a) + localZ * Math.cos(a)];
}

test('weight in the up Cell tips the Hive near the team\'s own "3 nectar + 4 pollen" test, and the balls spill out', async () => {
  const w = await createWorld(init());
  let tippedAfter = null;
  const added = [];
  for (let n = 1; n <= 10 && tippedAfter === null; n++) {
    const [x, y, z] = cellPoint(300, 40, 120 + n * 20);
    added.push(w.debugAddBall(0, x, y, z));
    const state = w.debugStep(1.5);
    if (state.events.some((e) => e.name === 'red-hive tipped')) tippedAfter = n;
  }
  assert.ok(tippedAfter !== null, 'the Hive never tipped');
  assert.ok(tippedAfter >= 3 && tippedAfter <= 5, `tipped after ${tippedAfter} extra pollen (3 nectar + 4 pollen is the team's test)`);
  const [red] = w.hiveAngles();
  assert.ok(red < 0, `the red Hive is now tilted the other way (${deg(red).toFixed(1)} deg)`);

  // after the tip the balls spill out and end up on the floor
  w.debugStep(5);
  const info = w.ballInfo();
  const redNectar = info.filter(([, type]) => type === 1).map(([id]) => w.debugBall(id));
  for (const n of redNectar) assert.ok(n.z < 120, `spilled nectar should be on the floor, z ${n.z.toFixed(0)}`);
  for (const id of added) assert.ok(w.debugBall(id).z < 120, `added pollen should be on the floor, z ${w.debugBall(id).z.toFixed(0)}`);
});

test('the same ball run twice gives the same positions', async () => {
  const run = async () => {
    const w = await createWorld(init());
    w.debugAddBall(0, ...cellPoint(300, 40, 150));
    w.debugStep(2);
    return JSON.stringify(w.ballsMoved());
  };
  assert.equal(await run(), await run());
});

test('driving the intake along the red Garden picks the balls up, never more than 4', async () => {
  // facing -x with the front toward the Garden's corner, close to the wall but with the whole body inside the field
  const setup = init({ pose: [700, 3404, Math.PI] });
  const w = await createWorld(setup);
  let last;
  for (let i = 0; i < 60; i++) last = w.debugStep(0.1, [0.15, 0.15, 0.15, 0.15], { intake: 1 });
  assert.ok(last.held >= 3 && last.held <= 4, `picked up ${last.held} Garden balls`);
  assert.equal(last.slots.split('').filter((c) => c === '1').length, last.held);
});

test('the preload: 4 held pollen show in the slots and cannot be exceeded', async () => {
  const setup = init();
  setup.balls.preload = 4;
  const w = await createWorld(setup);
  const s = w.debugStep(0.05);
  assert.equal(s.held, 4);
  assert.equal(s.slots, '1111');
  // another ball in the mouth with the intake running is not taken: the robot is full
  const id = w.debugAddBall(0, 600 + 300 + 45, 600, 36);
  const s2 = w.debugStep(0.5, [0, 0, 0, 0], { intake: 1 });
  assert.equal(s2.held, 4);
  assert.ok(w.debugBall(id).z < 100, 'the extra ball stays on the floor');
});

test('a ball in the front mouth is taken when the intake runs, and not when it is off', async () => {
  const setup = init();
  const w = await createWorld(setup);
  const id = w.debugAddBall(0, 600 + 300 + 45, 600, 36);       // just ahead of the front tip
  const off = w.debugStep(0.3, [0, 0, 0, 0], { intake: 0 });
  assert.equal(off.held, 0);
  const on = w.debugStep(0.3, [0, 0, 0, 0], { intake: 1 });
  assert.equal(on.held, 1);
  assert.equal(on.slots, '1000', 'a ball entering at the front sits in the outermost front slot (A_OUTER)');
  const where = w.debugBall(id);
  assert.ok(Math.abs(where.x - (600 + 0.12 * 1000)) < 1 && where.z > 100, `held ball is inside the robot (${where.x.toFixed(0)}, ${where.z.toFixed(0)})`);
  assert.ok(on.events.some((e) => e.name.startsWith('*picked up')));
});

// ---- shots ----
const OPEN = { stopper: 0.31, intake: 1, rps: 45.6, hood: 0.1674, turret: 0 }; // what the code's table gives for launch point A
const LAUNCH_A = [1524, 3048.6];

function shotSetup(shotMode, theta = -Math.PI / 2) {
  const setup = init({ pose: [LAUNCH_A[0], LAUNCH_A[1], theta] });
  setup.balls.preload = 4;
  setup.balls.params = { shotMode };
  return setup;
}

test('Mode B (auto-aimed): the 4 preloaded balls fly into the red up Cell and, with its 3 nectar, tip it', async () => {
  const w = await createWorld(shotSetup(1));
  const events = [];
  for (let i = 0; i < 25; i++) events.push(...w.debugStep(0.1, [0, 0, 0, 0], OPEN).events);
  // stop the shooter so spilled balls the intake picks up are not shot again
  for (let i = 0; i < 55; i++) events.push(...w.debugStep(0.1, [0, 0, 0, 0], { ...OPEN, intake: 0 }).events);
  const shots = events.filter((e) => e.name.startsWith('*shot'));
  const inCell = events.filter((e) => e.name.includes('went into'));
  assert.equal(shots.length, 4, 'four shots');
  assert.equal(inCell.length, 4, `all four go in (${inCell.map((e) => e.name).join('; ')})`);
  assert.ok(events.some((e) => e.name === 'red-hive tipped'), '3 nectar + 4 pollen tips the Cell (the team\'s test)');
});

test('Mode A (from the flywheel and hood): the code\'s own table value for launch point A, aimed at the Cell, goes in', async () => {
  // aim: from (1524, 3048.6) toward the Cell opening at about (1505, 2229): straight along -y
  const w = await createWorld(shotSetup(0, -Math.PI / 2));
  const events = [];
  for (let i = 0; i < 30; i++) events.push(...w.debugStep(0.1, [0, 0, 0, 0], { ...OPEN, turret: 0 }).events);
  const inCell = events.filter((e) => e.name.includes('went into'));
  assert.ok(inCell.length >= 3, `at least 3 of 4 go in with the default numbers (${inCell.length})`);
});

test('no shot without the flywheel up to speed, the stopper open and the intake pushing', async () => {
  for (const act of [{ ...OPEN, rps: 5 }, { ...OPEN, stopper: 0.47 }, { ...OPEN, intake: 0 }]) {
    const w = await createWorld(shotSetup(1));
    const events = [];
    for (let i = 0; i < 10; i++) events.push(...w.debugStep(0.1, [0, 0, 0, 0], act).events);
    assert.equal(events.filter((e) => e.name.startsWith('*shot')).length, 0, JSON.stringify(act));
  }
});

test('Mode A shots follow the turret: aimed the other way they miss', async () => {
  const w = await createWorld(shotSetup(0, -Math.PI / 2));
  const events = [];
  for (let i = 0; i < 20; i++) events.push(...w.debugStep(0.1, [0, 0, 0, 0], { ...OPEN, turret: Math.PI }).events);
  assert.equal(events.filter((e) => e.name.includes('went into')).length, 0);
});

test('ballPositions lists every ball with its type, place and whether it is held', async () => {
  const w = await createWorld(init());
  await w.debugStep(1);
  const all = w.ballPositions();
  assert.equal(all.length, w.ballInfo().length);
  const gardenBall = all.find(([, type, x, y, z]) => type === 0 && z < 40 && y > SIZE - 100 && x < 300);
  assert.ok(gardenBall, 'a pollen ball lies on the floor in the red Garden');
  assert.ok(all.every(([, , , , , held]) => held === 0), 'nothing is held at the start');
});
