import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorld } from './world.mjs';

// robot.json (sim/robot.schema.md) reaches the world as init.robotModel. Without it the old built-in values apply.
const SIZE = 3657.6;
const POSE = [600, 600, 0];
const BEAM = (name, x, z) => ({ name, emitter: [x, -40, z], receiver: [x, 40, z] });

function init(robotModel, preload = 0) {
  return {
    pose: POSE,
    robot: { frontMm: 300, backMm: 229, widthMm: 457, massKg: 12 },
    field: { sizeMm: SIZE, boxes: [] },
    drive: { kV: 0.00036, kOmegaMmPerRad: 180.7, responseSeconds: 0.08, brakeResponseSeconds: 0.03, trackWidthMm: 310.1, backDistanceMm: -91.2 },
    balls: { flowers: [], gardens: [], hives: [{ name: 'red-hive', x: 2000, y: 1800, upSide: 1, nectarType: 1 }], preload },
    robotModel,
  };
}
const held = (w) => w.ballPositions().filter((b) => b[5] === 1).map(([id, , x, , z]) => ({ id, x, z }));
const OPEN = { intake: 1, stopper: 0, rps: 50, hood: 0.5, turret: 0 };

test('beams place the four ball slots: A_OUTER, A_INNER, B_INNER, B_OUTER', async () => {
  const model = { beams: [BEAM('beamBreakBOuter', -110, 140), BEAM('beamBreakAOuter', 110, 160), BEAM('beamBreakBInner', -30, 150), BEAM('beamBreakAInner', 30, 155)] };
  const w = await createWorld(init(model, 4));
  const got = held(w).map((b) => [Math.round(b.x - POSE[0]), Math.round(b.z)]).sort((a, b) => b[0] - a[0]);
  assert.deepEqual(got, [[110, 160], [30, 155], [-30, 150], [-110, 140]]);
});

test('without a robot model the slots are where they always were', async () => {
  const w = await createWorld(init(undefined, 4));
  assert.deepEqual(held(w).map((b) => Math.round(b.x - POSE[0])).sort((a, b) => b - a), [120, 40, -40, -120]);
});

test('shooter.exit sets where the shot leaves: height, and the turret pivot swings it around', async () => {
  const model = { turret: { pivot: [0, 0, 300] }, shooter: { exit: [100, 0, 620], hoodPivot: [80, 0, 500], hoodMinDeg: 68.5, hoodMaxDeg: 40 } };
  const w = await createWorld(init(model, 1));
  const [before] = held(w);
  w.debugStep(0.001, [0, 0, 0, 0], { ...OPEN, turret: Math.PI / 2 }); // turret quarter turn left: the exit swings to +y of the pivot
  const shot = w.ballPositions().find((b) => b[0] === before.id);
  assert.equal(shot[5], 0, 'the ball was shot');
  assert.ok(Math.abs(shot[4] - 620) < 25, `z ${shot[4]}`);
  assert.ok(Math.abs(shot[2] - POSE[0]) < 30 && shot[3] - POSE[1] > 80, `exit swung to +y: (${shot[2]}, ${shot[3]})`);
});

test('an intake zone sets how far past the front a floor ball is pulled in', async () => {
  const ballAhead = async (model, reachMm) => {
    const w = await createWorld(init(model));
    const id = w.debugAddBall(0, POSE[0] + 300 + reachMm, POSE[1], 36);
    w.debugStep(0.05, [0, 0, 0, 0], { intake: 1, stopper: 0.47, rps: 0, hood: 0, turret: 0 });
    return w.ballPositions().find((b) => b[0] === id)[5] === 1;
  };
  assert.equal(await ballAhead(undefined, 120), true, 'default reach is 140 mm');
  const short = { intakeZones: [{ name: 'front', side: 'front', min: [300, -150, 0], max: [400, 150, 60] }] };
  assert.equal(await ballAhead(short, 120), false, 'zone reaches 100 mm: 120 mm is out');
  assert.equal(await ballAhead(short, 80), true, '80 mm is in');
});
