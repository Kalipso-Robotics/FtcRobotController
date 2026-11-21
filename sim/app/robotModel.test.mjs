import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { robotToModel, modelToRobot, originOffset } from './web/robotModel.js';

const md = readFileSync(new URL('../robot.schema.md', import.meta.url), 'utf8');
const example = JSON.parse(md.match(/```json\n([\s\S]*?)```/)[1]);

test('schema example round-trips losslessly', () => {
  assert.deepEqual(modelToRobot(robotToModel(example)), example);
});

test('missing optional keys stay missing', () => {
  const r = { version: 1, beams: [{ name: 'beamBreakAOuter', emitter: [1, 2, 3], receiver: [4, 5, 6] }] };
  const out = modelToRobot(robotToModel(r));
  assert.deepEqual(out, r);
  assert.deepEqual(modelToRobot(robotToModel(null), { front: 1, back: 2, width: 3 }), { version: 1, footprint: { front: 1, back: 2, width: 3 } });
});

test('origin click: offset = -(R * (s * p))', () => {
  assert.deepEqual(originOffset([10, 20, 30], 1, [0, 0, 0]), [-10, -20, -30]);
  assert.deepEqual(originOffset([1, 2, 3], 25.4, [0, 0, 0]).map((c) => +c.toFixed(6)), [-25.4, -50.8, -76.2]);
  const o = originOffset([10, 0, 0], 2, [0, 0, 90]);   // 20 along x, rotated +90 about z -> +y
  assert.ok(Math.abs(o[0]) < 1e-9 && Math.abs(o[1] + 20) < 1e-9 && Math.abs(o[2]) < 1e-9);
  const o2 = originOffset([0, 0, 5], 1, [90, 0, 0]);   // +z rotated +90 about x -> -y
  assert.ok(Math.abs(o2[1] - 5) < 1e-9);
});
