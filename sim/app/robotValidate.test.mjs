import test from 'node:test';
import assert from 'node:assert/strict';
import { validateRobot } from './robotValidate.mjs';

const ok = { version: 1, meshes: [{ file: 'body.stl', part: 'body', scale: 1, rotationDeg: [0, 0, 0], offset: [0, 0, 0] }],
  shooter: { exit: [60, 0, 500], hoodPivot: [40, 0, 420], hoodMinDeg: 68.5, hoodMaxDeg: 40 }, beams: [{ name: 'beamBreakAOuter', emitter: [1, 2, 3], receiver: [1, 4, 3] }] };

test('a well-formed robot.json passes, and an empty one does too (every key is optional)', () => {
  assert.equal(validateRobot(ok), null);
  assert.equal(validateRobot({}), null);
});

test('bad files, parts, numbers and beam names are rejected before they reach the world', () => {
  assert.match(validateRobot({ meshes: [{ ...ok.meshes[0], file: '../x.stl' }] }), /mesh file/i);
  assert.match(validateRobot({ meshes: [{ ...ok.meshes[0], part: 'arm' }] }), /part/i);
  assert.match(validateRobot({ meshes: [{ ...ok.meshes[0], scale: 0 }] }), /scale/i);
  assert.match(validateRobot({ meshes: [{ ...ok.meshes[0], color: 'purple' }] }), /color/i);
  assert.equal(validateRobot({ meshes: [{ ...ok.meshes[0], color: '#6a3fa6' }] }), null);
  assert.match(validateRobot({ shooter: { ...ok.shooter, exit: [1, 2] } }), /shooter/i);
  assert.match(validateRobot({ beams: [{ name: 'nope', emitter: [1, 2, 3], receiver: [1, 2, 3] }] }), /beam/i);
  assert.match(validateRobot([]), /object/i);
});

test('missingMeshes warns (not errors) about mesh files that are not in sim/robot/', async () => {
  const { missingMeshes } = await import('./robotValidate.mjs');
  assert.deepEqual(missingMeshes(ok, ['body.stl']), []);
  assert.match(missingMeshes(ok, [])[0], /body\.stl/);
  assert.deepEqual(missingMeshes(null, []), []);
  assert.equal(validateRobot(ok), null);
});
