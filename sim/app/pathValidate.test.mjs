import test from 'node:test';
import assert from 'node:assert/strict';
import { validatePath } from './pathValidate.mjs';

const ok = () => ({ name: 'a', alliance: 'red', start: [100, 200, 90], steps: [{ drive: [500, 500, 0] }, { intake: true }, { shoot: 3 }, { wait: 1.5 }] });

test('a good path passes', () => assert.equal(validatePath(ok()), null));
test('bad shapes fail', () => {
  assert.match(validatePath(null), /object/);
  assert.match(validatePath({ ...ok(), name: '../x' }), /name/);
  assert.match(validatePath({ ...ok(), steps: [] }), /steps/);
  assert.match(validatePath({ ...ok(), alliance: 'green' }), /alliance/);
});
test('poses must be inside the field', () => {
  assert.match(validatePath({ ...ok(), start: [-1, 0, 0] }), /start/);
  assert.match(validatePath({ ...ok(), steps: [{ drive: [4000, 10, 0] }] }), /Step 1/);
  assert.match(validatePath({ ...ok(), steps: [{ drive: [1, 2] }] }), /Step 1/);
});
test('each step has one known type with a sane value', () => {
  assert.match(validatePath({ ...ok(), steps: [{}] }), /exactly one/);
  assert.match(validatePath({ ...ok(), steps: [{ drive: [1, 1, 0], wait: 1 }] }), /exactly one/);
  assert.match(validatePath({ ...ok(), steps: [{ intake: 'yes' }] }), /intake/);
  assert.match(validatePath({ ...ok(), steps: [{ shoot: 0 }] }), /shoot/);
  assert.match(validatePath({ ...ok(), steps: [{ wait: -1 }] }), /wait/);
  assert.match(validatePath({ ...ok(), steps: [{ drive: [1, 1, 0], speedMmS: 0 }] }), /speedMmS/);
});
