import test from 'node:test';
import assert from 'node:assert/strict';
import { parseCompileError } from './buildError.mjs';

const out142 = '/r/TeamCode/src/main/java/a/RedCloseAuto.java:142: error: cannot find symbol\n        shooter.shootVolly(3);\n               ^\n  symbol:   method shootVolly(int)\n  location: variable shooter of type Shooter\n';

test('first compile error becomes file, line, message and three context lines', () => {
  const src = Array.from({ length: 150 }, (_, i) => `line${i + 1}`).join('\n');
  const e = parseCompileError(out142, '/r', () => src);
  assert.equal(e.file, 'TeamCode/src/main/java/a/RedCloseAuto.java');
  assert.equal(e.line, 142);
  assert.equal(e.col, 16);
  assert.equal(e.message, 'cannot find symbol');
  assert.equal(e.detail, 'symbol: method shootVolly(int) · location: variable shooter of type Shooter');
  assert.deepEqual(e.context.map((c) => c.n), [141, 142, 143]);
  assert.equal(e.context[1].caret, 16);
});
test('no error line gives null', () => assert.equal(parseCompileError('BUILD FAILED', '/r', () => null), null));
test('unreadable source still gives the message with no context', () => assert.deepEqual(parseCompileError(out142, '/r', () => null).context, []));
