import test from 'node:test';
import assert from 'node:assert/strict';
import { parseRun, simArgs, ROOT, OPMODES, PARAM_DEFS, BALL_DEFAULTS, JAVA_HOME } from './runfile.mjs';

const text = [
  '#matchStart 0.75', '#used 4.5', '#finished true', '#alliance ignored',
  '0.5,10,20,0,10,20,0', '1.0,100,200,1,100,200,1', '1.5,300,400,1,300,400,1',
  '#event 1.25|robot touched partner', '#warning slow', '#field 3657.6', '#robot 300 229 457',
].join('\n');

test('parseRun shifts times to match seconds and reads events, warnings and field', () => {
  const run = parseRun(text, { name: 'X', alliance: 'red', start: { x: 0, y: 0 } });
  assert.equal(run.opMode, 'X'); assert.equal(run.used, 4.5); assert.equal(run.finished, true);
  assert.deepEqual(run.frames[0], [0, 10, 20, 0, 10, 20, 0]);   // last frame before START kept at t = 0
  assert.equal(run.frames[run.frames.length - 1][0], 0.75);
  assert.deepEqual(run.events, [{ t: 0.5, text: 'robot touched partner' }]);
  assert.deepEqual(run.warnings, ['slow']);
  assert.equal(run.field.robot.width, 457);
});

test('importing runfile.mjs loads the shared constants and starts nothing', () => {
  assert.ok(ROOT.endsWith('FtcRobotController') || ROOT.length > 1);
  assert.ok(OPMODES.length && PARAM_DEFS.length && BALL_DEFAULTS && JAVA_HOME);
});

test('simArgs builds the JVM -D arguments and leaves out what is not given', () => {
  assert.deepEqual(simArgs(), ['-Djdk.attach.allowAttachSelf=true']);
  assert.deepEqual(
    simArgs({ seed: 7, logDir: '/tmp/o', autoParams: '/tmp/p.json', params: ['-Dsim.param.a=1'], partnerPath: '/x/r2.json', partnerSpeed: 0.5 }),
    ['-Djdk.attach.allowAttachSelf=true', '-Dsim.seed=7', '-Dsim.physics.seed=7', '-Dsim.logDir=/tmp/o', '-Dsim.autoParams=/tmp/p.json',
      '-Dsim.param.a=1', '-Dsim.partnerPath=/x/r2.json', '-Dsim.partnerSpeed=0.5']);
});

test('parseRun reads the defenders: info, then frames shifted to match seconds', () => {
  const t = ['#matchStart 1', '0.5,0,0,0,0,0,0', '2.0,0,0,0,0,0,0', '#defenderinfo defend_red_hive 200.0 200.0 400.0',
    '#defender 0 0.5 2000 1800 3.14', '#defender 0 2.0 1900 1700 3.14'].join('\n');
  const run = parseRun(t, { name: 'X', alliance: 'red', start: { x: 0, y: 0 } });
  assert.equal(run.defenders.length, 1);
  assert.deepEqual(run.defenders[0].robot, { front: 200, back: 200, width: 400 });
  assert.deepEqual(run.defenders[0].frames, [[0, 2000, 1800, 3.14], [1, 1900, 1700, 3.14]]);
});

test('simArgs passes the defender file', () => {
  assert.ok(simArgs({ defenderPath: '/x/d.json' }).includes('-Dsim.defenders=/x/d.json'));
});

test('teleops are kept out of the auto list', async () => {
  const { OPMODES, TELEOPS } = await import('./runfile.mjs');
  assert.ok(OPMODES.length && OPMODES.every((o) => o.kind !== 'teleop'));
  assert.ok(TELEOPS.length && TELEOPS.every((o) => o.kind === 'teleop'));
});

test('parseRun reads turret/hood/rps columns, #log, and merges #shot with #shotresult', () => {
  const t = ['#matchStart 1', '0.5,1,2,0,1,2,0,5,0.5,40', '2.0,1,2,0,1,2,0,10,0.6,41.5',
    '#log 2.0|KLog|BLOCKED - Waiting on a|b', '#shot 2.0|{"n":1,"waitStart":1.5,"gateMs":{"rps":230},"lastGate":"rps","aimed":"red-up","aimedUp":true}',
    '#shotresult 2.4|{"n":1,"in":false,"long":74,"left":-12}'].join('\n');
  const run = parseRun(t, { name: 'X', alliance: 'red', start: {} });
  assert.deepEqual(run.frames[1], [1, 1, 2, 0, 1, 2, 0, 10, 0.6, 41.5]);
  assert.deepEqual(run.logs, [{ t: 1, tag: 'KLog', msg: 'BLOCKED - Waiting on a|b' }]);
  assert.equal(run.shots.length, 1);
  assert.equal(run.shots[0].t, 1); assert.equal(run.shots[0].waitStart, 0.5);
  assert.equal(run.shots[0].result.long, 74);
});

test('old 7-column runs have no logs or shots', () => {
  const run = parseRun('0,1,2,0,1,2,0', { name: 'X', alliance: 'red', start: {} });
  assert.equal(run.frames[0].length, 7); assert.deepEqual(run.logs, []); assert.deepEqual(run.shots, []);
});
