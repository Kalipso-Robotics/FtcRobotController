import test from 'node:test';
import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { gamepadLine, setLine, createLive, BUTTON_NAMES } from './live.mjs';

/** A stand-in for the Java process: records what is written to stdin, lets the test push stdout lines and end the process. */
function fakeChild() {
  const c = new EventEmitter();
  c.stdout = new EventEmitter(); c.stderr = new EventEmitter();
  c.written = []; c.stdinEnded = false; c.killed = null;
  c.stdin = new EventEmitter();
  c.stdin.write = (s) => { c.written.push(s); return true; };
  c.stdin.end = () => { c.stdinEnded = true; };
  c.kill = (sig) => { c.killed = sig; c.emit('close', null); };
  c.say = (...lines) => c.stdout.emit('data', `${lines.join('\n')}\n`);
  return c;
}
const OPMODE = { name: 'Live', alliance: 'red', start: { x: 0, y: 0, headingDeg: 0 } };
function harness(opts = {}) {
  const child = fakeChild();
  const events = [];
  const live = createLive({ launch: async () => child, emit: (name, data) => events.push([name, data]), opMode: OPMODE, ...opts });
  return { child, events, live, phases: () => events.filter((e) => e[0] === 'state').map((e) => e[1].phase) };
}

test('gamepadLine builds the stdin line and rejects malformed input', () => {
  assert.equal(gamepadLine({ lx: 0.5, ly: -1, rx: 0, ry: 0, lt: 0, rt: 1, buttons: 5 }), 'gp 1 0.5 -1 0 0 0 1 5');
  assert.equal(gamepadLine({}), 'gp 1 0 0 0 0 0 0 0');
  for (const bad of [null, 'x', { lx: 'a' }, { lx: NaN }, { buttons: -1 }, { buttons: 1.5 }, { buttons: 2 ** BUTTON_NAMES.length }]) assert.equal(gamepadLine(bad), null, JSON.stringify(bad));
});

test('a session goes starting -> running on #ready, passing the header and the frames on', async () => {
  const { child, events, live, phases } = harness();
  await live.start();
  assert.equal(live.state().phase, 'starting');
  child.say('#field 3657.6', '#robot 300 252 457', '#balltype 0 0 50', '#frame {"t":0}');   // a frame before #ready is dropped
  assert.equal(events.filter((e) => e[0] === 'frame').length, 0);
  child.say('#ready');
  assert.equal(live.state().phase, 'running');
  assert.equal(live.header().field.robot.back, 252);
  assert.equal(live.header().balls.info.length, 1);
  child.say('#frame {"t":1,"x":5}');
  assert.deepEqual(events.filter((e) => e[0] === 'frame').map((e) => e[1]), ['{"t":1,"x":5}']);
  assert.deepEqual(phases(), ['starting', 'running']);
});

test('input reaches stdin only while running, and bad input is refused', async () => {
  const { child, live } = harness();
  await live.start();
  assert.equal(live.input({ lx: 1 }), false);          // not running yet
  child.say('#ready');
  assert.equal(live.input({ lx: 1, buttons: 1 }), true);
  assert.equal(live.input({ lx: 'nope' }), false);
  assert.deepEqual(child.written, ['gp 1 1 0 0 0 0 0 1\n']);
});

test('only one session at a time; stop sends "stop", then the exit makes it stopped', async () => {
  const { child, live, phases } = harness();
  await live.start();
  await assert.rejects(() => live.start(), /already running/);
  child.say('#ready');
  live.stop(); live.stop();                              // twice is fine
  assert.deepEqual(child.written, ['stop\n']);
  assert.equal(child.stdinEnded, true);
  child.emit('close', 0);
  assert.equal(live.state().phase, 'stopped');
  assert.equal(live.active(), false);
  assert.deepEqual(phases().slice(-1), ['stopped']);
});

test('a process that ignores stop is killed after killAfterMs', async () => {
  const { child, live } = harness({ killAfterMs: 20 });
  await live.start();
  child.say('#ready');
  live.stop();
  await new Promise((r) => setTimeout(r, 80));
  assert.equal(child.killed, 'SIGKILL');
  assert.equal(live.state().phase, 'stopped');
});

test('a process that dies on its own is an error with its stderr text', async () => {
  const { child, live } = harness();
  await live.start();
  child.stderr.emit('data', 'WARNING: noise\nboom: no such opmode\n');
  child.emit('close', 1);
  assert.equal(live.state().phase, 'error');
  assert.match(live.state().message, /boom: no such opmode/);
  assert.doesNotMatch(live.state().message, /WARNING/);
  await live.start();                                    // a new session is allowed after an error
});

test('a launch failure is an error, and stop during the build still ends the session', async () => {
  const failing = createLive({ launch: async () => { throw new Error('build broke'); }, emit: () => {}, opMode: OPMODE });
  await failing.start();
  assert.deepEqual(failing.state(), { phase: 'error', message: 'build broke' });

  const child = fakeChild();
  let release;
  const slow = createLive({ launch: () => new Promise((r) => { release = () => r(child); }), emit: () => {}, opMode: OPMODE });
  const starting = slow.start();
  slow.stop();                                           // pressed Stop while compiling
  release();
  await starting;
  assert.deepEqual(child.written, ['stop\n']);
});

test('setLine accepts Class.FIELD with a number or boolean and nothing else', () => {
  assert.equal(setLine('SOTMConfig.RELEASE_LATENCY_MS', 60), 'set SOTMConfig.RELEASE_LATENCY_MS 60');
  assert.equal(setLine('SOTMConfig.enabled', true), 'set SOTMConfig.enabled true');
  for (const bad of [['a.b.C', 1], ['x', 1], ['A.b c', 1], ['A.b', '1; rm'], ['A.b', NaN], [null, 1], ['A.b', undefined]]) assert.equal(setLine(...bad), null, JSON.stringify(bad));
});

test('events, telemetry and set answers are passed on while running; set goes to stdin', async () => {
  const { child, events, live } = harness();
  await live.start();
  child.say('#ready', '#event 1.5|robot touched wall-x0', '#telemetry {"rps":"3"}', '#set ok SOTMConfig.enabled true');
  assert.deepEqual(events.filter((e) => e[0] !== 'state' && e[0] !== 'header').map((e) => e[0]), ['event', 'telemetry', 'set']);
  assert.equal(live.set('SOTMConfig.enabled', true), true);
  assert.equal(live.set('bad', true), false);
  assert.deepEqual(child.written, ['set SOTMConfig.enabled true\n']);
});
