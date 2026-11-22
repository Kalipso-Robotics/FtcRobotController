import test from 'node:test';
import assert from 'node:assert/strict';
import { parseRun } from '../app/runfile.mjs';
import { explainRun, javaWaits } from './explain.mjs';

export const FIXTURE = [
  '#matchStart 0', '#used 30', '#finished true',
  '0,0,0,0,0,0,0,0,0.5,40', '30,0,0,0,0,0,0,0,0.5,40',
  '#telemetry 0 decide%20says=SHOOT_A&target=red-up',
  '#telemetry 10 decide%20says=SHOOT_B&target=blue-up',
  '#log 11.6|ActionBlocking|[PushBall] BLOCKED - Waiting on: [TurretReadyAuto]',
  '#log 12.2|ActionBlocking|[PushBall] STILL BLOCKED for 0.6s - Waiting on: [TurretReadyAuto]',
  '#log 11.9|KLog_ActionLifecycle|[driveToLaunchB] COMPLETED after 1.3s (130 updates)',
  '#log 11.95|KLog_ActionSet|[shootB] Skipping action [shoot] - blocked by: [turretReady]',
  '#log 12.25|KLog_ActionSet|[shootB] Skipping action [shoot] - blocked by: [turretReady]',
  '#shot 12.4|{"n":3,"waitStart":11.56,"gateMs":{"rps":230,"stopper":0},"lastGate":"rps","aimed":"red-up","aimedUp":true}',
  '#shotresult 12.9|{"n":3,"in":false,"long":74,"left":-12,"dist":2100,"speed":0}',
].join('\n');

test('explainRun narrates a mismatched, missed shot with physics and java waits', () => {
  const out = explainRun(parseRun(FIXTURE, { name: 'X', alliance: 'red', start: {} }), { title: 'fixture' });
  assert.match(out, /shots in 0\/1/);
  assert.match(out, /10\.0 s  SHOOT_B/);
  assert.match(out, /shot 3 @12\.4s: DELAY 500 ms after arriving; feed window 840 ms \(physics over whole feed window: rps 230 ms; java: PushBall waiting on TurretReadyAuto 600 ms; shoot blocked by turretReady 300 ms\); aimed red-up, decide wanted blue-up ⚠; MISS long 74 mm left -12 mm, dist 2100 mm, robot 0 mm\/s/);
});

test('old runs explain gracefully', () => {
  const out = explainRun(parseRun('0,0,0,0,0,0,0\n30,0,0,0,0,0,0', { name: 'X', alliance: 'red', start: {} }));
  assert.match(out, /no shot records/);
});

test('javaWaits ignores lines outside the shot window', () => {
  assert.deepEqual(javaWaits({ t: 5, waitStart: 4 }, [{ t: 1, msg: '[A] BLOCKED - Waiting on: [B]' }]), []);
});

test('delay counts from the drive finishing; a second shot of a burst has none', () => {
  const run = parseRun(FIXTURE + '\n#shot 12.5|{"n":4,"waitStart":12.4,"gateMs":{"interval":100},"lastGate":"interval","aimed":"red-up","aimedUp":true}', { name: 'X', alliance: 'red', start: {} });
  const out = explainRun(run);
  assert.match(out, /shot 4 @12\.5s: burst follow-up/);
});
