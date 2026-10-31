import test from 'node:test';
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createWorld, partnerPoseAt } from './world.mjs';

const K_V = 0.00036;
const K_OMEGA = Math.sqrt(232.414 ** 2 + 276.58 ** 2) / 2;

function init(pose, partner) {
  return {
    pose,
    robot: { frontMm: 200, backMm: 200, widthMm: 388.7, massKg: 12 },
    field: { sizeMm: 3657.6, boxes: [] },
    drive: { kV: K_V, kOmegaMmPerRad: K_OMEGA, responseSeconds: 0.08, brakeResponseSeconds: 0.03, trackWidthMm: 310.1446862, backDistanceMm: -91.19632916 },
    ...(partner ? { partner } : {}),
  };
}
const R2 = (poses, speed = 1) => ({ name: 'p', front: 200, back: 200, width: 400, speed, poses });
const NO_BRAKE = [false, false, false, false];
const nanos = (s) => Math.round(s * 1e9);

test('partner follows its path exactly', async () => {
  const w = await createWorld(init([500, 500, 0], R2([[0, 2500, 2500, 0], [2, 2500, 1500, 0]])));
  w.partnerStart(0);
  w.advanceTo(nanos(1), [0, 0, 0, 0], NO_BRAKE);
  const [x, y] = w.partnerPose();
  assert.ok(Math.abs(x - 2500) < 1 && Math.abs(y - 2000) < 2, `at ${x}, ${y}`);
});

test('R1 driving into the partner stops and reports one partner contact', async () => {
  const w = await createWorld(init([500, 500, 0], R2([[0, 1500, 500, 0], [10, 1500, 500, 0]])));
  w.partnerStart(0);
  const names = [];
  let s;
  for (let t = 1; t <= 3; t++) { s = w.advanceTo(nanos(t), [1, 1, 1, 1], NO_BRAKE); names.push(...s.events.map((e) => e.name)); }
  assert.ok(s.x < 1100 + 5 && s.x > 1000, `R1 x = ${s.x}`);
  assert.deepEqual(names, ['partner']);
});

test('partner speed 0.5 is halfway along at the same time', async () => {
  const w = await createWorld(init([500, 500, 0], R2([[0, 2500, 2500, 0], [4, 2500, 1500, 0]], 0.5)));
  w.partnerStart(0);
  w.advanceTo(nanos(4), [0, 0, 0, 0], NO_BRAKE);
  const [, y] = w.partnerPose();
  assert.ok(Math.abs(y - 2000) < 2, `y = ${y}`);
});

test('partnerPoseAt holds the ends and takes the short way round', () => {
  assert.deepEqual(partnerPoseAt([[0, 1, 2, 0], [1, 3, 4, 0]], -1), [1, 2, 0]);
  assert.deepEqual(partnerPoseAt([[0, 1, 2, 0], [1, 3, 4, 0]], 5), [3, 4, 0]);
  const [, , th] = partnerPoseAt([[0, 0, 0, 3.0], [1, 0, 0, -3.0]], 0.5);
  assert.ok(Math.abs(Math.abs(th) - Math.PI) < 0.2, `theta ${th}`);
});

test('no partner: no partner field in the state', async () => {
  const w = await createWorld(init([500, 500, 0]));
  assert.equal('partner' in w.advanceTo(nanos(0.1), [0, 0, 0, 0], NO_BRAKE), false);
});

async function stepReplyTokens(partner) {
  const child = spawn('node', ['server.mjs'], { cwd: import.meta.dirname });
  let buf = '';
  const replies = [];
  child.stdout.on('data', (d) => { buf += d; let i; while ((i = buf.indexOf('\n')) >= 0) { replies.push(buf.slice(0, i)); buf = buf.slice(i + 1); } });
  const ask = async (line) => { const n = replies.length; child.stdin.write(`${line}\n`); while (replies.length === n) await new Promise((r) => setTimeout(r, 5)); return replies[n]; };
  await ask(`init ${JSON.stringify(init([500, 500, 0], partner))}`);
  if (partner) await ask('partner-start 0');
  const reply = await ask('step 1000000 0 0 0 0 1 1 1 1');
  child.kill();
  return reply.split(' ').length;
}

test('server: the step reply gains three fields only when a partner exists', async () => {
  const base = await stepReplyTokens(null);
  const withPartner = await stepReplyTokens(R2([[0, 2500, 2500, 0]]));
  assert.equal(withPartner, base + 3);
});
