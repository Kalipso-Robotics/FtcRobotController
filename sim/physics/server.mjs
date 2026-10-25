// Physics server. The Java runner starts this and talks to it one line at a time over stdin/stdout:
//   init <json>                                  -> "ok" and, when the field has balls, " id,type,radiusMm;id,type,radiusMm"
//   balls                                        -> "angleRed angleBlue id,xMm,yMm,zMm;..." the balls that moved since the last call, "-" if none
//   step <toNanos> <p0> <p1> <p2> <p3> <b0> <b1> <b2> <b3> [<intakePower> <stopper> <flywheelRps> <hood> <turretRad>]
//                                                -> "x y theta leftPodMm rightPodMm backPodMm fLeftWheelMm held slots events"
//                                                   events is "-" or "name@nanos;name@nanos" (contacts that began)
//   quit
// One reply per request, in order. Java waits for each reply, so the two stay in lockstep.

import readline from 'node:readline';
import { createWorld } from './world.mjs';

let world = null;
const lines = readline.createInterface({ input: process.stdin });

// readline lines arrive in order, but createWorld is async, so requests are chained.
let queue = Promise.resolve();
lines.on('line', (line) => {
  queue = queue.then(() => handle(line)).catch((error) => {
    process.stdout.write(`error ${String(error && error.stack || error).replace(/\n/g, ' | ')}\n`);
  });
});

async function handle(line) {
  const space = line.indexOf(' ');
  const command = space < 0 ? line : line.slice(0, space);
  const rest = space < 0 ? '' : line.slice(space + 1);

  if (command === 'init') {
    if (world) world.free();
    world = await createWorld(JSON.parse(rest));
    const info = world.ballInfo().map((b) => b.join(',')).join(';');
    process.stdout.write(info ? `ok ${info}\n` : 'ok\n');
  } else if (command === 'balls') {
    const angles = world.hiveAngles();
    const moved = world.ballsMoved().map((b) => b.join(',')).join(';');
    process.stdout.write(`${(angles[0] ?? 0).toFixed(5)} ${(angles[1] ?? 0).toFixed(5)} ${moved || '-'}\n`);
  } else if (command === 'step') {
    const v = rest.split(' ').map(Number);
    const actuators = v.length >= 14 ? { intake: v[9], stopper: v[10], rps: v[11], hood: v[12], turret: v[13] } : undefined;
    const s = world.advanceTo(v[0], v.slice(1, 5), v.slice(5, 9).map((b) => b !== 0), actuators);
    // spaces in an event name become ~ so the reply stays one token per field
    const events = s.events.length ? s.events.map((e) => `${e.name.replace(/ /g, '~')}@${e.nanos}`).join(';') : '-';
    process.stdout.write(`${s.x} ${s.y} ${s.theta} ${s.leftPodMm} ${s.rightPodMm} ${s.backPodMm} ${s.fLeftWheelMm} ${s.held} ${s.slots} ${events}\n`);
  } else if (command === 'quit') {
    process.exit(0);
  } else {
    throw new Error(`unknown command: ${command}`);
  }
}
