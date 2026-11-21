#!/usr/bin/env node
// Flywheel kp/kf/ks tune in the sim. Plant constants are UNMEASURED (SimHardware.FLYWHEEL_*). Never edits ShooterConfig;
// writes sim/opt/out/proposed-pidf.json. Usage: node sim/opt/pidf.mjs [--jobs 2] [--out file]
import fs from 'node:fs';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { ROOT, JAVA_HOME, simArgs } from '../app/runfile.mjs';
import { fitLine, standStart } from './shotsweep.mjs';

const AMP = 0.5;   // RPS: oscillation amplitude

/** power = ks + kv * rps from (power, rps) points. */
export function ffFit(points) {
  const f = fitLine(points.map(([p, r]) => [r, p]));
  return { ks: f.a, kv: f.b };
}

/** tr = [{t, rps}], shots = shot times. Recovery per shot = seconds until |error| stays within TOL (Infinity if it never does). */
export function score(tr, target, shots, TOL = 1) {
  const err = tr.map((p) => p.rps - target), dt = tr[1].t - tr[0].t;
  const overshoot = Math.max(0, ...err);
  const tail = err.filter((_, i) => tr[i].t >= tr[tr.length - 1].t - 0.5);
  const steadyErr = tail.reduce((s, e) => s + e, 0) / tail.length;
  const settle = tr.findIndex((p, i) => Math.abs(err[i]) <= TOL);
  const quiet = [];   // [from, to) index ranges away from the shot dips
  let from = settle;
  const recovery = shots.map((ts, k) => {
    const i0 = tr.findIndex((p) => p.t >= ts), i1 = tr.findIndex((p) => p.t >= (shots[k + 1] ?? Infinity));
    const end = i1 < 0 ? tr.length : i1;
    if (from >= 0) quiet.push([from, i0]);
    let last = -1;
    for (let i = i0; i < end; i++) if (Math.abs(err[i]) > TOL) last = i;
    if (last >= end - 1) { from = -1; return Infinity; }
    from = last < 0 ? i0 : last + 1;
    return last < 0 ? 0 : tr[last + 1].t - ts;
  });
  if (from >= 0) quiet.push([from, tr.length]);
  let flips = 0;   // sign changes of the error, ignoring |err| <= AMP
  for (const [a, b] of quiet) {
    let sign = 0;
    for (let i = a; i < b; i++) if (Math.abs(err[i]) > AMP) { const s = Math.sign(err[i]); if (sign && s !== sign) flips++; sign = s; }
  }
  return { overshoot, steadyErr, recovery, flips, oscillating: flips >= 2, settleT: settle < 0 ? null : tr[settle].t };
}

// ---- sim driver ----
const CLASS = 'org.firstinspires.ftc.teamcode.kalipsorobotics.simdemo.ShotSweepAuto';
const PLANT = { TOP: 90, KS: 0.04, TAU: 0.35 };
const TARGET_RPS = 60, HOOD = 0.2;

function run(classpath, name, gains, outDir) {
  const start = standStart(1500);
  const outFile = path.join(outDir, `${name}.csv`);
  fs.rmSync(outFile, { force: true });
  const sets = [`SimHardware.FLYWHEEL_TOP_RPS=${PLANT.TOP}`, `SimHardware.FLYWHEEL_KS_POWER=${PLANT.KS}`, `SimHardware.FLYWHEEL_TAU_S=${PLANT.TAU}`,
    ...Object.entries(gains).map(([k, v]) => `ShooterConfig.${k}=${v}`)];
  const props = [`-Dsweep.startX=${start.x}`, `-Dsweep.startY=${start.y}`, '-Dsweep.vx=0', '-Dsweep.vy=0', '-Dsweep.sotm=false',
    `-Dsweep.rps=${TARGET_RPS}`, `-Dsweep.hood=${HOOD}`, ...sets.map((s) => `-Dsweep.set.${s}`)];
  const args = [...simArgs({ params: ['-Dsim.physics.preload=1'] }), ...props, '-cp', classpath,
    'org.firstinspires.ftc.teamcode.kalipsorobotics.sim.SimMain', CLASS, String(start.x), String(start.y), String(-Math.PI / 2), outFile, 'rapier'];
  return new Promise((resolve) => {
    const fd = fs.openSync(outFile + '.log', 'w');
    const child = spawn(path.join(JAVA_HOME, 'bin', 'java'), args, { cwd: path.join(ROOT, 'TeamCode'), stdio: ['ignore', 'ignore', fd] });
    fs.closeSync(fd);
    child.on('close', () => resolve(readTrace(outFile)));
  });
}

/** CSV t,...,rps (last column); shot times come from '#event <matchSeconds>|shot ...' (+ #matchStart). */
export function readTrace(file) {
  const lines = fs.readFileSync(file, 'utf8').split('\n');
  const tr = lines.filter((l) => l && !l.startsWith('#')).map((l) => l.split(',')).filter((c) => c.length >= 10).map((c) => ({ t: Number(c[0]), rps: Number(c[9]) }));
  const shots = lines.filter((l) => l.startsWith('#event ') && /\|shot /.test(l)).map((l) => Number(l.slice(7).split('|')[0]));
  return { tr, shots };
}

async function main() {
  const argv = process.argv.slice(2), opt = (k, d) => (argv.includes(k) ? argv[argv.indexOf(k) + 1] : d);
  const jobs = Number(opt('--jobs', 2)), out = path.resolve(opt('--out', path.join(import.meta.dirname, 'out', 'proposed-pidf.json')));
  const classpath = fs.readFileSync(path.join(ROOT, 'TeamCode', 'build', 'sim-classpath.txt'), 'utf8').trim();
  const outDir = path.join(path.dirname(out), 'pidf-runs');
  fs.mkdirSync(outDir, { recursive: true });
  // kf is power per RPS (PIDFController velocity FF: kS*sign(sp) + kV*sp); plant power = KS + (1-KS)/TOP * rps
  const ff = { ks: PLANT.KS, kf: (1 - PLANT.KS) / PLANT.TOP };
  const cases = [['current', { kp: 0.085, kf: 0.01147578609, ks: 0 }],
    ...Array.from({ length: 14 }, (_, i) => { const kp = Number((0.02 + 0.01 * i).toFixed(2)); return [`kp${kp}`, { kp, ...ff }]; })];
  const res = {};
  const queue = [...cases];
  await Promise.all(Array.from({ length: jobs }, async () => {
    while (queue.length) {
      const [name, g] = queue.shift();
      const { tr, shots } = await run(classpath, `pidf-${name}`, g, outDir);
      res[name] = { gains: g, shots, ...(tr.length > 2 ? { ...score(tr, TARGET_RPS, shots), recoveryTight: score(tr, TARGET_RPS, shots, 0.25).recovery } : { error: 'no trace' }) };   // the dip is only ~1 RPS, so 0.25 RPS separates kps
      console.error(name, JSON.stringify(res[name]));
    }
  }));
  const ok = cases.slice(1).map(([n]) => res[n]).filter((r) => !r.error && !r.oscillating && r.overshoot < 1.5 && r.recovery.every(Number.isFinite));
  const best = ok.sort((a, b) => Math.max(...a.recovery, ...a.recoveryTight) - Math.max(...b.recovery, ...b.recoveryTight))[0];
  fs.writeFileSync(out, JSON.stringify({ kp: best?.gains.kp ?? null, kf: ff.kf, ks: ff.ks, plant: PLANT, targetRps: TARGET_RPS, scores: res, current: res.current }, null, 1));
  console.log(best ? `proposed kp ${best.gains.kp} kf ${ff.kf} ks ${ff.ks}` : 'no kp met the constraints');
}
if (import.meta.url === `file://${process.argv[1]}`) main();
