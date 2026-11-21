#!/usr/bin/env node
// Batch runner: one sim run per seed x scenario x speed, in a worker pool; writes run files, debriefs and scores.json.
// usage: node sim/opt/batch.mjs --auto RedDecideAuto [--params f.json] [--seeds N] [--jobs N] [--out dir] [--timeout s] [--best run.csv]
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { pathToFileURL } from 'node:url';
import { ROOT, JAVA_HOME, OPMODES, parseRun, simArgs } from '../app/runfile.mjs';
import { objective, aggregate } from './objective.mjs';
import { debrief, summarize } from './debrief.mjs';

const PATHS_DIR = path.join(ROOT, 'sim', 'partners', 'routes');
const DEFENDERS_DIR = path.join(ROOT, 'sim', 'partners', 'defenders');

/** An opmodes.json entry's params: an object, or the path (from the repo root) of its autoparams JSON file. */
export function opParams(op) {
  return typeof op.params === 'string' ? JSON.parse(fs.readFileSync(path.join(ROOT, op.params), 'utf8')) : op.params || {};
}

export function parseArgs(argv) {
  const a = { seeds: 1, jobs: Math.max(1, Math.floor(os.cpus().length / 2)), timeout: 120, out: null, params: null, best: null, auto: null };
  for (let i = 0; i < argv.length; i += 2) {
    const k = argv[i].replace(/^--/, '');
    if (!(k in a)) throw new Error(`unknown option ${argv[i]}`);
    a[k] = ['seeds', 'jobs', 'timeout'].includes(k) ? Number(argv[i + 1]) : argv[i + 1];
  }
  if (!a.auto) throw new Error('--auto <OpMode name> is required');
  return a;
}

/** Placement error for a seed: +-15 mm in x and y, +-1 degree of heading. mulberry32, so the same seed always gives the same error. */
export function jitter(seed) {
  let s = seed >>> 0;
  const next = () => { s = (s + 0x6d2b79f5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const u = () => next() * 2 - 1;
  return { dx: u() * 15, dy: u() * 15, dTheta: (u() * Math.PI) / 180 };
}

/** The runs of one OpMode: scenarios x speeds x seeds 1..seeds, each with its jittered start pose. Scenario order, then speed, then seed. */
export function expandRuns(op, seeds) {
  const runs = [];
  for (const sc of op.scenarios || [{ partner: null, speeds: [1] }]) {
    for (const speed of sc.speeds || [1]) {
      for (let seed = 1; seed <= seeds; seed++) {
        const j = jitter(seed);
        const label = (sc.partner ? `${sc.partner}${(sc.speeds || [1]).length > 1 ? `@${speed}` : ''}` : 'solo') + (sc.defenders ? `+${sc.defenders}` : '');
        runs.push({ name: `${label}-s${seed}`, seed, partner: sc.partner || null, defenders: sc.defenders || null, speed,
          x: op.start.x + j.dx, y: op.start.y + j.dy, theta: (op.start.headingDeg * Math.PI) / 180 + j.dTheta });
      }
    }
  }
  return runs;
}

/** scores.json content. Runs sorted by name so the file does not depend on which worker finished first. */
export function buildScores(opMode, rows) {
  const runs = {};
  for (const r of [...rows].sort((p, q) => (p.name < q.name ? -1 : 1))) runs[r.name] = r.result;
  return { opMode, runs, aggregate: aggregate(rows.map((r) => r.result)) };
}

function exec(cmd, args, opts, timeoutS) {
  return new Promise((resolve) => {
    const child = spawn(cmd, args, opts);
    let output = '', timedOut = false;
    const timer = timeoutS ? setTimeout(() => { timedOut = true; child.kill('SIGKILL'); }, timeoutS * 1000) : null;
    child.stdout.on('data', (d) => { output += d; });
    child.stderr.on('data', (d) => { output += d; });
    child.on('error', (e) => { clearTimeout(timer); resolve({ code: -1, output: String(e), timedOut }); });
    child.on('close', (code) => { clearTimeout(timer); resolve({ code, output, timedOut }); });
  });
}

async function runOne(op, r, a, classpath, outDir, best, baseParams) {
  const dir = path.join(outDir, r.name);
  fs.mkdirSync(dir, { recursive: true });
  const csv = path.join(dir, 'run.csv');
  let autoParams;
  if (Object.keys(baseParams).length) { autoParams = path.join(dir, 'params.json'); fs.writeFileSync(autoParams, JSON.stringify(baseParams, null, 2)); }
  const args = [...simArgs({ seed: r.seed, logDir: dir, autoParams, partnerPath: r.partner && path.join(PATHS_DIR, `${r.partner}.json`), partnerSpeed: r.speed, defenderPath: r.defenders && path.join(DEFENDERS_DIR, `${r.defenders}.json`) }),
    '-cp', classpath, 'org.firstinspires.ftc.teamcode.kalipsorobotics.sim.SimMain', op.className, String(r.x), String(r.y), String(r.theta), csv, 'rapier'];
  const sim = await exec(path.join(JAVA_HOME, 'bin', 'java'), args, { cwd: path.join(ROOT, 'TeamCode') }, a.timeout);
  fs.writeFileSync(path.join(dir, 'java.log'), sim.output);
  if (sim.timedOut) return { name: r.name, result: { error: `timeout after ${a.timeout} s` } };
  if (!fs.existsSync(csv)) return { name: r.name, result: { error: `no run file (exit ${sim.code}): ${sim.output.split('\n').find((l) => l.trim() && !l.startsWith('WARNING')) || ''}` } };
  const run = parseRun(fs.readFileSync(csv, 'utf8'), op);
  if (run.error) return { name: r.name, result: { error: run.error.split('\n')[0] } };
  const d = debrief(run, { title: r.name, best });
  fs.writeFileSync(path.join(dir, 'debrief.md'), d.md);
  fs.writeFileSync(path.join(dir, 'path.png'), d.png);
  return { name: r.name, result: objective(run) };
}

async function main() {
  const a = parseArgs(process.argv.slice(2));
  const op = OPMODES.find((o) => o.name === a.auto);
  if (!op) throw new Error(`unknown OpMode ${a.auto}; known: ${OPMODES.map((o) => o.name).join(', ')}`);
  const outDir = path.resolve(a.out || path.join(ROOT, 'TeamCode', 'build', 'sim-runs', `batch-${Date.now()}`));
  fs.mkdirSync(outDir, { recursive: true });
  const build = await exec(path.join(ROOT, 'gradlew'), [':TeamCode:simClasspath', '--offline', '-q'], { cwd: ROOT, env: { ...process.env, JAVA_HOME } });
  if (build.code !== 0) throw new Error(`gradle simClasspath failed:\n${build.output}`);
  const classpath = fs.readFileSync(path.join(ROOT, 'TeamCode', 'build', 'sim-classpath.txt'), 'utf8');
  const baseParams = { ...opParams(op), ...(a.params ? JSON.parse(fs.readFileSync(a.params, 'utf8')) : {}) };
  const best = a.best ? parseRun(fs.readFileSync(a.best, 'utf8'), op) : null;
  const queue = expandRuns(op, a.seeds), rows = [];
  await Promise.all(Array.from({ length: Math.min(a.jobs, queue.length) }, async () => {
    for (let r; (r = queue.shift()); ) {
      const row = await runOne(op, r, a, classpath, outDir, best, baseParams).catch((e) => ({ name: r.name, result: { error: String(e.message) } }));
      rows.push(row);
      console.log(`${row.name}: ${row.result.error ? `ERROR ${row.result.error}` : row.result.value.toFixed(2)}`);
    }
  }));
  const scores = buildScores(a.auto, rows);
  fs.writeFileSync(path.join(outDir, 'scores.json'), JSON.stringify(scores, null, 2) + '\n');
  fs.writeFileSync(path.join(outDir, 'summary.md'), summarize([...rows].sort((p, q) => (p.name < q.name ? -1 : 1)), `Batch: ${a.auto}`));
  console.log(`wrote ${path.join(outDir, 'scores.json')}  mean ${scores.aggregate.mean}  errors ${scores.aggregate.errors}`);
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) main().catch((e) => { console.error(e.message); process.exit(1); });
