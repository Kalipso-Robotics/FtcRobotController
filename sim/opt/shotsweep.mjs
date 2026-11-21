#!/usr/bin/env node
// Shot sweep: tune shooting and SOTM in the sim. Runs ShotSweepAuto (one ball per run) for standing shots at several distances
// and for passes at each speed and direction past launch point A, with SOTM off and on. Prints hit % and the mean miss per
// condition, fits flight time = A + B * distance from the standing shots, and writes everything to sim/opt/out/shotsweep.json.
//
// usage: node sim/opt/shotsweep.mjs [--speeds 500,1000] [--jobs 4] [--set SOTMConfig.RELEASE_LATENCY_MS=80 ...] [--out file]
// Every --set is a decode.configs tunable applied before the run (what the Dashboard would change on the robot).
//
// --wide 1: run widePlan() (standing + crossing passes at wide bearings) instead of plan(); prints hit % per bearing, writes --out.
//
// --table 1 [--latency 1] [--validate 1]: draft a shooter table from standing shots over an RPS x hood grid (coarse, then refined around
// the hit band), refit the SOTM flight time from the chosen shots, optionally sweep RELEASE_LATENCY_MS on moving passes, and rerun the
// standard sweep with the drafted table vs the current one. Writes sim/opt/out/shooter-table.json. Never touches the Java config.
// --grid 1 [--cell A|B] [--forwards 600,...] [--sides 0,...] [--hoodDegs 60,65,70] [--rpsMin 24 --rpsStep 4]: the 2D shooter grid in the Cell frame (forward = away from the
// opening, side along +x; B is mirrored). Per point: coarse rps x hood-deg sweep, refine, pickCell, then runs at +-1 RPS / +-1.5 deg (any miss = weak). Off-field and
// Hive/flower-blocked points are dropped. Writes out/shooter-grid.json, shooter-grid.java.txt (draft arrays) and shooter-grid-cache.json (rerun resumes from it).
// SWEEP_WRAP=<script> runs the gradle build and a snapshot copy of the compiled classes through that wrapper (a build lock); the sim runs
// then use the snapshot, so parallel agents rebuilding the tree can't break or slow the sweep.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { pathToFileURL } from 'node:url';
import { ROOT, JAVA_HOME, simArgs } from '../app/runfile.mjs';

const CLASS = 'org.firstinspires.ftc.teamcode.kalipsorobotics.simdemo.ShotSweepAuto';
const TARGET = { x: 1524, y: 3657.6 - 1524 };        // FieldConfig.aTargetPoint
const LAUNCH = { x: 1524, y: 3657.6 - 609 };         // FieldConfig.aLaunchPoint
const MAX_Y = 3657.6 - 252 - 5;                       // the robot's back (RobotFootprint.BACK_MM) stays off the wall

/** Keep runs whose name matches the regex (all when empty). */
export const onlyRuns = (runs, re) => (re ? runs.filter((r) => new RegExp(re).test(r.name)) : runs);

/** The runs: standing shots at distances along the line to the target, then passes. Pure, for the test. */
export function plan(speeds = [500, 1000]) {
  const runs = [];
  for (const d of [900, 1050, 1200]) {
    const y = Math.min(TARGET.y + d, MAX_Y);
    runs.push({ name: `stand-${Math.round(y - TARGET.y)}`, kind: 'stand', sotm: false, start: { x: TARGET.x, y }, v: { x: 0, y: 0 } });
  }
  for (const d of [1300, 1500, 1700]) {   // off to the side, so longer distances fit inside the field
    const x = TARGET.x - Math.sqrt(d * d - 1200 * 1200), y = TARGET.y + 1200;
    runs.push({ name: `stand-side-${d}`, kind: 'stand', sotm: false, start: { x, y }, v: { x: 0, y: 0 } });
  }
  const dirs = { toward: [0, -1], away: [0, 1], left: [1, 0], right: [-1, 0] };   // as the robot faces the Cell (-y): +x is its left
  for (const speed of speeds) {
    for (const [dir, [ux, uy]] of Object.entries(dirs)) {
      const runup = 350 + speed * 0.2;
      let start = { x: LAUNCH.x - ux * runup, y: LAUNCH.y - uy * runup };
      if (start.y > MAX_Y) start = { x: start.x, y: MAX_Y };   // toward: as much run-up as the wall allows
      for (const sotm of [false, true]) {
        runs.push({ name: `${dir}-${speed}-${sotm ? 'sotm' : 'plain'}`, kind: 'pass', dir, speed, sotm, start, v: { x: ux * speed, y: uy * speed }, release: LAUNCH });
      }
    }
  }
  return runs;
}

const RAD = Math.PI / 180;
const inField = (p) => p.x >= 252 && p.x <= 3405 && p.y <= MAX_Y;

/** Wide-angle runs: standing shots at bearings off the Cell's centre line (+y on red), both sides, then crossing passes at 45 and 70 deg. */
export function widePlan(speeds = [500, 1000]) {
  const runs = [];
  const side = (sgn, b) => (b === 0 ? 'C' : sgn > 0 ? 'L' : 'R');
  for (const bearing of [0, 20, 40, 55, 70]) for (const sgn of bearing === 0 ? [1] : [1, -1]) for (const d of [1000, 1500, 2000, 2500]) {
    const start = { x: TARGET.x + sgn * d * Math.sin(bearing * RAD), y: TARGET.y + d * Math.cos(bearing * RAD) };
    if (inField(start)) runs.push({ name: `wide-b${bearing}-${bearing === 0 ? 'L' : side(sgn, bearing)}-${d}`, kind: 'stand', bearing, side: side(sgn, bearing), d, sotm: false, start, v: { x: 0, y: 0 } });
  }
  for (const bearing of [45, 70]) for (const sgn of [1, -1]) for (const speed of speeds) {
    const release = { x: TARGET.x + sgn * 1500 * Math.sin(bearing * RAD), y: TARGET.y + 1500 * Math.cos(bearing * RAD) };
    const runup = 350 + speed * 0.2;
    for (const dirSgn of [1, -1]) {   // crossing direction (perpendicular to the line to the target); take the first one with room for the run-up
      const u = { x: dirSgn * sgn * Math.cos(bearing * RAD), y: -dirSgn * Math.sin(bearing * RAD) };
      const start = { x: release.x - u.x * runup, y: release.y - u.y * runup };
      if (!inField(release) || !inField(start)) continue;
      for (const sotm of [false, true]) {
        runs.push({ name: `wide-pass-b${bearing}-${side(sgn, bearing)}-${speed}-${sotm ? 'sotm' : 'plain'}`, kind: 'pass', bearing, side: side(sgn, bearing), speed, sotm, start, v: { x: u.x * speed, y: u.y * speed }, release });
      }
      break;
    }
  }
  return runs;
}

/** Hit % per bearing, standing and moving separately. */
export function bearingTable(done) {
  const out = { stand: {}, pass: {} };
  for (const kind of ['stand', 'pass']) {
    for (const b of [...new Set(done.filter((r) => r.kind === kind).map((r) => r.bearing))].sort((a, c) => a - c)) out[kind][b] = hitPct(done.filter((r) => r.kind === kind && r.bearing === b));
  }
  return out;
}


// ---- grid mode (--grid 1): pure helpers ----
const TARGETS = { A: { x: 1524, y: 3657.6 - 1524, facing: 1 }, B: { x: 1524, y: 3657.6 - 2135, facing: -1 } };   // FieldConfig a/bTargetPoint; the Cell opens toward +y (A) or -y (B)
const HIVE_LEGS = [[1200.8, 1334.4, 1351.6, 2322.9], [2305.6, 1334.4, 2456.4, 2322.9]];   // FieldConfig.HIVE_LEG_LOW_X / HIGH_X {minX,minY,maxX,maxY}
const FLOWERS = [[63.5, 2438.4], [1219.2, 63.5], [3594.1, 1219.2], [2438.4, 3594.1]];   // FieldConfig a..d FlowerPoint, 127 mm squares
const ROBOT_R = 260;   // circle that covers the robot's footprint
const hitsBox = (p, [x0, y0, x1, y1]) => Math.hypot(p.x - Math.min(Math.max(p.x, x0), x1), p.y - Math.min(Math.max(p.y, y0), y1)) < ROBOT_R;
export const blocked = (p) => HIVE_LEGS.some((b) => hitsBox(p, b)) || FLOWERS.some(([x, y]) => hitsBox(p, [x - 63.5, y - 63.5, x + 63.5, y + 63.5]));

/** Field position of a Cell-frame point: forward = away from the Cell opening, side = along +x. B is A mirrored through its own target. */
export function cellPoint(cell, forward, side) {
  const t = TARGETS[cell];
  return { x: t.x + side, y: t.y + t.facing * forward };
}

/** Grid points {cell, forward, side, x, y} (forward-major); off-field and blocked ones are dropped. Mirror for B is in cellPoint. */
export function gridPoints(cell, forwards, sides) {
  const pts = [];
  for (const forward of forwards) for (const side of sides) {
    const p = cellPoint(cell, forward, side), f = TARGETS[cell].facing;
    const onField = p.x >= 252 && p.x <= 3405 && (f > 0 ? p.y <= MAX_Y : p.y >= 3657.6 - MAX_Y);
    if (onField && !blocked(p)) pts.push({ cell, forward, side, ...p });
  }
  return pts;
}

/** Standing-shot runs: one per (point, rps, hood deg). The robot faces its Cell (heading -90 for A, +90 for B). */
export function gridPlan(points, rpsList, hoodDegList) {
  const runs = [];
  for (const p of points) for (const rps of rpsList) for (const hoodDeg of hoodDegList) {
    runs.push({ name: `grid-${p.cell}-f${p.forward}-s${p.side}-r${rps}-d${hoodDeg}`, kind: 'grid', cell: p.cell, forward: p.forward, side: p.side, rps, hoodDeg,
      sotm: false, start: { x: p.x, y: p.y }, v: { x: 0, y: 0 } });
  }
  return runs;
}

/** The consistency neighbours of a pick: rps +-1 and hood +-1.5 deg (a hood past the range is skipped, HoodMap clamps it to the same shot). */
export function weakCheck(pick, lowDeg = 60, highDeg = 70) {
  const out = [{ rps: pick.rps - 1, hoodDeg: pick.hoodDeg }, { rps: pick.rps + 1, hoodDeg: pick.hoodDeg }];
  for (const d of [pick.hoodDeg - 1.5, pick.hoodDeg + 1.5]) if (d >= lowDeg && d <= highDeg) out.push({ rps: pick.rps, hoodDeg: d });
  return out;
}

/** Paste-ready static final arrays (FORWARD_MM, SIDE_MM, RPS[][], HOOD_DEG[][]; rows = forward, columns = side). */
export function javaGrid(forwards, sides, rps, hoodDeg) {
  const mat = (m) => `{\n${m.map((r) => `            {${r.join(', ')}},`).join('\n')}\n    }`;
  return `// DRAFT from sim/opt/shotsweep.mjs --grid 1. Sim baseline, UNMEASURED until robot-checked. Rows = FORWARD_MM, columns = SIDE_MM.\n` +
    `static final double[] FORWARD_MM = {${forwards.join(', ')}};\nstatic final double[] SIDE_MM = {${sides.join(', ')}};\n` +
    `static final double[][] RPS = ${mat(rps)};\nstatic final double[][] HOOD_DEG = ${mat(hoodDeg)};\n`;
}

// ---- table mode (pure helpers first, for the test) ----

// ShooterInterpolationConfig.shooterData as final numbers {distanceMM, rps, hood} (rpsTune 0.25, compensatedMaxHood 0.295 already added).
// Hand-copied so the diff has something to compare with; re-copy it if the config changes.
export const CURRENT_TABLE = [[810, 45.65, 0.1284], [1050, 45.65, 0.21], [1310, 47.95, 0.21], [1518, 49.85, 0.245], [1705, 51.85, 0.245],
  [1898, 53.05, 0.275], [2075, 54.95, 0.295], [2244, 56.85, 0.295], [2417, 57.85, 0.295], [2561, 60.25, 0.295], [2769, 62.05, 0.295],
  [2974, 63.95, 0.295], [3154, 65.25, 0.295], [3240, 66.65, 0.295], [3460, 67.75, 0.295], [3700, 70.45, 0.295], [4202, 73.45, 0.295]];
export const MIN_HOOD = 0.1284, MAX_HOOD = 0.3, MAX_RPS = 73;

const round = (v, n) => Number(v.toFixed(n));

/** Where to stand for shooting distance d (robot centre to Cell A target). Straight back first, then out to the right; null if it won't fit. */
export function standStart(d, bearingDeg) {
  if (bearingDeg !== undefined) {   // on a given bearing off the Cell's centre line (+x side); null if off the field
    const start = { x: TARGET.x + d * Math.sin(bearingDeg * RAD), y: TARGET.y + d * Math.cos(bearingDeg * RAD) };
    return inField(start) ? start : null;
  }
  const dy = Math.min(d, MAX_Y - TARGET.y - 5), dx = Math.sqrt(d * d - dy * dy);
  const start = { x: TARGET.x + dx, y: TARGET.y + dy };
  return start.x <= 3357 ? start : null;
}

/** Standing-shot runs: one per (distance, rps, hood). Distances that don't fit on the field are dropped. */
export function tablePlan(distances, rpsList, hoodList, bearing) {
  const runs = [];
  for (const d of distances) {
    const start = standStart(d, bearing);
    if (!start) continue;
    for (const rps of rpsList) for (const hood of hoodList) {
      runs.push({ name: `tbl-${d}-r${rps}-h${hood}`, kind: 'table', d, rps, hood, sotm: false, start, v: { x: 0, y: 0 } });
    }
  }
  return runs;
}

/** rows [{d,rps,hood}] sorted by d: lifts any rps dip to the previous row's rps (a longer shot never needs less flywheel speed). */
export function enforceMonotonic(rows) {
  let top = -Infinity;
  return rows.map((r) => { top = Math.max(top, r.rps); return { ...r, rps: top }; });
}

/** Linear interpolation in a [[d,...],...] table (clamped at both ends). */
export function interp(table, d) {
  if (d <= table[0][0]) return table[0].slice(1);
  for (let i = 1; i < table.length; i++) {
    if (d <= table[i][0]) {
      const t = (d - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
      return [1, 2].map((k) => table[i - 1][k] + t * (table[i][k] - table[i - 1][k]));
    }
  }
  return table.at(-1).slice(1);
}

/**
 * The in-cell with the most margin. cells: [{rps, hood, in, long, left}] (long null = shot never got there). Takes the largest 8-connected
 * in-region (cells are neighbours when within 1.5 scaled units, so diagonal coarse cells join), then inside it the cell farthest from any
 * miss (distance in scaled units: rpsScale RPS or hoodScale hood = 1; defaults are the coarse grid steps: 4 RPS, 0.0858 hood), ties broken by the larger mean distance to all misses (centres it), then the smaller |long|,|left|. With no miss at all, just
 * the smallest |long|,|left|. Returns {rps, hood, margin, region} or null when nothing went in.
 */
export function pickCell(cells, { rpsScale = 4, hoodScale = 0.0858 } = {}) {
  const ins = cells.filter((c) => c.in), misses = cells.filter((c) => !c.in);
  if (!ins.length) return null;
  const dist = (a, b) => Math.hypot((a.rps - b.rps) / rpsScale, (a.hood - b.hood) / hoodScale);
  const seen = new Set(), regions = [];
  for (const c of ins) {
    if (seen.has(c)) continue;
    const region = [], stack = [c];
    seen.add(c);
    while (stack.length) {
      const x = stack.pop(); region.push(x);
      for (const y of ins) if (!seen.has(y) && dist(x, y) <= 1.5) { seen.add(y); stack.push(y); }
    }
    regions.push(region);
  }
  const best = regions.reduce((a, b) => (b.length > a.length ? b : a));
  const miss = (c) => Math.hypot(c.long ?? 0, c.left ?? 0);
  let pick = null;
  for (const c of best) {
    const ds = misses.map((m) => dist(c, m));
    const margin = ds.length ? Math.min(...ds) : 0, mean = ds.length ? ds.reduce((a, b) => a + b, 0) / ds.length : 0;
    const better = !pick || margin > pick.margin + 1e-9
      || (Math.abs(margin - pick.margin) <= 1e-9 && (mean > pick.mean + 1e-9 || (Math.abs(mean - pick.mean) <= 1e-9 && miss(c) < miss(pick.c))));
    if (better) pick = { c, margin, mean };
  }
  return { rps: pick.c.rps, hood: pick.c.hood, margin: round(pick.margin, 3), region: best.length };
}

/** New grid lines to run next: the midpoints between the coarse lines, within one coarse step of the in-cells (or of the nearest-to-in cell). */
export function refineAxes(cells, coarseRps, coarseHood) {
  let focus = cells.filter((c) => c.in);
  if (!focus.length) {   // no hit: refine around the cell that missed by least
    const near = cells.filter((c) => c.long !== null).sort((a, b) => Math.hypot(a.long, a.left) - Math.hypot(b.long, b.left))[0];
    focus = near ? [near] : [];
  }
  if (!focus.length) return { rps: [], hood: [] };
  const mids = (axis, key) => {
    const lo = Math.min(...focus.map((c) => c[key])), hi = Math.max(...focus.map((c) => c[key]));
    const step = axis.length > 1 ? axis[1] - axis[0] : 1;
    const out = [];
    for (let i = 0; i < axis.length - 1; i++) {
      const m = (axis[i] + axis[i + 1]) / 2;
      if (m >= lo - step && m <= hi + step) out.push(round(m, 4));
    }
    return out;
  };
  return { rps: mids(coarseRps, 'rps'), hood: mids(coarseHood, 'hood') };
}

/** Table rows vs the current table interpolated at the same distance. */
export function diffVsCurrent(rows, current = CURRENT_TABLE) {
  return rows.map(([d, rps, hood]) => {
    const [cr, ch] = interp(current, d);
    return { d, rps, hood, currentRps: round(cr, 2), currentHood: round(ch, 4), dRps: round(rps - cr, 2), dHood: round(hood - ch, 4) };
  });
}

/** The paste-ready double[] text, in the ShooterInterpolationConfig convention: final numbers, no rpsTune / compensatedMaxHood symbols. */
export function javaArray(rows) {
  const lines = rows.map(([d, rps, hood]) => `            ${d}, ${rps}, ${hood},`);
  return `// DRAFT from sim/opt/shotsweep.mjs --table. Values are FINAL: rpsTune (+0.25) and compensatedMaxHood (${round(MAX_HOOD - 0.005, 3)}) are already folded in,\n` +
    `// and hood is the servo position ShooterRun writes (HOOD_INIT_OFFSET is 0). Sim constants are unmeasured - a starting point only.\n` +
    `public static double[] shooterData = {\n${lines.join('\n')}\n};`;
}

/** "result: shot 1 in, long -10 mm, left 19 mm, flight 333 ms, dist 819 mm, robot 0 mm/s" -> object. Null if not a result line. */
export function parseResult(text) {
  const m = /result: shot \d+ (in|miss), (?:long (-?\d+) mm, left (-?\d+) mm, flight (\d+) ms|short[^,]*), dist (\d+) mm, robot (\d+) mm\/s/.exec(text);
  if (!m) return null;
  return { in: m[1] === 'in', long: m[2] === undefined ? null : Number(m[2]), left: m[3] === undefined ? null : Number(m[3]),
    flightMs: m[4] === undefined ? null : Number(m[4]), dist: Number(m[5]), robotMmS: Number(m[6]) };
}

/** Least squares t = A + B d. */
export function fitLine(points) {
  const n = points.length;
  if (n < 2) return null;
  const mx = points.reduce((s, p) => s + p[0], 0) / n, my = points.reduce((s, p) => s + p[1], 0) / n;
  const sxx = points.reduce((s, p) => s + (p[0] - mx) ** 2, 0);
  if (sxx === 0) return null;
  const b = points.reduce((s, p) => s + (p[0] - mx) * (p[1] - my), 0) / sxx;
  return { a: my - b * mx, b };
}

let runCount = 0;
function runOne(classpath, run, baseSets, outDir, tableProp) {
  runCount++;
  const sets = [...baseSets, ...(run.sets || [])];
  const outFile = path.join(outDir, `${run.name}.csv`);
  fs.rmSync(outFile, { force: true });   // a failed run must not report the last run's result
  const props = [`-Dsweep.startX=${run.start.x}`, `-Dsweep.startY=${run.start.y}`, `-Dsweep.vx=${run.v.x}`, `-Dsweep.vy=${run.v.y}`,
    `-Dsweep.sotm=${run.sotm}`, ...sets.map((s) => `-Dsweep.set.${s}`)];
  if (run.rps !== undefined) props.push(`-Dsweep.rps=${run.rps}`);
  if (run.hood !== undefined) props.push(`-Dsweep.hood=${run.hood}`);
  if (run.hoodDeg !== undefined) props.push(`-Dsweep.hoodDeg=${run.hoodDeg}`);
  if (run.cell) props.push(`-Dsweep.cell=${run.cell}`);
  if (run.table || tableProp) props.push(`-Dsweep.table=${run.table || tableProp}`);
  if (run.release) props.push(`-Dsweep.releaseX=${run.release.x}`, `-Dsweep.releaseY=${run.release.y}`);
  return new Promise((resolve) => {
    const javaArgs = [...simArgs({ params: ['-Dsim.physics.preload=1'] }), ...props, '-cp', classpath,
      'org.firstinspires.ftc.teamcode.kalipsorobotics.sim.SimMain', CLASS, String(run.start.x), String(run.start.y), String(run.cell === 'B' ? Math.PI / 2 : -Math.PI / 2), outFile, 'rapier'];
    const logFd = fs.openSync(outFile + '.log', 'w');
    const stdio = ['ignore', 'ignore', logFd];
    const java = path.join(JAVA_HOME, 'bin', 'java'), cwd = path.join(ROOT, 'TeamCode');
    const child = spawn(java, javaArgs, { cwd, stdio });
    fs.closeSync(logFd); // the child has its own copy; keeping ours leaks one fd per run (EMFILE at ulimit 256)
    child.on('close', () => {
      let text = '';
      try { text = fs.readFileSync(outFile, 'utf8'); } catch { /* no file: the run failed */ }
      const results = text.split('\n').filter((l) => l.startsWith('#event ')).map((l) => parseResult(l)).filter(Boolean);
      const error = text.split('\n').find((l) => l.startsWith('#error ')) || (text ? null : `no output file, see ${path.relative(ROOT, outFile)}.log`);
      resolve({ ...run, result: results[0] || null, error });
    });
  });
}

const fmt = (v, w = 6) => String(v ?? '–').padStart(w);

async function pool(queue, jobs, work) {
  const done = [];
  await Promise.all(Array.from({ length: Number(jobs) }, async () => {
    while (queue.length) { done.push(await work(queue.shift())); process.stderr.write('.'); }
  }));
  process.stderr.write('\n');
  return done;
}

const hitPct = (rs) => { const ok = rs.filter((r) => r.result); return { shots: ok.length, in: ok.filter((r) => r.result.in).length, pct: ok.length ? round(100 * ok.filter((r) => r.result.in).length / ok.length, 1) : null }; };

/** --table: coarse grid, refine, pick, SOTM refit, optional latency sweep and validation. */
async function tableMode(args, classpath, outDir) {
  const t0 = Date.now(), jobs = args.jobs;
  const distances = args.distances.split(',').map(Number);
  const bearing = args.bearing === '' ? undefined : Number(args.bearing);
  const run = (r) => runOne(classpath, r, args.set, outDir);
  const coarseRps = [], coarseHood = args.hoods.split(',').map(Number);
  for (let r = Number(args.rpsMin); r <= MAX_RPS; r += Number(args.rpsStep)) coarseRps.push(r);
  let all = await pool(tablePlan(distances, coarseRps, coarseHood, bearing), jobs, run);
  const seen = new Set(all.map((r) => r.name));
  const cellsOf = (d) => all.filter((r) => r.d === d && r.result).map((r) => ({ rps: r.rps, hood: r.hood, in: r.result.in, long: r.result.long, left: r.result.left, run: r }));
  const fine = [];
  for (const d of distances) {
    const ax = refineAxes(cellsOf(d), coarseRps, coarseHood);
    const rpsAll = [...new Set([...coarseRps, ...ax.rps])].sort((a, b) => a - b), hoodAll = [...new Set([...coarseHood, ...ax.hood])].sort((a, b) => a - b);
    for (const r of tablePlan([d], rpsAll, hoodAll, bearing)) if (!seen.has(r.name)) fine.push(r);
  }
  console.error(`coarse ${all.length} runs, refining with ${fine.length}`);
  all = all.concat(await pool(fine, jobs, run));

  const rows = [], notes = [], chosen = [];
  for (const d of distances) {
    if (!standStart(d, bearing)) { notes.push(`${d} mm: does not fit on the field (target is Cell A)`); continue; }
    const cells = cellsOf(d), pick = pickCell(cells);
    if (!pick) { notes.push(`${d} mm: no hit in ${cells.length} cells`); continue; }
    rows.push([d, round(pick.rps, 2), round(pick.hood, 4)]);
    const c = cells.find((x) => x.rps === pick.rps && x.hood === pick.hood);
    chosen.push({ d, ...pick, hits: cells.filter((x) => x.in).length, cells: cells.length, flightMs: c.run.result.flightMs, dist: c.run.result.dist });
  }
  enforceMonotonic(rows.map(([d, rps, hood]) => ({ d, rps, hood }))).forEach((r, i) => { rows[i][1] = r.rps; });
  // keep the current rows beyond what the sim could place, so the paste-ready array is a whole table
  const maxD = rows.length ? Math.max(...rows.map((r) => r[0])) : 0;
  const full = rows.concat(CURRENT_TABLE.filter((r) => r[0] > maxD + 100));
  const fit = fitLine(chosen.filter((c) => c.flightMs).map((c) => [c.dist, c.flightMs]));
  const out = { table: rows, javaArray: javaArray(full), notes: [...notes, `rows above ${maxD} mm are the CURRENT table's, unchanged: the sim cannot place a shot that far from Cell A`],
    diffVsCurrent: diffVsCurrent(rows), chosen, flightFit: fit && { FLIGHT_TIME_A_MS: round(fit.a, 1), FLIGHT_TIME_B_MS_PER_MM: round(fit.b, 4), from: chosen.length },
    caveat: 'sim constants unmeasured — starting point only' };
  console.log(`\ndistance  rps    hood    | now rps  hood   | hits/cells  margin`);
  for (const [i, r] of out.diffVsCurrent.entries()) console.log(`${fmt(r.d, 7)}${fmt(r.rps, 8)}${fmt(r.hood, 8)} |${fmt(r.currentRps, 8)}${fmt(r.currentHood, 8)} |${fmt(`${chosen[i].hits}/${chosen[i].cells}`, 9)}${fmt(chosen[i].margin, 8)}`);
  notes.forEach((n) => console.log(n));
  if (fit) console.log(`flight time refit: A ${out.flightFit.FLIGHT_TIME_A_MS} ms, B ${out.flightFit.FLIGHT_TIME_B_MS_PER_MM} ms/mm`);

  const standard = () => plan(args.speeds.split(',').map(Number));
  if (args.latency) {
    const a = out.flightFit ? [`SOTMConfig.FLIGHT_TIME_A_MS=${out.flightFit.FLIGHT_TIME_A_MS}`, `SOTMConfig.FLIGHT_TIME_B_MS_PER_MM=${out.flightFit.FLIGHT_TIME_B_MS_PER_MM}`] : [];
    const lat = [];
    for (const ms of [0, 25, 50, 75, 100, 150]) for (const r of standard().filter((p) => p.kind === 'pass' && p.sotm)) lat.push({ ...r, name: `lat${ms}-${r.name}`, latency: ms, sets: [`SOTMConfig.RELEASE_LATENCY_MS=${ms}`, ...a] });
    const done = await pool(lat, jobs, run);
    out.latency = [0, 25, 50, 75, 100, 150].map((ms) => {
      const rs = done.filter((r) => r.latency === ms && r.result);
      return { ms, ...hitPct(rs), meanMiss: round(rs.reduce((s, r) => s + Math.hypot(r.result.long ?? 1000, r.result.left ?? 0), 0) / (rs.length || 1), 0) };
    });
    out.bestLatencyMs = out.latency.slice().sort((x, y) => y.in - x.in || x.meanMiss - y.meanMiss)[0].ms;
    console.log('\nRELEASE_LATENCY_MS sweep (SOTM on, refit flight time):'); out.latency.forEach((l) => console.log(`  ${fmt(l.ms, 4)} ms: ${l.in}/${l.shots} in, mean miss ${l.meanMiss} mm`));
    console.log(`  best: ${out.bestLatencyMs} ms`);
  }
  if (args.validate) {
    const tableProp = full.flat().join(',');
    const comp = {};
    for (const [label, prop] of [['current', undefined], ['drafted', tableProp]]) {
      const done = await pool(standard().map((r) => ({ ...r, name: `val-${label}-${r.name}` })), jobs, (r) => runOne(classpath, r, args.set, outDir, prop));
      comp[label] = { all: hitPct(done), stand: hitPct(done.filter((r) => r.kind === 'stand')), pass: hitPct(done.filter((r) => r.kind === 'pass')), sotmOn: hitPct(done.filter((r) => r.sotm)) };
    }
    out.validation = comp;
    console.log('\nstandard sweep hit %:'); for (const [k, v] of Object.entries(comp)) console.log(`  ${k}: all ${v.all.in}/${v.all.shots} = ${v.all.pct}%  standing ${v.stand.in}/${v.stand.shots}  passes ${v.pass.in}/${v.pass.shots}`);
  }
  out.runs = runCount;
  out.seconds = round((Date.now() - t0) / 1000, 0);
  const file = path.join(path.dirname(args.out), 'shooter-table.json');
  fs.writeFileSync(file, JSON.stringify(out, null, 2));
  console.log(`\n${out.runs} sim runs in ${out.seconds} s; wrote ${path.relative(ROOT, file)}`);
}

/** --grid 1: per grid point, coarse rps x hood-deg sweep, refine, pickCell, then the +-1 RPS / +-1.5 deg consistency runs. Writes shooter-grid.json and .java.txt. */
async function gridMode(args, classpath, outDir) {
  const t0 = Date.now(), jobs = args.jobs, cell = args.cell;
  const nums = (v) => v.split(',').map(Number);
  const forwards = nums(args.forwards), sides = nums(args.sides), coarseHood = nums(args.hoodDegs);
  const coarseRps = [];
  for (let r = Number(args.rpsMin); r <= MAX_RPS; r += Number(args.rpsStep)) coarseRps.push(r);
  const points = gridPoints(cell, forwards, sides);
  const dropped = forwards.flatMap((f) => sides.map((s) => [f, s])).filter(([f, s]) => !points.some((p) => p.forward === f && p.side === s));
  const cachePath = path.join(path.dirname(args.out), 'shooter-grid-cache.json');   // results by run name, so a killed sweep resumes
  const cache = fs.existsSync(cachePath) ? JSON.parse(fs.readFileSync(cachePath, 'utf8')) : {};
  let sinceSave = 0;
  const run = async (r) => {
    if (cache[r.name]) return { ...r, ...cache[r.name] };
    const d = await runOne(classpath, r, args.set, outDir);
    if (d.result) { cache[r.name] = { result: d.result, error: null }; if (++sinceSave >= 25) { sinceSave = 0; fs.writeFileSync(cachePath, JSON.stringify(cache)); } }
    return d;
  };
  const key = (p) => `${p.forward}/${p.side}`;
  const all = new Map(points.map((p) => [key(p), []]));
  const runAll = async (runs) => { for (const d of await pool(runs, jobs, run)) all.get(`${d.forward}/${d.side}`).push(d); };
  const cellsOf = (p) => all.get(key(p)).filter((r) => r.result).map((r) => ({ rps: r.rps, hood: r.hoodDeg, in: r.result.in, long: r.result.long, left: r.result.left }));
  const seen = (p, rps, deg) => all.get(key(p)).some((r) => r.rps === rps && r.hoodDeg === deg);
  await runAll(gridPlan(points, coarseRps, coarseHood));
  const fine = [];
  for (const p of points) {
    const ax = refineAxes(cellsOf(p), coarseRps, coarseHood);
    const rpsAll = [...new Set([...coarseRps, ...ax.rps])], hoodAll = [...new Set([...coarseHood, ...ax.hood])];
    fine.push(...gridPlan([p], rpsAll, hoodAll).filter((r) => !seen(p, r.rps, r.hoodDeg)));
  }
  console.error(`coarse ${[...all.values()].flat().length} runs, refining with ${fine.length}`);
  await runAll(fine);
  const picks = new Map(points.map((p) => [key(p), pickCell(cellsOf(p), { rpsScale: Number(args.rpsStep), hoodScale: 5 })]));
  const checks = [];
  for (const p of points) {
    const pk = picks.get(key(p));
    if (pk) checks.push(...gridPlan([p], [pk.rps], [pk.hood]).filter((r) => !seen(p, r.rps, r.hoodDeg)),
      ...weakCheck({ rps: pk.rps, hoodDeg: pk.hood }, Math.min(...coarseHood), Math.max(...coarseHood)).flatMap((n) => gridPlan([p], [n.rps], [n.hoodDeg]).filter((r) => !seen(p, r.rps, r.hoodDeg))));
  }
  await runAll(checks);
  fs.writeFileSync(cachePath, JSON.stringify(cache));

  const report = points.map((p) => {
    const pk = picks.get(key(p)), cs = cellsOf(p);
    if (!pk) return { forward: p.forward, side: p.side, pick: null, weak: true, sweepIn: cs.filter((c) => c.in).length, sweepCells: cs.length };
    const nb = [{ rps: pk.rps, hoodDeg: pk.hood }, ...weakCheck({ rps: pk.rps, hoodDeg: pk.hood }, Math.min(...coarseHood), Math.max(...coarseHood))];
    const res = nb.map((n) => all.get(key(p)).find((r) => r.rps === n.rps && r.hoodDeg === n.hoodDeg)?.result?.in);
    return { forward: p.forward, side: p.side, rps: pk.rps, hoodDeg: pk.hood, margin: pk.margin, weak: res.some((x) => !x),
      inPct: round(100 * res.filter(Boolean).length / res.length, 0), sweepIn: cs.filter((c) => c.in).length, sweepCells: cs.length };
  });
  // matrices: rows = forward, columns = side. Dropped or no-hit points take the nearest kept point in the row (weak) so the Java array is whole.
  const good = report.filter((r) => r.rps !== undefined);
  const at = (f, s) => good.find((r) => r.forward === f && r.side === s);
  const fill = (key2) => forwards.map((f) => sides.map((s) => {
    const r = at(f, s) || good.filter((g) => g.forward === f).sort((a, b) => Math.abs(a.side - s) - Math.abs(b.side - s))[0] || good.sort((a, b) => Math.hypot(a.forward - f, a.side - s) - Math.hypot(b.forward - f, b.side - s))[0];
    return r[key2];
  }));
  const rowsKept = forwards.filter((f) => good.some((r) => r.forward === f));
  const keepRows = (m) => m.filter((_, i) => rowsKept.includes(forwards[i]));
  const out = { cell, forwards: rowsKept, sides, coarseRps, coarseHood, report, dropped, runs: runCount, seconds: round((Date.now() - t0) / 1000, 0),
    note: 'sim baseline, unmeasured until robot-checked; dropped points are filled from the nearest kept point in the Java draft' };
  fs.writeFileSync(path.join(path.dirname(args.out), 'shooter-grid.json'), JSON.stringify(out, null, 2));
  fs.writeFileSync(path.join(path.dirname(args.out), 'shooter-grid.java.txt'), javaGrid(rowsKept, sides, keepRows(fill('rps')), keepRows(fill('hoodDeg'))));
  console.log(`\nforward  side   rps  hood   in%  margin  sweep  weak`);
  for (const r of report) console.log(`${fmt(r.forward, 7)}${fmt(r.side, 6)}${fmt(r.rps, 6)}${fmt(r.hoodDeg, 6)}${fmt(r.inPct, 6)}${fmt(r.margin, 7)}${fmt(`${r.sweepIn}/${r.sweepCells}`, 8)}${r.weak ? '  WEAK' : ''}`);
  console.log(`\n${report.length} points kept, ${dropped.length} dropped (off field / blocked), ${report.filter((r) => r.weak).length} weak; ${runCount} sim runs in ${out.seconds} s`);
}

async function main() {
  const args = { speeds: '500,1000', jobs: String(Math.max(1, Math.floor(os.cpus().length / 2))), out: path.join(ROOT, 'sim', 'opt', 'out', 'shotsweep.json'), set: [],
    wide: '', only: '', bearing: '', useTable: '', table: '', latency: '', validate: '', distances: '810,1000,1200,1400,1600,1800,2000,2200', grid: '', cell: 'A', forwards: '600,900,1200,1500,1800,2100,2400,2700,3000', sides: '0,300,600,900,1200,1500,1800', hoodDegs: '60,65,70', rpsMin: '24', rpsStep: '4', hoods: '0.1284,0.2142,0.3' };
  const argv = process.argv.slice(2);
  for (let i = 0; i < argv.length; i += 2) {
    const k = argv[i].replace(/^--/, '');
    if (!(k in args)) throw new Error(`unknown option ${argv[i]}`);
    if (k === 'set') args.set.push(argv[i + 1]); else args[k] = argv[i + 1];   // Class.FIELD=value
  }
  const gradle = [path.join(ROOT, 'gradlew'), ':TeamCode:simClasspath', '--offline', '-q'];
  const build = process.env.SWEEP_WRAP ? spawn(process.env.SWEEP_WRAP, gradle, { env: { ...process.env, JAVA_HOME }, stdio: 'inherit' })
    : spawn(gradle[0], gradle.slice(1), { cwd: ROOT, env: { ...process.env, JAVA_HOME }, stdio: 'inherit' });
  if (await new Promise((r) => build.on('close', r)) !== 0) {
    if (!process.env.SWEEP_SKIP_BUILD_FAIL) process.exit(1);   // another agent's half-edited source can break the build; reuse the last compiled classes
    console.error('build failed, using the last compiled classes');
  }
  let classpath = fs.readFileSync(path.join(ROOT, 'TeamCode', 'build', 'sim-classpath.txt'), 'utf8');
  if (process.env.SWEEP_WRAP) {   // copy the build-dir entries once, under the lock
    const snap = path.join(path.dirname(path.resolve(args.out)), 'cp-snapshot');
    fs.rmSync(snap, { recursive: true, force: true });
    fs.mkdirSync(snap, { recursive: true });
    const buildDir = path.join(ROOT, 'TeamCode', 'build') + path.sep;
    classpath = await Promise.all(classpath.trim().split(path.delimiter).map(async (e, i) => {
      if (!e.startsWith(buildDir)) return e;
      const dest = path.join(snap, String(i));
      const cp = spawn(process.env.SWEEP_WRAP, ['cp', '-R', e, dest], { stdio: 'inherit' });
      await new Promise((r) => cp.on('close', r));
      return dest;
    })).then((l) => l.join(path.delimiter));
  }
  if (args.useTable) args.tableProp = JSON.parse(fs.readFileSync(args.useTable, 'utf8')).full.flat().join(',');   // proposed-table.json: run with that table, not the compiled one
  args.out = path.resolve(args.out);   // Java runs in TeamCode/, so a relative path would land in the wrong place
  const outDir = path.join(path.dirname(args.out), 'shotsweep-runs');
  fs.mkdirSync(outDir, { recursive: true });
  if (args.grid) return gridMode(args, classpath, outDir);
  if (args.table) return tableMode(args, classpath, outDir);

  if (args.wide) {
    const speeds = args.speeds.split(',').map(Number), order = widePlan(speeds).map((r) => r.name);
    const done = await pool(onlyRuns(widePlan(speeds), args.only), args.jobs, (r) => runOne(classpath, r, args.set, outDir, args.tableProp));
    done.sort((a, b) => order.indexOf(a.name) - order.indexOf(b.name));
    for (const r of done) console.log(`${r.name.padEnd(30)}${fmt(r.result ? (r.result.in ? 'IN' : '–') : 'ERR', 4)}${fmt(r.result?.long)}${fmt(r.result?.left)}${r.error ? `  ${r.error.slice(0, 80)}` : ''}`);
    const summary = bearingTable(done), movingOn = (on) => bearingTable(done.filter((r) => r.kind !== 'pass' || r.sotm === on)).pass;
    summary.passSotmOff = movingOn(false); summary.passSotmOn = movingOn(true);
    for (const [label, t] of [['standing', summary.stand], ['moving (SOTM off+on)', summary.pass], ['moving SOTM off', summary.passSotmOff], ['moving SOTM on', summary.passSotmOn]]) {
      console.log(`\nhit % per bearing, ${label}:`);
      for (const [b, h] of Object.entries(t)) console.log(`  ${String(b).padStart(2)} deg: ${h.in}/${h.shots} = ${h.pct}%`);
    }
    fs.writeFileSync(args.out, JSON.stringify({ sets: args.set, runs: done, summary }, null, 2));
    console.log(`\nwrote ${path.relative(ROOT, args.out)}`);
    return;
  }
  const queue = plan(args.speeds.split(',').map(Number));
  const done = await pool(queue, args.jobs, (r) => runOne(classpath, r, args.set, outDir, args.tableProp));
  done.sort((a, b) => plan().findIndex((p) => p.name === a.name) - plan().findIndex((p) => p.name === b.name));

  console.log(`${'run'.padEnd(22)}${fmt('in', 4)}${fmt('long')}${fmt('left')}${fmt('dist')}${fmt('fly ms')}${fmt('mm/s')}`);
  for (const r of done) {
    const x = r.result;
    console.log(`${r.name.padEnd(22)}${fmt(x ? (x.in ? 'IN' : '–') : 'ERR', 4)}${fmt(x?.long)}${fmt(x?.left)}${fmt(x?.dist)}${fmt(x?.flightMs)}${fmt(x?.robotMmS)}${r.error ? `  ${r.error.slice(0, 80)}` : ''}`);
  }
  const summary = {};
  for (const r of done.filter((d) => d.kind === 'pass' && d.result)) {
    const k = `${r.speed} mm/s ${r.sotm ? 'SOTM on ' : 'SOTM off'}`;
    const s = (summary[k] ||= { shots: 0, in: 0, absLong: 0, absLeft: 0 });
    s.shots++; s.in += r.result.in; s.absLong += Math.abs(r.result.long ?? 1000); s.absLeft += Math.abs(r.result.left ?? 0);
  }
  console.log('\nmoving shots (4 directions each):');
  for (const [k, s] of Object.entries(summary)) console.log(`  ${k}: ${s.in}/${s.shots} in, mean |long| ${(s.absLong / s.shots).toFixed(0)} mm, mean |left| ${(s.absLeft / s.shots).toFixed(0)} mm`);
  const fit = fitLine(done.filter((d) => d.kind === 'stand' && d.result?.flightMs).map((d) => [d.result.dist, d.result.flightMs]));
  if (fit) console.log(`\nflight time in the sim ball model: t = ${fit.a.toFixed(0)} + ${fit.b.toFixed(3)} * d ms  (SOTMConfig FLIGHT_TIME_A_MS / B_MS_PER_MM; only as true as the ball model)`);
  fs.writeFileSync(args.out, JSON.stringify({ sets: args.set, runs: done, summary, flightFit: fit }, null, 2));
  console.log(`\nwrote ${path.relative(ROOT, args.out)}`);
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) main();
