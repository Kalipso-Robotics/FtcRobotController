// Shared by the server and the batch runner: the run-file parser, the project constants and the JVM arguments for one run.
// Importing this file starts nothing.

import fs from 'node:fs';
import path from 'node:path';

export const ROOT = path.resolve(import.meta.dirname, '..', '..');
export const BALL_DEFAULTS = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'physics', 'ball-defaults.json'), 'utf8'));
export const JAVA_HOME = process.env.JAVA_HOME || '/Applications/Android Studio.app/Contents/jbr/Contents/Home';
const ALL_OPMODES = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'opmodes.json'), 'utf8'));
export const OPMODES = ALL_OPMODES.filter((o) => o.kind !== 'teleop');   // the autos: what the page lists and the batch runner runs
export const TELEOPS = ALL_OPMODES.filter((o) => o.kind === 'teleop');   // driven live from #/drive
// What the Setup screen can change: sim/params.json describes each one; the Java side (SimParams.java)
// owns the default values and applies them to the code.
export const PARAM_DEFS = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'params.json'), 'utf8'));

/**
 * The -D arguments for one sim run. `seed` seeds both the Java camera and the physics; `params` is a list of ready -D strings
 * (the Setup screen's overrides). Anything not given is left out so Java uses its default.
 */
export function simArgs({ seed, logDir, autoParams, params = [], partnerPath, partnerSpeed, defenderPath } = {}) {
  const args = ['-Djdk.attach.allowAttachSelf=true'];
  if (seed !== undefined) args.push(`-Dsim.seed=${seed}`, `-Dsim.physics.seed=${seed}`);
  if (logDir) args.push(`-Dsim.logDir=${logDir}`);
  if (autoParams) args.push(`-Dsim.autoParams=${autoParams}`);
  args.push(...params);
  if (partnerPath) args.push(`-Dsim.partnerPath=${partnerPath}`, `-Dsim.partnerSpeed=${partnerSpeed ?? 1}`);
  if (defenderPath) args.push(`-Dsim.defenders=${defenderPath}`);
  return args;
}

function safeJson(text) { try { return JSON.parse(text); } catch { return {}; } }

// The run file -> JSON for the page. All times become match seconds (0 = START).
export function parseRun(text, opMode) {
  const run = {
    opMode: opMode.name, alliance: opMode.alliance, start: opMode.start,
    frames: [], events: [], logs: [], shots: [], warnings: [], telemetry: [], error: null, params: {}, defenders: [],
    balls: { info: [], tracks: [], hive: [], held: [], params: BALL_DEFAULTS }, assumed: [],
    used: 0, finished: false, matchStart: 0,
    field: { size: 3657.6, robot: { front: 300, back: 229, width: 457 }, obstacles: [], hive: null, midline: 0, points: [] },
  };
  const rawFrames = [];
  const rawTelemetry = [];
  const rawBalls = [];
  const rawHive = [];
  const rawHeld = [];
  const rawPartner = [];
  const rawDefenders = [];
  const rawEvents = [];
  const errorLines = [];
  const rawLogs = [];
  const rawShots = [];
  const rawResults = [];
  for (const line of text.split('\n')) {
    if (!line) continue;
    if (!line.startsWith('#')) {
      rawFrames.push(line.split(',').map(Number));
      continue;
    }
    const space = line.indexOf(' ');
    const key = line.slice(1, space < 0 ? undefined : space);
    const rest = space < 0 ? '' : line.slice(space + 1);
    const n = rest.split(' ');
    switch (key) {
      case 'used': run.used = Number(rest); break;
      case 'finished': run.finished = rest === 'true'; break;
      case 'matchStart': run.matchStart = Number(rest); break;
      case 'event': { const bar = rest.indexOf('|'); rawEvents.push({ t: Number(rest.slice(0, bar)), text: rest.slice(bar + 1) }); break; }
      case 'log': { const [t, tag, ...msg] = rest.split('|'); rawLogs.push({ t: Number(t), tag, msg: msg.join('|') }); break; }
      case 'shot': { const bar = rest.indexOf('|'); rawShots.push({ t: Number(rest.slice(0, bar)), ...safeJson(rest.slice(bar + 1)) }); break; }
      case 'shotresult': { const bar = rest.indexOf('|'); rawResults.push(safeJson(rest.slice(bar + 1))); break; }
      case 'warning': run.warnings.push(rest); break;
      case 'error': errorLines.push(rest); break;
      case 'param': {
        const def = PARAM_DEFS.find((d) => d.id === n[0]);
        run.params[n[0]] = { used: Number(n[1]), default: Number(n[2]), label: def ? def.label : n[0], unit: def ? def.unit : '' };
        break;
      }
      case 'balltype': run.balls.info.push({ id: +n[0], type: +n[1], radius: +n[2] }); break;
      case 'ball': rawBalls.push([+n[0], +n[1], +n[2], +n[3], +n[4]]); break;
      case 'hivetilt': rawHive.push([+n[0], +n[1], +n[2]]); break;
      case 'held': rawHeld.push([+n[0], +n[1]]); break;
      case 'assumed': run.assumed.push(rest); break;
      case 'partnerinfo': run.partner = { name: n[0], robot: { front: +n[1], back: +n[2], width: +n[3] }, frames: [] }; break;
      case 'partner': rawPartner.push([+n[0], +n[1], +n[2], +n[3]]); break;
      case 'defenderinfo': run.defenders.push({ name: n[0], robot: { front: +n[1], back: +n[2], width: +n[3] }, frames: [] }); rawDefenders.push([]); break;
      case 'defender': rawDefenders[+n[0]].push([+n[1], +n[2], +n[3], +n[4]]); break;
      case 'field': run.field.size = Number(rest); break;
      case 'robot': run.field.robot = { front: Number(n[0]), back: Number(n[1]), width: Number(n[2]) }; break;
      case 'obstacle': run.field.obstacles.push({ name: n[0], minX: +n[1], minY: +n[2], maxX: +n[3], maxY: +n[4] }); break;
      case 'hive': run.field.hive = { minX: +n[0], minY: +n[1], maxX: +n[2], maxY: +n[3] }; break;
      case 'midline': run.field.midline = Number(rest); break;
      case 'point': run.field.points.push({ name: n[0], x: +n[1], y: +n[2] }); break;
      case 'telemetry': {
        const t = Number(n[0]);
        const items = {};
        for (const pair of rest.slice(n[0].length + 1).split('&')) {
          if (!pair) continue;
          const eq = pair.indexOf('=');
          items[decodeURIComponent(pair.slice(0, eq).replace(/\+/g, ' '))] = decodeURIComponent(pair.slice(eq + 1).replace(/\+/g, ' '));
        }
        rawTelemetry.push({ t, items });
        break;
      }
      default: break;
    }
  }
  if (errorLines.length) run.error = errorLines.join('\n').trim();

  const start = run.matchStart;
  const r1 = (v) => Math.round(v * 10) / 10;
  const r4 = (v) => Math.round(v * 1e4) / 1e4;
  let before = null;
  // 7 columns, plus turretDeg, hood, rps when the run recorded them (old run files have 7)
  const frameRow = (t, f) => {
    const row = [t, r1(f[1]), r1(f[2]), r4(f[3]), r1(f[4]), r1(f[5]), r4(f[6])];
    if (f.length >= 10 && Number.isFinite(f[7])) row.push(r1(f[7]), r4(f[8]), r1(f[9]));
    return row;
  };
  for (const f of rawFrames) {
    if (f[0] < start) { before = f; continue; }
    if (before && run.frames.length === 0 && f[0] > start) {
      run.frames.push(frameRow(0, before));
    }
    run.frames.push(frameRow(r4(f[0] - start), f));
  }
  // The other robots (R2 and the defenders): same shift as the frames, and the last frame before START is kept at t = 0.
  const shiftBot = (bot, raw) => {
    let pBefore = null;
    for (const f of raw) {
      if (f[0] < start) { pBefore = f; continue; }
      if (pBefore && bot.frames.length === 0 && f[0] > start) bot.frames.push([0, r1(pBefore[1]), r1(pBefore[2]), r4(pBefore[3])]);
      bot.frames.push([r4(f[0] - start), r1(f[1]), r1(f[2]), r4(f[3])]);
    }
  };
  if (run.partner) shiftBot(run.partner, rawPartner);
  run.defenders.forEach((d, i) => shiftBot(d, rawDefenders[i]));
  // Balls: one track per ball, [t, x, y, z] only when it moved. Moves before START collapse into where it sat at t = 0.
  const tracks = run.balls.info.map(() => []);
  for (const [t, id, x, y, z] of rawBalls) {
    const track = tracks[id];
    if (!track) continue;
    const point = [r4(t - start), Math.round(x), Math.round(y), Math.round(z)];
    if (point[0] <= 0) {
      // before START: only where the ball ended up counts
      if (track.length) track[track.length - 1] = [0, point[1], point[2], point[3]];
      else track.push([0, point[1], point[2], point[3]]);
      continue;
    }
    const prev = track[track.length - 1];
    // a ball that sat still is only recorded when it moves, so hold it in place until just before it does
    if (prev && point[0] - prev[0] > 0.03) track.push([r4(point[0] - 0.02), prev[1], prev[2], prev[3]]);
    track.push(point);
  }
  run.balls.tracks = tracks;
  // Hive tilt per frame (match time); the last one before START is kept at t = 0
  let hiveBefore = null;
  for (const [t, red, blue] of rawHive) {
    if (t < start) { hiveBefore = [0, r4(red), r4(blue)]; continue; }
    if (hiveBefore) { run.balls.hive.push(hiveBefore); hiveBefore = null; }
    run.balls.hive.push([r4(t - start), r4(red), r4(blue)]);
  }
  // balls held by the robot per frame; the last value before START is kept at t = 0
  let heldBefore = null;
  for (const [t, count] of rawHeld) {
    if (t < start) { heldBefore = [0, count]; continue; }
    if (heldBefore) { run.balls.held.push(heldBefore); heldBefore = null; }
    run.balls.held.push([r4(t - start), count]);
  }
  for (const l of rawLogs) run.logs.push({ t: r4(l.t - start), tag: l.tag, msg: l.msg });
  for (const s of rawShots) {
    const { t, ...rest } = s;
    const result = rawResults.find((r) => r.n === s.n) || null;
    run.shots.push({ ...rest, t: r4(t - start), waitStart: rest.waitStart == null ? null : r4(rest.waitStart - start), result });
  }
  for (const e of rawEvents) run.events.push({ t: r4(e.t - start), text: e.text });
  // telemetry: only when something changed, and only from the match (the line before START is kept as t = 0)
  let last = null;
  let lastKey = '';
  for (const entry of rawTelemetry) {
    const key = JSON.stringify(entry.items);
    if (entry.t < start) { last = entry; continue; }
    if (last) { run.telemetry.push({ t: 0, items: last.items }); lastKey = JSON.stringify(last.items); last = null; }
    if (key !== lastKey) { run.telemetry.push({ t: r4(entry.t - start), items: entry.items }); lastKey = key; }
  }
  return run;
}
