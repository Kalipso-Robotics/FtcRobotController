// The simulator's local server (127.0.0.1 only). It does four things:
//   1. serves the web page (web/) and three.js,
//   2. builds the Java and runs the sim when asked, or when a .java file is saved,
//   3. turns the run's output file into JSON for the page (GET /api/run),
//   4. tells the page what it is doing (Server-Sent Events on /api/events).
// The robot code is NOT run here. Java runs it, in its own process; this server only starts that process.

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { spawn } from 'node:child_process';

const ROOT = path.resolve(import.meta.dirname, '..', '..');
const WEB = path.join(import.meta.dirname, 'web');
const THREE = path.join(import.meta.dirname, 'node_modules', 'three', 'build');
const THREE_ADDONS = path.join(import.meta.dirname, 'node_modules', 'three', 'examples', 'jsm');
const BALL_DEFAULTS = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'physics', 'ball-defaults.json'), 'utf8'));
const PORT = Number(process.env.PORT || 5173);
const JAVA_HOME = process.env.JAVA_HOME || '/Applications/Android Studio.app/Contents/jbr/Contents/Home';
const WATCHED = [path.join(ROOT, 'TeamCode', 'src')];
const OPMODES = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'opmodes.json'), 'utf8'));
// What the Setup screen can change: sim/params.json describes each one; the Java side (SimParams.java)
// owns the default values and applies them to the code.
const PARAM_DEFS = JSON.parse(fs.readFileSync(path.join(ROOT, 'sim', 'params.json'), 'utf8'));
const PARAMS_FILE = path.join(ROOT, 'sim', '.last-params.json');
let paramDefaults = null;     // { id: value } the robot-size defaults come from Java (known after the first build); the physics ones from ball-defaults.json
const PHYSICS_DEFAULT = { preload: 4 };
const physicsDefault = (def) => ((PHYSICS_DEFAULT[def.id] ?? BALL_DEFAULTS[def.id]) / (def.scale || 1));
let paramValues = {};         // what the user last chose; used by every run, including run-on-save
try { paramValues = JSON.parse(fs.readFileSync(PARAMS_FILE, 'utf8')); } catch { /* none saved yet */ }

const state = {
  phase: 'idle',        // idle | building | running | done | compile-error | run-error
  message: '',
  runId: 0,
  opMode: OPMODES[0].name,
  watching: true,
  lastRunMs: 0,
};
let latestRun = null;
let queued = false;
let busy = false;

// ---------------------------------------------------------------------------------------------
// Status to the page
// ---------------------------------------------------------------------------------------------

const listeners = new Set();
function publicState() {
  return { ...state };
}
function broadcast() {
  const data = `data: ${JSON.stringify(publicState())}\n\n`;
  for (const res of listeners) res.write(data);
}
function setPhase(phase, message = '') {
  state.phase = phase;
  state.message = message;
  broadcast();
}

// ---------------------------------------------------------------------------------------------
// Build and run
// ---------------------------------------------------------------------------------------------

function run(command, args, options) {
  return new Promise((resolve) => {
    const child = spawn(command, args, options);
    let output = '';
    child.stdout.on('data', (d) => { output += d; });
    child.stderr.on('data', (d) => { output += d; });
    child.on('error', (error) => resolve({ code: -1, output: String(error) }));
    child.on('close', (code) => resolve({ code, output }));
  });
}

async function buildAndRun() {
  if (busy) {
    queued = true; // a save during a run: run again afterwards, once
    return;
  }
  busy = true;
  try {
    do {
      queued = false;
      const opMode = OPMODES.find((o) => o.name === state.opMode) || OPMODES[0];
      const started = Date.now();

      setPhase('building', 'Compiling');
      const build = await run(path.join(ROOT, 'gradlew'), [':TeamCode:simClasspath', '--offline', '-q'],
        { cwd: ROOT, env: { ...process.env, JAVA_HOME } });
      if (build.code !== 0) {
        setPhase('compile-error', cleanBuildOutput(build.output));
        continue;
      }

      const simStarted = Date.now();
      const classpath = fs.readFileSync(path.join(ROOT, 'TeamCode', 'build', 'sim-classpath.txt'), 'utf8');
      if (!paramDefaults) paramDefaults = await readParamDefaults(classpath);
      const paramArgs = paramOptions();

      setPhase('running', `Running ${opMode.name}`);
      const runId = state.runId + 1;
      const outFile = path.join(ROOT, 'TeamCode', 'build', 'sim-runs', `app-${runId}.csv`);
      fs.mkdirSync(path.dirname(outFile), { recursive: true });
      fs.rmSync(outFile, { force: true });
      const sim = await run(path.join(JAVA_HOME, 'bin', 'java'), [
        '-Djdk.attach.allowAttachSelf=true', ...paramArgs, '-cp', classpath,
        'org.firstinspires.ftc.teamcode.kalipsorobotics.sim.SimMain',
        opMode.className, String(opMode.start.x), String(opMode.start.y), String(opMode.start.headingDeg * Math.PI / 180),
        outFile, 'rapier',
      ], { cwd: path.join(ROOT, 'TeamCode') });

      if (!fs.existsSync(outFile)) {
        setPhase('run-error', sim.output.split('\n').filter((l) => !l.startsWith('WARNING')).join('\n').trim());
        continue;
      }
      latestRun = parseRun(fs.readFileSync(outFile, 'utf8'), opMode);
      addPhysicsParams(latestRun);
      state.runId = runId;
      state.lastRunMs = Date.now() - started;
      latestRun.meta = { runId, totalMs: state.lastRunMs, simMs: Date.now() - simStarted, buildMs: simStarted - started, world: 'rapier', finishedAt: new Date().toISOString() };
      if (latestRun.error) setPhase('run-error', latestRun.error);
      else setPhase('done', `${opMode.name}: ${latestRun.events.length} contacts, ${latestRun.warnings.length} warnings`);
    } while (queued);
  } finally {
    busy = false;
  }
}

/** Java's defaults plus the physics defaults, in the units the Setup screen shows. */
function allDefaults() {
  if (!paramDefaults) return null;
  const merged = { ...paramDefaults };
  for (const def of PARAM_DEFS) if (def.target === 'physics') merged[def.id] = Math.round(physicsDefault(def) * 1e6) / 1e6;
  return merged;
}

/** The -D options for a run: the values that differ from the defaults. Physics ones go to Java as sim.physics.<name>, in physics units. */
function paramOptions() {
  const defaults = allDefaults();
  const options = [];
  if (!defaults) return options;
  for (const def of PARAM_DEFS) {
    const value = paramValues[def.id];
    if (value === undefined || Number(value) === defaults[def.id]) continue;
    if (def.target === 'physics') options.push(`-Dsim.physics.${def.id}=${Number(value) * (def.scale || 1)}`);
    else options.push(`-Dsim.param.${def.id}=${Number(value)}`);
  }
  return options;
}

/** Asks the Java side for the default of every parameter ("#param <id> <default>" lines). */
async function readParamDefaults(classpath) {
  const result = await run(path.join(JAVA_HOME, 'bin', 'java'),
    ['-cp', classpath, 'org.firstinspires.ftc.teamcode.kalipsorobotics.sim.SimMain', '--defaults'], { cwd: path.join(ROOT, 'TeamCode') });
  const defaults = {};
  for (const line of result.output.split('\n')) {
    const m = /^#param (\S+) (\S+)$/.exec(line.trim());
    if (m) defaults[m[1]] = Number(m[2]);
  }
  return Object.keys(defaults).length ? defaults : null;
}

/** Checks the values from the Setup screen against params.json. Returns an error text, or null if they are fine. */
function validateParams(values) {
  for (const [id, value] of Object.entries(values)) {
    const def = PARAM_DEFS.find((d) => d.id === id);
    if (!def) return `Unknown parameter "${id}".`;
    if (typeof value !== 'number' || !Number.isFinite(value)) return `${def.label} must be a number.`;
    if (value < def.min || value > def.max) return `${def.label} must be between ${def.min} and ${def.max} ${def.unit}.`;
  }
  return null;
}

/**
 * Gradle prints each compiler error twice, plus a lot of boilerplate. Keep each error once:
 * "file:line: error: message", the source line and the caret, with the path shortened to the repo.
 */
function cleanBuildOutput(output) {
  const lines = output.split('\n');
  const seen = new Set();
  const blocks = [];
  for (let i = 0; i < lines.length; i++) {
    if (!/: error:/.test(lines[i])) continue;
    const head = lines[i].trim().replace(ROOT + path.sep, '');
    if (seen.has(head)) continue;
    seen.add(head);
    blocks.push([head, lines[i + 1], lines[i + 2]].filter((l) => l !== undefined).map((l, k) => (k === 0 ? l : l.replace(/^\s{0,2}/, ''))).join('\n'));
  }
  if (blocks.length) return blocks.join('\n\n');
  return lines.filter((l) => l.trim() && !/^(warning:|Note:|\d+ warnings?)/.test(l.trim())).slice(0, 40).join('\n');
}

/** The physics parameters of a run, as run.params entries (what was used and the default), and the ball parameters the 3D view needs. */
function addPhysicsParams(run) {
  const defaults = allDefaults() || {};
  const used = { ...BALL_DEFAULTS };
  for (const def of PARAM_DEFS) {
    if (def.target !== 'physics') continue;
    const value = paramValues[def.id] === undefined ? defaults[def.id] : Number(paramValues[def.id]);
    run.params[def.id] = { used: value, default: defaults[def.id], label: def.label, unit: def.unit, group: def.group };
    if (def.id !== 'preload') used[def.id] = value * (def.scale || 1);
  }
  run.balls.params = used;
}

// ---------------------------------------------------------------------------------------------
// The run file -> JSON for the page. All times become match seconds (0 = START).
// ---------------------------------------------------------------------------------------------

function parseRun(text, opMode) {
  const run = {
    opMode: opMode.name, alliance: opMode.alliance, start: opMode.start,
    frames: [], events: [], warnings: [], telemetry: [], error: null, params: {},
    balls: { info: [], tracks: [], hive: [], held: [], params: BALL_DEFAULTS }, assumed: [],
    used: 0, finished: false, matchStart: 0,
    field: { size: 3657.6, robot: { front: 300, back: 229, width: 457 }, obstacles: [], hive: null, midline: 0, points: [] },
  };
  const rawFrames = [];
  const rawTelemetry = [];
  const rawBalls = [];
  const rawHive = [];
  const rawHeld = [];
  const rawEvents = [];
  const errorLines = [];
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
  for (const f of rawFrames) {
    if (f[0] < start) { before = f; continue; }
    if (before && run.frames.length === 0 && f[0] > start) {
      run.frames.push([0, r1(before[1]), r1(before[2]), r4(before[3]), r1(before[4]), r1(before[5]), r4(before[6])]);
    }
    run.frames.push([r4(f[0] - start), r1(f[1]), r1(f[2]), r4(f[3]), r1(f[4]), r1(f[5]), r4(f[6])]);
  }
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

// ---------------------------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------------------------

const TYPES = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.svg': 'image/svg+xml' };

function sendFile(res, root, relative) {
  const file = path.resolve(root, relative);
  if (!file.startsWith(root + path.sep) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
    res.writeHead(404).end('not found');
    return;
  }
  res.writeHead(200, { 'content-type': TYPES[path.extname(file)] || 'application/octet-stream', 'cache-control': 'no-cache' });
  fs.createReadStream(file).pipe(res);
}

function readBody(req) {
  return new Promise((resolve) => {
    let body = '';
    req.on('data', (d) => { body += d; });
    req.on('end', () => { try { resolve(JSON.parse(body || '{}')); } catch { resolve({}); } });
  });
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, 'http://localhost');
  const json = (value, status = 200) => res.writeHead(status, { 'content-type': TYPES['.json'], 'cache-control': 'no-cache' }).end(JSON.stringify(value));

  if (url.pathname === '/api/opmodes') return json(OPMODES.map(({ name, alliance, start, note }) => ({ name, alliance, start, note })));
  if (url.pathname === '/api/state') return json(publicState());
  if (url.pathname === '/api/run' && req.method === 'GET') return latestRun ? json(latestRun) : json({ error: 'no run yet' }, 404);
  if (url.pathname === '/api/params') return json({ defs: PARAM_DEFS, defaults: allDefaults(), values: { ...allDefaults(), ...paramValues } });
  if (url.pathname === '/api/run' && req.method === 'POST') {
    const body = await readBody(req);
    if (body.params) {
      const problem = validateParams(body.params);
      if (problem) return json({ error: problem }, 400);
      paramValues = { ...body.params };
      try { fs.writeFileSync(PARAMS_FILE, JSON.stringify(paramValues, null, 2)); } catch { /* not saving is fine */ }
    }
    if (body.opMode && OPMODES.some((o) => o.name === body.opMode)) state.opMode = body.opMode;
    buildAndRun();
    return json({ ok: true });
  }
  if (url.pathname === '/api/watch' && req.method === 'POST') {
    state.watching = Boolean((await readBody(req)).enabled);
    broadcast();
    return json({ ok: true });
  }
  if (url.pathname === '/api/events') {
    res.writeHead(200, { 'content-type': 'text/event-stream', 'cache-control': 'no-cache', connection: 'keep-alive' });
    res.write(`data: ${JSON.stringify(publicState())}\n\n`);
    listeners.add(res);
    req.on('close', () => listeners.delete(res));
    return;
  }
  if (url.pathname.startsWith('/vendor/three/')) return sendFile(res, THREE, url.pathname.slice('/vendor/three/'.length));
  if (url.pathname.startsWith('/vendor/three-addons/')) return sendFile(res, THREE_ADDONS, url.pathname.slice('/vendor/three-addons/'.length));
  return sendFile(res, WEB, url.pathname === '/' ? 'index.html' : url.pathname.slice(1));
});

setInterval(() => { for (const res of listeners) res.write(': keep-alive\n\n'); }, 15000);

// Save a .java file and the sim runs again. Changes that arrive together count as one.
let debounce = null;
for (const dir of WATCHED) {
  fs.watch(dir, { recursive: true }, (event, name) => {
    if (!state.watching || !name || !name.endsWith('.java')) return;
    clearTimeout(debounce);
    debounce = setTimeout(() => buildAndRun(), 800);
  });
}

server.listen(PORT, '127.0.0.1', () => {
  console.log(`Auto simulator: http://localhost:${PORT}`);
  buildAndRun();
});
