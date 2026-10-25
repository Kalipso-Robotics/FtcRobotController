// The page. It draws what the simulator recorded: the field, the robot, its path, and the events.
// It does NOT run any robot code: Java runs the auto and the server hands over the result.
//
// Field frame (same as the robot code): x, y in mm, y points DOWN in the manual's top view for Red.
// The scene uses (x, -y) so the picture matches that top view. Robot heading theta is counter-clockwise
// in the field frame, so the scene rotation is -theta.

import * as THREE from 'three';
import { analyse } from './stats.js';
import { renderStats } from './statsView.js';
import { build3D } from './scene3d.js';

const $ = (id) => document.getElementById(id);
const FRAME_SECONDS = 0.02;

// ---------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------

let run = null;          // the latest run (JSON from /api/run)
let loadedRunId = -1;
let loadedOnce = false;
let time = 0;            // match seconds
let playing = false;
let speed = 1;
let duration = 30;
let scrubbing = false;
let panelOpen = true;
let serverState = { phase: 'idle', message: '' };
let view = 'landing';    // landing | setup | stats | replay
let analysis = null;     // numbers and checks for the latest run (stats.js)
let launching = false;   // the user pressed Run on the landing page and is waiting for the stats
let opmodes = [];
let paramDefs = [];      // what the Setup screen can change (sim/params.json)
let paramDefaults = null; // the code's own values, from Java; null until the first build
let paramValues = {};    // what the user has chosen
let selectedOpMode = null;
let renderUntil = 0;     // keep drawing the scene until a fade-out has finished

// ---------------------------------------------------------------------------------------------
// Three.js scene
// ---------------------------------------------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ canvas: $('view'), antialias: true, alpha: true });
renderer.setClearColor(0x000000, 0);
const scene = new THREE.Scene();
const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10, 10);
camera.position.z = 5;

let world = null;        // everything that belongs to one run's picture
let robot = null;        // { group } true pose
let ghost = null;        // odometry pose
let trails = null;       // { truth, odo, count }
let ballLayer = null;    // { discs: [{ disc, shadow }], cells: [{ mesh, hive, side }] } the balls and Hive Cells in the top view
let scene3d = null;      // the 3D view (scene3d.js)
let viewMode = '2d';     // 2d | 3d
let labels = [];         // { el, x, y }

const css = () => getComputedStyle(document.documentElement);
const color = (name) => new THREE.Color(css().getPropertyValue(name).trim() || '#888888');

function rectPoints(minX, minY, maxX, maxY, z = 0) {
  return [[minX, -minY], [maxX, -minY], [maxX, -maxY], [minX, -maxY]].map(([x, y]) => new THREE.Vector3(x, y, z));
}

/** A filled and/or outlined rectangle in field coordinates. */
function rect(group, [minX, minY, maxX, maxY], { fill, fillOpacity = 0.2, line, dashed = false, lineOpacity = 1, z = 0 }) {
  if (fill) {
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(maxX - minX, maxY - minY),
      new THREE.MeshBasicMaterial({ color: fill, transparent: true, opacity: fillOpacity, depthWrite: false }));
    mesh.position.set((minX + maxX) / 2, -(minY + maxY) / 2, z);
    group.add(mesh);
  }
  if (line) group.add(outline(rectPoints(minX, minY, maxX, maxY, z + 0.01), line, dashed, lineOpacity));
}

function outline(points, lineColor, dashed, opacity = 1) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = dashed
    ? new THREE.LineDashedMaterial({ color: lineColor, dashSize: 40, gapSize: 28, transparent: true, opacity })
    : new THREE.LineBasicMaterial({ color: lineColor, transparent: true, opacity });
  const loop = new THREE.LineLoop(geometry, material);
  if (dashed) loop.computeLineDistances();
  return loop;
}

function segments(group, pairs, lineColor, dashed, opacity = 1) {
  const points = pairs.flatMap(([a, b]) => [new THREE.Vector3(a[0], -a[1], 0.01), new THREE.Vector3(b[0], -b[1], 0.01)]);
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = dashed
    ? new THREE.LineDashedMaterial({ color: lineColor, dashSize: 30, gapSize: 26, transparent: true, opacity })
    : new THREE.LineBasicMaterial({ color: lineColor, transparent: true, opacity });
  const lines = new THREE.LineSegments(geometry, material);
  if (dashed) lines.computeLineDistances();
  group.add(lines);
}

function addLabel(text, x, y) {
  const el = document.createElement('span');
  el.textContent = text;
  $('labels').appendChild(el);
  labels.push({ el, x, y });
}

/** The robot's footprint in the robot frame: x forward, y left. Placeholder numbers from RobotFootprint.java. */
function robotShape({ front, back, width }, { fill, line, opacity }) {
  const group = new THREE.Group();
  const h = width / 2;
  const body = [[front, h], [front, -h], [-back, -h], [-back, h]].map(([x, y]) => new THREE.Vector3(x, -y, 0)); // y flipped for the scene
  if (fill) {
    const shape = new THREE.Shape(body.map((p) => new THREE.Vector2(p.x, p.y)));
    group.add(new THREE.Mesh(new THREE.ShapeGeometry(shape),
      new THREE.MeshBasicMaterial({ color: fill, transparent: true, opacity, depthWrite: false })));
  }
  group.add(outline(body, line, false, fill ? 1 : 0.9));
  // a small arrow on the front edge shows which way the robot faces
  const arrow = [new THREE.Vector3(front - 70, -h * 0.45, 0), new THREE.Vector3(front + 30, 0, 0), new THREE.Vector3(front - 70, h * 0.45, 0)];
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(arrow), new THREE.LineBasicMaterial({ color: line })));
  return group;
}

function buildScene() {
  if (world) {
    scene.remove(world);
    world.traverse((o) => { o.geometry?.dispose(); o.material?.dispose(); });
  }
  labels.forEach(({ el }) => el.remove());
  labels = [];
  world = new THREE.Group();
  scene.add(world);
  if (!run) return;

  const f = run.field;
  const S = f.size;
  const c = { field: color('--field'), line: color('--line'), tile: color('--tile'), accent: color('--accent'),
    ghost: color('--ghost'), red: color('--red'), blue: color('--blue'), obstacle: color('--obstacle') };

  // floor, tile grid (24 in tiles), wall, midline
  rect(world, [0, 0, S, S], { fill: c.field, fillOpacity: 1, line: c.line, z: -1 });
  const tile = 609.6;
  const grid = [];
  for (let k = 1; k < 6; k++) {
    grid.push([[k * tile, 0], [k * tile, S]], [[0, k * tile], [S, k * tile]]);
  }
  segments(world, grid, c.tile, true, 0.9);
  segments(world, [[[f.midline, 0], [f.midline, S]]], c.line, true, 0.5);

  // Hive: frame outline, red cells on the left half, blue on the right (manual top view)
  if (f.hive) {
    const h = f.hive;
    const mid = (h.minX + h.maxX) / 2;
    rect(world, [h.minX, h.minY, mid, h.maxY], { fill: c.red, fillOpacity: 0.07 });
    rect(world, [mid, h.minY, h.maxX, h.maxY], { fill: c.blue, fillOpacity: 0.07 });
    rect(world, [h.minX, h.minY, h.maxX, h.maxY], { line: c.line, lineOpacity: 0.6 });
    addLabel('Hive', mid, (h.minY + h.maxY) / 2);
  }

  // obstacles. Dashed = the size is a placeholder in FieldConfig (the leg lean), so it is unverified.
  for (const o of f.obstacles) {
    const isLeg = o.name.startsWith('hive-leg');
    rect(world, [o.minX, o.minY, o.maxX, o.maxY],
      { fill: c.obstacle, fillOpacity: isLeg ? 0.5 : 0.35, line: c.obstacle, dashed: isLeg, z: 0.02 });
    if (o.name.startsWith('flower-')) addLabel(o.name.slice(-1), (o.minX + o.maxX) / 2, (o.minY + o.maxY) / 2 + (o.maxY - o.minY));
  }

  // named points from FieldConfig
  const ring = new THREE.RingGeometry(26, 34, 24);
  for (const p of f.points) {
    const mesh = new THREE.Mesh(ring, new THREE.MeshBasicMaterial({ color: p.name.startsWith('target') ? c.line : c.accent, transparent: true, opacity: p.name.startsWith('target') ? 0.6 : 0.8 }));
    mesh.position.set(p.x, -p.y, 0.03);
    world.add(mesh);
    addLabel(p.name.replace('-', ' '), p.x, p.y + 90);
  }

  // trails and robots
  const n = run.frames.length;
  const trail = (index, lineColor, opacity) => {
    const positions = new Float32Array(n * 3);
    run.frames.forEach((fr, i) => { positions.set([fr[index], -fr[index + 1], 0.05], i * 3); });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: lineColor, transparent: true, opacity }));
    line.frustumCulled = false;
    world.add(line);
    return line;
  };
  trails = { truth: trail(1, c.accent, 0.7), odo: trail(4, c.ghost, 0.55), count: n };

  ghost = { group: robotShape(f.robot, { line: c.ghost }) };
  ghost.group.position.z = 0.2;
  robot = { group: robotShape(f.robot, { fill: c.accent, line: c.accent, opacity: 0.18 }) };
  robot.group.position.z = 0.3;
  world.add(ghost.group, robot.group);

  buildBalls(c);
  if (scene3d) scene3d.dispose();
  scene3d = build3D(run, c, $('view'));
  scene3d.resize($('view').clientWidth || 1, $('view').clientHeight || 1);
  scene3d.controls.enabled = view === 'replay' && viewMode === '3d';
}

// ---------------------------------------------------------------------------------------------
// Balls and Hive Cells in the top view
// ---------------------------------------------------------------------------------------------

function buildBalls(c) {
  ballLayer = { discs: [], cells: [] };
  if (!run.balls.info.length) return;
  const ballColor = (type) => (type === 0 ? new THREE.Color(0xe8b923) : type === 1 ? c.red : c.blue);
  for (const b of run.balls.info) {
    const shadow = new THREE.Mesh(new THREE.CircleGeometry(b.radius, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.16, depthWrite: false }));
    const disc = new THREE.Mesh(new THREE.CircleGeometry(b.radius, 24), new THREE.MeshBasicMaterial({ color: ballColor(b.type) }));
    const rim = new THREE.Mesh(new THREE.RingGeometry(b.radius * 0.86, b.radius, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.35, depthWrite: false }));
    disc.add(rim);
    rim.position.z = 0.01;
    world.add(shadow, disc);
    ballLayer.discs.push({ disc, shadow, radius: b.radius });
  }
  // the two Cells of each Hive, seen from above: the lifted Cell is drawn stronger
  const p = run.balls.params, h = run.field.hive;
  const centreX = (h.minX + h.maxX) / 2, centreY = (h.minY + h.maxY) / 2, offset = (25.5 * 25.4) / 2;
  [[centreX - offset, c.red], [centreX + offset, c.blue]].forEach(([x, color], hive) => {
    for (const side of [1, -1]) {
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.2, depthWrite: false }));
      mesh.position.z = 0.04;
      world.add(mesh);
      ballLayer.cells.push({ mesh, hive, side, x, centreY, width: p.hiveCellWidthMm, inner: p.hiveCellInnerMm, depth: p.hiveCellDepthMm });
    }
  });
}

/** Where every ball is at match time t: [[x, y, z]] in mm. Each ball has a track of [t, x, y, z] points. */
function ballsAt(t) {
  return run.balls.tracks.map((track) => {
    if (!track.length) return [0, 0, 0];
    let lo = 0, hi = track.length - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (track[mid][0] <= t) lo = mid; else hi = mid - 1; }
    const a = track[lo], b = track[Math.min(lo + 1, track.length - 1)];
    const k = b[0] > a[0] ? Math.min(1, Math.max(0, (t - a[0]) / (b[0] - a[0]))) : 0;
    return [a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, a[3] + (b[3] - a[3]) * k];
  });
}

/** The red and blue Hive tilt at match time t, in radians. */
function hiveTiltAt(t) {
  const frames = run.balls.hive;
  if (!frames.length) return [0, 0];
  const i = frameIndex(frames, t);
  const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)];
  const k = b[0] > a[0] ? Math.min(1, Math.max(0, (t - a[0]) / (b[0] - a[0]))) : 0;
  return [a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k];
}

function updateTopBalls(positions, tilts) {
  if (!ballLayer) return;
  ballLayer.discs.forEach((d, i) => {
    const [x, y, z] = positions[i];
    const lift = 1 + z / 2400;                       // a ball high up looks a little bigger, and its shadow slides away
    d.disc.position.set(x, -y, 0.2 + z / 4000);
    d.disc.scale.setScalar(lift);
    d.shadow.position.set(x + z * 0.12, -y - z * 0.12, 0.1);
    d.shadow.material.opacity = Math.max(0.05, 0.2 - z / 12000);
  });
  for (const cell of ballLayer.cells) {
    const tilt = tilts[cell.hive];
    const along = Math.cos(tilt);
    const lifted = cell.side * tilt > 0;
    cell.mesh.scale.set(cell.width, cell.depth * along, 1);
    cell.mesh.position.x = cell.x;
    cell.mesh.position.y = -(cell.centreY + cell.side * (cell.inner + cell.depth / 2) * along);
    cell.mesh.material.opacity = lifted ? 0.3 : 0.08;
  }
}

/** Counts of where the balls are right now, for the panel. */
function ballCounts(positions, pose) {
  const f = run.field, hive = f.hive, S = f.size, rb = f.robot;
  const flowers = f.obstacles.filter((o) => o.name.startsWith('flower-')).map((o) => [(o.minX + o.maxX) / 2, (o.minY + o.maxY) / 2]);
  const counts = { 'In the robot': 0, 'In a Flower': 0, 'In a Garden': 0, 'In a Hive Cell': 0, 'On the floor': 0, 'In the air': 0 };
  const c = Math.cos(pose.theta), sn = Math.sin(pose.theta);
  for (const [x, y, z] of positions) {
    const lx = (x - pose.x) * c + (y - pose.y) * sn, ly = -(x - pose.x) * sn + (y - pose.y) * c;
    if (z > 100 && z < 300 && lx > -rb.back && lx < rb.front && Math.abs(ly) < rb.width / 2) counts['In the robot']++;
    else if (z > 600 && x > hive.minX - 100 && x < hive.maxX + 100 && y > hive.minY - 150 && y < hive.maxY + 150) counts['In a Hive Cell']++;
    else if (z < 600 && flowers.some(([fx, fy]) => Math.abs(x - fx) < 80 && Math.abs(y - fy) < 80 && (z > 60 || Math.hypot(x - fx, y - fy) < 40))) counts['In a Flower']++;
    else if (z < 60 && (y > S - 130 || y < 130) && (x < 700 || x > S - 700)) counts['In a Garden']++;
    else if (z < 100) counts['On the floor']++;
    else counts['In the air']++;
  }
  return counts;
}

// ---------------------------------------------------------------------------------------------
// Time -> pose
// ---------------------------------------------------------------------------------------------

/** Index of the last frame at or before t. */
function frameIndex(frames, t) {
  let lo = 0, hi = frames.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (frames[mid][0] <= t) lo = mid; else hi = mid - 1;
  }
  return lo;
}

function poseAt(t) {
  const frames = run.frames;
  const i = frameIndex(frames, t);
  const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)];
  const k = b[0] > a[0] ? Math.min(1, Math.max(0, (t - a[0]) / (b[0] - a[0]))) : 0;
  const mix = (index) => a[index] + (b[index] - a[index]) * k;
  return { index: i, x: mix(1), y: mix(2), theta: mix(3), ox: mix(4), oy: mix(5), otheta: mix(6) };
}

const degrees = (rad) => {
  let d = (rad * 180) / Math.PI;
  d = ((d + 180) % 360 + 360) % 360 - 180;
  return d;
};

// ---------------------------------------------------------------------------------------------
// Layout: fit the field into the space the bars leave free, and ease it when the panel moves
// ---------------------------------------------------------------------------------------------

let padRight = 0;
function layout(dt) {
  const w = $('view').clientWidth, h = $('view').clientHeight;
  if (!w || !h) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (renderer.domElement.width !== Math.round(w * dpr) || renderer.domElement.height !== Math.round(h * dpr)) {
    renderer.setPixelRatio(dpr);
    renderer.setSize(w, h, false);
    if (scene3d) scene3d.resize(w, h);
  }
  const wantRight = panelOpen && w > 760 ? 344 : 0;
  padRight += (wantRight - padRight) * (1 - Math.exp(-dt * 9));
  const padTop = 68, padBottom = 84, margin = 28;
  const S = run ? run.field.size : 3657.6;
  const scale = Math.max(0.01, Math.min((w - padRight - margin * 2) / S, (h - padTop - padBottom - margin * 2) / S));
  camera.left = -w / 2 / scale; camera.right = w / 2 / scale;
  camera.top = h / 2 / scale; camera.bottom = -h / 2 / scale;
  // put the field centre in the middle of the free area
  const dx = (0 - padRight) / 2, dy = (padTop - padBottom) / 2; // free-area centre minus screen centre, in px
  camera.position.x = S / 2 - dx / scale;
  camera.position.y = -S / 2 + dy / scale;
  camera.updateProjectionMatrix();
  camera.updateMatrixWorld();

  const v = new THREE.Vector3();
  for (const l of labels) {
    v.set(l.x, -l.y, 0).project(camera);
    l.el.style.left = `${((v.x + 1) / 2) * w}px`;
    l.el.style.top = `${((1 - v.y) / 2) * h}px`;
  }
}

// ---------------------------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------------------------

const readoutRows = {};
function buildReadout() {
  const dl = $('readout');
  dl.textContent = '';
  for (const [key, label] of [['true', 'Robot (true)'], ['odo', 'Odometry'], ['gap', 'Odometry error'], ['result', 'Run']]) {
    const dt = document.createElement('dt'); dt.textContent = label;
    const dd = document.createElement('dd');
    dl.append(dt, dd);
    readoutRows[key] = dd;
  }
}

const fmtPose = (x, y, th) => `${x.toFixed(0)}, ${y.toFixed(0)}, ${degrees(th).toFixed(1)}°`;

function updateReadout(pose) {
  readoutRows.true.textContent = fmtPose(pose.x, pose.y, pose.theta);
  readoutRows.odo.textContent = Number.isFinite(pose.ox) ? fmtPose(pose.ox, pose.oy, pose.otheta) : '—';
  const gap = Math.hypot(pose.x - pose.ox, pose.y - pose.oy);
  readoutRows.gap.textContent = Number.isFinite(gap) ? `${gap.toFixed(0)} mm` : '—';
  readoutRows.result.textContent = run.finished ? `${run.used.toFixed(1)} s, finished` : 'did not finish';
}

const ballRows = {};
function updateBallPanel(positions, pose) {
  if (!positions.length) return;
  const counts = ballCounts(positions, pose);
  const dl = $('ballCounts');
  if (!Object.keys(ballRows).length) {
    dl.textContent = '';
    for (const label of ['On the field', ...Object.keys(counts)]) {
      const dt = document.createElement('dt'); dt.textContent = label;
      const dd = document.createElement('dd');
      dl.append(dt, dd);
      ballRows[label] = dd;
    }
  }
  ballRows['On the field'].textContent = String(positions.length);
  for (const [label, n] of Object.entries(counts)) ballRows[label].textContent = String(n);
  $('ballsSection').hidden = false;
}

let shownTelemetryIndex = -2;
function updateTelemetry() {
  const list = run.telemetry;
  let index = -1;
  for (let lo = 0, hi = list.length - 1; lo <= hi;) {
    const mid = (lo + hi) >> 1;
    if (list[mid].t <= time) { index = mid; lo = mid + 1; } else hi = mid - 1;
  }
  if (index === shownTelemetryIndex) return;
  shownTelemetryIndex = index;
  const dl = $('telemetry');
  dl.textContent = '';
  const items = index >= 0 ? list[index].items : {};
  const keys = Object.keys(items);
  if (!keys.length) {
    const dd = document.createElement('dd'); dd.className = 'empty'; dd.textContent = 'none yet';
    dl.append(dd);
  }
  for (const key of keys.slice(0, 40)) {
    const dt = document.createElement('dt'); dt.textContent = key;
    const dd = document.createElement('dd'); dd.textContent = items[key]; dd.title = items[key];
    dl.append(dt, dd);
  }
}

let nowEvent = -2;
function updateEventHighlight() {
  let index = -1;
  run.events.forEach((e, i) => { if (e.t <= time) index = i; });
  if (index === nowEvent) return;
  nowEvent = index;
  [...$('events').children].forEach((li, i) => li.classList.toggle('now', i === index));
}

function buildRunPanel() {
  const events = $('events');
  events.textContent = '';
  $('eventCount').textContent = run.events.length ? `(${run.events.length})` : '';
  if (!run.events.length) {
    const li = document.createElement('li'); li.className = 'empty'; li.textContent = 'Nothing touched anything.';
    li.style.cursor = 'default';
    events.append(li);
  }
  for (const e of run.events) {
    const li = document.createElement('li');
    const t = document.createElement('span'); t.className = 't'; t.textContent = `${e.t < 0 ? '−' : ''}${Math.abs(e.t).toFixed(2)} s`;
    const text = document.createElement('span'); text.textContent = e.text;
    li.append(t, text);
    li.addEventListener('click', () => { time = Math.max(0, e.t); playing = false; syncPlayButton(); });
    events.append(li);
  }
  $('warningsSection').hidden = run.warnings.length === 0;
  const warnings = $('warnings');
  warnings.textContent = '';
  for (const w of run.warnings) {
    const li = document.createElement('li'); li.textContent = w; warnings.append(li);
  }
  const markers = $('markers');
  markers.textContent = '';
  for (const e of run.events) {
    if (e.t < 0 || e.t > duration) continue;
    const m = document.createElement('i');
    m.className = e.text.startsWith('shot') ? 'shot' : e.text.startsWith('picked') ? 'pick' : e.text.endsWith('tipped') ? 'tip' : e.text.includes('went into') ? 'in' : '';
    m.style.left = `${(e.t / duration) * 100}%`;
    markers.append(m);
  }
  const r = run.field.robot;
  const custom = ['robotFront', 'robotBack', 'robotWidth'].some((id) => run.params && run.params[id] && run.params[id].used !== run.params[id].default);
  const lines = [
    `Robot body: front ${r.front}, back ${r.back}, width ${r.width} mm (${custom ? 'set on the Setup screen' : 'RobotFootprint.java placeholder'})`,
    'Hive leg lean (FieldConfig placeholder): the legs are drawn dashed',
    'Loop times: main 10 ms, odometry 2 ms (SimRunner)',
    'Wheel response, robot mass, battery voltage (SimWorld, RapierWorld)',
  ];
  const list = $('unmeasured');
  list.textContent = '';
  for (const l of lines) { const li = document.createElement('li'); li.textContent = l; list.append(li); }
  shownTelemetryIndex = -2;
  nowEvent = -2;
}

// ---------------------------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------------------------

function syncPlayButton() {
  $('play').textContent = playing ? '❚❚' : '▶';
}

function clampTime() {
  time = Math.min(duration, Math.max(0, time));
}

$('play').addEventListener('click', togglePlay);
function setViewMode(mode) {
  if (mode === viewMode || !run) return;
  const canvas = $('view');
  canvas.style.opacity = '0';                         // a quick fade hides the switch between the two cameras
  setTimeout(() => {
    viewMode = mode;
    document.body.dataset.mode = mode;
    document.querySelectorAll('#viewMode button').forEach((b) => b.setAttribute('aria-pressed', b.dataset.mode === mode ? 'true' : 'false'));
    if (scene3d) scene3d.controls.enabled = view === 'replay' && mode === '3d';
    canvas.style.opacity = '1';
  }, 160);
}
document.querySelectorAll('#viewMode button').forEach((b) => b.addEventListener('click', () => setViewMode(b.dataset.mode)));
$('resetCamera').addEventListener('click', () => scene3d && scene3d.resetCamera());

function togglePlay() {
  if (!run) return;
  if (!playing && time >= duration - 1e-3) time = 0;
  playing = !playing;
  syncPlayButton();
}
$('back').addEventListener('click', () => step(-1));
$('forward').addEventListener('click', () => step(1));
function step(direction, big = false) {
  playing = false;
  time += direction * FRAME_SECONDS * (big ? 10 : 1);
  clampTime();
  syncPlayButton();
}
$('scrubber').addEventListener('input', (e) => { time = Number(e.target.value); scrubbing = true; });
$('scrubber').addEventListener('change', () => { scrubbing = false; });
$('speed').addEventListener('change', (e) => { speed = Number(e.target.value); });
$('togglePanel').addEventListener('click', togglePanel);
function togglePanel() {
  panelOpen = !panelOpen;
  document.body.dataset.panel = panelOpen ? 'open' : 'closed';
}
$('run').addEventListener('click', () => requestRun());
$('opmode').addEventListener('change', () => { selectOpMode($('opmode').value); requestRun(); });
$('watch').addEventListener('change', (e) => post('/api/watch', { enabled: e.target.checked }));
$('watchLanding').addEventListener('change', (e) => post('/api/watch', { enabled: e.target.checked }));

function requestRun(extra = {}) {
  return post('/api/run', { opMode: selectedOpMode || $('opmode').value, ...extra });
}
function post(url, body) {
  return fetch(url, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) }).catch(() => {});
}

window.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  const tag = e.target.tagName;
  if (tag === 'SELECT' || tag === 'TEXTAREA') return;
  if (view !== 'replay') {
    if (e.key === 'Enter' && view === 'landing' && tag !== 'BUTTON') openSetup();
    else if (e.key === 'Enter' && view === 'setup' && tag !== 'BUTTON') startFromSetup();
    else if (e.key === 'Escape' && view === 'setup') go('landing');
    else if ((e.key === 'r' || e.key === 'R') && view === 'stats') requestRun();
    return;
  }
  if (e.key === ' ' && tag !== 'BUTTON') { e.preventDefault(); togglePlay(); }
  else if (e.key === 'ArrowLeft') { e.preventDefault(); step(-1, e.shiftKey); }
  else if (e.key === 'ArrowRight') { e.preventDefault(); step(1, e.shiftKey); }
  else if (e.key === 'Home') { time = 0; }
  else if (e.key === 'End') { time = duration; }
  else if (e.key === 'p' || e.key === 'P') togglePanel();
  else if (e.key === 'v' || e.key === 'V') setViewMode(viewMode === '2d' ? '3d' : '2d');
  else if (e.key === 'r' || e.key === 'R') requestRun();
  else if (e.key === '[' || e.key === ']') {
    const select = $('speed');
    const next = Math.min(select.options.length - 1, Math.max(0, select.selectedIndex + (e.key === ']' ? 1 : -1)));
    select.selectedIndex = next;
    speed = Number(select.value);
  }
});

// ---------------------------------------------------------------------------------------------
// Server status
// ---------------------------------------------------------------------------------------------

const LABELS = { idle: 'Ready', building: 'Compiling…', running: 'Running…', done: 'Up to date', 'compile-error': 'Build failed',
  'run-error': 'Run failed', offline: 'Server not reachable' };

let loadingRunId = -1;
let booted = false;      // the first state from the server has been handled

function applyState(state) {
  serverState = state;
  const phase = state.phase;
  $('status').dataset.phase = phase;
  $('statusText').textContent = LABELS[phase] || phase;
  const busy = phase === 'building' || phase === 'running';
  document.body.dataset.busy = busy ? 'true' : 'false';
  $('run').disabled = busy;
  $('watch').checked = state.watching;
  $('watchLanding').checked = state.watching;
  if (state.opMode && selectedOpMode !== state.opMode) selectOpMode(state.opMode);
  updatePipeline(phase);

  const failedPhase = phase === 'compile-error' || phase === 'run-error';
  if (failedPhase && launching) endLaunch();

  const overlay = $('overlay');
  if (failedPhase || phase === 'offline') {
    $('overlayTitle').textContent = phase === 'compile-error' ? 'The build failed'
      : phase === 'run-error' ? 'The auto stopped with an error' : 'Cannot reach the simulator server';
    $('overlayBody').textContent = phase === 'offline' ? 'Start it with: sim/app/start.sh  (it retries by itself)' : state.message;
    overlay.hidden = false;
  } else {
    overlay.hidden = true;
  }

  // show the latest finished run behind any error card, so the picture is never lost while you fix something
  if (state.runId > 0 && state.runId !== loadedRunId && state.runId !== loadingRunId) {
    const wasLaunching = launching;
    const firstLoad = !run;
    loadRun(state.runId).then(() => {
      if (wasLaunching && launching) finishLaunch();
      else if (!firstLoad && run) toast(`Updated · ${analysis.verdictText} · ${analysis.numbers.contacts} contact${analysis.numbers.contacts === 1 ? '' : 's'}`);
    });
  } else if (state.runId === 0 && !booted) {
    booted = true;
    route();
  }
  if (!paramDefaults && (phase === 'done' || phase === 'run-error')) loadParams();
}

async function loadRun(runId) {
  loadingRunId = runId;
  try {
    const response = await fetch('/api/run');
    if (!response.ok) return;
    const data = await response.json();
    if (!data.frames || !data.frames.length) return;
    run = data;
    loadedRunId = runId;
    duration = run.frames[run.frames.length - 1][0];
    $('scrubber').max = duration;
    $('duration').textContent = `${duration.toFixed(2)} s`;
    if (loadedOnce === false) {
      // ?t=12 opens the replay at 12 s into the match; ?play=1 starts playing
      const params = new URLSearchParams(location.search);
      if (params.has('t')) time = Number(params.get('t')) || 0;
      if (params.get('play') === '1') { playing = true; syncPlayButton(); }
      if (params.get('mode') !== '2d') setTimeout(() => setViewMode('3d'), 50); // 3D is the default; ?mode=2d or V switches
      if (params.get('panel') === '0') { panelOpen = false; document.body.dataset.panel = 'closed'; }
      loadedOnce = true;
    }
    clampTime();
    buildScene();
    buildRunPanel();
    analysis = analyse(run);
    refreshStats();
    updateLanding();
    if (!booted) { booted = true; route(); }
  } catch (error) {
    console.error(error);
  } finally {
    loadingRunId = -1;
  }
}

// ---------------------------------------------------------------------------------------------
// Views: landing, stats, replay. The address (#/, #/stats, #/replay) says which one is showing.
// ---------------------------------------------------------------------------------------------

function parseHash() {
  const name = location.hash.replace(/^#\/?/, '');
  return name === 'stats' || name === 'replay' || name === 'setup' ? name : 'landing';
}

function go(next) {
  location.hash = next === 'landing' ? '#/' : `#/${next}`;
}

function route() {
  let next = parseHash();
  if (next !== 'landing' && next !== 'setup' && !run) {
    if (!booted) return;     // a deep link waits for the first run to load
    next = 'landing';
    history.replaceState(null, '', '#/');
  }
  setView(next);
}

function setView(next) {
  if (next === view && document.body.dataset.view === next) return;
  view = next;
  document.body.dataset.view = next;
  $('landing').toggleAttribute('inert', next !== 'landing');
  $('stats').toggleAttribute('inert', next !== 'stats');
  $('setup').toggleAttribute('inert', next !== 'setup');
  renderUntil = performance.now() + 900;
  if (scene3d) scene3d.controls.enabled = next === 'replay' && viewMode === '3d';
  if (next === 'stats') enterStats();
  if (next === 'setup') enterSetup();
  if (next === 'landing') { $('landing').scrollTop = 0; }
  updateTabs();
}

function updateTabs() {
  const indicator = $('tabIndicator');
  let active = null;
  document.querySelectorAll('#tabs a').forEach((a) => {
    const on = a.dataset.tab === view;
    if (on) { a.setAttribute('aria-current', 'page'); active = a; } else a.removeAttribute('aria-current');
  });
  if (!active) { indicator.style.opacity = 0; return; }
  indicator.style.opacity = 1;
  indicator.style.width = `${active.offsetWidth}px`;
  indicator.style.transform = `translateX(${active.offsetLeft - 3}px)`;
}

function enterStats() {
  const el = $('stats');
  el.scrollTop = 0;
  refreshStats();
}

function refreshStats() {
  if (!run || !analysis) return;
  const el = $('stats');
  el.classList.remove('entering');
  renderStats($('statsBody'), analysis, run, {
    seek: (t) => { time = Math.max(0, Math.min(duration, t)); playing = false; syncPlayButton(); go('replay'); },
    replay: () => { time = 0; playing = true; syncPlayButton(); go('replay'); },
    rerun: () => requestRun(),
  });
  void el.offsetWidth;       // restart the entrance animations
  el.classList.add('entering');
}


// ---------------------------------------------------------------------------------------------
// Setup: the numbers you can change before a run
// ---------------------------------------------------------------------------------------------

let loadingParams = false;
async function loadParams() {
  if (loadingParams) return;
  loadingParams = true;
  try {
    const data = await (await fetch('/api/params')).json();
    paramDefs = data.defs;
    paramDefaults = data.defaults;
    if (data.defaults) paramValues = { ...data.defaults, ...data.values };
    buildParams();
  } catch (error) {
    console.error(error);
  } finally {
    loadingParams = false;
  }
}

const paramControls = {};   // id -> { range, number, row, box }

function buildParams() {
  const list = $('paramList');
  list.textContent = '';
  for (const key of Object.keys(paramControls)) delete paramControls[key];
  $('paramWait').hidden = Boolean(paramDefaults);
  if (!paramDefaults) return;

  const OPEN_BY_DEFAULT = new Set(['Robot size', 'Shooting']);
  for (const group of [...new Set(paramDefs.map((d) => d.group))]) {
    const section = document.createElement('div');
    section.className = 'paramGroup';
    section.dataset.group = group;
    section.dataset.open = String(OPEN_BY_DEFAULT.has(group));
    const head = document.createElement('button');
    head.type = 'button';
    head.className = 'groupHead';
    head.setAttribute('aria-expanded', section.dataset.open);
    const title = document.createElement('span'); title.textContent = group;
    const badge = document.createElement('span'); badge.className = 'groupBadge';
    const chevron = document.createElement('i'); chevron.className = 'chevron';
    head.append(title, badge, chevron);
    head.addEventListener('click', () => {
      const open = section.dataset.open !== 'true';
      section.dataset.open = String(open);
      head.setAttribute('aria-expanded', String(open));
    });
    const body = document.createElement('div'); body.className = 'groupBody';
    const inner = document.createElement('div'); inner.className = 'groupInner';
    for (const def of paramDefs.filter((d) => d.group === group)) inner.append(paramRow(def));
    body.append(inner);
    section.append(head, body);
    list.append(section);
  }
  syncParamUI();
  snapPreview();
}

function choiceRow(def) {
  const row = document.createElement('div');
  row.className = 'param';
  row.dataset.id = def.id;
  const top = document.createElement('div'); top.className = 'paramTop';
  const left = document.createElement('div');
  const label = document.createElement('label'); label.textContent = def.label;
  const changed = document.createElement('button'); changed.className = 'changed'; changed.type = 'button'; changed.textContent = 'Reset';
  changed.addEventListener('click', () => setParam(def.id, paramDefaults[def.id]));
  left.append(label, changed);
  top.append(left);
  const desc = document.createElement('p'); desc.className = 'desc'; desc.textContent = def.description;
  const group = document.createElement('div'); group.className = 'choice'; group.setAttribute('role', 'radiogroup'); group.setAttribute('aria-label', def.label);
  const buttons = def.options.map((o) => {
    const b = document.createElement('button');
    b.type = 'button'; b.textContent = o.label; b.setAttribute('role', 'radio');
    b.addEventListener('click', () => setParam(def.id, o.value));
    group.append(b);
    return { b, value: o.value };
  });
  row.append(top, desc, group);
  paramControls[def.id] = { choice: buttons, row, def };
  return row;
}

function paramRow(def) {
  if (def.type === 'choice') return choiceRow(def);
  const row = document.createElement('div');
  row.className = 'param';
  row.dataset.id = def.id;

  const top = document.createElement('div'); top.className = 'paramTop';
  const left = document.createElement('div');
  const label = document.createElement('label'); label.textContent = def.label; label.htmlFor = `param-${def.id}`;
  const changed = document.createElement('button'); changed.className = 'changed'; changed.type = 'button'; changed.textContent = 'Reset';
  changed.addEventListener('click', () => setParam(def.id, paramDefaults[def.id]));
  left.append(label, changed);
  top.append(left);

  const desc = document.createElement('p'); desc.className = 'desc'; desc.textContent = def.description;

  const range = document.createElement('input');
  range.type = 'range'; range.id = `param-${def.id}`; range.min = def.min; range.max = def.max; range.step = def.step;
  range.addEventListener('input', () => setParam(def.id, Number(range.value)));

  const box = document.createElement('div'); box.className = 'numberBox';
  const number = document.createElement('input');
  number.type = 'number'; number.min = def.min; number.max = def.max; number.step = def.step; number.inputMode = 'decimal';
  number.setAttribute('aria-label', `${def.label} in ${def.unit}`);
  number.addEventListener('input', () => {
    const v = Number(number.value);
    const ok = number.value !== '' && Number.isFinite(v) && v >= def.min && v <= def.max;
    box.classList.toggle('invalid', !ok);
    if (ok) setParam(def.id, v, number);
  });
  number.addEventListener('blur', () => syncParamUI());
  const unit = document.createElement('span'); unit.textContent = def.unit;
  box.append(number, unit);

  const inputs = document.createElement('div'); inputs.className = 'paramRow';
  inputs.append(range, box);

  const meta = document.createElement('div'); meta.className = 'paramMeta';
  const rangeText = document.createElement('span'); rangeText.textContent = `${def.min}–${def.max} ${def.unit}`;
  const defaultText = document.createElement('span'); defaultText.textContent = `Default ${paramDefaults[def.id]} ${def.unit}`;
  meta.append(rangeText, defaultText);

  row.append(top, desc, inputs, meta);
  paramControls[def.id] = { range, number, row, box, def };
  return row;
}

function setParam(id, value, from = null) {
  paramValues[id] = value;
  syncParamUI(from);
  updatePreview();
}

function syncParamUI(skip = null) {
  for (const [id, c] of Object.entries(paramControls)) {
    const v = paramValues[id];
    if (c.choice) {
      c.choice.forEach(({ b, value }) => b.setAttribute('aria-checked', String(value === v)));
    } else {
      c.range.value = v;
      c.range.style.setProperty('--fill', `${((v - c.def.min) / (c.def.max - c.def.min)) * 100}%`);
      if (skip !== c.number) { c.number.value = v; c.box.classList.remove('invalid'); }
    }
    c.row.dataset.changed = String(v !== paramDefaults[id]);
  }
  document.querySelectorAll('.paramGroup').forEach((g) => {
    const n = g.querySelectorAll('.param[data-changed="true"]').length;
    g.querySelector('.groupBadge').textContent = n ? `${n} changed` : '';
  });
}

function paramsValid() {
  return Object.values(paramControls).every((c) => c.choice || !c.box.classList.contains('invalid'));
}

function resetParams() {
  if (!paramDefaults) return;
  paramValues = { ...paramDefaults };
  syncParamUI();
  updatePreview();
}

function enterSetup() {
  const mode = opmodes.find((m) => m.name === selectedOpMode);
  $('setupEyebrow').textContent = mode ? `${mode.name} · ${mode.alliance} alliance` : 'Set up';
  $('setup').scrollTop = 0;
  if (!paramDefaults) loadParams();
  else { syncParamUI(); snapPreview(); }
}

// ---- the preview: the robot body from above, front up, animated toward the chosen size ----

const SVGNS = 'http://www.w3.org/2000/svg';
let previewShown = null;       // the size being drawn right now { front, back, width }
let previewAnimating = false;
let previewParts = null;

function previewTarget() {
  return { front: paramValues.robotFront, back: paramValues.robotBack, width: paramValues.robotWidth };
}

function el(tag, attrs = {}, parent = null) {
  const node = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.append(node);
  return node;
}

function buildPreviewParts() {
  const svg = $('robotPreview');
  svg.textContent = '';
  previewParts = {
    ghost: el('rect', { class: 'ghostBody', rx: 8 }, svg),
    body: el('rect', { class: 'body', rx: 8 }, svg),
    arrow: el('polyline', { class: 'arrow' }, svg),
    centre: el('circle', { class: 'centre', r: 4 }, svg),
    widthLine: el('line', { class: 'dim' }, svg), widthText: el('text', { class: 'dimText', 'text-anchor': 'middle' }, svg),
    frontLine: el('line', { class: 'dim' }, svg), frontText: el('text', { class: 'dimText' }, svg),
    backLine: el('line', { class: 'dim' }, svg), backText: el('text', { class: 'dimText' }, svg),
    originText: el('text', { class: 'dimText', 'text-anchor': 'end' }, svg),
  };
  previewParts.originText.textContent = 'odometry point';
}

function drawPreview(size) {
  if (!previewParts) buildPreviewParts();
  const d = paramDefaults || size;
  const defaults = { front: d.robotFront ?? size.front, back: d.robotBack ?? size.back, width: d.robotWidth ?? size.width };
  const maxFront = Math.max(size.front, defaults.front), maxBack = Math.max(size.back, defaults.back), maxWidth = Math.max(size.width, defaults.width);
  const scale = Math.min(250 / (maxFront + maxBack), 215 / maxWidth);
  const cx = 232, top = 165 - ((maxFront + maxBack) * scale) / 2, cy = top + maxFront * scale;
  const P = previewParts;
  const box = (r, s) => { r.setAttribute('x', cx - (s.width * scale) / 2); r.setAttribute('y', cy - s.front * scale); r.setAttribute('width', s.width * scale); r.setAttribute('height', (s.front + s.back) * scale); };
  box(P.ghost, defaults);
  box(P.body, size);
  const tipY = cy - size.front * scale, backY = cy + size.back * scale, halfW = (size.width * scale) / 2;
  P.arrow.setAttribute('points', `${cx - 14},${tipY + 26} ${cx},${tipY + 10} ${cx + 14},${tipY + 26}`);
  P.centre.setAttribute('cx', cx); P.centre.setAttribute('cy', cy);
  const topY = Math.min(tipY, cy - defaults.front * scale);
  P.widthLine.setAttribute('x1', cx - halfW); P.widthLine.setAttribute('x2', cx + halfW);
  P.widthLine.setAttribute('y1', topY - 16); P.widthLine.setAttribute('y2', topY - 16);
  P.widthText.setAttribute('x', cx); P.widthText.setAttribute('y', topY - 24);
  P.widthText.textContent = `${Math.round(size.width)} mm wide`;
  const sideX = cx + Math.max(halfW, (defaults.width * scale) / 2) + 22;
  P.frontLine.setAttribute('x1', sideX); P.frontLine.setAttribute('x2', sideX); P.frontLine.setAttribute('y1', cy); P.frontLine.setAttribute('y2', tipY);
  P.frontText.setAttribute('x', sideX + 8); P.frontText.setAttribute('y', (cy + tipY) / 2 + 4);
  P.frontText.textContent = `front ${Math.round(size.front)}`;
  P.backLine.setAttribute('x1', sideX); P.backLine.setAttribute('x2', sideX); P.backLine.setAttribute('y1', cy); P.backLine.setAttribute('y2', backY);
  P.backText.setAttribute('x', sideX + 8); P.backText.setAttribute('y', (cy + backY) / 2 + 4);
  P.backText.textContent = `back ${Math.round(size.back)}`;
  P.originText.setAttribute('x', cx - halfW - 10); P.originText.setAttribute('y', cy + 4);
}

function snapPreview() {
  if (!paramDefaults) return;
  previewShown = previewTarget();
  drawPreview(previewShown);
  updateChecks();
}

function updatePreview() {
  updateChecks();
  if (!previewShown) { snapPreview(); return; }
  if (!previewAnimating) { previewAnimating = true; requestAnimationFrame(stepPreview); }
}

function stepPreview() {
  const target = previewTarget();
  let settled = true;
  for (const k of Object.keys(target)) {
    const diff = target[k] - previewShown[k];
    if (Math.abs(diff) > 0.15) { previewShown[k] += diff * 0.22; settled = false; } else previewShown[k] = target[k];
  }
  drawPreview(previewShown);
  if (settled || view !== 'setup') { previewAnimating = false; if (settled) return; }
  else requestAnimationFrame(stepPreview);
}

// ---- checks that update as you change the size (the Java side still checks again when it runs) ----

function bodyCorners(pose, size) {
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta), h = size.width / 2;
  return [[size.front, h], [size.front, -h], [-size.back, -h], [-size.back, h]]
    .map(([lx, ly]) => [pose.x + lx * c - ly * s, pose.y + lx * s + ly * c]);
}

/** True if the rotated rectangle overlaps the box {minX, minY, maxX, maxY}. Separating-axis test. */
function overlapsBox(corners, theta, box) {
  const boxCorners = [[box.minX, box.minY], [box.maxX, box.minY], [box.maxX, box.maxY], [box.minX, box.maxY]];
  const axes = [[1, 0], [0, 1], [Math.cos(theta), Math.sin(theta)], [-Math.sin(theta), Math.cos(theta)]];
  const project = (pts, [ax, ay]) => { const v = pts.map(([x, y]) => x * ax + y * ay); return [Math.min(...v), Math.max(...v)]; };
  return axes.every((axis) => { const a = project(corners, axis), b = project(boxCorners, axis); return !(a[1] <= b[0] || b[1] <= a[0]); });
}

function updateChecks() {
  const list = $('previewChecks');
  if (!paramDefaults) return;
  const size = previewTarget();
  const mode = opmodes.find((m) => m.name === selectedOpMode);
  const rows = [];
  const mm = (v) => `${Math.round(v)} mm`;
  const inches = (v) => (v / 25.4).toFixed(1);
  rows.push(['info', `Body ${mm(size.front + size.back)} long × ${mm(size.width)} wide (${inches(size.front + size.back)} × ${inches(size.width)} in).`]);
  if (mode) {
    const pose = { x: mode.start.x, y: mode.start.y, theta: (mode.start.headingDeg * Math.PI) / 180 };
    const fieldSize = run ? run.field.size : 3657.6;
    const corners = bodyCorners(pose, size);
    let outside = 0, spare = Infinity;
    for (const [x, y] of corners) {
      outside = Math.max(outside, -x, x - fieldSize, -y, y - fieldSize);
      spare = Math.min(spare, x, fieldSize - x, y, fieldSize - y);
    }
    rows.push(outside > 0.5
      ? ['warn', `At the start pose the body is ${mm(outside)} inside the wall, so the robot gets pushed out and odometry starts that far off.`]
      : ['pass', `Fits inside the field at the start pose, with ${mm(spare)} to spare.`]);
    if (run) {
      const hit = run.field.obstacles.find((o) => overlapsBox(corners, pose.theta, o));
      rows.push(hit ? ['warn', `At the start pose the body is inside ${hit.name}.`] : ['pass', 'Clear of every obstacle at the start pose.']);
    }
  }
  list.textContent = '';
  for (const [status, text] of rows) {
    const li = document.createElement('li');
    li.dataset.status = status;
    const icon = document.createElement('span'); icon.className = 'icon'; icon.textContent = status === 'warn' ? '!' : status === 'info' ? 'i' : '✓';
    if (status === 'info') icon.style.background = 'var(--dim)';
    const span = document.createElement('span'); span.textContent = text;
    li.append(icon, span);
    list.append(li);
  }
}

// ---------------------------------------------------------------------------------------------
// Landing
// ---------------------------------------------------------------------------------------------

function buildCards() {
  const holder = $('opmodeCards');
  holder.textContent = '';
  for (const m of opmodes) {
    const card = document.createElement('button');
    card.className = 'opcard';
    card.setAttribute('role', 'radio');
    card.dataset.name = m.name;
    card.dataset.alliance = m.alliance;
    const title = document.createElement('span'); title.className = 'title';
    const dot = document.createElement('i'); dot.className = 'dotc';
    title.append(dot, m.name);
    const meta = document.createElement('span'); meta.className = 'meta'; meta.style.display = 'block';
    meta.textContent = `${m.alliance} alliance · starts at ${Math.round(m.start.x)}, ${Math.round(m.start.y)} mm, ${m.start.headingDeg}°`;
    card.append(title, meta);
    card.addEventListener('click', () => selectOpMode(m.name));
    holder.append(card);
  }
  selectOpMode(selectedOpMode || (opmodes[0] && opmodes[0].name));
}

function selectOpMode(name) {
  if (!name) return;
  selectedOpMode = name;
  $('opmode').value = name;
  document.querySelectorAll('.opcard').forEach((c) => c.setAttribute('aria-checked', c.dataset.name === name ? 'true' : 'false'));
}

function updateLanding() {
  const has = Boolean(run && analysis);
  $('openLast').hidden = !has || launching;
  if (!has) { $('lastRun').textContent = ''; return; }
  const n = analysis.numbers;
  const when = run.meta && run.meta.finishedAt ? new Date(run.meta.finishedAt).toLocaleTimeString() : '';
  $('lastRun').textContent = `Last run: ${analysis.verdictText} · ${n.contacts} contact${n.contacts === 1 ? '' : 's'} · ${when}`;
}

function openSetup() {
  go('setup');
}

function startFromSetup() {
  if (launching || serverState.phase === 'building' || serverState.phase === 'running') return;
  if (!paramDefaults || !paramsValid()) return;
  launching = true;
  document.body.dataset.launching = 'true';
  $('start').disabled = true;
  $('start').querySelector('.label').textContent = 'Running…';
  updatePipeline('building');
  requestRun({ params: { ...paramValues } });
}

function updatePipeline(phase) {
  if (!launching) {
    document.querySelectorAll('#pipeline li').forEach((li) => { li.dataset.state = ''; });
    return;
  }
  const states = { build: '', sim: '', analyse: '' };
  if (phase === 'building') states.build = 'active';
  else if (phase === 'running') { states.build = 'done'; states.sim = 'active'; }
  else if (phase === 'done') { states.build = 'done'; states.sim = 'done'; states.analyse = 'active'; }
  document.querySelectorAll('#pipeline li').forEach((li) => { li.dataset.state = states[li.dataset.step]; });
}

/** The sim finished and the run is loaded: show the analysing step for a moment, then open the stats. */
function finishLaunch() {
  updatePipeline('done');
  setTimeout(() => {
    document.querySelector('#pipeline li[data-step="analyse"]').dataset.state = 'done';
    setTimeout(() => { endLaunch(); go('stats'); }, 450);
  }, 650);
}

function endLaunch() {
  launching = false;
  document.body.dataset.launching = 'false';
  $('start').disabled = false;
  $('start').querySelector('.label').textContent = 'Start simulation';
  updatePipeline(serverState.phase);
  updateLanding();
}

$('landingPlay').addEventListener('click', openSetup);
$('start').addEventListener('click', startFromSetup);
$('setupBack').addEventListener('click', () => go('landing'));
$('setupReset').addEventListener('click', resetParams);
$('openLast').addEventListener('click', () => go('stats'));
window.addEventListener('hashchange', route);
window.addEventListener('resize', updateTabs);

let toastTimer = null;
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 3200);
}

function connect() {
  const source = new EventSource('/api/events');
  source.onmessage = (message) => applyState(JSON.parse(message.data));
  source.onerror = () => applyState({ ...serverState, phase: 'offline', message: '' });
}

async function init() {
  panelOpen = window.innerWidth > 760; // on a narrow screen the panel covers the field, so it starts closed
  document.body.dataset.panel = panelOpen ? 'open' : 'closed';
  document.body.dataset.view = 'landing';
  document.body.dataset.mode = '2d';
  document.body.dataset.launching = 'false';
  $('stats').setAttribute('inert', '');
  $('setup').setAttribute('inert', '');
  padRight = panelOpen ? 344 : 0;
  buildReadout();
  syncPlayButton();
  try {
    opmodes = await (await fetch('/api/opmodes')).json();
    for (const m of opmodes) {
      const option = document.createElement('option');
      option.value = m.name; option.textContent = m.name;
      $('opmode').append(option);
    }
    buildCards();
  } catch { /* the status events will show the problem */ }
  loadParams();
  connect();
  updateTabs();
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { buildScene(); refreshStats(); });
}

// ---------------------------------------------------------------------------------------------
// Frame loop
// ---------------------------------------------------------------------------------------------

let last = performance.now();
function frame(now) {
  // the first frame's timestamp can be earlier than 'last', so dt is never allowed to go negative
  const dt = Math.max(0, Math.min(0.1, (now - last) / 1000));
  last = now;

  if (run && playing) {
    time += dt * speed;
    if (time >= duration) { time = duration; playing = false; syncPlayButton(); }
  }
  layout(dt);

  const drawing = view === 'replay' || performance.now() < renderUntil;
  if (run && robot && drawing) {
    const pose = poseAt(time);
    robot.group.position.set(pose.x, -pose.y, 0.3);
    robot.group.rotation.z = -pose.theta;
    const odoKnown = Number.isFinite(pose.ox);
    ghost.group.visible = odoKnown;
    if (odoKnown) {
      ghost.group.position.set(pose.ox, -pose.oy, 0.2);
      ghost.group.rotation.z = -pose.otheta;
    }
    trails.truth.geometry.setDrawRange(0, pose.index + 1);
    trails.odo.geometry.setDrawRange(0, pose.index + 1);

    const ballPositions = ballsAt(time);
    const tilts = hiveTiltAt(time);
    if (viewMode === '3d' && scene3d) scene3d.update(pose, ballPositions, tilts, pose.index);
    else updateTopBalls(ballPositions, tilts);
    updateBallPanel(ballPositions, pose);

    if (!scrubbing) $('scrubber').value = time;
    $('time').textContent = `${time.toFixed(2)} s`;
    updateReadout(pose);
    updateTelemetry();
    updateEventHighlight();
  }
  if (drawing) {
    if (viewMode === '3d' && scene3d) renderer.render(scene3d.scene, scene3d.camera);
    else renderer.render(scene, camera);
  }
  requestAnimationFrame(frame);
}

window.simCamera3D = (...a) => scene3d && scene3d.setCamera(...a); // for tests: put the 3D camera somewhere
// for inspecting the page from the browser console / tests
window.simDebug = () => ({ camera: { l: camera.left, r: camera.right, t: camera.top, b: camera.bottom, x: camera.position.x, y: camera.position.y }, padRight, time, w: $('view').clientWidth, h: $('view').clientHeight });

init();
requestAnimationFrame(frame);
