// The page. It draws what the simulator recorded: the field, the robot, its path, and the events.
// It does NOT run any robot code: Java runs the auto and the server hands over the result.
//
// Field frame (same as the robot code): x, y in mm, y points DOWN in the manual's top view for Red.
// The scene uses (x, -y) so the picture matches that top view. Robot heading theta is counter-clockwise
// in the field frame, so the scene rotation is -theta.
//
// Views: #/ (home), #/run (the workspace: rail + field + details drawer + timeline), #/params.

import * as THREE from 'three';
import { analyse, diffRuns, estimatedChecks, nextProblem, THRESHOLDS, fmtLen } from './stats.js';
import { renderRail, errorCard, h } from './rail.js';
import { mountTimeline } from './timeline.js';
import { mountDetails, gapLabel, decideAt } from './details.js';
import { chipFor } from './runState.js';
import { mountParams } from './params.js';
import { mountHome } from './home.js';
import { build3D } from './scene3d.js';
import { badge, icon } from './icons.js';

const $ = (id) => document.getElementById(id);
const FRAME_SECONDS = 0.02;
const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];

// ---------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------

let run = null;          // the latest run (JSON from /api/run)
let prevRun = null;      // the run before it, for "changed since"
let analysis = null;     // numbers and checks for the latest run (stats.js)
let prevAnalysis = null;
let loadedRunId = -1;
let loadingRunId = -1;
let time = 0;            // match seconds
let playing = false;
let speed = 1;
let duration = 30;
let serverState = { phase: 'idle', message: '', runId: 0, watching: true, since: Date.now(), lastSaveAt: 0, error: null };
let view = 'home';       // home | run | params
let viewMode = '3d';     // 2d | 3d
let detailsOpen = false;
let railView = { view: 'summary', checkId: null };
let opmodes = [];
let selectedOpMode = null;
let paramDefs = [];
let paramDefaults = null;    // the code's own values, from Java; null until the first build
let paramValues = {};
let partners = [];           // path files from /api/partners
let partnerChoice = { mode: 'none', file: null, speedPct: 100 };
let deepLink = null;         // query from the address, applied once when the first run loads
let firstRoute = true;
let renderUntil = 0;
const verdicts = new Map();  // opMode -> { status, id } of the last run of each auto

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
let partner = null;      // { group, frames } R2 in the top view
let trails = null;       // { truth, odo, count }
let ballLayer = null;    // { discs, cells } the balls and Hive Cells in the top view
let scene3d = null;      // the 3D view (scene3d.js)
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

/** A robot footprint in the robot frame: x forward, y left. */
function robotShape({ front, back, width }, { fill, line, opacity, dashed = false }) {
  const group = new THREE.Group();
  const hw = width / 2;
  const body = [[front, hw], [front, -hw], [-back, -hw], [-back, hw]].map(([x, y]) => new THREE.Vector3(x, -y, 0)); // y flipped for the scene
  if (fill) {
    const shape = new THREE.Shape(body.map((p) => new THREE.Vector2(p.x, p.y)));
    group.add(new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color: fill, transparent: true, opacity, depthWrite: false })));
  }
  group.add(outline(body, line, dashed, fill ? 1 : 0.95));
  // a small arrow on the front edge shows which way the robot faces
  const arrow = [new THREE.Vector3(front - 70, -hw * 0.45, 0), new THREE.Vector3(front + 30, 0, 0), new THREE.Vector3(front - 70, hw * 0.45, 0)];
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(arrow), new THREE.LineBasicMaterial({ color: line })));
  return group;
}

function buildScene({ keepCamera = false } = {}) {
  const savedCamera = keepCamera && scene3d ? scene3d.getCamera() : null;
  const wasFollowing = keepCamera && scene3d ? scene3d.isFollowing() : false;
  if (world) {
    scene.remove(world);
    world.traverse((o) => { o.geometry?.dispose(); o.material?.dispose(); });
  }
  labels.forEach(({ el }) => el.remove());
  labels = [];
  world = new THREE.Group();
  scene.add(world);
  robot = ghost = partner = trails = ballLayer = null;
  if (!run) return;

  const f = run.field;
  const S = f.size;
  const c = { field: color('--floor'), line: color('--wall'), tile: color('--grid'), accent: color('--accent'),
    ghost: color('--body'), red: color('--red'), blue: color('--blue'), obstacle: color('--obstacle') };

  // floor, tile grid (24 in tiles), wall, midline
  rect(world, [0, 0, S, S], { fill: c.field, fillOpacity: 1, line: c.line, z: -1 });
  const tile = 609.6;
  const grid = [];
  for (let k = 1; k < 6; k++) grid.push([[k * tile, 0], [k * tile, S]], [[0, k * tile], [S, k * tile]]);
  segments(world, grid, c.tile, false, 0.9);
  segments(world, [[[f.midline, 0], [f.midline, S]]], c.line, true, 0.6);

  // Hive: frame outline, red cells on the left half, blue on the right (manual top view)
  if (f.hive) {
    const hv = f.hive;
    const mid = (hv.minX + hv.maxX) / 2;
    rect(world, [hv.minX, hv.minY, mid, hv.maxY], { fill: c.red, fillOpacity: 0.07 });
    rect(world, [mid, hv.minY, hv.maxX, hv.maxY], { fill: c.blue, fillOpacity: 0.07 });
    rect(world, [hv.minX, hv.minY, hv.maxX, hv.maxY], { line: c.line, lineOpacity: 0.6 });
    addLabel('Hive', mid, (hv.minY + hv.maxY) / 2);
  }

  // obstacles. Dashed = the size is a placeholder in FieldConfig (the leg lean), so it is unverified.
  for (const o of f.obstacles) {
    const isLeg = o.name.startsWith('hive-leg');
    rect(world, [o.minX, o.minY, o.maxX, o.maxY], { fill: c.obstacle, fillOpacity: isLeg ? 0.5 : 0.35, line: c.obstacle, dashed: isLeg, z: 0.02 });
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
  trails = { truth: trail(1, c.accent, 0.8), odo: trail(4, c.ghost, 0.45), count: n };

  // R2 (the partner): a darker fill of the alliance colour, with its path dashed
  if (run.partner && run.partner.frames.length) {
    const rc = run.alliance === 'blue' ? c.blue : c.red;
    const dark = rc.clone().multiplyScalar(0.45);
    partner = { group: robotShape(run.partner.robot, { fill: dark, line: rc, opacity: 0.85 }), frames: run.partner.frames };
    partner.group.position.z = 0.25;
    const pts = run.partner.frames.map((fr) => new THREE.Vector3(fr[1], -fr[2], 0.04));
    const path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color: rc, dashSize: 40, gapSize: 30, transparent: true, opacity: 0.5 }));
    path.computeLineDistances();
    world.add(path, partner.group);
    addLabel('R2', run.partner.frames[0][1], run.partner.frames[0][2] - 240);
  }

  ghost = { group: robotShape(f.robot, { line: c.ghost, dashed: true }) };
  ghost.group.position.z = 0.4;
  robot = { group: robotShape(f.robot, { fill: c.accent, line: c.accent, opacity: 0.45 }) };
  robot.group.position.z = 0.3;
  world.add(ghost.group, robot.group);

  buildBalls(c);
  if (scene3d) scene3d.dispose();
  scene3d = build3D(run, c, $('view'));
  scene3d.resize($('view').clientWidth || 1, $('view').clientHeight || 1);
  if (savedCamera) scene3d.setCamera(...savedCamera);
  if (wasFollowing) scene3d.follow(true);
  syncControls();
}

function syncControls() {
  if (scene3d) scene3d.controls.enabled = (view === 'home') || (view === 'run' && viewMode === '3d');
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
  const p = run.balls.params, hv = run.field.hive;
  const centreX = (hv.minX + hv.maxX) / 2, centreY = (hv.minY + hv.maxY) / 2, offset = (25.5 * 25.4) / 2;
  [[centreX - offset, c.red], [centreX + offset, c.blue]].forEach(([x, col], hive) => {
    for (const side of [1, -1]) {
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.2, depthWrite: false }));
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

/** Counts of where the balls are right now, for the drawer. */
function ballCounts(positions, pose) {
  const f = run.field, hive = f.hive, S = f.size, rb = f.robot;
  const flowers = f.obstacles.filter((o) => o.name.startsWith('flower-')).map((o) => [(o.minX + o.maxX) / 2, (o.minY + o.maxY) / 2]);
  const counts = { 'On the field': positions.length, 'In the robot': 0, 'In a Flower': 0, 'In a Garden': 0, 'In a Hive Cell': 0, 'On the floor': 0, 'In the air': 0 };
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

/** R2's pose at t from its recorded frames [t, x, y, theta], or null. */
function partnerPoseAt(t) {
  if (!partner) return null;
  const fr = partner.frames;
  const i = frameIndex(fr, t);
  const a = fr[i], b = fr[Math.min(i + 1, fr.length - 1)];
  const k = b[0] > a[0] ? Math.min(1, Math.max(0, (t - a[0]) / (b[0] - a[0]))) : 0;
  const dth = Math.atan2(Math.sin(b[3] - a[3]), Math.cos(b[3] - a[3]));
  return { x: a[1] + (b[1] - a[1]) * k, y: a[2] + (b[2] - a[2]) * k, theta: a[3] + dth * k };
}

// ---------------------------------------------------------------------------------------------
// Layout: the canvas fills its host (the field area or the home stage); the camera fits the field into it
// ---------------------------------------------------------------------------------------------

function layout() {
  const canvas = $('view');
  const box = canvas.getBoundingClientRect();
  const w = box.width, hgt = box.height;
  if (!w || !hgt) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (renderer.domElement.width !== Math.round(w * dpr) || renderer.domElement.height !== Math.round(hgt * dpr)) {
    renderer.setPixelRatio(dpr);
    renderer.setSize(w, hgt, false);
    if (scene3d) scene3d.resize(w, hgt);
  }
  const margin = view === 'run' ? 56 : 28;
  const S = run ? run.field.size : 3657.6;
  const scale = Math.max(0.01, Math.min((w - margin * 2) / S, (hgt - margin * 2) / S));
  camera.left = -w / 2 / scale; camera.right = w / 2 / scale;
  camera.top = hgt / 2 / scale; camera.bottom = -hgt / 2 / scale;
  camera.position.x = S / 2;
  camera.position.y = -S / 2;
  camera.updateProjectionMatrix();
  camera.updateMatrixWorld();

  const v = new THREE.Vector3();
  for (const l of labels) {
    v.set(l.x, -l.y, 0).project(camera);
    l.el.style.left = `${((v.x + 1) / 2) * w}px`;
    l.el.style.top = `${((1 - v.y) / 2) * hgt}px`;
  }
}

const effectiveMode = () => (view === 'home' ? '3d' : viewMode);

// ---------------------------------------------------------------------------------------------
// Toolbar: run chip, score chip, auto picker, changed-count pill
// ---------------------------------------------------------------------------------------------

let busyTick = null;
let lastChip = null;

const verdictStatus = () => (analysis ? { clean: 'pass', attention: 'warn', failed: 'fail' }[analysis.verdict] : 'none');

function currentChip() {
  return chipFor(serverState, run, Date.now(), verdictStatus());
}

function chipIcon(chip) {
  if (chip.status === 'busy') return badge('none', 'stale');
  if (chip.status === 'fail') return badge('fail', serverState.phase === 'offline' ? 'offline' : 'fail');
  if (chip.status === 'none') return badge('none', chip.title.includes('out of date') ? 'stale' : chip.title === 'Run on save off' ? 'paused' : 'none');
  return badge(chip.status);
}

function renderChip() {
  const chip = currentChip();
  lastChip = chip;
  const one = (c) => {
    const el = h('div', { class: 'runchip', 'data-status': c.status }, chipIcon(c), h('b', { text: c.title }), c.sub ? h('span', { class: 'sub', text: c.sub }) : null);
    if (c.action === 'retry') el.append(h('button', { class: 'retry', onclick: () => retry() }, 'Retry'));
    return el;
  };
  $('runChip').replaceChildren(one(chip), ...(chip.extra ? [one(chip.extra)] : []));
  const problem = ['compile-error', 'run-error', 'offline'].includes(serverState.phase);
  $('fieldArea').classList.toggle('old', chip.old);
  $('timeline').classList.toggle('old', chip.old);
  $('scoreChip').classList.toggle('old', chip.old);
  const note = $('fieldNote');
  if (serverState.phase === 'compile-error' && run) { note.hidden = false; note.textContent = `Replay of old run #${run.meta.runId} · still works while you fix the build`; }
  else note.hidden = true;
  const busy = serverState.phase === 'building' || serverState.phase === 'running';
  document.body.dataset.busy = busy ? 'true' : 'false';
  $('run').disabled = busy;
  if (busy && !busyTick) busyTick = setInterval(renderChip, 1000);
  if (!busy && busyTick) { clearInterval(busyTick); busyTick = null; }
  return problem;
}

function retry() {
  if (serverState.phase === 'offline') { connect(); return; }
  requestRun();
}

let scoreOpen = false;
let shownScoreKey = '';
function renderScoreChip() {
  const chip = $('scoreChip');
  if (!analysis) { chip.hidden = true; return; }
  chip.hidden = false;
  const s = analysis.score;
  chip.replaceChildren(
    h('span', { class: 'cap' }, h('span', { text: 'AUTO SCORE' }), h('span', { id: 'scoreAt', text: 'at 0.0 s' })),
    h('span', { class: 'big', id: 'scoreNow', text: String(s.at(time)) }),
    h('span', { class: 'fin' }, 'final ', h('b', { text: String(s.final) }), s.est ? h('span', { class: 'est', text: '~' }) : null),
    icon('chevron'));
  shownScoreKey = '';
}

function updateScoreChip() {
  if (!analysis) return;
  const pts = analysis.score.at(time);
  const key = `${pts}|${time.toFixed(1)}`;
  if (key === shownScoreKey) return;
  shownScoreKey = key;
  const now = $('scoreNow'), at = $('scoreAt');
  if (now) now.textContent = String(pts);
  if (at) at.textContent = `at ${time.toFixed(1)} s`;
  if (scoreOpen) renderScorePopover();
}

function renderScorePopover() {
  const s = analysis.score;
  $('scorePop').replaceChildren(
    h('h3', { class: 'h', text: 'Auto score' }),
    ...s.items.map((i) => h('div', { class: `row${i.t > time ? ' after' : ''}` },
      h('span', { class: 'm', text: `${i.t.toFixed(1)} s` }),
      h('span', {}, i.label, i.t > time ? h('span', { class: 'dim', text: ' · after playhead' }) : null),
      h('span', { class: 'pill pass' }, `+${i.points} ${i.kind.toUpperCase()}`, i.est ? h('span', { class: 'est', text: '~' }) : null))),
    s.items.length ? null : h('p', { class: 'dim', text: 'Nothing scored.' }),
    s.est ? h('p', { class: 'foot', text: 'The tip rests on the unmeasured tip mass, so the total carries ~.' }) : null,
    h('p', { class: 'foot', text: 'Point values are the design’s, not yet confirmed against the manual.' }));
}

function setScoreOpen(open) {
  scoreOpen = open && Boolean(analysis);
  const pop = $('scorePop');
  $('scoreChip').setAttribute('aria-expanded', String(scoreOpen));
  if (scoreOpen) {
    renderScorePopover();
    const r = $('scoreChip').getBoundingClientRect();
    pop.style.left = `${Math.max(8, Math.min(window.innerWidth - 368, r.left))}px`;
    pop.hidden = false;
    requestAnimationFrame(() => pop.classList.add('open'));
  } else {
    pop.classList.remove('open');
    setTimeout(() => { if (!scoreOpen) pop.hidden = true; }, 200);
  }
}

$('scoreChip').addEventListener('click', () => setScoreOpen(!scoreOpen));
document.addEventListener('pointerdown', (e) => { if (scoreOpen && !$('scorePop').contains(e.target) && !$('scoreChip').contains(e.target)) setScoreOpen(false); });

function renderAutoPicker() {
  const m = opmodes.find((x) => x.name === selectedOpMode);
  $('autoName').textContent = m ? m.name : '—';
  const tag = $('autoTag');
  tag.textContent = m ? m.alliance.toUpperCase() : '';
  tag.className = `tag ${m ? (m.alliance === 'red' ? 'red' : 'blue') : ''}`;
  $('opmode').value = selectedOpMode || '';
}

function selectOpMode(name) {
  if (!name) return;
  selectedOpMode = name;
  renderAutoPicker();
  homeUI.render();
}

function updateChangedPill() {
  const n = paramDefaults ? paramDefs.filter((d) => paramValues[d.id] !== paramDefaults[d.id]).length : 0;
  const pill = $('changedPill');
  pill.hidden = n === 0;
  pill.className = 'pill count';
  pill.textContent = String(n);
}

// ---------------------------------------------------------------------------------------------
// Rail
// ---------------------------------------------------------------------------------------------

function guessWords() {
  const ids = new Set();
  for (const d of paramDefs) if (d.unmeasured) for (const a of d.affects || []) ids.add(a);
  ids.delete('score');
  const words = { shots: 'shots', tips: 'the tip', contacts: 'contacts', side: 'the side check' };
  return [...ids].map((i) => words[i] || i).join(' and ');
}

function railProblem() {
  const phase = serverState.phase;
  if (phase === 'compile-error') {
    const e = serverState.error;
    const oldText = run ? `Last good run · #${run.meta.runId} · ${new Date(run.meta.finishedAt).toLocaleTimeString('en-GB', { hour12: false })}` : '';
    const copy = e ? `${e.file}:${e.line}${e.col ? `:${e.col}` : ''} error: ${e.message}\n${e.detail}` : serverState.message;
    return { oldText, card: errorCard({
      title: 'Your code didn’t compile',
      where: e ? `${e.file.split('/').pop()}:${e.line}${e.col ? `:${e.col}` : ''}` : null,
      message: e ? `error: ${e.message}` : null, context: e ? e.context : null, detail: e ? e.detail : null,
      raw: e ? null : serverState.message,
      actions: [{ label: 'Copy error', onclick: () => { navigator.clipboard?.writeText(copy).then(() => toast('Copied'), () => {}); } }, { label: 'Retry build', onclick: () => requestRun() }],
      footer: 'Fix it in Android Studio and save. A new run starts by itself.' }) };
  }
  if (phase === 'run-error') {
    const first = (run && run.error ? run.error : serverState.message || '').split('\n').find((l) => l.trim()) || 'The auto stopped with an error.';
    const last = run && run.error && run.frames.length ? run.frames[run.frames.length - 1][0] : null;
    return { oldText: '', card: errorCard({
      title: 'The auto stopped with an error', where: last !== null ? `at ${last.toFixed(2)} s` : null, raw: first,
      actions: [last !== null ? { label: `Jump to ${last.toFixed(2)} s`, onclick: () => seek(last) } : null, { label: 'Run again', onclick: () => requestRun(), primary: true }].filter(Boolean),
      footer: last !== null ? 'The part that ran is still here to replay.' : null }) };
  }
  if (phase === 'offline') {
    return { oldText: '', card: errorCard({
      title: 'Runner disconnected', raw: 'Can’t reach the sim server. Start it with sim/app/start.sh',
      actions: [{ label: 'Retry', onclick: () => connect(), primary: true }] }) };
  }
  return null;
}

function refreshRail() {
  const root = $('rail');
  const scroll = root.scrollTop;
  const problem = railProblem();
  const chip = lastChip || currentChip();
  renderRail(root, {
    a: analysis, run, prevA: prevAnalysis, prevId: prevRun ? prevRun.meta.runId : null,
    diff: analysis && prevAnalysis ? diffRuns(prevAnalysis, analysis) : [], score: analysis ? analysis.score : null,
    est: estimatedChecks(paramDefs), view: railView.view, checkId: railView.checkId, paramDefs,
    guessCount: paramDefs.filter((d) => d.unmeasured).length, guessAffects: guessWords(),
    problem, stale: chip.old && !problem && Boolean(run), oldId: run ? run.meta.runId : null,
    partnerCheck: analysis ? analysis.checks.some((c) => c.id === 'partner') : false,
  }, {
    seek, rerun: () => requestRun(),
    open: (v, id = null) => { railView = { view: v, checkId: id }; refreshRail(); $('rail').scrollTop = 0; },
  });
  root.scrollTop = scroll;
}

// ---------------------------------------------------------------------------------------------
// Timeline, details, moment chip, legend
// ---------------------------------------------------------------------------------------------

const clampT = (t) => Math.min(duration, Math.max(0, t));
function seek(t) { time = clampT(t); playing = false; syncPlay(); }
function syncPlay() { timelineUI.setPlaying(playing); }

const timelineUI = mountTimeline($('timeline'), {
  duration, onSeek: (t) => seek(t), onPlay: () => togglePlay(), onStep: (dir, big) => step(dir, big),
  onSpeed: (x) => { speed = x; },
});
const detailsUI = mountDetails($('details'), { onClose: () => setDetails(false) });

function setDetails(open) {
  detailsOpen = open;
  document.body.dataset.details = open ? 'open' : 'closed';
  $('details').setAttribute('aria-hidden', String(!open));
  $('details').toggleAttribute('inert', !open);
  renderUntil = performance.now() + 400;
}

function renderLegend() {
  const items = [['', 'True'], ['odo', 'Odometry']];
  if (run && run.planned) items.push(['planned', 'Planned']);
  items.push(['contact', 'Contact']);
  if (run && run.partner) items.push(['r2', 'R2 partner · path file']);
  $('legend').replaceChildren(...items.map(([cls, text]) => h('span', {}, h('i', { class: cls }), text)));
}

function buildTimelineModel() {
  const n = analysis.numbers;
  const p = run.partner ? {
    name: `R2 · ${run.partner.file || run.partner.name}`,
    duration: run.partner.frames.length ? run.partner.frames[run.partner.frames.length - 1][0] : 0,
    closestAt: n.partnerClosestAt,
    closestText: Number.isFinite(n.partnerClosest) ? `Closest to R1: ${fmtLen(n.partnerClosest)} at ${n.partnerClosestAt.toFixed(1)} s` : '',
    closestShort: Number.isFinite(n.partnerClosest) ? `closest ${fmtLen(n.partnerClosest)}` : '',
  } : null;
  return { duration, events: run.events, decisions: analysis.decisions, score: analysis.score, partner: p };
}

// ---------------------------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------------------------

function setViewMode(mode) {
  if (mode === viewMode || !run) return;
  const canvas = $('view');
  canvas.style.opacity = '0';                         // a quick fade hides the switch between the two cameras
  setTimeout(() => {
    viewMode = mode;
    document.body.dataset.mode = effectiveMode();
    document.querySelectorAll('#viewMode button').forEach((b) => b.setAttribute('aria-pressed', b.dataset.mode === mode ? 'true' : 'false'));
    syncControls();
    canvas.style.opacity = '1';
  }, 120);
}
document.querySelectorAll('#viewMode button').forEach((b) => b.addEventListener('click', () => setViewMode(b.dataset.mode)));
$('resetCamera').addEventListener('click', () => scene3d && scene3d.resetCamera());

function togglePlay() {
  if (!run) return;
  if (!playing && time >= duration - 1e-3) time = 0;
  playing = !playing;
  syncPlay();
}

function step(direction, big = false) {
  playing = false;
  time = clampT(time + direction * FRAME_SECONDS * (big ? 10 : 1));
  syncPlay();
}

function setSpeed(x) { speed = x; timelineUI.setSpeed(x); }

function toggleFollow() {
  if (!scene3d || effectiveMode() !== '3d') return;
  scene3d.follow(!scene3d.isFollowing());
  toast(scene3d.isFollowing() ? 'Following the robot' : 'Camera is free');
}

$('run').addEventListener('click', () => requestRun());
$('opmode').addEventListener('change', () => { selectOpMode($('opmode').value); requestRun(); });
$('watch').addEventListener('change', (e) => post('/api/watch', { enabled: e.target.checked }));
$('momentChip').addEventListener('click', () => setDetails(true));

function requestRun(extra = {}) {
  const body = { opMode: selectedOpMode, ...extra };
  body.partner = partnerChoice.mode === 'file' && partnerChoice.file ? { file: partnerChoice.file, speedPct: partnerChoice.speedPct } : { file: null, speedPct: partnerChoice.speedPct };
  if (view !== 'run') go('run');
  return post('/api/run', body);
}
function post(url, body) {
  return fetch(url, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) }).then((r) => r.json().catch(() => ({}))).catch(() => ({}));
}

const inField = (e) => ['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName);

window.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  const key = e.key;
  if (key === 'Escape' && inField(e)) { e.target.blur(); return; }
  if (inField(e)) return;
  if (key === '?') { if (!timelineUI.isHelpOpen()) timelineUI.openHelp(); return; }
  if (view === 'params') {
    if (key === '/') { e.preventDefault(); paramsUI.focusSearch(); }
    else if (key === 'Escape') go(run ? 'run' : 'home');
    return;
  }
  if (key === 'r' || key === 'R') { e.preventDefault(); requestRun(); return; }
  if (view === 'home') { if (key === 'Enter' && e.target.tagName !== 'BUTTON') requestRun(); return; }
  if (key === ' ' && e.target.tagName !== 'BUTTON') { e.preventDefault(); togglePlay(); }
  else if (key === 'ArrowLeft') { e.preventDefault(); step(-1, e.shiftKey); }
  else if (key === 'ArrowRight') { e.preventDefault(); step(1, e.shiftKey); }
  else if (key === 'Home') seek(0);
  else if (key === 'End') seek(duration);
  else if (key === 'v' || key === 'V') setViewMode(viewMode === '2d' ? '3d' : '2d');
  else if (key === 'f' || key === 'F') toggleFollow();
  else if (key === 'd' || key === 'D') setDetails(!detailsOpen);
  else if (key === '/') { e.preventDefault(); go('params'); }
  else if (key === 'Escape') {
    if (timelineUI.isHelpOpen()) return;
    if (scoreOpen) setScoreOpen(false);
    else if (detailsOpen) setDetails(false);
    else if (railView.view !== 'summary') { railView = { view: 'summary', checkId: null }; refreshRail(); }
  } else if ((key === 'n' || key === 'N') && analysis) {
    const p = nextProblem(analysis, time, e.shiftKey ? -1 : 1);
    if (p) { railView = { view: 'check', checkId: p.id }; refreshRail(); if (p.seek !== null) seek(p.seek); }
  } else if (key === '[' || key === ']') {
    const i = SPEEDS.indexOf(speed);
    setSpeed(SPEEDS[Math.min(SPEEDS.length - 1, Math.max(0, (i < 0 ? 2 : i) + (key === ']' ? 1 : -1)))]);
  }
});

// ---------------------------------------------------------------------------------------------
// Server status
// ---------------------------------------------------------------------------------------------

let source = null;

function applyState(state) {
  serverState = state;
  $('watch').checked = state.watching;
  if (state.opMode && selectedOpMode !== state.opMode) selectOpMode(state.opMode);
  renderChip();
  if (state.runId > 0 && state.runId !== loadedRunId && state.runId !== loadingRunId) {
    const firstLoad = !run;
    loadRun(state.runId).then(() => { if (!firstLoad && run && analysis) toast(`Updated · ${analysis.verdictText} · ${analysis.numbers.contacts} contact${analysis.numbers.contacts === 1 ? '' : 's'}`); });
  } else {
    refreshRail();
  }
  if (!paramDefaults && (state.phase === 'done' || state.phase === 'run-error')) loadParams();
  homeUI.render();
}

async function loadRun(runId) {
  loadingRunId = runId;
  try {
    const [response, prevResponse] = await Promise.all([fetch('/api/run'), fetch('/api/run?prev=1')]);
    if (!response.ok) return;
    const data = await response.json();
    if (!data.frames || !data.frames.length) return;
    const prev = prevResponse.ok ? await prevResponse.json() : null;
    const hadScene = run !== null;
    run = data;
    prevRun = prev && prev.frames && prev.frames.length ? prev : null;
    loadedRunId = runId;
    duration = run.frames[run.frames.length - 1][0];
    analysis = analyse(run);
    prevAnalysis = prevRun ? analyse(prevRun) : null;
    verdicts.set(run.opMode, { status: verdictStatus(), id: run.meta.runId });
    if (deepLink) applyDeepLink(deepLink);
    time = clampT(time);
    buildScene({ keepCamera: hadScene });
    timelineUI.setRun(buildTimelineModel());
    timelineUI.setTime(time);
    detailsUI.setRun(run);
    renderLegend();
    renderChip();
    renderScoreChip();
    refreshRail();
    updateHome();
  } catch (error) {
    console.error(error);
  } finally {
    loadingRunId = -1;
  }
}

function applyDeepLink(q) {
  deepLink = null;
  if (q.has('t')) time = Number(q.get('t')) || 0;
  if (q.get('play') === '1') { playing = true; syncPlay(); }
  if (q.get('mode') === '2d') {
    viewMode = '2d';
    document.body.dataset.mode = effectiveMode();
    document.querySelectorAll('#viewMode button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === '2d')));
    syncControls();
  }
  if (q.get('details') === '1' || q.get('panel') === '1') setDetails(true);
}

// ---------------------------------------------------------------------------------------------
// Views: home, run, params. The address (#/, #/run, #/params) says which one is showing.
// ---------------------------------------------------------------------------------------------

function parseHash() {
  const raw = location.hash.replace(/^#\/?/, '');
  const [name, qs = ''] = raw.split('?');
  const alias = { stats: 'run', replay: 'run', setup: 'params' }[name] || name;
  const next = alias === 'run' || alias === 'params' ? alias : 'home';
  const query = new URLSearchParams(qs);
  for (const [k, v] of new URLSearchParams(location.search)) if (!query.has(k)) query.set(k, v);   // old links: ?t=12#/replay
  return { view: next, query, redirected: alias !== name && name !== '' };
}

function go(next, query = '') {
  location.hash = next === 'home' ? '#/' : `#/${next}${query ? `?${query}` : ''}`;
}

function route() {
  const { view: next, query, redirected } = parseHash();
  if (redirected) history.replaceState(null, '', `#/${next}${query.toString() ? `?${query}` : ''}`);
  if (firstRoute) {
    firstRoute = false;
    if ([...query.keys()].length) { if (run) applyDeepLink(query); else deepLink = query; }
  }
  setView(next);
}

function placeStage() {
  const host = view === 'home' ? $('homeStage') : $('fieldArea');
  const stage = $('stage');
  if (stage.parentElement !== host) host.prepend(stage);
}

function setView(next) {
  if (next === view && document.body.dataset.view === next) return;
  view = next;
  document.body.dataset.view = next;
  document.body.dataset.mode = effectiveMode();
  $('home').toggleAttribute('inert', next !== 'home');
  $('work').toggleAttribute('inert', next !== 'run');
  $('params').toggleAttribute('inert', next !== 'params');
  placeStage();
  syncControls();
  renderUntil = performance.now() + 900;
  if (next === 'params') paramsUI.show();
  if (next === 'home') updateHome();
  if (next !== 'run') { playing = false; syncPlay(); }
  layout();
}

window.addEventListener('hashchange', route);

let toastTimer = null;
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 3200);
}

// ---------------------------------------------------------------------------------------------
// Parameters and Home glue
// ---------------------------------------------------------------------------------------------

async function loadParams() {
  try {
    const data = await (await fetch('/api/params')).json();
    paramDefs = data.defs;
    paramDefaults = data.defaults;
    if (data.defaults) paramValues = { ...data.defaults, ...data.values };
    updateChangedPill();
    if (view === 'params') paramsUI.show();
    refreshRail();
  } catch (error) {
    console.error(error);
  }
}

async function loadPartners() {
  try { partners = await (await fetch('/api/partners')).json(); } catch { partners = []; }
  try {
    const saved = JSON.parse(localStorage.getItem('sim.partner'));
    if (saved && (saved.mode === 'none' || partners.some((p) => p.file === saved.file))) partnerChoice = { ...partnerChoice, ...saved };
  } catch { /* nothing saved */ }
  homeUI.render();
  if (view === 'params') paramsUI.build();
}

function setPartner(mode, file, speedPct) {
  partnerChoice = { mode, file: mode === 'none' ? null : file, speedPct: speedPct ?? partnerChoice.speedPct };
  try { localStorage.setItem('sim.partner', JSON.stringify(partnerChoice)); } catch { /* fine without */ }
}

function buildPartnerGroup() {
  const select = h('select', { 'aria-label': 'Partner path file', class: 'numin' },
    h('option', { value: '', text: 'None' }), ...partners.map((p) => h('option', { value: p.file, text: p.file })));
  select.value = partnerChoice.mode === 'file' ? partnerChoice.file || '' : '';
  select.addEventListener('change', () => { setPartner(select.value ? 'file' : 'none', select.value || null); homeUI.render(); });
  const pct = h('input', { class: 'numin m', type: 'number', min: 25, max: 150, step: 5, value: partnerChoice.speedPct, 'aria-label': 'Partner speed in percent' });
  pct.addEventListener('input', () => { const v = Number(pct.value); if (Number.isFinite(v) && v >= 25 && v <= 150) setPartner(partnerChoice.mode, partnerChoice.file, v); });
  const row = (label, el, unit) => h('div', { class: 'prow' }, h('div', { class: 'prow-main' }, h('div', { class: 'plabel' }, h('b', { text: label })), el, h('span', { class: 'unit m', text: unit || '' }), h('span', { class: 'estcol' }), h('span')));
  return { count: 2, subtitle: 'R2 path file and speed', rows: [row('Path file', select), row('Speed', pct, '%')] };
}

const paramsUI = mountParams($('params'), {
  defs: () => paramDefs, defaults: () => paramDefaults || {}, values: () => paramValues,
  setValue: (id, v) => { paramValues[id] = v; updateChangedPill(); },
  resetAll: () => { if (paramDefaults) paramValues = { ...paramDefaults }; setPartner('none', null, 100); updateChangedPill(); },
  run: () => { if (paramsUI.problems()) return; requestRun({ params: { ...paramValues } }); },
  back: () => go(run ? 'run' : 'home'),
  start: () => { const m = opmodes.find((x) => x.name === selectedOpMode); return m ? m.start : null; },
  field: () => (run ? run.field : null),
  partnerGroup: buildPartnerGroup,
});

const homeUI = mountHome($('homePanel'), $('homeStage'), {
  opmodes: () => opmodes, selected: () => selectedOpMode, select: selectOpMode,
  verdictFor: (name) => verdicts.get(name) || null,
  run: () => requestRun(), watching: () => serverState.watching, setWatching: (on) => post('/api/watch', { enabled: on }),
  connected: () => serverState.phase !== 'offline', hasRun: () => Boolean(run), openLast: () => go('run'),
  partner: { mode: () => partnerChoice.mode, file: () => partnerChoice.file, list: () => partners, set: (mode, file) => setPartner(mode, file) },
});

function updateHome() {
  homeUI.setEmpty(!run);
  homeUI.setCaption(run ? `Last run at ${duration.toFixed(1)} s · ${run.partner ? 'R1 yours, R2 partner · ' : ''}drag to orbit` : '');
  homeUI.render();
}

function connect() {
  if (source) source.close();
  source = new EventSource('/api/events');
  source.onmessage = (message) => applyState(JSON.parse(message.data));
  source.onerror = () => applyState({ ...serverState, phase: 'offline', message: '' });
}

async function init() {
  document.body.dataset.view = 'home';
  document.body.dataset.mode = '3d';
  placeStage();
  $('work').setAttribute('inert', '');
  $('params').setAttribute('inert', '');
  setDetails(false);
  try {
    opmodes = await (await fetch('/api/opmodes')).json();
    for (const m of opmodes) $('opmode').append(h('option', { value: m.name, text: m.name }));
    selectOpMode(selectedOpMode || (opmodes[0] && opmodes[0].name));
  } catch (err) { console.error('init', err); }
  loadParams();
  loadPartners();
  homeUI.render();
  connect();
  window.matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => buildScene({ keepCamera: true }));
  new ResizeObserver(() => layout()).observe($('homeStage'));
  new ResizeObserver(() => layout()).observe($('fieldArea'));
  route();
}

// ---------------------------------------------------------------------------------------------
// Frame loop
// ---------------------------------------------------------------------------------------------

let last = performance.now();
let shownMoment = '';
const gapEl = h('span', { class: 'gaplabel m', hidden: true });
$('fieldArea').append(gapEl);

function frame(now) {
  const dt = Math.max(0, Math.min(0.1, (now - last) / 1000));
  last = now;
  if (run && playing && view === 'run') {
    time += dt * speed;
    if (time >= duration) { time = duration; playing = false; syncPlay(); }
  }
  const drawing = view === 'run' || view === 'home';
  if (drawing) layout();
  if (run && robot && drawing) {
    const pose = poseAt(time);
    robot.group.position.set(pose.x, -pose.y, 0.3);
    robot.group.rotation.z = -pose.theta;
    const odoKnown = Number.isFinite(pose.ox);
    ghost.group.visible = odoKnown;
    if (odoKnown) { ghost.group.position.set(pose.ox, -pose.oy, 0.4); ghost.group.rotation.z = -pose.otheta; }
    trails.truth.geometry.setDrawRange(0, pose.index + 1);
    trails.odo.geometry.setDrawRange(0, pose.index + 1);
    const pp = partnerPoseAt(time);
    if (pp) { partner.group.position.set(pp.x, -pp.y, 0.25); partner.group.rotation.z = -pp.theta; }

    const ballPositions = ballsAt(time);
    const tilts = hiveTiltAt(time);
    if (effectiveMode() === '3d' && scene3d) scene3d.update(pose, ballPositions, tilts, pose.index, pp);
    else updateTopBalls(ballPositions, tilts);

    if (view === 'run') {
      timelineUI.setTime(time);
      updateScoreChip();
      updateMoment(pose);
      updateGapLabel(pose);
      if (detailsOpen) detailsUI.update(time, pose, run, analysis, { balls: ballPositions.length ? ballCounts(ballPositions, pose) : null, odoWarnMm: THRESHOLDS.odoMmWarn });
    }
  }
  if (drawing || performance.now() < renderUntil) {
    if (effectiveMode() === '3d' && scene3d) renderer.render(scene3d.scene, scene3d.camera);
    else renderer.render(scene, camera);
  }
  requestAnimationFrame(frame);
}

function heldNow() {
  let v = 0;
  for (const [t, c] of run.balls.held) { if (t <= time) v = c; else break; }
  return v;
}

function updateMoment(pose) {
  const d = decideAt(analysis.decisions, time);
  const gap = Math.hypot(pose.x - pose.ox, pose.y - pose.oy);
  const held = heldNow();
  const key = `${time.toFixed(2)}|${d ? d.move : ''}|${Number.isFinite(gap) ? gap.toFixed(0) : ''}|${held}`;
  if (key === shownMoment) return;
  shownMoment = key;
  const warn = Number.isFinite(gap) && gap > THRESHOLDS.odoMmWarn;
  $('momentChip').replaceChildren(
    h('span', { class: 'top' }, h('b', { text: `At ${time.toFixed(2)} s` }), h('span', { text: 'Details →' })),
    h('span', { class: 'line' }, h('span', { text: 'decide()' }), h('span', { class: 'm', text: d ? d.move : '—' })),
    h('span', { class: 'line' }, h('span', { text: 'Odometry gap' }), h('span', { class: `m${warn ? ' warnc' : ''}`, text: Number.isFinite(gap) ? `${gap.toFixed(0)} mm` : '—' })),
    h('span', { class: 'line' }, h('span', { text: 'Holding' }), h('span', { class: 'm', text: `${held} of 4` })));
}

/** The "gap 38 mm · 2.8°" label next to the robot, only when odometry is off by more than the warn level. */
function updateGapLabel(pose) {
  const text = gapLabel(pose, THRESHOLDS.odoMmWarn);
  if (!text) { if (!gapEl.hidden) gapEl.hidden = true; return; }
  const mode3d = effectiveMode() === '3d' && scene3d;
  const v = new THREE.Vector3(pose.x, -pose.y, mode3d ? 500 : 0).project(mode3d ? scene3d.camera : camera);
  const box = $('view').getBoundingClientRect(), area = $('fieldArea').getBoundingClientRect();
  gapEl.hidden = false;
  if (gapEl.textContent !== text) gapEl.textContent = text;
  gapEl.style.left = `${box.left - area.left + ((v.x + 1) / 2) * box.width + 18}px`;
  gapEl.style.top = `${box.top - area.top + ((1 - v.y) / 2) * box.height - 26}px`;
}

window.simCamera3D = (...a) => scene3d && scene3d.setCamera(...a); // for tests: put the 3D camera somewhere
window.simDebug = () => ({ time, view, viewMode, detailsOpen, railView, camera3d: scene3d ? scene3d.getCamera() : null, runId: run ? run.meta.runId : null, chip: lastChip, w: $('view').clientWidth, h: $('view').clientHeight });
window.simSet = (t) => { time = clampT(t); };

init();
requestAnimationFrame(frame);
