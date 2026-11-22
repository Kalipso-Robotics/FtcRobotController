// The #/robot view: a small three.js editor for sim/robot.json (format: sim/robot.schema.md; pure model: robotModel.js).
// Everything is millimetres in the robot frame (x forward, y left, z up). Handles edit the model's point arrays in place.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { h } from './rail.js';
import { createHistory } from './history.js';
import { BEAM_NAMES, UNIT_SCALE, robotToModel, modelToRobot, originOffset } from './robotModel.js';

const COLORS = { intake: 0x3dd68c, transfer: 0xf5a524, turret: 0xa993e0, shooter: 0xff6b6b, hood: 0xff9ecb, beam: 0x6aa3ff, noodle: 0x22d3ee };
const PART_COLOR = { body: 0x9aa3ad, turret: 0x7a61ac, hood: 0xf5a524 };
const AXES = ['x', 'y', 'z'];
const MAX_STL = 50 * 1024 * 1024;   // same cap as the server (stlUpload.mjs)
const hex = (n) => `#${n.toString(16).padStart(6, '0')}`;

/** The typed blocks in a model, as { key, label, color, pts: [{ name, ref, handle }], fields, remove }. */
function blocksOf(m) {
  const out = [];
  const num = (label, o, k, opt) => ({ label, get: () => o[k], set: (v) => { if (v === '' && opt) delete o[k]; else o[k] = Number(v); }, opt });
  m.intakeZones.forEach((z, i) => out.push({
    key: `intake:${i}`, label: `Intake zone ${z.name || i}`, color: COLORS.intake,
    pts: [{ name: 'min', ref: z.min, handle: true }, { name: 'max', ref: z.max, handle: true }],
    fields: [{ label: 'name', text: true, get: () => z.name || '', set: (v) => { z.name = v; } }, { label: 'side', options: ['front', 'back'], get: () => z.side, set: (v) => { z.side = v; } }],
    remove: () => m.intakeZones.splice(i, 1),
  }));
  if (m.transferPath.length) out.push({
    key: 'transfer', label: 'Transfer path', color: COLORS.transfer,
    pts: m.transferPath.map((p, i) => ({ name: `p${i}`, ref: p, handle: true })),
    fields: [], buttons: [
      ['add point', () => { const l = m.transferPath[m.transferPath.length - 1]; m.transferPath.push([l[0] - 50, l[1], l[2]]); }],
      ['remove last point', () => { if (m.transferPath.length > 1) m.transferPath.pop(); }],
    ],
    remove: () => { m.transferPath = []; },
  });
  if (m.turret) out.push({ key: 'turret', label: 'Turret pivot', color: COLORS.turret, pts: [{ name: 'pivot', ref: m.turret.pivot, handle: true }], fields: [], remove: () => { m.turret = null; } });
  if (m.shooter) {
    const s = m.shooter;
    out.push({
      key: 'shooter', label: 'Shooter / hood', color: COLORS.shooter,
      pts: [{ name: 'exit', ref: s.exit, handle: true }, ...(s.hoodPivot ? [{ name: 'hoodPivot', ref: s.hoodPivot, handle: true }] : [])],
      fields: [num('hoodMinDeg', s, 'hoodMinDeg', true), num('hoodMaxDeg', s, 'hoodMaxDeg', true)],
      buttons: s.hoodPivot ? [] : [['add hood pivot', () => { s.hoodPivot = [s.exit[0] - 20, s.exit[1], s.exit[2] - 80]; }]],
      remove: () => { m.shooter = null; },
    });
  }
  m.beams.forEach((b, i) => out.push({
    key: `beam:${i}`, label: b.name, color: COLORS.beam,
    pts: [{ name: 'emitter', ref: b.emitter, handle: true }, { name: 'receiver', ref: b.receiver, handle: true }],
    fields: [], remove: () => m.beams.splice(i, 1),
  }));
  m.noodles.forEach((n, i) => out.push({
    key: `noodle:${i}`, label: `Noodle ${n.name || i}`, color: COLORS.noodle,
    pts: [{ name: 'center', ref: n.center, handle: true }, { name: 'axis', ref: n.axis, handle: false }],
    fields: [{ label: 'name', text: true, get: () => n.name || '', set: (v) => { n.name = v; } }, num('radius', n, 'radius'), num('length', n, 'length'), num('blades', n, 'blades')],
    remove: () => m.noodles.splice(i, 1),
  }));
  return out;
}

const ADDERS = {
  intake: (m) => m.intakeZones.push({ name: m.intakeZones.length ? `zone${m.intakeZones.length}` : 'front', side: 'front', min: [250, -150, 0], max: [330, 150, 60] }),
  transfer: (m) => { m.transferPath = [[250, 0, 90], [0, 0, 150], [-250, 0, 90]]; },
  turret: (m) => { m.turret = { pivot: [0, 0, 300] }; },
  shooter: (m) => { m.shooter = { exit: [60, 0, 500], hoodPivot: [40, 0, 420] }; },
  noodle: (m) => m.noodles.push({ name: `n${m.noodles.length}`, center: [200, 0, 80], axis: [0, 1, 0], radius: 40, length: 300, blades: 6 }),
};
const KEY_OF = { intake: 'intake:', transfer: 'transfer', turret: 'turret', shooter: 'shooter', noodle: 'noodle:' };

export function mountRobot(root) {
  let model = robotToModel(null), footprint = null, files = [], warnings = [];
  let selMesh = -1, selKey = null, selHandle = 0, pickMode = false, loaded = false, inited = false, visible = false;
  let blocks = [], handles = [], meshObjs = [], syncInputs = [], gen = 0;
  let snap = '';   // JSON of the model as of the last commit (the undo baseline)
  const hist = createHistory(50);
  const geoCache = new Map();
  const loader = new STLLoader();

  // ---- DOM ----
  const msg = h('span', { class: 'rmsg dim', role: 'status', 'aria-live': 'polite' });
  const drop = h('div', { class: 'rdrop', hidden: true }, 'Drop .stl files to upload');
  const warn = h('div', { class: 'rwarn', hidden: true });
  const panel = h('aside', { class: 'rpanel' });
  const host = h('div', { class: 'rview' });
  root.append(panel, h('div', { class: 'rmain' }, warn, host, drop));

  // ---- three ----
  let renderer, scene, camera, orbit, tc, handleGroup, lineGroup, meshGroup, fpGroup;
  const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();

  function initThree() {
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    host.append(renderer.domElement);
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d0f12);   // show() re-reads --bg so the light theme matches
    camera = new THREE.PerspectiveCamera(45, 1, 1, 20000);
    camera.up.set(0, 0, 1);
    camera.position.set(-700, -800, 550);
    orbit = new OrbitControls(camera, renderer.domElement);
    orbit.target.set(0, 0, 150);
    scene.add(new THREE.AmbientLight(0xffffff, 1.4));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(300, -500, 900); scene.add(sun);
    const grid = new THREE.GridHelper(2000, 40, 0x4a505a, 0x30343b); grid.rotation.x = Math.PI / 2; scene.add(grid);
    scene.add(new THREE.AxesHelper(250));   // x red forward, y green left, z blue up
    meshGroup = new THREE.Group(); lineGroup = new THREE.Group(); handleGroup = new THREE.Group(); fpGroup = new THREE.Group();
    scene.add(meshGroup, lineGroup, handleGroup, fpGroup);
    tc = new TransformControls(camera, renderer.domElement);
    tc.setMode('translate'); tc.setSize(0.7);
    tc.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; if (!e.value) commit(); });
    tc.addEventListener('objectChange', () => {
      const hd = handles.find((x) => x.obj === tc.object);
      if (!hd) return;
      hd.ref[0] = round(tc.object.position.x); hd.ref[1] = round(tc.object.position.y); hd.ref[2] = round(tc.object.position.z);
      syncInputs.forEach((f) => f());
      drawLines();
    });
    scene.add(tc.getHelper());
    new ResizeObserver(resize).observe(host);
    const main = host.parentElement;
    main.addEventListener('dragover', (e) => { if (e.dataTransfer && [...e.dataTransfer.types].includes('Files')) { e.preventDefault(); drop.hidden = false; } });
    main.addEventListener('dragleave', (e) => { if (!main.contains(e.relatedTarget)) drop.hidden = true; });
    main.addEventListener('drop', (e) => { e.preventDefault(); drop.hidden = true; upload([...e.dataTransfer.files]); });

    let down = null;
    renderer.domElement.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, axis: tc.axis }; });
    renderer.domElement.addEventListener('pointerup', (e) => {
      if (!down || down.axis || tc.dragging || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 4) return;
      click(e);
    });
    inited = true;
  }

  const round = (v) => Math.round(v * 100) / 100;
  function resize() {
    const w = host.clientWidth, hh = host.clientHeight;
    if (!renderer || !w || !hh) return;
    renderer.setSize(w, hh); camera.aspect = w / hh; camera.updateProjectionMatrix();
  }
  function loop() {
    if (!visible) return;
    orbit.update(); renderer.render(scene, camera);
    requestAnimationFrame(loop);
  }

  function setRay(e) {
    const r = renderer.domElement.getBoundingClientRect();
    ptr.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ptr, camera);
  }

  function click(e) {
    setRay(e);
    if (pickMode) {
      const obj = meshObjs[selMesh];
      const hit = obj && ray.intersectObject(obj, false)[0];
      if (!hit) return say('No hit: click on the selected model.');
      const mesh = model.meshes[selMesh];
      const p = obj.worldToLocal(hit.point.clone());
      mesh.offset = originOffset([p.x, p.y, p.z], mesh.scale, mesh.rotationDeg).map(round);
      pickMode = false; say('Origin set.'); refreshMeshes(); renderPanel(); commit();
      return;
    }
    const hit = ray.intersectObjects(handles.map((x) => x.obj), false)[0];
    if (hit) { const hd = handles.find((x) => x.obj === hit.object); selKey = hd.key; selHandle = hd.i; rebuild(); }
  }

  // ---- scene building ----
  function geoFor(file) {
    if (!geoCache.has(file)) geoCache.set(file, loader.loadAsync(`/robot/${encodeURIComponent(file)}`));
    return geoCache.get(file);
  }

  function applyTransform(obj, m) {
    obj.scale.setScalar(m.scale || 1);
    const r = (m.rotationDeg || [0, 0, 0]).map((d) => (d * Math.PI) / 180);
    obj.rotation.set(r[0], r[1], r[2], 'XYZ');
    obj.position.set(...(m.offset || [0, 0, 0]));
    obj.updateMatrixWorld(true);
  }

  function refreshMeshes() {
    const g = ++gen;
    meshGroup.clear(); meshObjs = [];
    model.meshes.forEach((m, i) => {
      geoFor(m.file).then((geo) => {
        if (g !== gen) return;
        const sel = i === selMesh;
        const obj = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: m.color || PART_COLOR[m.part] || 0x9aa3ad, transparent: true, opacity: sel ? 0.55 : 0.35, side: THREE.DoubleSide, depthWrite: false }));
        applyTransform(obj, m);
        meshObjs[i] = obj; meshGroup.add(obj);
      }).catch((err) => say(`Could not load ${m.file}: ${err.message || err}`, true));
    });
  }

  const mat = (c) => new THREE.LineBasicMaterial({ color: c });
  const V = (p) => new THREE.Vector3(...p);
  const line = (pts, c) => new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map(V)), mat(c));

  function drawFootprint() {
    fpGroup.clear();
    if (!footprint) return;
    const { front, back, width } = footprint, w = width / 2;
    fpGroup.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([[front, -w, 1], [front, w, 1], [-back, w, 1], [-back, -w, 1]].map(V)), mat(0xf5a524)));
  }

  function drawLines() {
    lineGroup.clear();
    for (const b of blocks) {
      const sel = b.key === selKey, c = b.color;
      const p = (n) => b.pts.find((x) => x.name === n).ref;
      if (b.key.startsWith('intake')) {
        const a = p('min'), z = p('max');
        const bx = new THREE.Box3(new THREE.Vector3(Math.min(a[0], z[0]), Math.min(a[1], z[1]), Math.min(a[2], z[2])), new THREE.Vector3(Math.max(a[0], z[0]), Math.max(a[1], z[1]), Math.max(a[2], z[2])));
        lineGroup.add(new THREE.Box3Helper(bx, c));
      } else if (b.key === 'transfer') lineGroup.add(line(b.pts.map((x) => x.ref), c));
      else if (b.key.startsWith('beam')) lineGroup.add(line([p('emitter'), p('receiver')], c));
      else if (b.key === 'shooter' && b.pts[1]) lineGroup.add(line([p('hoodPivot'), p('exit')], COLORS.hood));
      else if (b.key.startsWith('noodle')) {
        const n = model.noodles[Number(b.key.split(':')[1])];
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(n.radius, n.radius, n.length, 24, 1, true), new THREE.MeshBasicMaterial({ color: c, wireframe: true, transparent: true, opacity: sel ? 0.5 : 0.25 }));
        cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), V(n.axis).normalize());
        cyl.position.copy(V(n.center)); lineGroup.add(cyl);
      }
    }
  }

  function drawHandles() {
    tc.detach(); handleGroup.clear(); handles = [];
    for (const b of blocks) b.pts.forEach((pt, i) => {
      if (!pt.handle) return;
      const sel = b.key === selKey;
      const color = b.key === 'shooter' && pt.name === 'hoodPivot' ? COLORS.hood : b.color;
      const obj = new THREE.Mesh(new THREE.SphereGeometry(sel ? 9 : 6, 14, 10), new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: sel ? 1 : 0.7 }));
      obj.renderOrder = 10; obj.position.set(...pt.ref);
      handleGroup.add(obj); handles.push({ obj, ref: pt.ref, key: b.key, i });
    });
    const hd = handles.find((x) => x.key === selKey && x.i === selHandle) || handles.find((x) => x.key === selKey);
    if (hd) tc.attach(hd.obj);
  }

  /** Structure or selection changed: rebuild blocks, handles, lines, panel. */
  function rebuild() {
    blocks = blocksOf(model);
    if (selKey && !blocks.some((b) => b.key === selKey)) selKey = null;
    drawHandles(); drawLines(); renderPanel();
  }

  // ---- panel ----
  function say(text, bad) { msg.textContent = text; msg.className = `rmsg ${bad ? 'bad' : 'dim'}`; }

  const numInput = (get, set) => {
    const i = h('input', { type: 'number', step: 'any', class: 'rnum' });
    const sync = () => { if (document.activeElement !== i) i.value = get() ?? ''; };
    i.value = get() ?? '';
    i.addEventListener('change', () => { set(i.value); });
    return { el: i, sync };
  };
  // Any click/change in the panel may have edited the model: compare after the handler ran and push an undo step if it did.
  panel.addEventListener('click', () => setTimeout(commit), true);
  panel.addEventListener('change', () => setTimeout(commit), true);
  const row = (label, ...kids) => h('label', { class: 'rrow' }, h('span', { class: 'dim', text: label }), ...kids);

  function renderPanel() {
    syncInputs = [];
    panel.replaceChildren();
    // top bar
    undoBtn = h('button', { class: 'btn', title: 'Undo (Ctrl/Cmd+Z)', 'aria-label': 'Undo', onclick: undo }, '↶ Undo');
    redoBtn = h('button', { class: 'btn', title: 'Redo (Shift+Ctrl/Cmd+Z)', 'aria-label': 'Redo', onclick: redo }, 'Redo ↷');
    panel.append(h('div', { class: 'rhead' },
      h('a', { class: 'pback', href: '#/run' }, '← Workspace'), h('h1', { text: 'Robot' }),
      h('div', { class: 'rbar' },
        h('button', { class: 'btn btn-p', onclick: save }, 'Save'), h('button', { class: 'btn', title: 'Discard edits and reload robot.json', onclick: () => load() }, 'Reload'),
        undoBtn, redoBtn), msg));
    syncHist();

    // files
    const partSel = h('select', { class: 'rsel', 'aria-label': 'Part for new mesh' }, ['body', 'turret', 'hood'].map((p) => h('option', { value: p, text: p })));
    const fileSel = h('div', { class: 'rlist' }, files.length ? files.map((f) => h('div', { class: 'rline' }, h('span', { class: 'm', text: f }),
      h('button', { class: 'btn', 'aria-label': `Add ${f} as a mesh`, onclick: () => { model.meshes.push({ file: f, part: partSel.value, scale: 1, rotationDeg: [0, 0, 0], offset: [0, 0, 0] }); selMesh = model.meshes.length - 1; refreshMeshes(); renderPanel(); } }, 'add')))
      : h('p', { class: 'dim empty', text: 'No STL files in sim/robot/ yet. Upload one, or drop it on the model.' }));
    const picker = h('input', { type: 'file', accept: '.stl', multiple: true, hidden: true, onchange: () => { upload([...picker.files]); picker.value = ''; } });
    panel.append(h('h2', { text: 'Files' }), row('part for new mesh', partSel), fileSel,
      h('button', { class: 'btn', disabled: busyUp, onclick: () => picker.click() }, busyUp ? 'Uploading...' : 'Upload STL...'), picker);

    panel.append(h('h2', { text: 'Meshes' }), h('div', { class: 'rlist' }, model.meshes.length ? model.meshes.map((m, i) => act(`rline${i === selMesh ? ' sel' : ''}`, () => { selMesh = i; pickMode = false; refreshMeshes(); renderPanel(); },
      h('span', { class: 'swatch', style: `background:${m.color || hex(PART_COLOR[m.part] || 0x9aa3ad)}` }),
      h('span', { class: 'm', text: `${m.file} (${m.part})` }), !files.includes(m.file) && h('span', { class: 'tag-miss', text: 'file missing' }),
      h('button', { class: 'btn', 'aria-label': `Delete mesh ${m.file}`, onclick: (e) => { e.stopPropagation(); model.meshes.splice(i, 1); selMesh = -1; refreshMeshes(); renderPanel(); } }, 'delete'))) : h('p', { class: 'dim', text: 'None yet.' })));
    const mesh = model.meshes[selMesh];
    if (mesh) panel.append(meshEditor(mesh));

    // blocks
    const addSel = h('select', { class: 'rsel' }, [
      ...Object.keys(ADDERS).filter((k) => k === 'intake' || k === 'noodle' || !blocks.some((b) => b.key.startsWith(KEY_OF[k]))).map((k) => h('option', { value: k, text: k })),
      ...BEAM_NAMES.filter((n) => !model.beams.some((b) => b.name === n)).map((n) => h('option', { value: `beam:${n}`, text: n })),
    ]);
    panel.append(h('h2', { text: 'Blocks' }), h('div', { class: 'rline' }, addSel, h('button', { class: 'btn', onclick: () => {
      const v = addSel.value;
      if (!v) return;
      if (v.startsWith('beam:')) model.beams.push({ name: v.slice(5), emitter: [100, -40, 150], receiver: [100, 40, 150] });
      else ADDERS[v](model);
      blocks = blocksOf(model);
      selKey = v.startsWith('beam:') ? `beam:${model.beams.length - 1}` : v === 'transfer' || v === 'turret' || v === 'shooter' ? v : `${KEY_OF[v]}${model[v === 'intake' ? 'intakeZones' : 'noodles'].length - 1}`;
      selHandle = 0; rebuild();
    } }, 'add')));
    panel.append(h('div', { class: 'rlist' }, blocks.map((b) => act(`rline${b.key === selKey ? ' sel' : ''}`, () => { selKey = b.key; selHandle = 0; rebuild(); },
      h('span', { class: 'swatch', style: `background:${hex(b.color)}` }), h('span', { class: 'm', text: b.label })))));
    panel.append(footprintBox());
    const b = blocks.find((x) => x.key === selKey);
    if (b) panel.append(blockEditor(b));
  }

  function meshEditor(m) {
    const units = Object.entries(UNIT_SCALE).find(([, s]) => s === m.scale)?.[0] || 'custom';
    const unitSel = h('select', { class: 'rsel' }, [...Object.keys(UNIT_SCALE), 'custom'].map((u) => h('option', { value: u, text: u, selected: u === units })));
    unitSel.addEventListener('change', () => { if (UNIT_SCALE[unitSel.value]) { m.scale = UNIT_SCALE[unitSel.value]; refreshMeshes(); renderPanel(); } });
    // compose a 90 degree turn about the world axis k onto the current rotation, store back as XYZ Euler degrees
    const rot = (k, d) => () => {
      const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...[0, 1, 2].map((i) => (i === k ? 1 : 0))), (d * Math.PI) / 180)
        .multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(...m.rotationDeg.map((x) => (x * Math.PI) / 180), 'XYZ')));
      const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
      m.rotationDeg = [e.x, e.y, e.z].map((r) => { const v = Math.round(((r * 180) / Math.PI) * 1000) / 1000; return v === 0 ? 0 : v; });
      refreshMeshes(); renderPanel();
    };
    const apply = () => { refreshMeshes(); };
    const sc = numInput(() => m.scale, (v) => { m.scale = Number(v); apply(); renderPanel(); });
    const part = h('select', { class: 'rsel' }, ['body', 'turret', 'hood'].map((p) => h('option', { value: p, text: p, selected: p === m.part })));
    part.addEventListener('change', () => { m.part = part.value; refreshMeshes(); renderPanel(); });
    const offs = AXES.map((a, k) => row(`offset ${a}`, numInput(() => m.offset[k], (v) => { m.offset[k] = Number(v); apply(); }).el));
    const rots = AXES.map((a, k) => h('div', { class: 'rline' }, h('span', { class: 'dim', text: `rot ${a}` }), h('span', { class: 'm', text: `${m.rotationDeg[k]}°` }),
      h('button', { class: 'btn', onclick: rot(k, -90) }, '-90'), h('button', { class: 'btn', onclick: rot(k, 90) }, '+90')));
    const dflt = hex(PART_COLOR[m.part] || 0x9aa3ad);
    const colour = h('input', { type: 'color', class: 'rcolor', value: m.color || dflt, 'aria-label': `Colour of ${m.file}` });
    colour.addEventListener('input', () => { m.color = colour.value; refreshMeshes(); });
    const colourReset = h('button', { class: 'btn', disabled: !m.color, onclick: () => { delete m.color; refreshMeshes(); renderPanel(); } }, 'reset');
    return h('div', { class: 'rbox' }, row('part', part), row('colour', h('span', { class: 'rcell' }, colour, colourReset)), row('units', unitSel), row('scale', sc.el), ...rots, ...offs,
      h('button', { class: `btn${pickMode ? ' btn-p' : ''}`, onclick: () => { pickMode = !pickMode; say(pickMode ? 'Click the odometry point on the model.' : ''); renderPanel(); } }, 'Set origin: click the odometry point'));
  }

  function blockEditor(b) {
    const box = h('div', { class: 'rbox' }, h('strong', { text: b.label }));
    for (const f of b.fields) {
      if (f.options) { const s = h('select', { class: 'rsel' }, f.options.map((o) => h('option', { value: o, text: o, selected: o === f.get() }))); s.addEventListener('change', () => { f.set(s.value); drawLines(); }); box.append(row(f.label, s)); }
      else if (f.text) { const i = h('input', { class: 'rnum', value: f.get() }); i.addEventListener('change', () => { f.set(i.value); blocks = blocksOf(model); drawLines(); }); box.append(row(f.label, i)); }
      else { const n = numInput(f.get, (v) => { f.set(v); drawLines(); }); box.append(row(f.label, n.el)); }
    }
    b.pts.forEach((pt, pi) => {
      const cells = AXES.map((a, k) => { const n = numInput(() => pt.ref[k], (v) => { pt.ref[k] = Number(v); const hd = handles.find((x) => x.ref === pt.ref); if (hd) hd.obj.position.set(...pt.ref); drawLines(); }); syncInputs.push(n.sync); return h('span', { class: 'rcell' }, h('span', { class: 'dim', text: a }), n.el); });
      box.append(h('div', { class: `rpt${pt.handle && key(b, pi) ? ' sel' : ''}` }, h('button', { class: 'm rptn linkbtn', text: pt.name, onclick: () => { if (pt.handle) { selHandle = pi; drawHandles(); renderPanel(); } } }), ...cells));
    });
    for (const [label, fn] of b.buttons || []) box.append(h('button', { class: 'btn', onclick: () => { fn(); rebuild(); } }, label));
    box.append(h('button', { class: 'btn', onclick: () => { b.remove(); selKey = null; rebuild(); } }, 'Delete block'));
    return box;
  }
  const key = (b, pi) => b.key === selKey && pi === selHandle;

  // ---- keyboard-reachable list rows ----
  const act = (cls, fn, ...kids) => h('div', { class: cls, tabindex: '0', role: 'button', onclick: fn,
    onkeydown: (e) => { if ((e.key === 'Enter' || e.key === ' ') && e.target === e.currentTarget) { e.preventDefault(); fn(e); } } }, ...kids);

  // ---- undo / redo ----
  let undoBtn, redoBtn;
  function syncHist() { if (undoBtn) { undoBtn.disabled = !hist.canUndo(); redoBtn.disabled = !hist.canRedo(); } }
  function commit() {
    const now = JSON.stringify(model);
    if (now === snap) return;
    hist.push(snap); snap = now; syncHist();
  }
  function restore(text) {
    if (text === null) return;
    model = JSON.parse(text); snap = text;
    selMesh = Math.min(selMesh, model.meshes.length - 1); pickMode = false;
    refreshMeshes(); rebuild(); say('');
  }
  const undo = () => { commit(); restore(hist.undo(snap)); syncHist(); };
  const redo = () => { restore(hist.redo(snap)); syncHist(); };
  function onKey(e) {
    if (!visible || !(e.metaKey || e.ctrlKey) || e.key.toLowerCase() !== 'z') return;
    if (e.target.matches && e.target.matches('input:not([type=color]),textarea,select')) return;   // leave text editing to the browser
    e.preventDefault();
    if (e.shiftKey) redo(); else undo();
  }

  // ---- STL upload ----
  let busyUp = false;
  function put(file, overwrite) {
    return new Promise((resolve) => {
      const xhr = new XMLHttpRequest();
      xhr.open('POST', `/api/robot/stl?name=${encodeURIComponent(file.name)}${overwrite ? '&overwrite=1' : ''}`);
      xhr.upload.onprogress = (e) => { if (e.lengthComputable) say(`Uploading ${file.name}... ${Math.round((e.loaded / e.total) * 100)}%`); };
      xhr.onload = () => { let b = {}; try { b = JSON.parse(xhr.responseText); } catch { /* not JSON */ } resolve({ status: xhr.status, error: b.error }); };
      xhr.onerror = () => resolve({ status: 0, error: 'Could not reach the server.' });
      xhr.send(file);
    });
  }
  async function upload(list) {
    if (busyUp) return;
    const picked = list.filter((f) => f.name);
    if (!picked.length) return;
    busyUp = true; renderPanel();
    let ok = 0;
    for (const f of picked) {
      if (!/\.stl$/i.test(f.name)) { say(`${f.name}: only .stl files can be uploaded.`, true); continue; }
      if (f.size > MAX_STL) { say(`${f.name}: over the 50 MB limit.`, true); continue; }
      let r = await put(f, false);
      if (r.status === 409) {
        if (!confirm(`${f.name} is already in sim/robot/. Replace it?`)) { say(`Kept the existing ${f.name}.`); continue; }
        r = await put(f, true);
        geoCache.delete(f.name);
      }
      if (r.status === 201) ok++; else say(`${f.name}: ${r.error || `upload failed (${r.status})`}`, true);
    }
    busyUp = false;
    await refreshFiles();
    if (ok) { say(`Uploaded ${ok} file${ok > 1 ? 's' : ''}.`); refreshMeshes(); }
  }
  /** Re-reads the file list and warnings only; the model being edited is left alone. */
  async function refreshFiles() {
    try { const d = await (await fetch('/api/robot')).json(); files = d.files || []; showWarnings(d.warnings || []); } catch (err) { say(`Could not refresh files: ${err.message}`, true); }
    renderPanel();
  }
  function showWarnings(w) { warnings = w; warn.hidden = !w.length; warn.replaceChildren(...w.map((x) => h('div', { text: x }))); }

  // ---- footprint ----
  /** Puts the designer's footprint into the sim's robotFront/Back/Width params (the Setup screen's "Robot size"), for sim runs only. */
  async function useFootprint() {
    const fp = model.footprint;
    say('Applying...');
    try {
      const cur = await (await fetch('/api/params')).json();
      const params = { ...cur.values, robotFront: fp.front, robotBack: fp.back, robotWidth: fp.width };
      const res = await fetch('/api/run', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ params }) });
      const b = await res.json().catch(() => ({}));
      say(res.ok ? 'Sim runs now use the designer footprint. The sim is re-running.' : b.error || `Failed (${res.status})`, !res.ok);
    } catch (err) { say(`Failed: ${err.message}`, true); }
  }
  function footprintBox() {
    const fp = model.footprint, real = footprint && footprint !== fp ? footprint : null;
    const fmt = (f) => `front ${f.front} · back ${f.back} · width ${f.width} mm`;
    return h('div', { class: 'rbox' }, h('h2', { text: 'Footprint' }),
      fp ? h('p', { class: 'dim', text: `Designer: ${fmt(fp)}` }) : h('p', { class: 'dim empty', text: 'robot.json has no footprint yet.' }),
      real && h('p', { class: 'dim', text: `RobotFootprint.java: ${fmt(real)}` }),
      h('button', { class: 'btn', disabled: !fp, onclick: useFootprint }, 'Use designer footprint in sim runs'),
      h('p', { class: 'dim fine', text: 'RobotFootprint.java stays the source of truth. This only overrides robotFront / robotBack / robotWidth for sim runs (same as Setup > Robot size); it never edits code.' }));
  }

  // ---- server ----
  async function save() {
    say('Saving...');
    try {
      const res = await fetch('/api/robot', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(modelToRobot(model, footprint)) });
      const body = await res.json().catch(() => ({}));
      if (res.ok) say('Saved. The sim is re-running.'); else say(body.error || `Save failed (${res.status})`, true);
    } catch (err) { say(`Save failed: ${err.message}`, true); }
  }

  async function load() {
    try {
      const data = await (await fetch('/api/robot')).json();
      model = robotToModel(data.robot);
      footprint = data.footprint || model.footprint || null;
      files = data.files || []; warnings = data.warnings || [];
    } catch (err) { warnings = [`Could not read /api/robot: ${err.message}`]; }
    showWarnings(warnings);
    snap = JSON.stringify(model); hist.clear();
    selMesh = model.meshes.length ? 0 : -1; selKey = null; pickMode = false; loaded = true;
    if (inited) { drawFootprint(); refreshMeshes(); rebuild(); }
    say('');
  }

  return {
    async show() {
      visible = true;
      document.addEventListener('keydown', onKey);
      if (!inited) initThree();
      scene.background.set(getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#0d0f12');
      if (!loaded) say('Loading robot...');
      if (!loaded) await load();
      resize(); loop();
    },
    hide() { visible = false; document.removeEventListener('keydown', onKey); },
  };
}
