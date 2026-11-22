// The #/path view: draw an R1 path file (sim/paths/r1/<name>.json, see sim/README.md "R1 paths") on a top view of the field.
// mm, alliance frame (red: x right, y DOWN from the top-left corner). The scene uses (x, -y), same as the run view.

import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { h } from './rail.js';
import { createHistory } from './history.js';

const FIELD = 3657.6;
const COLOR = { path: 0x6aa3ff, start: 0xffffff, drive: 0x6aa3ff, sel: 0xf5a524, intake: 0x3dd68c, shoot: 0xff6b6b, wait: 0xa993e0 };
const CSS = `
body[data-view="path"] #path { opacity: 1; visibility: visible; pointer-events: auto; transition: opacity 200ms var(--ease); }
body[data-view="path"] .when-auto, body[data-view="path"] .when-run { display: none; }
#path { display: grid; grid-template-columns: 360px 1fr; background: var(--bg); }
.pstep { display: flex; gap: 8px; align-items: center; padding: 4px 6px; border-radius: 6px; cursor: pointer; font: 400 12.5px var(--mono); }
.pstep:hover { background: var(--line); } .pstep[aria-selected=true] { outline: 2px solid var(--accent-text); }
.pstep i { width: 10px; height: 10px; border-radius: 50%; flex: none; }
.pbtns { display: flex; gap: 6px; flex-wrap: wrap; }
`;
const round = (v) => Math.round(v * 10) / 10;
const blank = () => ({ name: 'new', alliance: 'red', start: [1389.5, 3406, -90], steps: [] });
const label = (s) => (s.drive ? `drive ${s.drive.map(round).join(', ')}` : 'intake' in s ? `intake ${s.intake ? 'on' : 'off'}` : 'shoot' in s ? `shoot ${s.shoot}` : `wait ${s.wait} s`);
const kind = (s) => (s.drive ? 'drive' : 'intake' in s ? 'intake' : 'shoot' in s ? 'shoot' : 'wait');

export function mountPath(root, { done }) {
  const st = document.createElement('style'); st.textContent = CSS; document.head.append(st);
  let model = blank(), sel = -1;   // sel: -1 = the start pose, else a step index
  let snap = JSON.stringify(model), visible = false, inited = false, loaded = false, names = [];
  const hist = createHistory(50);
  const msg = h('span', { class: 'rmsg dim', role: 'status', 'aria-live': 'polite' });
  const panel = h('aside', { class: 'rpanel' });
  const host = h('div', { class: 'rview' });
  root.append(panel, h('div', { class: 'rmain' }, host));

  let renderer, scene, camera, tc, markers, dots, handles = [];
  const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
  const say = (t, bad) => { msg.textContent = t; msg.className = `rmsg ${bad ? 'bad' : 'dim'}`; };

  function initThree() {
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    host.append(renderer.domElement);
    scene = new THREE.Scene();
    camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 5000);
    camera.position.set(FIELD / 2, -FIELD / 2, 2000);
    const rect = (x0, y0, x1, y1, c) => new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([[x0, -y0], [x1, -y0], [x1, -y1], [x0, -y1]].map(([x, y]) => new THREE.Vector3(x, y, 0))), new THREE.LineBasicMaterial({ color: c }));
    scene.add(rect(0, 0, FIELD, FIELD, 0x8a9099));
    const hc = FIELD / 2, hw = 49.46 * 25.4 / 2, hd = 38.95 * 25.4 / 2;   // the Hive frame (FieldConfig)
    scene.add(rect(hc - hw, hc - hd, hc + hw, hc + hd, 0x4a505a));
    const grid = new THREE.GridHelper(FIELD, 6, 0x30343b, 0x30343b); grid.rotation.x = Math.PI / 2; grid.position.set(FIELD / 2, -FIELD / 2, -1); scene.add(grid);
    markers = new THREE.Group(); dots = new THREE.Group(); scene.add(markers, dots);   // markers: lines, redrawn on a drag; dots: the clickable points
    tc = new TransformControls(camera, renderer.domElement);
    tc.setMode('translate'); tc.showZ = false; tc.setSize(0.8);
    tc.addEventListener('dragging-changed', (e) => { if (!e.value) commit(); });
    tc.addEventListener('objectChange', () => {
      const p = poseOf(sel); if (!p) return;
      p[0] = round(tc.object.position.x); p[1] = round(-tc.object.position.y);
      draw(true); renderPanel();
    });
    scene.add(tc.getHelper());
    new ResizeObserver(resize).observe(host);
    let down = null;
    renderer.domElement.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, axis: tc.axis }; });
    renderer.domElement.addEventListener('pointerup', (e) => {
      if (!down || down.axis || tc.dragging || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 4) return;   // a drag is not a click
      click(e);
    });
    inited = true;
  }

  function resize() {
    const w = host.clientWidth, hh = host.clientHeight;
    if (!renderer || !w || !hh) return;
    renderer.setSize(w, hh);
    const half = FIELD * 0.55, a = w / hh;
    camera.left = FIELD / 2 - half * Math.max(a, 1); camera.right = FIELD / 2 + half * Math.max(a, 1);
    camera.top = -FIELD / 2 + half * Math.max(1 / a, 1); camera.bottom = -FIELD / 2 - half * Math.max(1 / a, 1);
    camera.updateProjectionMatrix();
  }
  function loop() { if (!visible) return; renderer.render(scene, camera); requestAnimationFrame(loop); }

  /** The [x, y, deg] array that sel points at, or null if it is not a drive/start. */
  const poseOf = (i) => (i === -1 ? model.start : model.steps[i] && model.steps[i].drive) || null;
  const V = (x, y, z = 1) => new THREE.Vector3(x, -y, z);

  function click(e) {
    const r = renderer.domElement.getBoundingClientRect();
    ptr.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ptr, camera);
    const hit = ray.intersectObjects(handles.map((x) => x.obj), false)[0];
    if (hit) { sel = handles.find((x) => x.obj === hit.object).i; draw(); renderPanel(); return; }
    const p = ray.ray.origin;   // ortho top view: the ray starts straight above the clicked spot
    const x = round(p.x), y = round(-p.y);
    if (x < 0 || x > FIELD || y < 0 || y > FIELD) return;
    const prev = [...model.steps].reverse().find((s) => s.drive)?.drive || model.start;
    const deg = Math.round(Math.atan2(y - prev[1], x - prev[0]) * 180 / Math.PI);
    model.steps.push({ drive: [x, y, deg] }); sel = model.steps.length - 1;
    commit(); draw(); renderPanel();
  }

  /** The polyline, the point handles and the step markers. `keep`: a drag is under way, leave the handles alone. */
  function draw(keep) {
    markers.clear();
    if (!keep) { dots.clear(); handles = []; }
    const pts = [model.start, ...model.steps.filter((s) => s.drive).map((s) => s.drive)];
    if (pts.length > 1) markers.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map((p) => V(p[0], p[1], 2))), new THREE.LineBasicMaterial({ color: COLOR.path })));
    let at = model.start, n = 0;
    const dot = (p, color, size, z) => { const m = new THREE.Mesh(new THREE.CircleGeometry(size, 20), new THREE.MeshBasicMaterial({ color, depthTest: false })); m.position.copy(V(p[0], p[1], z)); m.renderOrder = z; dots.add(m); return m; };
    const arrow = (p, color) => { const a = p[2] * Math.PI / 180; markers.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([V(p[0], p[1], 3), V(p[0] + 160 * Math.cos(a), p[1] + 160 * Math.sin(a), 3)]), new THREE.LineBasicMaterial({ color }))); };
    const mk = (i, p, color, size) => {
      if (!keep) handles.push({ obj: dot(p, i === sel ? COLOR.sel : color, i === sel ? size + 8 : size, 4), i });
      arrow(p, i === sel ? COLOR.sel : color);
    };
    mk(-1, model.start, COLOR.start, 34);
    model.steps.forEach((s, i) => {
      if (s.drive) { mk(i, s.drive, COLOR.drive, 26); at = s.drive; n = 0; return; }
      if (!keep) handles.push({ obj: dot([at[0] + 60 + n * 70, at[1] - 70], i === sel ? COLOR.sel : COLOR[kind(s)], 22, 5), i });
      n++;
    });
    if (keep) return;
    tc.detach();
    const target = handles.find((x) => x.i === sel && poseOf(sel));
    if (target) tc.attach(target.obj);
  }

  // ---- panel ----
  const numIn = (get, set, w) => { const i = h('input', { type: 'number', step: 'any', class: 'rnum', style: w ? `width:${w}px` : '' }); i.value = get(); i.addEventListener('change', () => { const v = Number(i.value); if (Number.isFinite(v)) { set(v); commit(); draw(); renderPanel(); } }); return i; };
  const btn = (text, fn, extra = '', disabled = false) => h('button', { class: `btn ${extra}`, text, onclick: fn, disabled });
  const row = (l, ...kids) => h('label', { class: 'rrow' }, h('span', { class: 'dim', text: l }), ...kids);
  const insert = (step) => { model.steps.splice(sel + 1, 0, step); sel += 1; commit(); draw(); renderPanel(); };

  function renderPanel() {
    const nameIn = h('input', { class: 'rnum', value: model.name, 'aria-label': 'Path name', style: 'width:140px' });
    nameIn.addEventListener('change', () => { model.name = nameIn.value.trim(); commit(); });
    const picker = h('select', { class: 'rsel', 'aria-label': 'Load a path file' }, h('option', { value: '', text: 'Load...' }), ...names.map((n) => h('option', { value: n, text: n })));
    picker.addEventListener('change', () => { if (picker.value) load(picker.value); });
    const s = model.steps[sel];
    const pose = poseOf(sel);
    const editor = [];
    if (pose) editor.push(row('x mm', numIn(() => pose[0], (v) => { pose[0] = v; })), row('y mm', numIn(() => pose[1], (v) => { pose[1] = v; })), row('heading deg', numIn(() => pose[2], (v) => { pose[2] = v; })));
    if (s && 'shoot' in s) editor.push(row('balls', numIn(() => s.shoot, (v) => { s.shoot = Math.round(v); })));
    if (s && 'wait' in s) editor.push(row('seconds', numIn(() => s.wait, (v) => { s.wait = v; })));
    if (s && s.drive) editor.push(row('speed mm/s', numIn(() => s.speedMmS ?? '', (v) => { if (v > 0) s.speedMmS = v; else delete s.speedMmS; })));
    panel.replaceChildren(
      h('h1', { text: 'R1 path' }),
      h('div', { class: 'rhead' }, nameIn, picker, btn('New', () => { model = blank(); reset(); }), msg),
      h('div', { class: 'rhead' },
        btn('Save & Run', save, 'btn-p'),
        btn('Undo', undo, '', !hist.canUndo()),
        btn('Copy JSON', () => navigator.clipboard?.writeText(JSON.stringify(model, null, 2)).then(() => say('Copied'), () => say('Copy failed', true)))),
      h('h2', { text: 'Steps' }),
      h('div', { role: 'listbox', 'aria-label': 'Steps' },
        h('div', { class: 'pstep', role: 'option', 'aria-selected': String(sel === -1), tabindex: 0, onclick: () => { sel = -1; draw(); renderPanel(); } }, h('i', { style: 'background:#fff' }), `start ${model.start.map(round).join(', ')}`),
        ...model.steps.map((x, i) => h('div', { class: 'pstep', role: 'option', 'aria-selected': String(sel === i), tabindex: 0, onclick: () => { sel = i; draw(); renderPanel(); } },
          h('i', { style: `background:#${COLOR[kind(x)].toString(16).padStart(6, '0')}` }), `${i + 1}. ${label(x)}`))),
      h('h2', { text: 'Insert after selected' }),
      h('div', { class: 'pbtns' }, btn('Intake on', () => insert({ intake: true })), btn('Intake off', () => insert({ intake: false })), btn('Shoot', () => insert({ shoot: 3 })), btn('Wait', () => insert({ wait: 1 }))),
      ...(editor.length ? [h('h2', { text: sel === -1 ? 'Start pose' : `Step ${sel + 1}` }), ...editor] : []),
      ...(sel >= 0 ? [btn('Delete step', () => { model.steps.splice(sel, 1); sel = Math.min(sel, model.steps.length - 1); commit(); draw(); renderPanel(); })] : []),
      h('p', { class: 'dim fine', text: 'Click the field to add a drive point at the end. Click a point to select it, drag its arrows to move it. Cmd/Ctrl+Z undoes.' }),
    );
  }

  // ---- undo, load, save ----
  function commit() { const now = JSON.stringify(model); if (now === snap) return; hist.push(snap); snap = now; }
  function undo() { const prev = hist.undo(snap); if (prev === null) return; snap = prev; model = JSON.parse(prev); sel = Math.min(sel, model.steps.length - 1); draw(); renderPanel(); }
  function reset() { snap = JSON.stringify(model); hist.clear(); sel = -1; draw(); renderPanel(); }
  const onKey = (e) => { if ((e.metaKey || e.ctrlKey) && e.key === 'z' && !e.shiftKey && e.target.tagName !== 'INPUT') { e.preventDefault(); undo(); } };

  async function listNames() { try { const r = await (await fetch('/api/paths')).json(); names = r.names; return r.active; } catch { return null; } }
  async function load(name) {
    try {
      const r = await fetch(`/api/path/${encodeURIComponent(name)}`);
      if (!r.ok) throw new Error((await r.json()).error);
      model = await r.json(); reset(); say(`Loaded ${name}`);
    } catch (err) { say(`Could not load ${name}: ${err.message || err}`, true); }
  }
  async function save() {
    say('Saving...');
    try {
      const r = await fetch('/api/path', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(model) });
      const j = await r.json();
      if (!r.ok) return say(j.error || 'Save failed', true);
      await listNames(); say('Saved. Running...'); done();
    } catch (err) { say(`Save failed: ${err.message || err}`, true); }
  }

  return {
    async show() {
      visible = true;
      document.addEventListener('keydown', onKey);
      if (!inited) initThree();
      scene.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue('--bg').trim() || '#0d0f12');
      if (!loaded) { loaded = true; const active = await listNames(); if (active) await load(active); else renderPanel(); }
      resize(); draw(); loop();
    },
    hide() { visible = false; document.removeEventListener('keydown', onKey); },
  };
}
