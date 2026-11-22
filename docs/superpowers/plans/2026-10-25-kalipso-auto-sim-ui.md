# Kalipso Auto Sim UI Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the sim web app (`sim/app/web/`) to the "Kalipso Auto Sim UI" design: Home, a single Workspace (field + summary rail + details drawer + lane timeline), a Parameters screen, honest `~`/`</>` value marks, run-to-run diff, auto score, every failure state, and a physical partner robot R2.

**Architecture:** Same stack, no build step. The four hash views (`#/`, `#/setup`, `#/stats`, `#/replay`) become three (`#/`, `#/run`, `#/params`); Stats and Replay merge into the Workspace. New logic lands as pure functions (`stats.js`, `timeline.js`, `buildError.mjs`) with `node --test` checks; DOM code splits out of the 1,237-line `app.js` into `rail.js` (was `statsView.js`), `params.js` and `timeline.js`. The server keeps the previous run for the diff and structures compile errors. R2 is a kinematic Rapier body driven from a path file, recorded as `#partner` lines.

**Tech Stack:** plain HTML/CSS/ES modules, three.js 0.170 (already vendored), Node 20+ `node:test`, Rapier (sim/physics), Java sim runner (TeamCode test sources).

**Spec:** design `/Users/darren/Downloads/Kalipso Auto Sim UI.html` (8 boards: Home, Workspace 2D, Workspace 3D + details, Parameters, Compile error, Kit, Field 2D, Timeline) + brief `sim/design_spec.md`. The design file is a self-unpacking bundle: to read a board, decode `__bundler/manifest` (base64 + gzip), then each page's own nested manifest/template. A decoded copy lives in `$CLAUDE_JOB_DIR/tmp/design/t0.html … t7.html` for this job only; executors should re-extract if missing (script in Task 0).

**Decisions already made (Darren, 2026-10-25):** partner R2 = full physics + Java; auto score = design point values, flagged to confirm against the manual; theme = Kit dark + a derived light set; details drawer = recorded data only (no motor/shooter/sensor/log sections).

**On approval:** copy this file to `docs/superpowers/plans/2026-10-25-kalipso-auto-sim-ui.md` (left uncommitted).

**Execution:** recommended **subagent-driven**: 14 tasks across three layers (Java, Node physics, web), Tasks 3/4/8 export interfaces that 6/7/13 consume, and a reviewer gate per task catches name drift before it spreads. Order: 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13 → 14 (11–12 can run in parallel with 5–10).

## Global Constraints

- Plain HTML, CSS, ES modules. No framework, no bundler, no new npm dependency.
- Desktop Chrome, 1280×800 minimum, works at 1920+; glance check still works at ~900 px wide (IDE side by side).
- Read-only: browser never runs robot code or edits Java.
- Scrubbing stays instant at ~1,800 frames per run; motion plays at 60 fps.
- Honesty rule: every unmeasured/estimated value shows the grey `~` box (`.est`), and the mark follows it into every result that depends on it. `</>` + file name = read from TeamCode. No mark = measured.
- Status is never color alone: always the rounded-square badge icon + a word. Alliance: `▲ RED`, `◆ BLUE`.
- Purple `#7A61AC` is an accent, never a background, and never means an alliance. Used for: playhead, primary button, selection, changed values, planned path.
- Fonts: Silkscreen (logo, verdict word, score only), IBM Plex Sans 600/500/400 at 30/17/15/13/12 px, IBM Plex Mono for every number, time, file and decide() state. Load from Google Fonts with system fallbacks (works offline in the pit, just plainer).
- Motion: 120 ms hover/press, 200 ms panels/popovers, 320 ms run swap, `cubic-bezier(.2,.8,.2,1)`; `prefers-reduced-motion`: fades only.
- Copy voice: plain, short, literal ("The robot touched the top wall at 12.40 s"). No marketing tone.
- WCAG AA contrast in both themes.
- Never edit Gradle files (memory: ftc-gradle-changes-need-flagging). Darren owns git: commit steps are "stage + propose message"; don't push.

## Design tokens (copy verbatim into `style.css`)

Dark (from the Kit board): `--bg #0d0f12`, `--bar #121417`, `--panel #16191e`, `--line #23272d`, `--line-2 #2a2f37`, `--text #e8eaed`, `--body #c4cad1`, `--muted #9aa3ad`, `--accent #7A61AC`, `--accent-text #A993E0`, `--selected #221c33`, `--pass #3DD68C/#0f2a1e/#1f5a3d`, `--warn #F5A524/#2b2210/#5c4317`, `--fail #FF6B6B/#2e1416/#6b2a2e`, `--none #9aa3ad/#181b20/#2a2f37` (fg/bg/ring). Field: floor `#1E2126`, wall `#4A505A`, tile grid `#30343B`; alliance red/blue keep today's `--red`/`--blue`.
Light: derived, not designed. Same token names, under `@media (prefers-color-scheme: light)` and `:root[data-theme="light"]`. Start from: bg `#f6f6f8`, bar `#ffffff`, panel `#ffffff`, line `#e3e4e8`, text `#16181d`, body `#3b4049`, muted `#646b76`, accent `#7A61AC`, accent-text `#5d4596`, selected `#eee9f8`; status fg darkened to pass `#14804f`, warn `#a56a00`, fail `#c93b3b` on tints `#e6f6ee/#fbf1dc/#fbe7e7`. Verify AA with Chrome DevTools contrast and flag to Darren for a visual pass.

Component classes from the Kit (keep the names): `.sb .sb-pass|warn|fail|none` (20 px badge, 6 px radius, 12 px icon, 1.9 px round stroke), `.btn`, `.btn-p`, `.kbd`, `.m` (mono), `.h` (11 px 600 uppercase .08em muted), `.card` (12 px radius, 18 px pad), `.chip` (30 px), `.pill` (18 px), `.ic` (32 px icon tile, 1.6 px stroke), `.est`, `.src`, `.tag`.

## Layout (from the boards, 1440×900)

- Top bar 56 px, `--bar`, bottom border. Contents left→right: logo (Silkscreen "KALIPSO AUTO SIM", links `#/`), auto button (name · alliance tag · run chip), score chip, spacer, Run on save switch, Parameters button with changed-count pill, Run `.btn-p` with `R` kbd.
- Workspace: left rail 330 px (320 when the details drawer is open) | field `main` flex 1 | details drawer 360 px (only when open) ; timeline 170 px full width under all three.
- Parameters: nav 230 px | list main (14/18 px pad) | right aside 330 px (start-pose preview, legend, Reset all, primary "Run"/"Fix N problems to run").
- Below 1100 px wide: rail collapses to a 56 px verdict strip at the top of the field; details drawer overlays the field. Below 900 px: top bar hides Run on save text (switch only) and the score breakdown.

## Keyboard scheme (write into the `?` overlay and README)

Space play/pause · ←/→ one frame, Shift ×10 · Home/End · `[` `]` speed · `V` 2D/3D · `F` follow robot (3D) · `R` run · `N` / Shift+`N` next/previous problem · `D` details drawer · `Esc` close drawer / back to summary · `/` search parameters · `?` shortcuts. (`P` is retired; `?panel=0` maps to the drawer.)
Deep links: `#/run?t=12.4&play=1&details=1&mode=2d`.

## Review Focus

1. **Stale run read as new:** a .java save while a run is showing → chip goes to "Run #48 is out of date" and the rail/field get the `.old` treatment the moment the server enters `building`, not when the build ends. Test: Task 8 state-model test.
2. **900 px window beside the IDE:** verdict word, run chip and "changed since" must stay visible with no horizontal scroll. Test: Task 5 manual check at 900×800 in Chrome device toolbar.
3. **Dense events (shot + pickup + contact in 0.2 s):** markers closer than 6 px merge into one pill with a count; the rail "Near this moment" groups "3 EVENTS IN 0.03 s". Test: Task 4 `clusterMarkers` test.
4. **True vs odometry pose overlapping:** odometry is a dashed outline drawn above the true body; when the gap > `THRESHOLDS.odoMmWarn` a gap label appears. Test: Task 7 `gapLabel` test.
5. **Run swap losing context:** a new run keeps time, 2D/3D, 3D camera, rail sub-view and scroll. Test: Task 5 Step 3 manual re-run check (camera, time) and Task 6 Step 3 (rail sub-view kept across a re-run).

---

### Task 0: Design extraction helper (job-local, not committed)

Only needed if the decoded boards are gone. Python, run with `-I`, outside the Downloads dir:
decode root `<script type="__bundler/manifest">` JSON → for each uuid in `__bundler/page_order`: `gzip.decompress(base64.b64decode(data))` → page HTML; in each page repeat for its own manifest + `__bundler/template` (JSON string), replacing uuids with extracted file names. Board order: 0 Home, 1 Workspace 2D, 2 Workspace 3D+details, 3 Parameters, 4 Compile error, 5 Kit, 6 Field 2D, 7 Timeline. Read boards with inline styles; ignore bundled React/three/runtime JS.

### Task 1: Tokens, fonts, base components

**Files:**
- Modify: `sim/app/web/style.css` (replace the `:root` token blocks lines 1–45 and base button/select rules; delete rules for `#landing .hero`, `#setup`, `#stats` cards as their views are rebuilt in later tasks)
- Modify: `sim/app/web/index.html` (`<head>`: Google Fonts link for IBM Plex Mono 400/500/600, IBM Plex Sans 400/500/600/700, Silkscreen 400/700)
- Modify: `sim/app/web/app.js:136` (`buildScene` color names: add `--floor`, `--wall`, `--grid`; keep `--accent` for the true robot and trail, `--ghost` becomes `--body` for odometry)

**Interfaces:**
- Produces: CSS custom properties listed under *Design tokens*; component classes `.sb .btn .btn-p .kbd .m .h .card .chip .pill .ic .est .src .tag .old`; `.old` = `opacity:.55; filter:saturate(.4)` plus an "OLD" tag slot.
- Produces: `web/icons.js` exporting `icon(name: string): SVGElement` for the Kit set: `pass warn fail none play pause back fwd chevron robot shooter intake ball hive partner shots contact odometry pose motors sensors telemetry logs trust checks path code`. Paths copied from Kit board (t5) `<svg>` children.

- [ ] **Step 1:** Copy the dark tokens verbatim, then the derived light set. Map old names used by JS (`--accent --ghost --red --blue --obstacle --field --line --tile --good --warn --bad --dim`) to the new ones so `color()` lookups in `app.js` keep working; delete aliases no JS reads.
- [ ] **Step 2:** Add `icons.js`; replace the text glyph icons in `statsView.js` `ICONS` and `syncPlayButton()` (`▶`/`❚❚`) with `icon()`.
- [ ] **Step 3: Verify** `cd sim/app && ./start.sh`, open `http://localhost:5173/#/replay` in Chrome: dark background `#0d0f12`, Plex fonts load (DevTools → Rendered Fonts), field renders. Toggle DevTools "Emulate prefers-color-scheme: light": readable, no hard-coded dark colors left (`grep -n '#[0-9a-fA-F]\{6\}' web/*.js` returns only ball/alliance colors).
- [ ] **Step 4:** Stage `web/style.css web/index.html web/icons.js web/app.js web/statsView.js`; propose "sim ui: Kit tokens, fonts, icons".

### Task 2: Server data the design needs

**Files:**
- Create: `sim/app/buildError.mjs`, `sim/app/buildError.test.mjs`
- Modify: `sim/app/server.mjs` (move `cleanBuildOutput` lines 176–193 into `buildError.mjs`; `state`; `buildAndRun`; routes)
- Modify: `sim/params.json` (metadata only)

**Interfaces:**
- Produces: `parseCompileError(output: string, root: string, readFile: (p)=>string|null): { file, line, col, message, detail, context: [{ n, text, caret? }] } | null` — first `path:line: error: msg` block; `context` = lines line-1..line+1 of the source file; `detail` = the `symbol:`/`location:` lines joined with ` · `.
- Produces: `cleanBuildOutput(output, root)` (moved, unchanged behaviour).
- Produces: state fields `since: number` (ms epoch, set in `setPhase`), `error: <parseCompileError result> | null` (compile-error phase only), `lastSaveAt: number` (set in the `fs.watch` callback even when `watching` is false).
- Produces: `GET /api/run?prev=1` → the run before the latest (404 when none). Server keeps `previousRun = latestRun` before replacing it.
- Produces: `params.json` optional fields per entry: `"unmeasured": true` (every entry whose description says "Unmeasured" or "no mass"), `"affects": ["shots"|"tips"|"score"|"contacts"|"side"|…check ids]`, `"source": "RobotFootprint.java"` (robot size). Group `"Shooting"` renamed `"Shooter"`.

- [ ] **Step 1: Failing test** `buildError.test.mjs`:
```js
test('first compile error becomes file, line, message and three context lines', () => {
  const out = '/r/TeamCode/src/main/java/a/RedCloseAuto.java:142: error: cannot find symbol\n        shooter.shootVolly(3);\n               ^\n  symbol:   method shootVolly(int)\n  location: variable shooter of type Shooter\n';
  const src = Array.from({ length: 150 }, (_, i) => `line${i + 1}`).join('\n');
  const e = parseCompileError(out, '/r', () => src);
  assert.equal(e.file, 'TeamCode/src/main/java/a/RedCloseAuto.java');
  assert.equal(e.line, 142);
  assert.equal(e.message, 'cannot find symbol');
  assert.equal(e.detail, 'symbol: method shootVolly(int) · location: variable shooter of type Shooter');
  assert.deepEqual(e.context.map((c) => c.n), [141, 142, 143]);
});
test('no error line gives null', () => assert.equal(parseCompileError('BUILD FAILED', '/r', () => null), null));
test('unreadable source still gives the message with no context', () => assert.deepEqual(parseCompileError(out142, '/r', () => null).context, []));
```
- [ ] **Step 2:** `cd sim/app && node --test buildError.test.mjs` → FAIL (module missing).
- [ ] **Step 3:** Implement `buildError.mjs`; wire into `server.mjs` (`setPhase('compile-error', cleanBuildOutput(...))` also sets `state.error`); add `since`, `lastSaveAt`, `previousRun`, `?prev=1`.
- [ ] **Step 4:** `node --test` → all pass (stats + buildError).
- [ ] **Step 5: Verify** with the server running: `curl -s localhost:5173/api/state | jq '.since,.lastSaveAt'`; after two runs `curl -s 'localhost:5173/api/run?prev=1' | jq .meta.runId` is one less than `/api/run`.
- [ ] **Step 6:** Stage; propose "sim server: previous run, structured compile error, param metadata".

### Task 3: Run diff, score, problems (pure, `stats.js`)

**Files:**
- Modify: `sim/app/web/stats.js`
- Test: `sim/app/stats.test.mjs`

**Interfaces:**
- Consumes: `analyse(run)` (existing), `params.json` defs from Task 2.
- Produces:
  - `SCORING = { leave: 3, tip: 20, park: 5, leaveMm: 300 }` — **design values, confirm against the BIOBUZZ manual** (comment says so).
  - `score(run, a): { total, final, items: [{ t, label, points, kind: 'leave'|'tip'|'park', est: boolean }] , est: boolean, at(t): number }`. LEAVE when the true pose first gets `leaveMm` from the start pose; TIP per `" tipped"` event of our alliance's Hive (`est: true` — rests on unmeasured `hiveTipBalls`); PARK when the last decide() move is `PARK` and the robot is standing still at the end (`est: true`, label "Parked (no park zone in the field model yet)"). `total` = points with `t ≤ time` via `at(t)`; `final` = all; `est` = any item est.
  - `estimatedChecks(paramDefs): Set<string>` — check ids (and `'score'`) named in `affects` of any `unmeasured` def.
  - `placeName(name): string` — `wall-yMin` "the top wall", `wall-yMax` "the bottom wall", `wall-xMin` "the left wall", `wall-xMax` "the right wall", `flower-A` "Flower A" (B–D alike), `hive-leg-low`/`hive-leg-high` "the Hive's low/high leg", `partner` "R2"; anything else unchanged. Used by check details, diff labels and the rail.
  - `diffRuns(prev, cur): [{ kind: 'new'|'fixed'|'changed', id, label, from?, to?, better?: boolean, seek? }]` comparing two `analyse()` results: a check that went pass→warn/fail is `new` (contacts: `New: touched ${placeName(first)}`; others `New: ${label} needs a look` / `fails`), warn/fail→pass is `fixed` (`Fixed: ${label}`), and the numbers `shotsIn` (of `shots`), `odoMax` (mm), `pickups`, `contacts`, score `final` are `changed` when different. Ordered new, fixed, changed. Empty when `prev` is null.
  - `problems(a): check[]` — non-pass checks ordered by `seek` time; `nextProblem(a, t, dir: 1|-1): check|null` wraps around.

- [ ] **Step 1: Failing tests** (extend the fixture `run()`):
```js
test('diff: a new contact is a new problem, fewer shots in is worse', () => {
  const prev = analyse(run()), cur = analyse(run({ events: [{ t: 12.4, text: 'robot touched wall-yMin' }] }));
  const d = diffRuns(prev, cur);
  assert.equal(d[0].kind, 'new'); assert.equal(d[0].id, 'contacts'); assert.equal(d[0].seek, 12.4);
  assert.deepEqual(diffRuns(null, cur), []);
});
test('score: leave + one red tip, tip is an estimate', () => {
  const r = run({ events: [{ t: 9.8, text: 'red-hive tipped' }] }); r.alliance = 'red';
  const s = score(r, analyse(r));
  assert.equal(s.final, SCORING.leave + SCORING.tip); assert.equal(s.est, true);
  assert.equal(s.at(5), SCORING.leave); assert.equal(s.at(10), SCORING.leave + SCORING.tip);
});
test('score: a blue tip does not count for red', () => { /* final === SCORING.leave */ });
test('nextProblem wraps from the last problem to the first', () => { /* two warn checks with seek 2 and 4; nextProblem(a, 4.5, 1).seek === 2 */ });
test('estimatedChecks follows affects of unmeasured params only', () => {
  assert.deepEqual([...estimatedChecks([{ id: 'a', unmeasured: true, affects: ['tips', 'score'] }, { id: 'b', affects: ['shots'] }])].sort(), ['score', 'tips']);
});
```
- [ ] **Step 2:** `node --test stats.test.mjs` → new tests FAIL.
- [ ] **Step 3:** Implement the five exports in `stats.js`.
- [ ] **Step 4:** `node --test` → PASS.
- [ ] **Step 5:** Stage; propose "sim stats: run diff, auto score, problem navigation".

### Task 4: Lane timeline + transport

**Files:**
- Create: `sim/app/web/timeline.js`
- Test: `sim/app/timeline.test.mjs`
- Modify: `sim/app/web/index.html` (replace `<footer id="transport">` with `<footer id="timeline">`), `sim/app/web/app.js` (transport handlers lines 489–572 move behind `timeline.js` callbacks)

**Interfaces:**
- Consumes: `score()` (Task 3), `analyse().decisions`, `run.events`.
- Produces:
  - `eventKind(text): 'shot'|'in'|'pickup'|'tip'|'contact'|'decide'|'other'` (today's inline classifier at `app.js:466`).
  - `clusterMarkers(events: {t,text}[], duration, widthPx, minGapPx = 6): [{ t, kinds: Set, events, label }]` — single pass over time-sorted events, merge while next is within `minGapPx`. Shot clusters label like the design: `"3 shots"`, `"3 · 1 miss"`, `"2 · 2 miss"` (miss = shot without a following `went into` within the cluster).
  - `mountTimeline(root, { duration, lanes, onSeek(t), onPlay(), onStep(dir, big), onSpeed(x) }): { setTime(t), setPlaying(b), setRun(model), setStale(b) }`. Lanes: EVENTS (pills/markers), DECIDE (segments, mono labels, accent for the current one), SCORE (step line, `+3 LEAVE` / `+20 TIP ~` pills, total "23 pts" at right), PARTNER (Task 13). Header row 32 px: back, play, forward, `12.40 s / 30.00` mono, speed button cycling 0.25–8×, `+ Lanes` (toggle lanes on/off, kept in `localStorage` with try/catch), `?` shortcuts.
  - `?` (button or key) opens a native `<dialog>` listing the *Keyboard scheme* section verbatim; `Esc` closes it.
  - Scrub: pointer drag anywhere on the lanes area sets time; the playhead is a 2 px `--accent` line across all lanes. Keep a hidden `<input type="range">` for keyboard/a11y.

- [ ] **Step 1: Failing test:**
```js
test('events closer than 6 px merge into one pill with a count', () => {
  const ev = [{ t: 12.38, text: 'picked up a pollen ball (1 held)' }, { t: 12.40, text: 'robot touched wall-yMin' }, { t: 12.41, text: 'shot a pollen ball' }, { t: 20, text: 'red-hive tipped' }];
  const c = clusterMarkers(ev, 30, 1200);
  assert.equal(c.length, 2); assert.equal(c[0].events.length, 3); assert.ok(c[0].kinds.has('contact'));
});
test('shot clusters count misses', () => {
  const ev = [0, 0.01, 0.02].flatMap((t, i) => [{ t, text: 'shot a pollen ball' }, ...(i < 2 ? [{ t: t + 0.005, text: "a pollen ball went into the red Hive's up Cell" }] : [])]);
  assert.equal(clusterMarkers(ev, 30, 1200)[0].label, '3 · 1 miss');
});
```
- [ ] **Step 2:** `node --test timeline.test.mjs` → FAIL.
- [ ] **Step 3:** Implement `timeline.js` (pure helpers at top, DOM in `mountTimeline`); hook into `app.js` frame loop (`setTime` each frame, no DOM rebuild per frame — move the playhead with `transform`).
- [ ] **Step 4:** `node --test` → PASS. Manual: scrub 30 s run back and forth fast in Chrome Performance panel, no long tasks > 16 ms.
- [ ] **Step 5:** Stage; propose "sim ui: lane timeline with merged markers and score lane".

### Task 5: Workspace shell and routing

**Files:**
- Modify: `sim/app/web/index.html` (body becomes: `#bar` top bar; `#home`; `#work` = `aside#rail` + `main#fieldArea` (contains `#stage`, view toggle, legend, "At 12.40 s" chip) + `aside#details`; `#params`; `#timeline`); delete `#landing`, `#setup`, `#stats`, `#panel`, `#overlay`, `#progress`.
- Modify: `sim/app/web/app.js` (`parseHash`/`route`/`setView` lines 664–696; `layout()` lines 327–357; `buildScene()` 123–201; `loadRun()` 625–658)

**Interfaces:**
- Produces: views `'home' | 'run' | 'params'`; `go(view, query?)`. Old hashes redirect: `#/stats` → `#/run`, `#/replay` → `#/run`, `#/setup` → `#/params`.
- Produces: `layout()` reads the field area's `getBoundingClientRect()` instead of the hard-coded `padTop=68, padBottom=84, padRight=344` (so rail/drawer/timeline sizes come from CSS only); the eased `padRight` animation is replaced by the 200 ms CSS width transition of `#details`.
- Produces: `buildScene({ keepCamera: true })` — reads `scene3d.camera.position` and `scene3d.controls.target` before `dispose()` and calls `scene3d.setCamera(...)` after rebuild. `loadRun` keeps `time` (clamped), `viewMode`, rail sub-view and `#rail.scrollTop`.
- Produces: field overlay chip (bottom-left of field): `At 12.40 s · Details →` + decide() move, odometry gap, holding `n of 4` (all `.m`); clicking opens the drawer.
- Legend (top-right of field): `■ True`, `┅ Odometry`, `┅ Planned` only when the run has a planned path (none today → hidden), `◆ Contact`, `■ R2 partner` (Task 13).
- Score chip in the top bar (button, aria-label "Score details"): `AUTO SCORE` / `at 12.4 s` caption, `score.at(time)` in Silkscreen, `final` + mono `28` + `~` when `score.est`. Click opens a 200 ms popover: one row per `score.items` (mono time, label, `+3 LEAVE` pill, `~` when est); items after the playhead are muted with "after playhead"; footnote "The tip rests on the unmeasured tip mass, so the total carries ~." when est.
- `F` toggles follow in 3D: new `scene3d.follow(on: boolean)`; while on, `update()` moves `controls.target` to the robot pose each frame (camera offset kept). Caption under the legend: "Drag to orbit · scroll to zoom · F follow robot".

- [ ] **Step 1:** Restructure `index.html`; CSS grid for `#work`: `grid-template: 1fr 170px / 330px 1fr auto`, timeline spans all columns; `#details` width 0 ↔ 360 px; rail 320 px when `[data-details=open]`. Media queries per *Layout*.
- [ ] **Step 2:** Routing + redirects; `layout()` from the bounding rect; `keepCamera`.
- [ ] **Step 3: Verify** in Chrome at 1440×900, 1280×800 and 900×800 (device toolbar): no horizontal scroll, field centred in the free area, 3D orbit works, `#/replay?t=12` lands on `#/run` at 12.00 s. Trigger a re-run (save any `.java`) while at t=12 in 3D with a rotated camera: after the swap time stays 12.00 s, camera unchanged.
- [ ] **Step 4:** Stage; propose "sim ui: single workspace, routes, context-keeping run swap".

### Task 6: Summary rail (replaces the Stats page)

**Files:**
- Rename: `sim/app/web/statsView.js` → `sim/app/web/rail.js` (keep `h()`, `svg()`, `lineChart()`, `niceMax()`; delete `renderStats`)
- Modify: `sim/app/web/app.js` (`refreshStats` 717–728 → `refreshRail`)

**Interfaces:**
- Consumes: `analyse`, `diffRuns`, `score`, `estimatedChecks`, `problems`, `nextProblem` (Task 3); `icon()` (Task 1).
- Produces: `renderRail(root, model, actions)` where `model = { a, run, prevA, diff, score, est: Set, view: 'summary'|'check'|'checks'|'numbers'|'trust', checkId? }` and `actions = { seek(t), open(view, checkId?), rerun() }`.
  - **summary:** verdict card (status badge + Silkscreen verdict word "NEEDS A LOOK", "2 of 10 checks need a look · vs #47: **1 new problem**", border tinted by status); "Changed since #47" rows (`.ic` tile tinted warn for new problems, label, `6 → 5` mono, ↑/↓ with *better/worse* word in `aria-label`, seek time right-aligned and clickable); "More" rows: All checks (`8 pass · 2 need a look`), How far to trust this (`~ 3 guesses affect shots and the tip`), Run numbers (`5 picked up · 8 shots · 34 ft driven`). Partner R2 row added in Task 13.
  - **check:** "← Back to summary"; badge + label + status word; detail sentence with mono numbers; "Pass: … / Fail: …" thresholds line from `THRESHOLDS`; `← Prev` / `Next problem N` buttons; "Near this moment" = events within ±0.5 s of `seek`, grouped with the Task 4 cluster ("3 EVENTS IN 0.03 s").
  - **checks:** every check row (badge, label, one-line detail, `~` if in `est`), click → check view + seek.
  - **numbers:** today's tiles condensed to rows + the speed / odometry / held `lineChart`s (reuse), seek on click.
  - **trust:** unmeasured params used by this run (label, value, unit, `~`, what it affects) + `run.assumed` list + the static placeholders now in `statsView.js:234–236`.
- Distances shown in ft/in per the Kit units table (`34 ft`, `4.8 ft/s`), odometry in mm.

- [ ] **Step 1:** Port `h/svg/lineChart` into `rail.js`; write the five views.
- [ ] **Step 2:** `N`/Shift+`N` call `nextProblem` → open check view + seek; `Esc` returns to summary.
- [ ] **Step 3: Verify** with BallShowcaseAuto (has shots, tip, contacts): every seek row moves the playhead; Next problem cycles; second run shows a "Changed since #N" list; first run after server start shows "First run, nothing to compare yet." in place of the list; open a check view, trigger a re-run → still on that check view, same scroll.
- [ ] **Step 4:** Stage; propose "sim ui: summary rail with run diff and problem navigation".

### Task 7: Details drawer

**Files:**
- Modify: `sim/app/web/app.js` (panel code lines 359–483 → `details.js`)
- Create: `sim/app/web/details.js`
- Test: `sim/app/details.test.mjs`

**Interfaces:**
- Produces: `gapLabel(pose, warnMm): string | null` — `"gap 38 mm · 2.8°"` when odometry is known, else null; field shows the label next to the robot only when gap > `warnMm`.
- Produces: `mountDetails(root, { onClose }): { update(t, pose, run, a) }` — header `At 12.40 s` + ✕ (aria-label "Close details"); decide() block (`COLLECT_A`, `since 9.90 s · was WAIT_TIP` from `a.decisions`); accordions (`.acc`, `aria-expanded`): **Pose** (True / Odometry `x, y in · heading°`, summary = `gapLabel`), **Holding** (`1 of 4` from `run.balls.held`), **Balls** (today's `ballCounts`), **Telemetry** (`9 keys`, today's list), **Events near here**. No drive power / shooter / sensors / logs sections (not recorded; Darren's call).
- `update` must not rebuild DOM when the frame index has not changed (today's `shownTelemetryIndex` pattern).

- [ ] **Step 1: Failing test:**
```js
test('gap label only when odometry is known', () => {
  assert.equal(gapLabel({ x: 0, y: 0, theta: 0, ox: 38, oy: 0, otheta: 2.8 * Math.PI / 180 }, 30), 'gap 38 mm · 2.8°');
  assert.equal(gapLabel({ x: 0, y: 0, theta: 0, ox: NaN, oy: NaN, otheta: NaN }, 30), null);
});
```
- [ ] **Step 2:** FAIL → implement → `node --test` PASS.
- [ ] **Step 3:** Odometry robot: dashed outline (`LineDashedMaterial`) drawn at z above the true body so overlap stays visible; true body solid accent fill.
- [ ] **Step 4: Verify:** `D` and the field chip open/close the drawer with a 200 ms width transition; `?details=1` opens it on load; scrubbing at 60 fps with the drawer open stays smooth.
- [ ] **Step 5:** Stage; propose "sim ui: details drawer".

### Task 8: Run state chip and failure states

**Files:**
- Create: `sim/app/web/runState.js`
- Test: `sim/app/runState.test.mjs`
- Modify: `sim/app/web/app.js` (`applyState` 584–623, `connect` 1151–1155, delete overlay code), `sim/app/web/rail.js` (error card)

**Interfaces:**
- Consumes: server state `{ phase, message, runId, watching, since, lastSaveAt, error }` (Task 2).
- Produces: `chipFor(state, loadedRun, now): { status: 'pass'|'warn'|'fail'|'none'|'busy', title, sub, action?: 'retry'|'run', old: boolean }`:
  - done & loaded run is latest → pass/warn/fail by verdict, `Run #48`, `fresh · 21:41:07`, old false.
  - building/running with a run loaded → busy `Compiling` / `Simulating`, sub `6 s` from `since`, **old true**.
  - `lastSaveAt > loadedRun.finishedAt` and not watching → none `Run #48 is out of date`, sub `press R`, old true.
  - compile-error → fail `Build failed`, sub `21:44:19 · showing old run #48`, action retry, old true.
  - run-error → fail `Run failed`, sub `crashed at 14.2 s` (last frame time), old false (partial run replays).
  - offline → fail `Runner disconnected`, action retry.
  - not watching, otherwise fresh → extra chip none `Run on save off`, sub `press R`.
- Produces: one error card pattern (`rail.js` `errorCard({ title, where, raw, stillWorks, action })`, `role="alert"`): compile error shows `file:line:col` (mono link text), message, the 3 context lines with line numbers and caret, detail line, `Copy error` (clipboard) and `Retry build`, footer "Fix it in Android Studio and save. A new run starts by itself."; below it an `.old` mini verdict "OLD · Last good run · #48 · 21:41:07". Runtime error: first stack line, `Jump to 14.20 s`. Offline: "Can't reach the sim server. Start it with sim/app/start.sh", Retry reconnects the EventSource.
- `.old` applied to `#rail` summary, field and timeline while `old` is true; field caption "Replay of old run #48 · still works while you fix the build".

- [ ] **Step 1: Failing tests** — one per bullet above, e.g.:
```js
test('a save while a run shows marks it old at once', () => {
  const c = chipFor({ phase: 'building', since: 1000, runId: 48, watching: true }, { meta: { runId: 48 } }, 7000);
  assert.equal(c.title, 'Compiling'); assert.equal(c.sub, '6 s'); assert.equal(c.old, true);
});
test('run on save off and a newer save says out of date', () => { /* title 'Run #48 is out of date', sub 'press R' */ });
```
- [ ] **Step 2:** FAIL → implement → PASS.
- [ ] **Step 3:** Tick the Compiling seconds with a 1 s interval only while busy.
- [ ] **Step 4: Verify** in Chrome: break a Java line (e.g. rename a method call in `BallShowcaseAuto`), save → chip Compiling then Build failed; rail shows file:line and the source lines; old run still plays. Fix and save → fresh run, `.old` gone. Kill the server → Runner disconnected; restart + Retry → reconnects. Revert the Java change.
- [ ] **Step 5:** Stage; propose "sim ui: run state chip and error cards".

### Task 9: Parameters screen

**Files:**
- Create: `sim/app/web/params.js` (move Setup code `app.js:731–1046`: `loadParams`, `buildParams`, rows, `setParam`, preview, `updateChecks`, `overlapsBox`)
- Test: `sim/app/params.test.mjs`
- Modify: `sim/app/web/index.html` (`#params` view), `sim/app/web/app.js`

**Interfaces:**
- Consumes: `/api/params` (`defs` with `unmeasured/affects/source`, `defaults`, `values`); `estimatedChecks` label names.
- Produces: `filterParams(defs, values, defaults, { filter: 'all'|'changed'|'problems'|'unmeasured'|'code'|<group>, query }): def[]` and `paramProblem(def, value): string | null` (`"6,800 RPM is above the max (6,000). Runs are blocked until you fix it."` / below-min twin; null when in range).
- Screen: header "← Workspace", `Parameters` + `26 · sim values first, then your TeamCode config`, search box (`/` focuses, matches label, id, description, group). Nav: All / Changed / Problems (fail badge) / `~` Unmeasured / `</>` From your code, then one row per group with an `.ic` icon and count. Main: one section per group, header (icon tile, group name, one-line subtitle, `</> File.java` source tag, "1 changed", "3 unmeasured", "1 problem"); each row: label (+ "sim override · code says 16.0" when changed and `source`), number input, unit, `~` column, reset `↺` (aria-label "Reset width to 16.0"). The row with focus or a problem expands: description, range bar with min/default/max marks, problem text with fail badge, "Affects" chips. Choice params (`shotMode`) stay a segmented control. Sliders are dropped (number box + range bar only).
- Aside: "Start pose preview" (reuse `drawPreview`; code value shown dashed), today's start-pose checks as `.sb` rows, legend (`~` guess, `</>` read from TeamCode, accent = changed in the sim), `2 changed · 1 problem`, `Reset all`, primary button: `Run` or `Fix 1 problem to run` (disabled while problems > 0). Run posts params (existing `requestRun({ params })`) then `go('run')`.

- [ ] **Step 1: Failing tests:**
```js
test('changed filter shows only values that differ from the default', () => { /* 2 defs, 1 changed → 1 */ });
test('search matches label and description, case-insensitive', () => { /* 'flywheel' finds shotSpeedPerRps */ });
test('out of range gives a blocking problem with the unit', () => {
  assert.match(paramProblem({ label: 'Width', unit: 'mm', min: 150, max: 600 }, 700), /above the max \(600\)/);
  assert.equal(paramProblem({ label: 'Width', unit: 'mm', min: 150, max: 600 }, 400), null);
});
```
- [ ] **Step 2:** FAIL → implement → PASS.
- [ ] **Step 3: Verify:** type 700 in Width → row turns fail, nav Problems shows 1, Run button reads "Fix 1 problem to run" and is disabled; Reset all clears; server still rejects bad values (`curl -XPOST -d '{"params":{"robotWidth":700}}' localhost:5173/api/run` → 400).
- [ ] **Step 4:** Stage; propose "sim ui: parameters screen with filters, search and blocking checks".

### Task 10: Home

**Files:**
- Modify: `sim/app/web/index.html` (`#home`), `sim/app/web/app.js` (`buildCards`/`updateLanding` 1052–1087), `sim/app/web/scene3d.js` (no API change; reuse `build3D`)

**Interfaces:**
- Consumes: `/api/opmodes`, last run + `analyse`, `chipFor`.
- Screen: 3D field canvas behind, from the last run's `build3D` (drag to orbit; no auto-orbit). If no run exists yet, show the dark tile-grid background instead: no fake robot, no fake run. Left column: eyebrow `KALIPSO AUTO SIM`, h1 "Test your auto before the robot does.", sub "Runs your real `@Autonomous` code on a simulated robot and field."; `Your auto · N found in TeamCode` list of `.op` rows (icon tile, mono name, alliance tag, `start x, y in · θ°`, right side: last verdict badge + `#48`, or "not run yet"); Partner robot segmented control (None / Path file / Draw a path `LATER` disabled) + selected path row (Task 13 fills it); primary `Run RedCloseAuto R`; Run on save card ("Run on save is on · Saving a Java file in Android Studio starts a run."); status line "Runner connected · physics ready" and `Last results →`. Caption bottom-right "Last run at 12.4 s · drag to orbit".
- Start positions shown in inches (`mm / 25.4`, 0 dp).

- [ ] **Step 1:** Build the view; `Enter`/`R` runs the selected auto and opens `#/run`.
- [ ] **Step 2: Verify:** first open with the server just started (no run): no empty dashboard, one obvious action; after a run, the row shows its verdict badge and number.
- [ ] **Step 3:** Stage; propose "sim ui: home".

### Task 11: Partner R2 in the physics world (Node)

R2 follows its path file exactly, never runs code, never shoots, but is solid: R1 bumps into it and it pushes balls.

**Files:**
- Modify: `sim/physics/world.mjs` (collision groups :29-37; robot body :103-129; `step()` :412-466; contact loop :448-461; start-pose reset :469-480)
- Modify: `sim/physics/server.mjs` (protocol comment :1-8; new `partner-start` request; `step` reply)
- Test: `sim/physics/partner.test.mjs` (same style as `world.test.mjs`: `node:test`, `createWorld(init(...))`, reuse its `init(pose, boxes)` helper shape)

**Interfaces:**
- Consumes: init JSON gains optional `partner: { name: string, front: mm, back: mm, width: mm, speed: number /* 1 = 100% */, poses: [[tSec, xMm, yMm, thetaRad], …] }` (t from the partner's own start, ascending).
- Produces:
  - New group bit `G_PARTNER = 32`. Partner collider `groups(G_PARTNER, G_ROBOT | G_BALL)`; add `G_PARTNER` to the robot filter and the ball filter. Partner body: `RigidBodyDesc.kinematicPositionBased()`, cuboid sized like R1's (same `ROBOT_HEIGHT_M`), offset the same way from its reference point.
  - `partnerPoseAt(poses, tSec): [x, y, theta]` (exported, pure): linear interpolation, theta via shortest arc; holds the first pose before 0 and the last after the end.
  - Each `step`: before `world.step()`, `setNextKinematicTranslation/Rotation` from `partnerPoseAt(poses, (nanos - partnerStartNanos) / 1e9 * speed)`; before `partner-start` the partner holds `poses[0]`. The ball-settle start-pose reset also puts R2 back on `poses[0]`.
  - Contact: R1↔R2 counted in the existing contact loop under the name `partner` → Java turns it into `robot touched partner`.
  - Protocol: `partner-start <nanos>` → `ok`. `step` reply appends ` px py ptheta` (mm, mm, rad) at the end **only when a partner exists**, so old readers are unchanged.

- [ ] **Step 1: Failing tests:**
```js
test('partner follows its path exactly', () => {
  const w = createWorld({ ...init([500, 500, 0]), partner: { name: 'p', front: 200, back: 200, width: 400, speed: 1, poses: [[0, 2500, 2500, 0], [2, 2500, 1500, 0]] } });
  w.partnerStart(0); stepTo(w, 1e9);
  const [x, y] = w.partnerPose(); assert.ok(Math.abs(x - 2500) < 1 && Math.abs(y - 2000) < 2);
});
test('R1 driving into the partner stops and reports one partner contact', () => { /* partner parked at x=1500, R1 drives +x from 500; final R1 x < 1500 - front - partner.back + 5; contact names deepEqual ['partner'] */ });
test('partner speed 0.5 is halfway along at the same time', () => { /* speed 0.5 at 2 s → y ≈ 2000 */ });
test('no partner: step reply has no extra fields', () => { /* server-level: parse reply token count unchanged */ });
test('partnerPoseAt holds the ends', () => { assert.deepEqual(partnerPoseAt([[0, 1, 2, 0], [1, 3, 4, 0]], -1), [1, 2, 0]); });
```
(`stepTo`, `partnerStart`, `partnerPose` follow the existing `debugStep` pattern at `world.mjs:523-532`.)
- [ ] **Step 2:** `cd sim/physics && node --test` → new tests FAIL.
- [ ] **Step 3:** Implement; existing 18 tests must stay green (determinism test included).
- [ ] **Step 4:** `node --test` → all PASS.
- [ ] **Step 5:** Stage; propose "sim physics: kinematic partner robot".

### Task 12: Partner R2 in the Java runner and run file

**Files:**
- Modify: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/kalipsorobotics/sim/RapierWorld.java` (`sendInit` :258-270, step-reply parsing, contact naming in `syncToNow` :234-240), `SimWorld.java` (default methods :40-79), `SimRunner.java` (frame :220-229, START), `SimMain.java` (`write()` :55-106, `writeGeometry` :111-127)
- Create: `sim/paths/red_partner_far.json` (sample)
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/kalipsorobotics/sim/PartnerSimTest.java` (JUnit 4, `runInNewJvm` like `RedTippingAutoSimTest`)

**Interfaces:**
- Consumes: Task 11 protocol.
- Produces:
  - System properties `-Dsim.partnerPath=<absolute path>` and `-Dsim.partnerSpeed=<0.25..1.5>` (default 1). Read in `RapierWorld.sendInit`; Java parses the file and sends `poses` inline (headings in the file are degrees, sent as radians); a missing/invalid file → `#warning Partner path <name> could not be read: <reason>` and no partner.
  - Path file format (document in README):
    ```json
    { "name": "red_partner_far", "robot": { "front": 200, "back": 200, "width": 400 },
      "poses": [[0, 254, 2743, 0], [3.0, 900, 2743, 0], …] }
    ```
    `[seconds from START, x mm, y mm, heading °]`, field frame same as `opmodes.json`. Sample: starts at 10, 108 in (254, 2743 mm) heading 0°, runs 24.5 s, ends parked, never crosses the midline; check its start with the same rules as `RapierWorld.checkStartPose`.
  - `SimWorld.partnerPose(): double[] /* x, y, theta */` default `null`; `RapierWorld` returns the last step reply's pose. SimRunner sends `partner-start <nanos>` when the match starts.
  - Run file lines (only when a partner exists): `#partnerinfo <name> <front> <back> <width>` (geometry block) and `#partner <t> <x> <y> <theta>` per recorded frame (same 20 ms cadence, same trimming at match end). Contacts arrive as `#event t|robot touched partner`.
- [ ] **Step 1: Failing test** `PartnerSimTest`: run `BallShowcaseAuto` with `-Dsim.partnerPath=sim/paths/red_partner_far.json`; assert `result` has `#partner` lines, the partner pose at the last frame equals the path's last pose within 2 mm, and two runs give identical partner tracks. Second test: an unreadable path gives a `#warning` and no `#partner` lines.
- [ ] **Step 2:** `JAVA_HOME=… ./gradlew :TeamCode:testDebugUnitTest --tests "*kalipsorobotics.sim.PartnerSimTest"` → FAIL.
- [ ] **Step 3:** Implement; parity and existing sim tests unchanged (`--tests "*kalipsorobotics.sim.*"` all green).
- [ ] **Step 4:** Stage; propose "sim runner: partner robot path, recording and contacts".

### Task 13: Partner R2 in the app

**Files:**
- Modify: `sim/app/server.mjs` (`parseRun` switch :237; `buildAndRun` java args :107-112; `POST /api/run`; new `GET /api/partners`), `sim/app/web/stats.js`, `sim/app/stats.test.mjs`, `sim/app/web/app.js` (2D scene), `sim/app/web/scene3d.js` (3D box), `sim/app/web/timeline.js` (PARTNER lane), `sim/app/web/rail.js`, `sim/app/web/params.js`, Home in `app.js`.

**Interfaces:**
- Server: `GET /api/partners` → `[{ file: 'red_partner_far.json', name, duration, start: [x, y, deg] }]` from `sim/paths/*.json`. `POST /api/run { partner: { file: string|null, speedPct: 25..150 } }` — `file` must be one of the listed names (no paths from the browser), else 400. Kept in `state.partner` (broadcast) and persisted to `sim/.last-partner.json`; passed as `-Dsim.partnerPath=<ROOT>/sim/paths/<file>` and `-Dsim.partnerSpeed=<speedPct/100>`. `parseRun` → `run.partner = { name, robot: { front, back, width }, frames: [[t, x, y, theta]] }` shifted by `matchStart` like `frames`.
- `stats.js`: `THRESHOLDS.partnerWarnMm = 150`; `rectGap(cornersA, cornersB): mm` (0 when overlapping; min of point-to-segment distances both ways); in `analyse`, when `run.partner` exists, add check `partner` "Avoids partner": fail on a `robot touched partner` event (seek = first one), warn when closest gap < `partnerWarnMm`, else pass; detail `Closest 9 in at 18.1 s.`; numbers `partnerClosest`, `partnerClosestAt`.
- Field: R2 drawn with a darker fill of the alliance colour, light alliance-coloured dashed path, label `R2`; 3D a box of its size. Legend `■ R2 partner · path file`.
- Timeline PARTNER lane: bar from 0 to the path end (`done 24.5 s`), `closest 9 in` marker at `partnerClosestAt` (title "Closest to R1: 9 in at 18.1 s").
- Rail summary "More": `Partner R2 · path file · closest 9 in at 18.1 s` → opens the `partner` check view.
- Params group "Partner robot" (nav count 2): path file `<select>` from `/api/partners` + None, speed % number (25–150). Home segmented control None / Path file / Draw a path (`LATER`, disabled); path row shows `R2 · start 10, 108 in · runs 24.5 s · no shooting`.

- [ ] **Step 1: Failing tests** (`stats.test.mjs`):
```js
test('rectGap is 0 when overlapping and the edge distance when apart', () => {
  const sq = (x) => [[x, 0], [x + 100, 0], [x + 100, 100], [x, 100]];
  assert.equal(rectGap(sq(0), sq(50)), 0); assert.equal(rectGap(sq(0), sq(250)), 150);
});
test('touching the partner fails Avoids partner', () => { /* run with partner frames + event 'robot touched partner' at 18.1 → status fail, seek 18.1 */ });
test('a close pass warns', () => { /* partner parked 100 mm from R1's path → warn, partnerClosest ≈ 100 */ });
test('no partner, no check', () => { assert.equal(analyse(run()).checks.some((c) => c.id === 'partner'), false); });
```
- [ ] **Step 2:** FAIL → implement → `node --test` PASS.
- [ ] **Step 3: Verify:** pick `red_partner_far.json` on Home, run BallShowcaseAuto: R2 moves in 2D and 3D, lane shows, the check appears; edit the sample path to cross R1's route → `robot touched partner` event, check fails, R1 visibly stops against R2. `curl -XPOST -d '{"partner":{"file":"../../etc/passwd","speedPct":100}}' localhost:5173/api/run` → 400.
- [ ] **Step 4:** Stage; propose "sim ui: partner robot R2".

### Task 14: Docs and cleanup

**Files:** `sim/README.md` (views, keys, deep links, partner path file format), delete dead CSS/JS left from the old views (`grep -n 'landing\|setup\|statsBody\|togglePanel' sim/app/web`).

- [ ] **Step 1:** Update README; remove dead code; `node --test` in `sim/app` and `sim/physics` all pass; Java sim tests pass.
- [ ] **Step 2:** Stage; propose "sim: docs for the new UI".

## Verification (end to end)

1. `cd sim/app && node --test` (stats, timeline, buildError, runState, params, details) — all pass.
2. `cd sim/physics && node --test` — all pass (incl. partner tests).
3. `JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:testDebugUnitTest --tests "*kalipsorobotics.sim.*"` — pass.
4. `sim/app/start.sh`, Chrome at `http://localhost:5173`: walk M1–M7 from `sim/design_spec.md` (first open, save→rebuild keeps time/camera, verdict glance from 2 m away, investigate a contact via Next problem, tune and block on a bad value, trust view, compile error/runtime error/disconnect) at 1440×900 and 900×800, dark and emulated light, and once with DevTools "prefers-reduced-motion".
