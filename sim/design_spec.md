# Auto Sim: UI / UX Design Spec

> **For the design agent:** This is a brief, not a wireframe. It tells you **who** uses the app, **what they need
> to get done**, **what data exists**, and the **hard constraints**. You choose the screens, the navigation, the
> layout and the visual language. The current app is described at the end only as a reference. Its page
> structure is **not** a requirement. If one screen can replace three, or the field should sit behind
> everything, or the "pages" should be modes of one workspace, do that and say why.

**Goal:** Turn "did my autonomous work, and if not, why?" from minutes of digging into a few seconds of looking.

**Product in one line:** A local browser app (`http://localhost:5173`, Chrome, laptop) that runs an FTC team's real
`@Autonomous` Java code against a simulated robot and field (BIOBUZZ, 2026-27), then shows what happened.

**What to deliver:**
1. A short design rationale: the mental model, the structure you picked, and what you rejected.
2. Hi-fi designs for the key moments and jobs below, in light and dark.
3. Every system state from the State Matrix.
4. A small token and component sheet (color, type, spacing, motion) that a developer can rebuild in plain HTML/CSS/JS.
5. Clickable prototypes for the 2–3 interactions where the feel decides the design (scrubbing, the run-on-save reload, going from a verdict to the moment it happened).

---

## Global Constraints

These are fixed. Everything else is open.

- **Platform:** desktop Chrome on a laptop, 1280×800 minimum, must still work at 1920+. Phones are out of scope.
- **Tech the result must fit:** plain HTML, CSS and ES modules, no framework and no build step. Three.js is already
  loaded for 3D. The 2D field is a `<canvas>`. Don't design anything that only works with React or a component library.
- **Read-only:** the browser never runs robot code and never edits Java. It shows runs and sets sim parameters.
- **Frame rate:** robot motion plays at 60 fps. Scrubbing must feel instant (about 1,800 frames per 30 s run).
- **Honesty rule (the most important one):** every number that is **unmeasured, estimated or unverified** must look
  different from a measured one, wherever it appears. The app must never make a run look better than the evidence supports.
- **Themes:** light and dark, both first-class.
- **Keyboard:** the whole replay works without the mouse. Today's keys (Space, ←/→, Shift+arrow ×10, `[` `]` speed,
  `P` panel, `R` re-run, `V` 2D/3D) can change if you have a better scheme. Write the scheme down.
- **Deep links:** any moment in a run can be opened from a URL (today: `?t=12`, `?play=1`, `?panel=0`).
- **Accessibility:** WCAG AA contrast. Status is never shown by color alone (red/blue alliance and pass/warn/fail
  each also need a shape, label or icon). Respect `prefers-reduced-motion`.
- **Copy voice:** plain, short, literal, like "The robot touched the wall at 12.4 s". No marketing tone, no "AI".
  Many users are high-school students.

---

## Users and context

- **Who:** FTC team programmers, ages about 14–18, plus a mentor. They know Java and their robot well. They are
  not UI experts and they do not want to learn a tool.
- **Loop:** edit Java in Android Studio → save → the sim rebuilds and re-runs **by itself** → glance at the browser →
  back to the code. This happens **dozens of times an hour**, often on a second monitor, or side by side with the IDE
  on one laptop screen.
- **Where:** at home, at practice, and in the pit at competitions (noisy, rushed, sometimes bright light or a small screen).
- **Emotional job:** confidence. "Can I trust this auto on the real robot?" The app is a skeptical teammate, not a cheerleader.

## Jobs to be done (design around these, not around pages)

Ordered by frequency. The first two are most of the use.

1. **Glance check after a save.** *"Did that change break or fix anything?"* An answer in under 2 s from across
   the desk: clean / needs a look / broken, and what changed compared to the previous run.
2. **Find out why.** *"It missed a shot / hit the wall / drifted. When, where, and what was the code thinking?"*
   Go from a problem straight to the exact moment, and see the pose, the odometry belief, motor powers, sensors,
   telemetry, logs and the current `decide()` move side by side with the field.
3. **Watch the run.** Play it back like a match video: smooth, scrubbable, 2D top view and 3D orbit.
4. **Change the conditions and try again.** Robot size, shooting physics, ball and Hive numbers, intake, how a shot
   leaves. About 20+ parameters today and more coming. Each has a range, a unit, a description, and is measured or
   unmeasured. You need to see what's changed from the defaults.
5. **Judge how far to trust the sim.** Which numbers are guesses, what the sim doesn't model ("assumed to work"),
   and how much they could matter.
6. **Pick which auto to run.** A few `@Autonomous` OpModes, each with an alliance and a start pose.

**Future jobs (design so they fit later, don't need full designs now):**
- Compare two or more runs (path overlay, a time/result diff), including a sim run against a real match log.
- Scenario editing: drag balls and Cells, set Hive states, partner behavior, vision noise.
- Calibration: fit unmeasured numbers from a real robot log, with before/after curves and accept/reject.

---

## Key moments (must be designed)

These are moments, not screens. Show how your structure handles each one.

| # | Moment | What must be true |
|---|--------|-------------------|
| M1 | **First open, nothing run yet** | Clear what this is and the one action to take. No empty-dashboard feeling. |
| M2 | **Save in IDE → rebuild → new run arrives** | Calm progress (Compile → Simulate → Analyse). The new run replaces the old one **without losing camera, scrub time or scroll position**. A "what changed vs. last run" summary. |
| M3 | **Verdict at a glance** | Clean run / Needs a look / Problems found / Run failed, readable from across the desk. Each check says what it measured and the threshold. One click takes you to the moment it's about. |
| M4 | **Investigating a moment** | Field + robot (true pose and odometry pose, the gap visible) + trail + planned path, next to the code's state at that time. Event markers on the timeline. |
| M5 | **Tuning parameters** | Many parameters stay scannable: grouped, searchable or filterable, changed ones obvious, a reset per item and for all, measured/unmeasured badges, a live preview where it helps (robot footprint). Range errors before you run. |
| M6 | **Trust review** | Unmeasured numbers and "assumed to work" items in one place, linked from wherever they affect a result. |
| M7 | **Failure** | Compile error (file:line, message), runtime exception (stack trace + jump to the moment), runner/physics disconnected (status + retry). |

## State Matrix (design all of these)

| State | Notes |
|-------|-------|
| Idle, no run yet | M1 |
| Compiling | Can take 5–30 s. Show progress without a modal blocking the last run. |
| Simulating / analysing | Short. Shouldn't flash. |
| Run ready | The normal state. |
| Run ready, but stale (code saved, rebuild pending) | The user must not mistake an old run for the new one. |
| Compile error | Keep the last good run visible but clearly old. |
| Runtime exception | A partial run. Can still replay up to the crash. |
| Did not finish (timeout) | |
| Runner or physics disconnected | |
| Run on save **off** | Manual Run only. Must be visible so nobody waits for a run that never comes. |
| Parameter out of range / invalid start pose | Caught before the run starts. |

---

## Domain: the field (draw it accurately)

- 144 × 144 in square (3658 mm), 24 in tiles (dashed grid).
- **Two Hives** in the middle, one red, one blue. Each is a pivoting body with an **up Cell** and a **down Cell**,
  held at ±30° by an over-center spring. Enough weight in the up Cell tips it and the balls spill.
- **Four Flowers** (A–D) in a pinwheel, 48 in from the corners: pipe cages with 4 pollen each.
- **Gardens:** 4 pollen in a line along the wall. **Loading Zones** against the wall.
- **Balls:** pollen (yellow) and nectar (red/blue). They can be in the robot, a Flower, a Garden, a Hive Cell,
  on the floor, or in the air. A height cue matters (a ball in the air vs. on the floor).
- **Coordinates:** red origin (0,0) is the top-left of the top view, +x right, +y down. Blue is mirrored. The
  origin marker and axes can be toggled.
- **Robot:** footprint rectangle (front/back/width from the center), heading, up to 4 balls held. Draw two poses:
  **TRUE** (physics) and **ODOMETRY** (what the code believes).
- Alliance red/blue appears **only** where the field has it. Pick the UI accent color so it never gets confused with an alliance.

## Data available (so designs are realistic)

**Per frame** (about 60 per sim second, about 1,800 per run): time, true pose (x, y, heading), odometry pose,
motor powers, servo positions, beam-break states, every ball's position and state, Hive Cell states (up / down / tipping),
telemetry key/values, log lines (with tags), the current `decide()` move.

**Per run:** finished?, error/stack trace, time used, warnings, events (shot, pickup, ball went in, Hive tip,
contact with wall/obstacle, decide() change, park, error), and the parameter values used and which ones changed.

**Derived (already computed, `web/stats.js`):** verdict; checks such as *Ran to the end*, *Setup is valid*,
*Touched nothing*, *Odometry tracks the robot* (mm, warn/fail thresholds), *Heading tracks the robot* (°),
*Stays on its own side*, *Uses the whole match* (idle tail), *Shots land in a Hive Cell* (n of m), *Tips a Hive*.
Each check has a status, a one-line detail and an optional `seek` time. Number tiles (balls picked up, shots,
tips, distance, top speed …), speed and odometry-error over time, balls held over time, the decide() timeline
(move, from, to), a contacts list, the unmeasured list and the "assumed to work" list.

**Parameters** (`sim/params.json`): `{ id, group, label, unit, min, max, step, description, type?: "choice", options?, target? }`.
Groups today: Robot size, Shooting, Balls, Hive, Intake. Descriptions often say "Unmeasured."

You may propose **new derived views** (e.g. a run-to-run diff, a "why did decide() pick this" explanation, a
heat map of where time was spent) if they serve a job. Mark anything that would need new data.

---

## Design principles (direction, not rules)

- **The field is the hero; the app is the instrument.** Quiet chrome, dense but calm data, thin lines, flat color.
- **Answer first, evidence one click away.** Verdict → check → moment → raw data. Every layer links down and back up.
- **Time is the main axis.** Everything that happened at a time (event, check, chart point, log line) should be able to put you at that time.
- **Make continuity felt.** Re-runs morph in place. No full reloads, no layout jumps. Motion only to explain change.
- **Make honesty visible, not noisy.** One consistent "unmeasured" treatment that you notice but that doesn't shout.
- **Make the difference the signal.** After the 50th run, the most useful thing is *what changed*, not the whole report again.

Explore freely: one-workspace vs. multi-view, a timeline-centric layout, a command palette, a "glance mode" for a
second monitor, split field/data, picture-in-picture field on the analysis view. Show at least **two distinct
structural directions** before you converge, and say which one you recommend.

---

## Review Focus (likely failure points; please show how the design handles each)

1. **Stale run confusion:** code saved, rebuild still running or failed. The user reads the old run as the new one.
2. **Parameter sprawl:** the list grows from 20 to 60+. Can you still find, see and reset what you changed?
3. **Side-by-side with the IDE:** the browser window is about 900 px wide or on a small laptop. Does the glance check still work?
4. **Two poses overlapping:** when true and odometry pose are almost the same (most of the time) and when they're far apart. Both must read clearly.
5. **Dense moments:** several events within 0.2 s (shot + pickup + contact). The timeline markers and the event list must not turn to mush.

---

## Reference: what exists today (not a template)

Built as four hash routes in `sim/app/web/` (`index.html`, `app.js`, `style.css`, `stats.js`, `statsView.js`, `scene3d.js`):

- **Landing** `#/`: a hero ("Test your auto before the robot does."), one card per OpMode, Play, "Open last results", Run on save toggle.
- **Setup** `#/setup`: collapsible parameter groups (slider + number box, per-item reset, changed count), a robot footprint
  preview with start-pose checks, Start simulation with a Compile / Simulate / Analyse step line, which then opens Stats.
- **Stats** `#/stats`: verdict card, checklist, nine tiles, speed and odometry-error charts (click → replay at that time),
  decide() timeline, contacts table, "how far to trust this run".
- **Replay** `#/replay`: full-bleed field canvas (2D/3D toggle), a header bar (tabs, OpMode select, Run, Run on save, status chip),
  a right side panel (Robot now, Balls, Warnings, Events, Telemetry, Not measured yet), a bottom transport (play, step, time,
  scrubber with colored event markers, 2D/3D, speed).

Known pain points: going from Stats to the moment and back loses context; the side panel is one long scroll; Setup and
results feel like separate apps; there's no "what changed vs. last run"; the stale state is weak.

The older feature-list brief is in `sim/design_specs.md`, for background only. Where it conflicts with this file, this file wins.
