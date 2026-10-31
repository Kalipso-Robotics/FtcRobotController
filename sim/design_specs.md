# Auto Simulator: UI design specs

Hand this file to the UI-design agent, together with the design references.

## What this app is
A browser app (Chrome, `http://localhost:5173`) that shows an FTC autonomous run of the BIOBUZZ game.
The robot code runs in a separate Java program on the laptop. The browser does NOT run code. It only draws
what the runner sends: robot pose, motor powers, sensor values, telemetry, log lines, decide() moves.

Users: FTC team programmers. They edit Java in Android Studio, save, and expect this page to show the new run
by itself. They look at it many times per hour, so it must be fast to read and calm.

## Goal of the UI
1. Show clearly what the auto did, on an accurate field.
2. Let the user go back to any moment (scrub) and see why the robot did it.
3. Be honest. Anything estimated, unmeasured or unverified is visibly marked. Nothing is dressed up.

## Principles
- Minimalistic: thin lines, flat colors, no textures. Alliance colors (red / blue) only where the field has them.
- Smooth: animations and transitions feel buttery (60 fps robot motion, eased panel changes, no layout jumps).
- Data first: the field is the hero. Panels stay quiet until needed.
- Light and dark themes. Works from laptop size up. Keyboard friendly (space = play/pause, arrows = step).
- 2D top view now. The scene must be able to grow to 3D later without a redesign.

## Field (drawn in the main view)
- 144 x 144 in square, 24 in tile grid (dashed lines).
- Two Hives in the middle: one red, one blue. Each has an up Cell and a down Cell. Cells can be up / down / tipping.
- Four Flowers (A, B, C, D) in a pinwheel, 48 in from corners. Garden strip in a corner. Loading Zone boxes against the wall.
- Balls: pollen and nectar, drawn as small circles.
- Red origin (0,0) = top-left of the top view, +x right, +y down. Blue origin = bottom-right, +x left, +y up.
  Show a small origin marker and axes, toggleable.
- Values that are not verified on the real field get a dashed outline and a tooltip "unverified".

## Screens

### 1. Launcher
Pick what to run.
- List of discovered `@Autonomous` OpModes (name, file).
- Alliance, partner mode (SOLO_NOTHING / SOLO_PRELOAD / NOT_SOLO), start pose.
- Big Run button. Shows the last run time and result.

### 2. Simulation view (the main screen)
- Field with the robot (footprint rectangle + heading arrow), the path it drove (trail), and the planned path.
- Two robot markers: TRUE pose (from physics) and ODOMETRY pose (what the code believes). The gap between them is visible.
- Match clock (AUTO 30 s) and controls: play, pause, step, speed (0.25x to 8x).
- Timeline scrubber with event markers: decide() move, shot, pickup, tip, park, errors.
- Side panel (collapsible): telemetry, log lines (filter by tag), selected-moment details (motor powers, sensors, decide() inputs and decision).

### 3. Scenario editor
- Drag balls and Cells on the field. Set Hive state, partner behavior, vision noise and miss rate.
- Save, load and duplicate scenarios.

### 4. Run compare
- Overlay two or more runs (or a run against a real match log). Show differences in path, time and result.
- Per-run color, toggle visibility, a table of key numbers.

### 5. Robot calibration
- Drive model numbers (max speed, acceleration, wheel slip, loop time) with a measured / unmeasured badge each.
- "Fit from log" flow: choose a real odometry log, see before / after curves, accept or reject.

### 6. System states (design these too)
- Compiling (progress, calm).
- Compile error (file, line, message, clickable).
- Runtime exception in the OpMode (stack trace, jump to the moment it happened).
- Runner or physics disconnected (clear status, retry).
- Reload transition: the new run replaces the old one without losing the camera position or scrub time.

## Data the UI receives (so the designs are realistic)
Per frame, about 60 per second of sim time: time, true pose, odometry pose, motor powers, servo positions,
beam-break states, ball positions, Hive Cell states, telemetry key / values, log lines, current decide() move.
Per run: result summary (points, time used, errors), list of events.

## Out of scope for this design pass
3D view, camera image rendering, multi-user sharing, accounts.


## Built so far (reference for the design pass)

- **Landing** (`#/`): hero, one card per OpMode, a **Play** button.
- **Setup** (`#/setup`): opened by Play. Parameter groups with a slider and a number box each (robot front, back and width for now), a "Reset" link on changed ones, a live top-view preview of the robot with the default size dashed behind it, live start-pose checks, and Start simulation, which shows a Compile / Simulate / Analyse progress line and then opens Stats by itself.
- **Stats** (`#/stats`): verdict card (Clean run / Needs a look / Problems found / Run failed), a checklist (each check says what it measured
  and the threshold), nine number tiles, speed and odometry-error charts (hover for values, click to open the replay at that moment),
  a decide() timeline, a contacts table, and a "how far to trust this run" list of unmeasured numbers. Everything is computed from the
  recorded frames, events and telemetry (`sim/app/web/stats.js`, tested in `sim/app/stats.test.mjs`).
- **Replay** (`#/replay`): the field view described above.
- Not built yet: Scenario editor, Run compare, Robot calibration. More Setup parameters will be added later (loop times, wheel response, mass, ...).

### Balls (added)
- **Replay, 2D:** balls as discs (yellow pollen, red and blue nectar) with a height cue (a high ball looks larger and its shadow slides away); the two Cells of each Hive drawn from above, the lifted one stronger.
- **Replay, 3D:** orbit camera. Flowers (pipe cages), the Hive frame, Cells that pivot, balls as spheres, the robot as a box, trails on the floor.
- **Panel:** a Balls section (in the robot, in a Flower, in a Garden, in a Hive Cell, on the floor, in the air); scrubber markers colored by event (shot, pick up, went in, Hive tip).
- **Stats:** tiles for balls picked up, shots fired, Hive tips; checks "Shots land in a Hive Cell" and "Tips a Hive"; a "Balls held" chart; the "Assumed to work" list.
- **Setup:** collapsible groups (Robot size, Shooting, Balls, Hive, Intake) with a count of changed values, and a choice control for how a shot leaves.
