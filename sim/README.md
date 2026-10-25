# Auto simulator

Runs the team's real `@Autonomous` code on a laptop, on a virtual clock, against a simulated robot.

- Java side: `TeamCode/src/test/java/.../kalipsorobotics/sim/` (clock, fake FTC hardware, runner).
- Physics: `sim/physics/` (Rapier, in a Node process the Java runner starts).
- UI brief: `sim/design_spec.md`.

## One-time setup
Node 22 lives in `sim/.node/` (not committed). On a new machine:
download the macOS/Linux build from https://nodejs.org/dist/ (check it against `SHASUMS256.txt`),
unpack it into `sim/.node/` so that `sim/.node/bin/node` exists, then:

    cd sim/physics && PATH="$PWD/../.node/bin:$PATH" npm install

## The app
    sim/app/start.sh        then open http://localhost:5173

Three views, one address each: **Home** (`#/`) picks the auto (and an optional partner path) and has the **Run** button. **Workspace**
(`#/run`) is the run itself: the field (2D or 3D), a summary rail with the verdict, checks and "changed since the last run", a details
drawer, and the lane timeline (EVENTS, DECIDE, PARTNER, SCORE). **Parameters** (`#/params`) changes sim values and shows which ones are
guesses (`~`) or read from your TeamCode (`</>`). Old `#/stats`, `#/replay` and `#/setup` links still redirect.

It builds your Java, runs the auto in its own JVM with the Rapier physics, and shows the result. Save a `.java` file under `TeamCode/src`
and it runs again by itself (switch "Run on save" off to stop that).

Keys (the `?` button in the timeline lists them): `R` run, Space play/pause, arrows step (Shift = 10 frames), `[` `]` speed, `V` 2D/3D,
`F` follow the robot (3D), `D` details drawer, `N` next problem, `/` parameters search, `Esc` back.
Deep links: `#/run?t=12` opens 12 s into the match; add `play=1` to start playing, `mode=2d` for the top view, `details=1` for the drawer.
Opmodes are listed in `sim/opmodes.json`, with the start pose each one gives its Odometry.

### Adding a parameter to the Parameters screen
1. Describe it in `sim/params.json` (id, group, label, unit, min, max, step, description; `unmeasured`, `affects`, `source` for the marks).
2. Add its id to `defaults()` and `set()` in `TeamCode/src/test/.../sim/SimParams.java`.
That is all. The default comes from the code, the control appears by itself, the server checks the range, the value is passed to the run as
`-Dsim.param.<id>`, and the page lists what you changed. The robot size is applied to `RobotFootprint`, which both the physics and
your path planner read.

### Partner robot R2
R2 replays a path file, never shoots, but is solid: R1 bumps into it and it pushes balls. Pick one on Home (or in Parameters, with a speed of
25 to 150 %). Path files live in `sim/paths/*.json`:

    { "name": "red_partner_far", "robot": { "front": 200, "back": 200, "width": 400 },
      "poses": [[0, 250, 1400, 0], [12, 250, 800, 0], [24.5, 250, 500, 0]] }

Each pose is `[seconds from START, x mm, y mm, heading in degrees]`, in the same field frame as `opmodes.json`; times must go up. R2 holds its
first pose until START and its last pose after the end. The runner checks R2's start the same way as R1's (inside the wall, inside an obstacle)
and says so in the run's warnings. A touch between R1 and R2 is the event `robot touched partner` and fails the "Avoids partner" check;
closer than 150 mm is a warning. The run records `#partnerinfo` and `#partner t x y theta` lines next to the frames.
`red_partner_far.json` stays clear of both `RedTippingAuto` and `BallShowcaseAuto` (closest about 9 in).

## Tests
Physics tests:   `cd sim/physics && PATH="$PWD/../.node/bin:$PATH" node --test`
App analysis tests: `cd sim/app && PATH="$PWD/../.node/bin:$PATH" node --test`
Sim tests (Java): `JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:testDebugUnitTest --tests "*kalipsorobotics.sim.*"`

Every sim run is its own JVM (the robot code keeps static state). Run output goes to `TeamCode/build/sim-runs/`.
Numbers marked UNMEASURED in `SimWorld`/`KinematicWorld`/`SimRunner`/`RapierWorld` are guesses until fitted from a real robot log.

## Balls, the Hive and shots
The physics is Rapier **3D** (`sim/physics/world.mjs`). It places the balls where the manual does (Figure 10-2): 4 pollen in each Flower
(stacked in its pipe cage), 4 in each Garden (a line along the wall), 3 nectar in each up Cell, plus the robot's 4 preloaded pollen. Each Hive
is a body on a pivot with two Cells, held at +-30 degrees by an over-center spring: enough weight in the up Cell swings it over and the balls
spill out. The intake picks balls up from either end (never more than 4, the beam breaks read them), and a shot leaves when the stopper is open,
the intake is pushing and the flywheel is up to speed.

- **Showcase auto:** `BallShowcaseAuto` (sim-only, `TeamCode/src/test/.../simdemo/`) drives your real `RoundTripAction`, `Shooter`, `Intake`,
  `Stopper`, `Turret` and `TurretAutoAlign`. RedTippingAuto has all of that commented out, so it never touches a ball.
- **Two ways a shot leaves** (a choice on the Parameters screen): from the flywheel and hood your code sets, or auto-aimed into the nearest up Cell.
- **Every number** (ball mass, bounce, Hive tip weight, shot speed per RPS, hood angles, exit height, intake reach, ...) is in
  `sim/physics/ball-defaults.json` with a note on where it comes from or why it is unmeasured, and is a parameter on the Parameters screen
  (`sim/params.json`, entries with `"target": "physics"`). To add one: put it in `ball-defaults.json` and `params.json`. No Java change.
- **Anything the sim does not model** returns a harmless value and is listed under "How far to trust this" in the summary rail.
- **View:** the Workspace has a 2D / 3D toggle (key `V`). In 3D, drag to orbit and use "Reset view".
