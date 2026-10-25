# Auto simulator

Runs the team's real `@Autonomous` code on a laptop, on a virtual clock, against a simulated robot.

- Java side: `TeamCode/src/test/java/.../kalipsorobotics/sim/` (clock, fake FTC hardware, runner).
- Physics: `sim/physics/` (Rapier, in a Node process the Java runner starts).
- UI brief: `sim/design_specs.md`.

## One-time setup
Node 22 lives in `sim/.node/` (not committed). On a new machine:
download the macOS/Linux build from https://nodejs.org/dist/ (check it against `SHASUMS256.txt`),
unpack it into `sim/.node/` so that `sim/.node/bin/node` exists, then:

    cd sim/physics && PATH="$PWD/../.node/bin:$PATH" npm install

## The app
    sim/app/start.sh        then open http://localhost:5173

Four views, one address each: **Landing** (`#/`) picks the auto and has the **Play** button. Play opens **Setup** (`#/setup`),
where you change parameters (robot size for now) and press Start. The run goes Compile, Simulate, Analyse and opens **Stats**
(`#/stats`), the verdict and everything behind it. **Replay** (`#/replay`) shows the run on the field.

### Adding a parameter to the Setup screen
1. Describe it in `sim/params.json` (id, group, label, unit, min, max, step, description).
2. Add its id to `defaults()` and `set()` in `TeamCode/src/test/.../sim/SimParams.java`.
That is all. The default comes from the code, the control appears by itself, the server checks the range, the value is passed to the run as
`-Dsim.param.<id>`, and the Stats page lists what you changed. The robot size is applied to `RobotFootprint`, which both the physics and
your path planner read.
It builds your Java, runs the auto in its own JVM with the Rapier physics, and shows the result.
Save a `.java` file under `TeamCode/src` and it runs again by itself (switch "Run on save" off to stop that).
Keys: Space play/pause, arrows step (Shift = 10 frames), `[` `]` speed, `P` panel, `R` run again.
`?t=12` opens the page 12 s into the match; `?play=1` starts playing; `?panel=0` hides the panel.
Opmodes are listed in `sim/opmodes.json`, with the start pose each one gives its Odometry.

## Tests
Physics tests:   `cd sim/physics && PATH="$PWD/../.node/bin:$PATH" node --test`
App analysis tests: `cd sim/app && PATH="$PWD/../.node/bin:$PATH" node --test stats.test.mjs`
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
- **Two ways a shot leaves** (a choice on the Setup screen): from the flywheel and hood your code sets, or auto-aimed into the nearest up Cell.
- **Every number** (ball mass, bounce, Hive tip weight, shot speed per RPS, hood angles, exit height, intake reach, ...) is in
  `sim/physics/ball-defaults.json` with a note on where it comes from or why it is unmeasured, and is a parameter on the Setup screen
  (`sim/params.json`, entries with `"target": "physics"`). To add one: put it in `ball-defaults.json` and `params.json`. No Java change.
- **Anything the sim does not model** returns a harmless value and is listed on the Stats page under "Assumed to work".
- **View:** the Replay has a 2D / 3D toggle (key `V`). In 3D, drag to orbit and use "Reset view".
