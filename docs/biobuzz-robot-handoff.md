# BIOBUZZ robot code handoff

For: a Fable 5.2 orchestrator session in this repo. Owner: Ken (ask him live when the rules below say to).

## Goal
Get the BIOBUZZ (2026-27) robot code as close to working as possible before the robot is finished, so first power-on is tuning, not writing. Work by editing last year's (DECODE) modules in place. The simulator is OUT of scope this pass; a coworker wires it later from `teleop-to-sim-handoff.md`, which you write at the end.

## Hard rules (and why)
1. **Every new class copies an existing class's pattern.** Before writing a class, name the existing class it is modeled on (in the PR/commit notes). If nothing in the repo is a fair model, it is a new concept: stop and ask Ken. Why: the code must stay readable to the students who maintain it.
2. **Reuse, never re-implement.** Use `MathFunctions`, `Vector`, `Point`, `Position`, `KMotor`, `KServo`, `KCRServo`, `KActionSet`, `KServoAutoAction`, `WaitAction`, `OpModeUtilities`, `SharedData`, `KLog`, `KGamePad`, and the config classes (`ModuleConfig`, `OdometryConfig`, `ShooterConfig`, `TurretConfig`, `BeamBreakConfig`, `IntakeConfig`, `AprilTagConfig`, etc.). Search before writing any helper. Example: `BeamBreakPosition.IntakeSide {A, B}` already exists; do not create a second side enum.
3. **Edit in place, do not duplicate.** `Shooter` keeps its name and drops to one motor. `Turret` keeps its name and becomes a servo. Delete what no longer exists (`Tilter`, `DriveBrake` + brake actions, second shooter motor, turret motor/PID code, their tests and configs). Last year's code is safe on branch `decode-25`.
4. **Hardware layer owns safety, policy layer owns choices.** Interlocks live inside module methods so no caller can violate them. Behavior that may change later (which side intakes, feed order, timings) lives in one swappable place so a change is a few lines, not a rewrite. Do not encode situations as a growing enum of modes.
5. **No sim work**, but keep `TeamCode/src/test` compiling with the smallest possible edits so `./gradlew :TeamCode:testDebugUnitTest` stays green. Do not touch `sim/`.
6. **Do not touch others' work.** Master has uncommitted files from teammates (biobuzz autos, `AimPoint`, `ShooterGridConfig`, etc.) and `handoff.md` is a teammate's vision handoff. Work in a git worktree on a new branch `biobuzz-robot` off master. Copy this file into the branch.
7. **Placeholders are explicit.** Any value not measured (servo positions, ports, timings, ratios) gets `// TODO-measure` and goes on the checklist in `teleop-to-sim-handoff.md`.

## Commits: max 3, one per milestone
Each commit must compile and pass unit tests so reverting to it is safe.
1. **Hardware**: configs, hardware map names, modules, single-module actions, unit tests, this doc + a one-line pointer in `CLAUDE.md` ("Robot code plan: see docs/biobuzz-robot-handoff.md").
2. **Behavior**: intake controller, shoot action, TeleOp, per-subsystem test OpModes, shooter PIDF baselines + interpolation collector.
3. **Auto + handoff**: `AutoDecider` moves wired to real actions, `teleop-to-sim-handoff.md`.

## Agent roles
- **Fable 5.2 (you)**: plan, split work, keep a decision log, ask Ken. Do not write large code yourself.
- **Sonnet 5.5** (`Agent`, model sonnet): codebase search. For each new piece, report which existing class to copy and which helpers to reuse.
- **Opus 5.5** (model opus): implement one subsystem at a time. A different Opus agent that did not write the code reviews it against the hard rules (pattern source named, no duplicate helpers, interlocks in hardware layer, placeholders marked) before each milestone commit.

## Robot facts
- Axes: odometry side is the **front**; +X straight ahead, +Y to the right. Intake side A = front (+X), B = back (confirm against `BeamBreakPosition` comments, keep consistent).
- Drivetrain: 4 mecanum motors, OctoQuad + 3 goBILDA odometry pods (unchanged). No brake servo.
- Ports: 7 motors (4 drive, 2 intake, 1 shooter), 6 servos (2 intake pivots, stopper, hood, turret, flower).
- **Intake** (double sided, independent): each side has its own motor and its own pivot servo. That motor drives the side's intake wheels AND its 2 transfer wheel sets. Spring loaded to ground. Balls stack from the outside in toward the center, then up to the turret.
- **Stopper**: servo that holds balls just before the shooter. Opening it while feeding shoots. Running intake against a closed stopper is OK briefly; too long stalls the motor.
- **Shooter**: one 6000 RPM goBILDA motor, chain to 6x 72 mm flywheels (3 per side) with 4 goBILDA flywheel weights. Deflector design, no counter-roller. `KMotor.getRPS()` is motor-shaft RPS.
- **Hood**: servo, about 30 to 40 degrees of travel.
- **Turret**: one servo (new model, same pattern as `Stopper`), no encoder. Can mechanically turn about 360 degrees; keep it simple: aim from odometry, map heading to servo position, clamp to range, no wrap logic.
- **Turret gearing (from CAD)**: servo output 50T -> 15T pinion (confirmed by Ken) on a shaft with 37T -> 92T turret ring. Two external meshes, so the turret turns the same direction as the servo. `turretDeg = servoDeg * (50 / 15) * (37 / 92)` = servoDeg * 1.3406. A 360 deg turret needs ~268.5 deg of servo travel; a 255 deg servo gives ~342 deg of turret travel, so clamp to that if the servo is 255. Put `SERVO_GEAR_T = 50`, `PINION_T = 15`, `TOP_GEAR_T = 37`, `RING_T = 92`, `SERVO_RANGE_DEG` (TODO-measure, servo model) and the derived ratio in `TurretConfig`, following its existing `TICKS_PER_DEGREE` / `BIG_TO_SMALL_PULLEY` style. `Turret.setAngleRad/Deg()` converts: `servoPos = zeroPos + dir * turretDeg / ratio / SERVO_RANGE_DEG`, clamped to the reachable turret range (log when the target is out of range); `getAngleRad()` returns the last commanded angle (no encoder). `KServo` already stores `rangeDegrees`; reuse it, do not add a second range field. Unit test the round trip deg -> pos -> deg.
- **Limelight 3A** on the turret: re-zero odometry and goal status only (existing `ResetOdometryToLimelight` / `LimelightAprilTagCamera`). Ball camera (Arducam) is out of scope here.
- **Flower mech**: one servo arm on one intake side (which side is undecided: make it one config constant). Pivots down in auto or teleop; driving into a Flower knocks its balls into the intake.
- **Ball sensing**: beam-break array only, about 6 sensors, count and positions still changing. `BeamBreakArray` already handles a double-sided array with a plausibility pass; make it generic to any count/positions from config instead of rewriting it. No color sensors.
- Capacity: 4 balls.

## Architecture (agreed)
| Layer | Class | Pattern source | Owns |
|---|---|---|---|
| Hardware | `IntakeSide` (2 instances, A and B) | `Stopper` / `Tilter` | motor, pivot servo, `lower()`, `raise()`, `run(power)`, `stop()`, `isDown()`. If the flower is on this side, `raise()` raises the flower first |
| Hardware | `Flower` | `Stopper` | one servo; refuses to go down unless its side `isDown()` |
| Hardware | `Intake` | existing module | both sides + beam-break state; `getSide(IntakeSide)`, `getSideFacing(direction)` |
| Policy | `IntakeController` interface | `DriveController` | decides which side runs each loop |
| Policy | `DriveDirectionIntakeController` | `FieldOrientedDrive` | robot-frame X velocity (deadband + hysteresis) picks the side: that side lowers and runs, the other side raises; at capacity both stop and raise |
| Action | shoot `KActionSet` | `PushBall` / `ShootAllAction` | default: (intakes already stopped at 4 balls) wait RPS ready + turret aligned, run feed briefly, open stopper, feed side A, wait, feed side B, close stopper. Order = `setDependentActions` lines, timings in `IntakeConfig` |

Two instances of `IntakeSide` is deliberate: the hardware is identical, so duplicated behavior would be the bug. Side-specific behavior lives in the controller. Changing "other side raises" must be about one line; a new strategy (driver assist, ball-aware) is a new controller class plus one line in TeleOp.

**Flower interlock (critical):** the flower may be down only while its intake is down. If that intake raises for any reason while the flower is down, that is a fault. Enforce it inside `IntakeSide.raise()` and `Flower.lower()`, and unit test it.

## Shooter specifics
- **Ball types**: POLLEN 2.8 in, NECTAR 3.6 in, no way to tell them apart yet. Shoot all at the same RPS/hood per distance. Problems seen: exit power differs by type, and NECTAR shots sag robot voltage. Only 4x POLLEN is somewhat consistent today.
- **Voltage compensation already exists but is commented out** in `KMotor.goToRPS` (`DEFAULT_VOLTAGE / voltage`). Re-enable it behind a `ShooterConfig` flag rather than writing new code; voltage is read in `KOpMode` into `SharedData`.
- **PIDF baselines** in `ShooterConfig` (one motor now): start from last year's `kp = 0.085, ki = 0, kd = 0, kf = 0.01148`. Mark kf `TODO-measure` and make the existing `MotorPowerRPSDataCollector` the way to measure it (kf = steady power / steady RPS at ~0.8 power, full battery). Expect a slightly higher kf and slower recovery than last year because one motor now spins 6 wheels + 4 weights.
- **Chain is 1:1 (confirmed by Ken)**, so flywheel RPS = motor RPS and `KMotor.getRPS()` needs no conversion. Ceiling is about 1/kf, roughly 87 RPS at 12 V loaded (free speed ~100). Keep shooting setpoints at or below ~70 RPS so the PIDF has headroom to recover between balls and under NECTAR voltage sag. If far shots need more than that, tell Ken: it is a gearing change, not a code fix. Clamp `MAX_RPS` in config accordingly (TODO-measure).
- **Interpolation**: replace `ShooterInterpolationConfig.shooterData` with an empty table (guard the lookup so an empty or one-row table returns a safe default instead of crashing). Keep `IShooterPredictor` / `ShooterInterpolationDataLookup`. Revive the commented-out `test/shooter/ShooterDataCollector` as the collector: dashboard-tunable RPS and hood, shows odometry distance to target, a button logs distance/RPS/hood/hit-or-miss to CSV via `ShotLogger` / `KFileWriter`. Rows go straight into `shooterData`.
- Add a zeroed per-ball-type RPS/hood offset in config, unused until ball typing exists. Optional hardware idea to mention to Ken, do not build: one beam break at the stopper between 2.8 and 3.6 in high would detect NECTAR only.

## Logging (every subsystem)
Why: on the real robot, logcat grep is the fastest way to see what each subsystem did and why. Every module, action, controller and test OpMode logs through `KLog` only (no `Log.d`, no `System.out`, no telemetry-only debugging).
- **Tags are `Group_Thing`** so one grep gets a layer or a whole flow: `Robot_<Module>` for hardware modules (`Robot_Intake`, `Robot_Flower`, `Robot_Stopper`, `Robot_Shooter`, `Robot_Hood`, `Robot_Turret`, `Robot_BeamBreak`); `Shooting_<Action>` for the shot flow (`Shooting_ShootAction`, `Shooting_ShooterReady`, `Shooting_PushBall`); `Intaking_<Thing>` (`Intaking_IntakeController`, `Intaking_RunIntake`); `Vision_<Thing>` (`Vision_Limelight`, `Vision_OdoReset`); `Auto_<Thing>` (`Auto_Decider`, `Auto_Path`). `grep Robot_` = all hardware, `grep Shooting_` = one full shot end to end, `grep Robot_Turret` = one module. Keep tag strings as constants next to each class.
- **Log state changes and decisions, not every loop.** E.g. side switched A->B and why (velocity, deadband), pivot lowered/raised, flower blocked by interlock, stopper open/close, shot fired (RPS dip), action start/done/timeout, beam-break count changed, turret target clamped. Per-loop numbers (RPS, servo targets) only behind the existing `KLog.isDebug()` gate.
- **Use the `Supplier<String>` overload** (`KLog.d(tag, () -> ...)`) for anything with string building, so disabled logs cost nothing in the loop, as `Shooter` already does.
- **Same line format inside a tag**: `key=value` pairs (e.g. `side=A event=lower reason=driveDir vx=312`) so logs can be grepped and parsed into CSV without regex gymnastics.
- **Faults use `KLog.e`** (interlock violation, stall timeout, sensor unreliable per `BeamBreakArray.isCountReliable()`, out-of-range turret target).
- List every tag and its key events in `teleop-to-sim-handoff.md` so the sim and humans use the same vocabulary.

## Ask Ken live (do not decide alone)
- Feed order and timings inside the shoot action (run intake into a closed stopper first, or start feeding when it opens).
- Intake side-switching details (deadband, what happens when stopped, driver override buttons).
- Which side the flower is on, gamepad mapping (start from last year's `decode/TeleOp`).
- Servo models and ranges, hardware map names if unsure.

## Build order
1. Configs and hardware map names (placeholders marked).
2. Modules: `IntakeSide`, `Intake`, `Flower`, `Stopper` (keep), `Shooter` (one motor, voltage flag), `Turret` (servo), hood, generic beam breaks. Delete obsolete modules.
3. Small actions per module (lower, raise, run, stop, open/close, aim, spin up), modeled on existing actions.
4. `IntakeController` + `DriveDirectionIntakeController`.
5. Shoot `KActionSet`.
6. TeleOp rewired to the new modules.
7. One test OpMode per subsystem (`test/` has the patterns) so each part is a 5-minute check on the real robot.
8. Unit tests for pure logic: flower interlock, side selection, beam-break counting with N sensors, turret heading-to-servo mapping, empty interpolation table.
9. Shooter PIDF baselines + interpolation collector.
10. Wire `AutoDecider` moves (GET_FLOWER_*, intake, shoot) in the `biobuzz/` autos to the real actions.
11. Write `teleop-to-sim-handoff.md` at repo root: every changed/added/deleted module and action, the hardware map names, what the sim's fake hardware in `TeamCode/src/test/.../sim/` must change, and the full "measure on robot" checklist.

## Done means
Compiles; unit tests pass; each milestone reviewed by a separate Opus agent; no duplicate classes or helpers; every class names its pattern source; every subsystem logs through KLog with its subsystem tag; every placeholder is on the checklist; at most 3 commits on `biobuzz-robot`, not merged (Ken merges).

## Decision log
Decisions made during execution (owner answers and orchestrator rulings).

**Overrides of this handoff (owner instructions)**
- No `biobuzz-robot` branch and no worktree: milestone commits go on local `master`; the owner pushes once at the end.
- Top-level `sim/` is not touched. Sim Java under `src/test` gets compile-only edits; sim-run tests that need new fake hardware are `@Ignore`d and listed in `teleop-to-sim-handoff.md`. `SimHardware` is unchanged.

**Owner answers ("Ask Ken live")**
- `stash@{0}` is left as is. Work starts from `f4deb18c`. 23 untracked files that only compiled against the stash were moved to `~/biobuzz-untracked-backup-2026-11-15/` (same relative paths).
- Old DECODE autos under `decode/auto/` are kept, marked `@Disabled`, and edited only enough to compile. They are the layout to copy. Tuning OpModes are ported to the new robot; ones not worth porting are deleted.
- Shoot feed order: no pre-feed. Open the stopper, feed side A, wait (configurable), feed side B, close the stopper.
- Inside the drive deadband, the last selected side stays down and running. Side A is the default before any motion.
- Intake overrides: gamepad 2 right trigger = stop, right bumper = reverse. Nothing else.
- Flower is on side A (front). Gamepad 2 B toggles it.
- Turret servo travel: 355 deg placeholder (`TODO-measure`). The turret target clamps to ±180 deg.
- Device names: motors `shooter`, `intakeA`, `intakeB`; servos `intakePivotA`, `intakePivotB`, `turret`, `flower`. `stopper`, `hood`, drive, `limelight`, `octoquad` are unchanged.

**Orchestrator rulings (milestone 1)**
- `Turret` keeps its `getInstance` / `setInstanceNull` shell so about 20 callers do not change. `getAngleRad()` returns the last commanded angle; `isMoving()` is the `KServo` travel-time estimate.
- `TurretConfig.TRIM_DEG` is a static Dashboard value, so it persists across OpModes; autos zero it at start.
- Interlocks live in the hardware layer: `Flower.lower()` refuses unless its side `isDown()`; `IntakeSide.raise()` raises the flower first and holds the pivot until the flower is up. A pending raise completes only when `update()` or `raise()` is called again, so the milestone 2 controller must call `intake.update()` every loop.
- Each side's pivot is commanded raised at init (the side is spring loaded down and would exceed the starting size).
- `ShooterInterpolationDataLookup` no longer shares a static list (each `new Shooter` used to append another copy) and returns a safe default for empty and one-row tables.
- `KMotor.goToRPS` voltage compensation is back on, behind `ShooterConfig.USE_VOLTAGE_COMPENSATION`.
- `RunIntake` stall handling was removed: `STALL_POWER` equalled the only power callers used, so it did nothing.
- The hardware class `IntakeSide` shares its name with the enum `BeamBreakPosition.IntakeSide`; the enum is written qualified where both appear.
- Beam breaks: `BeamBreakConfig.SENSORS` is an ordered list (A end to B end) of device name, side and threshold; 6 placeholder sensors, capacity 4, all `TODO-measure`. `BeamBreakPosition` is now a value class (the nested `IntakeSide {A, B}` enum is kept). Each blocked sensor counts as one ball, capped at capacity. This is a placeholder: once sensor spacing is measured against both ball diameters (POLLEN 2.8 in, NECTAR 3.6 in), revisit it (one NECTAR may block two beams).
- Review, milestone 1: the turret is not wrapped at ±180 deg, because this handoff says "no wrap logic". A goal directly behind the robot can make it swing a full turn; `Turret.isMoving()` tracks its own busy-until time so a shot does not fire mid-swing.
- Review, milestone 1: until the milestone 2 `IntakeController` exists, the intake-running actions lower both sides when they start, so live OpModes do not spin rollers in the air.
- Moved to milestone 2: `IntakeConfig` `@Config` with per-side feed time, wait between sides and side-switch thresholds (they have no caller until the controller and the shoot flow exist). The deferred review minors are listed in `teleop-to-sim-handoff.md`.
