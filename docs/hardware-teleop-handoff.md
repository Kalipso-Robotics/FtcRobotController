# BIOBUZZ hardware + TeleOp handoff

For: the teammate wiring every subsystem and test OpMode into `TeleOp` and a new `BasicTeleop`. Read this first, then `teleop-to-sim-handoff.md` (what changed, measure checklist) and `docs/biobuzz-robot-handoff.md` (design decisions). Code root: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/kalipsorobotics/`.

## Goal
1. Every subsystem passes its test OpMode on the real robot.
2. `TeleOp` (and `TeleOpBlue`) drives every subsystem.
3. A `BasicTeleop`: the fallback with no assists, for first power-on and for when odometry or the Limelight is off.

Do not change module architecture or interlocks. If something needs a design change, ask Ken.

## Robot frame
Odometry side is the **front**. +X straight ahead, +Y to the right, heading positive clockwise seen from above. Intake side **A = front**, **B = back**.

## Hardware systems
| Subsystem | Hardware map names | Class | Config | Test OpMode | What to know |
|---|---|---|---|---|---|
| Drivetrain | `fLeft` `fRight` `bLeft` `bRight` | `modules/DriveTrain`, `actions/drivetrain/FieldOrientedDriveAction` | `DrivetrainConfig` | `utilities/HardwareTest` (`test/TestWiring` has no `@TeleOp`, so it is not on the Driver Station) | Mecanum, left stick drive, right stick turn. No brake servo. |
| Odometry | `octoquad` (I2C) | `localization/OctoQuadOdo`, `Odometry` | `localization/OctoConfig`, `OdometryConfig` | `test/octoquad/OctoStartup`, `OctoTune`, `CompareOdometryTest` | 3 goBILDA pods: port 0 right parallel, port 1 perpendicular, port 2 left parallel (heading check only). TCP offsets and track width are from CAD (2026-11-16) and need an `OctoTune` spin check. Never change OctoConfig values by hand without the tool that owns them. |
| Intake (x2) | `intakeA` `intakeB` (motors), `intakePivotA` `intakePivotB` (servos) | `modules/intake/Intake` (holds both), `IntakeSide` | `ModuleConfig` (pivot positions, motor directions), `actions/intake/IntakeConfig` | `test/IntakeMotorTest` | Each side's motor also drives that side's 2 transfer wheel sets. Spring-loaded to the ground. `Intake.update()` must run every loop (it finishes deferred raises). |
| Intake policy | (none) | `actions/intake/IntakeController`, `DriveDirectionIntakeController` | `IntakeConfig.SIDE_SWITCH_VX_MM_PER_S` | (unit tests) | The side facing the drive direction lowers and runs, the other raises; at capacity both stop and raise. To change the behavior, change or swap the controller, not TeleOp. |
| Flower arm | `flower` (servo) | `modules/intake/Flower` | `ModuleConfig.FLOWER_SIDE`, `FLOWER_UP/DOWN_POS` | `test/IntakeMotorTest` (B = toggle) | One arm on side A. **Interlock:** it may only be down while its intake is down, and raising that intake raises the flower first. This lives in `Flower`/`IntakeSide`; never bypass it from an OpMode. |
| Beam breaks | `beamBreak0` .. `beamBreak5` (analog), A end to B end | `modules/beambreak/BeamBreakArray` | `BeamBreakConfig.SENSORS`, thresholds, `BLOCKED_IS_LOW_VOLTAGE` | `test/BeamBreakTest` | Count and positions are placeholders. One blocked beam = one ball, capped at capacity (4). Check `isCountReliable()` before trusting the count. |
| Stopper | `stopper` (servo) | `modules/Stopper` | `ModuleConfig.STOPPER_SERVO_OPEN/CLOSED_POS` | `decode/calibrations/StopperCalibration` | Holds balls before the shooter. Running intake into a closed stopper is OK briefly; too long stalls the motor. |
| Shooter | `shooter` (motor, encoder) | `modules/shooter/Shooter`, `actions/shooter/ShooterRun`, `ShooterReady` | `ShooterConfig` (PIDF, `PASS_RPS`, timeouts), `ShooterInterpolationConfig` (table, `MAX_RPS` 70) | `test/shooter/MotorPowerRPSDataCollector` (kf), `ShooterDataCollector` (table rows), `ShooterEncoderTest` | One 6000 RPM motor, 1:1 chain to 6x 72 mm flywheels + 4 weights. Voltage compensation is behind a `ShooterConfig` flag. The shot table starts empty. |
| Hood | `hood` (servo) | inside `Shooter` | `ShooterInterpolationConfig.MIN/MAX_HOOD` | `test/HoodServoToZero` | About 30 to 40 degrees of travel. |
| Turret | `turret` (servo) | `modules/Turret`, `actions/turret/TurretAutoAlignTeleOp` | `TurretConfig` (gear ratio 60/20 * 37/92 = 1.2065, `SERVO_RANGE_DEG`, `SERVO_ZERO_POS`, `SERVO_DIRECTION`) | `test/turret/TurretServoTest` | Axon MAX, 60T > 20T, same shaft 37T > 92T ring (two meshes, turret turns the same way as the servo; PWM set to 500-2500 us). No encoder: `getAngleRad()` is the last commanded angle. Aims from odometry, clamps out-of-range targets (logged). |
| Limelight 3A | `limelight` | `vision/apriltag/LimelightAprilTagCamera`, `localization/ResetOdometryToLimelight` | `AprilTagConfig` | `test/cameraVision/LimeLightTest` | On the turret. Only re-zeroes odometry and reads goal state. Pipelines must be set on the device (36h11 IDs 30-45). |
| Shooting flow | (none) | `actions/shooter/ShootAllAction`, `pusher/PushBall` | `IntakeConfig.FEED_A_MS`, `WAIT_BETWEEN_SIDES_MS`, `FEED_B_MS` | (TeleOp) | Stop intakes, open stopper, feed A, wait, feed B, close (~950 ms). Order lives in `setDependentActions` lines. |

Ports used: 7 motors (4 drive, 2 intake, 1 shooter), 6 servos (2 pivots, flower, stopper, hood, turret).

## Codebase practices (follow these exactly)
- **Copy an existing pattern.** Every new class is modeled on one that already exists; name it in a comment (e.g. `Pattern source: StopperCalibration`). No fair model in the repo means ask Ken.
- **OpModes extend `KOpMode`.** Use its `initializeRobot()`, `updateActions()`, `setLast*Action()` and `isPending()` instead of hand-rolled loops. `decode/calibrations/StopperCalibration` is the skeleton the test OpModes copy.
- **Modules** take `OpModeUtilities` and wrap hardware in `KMotor` / `KServo`. OpModes call module or action methods, never `hardwareMap` directly (the old `HardwareTest` is the exception).
- **Actions** extend `Action` / `KActionSet`; chain steps with `setDependentActions`. Servo moves use `KServoAutoAction`, waits use `WaitAction`. A new behavior is a new action, not a block of `if`s in TeleOp.
- **Configs hold every number.** Positions, timings, powers and thresholds go in config classes (`ModuleConfig`, `ShooterConfig`, `ShooterInterpolationConfig`, `TurretConfig`, `IntakeConfig`, `BeamBreakConfig`, `AprilTagConfig`, `DrivetrainConfig`, `OctoConfig`, `OdometryConfig`). Values to tune live from FTC Dashboard are non-final `static` in an `@Config` class. Unmeasured values carry `// TODO-measure`. No magic numbers in OpModes.
- **Reuse helpers** (`MathFunctions`, `Vector`, `Point`, `Position`, `SharedData`, `KGamePad`). Search before writing one. Use `KGamePad` `is...FirstPressed()` for toggles so a held button fires once.
- **Logging:** only `KLog`, tag `Group_Thing`: `Robot_<Module>`, `Shooting_<Action>`, `Intaking_<Thing>`, `Vision_<Thing>`, `Auto_<Thing>`, `Test_<Subsystem>`. Log events (state changes, decisions, faults) as `key=value`, not every loop; per-loop numbers only behind `KLog.isDebug()`; use the `KLog.d(tag, () -> ...)` form when building strings; faults with `KLog.e`. Tag table: `teleop-to-sim-handoff.md` section 4.
- **Git:** small milestone commits that each compile, on a branch; don't commit teammates' untracked files.

## Current TeleOp controls (`decode/TeleOp`, `TeleOpBlue` extends it)
- **GP1:** left stick drive, right stick turn (field-oriented), RB = slow. X = flower assist (nearest Flower; any stick input cancels). Y = pass (fixed `PASS_RPS`/`PASS_HOOD`). LB = hive shot (fires only once RPS and turret are ready; aborts after `TELEOP_READY_TIMEOUT_MS`). LT = stop + flywheel off.
- **GP2:** RT = intake stop, RB = intake reverse, B = flower toggle. Left stick button = Limelight re-zero, right stick button = corner-pose zero (`FieldConfig.CORNER_ZERO_POSE`). Dpad = turret trim, LB + dpad = hood trim.
- Intake runs from `DriveDirectionIntakeController` by default.
- Structure: `initializeRobot()` builds modules; the loop calls `handleDriving / handleTurret / handleIntake / handleFlower / handleStopper / handleShooting / handleZeroing`. Shooting owns the intake while it runs.

## BasicTeleop (proposed, confirm with Ken)
Purpose: drive and score with no odometry-dependent help, so the robot is usable before odometry, the Limelight or the shot table are tuned.
- **Don't copy TeleOp's 600 lines.** Preferred: make TeleOp's `handle*` methods `protected` and add a small hook (e.g. an `assistsEnabled()` method) so `BasicTeleop extends TeleOp` the same way `TeleOpBlue` does. If that gets messy, ask Ken before duplicating.
- **Keep:** driving (robot-centric option is fine), intake per side by hand (lower/run/raise per side) with drive-direction auto intake as a toggle, flower toggle (interlock still applies), stopper, `PushBall` feed.
- **Shooter:** fixed presets from config (e.g. near / far RPS + hood in `ShooterConfig`), plus manual RPS and hood trim. No interpolation table.
- **Turret:** manual angle with dpad and a "center" button; no odometry aim, no SOTM, no Limelight.
- **Drop:** flower assist, pass point, zeroing, ready-gating on turret alignment (keep the RPS-ready gate).
- New preset constants go in `ShooterConfig` / `TurretConfig` with `// TODO-measure`.

## Bring-up order on the robot
Each step assumes the previous one passed. Log with `adb logcat | grep Robot_` (or the subsystem tag).
1. `HardwareTest`: every drive motor reads and spins the right way.
2. `OctoStartup` (signs, mirror), then `OctoTune` (counts/mm, heading, the CAD offsets + track width). Record results with dates in `OctoConfig` per its rules.
3. `IntakeMotorTest`: each side lowers, raises, runs both ways; flower toggles only when side A is down. Set pivot/flower positions and motor directions in `ModuleConfig`.
4. `StopperCalibration`: open/closed positions.
5. `HoodServoToZero`: hood zero and range.
6. `TurretServoTest`: zero, direction, range (`TurretConfig`).
7. `BeamBreakTest`: thresholds, order, which beams one POLLEN vs one NECTAR blocks.
8. `MotorPowerRPSDataCollector`: kf. Then `ShooterDataCollector`: fill the shot table.
9. `LimeLightTest`: tag detection after pipelines are configured.
10. `BasicTeleop`, then full `TeleOp`.

Full list of placeholders: `grep -rn TODO-measure TeamCode/src/main` and `teleop-to-sim-handoff.md` section 5.

## Ask Ken
- BasicTeleop control map and whether it extends TeleOp.
- Anything that would change an interlock, the intake controller policy, or the shoot sequence.
- Beam break count/spacing once measured (one NECTAR may block two beams).
