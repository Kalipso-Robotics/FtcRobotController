# BIOBUZZ robot code → sim handoff

What changed in the robot code for the BIOBUZZ robot (milestones 1–3), what the sim must change to run it again, and what still has to be measured on the robot. Plan and decision log: `docs/biobuzz-robot-handoff.md`.

Commits: M1 `62eb4339` (hardware), M2 `2b02c22f` (behavior), M3 (auto + this doc). Base before M1: `f4deb18c`.

## 1. Modules and actions

### New
| Class | What | Pattern source |
|---|---|---|
| `modules/intake/IntakeSide` | One intake side: motor + pivot servo. Owns the flower-raise interlock (raise defers until the flower is up, finishes on `update()`). | `Stopper` |
| `modules/intake/Flower` | Flower arm servo on `ModuleConfig.FLOWER_SIDE` (A). Interlock: `lower()` refuses unless its side `isDown()`. | `Stopper` |
| `actions/intake/IntakeController` + `DriveDirectionIntakeController` | Policy: run the side facing the drive direction (deadband, keep-last); full → both stop + raise. | `DriveController` / `FieldOrientedDriveAction` |
| `test/turret/TurretServoTest` | Hand-driven turret servo check. | `StopperCalibration` |

### Changed (edited in place)
- `Intake`: now holds sides A/B + flower; `run/lower/stop/update` for both, `getSide`, `getSideFacing`.
- `Shooter`: one motor (`shooter`). `Turret`: one servo, odometry aim, clamped to range, no encoder. `Stopper`: injection ctor.
- `BeamBreakArray` / `BeamBreakPosition` / `BeamBreakConfig.SENSORS`: ordered generic list (A end → B end), one blocked beam = one ball, capped at capacity.
- `PushBall`: intake stop → open stopper → feed A → wait → feed B → close (~950 ms/shot). `ShootAllAction`: no pre-feed.
- `IntakeFullAction`: new per-side ctor (`IntakeSide`, does not lower). `RunIntake`/`RunIntakeTime(both)` lower both sides; per-side `RunIntakeTime` stays raised.
- `TeleOp`: intake runs from `DriveDirectionIntakeController`; GP2 RT = stop, GP2 RB = reverse, GP2 B = flower toggle.
- `PathPlanning.planPickup(robot, heading, ball, Boolean forceBack)`: force one intake end (null = best of both, old behavior).
- `AutoToBallsPathingAction`: runs the side the planner chose (other side stops + raises); `useFlower` forces the flower's side (falls back to either side without the arm if that side cannot reach, `event=flowerFallback`) and lowers the flower once that side is down and within `INTAKE_START_DISTANCE_MM` of the stand-off; after arrival keeps the side running `POST_ARRIVAL_INTAKE_MS`, then stops it and raises the flower. A drop (`setIsDone(true)`) stops at once.
- `RedDecideAuto`: Flower piles call the pickup with `useFlower`; sweep uses `DriveDirectionIntakeController`; `intake.update()` every loop; `stowIntake()` (stop, flower up, both sides up) on park start, dropped move, sweep end and auto end.
- `MotorPowerRPSDataCollector` (one motor, shows kf, voltage compensated), `ShooterDataCollector` (CSV `distance, rps, hood, // hit|miss` via `KFileWriter`).
- Decode-era autos under `decode/auto/*`: compile-only edits (`@Disabled`).

### Deleted
`Tilter`, `DriveBrake`, brake/tilt actions (`ActivateBraking`, `ReleaseBrakeAction`, `ReleaseBraking`, `TiltUp`, `TiltDown`), `TilterCalibration`, `BrakeServoTest`, `IntakeMode`, `RunIntakeUntilFullSpeed`, `TurretMove`, turret motor/PID tools (`TurretTest`, `TurretAutoTest`, `TurretDataCollector`, `TurretKsTuner`, `TurretPIDDataCollector`), second shooter motor, `IntakeConfig.shootTimeMS`. Last year's code: branch `decode-25`.

## 2. Hardware map names
| Device | Name |
|---|---|
| Drive motors | `fLeft`, `fRight`, `bLeft`, `bRight` (unchanged) |
| Intake motors | `intakeA`, `intakeB` |
| Intake pivots (servo) | `intakePivotA`, `intakePivotB` |
| Flower (servo) | `flower` |
| Shooter motor | `shooter` |
| Hood / stopper / turret (servos) | `hood`, `stopper`, `turret` |
| Beam breaks (analog) | `beamBreak0` … `beamBreak5` (A end → B end, placeholders) |

## 3. What the sim must change
`SimHardware` (`TeamCode/src/test/.../sim/SimHardware.java`) still builds last year's map. To run the BIOBUZZ code:
- Motors: replace `shooter1`/`shooter2` with one `shooter`; replace `intake` with `intakeA` + `intakeB`; `turret` becomes a servo (`SimServo`), not a motor.
- Servos: add `intakePivotA`, `intakePivotB`, `flower`; drop `tilterLeft`/`tilterRight`.
- Beam breaks: replace `beamBreakAOuter/AInner/BInner/BOuter` with `beamBreak0..5` in `BeamBreakConfig.SENSORS` order.
- World actuators: `intake` → two per-side actuators (only the lowered, running side picks up); `turretRad` from the turret servo position via `TurretConfig` ratio; `flywheelRps` from the single `shooter`; a flower actuator (arm down + driving into a Flower knocks balls off).
- `@Ignore`d until then: `RedTippingAutoSimTest`, `BallShowcaseSimTest`. No unit test runs `RedDecideAuto`, so the M3 auto changes are compile-verified only.
- `sim/opmodes.json` runs `RedDecideAuto` and its subclasses through `SimMain`; those runs are broken since M1 until `SimHardware` gets the new map.
- Top-level `sim/` was not touched.

## 4. Log tags
Every subsystem logs through `KLog` with a subsystem tag; new and changed lines are `key=value`.

| Prefix | Tags | Key events |
|---|---|---|
| `Robot_` | `Robot_Intake`, `Robot_Flower`, `Robot_Shooter`, `Robot_Hood`, `Robot_Stopper`, `Robot_Turret`, `Robot_BeamBreak`, `Robot_KMotor` | `event=lower/raise/raiseDeferred/raiseCancelled/run`, `event=blocked reason=intakeUp` (flower), turret out-of-range clamp |
| `Shooting_` | `Shooting_PushBall`, `Shooting_ShootAction`, `Shooting_ShooterReady`, `Shooting_ShooterRun`, `Shooting_TurretAlign`, `Shooting_TurretReady` | `event=ready/waiting`, feed step progress |
| `Intaking_` | `Intaking_IntakeController`, `Intaking_Flower` (TeleOp toggle), `Intaking_IntakeFullAction`, `Intaking_RunIntake`, `Intaking_RunIntakeTime`, `Intaking_IntakeStop`, `Intaking_IntakeReverse` | `event=switch from= to= vx= reason=driveDir`, `event=full`, `event=start/done side=` |
| `Vision_` | `Vision_OdoReset` | odometry re-zero from Limelight |
| `Auto_` | `Auto_Decider`, `Auto_Path` | `event=dropMove reason=`, `event=stowIntake reason=`, `event=plan ... useFlower=`, `event=intakeOn side=`, `event=flowerLower/flowerRaise`, `event=failed reason=` |
| `Test_` | `Test_Intake`, `Test_Shooter`, `Test_Turret` | test OpModes |

## 5. Measure on the robot (`grep -rn TODO-measure TeamCode/src/main`)
- `ModuleConfig`: `INTAKE_PIVOT_A/B_DOWN_POS`, `INTAKE_PIVOT_A/B_UP_POS`, `INTAKE_MOTOR_A/B_DIRECTION`, `FLOWER_DOWN_POS`, `FLOWER_UP_POS`.
- `IntakeSide` / `Flower`: pivot and flower servo model, speed, range.
- `IntakeConfig`: `FEED_A_MS` (400), `WAIT_BETWEEN_SIDES_MS` (150), `FEED_B_MS` (400), `SIDE_SWITCH_VX_MM_PER_S` (100).
- `TurretConfig`: `SERVO_RANGE_DEG`, `SERVO_ZERO_POS`, `SERVO_DIRECTION`; `Turret` servo model/speed.
- `ShooterConfig.kf` (run `MotorPowerRPSDataCollector`); `Shooter` motor direction.
- `ShooterInterpolationConfig`: `MAX_RPS` (70), `DEFAULT_RPS` (40); shooter table from `ShooterDataCollector`.
- `BeamBreakConfig`: device names, sides, count, order, thresholds; `BLOCKED_IS_LOW_VOLTAGE`.
- `AutoToBallsPathingAction`: `INTAKE_START_DISTANCE_MM` (600), `POST_ARRIVAL_INTAKE_MS` (500), `INTAKE_RUN_MS` (3000).
- `FieldConfig.FLOWER_BALL_OFFSET_MM` (150): pickup target offset from a Flower's center (moved from `RedDecideAuto`); likely smaller since the arm knocks balls off by driving in. Untracked `DecideParams` duplicates this value: whoever lands that file should read `FieldConfig` instead.
- `AutoToBallsPathingAction.FLOWER_DEPLOY_CLEARANCE_MM` (350): arm deploys only this far or more from the Flower; must stay below `INTAKE_START_DISTANCE_MM` (600) (KLog.e at init if not).
- `FieldConfig`: `PASS_X` / `PASS_Y` (1524 / 600) pass target; `CORNER_ZERO_POSE` (230, 230, 0) robot pose when parked in the corner for the GP2 zero.
- `ShooterConfig`: `PASS_RPS` (25), `PASS_HOOD` (0.2) for the GP1 pass; `TELEOP_READY_TIMEOUT_MS` (3000), after which a TeleOp shot aborts without feeding.
- `AprilTagConfig`: `CELL_A_TAG_POSES` / `CELL_B_TAG_POSES` (null = unmeasured, tag left out of the layout, Limelight re-zero stays inert until filled). The tags face down on the tipping Cells, so a 2D wall-tag chain may not suffice (see section 6).

## 6. Ken to confirm
- Feeding with both pivots raised, including the preload shot (transfer wheels share the side motor).
- `RedDecideAuto` time budgets in sim with the ~950 ms `PushBall` (was 500 ms).
- Beam break spacing vs ball size: one NECTAR (3.6 in) may block two beams; the count is a placeholder until measured.
- Pickup keeps the side running `POST_ARRIVAL_INTAKE_MS` after arrival, then stops it (M3). Tune so a pull-in pickup finishes taking the ball in.
- In auto the pivots stay as the pickup left them while driving to shoot (not raised at shoot start).

- Limelight pipelines (index 0/1 per alliance, `LimelightAprilTagCamera`) must be reconfigured on the device for 36h11 IDs 30-45 (manual 9.9). Device config, not code.
- Cell tags face down: a real fix likely needs the camera tilted up plus MegaTag with a `.fmap`, or a 3D chain. Separate on-robot task.

## 6b. TeleOp controls (GP1 / GP2)
- GP1: X = flower assist (nearest Flower, full pickup; any stick input cancels), Y = pass (weak fixed RPS/hood at the far-half pass point), LB = hive shot (press edge only, gated on RPS + turret ready, never fires on timeout), LT = stop and flywheel off, RB = slow.
- GP2: left stick button = Limelight re-zero (never snaps to (0,0,0)); right stick button = corner-pose zero (`CORNER_ZERO_POSE`). LB + dpad now trims only the hood; the turret trim is dpad alone.
- TeleOp Blue exists (`TeleOpBlue`).

## 7. Known small items (not fixed)
- `DriveDirectionIntakeController` reads odometry velocity with `peek*`; a read can tear across odometry-thread updates (EMA + threshold keep the effect small).
- `DriveDirectionIntakeControllerTest` builds `BeamBreakData` by reflection (package-private ctor).
- `ShooterDataCollector` has no drive code; `intake.lower()` runs in init; GP2 RT overrides feeds.
- Untracked teammate work on master (`biobuzz/DecideParams.java`, `PathFile.java`, `src/main/resources/autoparams/`, `sim/`) is not part of these commits.
