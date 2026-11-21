package org.firstinspires.ftc.teamcode.kalipsorobotics.decode;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.*;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.AutoToBallsPathingAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak.BeamBreakCountAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeReverse;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.DriveDirectionIntakeController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Flower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = " TeleOp")
public class TeleOp extends KOpMode {
    // Robot-centric is the default drive mode. Flip to true to drive field-oriented instead.
    private static final boolean USE_FIELD_ORIENTED_DRIVE = false;

    private boolean hasClosedStopperInnit = false;
    private DriveTrain driveTrain;

    private Shooter shooter = null;
    private Intake intake = null;
    private Turret turret = null;
    private Stopper stopper = null;
    private SOTM sotm = null;

    ShootAllAction shootAllAction = null;


    AutoToBallsPathingAction flowerAssist = null; // GP1 X: drive to the nearest Flower and pick it up
    BeamBreakCountAction beamBreaks = null;
    ShootAllAction passAction = null; // GP1 Y: weak fixed-RPS shot at the far half


    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;

    IntakeController intakeController = null;
    private boolean flowerWanted = false;
    IntakeStop intakeStop = null;
    IntakeReverse intakeReverse = null;

    DriveController driveAction = null;

    ResetOdometryToLimelight resetToLimelight = null;
    ResetOdometryToPos resetOdometryToPos = null;

    // Button state variables
    private boolean drivingSticksActive = false;
    private boolean slowDrivePressed = false;
    private boolean shootAllActionPressed = false;
    private boolean intakeStopPressed = false;
    private boolean intakeReversePressed = false;
    private boolean flowerTogglePressed = false;
    private boolean stopShooterPressed = false;
    private boolean zeroLimelightPressed = false;
    private boolean zeroCornerPressed = false;
    private boolean releaseStopperPressed = false;
    private boolean toggleTurretAlign = false;
    private boolean incrementRPSPressed;
    private boolean decrementRPSPressed;
    private boolean incrementHoodPressed;
    private boolean decrementHoodPressed;
    private boolean kGamepad2IsDpadLeftFirstPressed;
    private boolean kGamepad2IsDpadRightFirstPressed;
    private boolean setTurretOffset0Pressed;
    private boolean flowerAssistPressed;
    private boolean passButtonPressed;
    private final boolean enableLimelightZeroing = true;

    private Point hiveTarget = null; // the Hive Cell we aim at, see updateHiveTarget()
    private int shootCount = 0;
    private double lastLoopTime = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor); // TeleOpBlue overrides this, so switching back to RED resets it
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        KLog.d("TeleOp_Init", "Starting initializeRobot()");
        // ============== MODULES =================
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        turret = Turret.getInstance(opModeUtilities);
        intake = new Intake(opModeUtilities);
        intakeController = new DriveDirectionIntakeController(intake);
        beamBreaks = new BeamBreakCountAction(opModeUtilities); // live ball count: capacity stop and Balls telemetry
        AutoToBallsPathingAction.checkFlowerClearance();
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        // =========================================


        // ============== LONG RUNNING ACTIONS =================

        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null);
//        odometry.setShouldFallbackToWheelTheta(true);
        aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, allianceColor);

        OpModeUtilities.runAprilTagExecutorService(aprilTagExecutorService, aprilTagDetectionAction);
        sotm = new SOTM(opModeUtilities);
        if (SOTMConfig.enabled) OpModeUtilities.runSOTMExecutorService(sotmExecutorService, sotm);
        driveAction = USE_FIELD_ORIENTED_DRIVE
                ? new FieldOrientedDriveAction(driveTrain, imuModule)
                : new DriveAction(driveTrain);
        // Both aim at the Hive Cell on the robot's end of the Hive; updateHiveTarget() keeps them on it every loop.
        hiveTarget = SharedData.getTargetPoint();
        shooterRun = new ShooterRun(opModeUtilities, shooter, hiveTarget);
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret);
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.STOP); // handleTurret() turns it on with the align toggle
        turretAutoAlignTeleOp.setTargetPoint(hiveTarget);
        SharedData.setSOTMActiveGoal(hiveTarget == TARGET_POINT_1 ? 0 : 1);

        // =========================================

        KLog.d("TeleOp_Init", "Creating intake, shooter, stopper, turret modules");
        KLog.d("TeleOp_Init", () -> "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));

        KLog.d("TeleOp_Init", () -> "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        KLog.d("TeleOp_Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("teleop", "--------------TELEOP STARTED-------------");
        KLog.d("TeleOp_Run", () -> "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_OPEN_POS);
        waitForStart();
        telemetry.setMsTransmissionInterval(200);
        sleep(50);
        //Wait for Executor Thread to start

        KLog.d("TeleOp_Run", () -> "After waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));

        turretAutoAlignTeleOp.setToleranceDeg(2);

        if (stopper == null) {
            KLog.e("TeleOp_Run", "ERROR: Stopper is NULL when trying to create closeStopper action!");
        } else {
            KLog.d("TeleOp_Run", "Creating closeStopper action - stopper is valid");
        }

        KLog.d("TeleOp_Run", "closeStopper created successfully");

        while (opModeIsActive()) {
            // Dashboard may flip SOTM on mid-match; the thread exits itself when it's turned off.
            if (SOTMConfig.enabled && !sotm.isRunning()) OpModeUtilities.runSOTMExecutorService(sotmExecutorService, sotm);
            opModeUtilities.clearBulkCache();
            lastLoopTime = loopTimer.milliseconds();
            loopTimer.reset();
            if (KLog.isDebug()) {
                KLog.d("Loop_Speed", () -> (int) lastLoopTime + "ms");
            }
            beamBreaks.updateCheckDone();
            if (!hasClosedStopperInnit) {
                stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
                hasClosedStopperInnit = true;
            }
            // ========== READ ALL INPUTS (makes it clear what buttons do) ==========
            drivingSticksActive = kGamePad1.isAnyStickActive();

            flowerAssistPressed = kGamePad1.isButtonXFirstPressed();
            passButtonPressed = kGamePad1.isButtonYFirstPressed();

            slowDrivePressed = kGamePad1.isRightBumperPressed();

            shootAllActionPressed = kGamePad1.isLeftBumperFirstPressed();
            stopShooterPressed = kGamePad1.isLeftTriggerFirstPressed();

            //============Driver 2============
            boolean kGamepad2IsDpadUpFirstPressed = kGamePad2.isDpadUpFirstPressed();
            boolean kGamepad2IsDpadDownFirstPressed = kGamePad2.isDpadDownFirstPressed();

            kGamepad2IsDpadLeftFirstPressed = kGamePad2.isDpadLeftFirstPressed();
            kGamepad2IsDpadRightFirstPressed = kGamePad2.isDpadRightFirstPressed();

            setTurretOffset0Pressed = kGamePad2.isAPressed();

            incrementRPSPressed = kGamepad2IsDpadUpFirstPressed;
            decrementRPSPressed = kGamepad2IsDpadDownFirstPressed;

            incrementHoodPressed = kGamepad2IsDpadRightFirstPressed && kGamePad2.isLeftBumperPressed();
            decrementHoodPressed = kGamepad2IsDpadLeftFirstPressed && kGamePad2.isLeftBumperPressed();


            zeroLimelightPressed = kGamePad2.isLeftStickButtonFirstPressed();
            zeroCornerPressed = kGamePad2.isRightStickButtonFirstPressed();

            intakeStopPressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();

            flowerTogglePressed = kGamePad2.isButtonBFirstPressed();
            releaseStopperPressed = kGamePad2.isYPressed();

            toggleTurretAlign = !kGamePad2.isToggleX();
            KLog.d("TeleOp_Button", () -> "toggleTurretAlign: " + toggleTurretAlign);



            // ========== HANDLE DRIVING ==========
            handleDriving();

            // ========== HANDLE ZEROING (LIMELIGHT & CORNER) ==========
            handleZeroing();

            // ========== HANDLE TURRET ==========
            updateHiveTarget();
            handleTurret();
            // ========== HANDLE INTAKE (Priority Order: Shoot > Stall > Manual > Idle) ==========
            handleIntake();
            handleFlower();

            // ========== HANDLE STOPPER ==========
            handleStopper();

            // ========== HANDLE SHOOTING ==========
            handleShooting();

            // ========== UPDATE ACTIONS ==========
            updateActions();
            telemetry.addData("Odometry: ", SharedData.getOdometryWheelIMUPosition().toCompactString());
            telemetry.addData("Shot: ", shootCount);
            telemetry.addData("READY", shooterRun.isWithinRange() && turretAutoAlignTeleOp.isWithinRange());
            telemetry.addData("Intake side", intakeController.getSelectedSide());
            telemetry.addData("Balls", intake.getBallCount());
            telemetry.addData("Flower wanted/down", flowerWanted + "/" + intake.getFlower().isDown());
            telemetry.addData("Target Rps ", "%.2f Target Hood %.2f",shooterRun.getTargetRPS(), shooterRun.getTargetHoodPosition());
            telemetry.addData("Current RPS ", "%.2f", shooterRun.getCurrentRPS());
            telemetry.addData("Hive Cell ", hiveTarget == TARGET_POINT_1 ? "A" : "B");
            telemetry.addData("Up CELL (seen) ", "%s (%.1fs ago)%s", SharedData.getHiveUpCell(), SharedData.getHiveUpCellAgeMs() / 1000,
                    isOurCellDown() ? " -> LB PASSES" : "");
            telemetry.addData("Distance ", "%.0f", shooterRun.getDistanceMM());
            telemetry.addData("Rps Offset ", "%.2f", ShooterInterpolationConfig.rpsOffset);
            telemetry.addData("Hood Offset ", "%.2f", ShooterInterpolationConfig.hoodOffset);
            telemetry.addData("Turret Offset Deg ", "%.1f", TurretConfig.TURRET_OFFSET_DEG);
            telemetry.addLine("Turret Mode: " + turretAutoAlignTeleOp.getTurretRunMode());
            telemetry.addData("Delta Heading (Turret)", turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Delta RPS (Shooter)", shooterRun.getTargetRPS() - shooterRun.getCurrentRPS());
            telemetry.update();
        }
        cleanupRobot();
    }
    private void handleDriving() {

        if (flowerAssist != null && !isPending(flowerAssist)) {
            endFlowerAssist(); // finished or failed by itself
        }
        if (flowerAssistPressed && !isPending(flowerAssist) && !shotOrPassPending()) {
            Point flower = FieldConfig.nearestFlower(new Position(SharedData.getOdometryWheelIMUPosition()).toPoint());
            flowerAssist = new AutoToBallsPathingAction(driveTrain, stopper, intake, FieldConfig.flowerPilePoint(flower), false, flower);
            flowerAssist.setName("FlowerAssist");
            setLastMoveAction(flowerAssist);
            KLog.d("TeleOp_Flower", () -> "event=assist flower=" + flower);
        }

        if (slowDrivePressed) {
            driveAction.setPowerCoefficient(0.3);
        } else {
            driveAction.setPowerCoefficient(1);
        }

        if (drivingSticksActive) {
            driveAction.move(gamepad1);
            if (flowerAssist != null) {
                flowerAssist.setIsDone(true); // any stick input cancels the assist (raises the arm, stops the intake)
                endFlowerAssist();
            }
            setLastMoveAction(null);
        } else if (!isPending(flowerAssist)) {
            driveTrain.setPower(0);
        }
    }

    private void endFlowerAssist() {
        flowerAssist = null;
        flowerWanted = false;
        setLastMoveAction(null);
        KLog.d("TeleOp_Flower", "event=assistEnd");
    }
    private boolean shotOrPassPending() {
        return isPending(shootAllAction) || isPending(passAction);
    }

    private void handleTurret() {
        if (shotOrPassPending()) {
            return;
        }

        // Auto Align
        if (toggleTurretAlign) {
            KLog.d("TeleOp", "Running with only odometry");
            turnOnTurret();
        } else {
            KLog.d("TeleOp", "Stopping turret");
            turretAutoAlignTeleOp.stop();
        }


        //Manual (LB + dpad is the hood, not the turret offset)
        boolean turretOffsetAllowed = !kGamePad2.isLeftBumperPressed();
        if (turretOffsetAllowed && kGamepad2IsDpadLeftFirstPressed) {
            TurretConfig.TURRET_OFFSET_DEG -= 2;
        } else if (turretOffsetAllowed && kGamepad2IsDpadRightFirstPressed) {
            TurretConfig.TURRET_OFFSET_DEG += 2;
        }

        if (setTurretOffset0Pressed) {
            TurretConfig.TURRET_OFFSET_DEG = 0;
        }


        if (KLog.isDebug()) {
            KLog.d("TeleOp_Turret", () -> "angleDeg=" + turret.getAngleDeg() + " offsetDeg=" + TurretConfig.TURRET_OFFSET_DEG);
        }
    }

    /**
     * Aim at the Hive Cell on the robot's end of the Hive (A at high y, B at low y). Same numbers for both
     * alliances: odometry is in our alliance's own frame (FieldConfig). ShooterRun recomputes RPS and hood
     * from the distance to this point on every update.
     */
    private void updateHiveTarget() {
        if (isPending(passAction)) {
            return; // the turret is on the pass point until the pass ends
        }
        Point next = SharedData.getTargetPoint();
        if (next == hiveTarget) {
            return;
        }
        hiveTarget = next;
        turretAutoAlignTeleOp.setTargetPoint(hiveTarget);
        shooterRun.setTargetPoint(hiveTarget);
        SharedData.setSOTMActiveGoal(hiveTarget == TARGET_POINT_1 ? 0 : 1);
        KLog.d("TeleOp_Target", () -> "event=cellSwitch cell=" + (hiveTarget == TARGET_POINT_1 ? "A" : "B")
                + " target=" + hiveTarget + " odometry=" + SharedData.getOdometryWheelIMUPosition());
    }

    private void turnOnTurret() {
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
    }

    /**
     * Handle intake with explicit priority:
     * 1. SHOOTING (highest) - if shoot action is running, it controls intake
     * 2. STALLING - run until stall (for 3-ball)
     * 3. MANUAL - forward/reverse
     * 4. IDLE - stop intake when nothing pressed
     */
    private void handleIntake() {
        // PRIORITY 1: If shooting, don't interfere (shoot action controls intake)
        if (shotOrPassPending() || shootAllActionPressed || isPending(flowerAssist)) {
            return;  // Exit early - shoot or the flower assist has control
        }

        // PRIORITY 4: Manual reverse
        if (intakeReversePressed) {
            if (!isPending(intakeReverse)) {
                intakeReverse = new IntakeReverse(intake);
                setLastIntakeAction(intakeReverse);
            }
            return;  // Exit early
        }

        if (intakeStopPressed) {
            // PRIORITY 5: Stop when nothing pending
            if (!isPending(intakeStop)) {
                intakeStop = new IntakeStop(intake);
                setLastIntakeAction(intakeStop);
            }
            return;
        }
        // PRIORITY 3: drive-direction intake policy
        intakeController.update();
    }

    /** GP2 B toggles driver intent; the interlock lifts the flower when side A raises, and it re-lowers when A is down. */
    private void handleFlower() {
        if (isPending(flowerAssist)) {
            return; // the assist owns the arm
        }
        Flower flower = intake.getFlower();
        if (flowerTogglePressed) {
            flowerWanted = !flowerWanted;
            KLog.d("Intaking_Flower", () -> "event=toggle wanted=" + flowerWanted);
        }
        if (flowerWanted && !flower.isDown()) {
            flower.lower();
        } else if (!flowerWanted && !flower.isUp()) {
            flower.raise();
        }
    }

    /**
     * Handle stopper with clear priority
     */
    private void handleStopper() {
        // PRIORITY 1: If shooting, don't interfere (shoot action controls intake)
        if (shotOrPassPending() || shootAllActionPressed || passButtonPressed) {
            return;  // Exit early - shoot has control
        }

        // Open on button press
        if (releaseStopperPressed) {
            if (!isPending(openStopper)) {
                openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
                openStopper.setName("openStopper");
                closeStopper = null;
                setLastStopperAction(openStopper);
            }
            return;
        }

        // Not shooting and not Y (both returned above): keep the stopper closed
        if (!isPending(closeStopper)) {
            KLog.d("TeleOp_Stopper", "Close Stopper");
            closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
            closeStopper.setName("closeStopper");
            openStopper = null;
            setLastStopperAction(closeStopper);
        }
    }

    /**
     * Handle shooting sequence
     */
    private void handleShooting() {


        if (incrementRPSPressed) {
            ShooterInterpolationConfig.rpsOffset += 0.1;
            KLog.d("TeleOp_Shooting_RPS_offset", () -> "Increment Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        } else if (decrementRPSPressed) {
            ShooterInterpolationConfig.rpsOffset -= 0.1;
            KLog.d("TeleOp_Shooting_RPS_offset", () -> "Decrement Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        }

        double hoodPosition;

        if (incrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset += 0.02;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() + 0.02);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting_Hood_Offset", () -> "Increment Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
        } else if (decrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset -= 0.02;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() - 0.02);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting_Hood_offset", () -> "Decrement Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
        }

        // Priority 1- Stop shooter
        if (stopShooterPressed) {
            if (shootAllAction != null) {
                shootAllAction.setIsDone(true);
            }
            shootAllAction = null;
            if (passAction != null) {
                passAction.setIsDone(true);
                endPass(); // before stop(): endPass sets SHOOT_USING_CURRENT_POINT, which would spin the flywheel back up
            }
            shooterRun.stop();
            setLastShooterAction(null);

            KLog.d("TeleOp_Shooting", "Shooter stop");
            return;
        }

        // Pass finished by itself (fed or aborted): hand the shooter back to the hive.
        if (passAction != null && !isPending(passAction)) {
            endPass();
        }

        //Shooting Running, or the flower assist owns the intake/stopper --> return;
        if (shotOrPassPending() || isPending(flowerAssist)) {
            //KLog.d("TeleOp_Shooting", "shootAllAction Pending -> Return");
            return;
        }

        // Priority 2- Execute shoot sequence. Only the up CELL scores: if the CELL on our end was last seen down,
        // LB passes under the Hive instead of shooting at it.
        if (shootAllActionPressed && isOurCellDown()) {
            startPass("ourCellDown");
        } else if (shootAllActionPressed) {
            if (!isPending(shootAllAction)) {
                if (toggleTurretAlign) {
                    turnOnTurret();
                }

                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlignTeleOp, true);
                shootAllAction.setEnableLimelightZeroing(enableLimelightZeroing);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                shootCount++;
                shootAllAction.setName("Shot_" + shootCount);
                KLog.d("Teleop_Shooting", () -> "Shot_" + shootCount + " -" +
                        " Delta RPS: " + (shooter.getRPS() - shootAllAction.getShooterRun().getTargetRPS())  +
                        " Distance: " + shooterRun.getDistanceMM() +
                        " Turret Delta Angle: " + turretAutoAlignTeleOp.getDeltaAngleDeg() +
                        " Odometry: " + SharedData.getOdometryWheelIMUPosition() +
                        " Limelight Pos: " + SharedData.getLimelightGlobalPosition()
                );
            }
        }

        // Pass (not while a shot or pass is pending, see return above)
        if (passButtonPressed && !shotOrPassPending()) {
            startPass("button");
        }
    }

    /**
     * True if vision last saw the CELL on the robot's end of the Hive down (SharedData.getTargetPoint picks that end).
     * ponytail: trusts the last sighting (a TIP seen from anywhere updates it); a TIP nobody saw leaves it stale until
     * the camera sees a CELL again. Telemetry shows the age; add a freshness limit if that bites.
     */
    private boolean isOurCellDown() {
        HiveTagLocalizer.Cell up = SharedData.getHiveUpCell();
        HiveTagLocalizer.Cell ours = hiveTarget == TARGET_POINT_1 ? HiveTagLocalizer.Cell.A : HiveTagLocalizer.Cell.B;
        return up != null && up != ours;
    }

    /** Weak fixed RPS/hood shot under the Hive to the far half (FieldConfig.passTarget). */
    private void startPass(String reason) {
        Point target = FieldConfig.passTarget(SharedData.getOdometryWheelIMUPosition().toPoint());
        turretAutoAlignTeleOp.setTargetPoint(target);
        if (toggleTurretAlign) {
            turnOnTurret();
        }
        passAction = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlignTeleOp, PASS_RPS, PASS_HOOD, true);
        passAction.setName("Pass_" + shootCount);
        setLastShooterAction(passAction);
        setLastStopperAction(null);
        KLog.d("TeleOp_Shooting", () -> "event=pass reason=" + reason + " target=" + target + " rps=" + PASS_RPS + " hood=" + PASS_HOOD
                + " odometry=" + SharedData.getOdometryWheelIMUPosition());
    }

    /** Pass over or cancelled: distance-based RPS again and re-aim at the Hive Cell. */
    private void endPass() {
        passAction = null;
        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);
        hiveTarget = null; // force updateHiveTarget() to re-apply the Cell to the turret, shooter and SOTM
        updateHiveTarget();
        KLog.d("TeleOp_Shooting", "event=passEnd");
    }

    private void handleZeroing() {
        if (zeroLimelightPressed) {
            if (!isPending(resetToLimelight)) {
                // Skips on its own with no detection or a moving turret, so it never snaps odometry to the origin.
                resetToLimelight = new ResetOdometryToLimelight(turret);
                setLastZeroAction(resetToLimelight);
                forceUpdateShootingActions();
                KLog.d("TeleOp_Zeroing", () -> "Limelight zero requested - Odometry Position: " + SharedData.getOdometryWheelIMUPosition() +
                        " Limelight Pos: " + SharedData.getLimelightGlobalPosition());
            }
            return;
        }

        if (zeroCornerPressed) {
            if (!isPending(resetOdometryToPos)) {
                resetOdometryToPos = new ResetOdometryToPos(FieldConfig.CORNER_ZERO_POSE);
                setLastZeroAction(resetOdometryToPos);
                KLog.d("TeleOp_Zeroing", () -> "Corner zero started - Position: " + SharedData.getOdometryWheelIMUPosition());
            }
        }
    }

    public void forceUpdateShootingActions() {
        turretAutoAlignTeleOp.updateCheckDone();
        shooterRun.updateCheckDone();
    }
}
