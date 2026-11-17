package org.firstinspires.ftc.teamcode.kalipsorobotics.decode;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.*;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig.*;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
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
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
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
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
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
    private Odometry odometry = null;
    private SOTM sotm = null;

    ShootAllAction shootAllAction = null;


    PurePursuitAction parkAction = null;
    PurePursuitAction leverAction = null;


    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;

    IntakeController intakeController = null;
    private boolean flowerWanted = false;
    IntakeStop intakeStop = null;
    IntakeReverse intakeReverse = null;

    DriveController driveAction = null;

    ResetOdometryToLimelight resetOdometryToLimelight = null;
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
    private boolean enableLimelightAlignTurret = false;
    private boolean enableOdometryAlignTurret = false;
    private boolean incrementRPSPressed;
    private boolean decrementRPSPressed;
    private boolean incrementHoodPressed;
    private boolean decrementHoodPressed;
    private boolean kGamepad2IsDpadLeftFirstPressed;
    private boolean kGamepad2IsDpadRightFirstPressed;
    private boolean setTurretOffset0Pressed;
    private boolean parkButtonPressed;
    private boolean leverButtonPressed;
    private boolean enableLimelightZeroing = true;

    private Point hiveTarget = null; // the Hive Cell we aim at, see updateHiveTarget()
    private int shootCount = 0;
    private double lastLoopTime = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = SharedData.getAllianceColor();
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        KLog.d("TeleOp-Init", "Starting initializeRobot()");
        int tagId;
        if (allianceColor == AllianceColor.RED) {
            tagId = 24;
        } else {
            tagId = 20;
        }
        // ============== MODULES =================
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        turret = Turret.getInstance(opModeUtilities);
        intake = new Intake(opModeUtilities);
        intakeController = new DriveDirectionIntakeController(intake);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        // =========================================


        // ============== LONG RUNNING ACTIONS =================

        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
//        odometry.setShouldFallbackToWheelTheta(true);
        aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, tagId, allianceColor);

        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);
        OpModeUtilities.runAprilTagExecutorService(aprilTagExecutorService, aprilTagDetectionAction);
        sotm = new SOTM(opModeUtilities);
        if (SOTMConfig.enabled) OpModeUtilities.runSOTMExecutorService(sotmExecutorService, sotm);
        driveAction = USE_FIELD_ORIENTED_DRIVE
                ? new FieldOrientedDriveAction(driveTrain, imuModule)
                : new DriveAction(driveTrain);
        // Both aim at the Hive Cell on the robot's end of the Hive; updateHiveTarget() keeps them on it every loop.
        hiveTarget = FieldConfig.hiveTargetForSide(SharedData.getOdometryWheelIMUPosition().getY(), null);
        shooterRun = new ShooterRun(opModeUtilities, shooter, hiveTarget);
        turretAutoAlignTeleOp = new TurretAutoAlignTeleOp(opModeUtilities, turret);
        turretAutoAlignTeleOp.setTargetPoint(hiveTarget);
        SharedData.setSOTMActiveGoal(hiveTarget == FieldConfig.aTargetPoint ? 0 : 1);

        // =========================================

        KLog.d("TeleOp-Init", "Creating intake, shooter, stopper, turret modules");
        KLog.d("TeleOp-Init", () -> "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));

        KLog.d("TeleOp-Init", () -> "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        KLog.d("TeleOp-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("teleop", "--------------TELEOP STARTED-------------");
        KLog.d("TeleOp-Run", () -> "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_OPEN_POS);
        waitForStart();
        telemetry.setMsTransmissionInterval(200);
        sleep(50);
        //Wait for Executor Thread to start

        KLog.d("TeleOp-Run", () -> "After waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));

        turretAutoAlignTeleOp.setToleranceDeg(2);

        if (stopper == null) {
            KLog.e("TeleOp-Run", "ERROR: Stopper is NULL when trying to create closeStopper action!");
        } else {
            KLog.d("TeleOp-Run", "Creating closeStopper action - stopper is valid");
        }

        KLog.d("TeleOp-Run", "closeStopper created successfully");

        while (opModeIsActive()) {
            // Dashboard may flip SOTM on mid-match; the thread exits itself when it's turned off.
            if (SOTMConfig.enabled && !sotm.isRunning()) OpModeUtilities.runSOTMExecutorService(sotmExecutorService, sotm);
            opModeUtilities.clearBulkCache();
            lastLoopTime = loopTimer.milliseconds();
            loopTimer.reset();
            Log.d("Loop_Speed", (int) lastLoopTime + "ms");
            if (!hasClosedStopperInnit) {
                stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
                hasClosedStopperInnit = true;
            }
            // ========== READ ALL INPUTS (makes it clear what buttons do) ==========
            drivingSticksActive = kGamePad1.isAnyStickActive();

            parkButtonPressed = kGamePad1.isButtonXFirstPressed();
            leverButtonPressed = kGamePad1.isButtonYFirstPressed();

            slowDrivePressed = kGamePad1.isRightBumperPressed();

            shootAllActionPressed = kGamePad1.isLeftBumperPressed();
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
//            enableLimelightAlignTurret = kGamePad2.isBackButtonToggle(); TURNED OFF LIMELIGHT
            enableLimelightAlignTurret = false;
            KLog.d("TeleOp_Button", () -> "enableLimelightAlignTurret: " + enableLimelightAlignTurret);
            enableOdometryAlignTurret = true; // odometry align always on; GP2 B is the flower toggle
            KLog.d("TeleOp_Button", () -> "enableOdometryAlignTurret: " + enableOdometryAlignTurret);



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
            telemetry.addData("Intake side", intakeController.getSelectedSide());
            telemetry.addData("Balls", intake.getBallCount());
            telemetry.addData("Flower wanted/down", flowerWanted + "/" + intake.getFlower().isDown());
            telemetry.addData("Target Rps ", "%.2f Target Hood %.2f",shooterRun.getTargetRPS(), shooterRun.getTargetHoodPosition());
            telemetry.addData("Current RPS ", "%.2f", shooterRun.getCurrentRPS());
            telemetry.addData("Hive Cell ", hiveTarget == FieldConfig.aTargetPoint ? "A" : "B");
            telemetry.addData("Distance ", "%.0f", shooterRun.getDistanceMM());
            telemetry.addData("Rps Offset ", "%.2f", ShooterInterpolationConfig.rpsOffset);
            telemetry.addData("Hood Offset ", "%.2f", ShooterInterpolationConfig.hoodOffset);
            telemetry.addData("Turret Trim Deg ", "%.1f", TurretConfig.TRIM_DEG);
            telemetry.addLine("Turret Mode: " + turretAutoAlignTeleOp.getTurretRunMode());
            telemetry.addData("Delta Heading (Turret)", turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Delta RPS (Shooter)", shooterRun.getTargetRPS() - shooterRun.getCurrentRPS());
            telemetry.update();
        }
        cleanupRobot();
    }
    private void handleDriving() {

        if (parkButtonPressed) {
            if (parkAction == null || parkAction.getIsDone()) {
                parkAction = new PurePursuitAction(driveTrain);
                parkAction.addPoint(650, -1400 * allianceColor.getPolarity(), -90 * allianceColor.getPolarity());
                parkAction.setFinalSearchRadiusMM(50);
                setLastMoveAction(parkAction);
            }
        }

        if (leverButtonPressed) {
            if (leverAction == null || leverAction.getIsDone()) {
                leverAction = new PurePursuitAction(driveTrain);
//                leverAction.addPoint(1575, 908 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
//                leverAction.addPoint(1325, 1025 * allianceColor.getPolarity(), 52 * allianceColor.getPolarity());
                //move to lever
                leverAction.addPoint(1600, 850 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                //eat at lever
                leverAction.addPoint(1325, 1075 * allianceColor.getPolarity(), 60 * allianceColor.getPolarity()); // eating point //theta 57.3
                setLastMoveAction(leverAction);
            }
        }

        if (slowDrivePressed) {
            driveAction.setPowerCoefficient(0.3);
        } else {
            driveAction.setPowerCoefficient(1);
        }

        if (drivingSticksActive) {
            driveAction.move(gamepad1);
            leverAction = null;
            parkAction = null;
            setLastMoveAction(null);
        } else if ((parkAction == null || parkAction.getIsDone()) && (leverAction == null || leverAction.getIsDone())) {
            driveTrain.setPower(0);
        }
    }
    private void handleTurret() {
        if (isPending(shootAllAction)) {
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


        //Manual
        if (kGamepad2IsDpadLeftFirstPressed) {
            TurretConfig.TRIM_DEG -= 2;
        } else if (kGamepad2IsDpadRightFirstPressed) {
            TurretConfig.TRIM_DEG += 2;
        }

        if (setTurretOffset0Pressed) {
            TurretConfig.TRIM_DEG = 0;
        }


        if (KLog.isDebug()) {
            KLog.d("TeleOp_Turret", () -> "angleDeg=" + turret.getAngleDeg() + " trimDeg=" + TurretConfig.TRIM_DEG);
        }
    }

    /**
     * Aim at the Hive Cell on the robot's end of the Hive (A at high y, B at low y). Same numbers for both
     * alliances: odometry is in our alliance's own frame (FieldConfig). ShooterRun recomputes RPS and hood
     * from the distance to this point on every update.
     */
    private void updateHiveTarget() {
        Point next = FieldConfig.hiveTargetForSide(SharedData.getOdometryWheelIMUPosition().getY(), hiveTarget);
        if (next == hiveTarget) {
            return;
        }
        hiveTarget = next;
        turretAutoAlignTeleOp.setTargetPoint(hiveTarget);
        shooterRun.setTargetPoint(hiveTarget);
        SharedData.setSOTMActiveGoal(hiveTarget == FieldConfig.aTargetPoint ? 0 : 1);
        KLog.d("TeleOp_Target", () -> "event=cellSwitch cell=" + (hiveTarget == FieldConfig.aTargetPoint ? "A" : "B")
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
        if (isPending(shootAllAction) || shootAllActionPressed) {
            return;  // Exit early - shoot has control
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
        if (isPending(shootAllAction) || shootAllActionPressed) {
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
            shooterRun.stop();

            if (shootAllAction != null) {
                shootAllAction.setIsDone(true);
            }
            shootAllAction = null;
            setLastShooterAction(null);

            KLog.d("TeleOp_Shooting", "Shooter stop");
            return;
        }

        //Shooting Running --> return;
        if (isPending(shootAllAction)) {
            //KLog.d("TeleOp_Shooting", "shootAllAction Pending -> Return");
            return;
        }

        // Priority 2- Execute shoot sequence
        if (shootAllActionPressed) {
            if (!isPending(shootAllAction)) {
                if (toggleTurretAlign) {
                    turnOnTurret();
                }

                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, shooterRun, turretAutoAlignTeleOp);
                shootAllAction.setEnableLimelightZeroing(enableLimelightZeroing);
                shooterRun.setUseOdometry(toggleTurretAlign);
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


    }

    private void handleZeroing() {
        if (zeroLimelightPressed) {
            if (!isPending(resetOdometryToPos)) {
                Position unfilteredLimelightPos = SharedData.getUnfilteredLimelightGlobalPos();
                resetOdometryToPos = new ResetOdometryToPos(unfilteredLimelightPos);
                resetOdometryToPos.updateCheckDone();
                setLastZeroAction(resetOdometryToPos);
                enableLimelightZeroing = true;
                forceUpdateShootingActions();
                KLog.d("TeleOp_Zeroing", () -> "Force Limelight zero action started - Odometry Position: " + SharedData.getOdometryWheelIMUPosition() +
                        " Unfiltered Limelight pos: " + unfilteredLimelightPos +
                        " Filtered Limelight Pos: " + SharedData.getLimelightGlobalPosition()
                );
            }
            return;
        }

        if (zeroCornerPressed) {
            if (!isPending(resetOdometryToPos)) {
                KLog.d("TeleOp_Zeroing", "Zero Corner button pressed - resetting odometry to corner position");

                // TODO: Update with actual corner coordinates
                double cornerX = 0;
                double cornerY = -2006.6 * allianceColor.getPolarity();
                double cornerTheta = 0;
                Position cornerPosition = new Position(cornerX, cornerY, cornerTheta);

                resetOdometryToPos = new ResetOdometryToPos(cornerPosition);
                enableLimelightZeroing = false;
                setLastZeroAction(resetOdometryToPos);
                KLog.d("TeleOp_Zeroing", () -> "Corner zero action started - Position: " + SharedData.getOdometryWheelIMUPosition());
            }
        }
    }

    public void forceUpdateShootingActions() {
        turretAutoAlignTeleOp.updateCheckDone();
        shooterRun.updateCheckDone();
    }
}
