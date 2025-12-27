package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.actions.drivetrain.ActivateBraking;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.ReleaseBrakeAction;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRunFullSpeed;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleop;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.ResetOdometryToPosition;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterInterpolationConfig;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.modules.shooter.ShotLogger;

/**
 * Simplified redesign of RedFarTeleOp
 *
 * Key improvements:
 * 1. Priority-based structure (highest priority first, early returns)
 * 2. All inputs read at top of loop (clear what buttons do)
 * 3. Grouped by functionality (intake, shooting, etc.)
 * 4. Less fragile - priority is explicit, not dependent on if-else order
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends KOpMode {
    private boolean hasClosedStopperInnit = false;
    private boolean turretReset = false;
    private DriveTrain driveTrain;
    private DriveBrake driveBrake;

    private Shooter shooter = null;
    private Intake intake = null;
    private Turret turret = null;
    private Stopper stopper = null;
    private Odometry odometry = null;

    ShootAllAction shootAllAction = null;

    ActivateBraking activateBraking = null;
    ReleaseBraking releaseBraking = null;
    ReleaseBrakeAction releaseBrakeAction = null;

    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;

    IntakeRunFullSpeed intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;
    ShotLogger shotLogger = null;
    AprilTagDetectionAction aprilTagDetectionAction = null;

    // Button state variables
    private boolean drivingSticksActive = false;
    private boolean shootAllPressed = false;
    private boolean forceShootFarPressed = false;
    private boolean intakeRunPressed = false;
    private boolean intakeReversePressed = false;
    private boolean stopShooterPressed = false;
    private boolean warmupFarPressed = false;
    private boolean warmupAutoPressed = false;
    private boolean releasePressed = false;
    private boolean markUndershotPressed = false;
    private boolean markOvershotPressed = false;
    private boolean shooterReadyWasNotDone = false;
    private boolean limelightCorrectionPressed = false;
    private boolean isTurretAutoAlignEnabled = false;
    private boolean useLimelight = true;
    private boolean forceShootNearPressed = false;
    private boolean enableOdometryAlignTurret = true;
    private double lastPower = 0;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = SharedData.getAllianceColor();
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        KLog.d("TeleOp-Init", "Starting initializeRobot()");

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveBrake = new DriveBrake(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000);

        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
//        odometry.setShouldFallbackToWheelTheta(true);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        driveAction = new DriveAction(driveTrain);

        KLog.d("TeleOp-Init", "Creating intake, shooter, stopper, turret modules");
        KLog.d("TeleOp-Init", "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        KLog.d("TeleOp-Init", "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        turret = Turret.getInstance(opModeUtilities);

        int tagId;
        if (allianceColor == AllianceColor.RED) {
            tagId = 24;
        }
        else {
            tagId = 20;
        }

        shooterRun = new ShooterRun(shooter, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()));

        aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, tagId, allianceColor);
        turretAutoAlignTeleop = new TurretAutoAlignTeleop(opModeUtilities, turret, aprilTagDetectionAction, allianceColor);

        shotLogger = new ShotLogger(opModeUtilities);

        KLog.d("TeleOp-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("teleop", "--------------TELEOP STARTED-------------");
        KLog.d("TeleOp-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        waitForStart();
        sleep(50);
        //Wait for Executor Thread to start

        KLog.d("TeleOp-Run", "After waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));

//        turretAutoAlign.setToleranceDeg(0.5);

        if (stopper == null) {
            KLog.e("TeleOp-Run", "ERROR: Stopper is NULL when trying to create closeStopper action!");
        } else {
            KLog.d("TeleOp-Run", "Creating closeStopper action - stopper is valid");
        }

        KLog.d("TeleOp-Run", "closeStopper created successfully");

        while (opModeIsActive()) {
            if (!hasClosedStopperInnit) {
                stopper.setPosition(stopper.STOPPER_SERVO_CLOSED_POS);
                hasClosedStopperInnit = true;
            }
            // ========== READ ALL INPUTS (makes it clear what buttons do) ==========
            drivingSticksActive = kGamePad1.isAnyStickActive();

            forceShootFarPressed = kGamePad1.isRightBumperFirstPressed();
            forceShootNearPressed = kGamePad1.isRightTriggerFirstPressed();
            shootAllPressed = kGamePad1.isLeftBumperFirstPressed();
            stopShooterPressed = (kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed()) || kGamePad1.isLeftTriggerFirstPressed();
            warmupFarPressed = kGamePad2.isDpadUpFirstPressed();
            warmupAutoPressed = kGamePad2.isDpadDownFirstPressed();

            intakeRunPressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();

            releasePressed = kGamePad2.isYPressed();

            isTurretAutoAlignEnabled = kGamePad2.isToggleX();
            enableOdometryAlignTurret = !kGamePad2.isBackButtonToggle();

            markUndershotPressed = kGamePad2.isButtonXFirstPressed();
            markOvershotPressed = kGamePad2.isButtonYFirstPressed();
            limelightCorrectionPressed = kGamePad1.isBackButtonFirstPressed();  // Back button for vision correction

            // ========== HANDLE DRIVING ==========
            handleDriving();

            // ========== HANDLE LIMELIGHT ODOMETRY CORRECTION ==========

            handleTurret();
            // ========== HANDLE INTAKE (Priority Order: Shoot > Stall > Manual > Idle) ==========
            handleIntake();

            // ========== HANDLE STOPPER ==========
            handleStopper();

            // ========== HANDLE SHOOTING ==========
            handleShooting();

            // ========== HANDLE SHOT LOGGING ==========
            handleShotLogging();

            // ========== UPDATE ACTIONS ==========
            updateActions();

            Log.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }
    private void handleDriving() {
        if (drivingSticksActive) {
            if (releaseBrakeAction == null || releaseBrakeAction.getIsDone()) {
                releaseBrakeAction = new ReleaseBrakeAction(driveBrake, releaseBraking);
                setLastBrakingAction(releaseBrakeAction);
            }
            driveAction.move(gamepad1);
        } else {
            driveTrain.setPower(0);
        }
    }
    private void handleTurret() {

        // Auto Align
        if (isTurretAutoAlignEnabled) {
            turretAutoAlignTeleop.runWithOdometryAndLimelight();
        } else {
            //Manual
            if (kGamePad2.isDpadLeftPressed()) {
                turretAutoAlignTeleop.runWithPower(-0.25);
                lastPower = -0.25;
            } else if (kGamePad2.isDpadRightPressed()) {
                turretAutoAlignTeleop.runWithPower(0.25);
                lastPower = 0.25;
            } else if (lastPower != 0) {
                turretAutoAlignTeleop.stop();
                lastPower = 0;
            } else {
                turretAutoAlignTeleop.runWithLimelight();
            }
        }
        KLog.d("TeleOp_Turret", "Turret Power" + turret.turretMotor.getPower());
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
        if (isPending(shootAllAction) || shootAllPressed) {
            return;  // Exit early - shoot has control
        }

        // PRIORITY 3: Manual forward
        if (intakeRunPressed) {
            if (!isPending(intakeRun)) {
                intakeRun = new IntakeRunFullSpeed(intake);
                setLastIntakeAction(intakeRun);
            }
            return;  // Exit early
        }

        // PRIORITY 4: Manual reverse
        if (intakeReversePressed) {
            if (!isPending(intakeReverse)) {
                intakeReverse = new IntakeReverse(intake);
                setLastIntakeAction(intakeReverse);
            }
            return;  // Exit early
        }

        // PRIORITY 5: Stop when nothing pending
        if (!isPending(intakeStop)) {
            intakeStop = new IntakeStop(intake);
            setLastIntakeAction(intakeStop);
        }
    }

    /**
     * Handle stopper with clear priority
     */
    private void handleStopper() {
        // PRIORITY 1: If shooting, don't interfere (shoot action controls intake)
        if (isPending(shootAllAction) || shootAllPressed) {
            return;  // Exit early - shoot has control
        }

        // Open on button press
        if (releasePressed) {
            if (!isPending(openStopper)) {
                openStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
                closeStopper = null;
                setLastStopperAction(openStopper);
            }
            return;
        }

        // Close when intaking
        if (intakeRunPressed || isPending(intakeRun)) {
            if (!isPending(closeStopper)) {
                closeStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
                openStopper = null;
                setLastStopperAction(closeStopper);
            }
        }
    }

    /**
     * Handle shot logging and quality marking
     */
    private void handleShotLogging() {
        // Track when ShooterReady transitions from not done to done
        if (isPending(shootAllAction) && shootAllAction.getShooterReady() != null) {
            boolean shooterReadyIsDone = shootAllAction.getShooterReady().getIsDone();

            // Log when ShooterReady just became done (transition from false to true)
            if (shooterReadyWasNotDone && shooterReadyIsDone) {
                double targetRps = shootAllAction.getShooterRun().getTargetRPS();
                double actualRps = shootAllAction.getShooterRun().getShooter().getRPS();
                double hoodPos = shootAllAction.getShooterRun().getTargetHoodPosition();
                double distMM = shootAllAction.getShooterRun().getDistanceMM();

                shotLogger.logShot(targetRps, actualRps, hoodPos, distMM);
                shooterReadyWasNotDone = false; // Reset for next shot
            } else if (!shooterReadyIsDone) {
                // Track that shooter ready is not done yet
                shooterReadyWasNotDone = true;
            }
        } else {
            // Reset when no shoot action is pending
            shooterReadyWasNotDone = false;
        }

        // Handle X/Y keys for marking shot quality
        if (markUndershotPressed) {
            shotLogger.markLastShotAsUndershot(); // X
        } else if (markOvershotPressed) {
            shotLogger.markLastShotAsOvershot(); // Y
        }
    }

    /**
     * Handle shooting sequence
     */
    private void handleShooting() {
        // Priority 1- Stop shooter
        if (stopShooterPressed) {
            releaseBrakeAction = new ReleaseBrakeAction(driveBrake, releaseBraking);
            setLastBrakingAction(releaseBrakeAction);

            shooterRun.stop();

            if (shootAllAction != null) {
                shootAllAction.setIsDone(true);
            }
            shootAllAction = null;
            setLastShooterAction(shootAllAction);

            KLog.d("TeleOp_Shooting", "Shooter stop");
            return;
        }

        //Shooting Running --> return;
        if (isPending(shootAllAction)) {
            return;
        }

        // Priority 2- Execute shoot sequence
        if (shootAllPressed) {
            if (!isPending(shootAllAction)) {
                shootAllAction = new ShootAllAction(stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleop);
//                resetOdometryToPosition = new ResetOdometryToPosition(turret);
                shooterRun.setUseLimelight(useLimelight);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                if (isTurretAutoAlignEnabled) {
                    shootAllAction.setTurretReady(true);
                }
                KLog.d("TeleOp_Shooting", "Shoot action started - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        //
        if (forceShootFarPressed) {
            if (!isPending(shootAllAction)) {
                shootAllAction = new ShootAllAction(stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleop, ShooterInterpolationConfig.getMaxValue()[0], ShooterInterpolationConfig.getMaxValue()[1]);
                shooterRun.setUseLimelight(useLimelight);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                if (isTurretAutoAlignEnabled) {
                    shootAllAction.setTurretReady(true);
                }
                KLog.d("TeleOp_Shooting", "Shoot action started force shoot from far - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        if (forceShootNearPressed) {
            if (!isPending(shootAllAction)) {
                shootAllAction = new ShootAllAction(stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleop, ShooterInterpolationConfig.getNearValue()[0], ShooterInterpolationConfig.getNearValue()[1]);
                shooterRun.setUseLimelight(useLimelight);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                KLog.d("TeleOp_Shooting", "Shoot action started force shoot from far - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        if (warmupFarPressed) {
            if (!isPending(shootAllAction)) {
                shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
                shooterRun.setTargetRPS(ShooterInterpolationConfig.getMaxValue()[0]);
                shooterRun.setTargetHoodPosition(ShooterInterpolationConfig.getMaxValue()[1]);
                KLog.d("TeleOp_Shooting_Warmup", "Warmup For Far Shoot");
            }
        } else if (warmupAutoPressed) {
            if (!isPending(shootAllAction)) {
                shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);
                KLog.d("TeleOp_Shooting_Warmup", "Warmup For Auto Shoot");
            }
        }

    }

    @Override
    protected void cleanupRobot() {
        // Write all accumulated shot data to file
        if (shotLogger != null) {
            shotLogger.writeToFile();
            KLog.d("TeleOp", "Shot logger data written to file");
        }

        // Call parent cleanup
        super.cleanupRobot();
    }

}
