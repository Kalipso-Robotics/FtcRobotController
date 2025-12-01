package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRunFullSpeed;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.shooter.ShooterWarmupAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
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
    protected final Point SHOOTER_TARGET_POINT = Shooter.TARGET_POINT;
    private boolean hasClosedStopperInnit = false;
    private DriveTrain driveTrain;
    private Shooter shooter = null;
    private Intake intake = null;
    private Turret turret = null;
    private Stopper stopper = null;
    private Odometry odometry = null;

    ShooterWarmupAction shooterWarmup = null;
    ShooterStop shooterStop = null;
    ShootAllAction shootAction = null;

    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;

    IntakeRunFullSpeed intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;
    TurretAutoAlign turretAutoAlign = null;

    ShotLogger shotLogger = null;

    // Button state variables
    private boolean drivingSticks = false;
    private boolean shootPressed = false;
    private boolean intakeRunPressed = false;
    private boolean intakeReversePressed = false;
    private boolean stopShooterPressed = false;
    private boolean warmupPressed = false;
    private boolean releasePressed = false;
    private boolean markUndershotPressed = false;
    private boolean markOvershotPressed = false;
    private boolean shooterReadyWasNotDone = false;
    private boolean limelightCorrectionPressed = false;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = SharedData.getAllianceColor();
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        KLog.d("TeleOp-Init", "Starting initializeRobot()");

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000);

        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        driveAction = new DriveAction(driveTrain);

        KLog.d("TeleOp-Init", "Creating intake, shooter, stopper, turret modules");
        KLog.d("TeleOp-Init", "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        KLog.d("TeleOp-Init", "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));

        turret = Turret.getInstance(opModeUtilities);

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        shotLogger = new ShotLogger(opModeUtilities);

        KLog.d("TeleOp-Init", "Finished initializeRobot()");

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("teleop", "--------------TELEOP STARTED-------------");
        KLog.d("TeleOp-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        waitForStart();
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
                stopper.setPosition(Stopper.STOPPER_SERVO_CLOSED_POS);
                hasClosedStopperInnit = true;
            }
            // ========== READ ALL INPUTS (makes it clear what buttons do) ==========
            drivingSticks = kGamePad1.getLeftStickY() != 0 ||
                    kGamePad1.getRightStickX() != 0 ||
                    kGamePad1.getLeftStickX() != 0;

            shootPressed = kGamePad1.isLeftBumperFirstPressed();
            intakeRunPressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();
            stopShooterPressed = (kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed()) || kGamePad1.isRightBumperFirstPressed();
            warmupPressed = kGamePad2.isDpadUpFirstPressed();
            releasePressed = kGamePad2.isYPressed();
            markUndershotPressed = kGamePad2.isButtonXFirstPressed();
            markOvershotPressed = kGamePad2.isButtonYFirstPressed();
//            limelightCorrectionPressed = kGamePad1.isBackPressed();  // Back button for vision correction

            // ========== HANDLE DRIVING ==========
            if (drivingSticks) {
                driveAction.move(gamepad1);
            } else {
                driveTrain.setPower(0);
            }

            // ========== HANDLE LIMELIGHT ODOMETRY CORRECTION ==========

            if (kGamePad2.isDpadLeftPressed()) {
                turretAutoAlign.incrementYInitSetupMM(-10);
                if (turretAutoAlign != null) {
                    turretAutoAlign.updateCheckDone();
                }
            } else if (kGamePad2.isDpadRightPressed()) {
                turretAutoAlign.incrementYInitSetupMM(10);
                if (turretAutoAlign != null) {
                    turretAutoAlign.updateCheckDone();
                }
            }

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

            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
        }

        cleanupRobot();
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
        if (isPending(shootAction) || shootPressed) {
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
        if (isPending(shootAction) || shootPressed) {
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
        if (isPending(shootAction) && shootAction.getShooterReady() != null) {
            boolean shooterReadyIsDone = shootAction.getShooterReady().getIsDone();

            // Log when ShooterReady just became done (transition from false to true)
            if (shooterReadyWasNotDone && shooterReadyIsDone) {
                double targetRps = shootAction.getShooterRun().getTargetRPS();
                double actualRps = shootAction.getShooterRun().getShooter().getRPS();
                double hoodPos = shootAction.getShooterRun().getTargetHoodPosition();
                double distMM = shootAction.getShooterRun().getDistanceMM();

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
        // Update turret when shooting or warmup
        if ((shooter.isRunning()) || (isPending(shooterWarmup)) || (isPending(shootAction))) {
            turretAutoAlign.updateCheckDone();
        } else {
            // Stop turret motor when not shooting to prevent drift
            turretAutoAlign.stop();
        }

        // Priority 1- Stop shooter
        if (stopShooterPressed) {
            if (!isPending(shooterStop)) {
                shooterStop = new ShooterStop(shooter, shootAction);
                shooterWarmup = null;
                shootAction = null;
                setLastShooterAction(shooterStop);
                KLog.d("Shooting", "Shooter stop");
            }
            return;
        }

        //Shooting Running --> return;
        if (isPending(shootAction)) {
            return;
        }

        // Priority 2- Execute shoot sequence
        if (shootPressed) {
            if (!isPending(shootAction)) {
                shootAction = new ShootAllAction(stopper, intake, shooter, turretAutoAlign, SHOOTER_TARGET_POINT.multiplyY(allianceColor.getPolarity()));
                shooterWarmup = null;
                setLastShooterAction(shootAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                KLog.d("Shooting", "Shoot action started - Target RPS: " + shootAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        //  Priority 3 -Warmup shooter
        if (warmupPressed) {
            if (!isPending(shooterWarmup)) {
                shooterWarmup = new ShooterWarmupAction(shooter, 0.3);
                setLastShooterAction(shooterWarmup);
                KLog.d("Shooting", "Warmup started");
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
