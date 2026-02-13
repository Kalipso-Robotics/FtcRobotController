package org.firstinspires.ftc.teamcode.kalipsorobotics.decode;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.ReleaseBrakeAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeReverse;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveBrake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Intake;
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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends KOpMode {
    private boolean hasClosedStopperInnit = false;
    private DriveTrain driveTrain;
    private DriveBrake driveBrake;

    private Shooter shooter = null;
    private Intake intake = null;
    private Turret turret = null;
    private Stopper stopper = null;
    private Odometry odometry = null;

    ShootAllAction shootAllAction = null;

    ReleaseBraking releaseBraking = null;
    ReleaseBrakeAction releaseBrakeAction = null;
    PurePursuitAction parkAction = null;
    PurePursuitAction leverAction = null;

    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;

    RunIntake intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;

    ResetOdometryToLimelight resetOdometryToLimelight = null;
    ResetOdometryToPos resetOdometryToCorner = null;

    // Button state variables
    private boolean drivingSticksActive = false;
    private boolean shootAllActionPressed = false;
    private boolean intakeStopPressed = false;
    private boolean intakeReversePressed = false;
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

    private int shootCount = 0;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = SharedData.getAllianceColor();
        TurretConfig.kP = TurretConfig.kP_teleop;
        TurretConfig.kI = TurretConfig.kI_teleop;
        TurretConfig.kD = TurretConfig.kD_teleop;
        TurretConfig.kF = TurretConfig.kF_teleop;
        TurretConfig.kS = TurretConfig.kS_teleop;
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

        shooterRun = new ShooterRun(opModeUtilities, shooter, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()));

        aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, tagId, allianceColor);
        turretAutoAlignTeleOp = new TurretAutoAlignTeleOp(opModeUtilities, aprilTagDetectionAction, turret, allianceColor);


        KLog.d("TeleOp-Init", "Finished initializeRobot()");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        KLog.d("teleop", "--------------TELEOP STARTED-------------");
        KLog.d("TeleOp-Run", "Before waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_OPEN_POS);
        waitForStart();
        sleep(50);
        //Wait for Executor Thread to start

        KLog.d("TeleOp-Run", "After waitForStart() - stopper is: " + (stopper != null ? "NOT NULL" : "NULL"));

        turretAutoAlignTeleOp.setToleranceDeg(2.5);

        if (stopper == null) {
            KLog.e("TeleOp-Run", "ERROR: Stopper is NULL when trying to create closeStopper action!");
        } else {
            KLog.d("TeleOp-Run", "Creating closeStopper action - stopper is valid");
        }

        KLog.d("TeleOp-Run", "closeStopper created successfully");

        while (opModeIsActive()) {
            if (!hasClosedStopperInnit) {
                stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
                hasClosedStopperInnit = true;
            }
            // ========== READ ALL INPUTS (makes it clear what buttons do) ==========
            drivingSticksActive = kGamePad1.isAnyStickActive();

            parkButtonPressed = kGamePad1.isButtonXFirstPressed();
            leverButtonPressed = kGamePad1.isButtonYFirstPressed();



            //============Driver 2============
            boolean kGamepad2IsDpadUpFirstPressed = kGamePad2.isDpadUpFirstPressed();
            boolean kGamepad2IsDpadDownFirstPressed = kGamePad2.isDpadDownFirstPressed();

            kGamepad2IsDpadLeftFirstPressed = kGamePad2.isDpadLeftFirstPressed();
            kGamepad2IsDpadRightFirstPressed = kGamePad2.isDpadRightFirstPressed();

            setTurretOffset0Pressed = kGamePad2.isAPressed();

            incrementRPSPressed = kGamepad2IsDpadUpFirstPressed && kGamePad2.isLeftBumperPressed();
            decrementRPSPressed = kGamepad2IsDpadDownFirstPressed && kGamePad2.isLeftBumperPressed();

            incrementHoodPressed = kGamepad2IsDpadRightFirstPressed && kGamePad2.isLeftBumperPressed();
            decrementHoodPressed = kGamepad2IsDpadLeftFirstPressed && kGamePad2.isLeftBumperPressed();

            shootAllActionPressed = kGamePad1.isLeftBumperPressed();
            stopShooterPressed = (kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed()) || kGamePad1.isLeftTriggerFirstPressed();
            zeroLimelightPressed = kGamepad2IsDpadUpFirstPressed;
            zeroCornerPressed = kGamepad2IsDpadDownFirstPressed;

            intakeStopPressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();

            releaseStopperPressed = kGamePad2.isYPressed();

            toggleTurretAlign = !kGamePad2.isToggleX();
            KLog.d("TeleOp_Button", "toggleTurretAlign: " + toggleTurretAlign);
//            enableLimelightAlignTurret = kGamePad2.isBackButtonToggle(); TURNED OFF LIMELIGHT
            enableLimelightAlignTurret = false;
            KLog.d("TeleOp_Button", "enableLimelightAlignTurret: " + enableLimelightAlignTurret);
            enableOdometryAlignTurret = !kGamePad2.isToggleB();
            KLog.d("TeleOp_Button", "enableOdometryAlignTurret: " + enableOdometryAlignTurret);


            // ========== HANDLE DRIVING ==========
            handleDriving();

            // ========== HANDLE ZEROING (LIMELIGHT & CORNER) ==========
            handleZeroing();

            // ========== HANDLE TURRET ==========
            handleTurret();
            // ========== HANDLE INTAKE (Priority Order: Shoot > Stall > Manual > Idle) ==========
            handleIntake();

            // ========== HANDLE STOPPER ==========
            handleStopper();

            // ========== HANDLE SHOOTING ==========
            handleShooting();

            // ========== UPDATE ACTIONS ==========
            updateActions();
            telemetry.addData("Odometry: ", SharedData.getOdometryWheelIMUPosition().toCompactString());
            telemetry.addData("Shot: ", shootCount);
            telemetry.addData("Target Rps ", "%.2f Target Hood %.2f",shooterRun.getTargetRPS(), shooterRun.getTargetHoodPosition());
            telemetry.addData("Current RPS ", "%.2f", shooter.getRPS());
            telemetry.addData("Distance ", "%.0f", ShooterRun.getDistanceToTargetFromCurrentPos(Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity())));
            telemetry.addData("Rps Offset ", "%.2f", ShooterInterpolationConfig.rpsOffset);
            telemetry.addData("Hood Offset ", "%.2f", ShooterInterpolationConfig.hoodOffset);
            telemetry.addLine("Turret Offset " + turretAutoAlignTeleOp.getTicksOffset());
            telemetry.addLine("Turret Mode: " + turretAutoAlignTeleOp.getTurretRunMode());
            telemetry.update();
        }
        cleanupRobot();
    }
    private void handleDriving() {

        if (parkButtonPressed) {
            if (parkAction == null || parkAction.getIsDone()) {
                parkAction = new PurePursuitAction(driveTrain);
                parkAction.addPoint(557.99, -1316.56 * allianceColor.getPolarity(), 0);
                setLastMoveAction(parkAction);
            }
        }

        if (leverButtonPressed) {
            if (leverAction == null || leverAction.getIsDone()) {
                leverAction = new PurePursuitAction(driveTrain);
                leverAction.addPoint(1575, 908 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
                leverAction.addPoint(1325, 1025 * allianceColor.getPolarity(), 52 * allianceColor.getPolarity());
                setLastMoveAction(leverAction);
            }
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
            turretAutoAlignTeleOp.incrementTicksOffset(-(int) TurretConfig.TICKS_PER_DEGREE * 2);
        } else if (kGamepad2IsDpadRightFirstPressed) {
            turretAutoAlignTeleOp.incrementTicksOffset((int) TurretConfig.TICKS_PER_DEGREE * 2);
        }

        if (setTurretOffset0Pressed) {
            turretAutoAlignTeleOp.setTicksOffset(0);
        }


        KLog.d("TeleOp_Turret", "Turret Power" + turret.turretMotor.getPower());
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
        // PRIORITY 3: Manual forward
        if (!isPending(intakeRun)) {
            intakeRun = new RunIntake(intake);
            setLastIntakeAction(intakeRun);
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

        // Close when intaking
        if (intakeStopPressed || isPending(intakeRun)) {
            KLog.d("TeleOp_Stopper", "Intake Pressed Close Stopper isPending: " + isPending(closeStopper));
            if (!isPending(closeStopper)) {
                KLog.d("TeleOp_Stopper", "Close Stopper");
                closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
                closeStopper.setName("closeStopper");
                openStopper = null;
                setLastStopperAction(closeStopper);
            }
        }
    }

    /**
     * Handle shooting sequence
     */
    private void handleShooting() {


        if (incrementRPSPressed) {
            ShooterInterpolationConfig.rpsOffset += 0.1;
            KLog.d("TeleOp_Shooting_RPS_offset", "Increment Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        } else if (decrementRPSPressed) {
            ShooterInterpolationConfig.rpsOffset -= 0.1;
            KLog.d("TeleOp_Shooting_RPS_offset", "Decrement Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        }

        double hoodPosition;

        if (incrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset += 0.02;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() + 0.02);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting_Hood_Offset", "Increment Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
        } else if (decrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset -= 0.02;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() - 0.02);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting_Hood_offset", "Decrement Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
        }

        // Priority 1- Stop shooter
        if (stopShooterPressed) {
            releaseBrakeAction = new ReleaseBrakeAction(driveBrake, releaseBraking);
            setLastBrakingAction(releaseBrakeAction);

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
                if (enableLimelightZeroing) {
                    resetOdometryToLimelight = new ResetOdometryToLimelight(turret);
                    resetOdometryToLimelight.updateCheckDone();
                    forceUpdateShootingActions();
                }
                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleOp);

                shooterRun.setUseOdometry(toggleTurretAlign);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                shootCount++;
                shootAllAction.setName("Shot_" + shootCount);
                KLog.d("Teleop_Shooting", "Shot_" + shootCount + " - " +
                        "Delta RPS: " + (shooter.getRPS() - shootAllAction.getShooterRun().getTargetRPS())  +
                        "Distance " + shooterRun.getDistanceMM() +
                        "Odometry " + SharedData.getOdometryWheelIMUPosition() +
                        "Limelight Pos " + SharedData.getLimelightGlobalPosition() +
                        "Turret Delta Angle " + turretAutoAlignTeleOp.getDeltaAngleDeg()
                );
            }
        }


    }

    private void handleZeroing() {
        if (zeroLimelightPressed) {
            if (!isPending(resetOdometryToLimelight)) {
                KLog.d("TeleOp_Zeroing", "Zero Limelight button pressed - resetting odometry to limelight position");
                resetOdometryToLimelight = new ResetOdometryToLimelight(turret);
                enableLimelightZeroing = true;
                setLastZeroAction(resetOdometryToLimelight);
                KLog.d("TeleOp_Zeroing", "Limelight zero action started");
            }
            return;
        }

        if (zeroCornerPressed) {
            if (!isPending(resetOdometryToCorner)) {
                KLog.d("TeleOp_Zeroing", "Zero Corner button pressed - resetting odometry to corner position");

                // TODO: Update with actual corner coordinates
                double cornerX = 0;
                double cornerY = -2006.6 * allianceColor.getPolarity();
                double cornerTheta = 0;
                Position cornerPosition = new Position(cornerX, cornerY, cornerTheta);

                resetOdometryToCorner = new ResetOdometryToPos(cornerPosition);
                enableLimelightZeroing = false;
                setLastZeroAction(resetOdometryToCorner);
                KLog.d("TeleOp_Zeroing", "Corner zero action started - Position: " + SharedData.getOdometryWheelIMUPosition());
            }
        }
    }

    public void forceUpdateShootingActions() {
        turretAutoAlignTeleOp.updateCheckDone();
        shooterRun.updateCheckDone();
    }
}
