package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.ReleaseBrakeAction;
import com.kalipsorobotics.actions.drivetrain.ReleaseBraking;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.RunIntake;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import com.kalipsorobotics.localization.ResetOdometryToPosition;
import com.kalipsorobotics.localization.ResetOdometryToPos;
import com.kalipsorobotics.decode.configs.ModuleConfig;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.test.turret.TurretRunMode;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;


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

    ResetOdometryToPosition resetOdometryToLimelight = null;
    ResetOdometryToPos resetOdometryToCorner = null;

    // Button state variables
    private boolean drivingSticksActive = false;
    private boolean shootAllActionPressed = false;
    private boolean forceShootFarPressed = false;
    private boolean intakeRunPressed = false;
    private boolean intakeReversePressed = false;
    private boolean stopShooterPressed = false;
    private boolean zeroLimelightPressed = false;
    private boolean zeroCornerPressed = false;
    private boolean releaseStopperPressed = false;
    private boolean toggleTurretAlign = false;
    private boolean forceShootNearPressed = false;
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

            forceShootFarPressed = kGamePad1.isRightBumperFirstPressed();
            forceShootNearPressed = kGamePad1.isRightTriggerFirstPressed();
            shootAllActionPressed = kGamePad1.isLeftBumperFirstPressed();
            stopShooterPressed = (kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed()) || kGamePad1.isLeftTriggerFirstPressed();
            zeroLimelightPressed = kGamepad2IsDpadUpFirstPressed;
            zeroCornerPressed = kGamepad2IsDpadDownFirstPressed;

            intakeRunPressed = kGamePad2.isRightTriggerPressed();
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

            telemetry.addLine("Target Rps " + shooterRun.getTargetRPS() + " Target Hood " + shooterRun.getTargetHoodPosition());
            telemetry.addData("Distance to April Tag", SharedData.getLimelightRawPosition().getAprilTagDistanceToCamMM());
            telemetry.addData("Rps Offset", "%.2f", ShooterInterpolationConfig.rpsOffset);
            telemetry.addData("Hood Offset", "%.2f", ShooterInterpolationConfig.hoodOffset);
            KLog.d("Odometry", "Position: " + SharedData.getOdometryWheelIMUPosition());

            telemetry.update();
        }
        cleanupRobot();
    }
    private void handleDriving() {

        if (parkButtonPressed) {
            if (parkAction == null || parkAction.getIsDone()) {
                parkAction = new PurePursuitAction(driveTrain);
                parkAction.addPoint(639.79, -1238.30 * allianceColor.getPolarity(), 0);
                setLastMoveAction(parkAction);
            }
        }

        if (leverButtonPressed) {
            if (leverAction == null || leverAction.getIsDone()) {
                leverAction = new PurePursuitAction(driveTrain);
                leverAction.addPoint(1325, 1200 * allianceColor.getPolarity(), 52 * allianceColor.getPolarity());
                setLastMoveAction(leverAction);
            }
        }


        if (drivingSticksActive) {
            if (releaseBrakeAction == null || releaseBrakeAction.getIsDone()) {
                releaseBrakeAction = new ReleaseBrakeAction(driveBrake, releaseBraking);
                setLastBrakingAction(releaseBrakeAction);
            }
            if (shootAllAction != null) {
                shootAllAction.setIsDone(true);
                closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
                setLastStopperAction(closeStopper);
            }
            driveAction.move(gamepad1);
            setLastMoveAction(null);
        } else if (parkAction == null || parkAction.getIsDone()){
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

        telemetry.addLine("Running using: " + turretAutoAlignTeleOp.getTurretRunMode());

        //Manual
        if (kGamepad2IsDpadLeftFirstPressed) {
            turretAutoAlignTeleOp.incrementTicksOffset(-(int) TurretConfig.TICKS_PER_DEGREE * 2);
        } else if (kGamepad2IsDpadRightFirstPressed) {
            turretAutoAlignTeleOp.incrementTicksOffset((int) TurretConfig.TICKS_PER_DEGREE * 2);
        }

        if (setTurretOffset0Pressed) {
            turretAutoAlignTeleOp.setTicksOffset(0);
        }

        telemetry.addLine("Turret Offset " + turretAutoAlignTeleOp.getTicksOffset());

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

        // PRIORITY 3: Manual forward
        if (intakeRunPressed) {
            if (!isPending(intakeRun)) {
                intakeRun = new RunIntake(intake);
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
        if (isPending(shootAllAction) || shootAllActionPressed) {
            return;  // Exit early - shoot has control
        }

        // Open on button press
        if (releaseStopperPressed) {
            if (!isPending(openStopper)) {
                openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
                closeStopper = null;
                setLastStopperAction(openStopper);
            }
            return;
        }

        // Close when intaking
        if (intakeRunPressed || isPending(intakeRun)) {
            KLog.d("TeleOp_Stopper", "Intake Pressed Close Stopper isPending: " + isPending(closeStopper));
            if (!isPending(closeStopper)) {
                KLog.d("TeleOp_Stopper", "Close Stopper");
                closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
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
            KLog.d("TeleOp_Shooting", "Increment Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        } else if (decrementRPSPressed) {
            ShooterInterpolationConfig.rpsOffset -= 0.1;
            KLog.d("TeleOp_Shooting", "Decrement Shooter rps offset: " + ShooterInterpolationConfig.rpsOffset);
        }

        double hoodPosition;

        if (incrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset += 0.1;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() + 0.1);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting", "Increment Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
        } else if (decrementHoodPressed) {
            ShooterInterpolationConfig.hoodOffset -= 0.1;
            hoodPosition = KServo.clampServoPos(shooter.getHoodPosition() - 0.1);
            shooter.getHood().setPosition(hoodPosition);
            KLog.d("TeleOp_Shooting", "Decrement Shooter hood offset: " + ShooterInterpolationConfig.hoodOffset);
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
            KLog.d("TeleOp_Shooting", "shootAllAction Pending -> Return");
            return;
        }

        // Priority 2- Execute shoot sequence
        if (shootAllActionPressed) {
            if (!isPending(shootAllAction)) {
                if (toggleTurretAlign) {
                    turnOnTurret();
                }
                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleOp);
                shooterRun.setUseOdometry(toggleTurretAlign);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                KLog.d("TeleOp_Shooting", "Shoot action started - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        //
        if (forceShootFarPressed) {
            if (!isPending(shootAllAction)) {
                if (toggleTurretAlign) {
                    turnOnTurret();
                }
                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleOp, ShooterInterpolationConfig.getFarShoot()[0], ShooterInterpolationConfig.getFarShoot()[1]);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                KLog.d("TeleOp_Shooting", "Shoot action started force shoot from far - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

        if (forceShootNearPressed) {
            if (!isPending(shootAllAction)) {
                if (toggleTurretAlign) {
                    turnOnTurret();
                }
                shootAllAction = new ShootAllAction(turret, stopper, intake, shooter, driveBrake, shooterRun, turretAutoAlignTeleOp, ShooterInterpolationConfig.getNearValue()[0], ShooterInterpolationConfig.getNearValue()[1]);
                setLastShooterAction(shootAllAction);
                setLastStopperAction(null);  // Clear stopper - shoot action controls it
                KLog.d("TeleOp_Shooting", "Shoot action started force shoot from far - Target RPS: " + shootAllAction.getShooterRun().getTargetRPS());
            }
            return;
        }

    }

    private void handleZeroing() {
        if (zeroLimelightPressed) {
            if (!isPending(resetOdometryToLimelight)) {
                KLog.d("TeleOp_Zeroing", "Zero Limelight button pressed - resetting odometry to limelight position");
                resetOdometryToLimelight = new ResetOdometryToPosition(turret);
                setLastZeroAction(resetOdometryToLimelight);
                KLog.d("TeleOp_Zeroing", "Limelight zero action started");
            }
            return;
        }

        if (zeroCornerPressed) {
            if (!isPending(resetOdometryToCorner)) {
                KLog.d("TeleOp_Zeroing", "Zero Corner button pressed - resetting odometry to corner position");

                // TODO: Update with actual corner coordinates
                double cornerX = 600;
                double cornerY = 0 * allianceColor.getPolarity();
                double cornerTheta = 0;
                Position cornerPosition = new Position(cornerX, cornerY, cornerTheta);

                resetOdometryToCorner = new ResetOdometryToPos(cornerPosition);
                setLastZeroAction(resetOdometryToCorner);
                KLog.d("TeleOp_Zeroing", "Corner zero action started - Position: " + cornerPosition);
            }
            return;
        }
    }

}
