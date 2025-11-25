package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.actions.RunUntilStallAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRunFullSpeed;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Red Teleop")
public class RedFarTeleOp extends KTeleOp {
    protected AllianceSetup allianceSetup = AllianceSetup.RED;
    protected final Point SHOOTER_TARGET_POINT = Shooter.TARGET_POINT;
    private DriveTrain driveTrain;
    private Shooter shooter = null;
    private Intake intake = null;
    Turret turret = null;
    private Stopper stopper = null;

    ShooterRun shooterWarmup = null;
    ShooterStop shooterStop = null;
    ShootAllAction shootAction = null;
    PushBall pushBall = null;

    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;


    IntakeRunFullSpeed intakeRunFullSpeed = null;
    IntakeStop intakeStop = null;

    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;

    TurretAutoAlign turretAutoAlign = null;

    GoalDetectionAction goalDetectionAction = null;

    RunUntilStallAction runUntilStallAction = null;

    double turretStickValue;
    boolean shootActionPressed = false;
    boolean releasePressed = false;
    boolean intakePressed = false;
    boolean intakeReversePressed = false;
    boolean shooterWarmupPressed = false;
    private boolean shooterStopPressed = false;
    boolean useAprilTagPressed = false;
    boolean useAprilTag = false;
    boolean runUntilStalledPressed = false;


    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // Create your modules
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        driveAction = new DriveAction(driveTrain);

        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);

        turret = Turret.getInstance(opModeUtilities);

        intakeRunFullSpeed = null;
        intakeStop = null;
        intakeReverse = null;

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.X_INIT_SETUP, TurretConfig.Y_INIT_SETUP * allianceSetup.getPolarity());

        //goalDetectionAction = new GoalDetectionAction(opModeUtilities);

        //todo just fed in testing motif pattern change later
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);

        shooterWarmup = null;
        shootAction = null;
        shooterStop = null;
        pushBall = null;
        openStopper = null;
        closeStopper = null;
        runUntilStallAction = null;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //darren cant digest cheese
        waitForStart();
        while (opModeIsActive()) {

            if(kGamePad1.getLeftStickY() != 0 || kGamePad1.getRightStickX() != 0 || kGamePad1.getLeftStickX() != 0) {
                driveAction.move(gamepad1);
            } else {
                driveTrain.setPower(0);
            }

            shootActionPressed = kGamePad2.isLeftTriggerPressed();
            turretStickValue = kGamePad2.getRightStickX(); //unused
            releasePressed = kGamePad2.isYPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();

            shooterWarmupPressed = kGamePad2.isDpadUpFirstPressed();
            shooterStopPressed =  kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed();

            useAprilTagPressed = kGamePad2.isBackButtonPressed();

            runUntilStalledPressed = kGamePad2.isButtonXFirstPressed();


            if (useAprilTagPressed) {
                useAprilTag = !useAprilTag;
            }


            if (intakePressed) {
                if (intakeRunFullSpeed == null || intakeRunFullSpeed.getIsDone()) {
                    intakeRunFullSpeed = new IntakeRunFullSpeed(intake, stopper);
                    addPendingAction(intakeRunFullSpeed);
                }
            } else if (intakeReversePressed) {
                if (intakeReverse == null || intakeReverse.getIsDone()) {
                    intakeReverse = new IntakeReverse(intake);
                    addPendingAction(intakeReverse);
                }
            } else {
                if (intakeStop == null || intakeStop.getIsDone()) {
                    intakeStop = new IntakeStop(intake);
                    addPendingAction(intakeStop);
                }
            }

            if (useAprilTag) {
                goalDetectionAction.updateCheckDone();
            }

            if (shooterStopPressed) {
                if (shooterStop == null || shooterStop.getIsDone()) {
                    shooterStop = new ShooterStop(shooterWarmup, shootAction);
                    addPendingAction(shooterStop);
                }
            }

            if (shooterWarmupPressed) {
                if (useAprilTag) {
                    goalDetectionAction.getLimelight().start();
                    goalDetectionAction.updateCheckDone();
                }
                if (shooterWarmup == null || shooterWarmup.getIsDone()) {
                    shooterWarmup = new ShooterRun(shooter, 20, 0.8);
                    KLog.d("ShooterReadyPressed", "Shooter Ready set Warming Up");
                    addPendingAction(shooterWarmup);
                }
            } else {
                //goalDetectionAction.getLimelight().stop();
            }

            if ((shootAction != null && !shootAction.getIsDone()) || (shooterWarmup != null && !shooterWarmup.getIsDone())) {
                turretAutoAlign.updateCheckDone();
            }

            if (shootActionPressed) {
                KLog.d("ShooterReadyPressed", "Shooter Ready Pressed");
                if (shootAction == null || shootAction.getIsDone()) {
                    KLog.d("ShooterReadyPressed", "Shooter Ready set");
                    shootAction = new ShootAllAction(stopper, intake, shooter, SHOOTER_TARGET_POINT.multiplyY(allianceSetup.getPolarity()));
                    KLog.d("ShooterReadyPressed", "Target RPS : " + shootAction.getShooterRun().getTargetRPS());
                    addPendingAction(shootAction);
                }
            }

            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
            updateActionsToBeExecuted();
        }

        //goalDetectionAction.getLimelight().close();
        cleanupRobot();
    }

    private void updateActionsToBeExecuted() {
        //shooter
        if (isPending(shooterStop)) {
            actionsToBeExecuted.add(shooterStop);
        } else if (isPending(shootAction)) {
            actionsToBeExecuted.add(shootAction);
        } else if (isPending(shooterWarmup)) {
            actionsToBeExecuted.add(shooterWarmup);
        }

        //intake
        if (!isPending(shootAction)) {
            if (isPending(intakeRunFullSpeed)) {
                actionsToBeExecuted.add(intakeRunFullSpeed);
            } else if (isPending(intakeReverse)) {
                actionsToBeExecuted.add(intakeReverse);
            } else if (isPending(intakeStop)) {
                actionsToBeExecuted.add(intakeStop);
            }
        }

        pendingActions.clear();
        super.updateActions();
        actionsToBeExecuted.clear();
    }


}
