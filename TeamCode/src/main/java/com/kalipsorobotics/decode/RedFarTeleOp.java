package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.actions.RunUntilStallAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
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
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "RedFarTeleop")
public class RedFarTeleOp extends KTeleOp {
    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_FAR;
    private DriveTrain driveTrain;
    private Shooter shooter = null;
    private Intake intake = null;
    Turret turret = null;
    private Stopper stopper = null;

    ShooterRun shooterRun = null;
    ShooterStop shooterStop = null;
    ShootAllAction shootAction = null;

    LaunchPosition launchPosition = LaunchPosition.AUTO;
    PushBall pushBall = null;

    KServoAutoAction openStopper = null;
    KServoAutoAction closeStopper = null;


    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeFullAction intakeFullAction = null;

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
        stopper = new Stopper(opModeUtilities);

        turret = Turret.getInstance(opModeUtilities);

        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(stopper, intake, 8);

        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.RED_X_INIT_SETUP, TurretConfig.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        //goalDetectionAction = new GoalDetectionAction(opModeUtilities);

        //todo just fed in testing motif pattern change later
//        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);

        shooterRun = new ShooterRun(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO);
        shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_FAR);
        shooterStop = new ShooterStop(shooterRun);
        pushBall = new PushBall(stopper, intake, shooter);
        openStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
        closeStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
        runUntilStallAction = new RunUntilStallAction(intake.getIntakeMotor(), 1, 3000);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        //darren cant digest cheese
        waitForStart();
        while (opModeIsActive()) {

            if(kGamePad1.getLeftStickY() != 0 || kGamePad1.getRightStickX() != 0 || kGamePad1.getLeftStickX() != 0) {
                setLastMoveAction(null);
                driveAction.move(gamepad1);
            } else {
                if (lastMoveAction == null || lastMoveAction.getIsDone()) {
                    driveTrain.setPower(0);
                }
            }

            shootActionPressed = kGamePad2.isLeftTriggerPressed();
            turretStickValue = kGamePad2.getRightStickX(); //unused
            releasePressed = kGamePad2.isYPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();

            shooterWarmupPressed = kGamePad2.isDpadUpFirstPressed();

            useAprilTagPressed = kGamePad2.isBackButtonPressed();

            runUntilStalledPressed = kGamePad2.isButtonXFirstPressed();


            if (useAprilTagPressed) {
                useAprilTag = !useAprilTag;
            }

            if (shooterStopPressed) {
                if (shooterStop == null || shooterStop.getIsDone()) {
                    shooterStop = new ShooterStop(shooterRun);
                    setLastShooterAction(shooterStop);
                }
            }


            if (shooterWarmupPressed) {
                if (useAprilTag) {
                    goalDetectionAction.getLimelight().start();
                    goalDetectionAction.updateCheckDone();
                }
                if (shooterRun == null || shooterRun.getIsDone()) {
                    shooterRun = new ShooterRun(shooter, 20, 0.8);
                    KLog.d("ShooterReadyPressed", "Shooter Ready set Warming Up For Position: " + launchPosition);
                    setLastShooterAction(shooterRun);
                }
            } else {
                //goalDetectionAction.getLimelight().stop();
            }


            if (shootActionPressed) {
                KLog.d("ShooterReadyPressed", "Shooter Ready Pressed");
                if (shootAction == null || shootAction.getIsDone()) {
                    KLog.d("ShooterReadyPressed", "Shooter Ready set");
                    shootAction = new ShootAllAction(stopper, intake, shooter, ROBOT_START_POINT_RED);
                    setLastShooterAction(shootAction);
                    setLastStopperAction(shootAction);
                }
            }

            if (shootAction != null && !shootAction.getIsDone()) {
                turretAutoAlign.updateCheckDone();
            }


            if (intakePressed) {
                if (intakeRun == null || intakeRun.getIsDone()) {
                    intakeRun = new IntakeRun(intake);
                    closeStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
                    setLastStopperAction(closeStopper);
                    setLastIntakeAction(intakeRun);

                }
            } else if (intakeReversePressed) {
                if (intakeReverse == null || intakeReverse.getIsDone()) {
                    intakeReverse = new IntakeReverse(intake);
                    setLastIntakeAction(intakeReverse);
                }
            } else if (!shootActionPressed){
                if (intakeStop == null || intakeStop.getIsDone()) {
                    intakeStop = new IntakeStop(intake);
                    setLastIntakeAction(intakeStop);
                }
            }


            if (releasePressed) {
                if (openStopper == null || openStopper.getIsDone()) {
                    openStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
                    setLastStopperAction(openStopper);
                }
            }


            //mostly for 3 ball
            if (runUntilStalledPressed) {
                KLog.d("teleop", "revolver pressed");
                if (runUntilStallAction == null || runUntilStallAction.getIsDone()) {
                    runUntilStallAction = new RunUntilStallAction(intake.getIntakeMotor(), 1, 4000);
                }
                setLastIntakeAction(runUntilStallAction);
            }


            if (useAprilTag) {
                goalDetectionAction.updateCheckDone();
            }

            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
            updateActions();
        }

        //goalDetectionAction.getLimelight().close();
        cleanupRobot();
    }
}
