package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.FullShootMotifAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.KickBall;
import com.kalipsorobotics.actions.shooter.ShootAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RedAutoNear extends KTeleOp {
    protected AllianceSetup allianceSetup = AllianceSetup.RED;

    protected final Point ROBOT_START_POINT_RED = Shooter.RED_TARGET_FROM_NEAR;
    private DriveTrain driveTrain;

    TripleColorSensor colorSensors = null;
    Shooter shooter = null;
    Intake intake = null;
    ShooterReady shooterReady = null;
    ShooterStop shooterStop = null;
    ShootAction shootAction = null;

    LaunchPosition launchPosition = LaunchPosition.AUTO;
    KickBall kickBall = null;
    Revolver revolver = null;
    Turret turret = null;

    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeFullAction intakeFullAction = null;

    RevolverTeleOp revolverTeleOp = null;

    FullShootMotifAction fullShootMotifAction = null;

    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;

    TurretAutoAlign turretAutoAlign = null;

    DetectColorsAction detectColorsAction = null;

    double turretStickValue;
    boolean shootActionPressed = false;
    boolean kickPressed = false;
    boolean intakePressed = false;
    boolean intakeReversePressed = false;
    boolean fullShootPressed = false;
    boolean revolverLeftPressed = false;
    boolean revolverRightPressed = false;
    boolean shooterReadyNearPressed = false;
    boolean shooterReadyMiddlePressed = false;
    boolean shooterReadyWallPressed = false;
    boolean shooterReadyBluePressed = false;
    boolean shooterReadyRedPressed = false;
    boolean shooterReadyRedMiddlePressed = false;
    boolean shooterReadyBlueMiddlePressed = false;
    private boolean shooterStopPressed = false;



    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        allianceSetup = AllianceSetup.RED;

        // Create your modules
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        driveAction = new DriveAction(driveTrain);

        colorSensors = new TripleColorSensor(opModeUtilities);

        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        revolver = new Revolver(opModeUtilities);
        revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(intake, revolver, colorSensors);

        revolverTeleOp = new RevolverTeleOp(revolver, false);

        turretAutoAlign = new TurretAutoAlign(turret, TurretAutoAlign.RED_X_INIT_SETUP, TurretAutoAlign.RED_Y_INIT_SETUP * allianceSetup.getPolarity());

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shootAction = new ShootAction(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        shooterStop = new ShooterStop(shooter);
        kickBall = new KickBall(shooter);
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

            turretStickValue = kGamePad2.getRightStickX();
            shootActionPressed = kGamePad2.isLeftTriggerPressed();
            kickPressed = kGamePad2.isButtonYFirstPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed() && !kGamePad2.isLeftBumperPressed();
            fullShootPressed = kGamePad2.isButtonAFirstPressed();
            revolverLeftPressed = kGamePad2.isButtonXFirstPressed();
            revolverRightPressed = kGamePad2.isButtonBFirstPressed();

            shooterReadyNearPressed = kGamePad2.isDpadUpFirstPressed();
            shooterReadyMiddlePressed = kGamePad2.isDpadDownFirstPressed();
            shooterReadyWallPressed = kGamePad2.isDpadUpFirstPressed() && kGamePad2.isLeftBumperPressed();
            shooterReadyRedPressed = kGamePad2.isDpadRightFirstPressed();
            shooterReadyBluePressed = kGamePad2.isDpadLeftFirstPressed();
            shooterReadyRedMiddlePressed = kGamePad2.isLeftBumperPressed() && kGamePad2.isDpadRightFirstPressed();
            shooterReadyBlueMiddlePressed = kGamePad2.isLeftBumperPressed() && kGamePad2.isDpadLeftFirstPressed();
            shooterStopPressed = kGamePad2.isLeftBumperPressed() && kGamePad2.isRightBumperPressed();

            boolean isWarmup = true;


            if (shooterStopPressed) {
                if (shooterStop != null || shooterStop.getIsDone()) {
                    shooterStop = new ShooterStop(shooter);
                    setLastShooterAction(shooterStop);
                }
            }

            if (shooterReadyNearPressed) {
                launchPosition = LaunchPosition.NEAR;
            } else if (shooterReadyMiddlePressed) {
                launchPosition = LaunchPosition.MIDDLE;
            } else if (shooterReadyWallPressed) {
                launchPosition = LaunchPosition.WALL;
            } else if (shooterReadyRedPressed) {
                launchPosition = LaunchPosition.RED;
            } else if (shooterReadyBluePressed) {
                launchPosition = LaunchPosition.BLUE;
            } else if (shooterReadyRedMiddlePressed) {
                launchPosition = LaunchPosition.MIDDLE_RED;
            } else if (shooterReadyBlueMiddlePressed) {
                launchPosition = LaunchPosition.MIDDLE_BLUE;
            } else {
                isWarmup = false;
                launchPosition = LaunchPosition.AUTO;
            }

            if (isWarmup) {
                if (shooterReady != null || shooterReady.getIsDone()) {
                    shooterReady = new ShooterReady(shooter, ROBOT_START_POINT_RED.multiplyY(allianceSetup.getPolarity()), launchPosition);
                    KLog.d("ShooterReadyPressed", "Shooter Ready set Warming Up For Position: " + launchPosition);
                    setLastShooterAction(shooterReady);
                }
            }


            if (shootActionPressed) {
                KLog.d("ShooterReadyPressed", "Shooter Ready Pressed");
                if (shootAction != null || shootAction.getIsDone()) {
                    KLog.d("ShooterReadyPressed", "Shooter Ready set");
                    shootAction = new ShootAction(shooter, ROBOT_START_POINT_RED, launchPosition);
                    setLastShooterAction(shootAction);
                }
            }



            if (fullShootPressed) {
                if (fullShootMotifAction != null || fullShootMotifAction.getIsDone()) {
                    fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);
                }
            }

            if (kickPressed) {
                if (kickBall != null || kickBall.getIsDone()) {
                    kickBall = new KickBall(shooter);
                    setLastKickerAction(kickBall);
                }
            }

            if (intakePressed) {
                if (intakeFullAction != null || intakeFullAction.getIsDone()) {
                    intakeFullAction = new IntakeFullAction(intake, revolver, colorSensors);
                    setLastIntakeAction(intakeFullAction);
                }
            } else if (intakeReversePressed) {
                if (intakeReverse != null || intakeReverse.getIsDone()) {
                    intakeReverse = new IntakeReverse(intake);
                    setLastIntakeAction(intakeReverse);
                }
            } else {
                intakeFullAction.setIsDone(true);
                if (intakeStop != null || intakeStop.getIsDone()) {
                    intakeStop = new IntakeStop(intake);
                    setLastIntakeAction(intakeStop);
                }
            }

            if (revolverLeftPressed) {
                KLog.d("teleop", "revolver pressed");
                if (revolverTeleOp != null || revolverTeleOp.getIsDone()) {
                    KLog.d("teleop", "revolverteleop reset");
                    revolverTeleOp = new RevolverTeleOp(revolver, true);
                }
                setLastRevolverAction(revolverTeleOp);
            } else if (revolverRightPressed) {
                KLog.d("teleop", "revolver pressed");
                if (revolverTeleOp != null || revolverTeleOp.getIsDone()) {
                    KLog.d("teleop", "revolverteleop reset");
                    revolverTeleOp = new RevolverTeleOp(revolver, false);
                }
                setLastRevolverAction(revolverTeleOp);
            }





            turretAutoAlign.updateCheckDone();
            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
            updateActions();
        }

        cleanupRobot();
    }
}
