package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.autoActions.shooterActions.KickBall;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.revolverActions.DetectColorsAction;
import com.kalipsorobotics.actions.revolverActions.FullShootMotifAction;
import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.ColorCalibration;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends KTeleOp {
    private DriveTrain driveTrain;

    TripleColorSensor colorSensors = null;
    Shooter shooter = null;
    Intake intake = null;
    ShooterReady shooterReady = null;
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
    boolean shooterReadyPressed = false;
    boolean kickPressed = false;
    boolean intakePressed = false;
    boolean intakeReversePressed = false;
    boolean fullShootPressed = false;
    boolean revolverLeftPressed = false;
    boolean revolverRightPressed = false;

    ObiliskDetection.MotifPattern testingMotif;

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

        colorSensors = new TripleColorSensor(opModeUtilities);

        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        revolver = new Revolver(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(intake, revolver, colorSensors);

        revolverTeleOp = new RevolverTeleOp(revolver, false);

        turretAutoAlign = new TurretAutoAlign(turret);

        detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        //todo just fed in testing motif pattern change later
        testingMotif = new ObiliskDetection.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif, colorSensors, opModeUtilities);

        shooterReady = new ShooterReady(shooter, Shooter.FAR_STARTING_POS_MM);
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
            shooterReadyPressed = kGamePad2.isLeftTriggerPressed();
            kickPressed = kGamePad2.isButtonYFirstPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed();
            fullShootPressed = kGamePad2.isButtonAFirstPressed();
            revolverLeftPressed = kGamePad2.isButtonXFirstPressed();
            revolverRightPressed = kGamePad2.isButtonBFirstPressed();

            if (shooterReadyPressed) {
                KLog.d("ShooterReadyPressed", "Shooter Ready Pressed");
                if (shooterReady != null || shooterReady.getIsDone()) {
                    KLog.d("ShooterReadyPressed", "Shooter Ready set");
                    shooterReady = new ShooterReady(shooter, Shooter.FAR_STARTING_POS_MM);
                    setLastShooterAction(shooterReady);
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
