package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeFullAction;
import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeReverse;
import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeRun;
import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeStop;
import com.kalipsorobotics.actions.autoActions.shooterActions.KickBall;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends KTeleOp {
    private DriveTrain driveTrain;
    private  Odometry odometry;

    Shooter shooter = null;
    Intake intake = null;
    Revolver revolver = null;
    ShooterReady shooterReady = null;
    KickBall kickBall = null;

    IntakeRun intakeRun = null;
    IntakeFullAction intakeFullAction = null;
    IntakeStop intakeStop = null;

    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;



    boolean shooterReadyPressed = false;
    boolean kickPressed = false;
    boolean intakePressed = false;
    boolean intakeReversePressed = false;

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // Create your modules
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        driveAction = new DriveAction(driveTrain);

        intake = new Intake(opModeUtilities);
        revolver = Revolver.getInstance(opModeUtilities);
        shooter = new Shooter(opModeUtilities);

        intakeFullAction = new IntakeFullAction(intake, revolver);
        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);

        shooterReady = new ShooterReady(shooter, ShooterReady.FAR_LAUNCH_POINT);
        kickBall = new KickBall(shooter);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
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

            shooterReadyPressed = kGamePad2.isLeftTriggerPressed();
            kickPressed = kGamePad2.isButtonYFirstPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();
            intakeReversePressed = kGamePad2.isRightBumperPressed();


            if (shooterReadyPressed) {
                if (shooterReady != null || shooterReady.getIsDone()) {
                    shooterReady = new ShooterReady(shooter, ShooterReady.FAR_LAUNCH_POINT);
                    setLastShooterAction(shooterReady);
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
                    intakeFullAction = new IntakeFullAction(intake, revolver);
                    setLastIntakeAction(intakeRun);
                }
            } else if (intakeReversePressed) {
                if (intakeReverse != null || intakeReverse.getIsDone()) {
                    intakeReverse = new IntakeReverse(intake);
                    setLastIntakeAction(intakeReverse);
                }
            } else {
                if (intakeFullAction != null) {
                    intakeFullAction.setIsDone(true);
                }
                if (intakeStop != null || intakeStop.getIsDone()) {
                    intakeStop = new IntakeStop(intake);
                    setLastIntakeAction(intakeStop);
                }
            }





            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
            updateActions();
        }

        cleanupRobot();
    }
}
