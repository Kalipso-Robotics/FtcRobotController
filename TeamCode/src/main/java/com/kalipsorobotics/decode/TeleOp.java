package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeRun;
import com.kalipsorobotics.actions.autoActions.intakeActions.IntakeStop;
import com.kalipsorobotics.actions.autoActions.shooterActions.KickBall;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
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
    ShooterReady shooterReady = null;
    KickBall kickBall = null;

    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;



    boolean shooterReadyPressed = false;
    boolean kickPressed = false;
    boolean intakePressed = false;

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
        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);

        kickBall = new KickBall(shooter);
        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        shooterReady = new ShooterReady(shooter, ShooterReady.FAR_LAUNCH_POINT);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {
            shooterReadyPressed = kGamePad2.isLeftTriggerPressed();
            kickPressed = kGamePad2.isButtonYFirstPressed();
            intakePressed = kGamePad2.isRightTriggerPressed();

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
                if (intakeRun != null || intakeRun.getIsDone()) {
                    intakeRun = new IntakeRun(intake);
                    setLastIntakeAction(intakeRun);
                }
            } else {
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
