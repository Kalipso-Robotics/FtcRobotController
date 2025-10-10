package com.kalipsorobotics.decode;

import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeReverse;
import com.kalipsorobotics.actions.intake.IntakeRun;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.autoActions.shooterActions.KickBall;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.revolverActions.FullShootMotifAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.MotifColors;
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
    ShooterReady shooterReady = null;
    KickBall kickBall = null;
    Revolver revolver = null;

    IntakeRun intakeRun = null;
    IntakeStop intakeStop = null;
    IntakeFullAction intakeFullAction = null;

    FullShootMotifAction fullShootMotifAction = null;

    IntakeReverse intakeReverse = null;

    DriveAction driveAction = null;


    double turretStickValue;
    boolean shooterReadyPressed = false;
    boolean kickPressed = false;
    boolean intakePressed = false;
    boolean intakeReversePressed = false;
    boolean fullShootPressed = false;

    ObiliskDetection.MotifPattern testingMotif;

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
        shooter = new Shooter(opModeUtilities);
        revolver = Revolver.getInstance(opModeUtilities);

        intakeRun = new IntakeRun(intake);
        intakeStop = new IntakeStop(intake);
        intakeReverse = new IntakeReverse(intake);
        intakeFullAction = new IntakeFullAction(intake, revolver);

        //todo just fed in testing motif pattern change later
        testingMotif = new ObiliskDetection.MotifPattern(MotifColors.PURPLE, MotifColors.PURPLE, MotifColors.GREEN);
        fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif);

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


            if (shooterReadyPressed) {
                Log.d("ShooterReadyPressed", "Shooter Ready Pressed");
                if (shooterReady != null || shooterReady.getIsDone()) {
                    Log.d("ShooterReadyPressed", "Shooter Ready set");
                    shooterReady = new ShooterReady(shooter, Shooter.FAR_STARTING_POS_MM);
                    setLastShooterAction(shooterReady);
                }
            }

            if (fullShootPressed) {
                if (fullShootMotifAction != null || fullShootMotifAction.getIsDone()) {
                    fullShootMotifAction = new FullShootMotifAction(revolver, shooter, testingMotif);
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

            Log.d("Odometry", "Position: " + SharedData.getOdometryPosition());
            updateActions();
        }

        cleanupRobot();
    }
}
