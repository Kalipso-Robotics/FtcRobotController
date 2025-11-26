package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.shooter.ShootAllAction;
import com.kalipsorobotics.actions.shooter.pusher.PushBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ShooterDataCollector extends KOpMode {

    DriveTrain driveTrain;

    private Shooter shooter;
    private Stopper stopper;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;
    private Intake intake = null;
    private PushBall pushBall;
    private ShootAllAction shootAllAction = null;

    private double targetRPS = 0.0;
    private double hoodPosition = 0.5;
    private double prevRPS = -1.0;
    private double prevHoodPosition = -1.0;

    KServoAutoAction stopperOpen = null;
    KServoAutoAction stopperClose = null;

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        // Create odometry
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        // Initialize shooter module
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        intake = new Intake(opModeUtilities);
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

        stopperOpen = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_OPEN_POS);
        stopperClose = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);

        telemetry.addLine("ShooterDataCollector Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("Gamepad1 DPad Up/Down: Adjust RPS");
        telemetry.addLine("Gamepad1 DPad Left/Right: Revolver");
        telemetry.addLine("Gamepad1 Left/Right Trigger: Adjust Hood Position");
        telemetry.addLine("Gamepad1 Y: Kick Ball");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        while (opModeIsActive()) {
            // ========== RPS Control with DPad ==========
            // Use dpad up to increase RPS, dpad down to decrease RPS
            if (kGamePad1.isDpadUpFirstPressed()) {
                targetRPS += 0.1; // Increase by 0.5 RPS per loop
            } else if (kGamePad1.isDpadDownFirstPressed()) {
                targetRPS -= 0.1; // Decrease by 0.5 RPS per loop
            }

            if (kGamePad1.isButtonXFirstPressed()) {
                targetRPS += 2; // Increase by 0.5 RPS per loop
            } else if (kGamePad1.isButtonBFirstPressed()) {
                targetRPS -= 2; // Decrease by 0.5 RPS per loop
            }

            // Ensure targetRPS doesn't go negative
            if (targetRPS < 0) {
                targetRPS = 0;
            }

            // ========== Hood Position Control with Triggers ==========
            // Use left trigger to increase hood position, right trigger to decrease
            if (kGamePad1.isLeftTriggerFirstPressed()) {
                hoodPosition += 0.01;
            } else if (kGamePad1.isRightTriggerFirstPressed()) {
                hoodPosition -= 0.01;
            }

            if (kGamePad1.isLeftBumperFirstPressed()) {
                hoodPosition += 0.1;
            } else if (kGamePad1.isRightBumperFirstPressed()) {
                hoodPosition -= 0.1;
            }

            // Clamp hood position between 0 and 1
            if (hoodPosition > 1.0) {
                hoodPosition = 1.0;
            } else if (hoodPosition < 0.0) {
                hoodPosition = 0.0;
            }

            // ========== ShooterReady Action Management ==========
            // Create a new ShooterReady action when RPS or hood position changes
            if (gamepad1.left_stick_button) {
                if (true) {
                    if (targetRPS > 0) {
                        // Use ShooterReady with direct RPS mode (same as RedFarTeleOp/RedAutos pattern)
                        shootAllAction = new ShootAllAction(stopper, intake, shooter, targetRPS, hoodPosition);
                        setLastShooterAction(shootAllAction);
                    } else {
                        // Stop the shooter when targetRPS is 0
                        shooter.stop();
                    }
                    prevRPS = targetRPS;
                    prevHoodPosition = hoodPosition;
                }
            }


            // ========== Kick Ball with Gamepad1 Y ==========
//            if (kGamePad1.isButtonYFirstPressed()) {
//                if (pushBall == null || pushBall.getIsDone()) {
//                    pushBall = new PushBall(stopper, intake, shooter);
//                    setLastStopperAction(pushBall);
//                    setLastIntakeAction(pushBall);
//                    setLastShooterAction(pushBall);
//                }
//            }
//            if (gamepad1.y) {
//                // Create a new KickBall action if one doesn't exist or has completed
//                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_OPEN_POS);
//            } else {
//                stopper.getStopper().setPosition(stopper.STOPPER_SERVO_CLOSED_POS);
//            }

            if (gamepad1.right_stick_y != 0) {
                intake.getIntakeMotor().setPower(-gamepad1.right_stick_y);
                stopper.getStopper().setPosition(0.55);
            } else if (gamepad1.right_stick_button) {
                intake.getIntakeMotor().setPower(0);
            }

            if (gamepad1.a) {
                targetRPS = 0;
            }

            // ========== Telemetry ==========
            telemetry.addLine("=== Shooter Data Collector ===");
            telemetry.addData("Target RPS", String.format("%.2f", targetRPS));
            telemetry.addData("Actual RPS", String.format("%.2f", shooter.getRPS()));
            telemetry.addData("Hood Position", String.format("%.3f", hoodPosition));
            telemetry.addData("Actual Hood Pos", String.format("%.3f", shooter.getHoodPosition()));
            telemetry.addLine();
            telemetry.addLine("Distance: " + shooter.getDistance(SharedData.getOdometryPosition(), Shooter.TARGET_POINT));
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("GP1 DPad Up/Down: Adjust RPS");
            telemetry.addLine("GP1 DPad Left/Right: Revolver");
            telemetry.addLine("GP1 L/R Trigger: Adjust Hood (0.01)");
            telemetry.addLine("GP1 L/R Bumper: Adjust Hood (0.1)");
            telemetry.addLine("GP1 Y: Kick Ball");
            telemetry.update();

            // Update all actions
            turretAutoAlign.updateCheckDone();
            updateActions();
        }

        // Cleanup
        shooter.stop();
        cleanupRobot();
    }
}