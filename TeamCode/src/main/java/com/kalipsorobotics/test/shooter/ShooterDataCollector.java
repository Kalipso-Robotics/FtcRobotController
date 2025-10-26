package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.revolverActions.RevolverTeleOp;
import com.kalipsorobotics.actions.shooter.KickBall;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ShooterDataCollector", group = "Test")
public class ShooterDataCollector extends KTeleOp {

    DriveTrain driveTrain;

    private Shooter shooter;
    private Revolver revolver;
    Turret turret = null;
    TurretAutoAlign turretAutoAlign = null;

    private KickBall kickBall;
    private RevolverTeleOp revolverTeleOp;

    private double targetRPS = 0.0;
    private double hoodPosition = 0.5;

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

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(turret, TurretAutoAlign.RED_X_INIT_SETUP, TurretAutoAlign.RED_Y_INIT_SETUP);


        // Initialize revolver module
        revolver = new Revolver(opModeUtilities);
        revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);

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
                targetRPS += 0.5; // Increase by 0.5 RPS per loop
            } else if (kGamePad1.isDpadDownFirstPressed()) {
                targetRPS -= 0.5; // Decrease by 0.5 RPS per loop
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

            // Set the shooter to target RPS
            if (targetRPS > 0) {
                shooter.goToRPS(targetRPS);
            } else {
                shooter.stop();
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

            // Set hood position
            shooter.getHood().setPosition(hoodPosition);

            // ========== Revolver Control with DPad Left/Right ==========
            if (kGamePad1.isDpadLeftFirstPressed()) {
                if (revolverTeleOp == null || revolverTeleOp.getIsDone()) {
                    revolverTeleOp = new RevolverTeleOp(revolver, true);
                    setLastRevolverAction(revolverTeleOp);
                }
            } else if (kGamePad1.isDpadRightFirstPressed()) {
                if (revolverTeleOp == null || revolverTeleOp.getIsDone()) {
                    revolverTeleOp = new RevolverTeleOp(revolver, false);
                    setLastRevolverAction(revolverTeleOp);
                }
            }

            // ========== Kick Ball with Gamepad1 Y ==========
            if (gamepad1.y) {
                // Create a new KickBall action if one doesn't exist or has completed
                if (kickBall == null || kickBall.getIsDone()) {
                    kickBall = new KickBall(shooter);
                    setLastKickerAction(kickBall);
                }
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
            telemetry.addLine("Distance: " + shooter.getDistance(SharedData.getOdometryPosition(), Shooter.RED_TARGET_FROM_NEAR));
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