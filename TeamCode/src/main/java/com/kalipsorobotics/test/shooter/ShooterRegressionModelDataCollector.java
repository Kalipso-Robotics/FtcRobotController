package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.shooter.ShootAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KGamePad;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

import java.security.PrivateKey;
import java.util.PriorityQueue;

public class ShooterRegressionModelDataCollector extends KTeleOp {
    KFileWriter kFileWriter = null;
    Shooter shooter = null;
    ShootAction shootAction = null;
    OpModeUtilities opModeUtilities = null;
    private double rps = 0;
    private double hoodPosition = 0;
    private double distance = 0;
    private boolean isIncrementRps = true;
    @Override
    public void initializeRobot() {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        kFileWriter = new KFileWriter("shooter_data", opModeUtilities); //log destination
        shooter = new Shooter(opModeUtilities);

        kFileWriter.writeLine("distance, rps, hood position");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {
            /*
                BUTTON A: LOG FILE
                BUTTON B: SHOOT

                BUTTON X: SWITCH CURRENT INCREMENT (RPS & HOOD POSITION)

                RIGHT BUMPER: increment current selected incrementation by 0.1
                RIGHT TRIGGER: increment current selected incrementation by -0.1

                LEFT BUMPER: increment current selected incrementation by 1
                LEFT TRIGGER: increment current selected incrementation by -1
             */
            shooter.getHood().setPosition(hoodPosition);
            shooter.goToRPS(rps);
            distance = shooter.getDistance(SharedData.getOdometryPosition(), Shooter.RED_TARGET_FROM_NEAR);
            if (kGamePad1.isButtonAFirstPressed()) { //log
                kFileWriter.writeLine(distance + "," + rps + "," + hoodPosition);
                KLog.d("Regression Module Data Collector", "data logged into file");
            }
            if (kGamePad1.isButtonBFirstPressed()) { //shoot
                if (shootAction != null || shootAction.getIsDone()) {
                    KLog.d("Regression Module Data Collector", "Shooter Ready set");
                    shootAction = new ShootAction(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
                    setLastShooterAction(shootAction);
                    KLog.d("Regression Module Data Collector", "ball shot");
                }
            }

            if (kGamePad1.isButtonXFirstPressed()) {
                if (isIncrementRps) {
                    isIncrementRps = false;
                } else {
                    isIncrementRps = true;
                }
            }
            if (kGamePad1.isRightBumperPressed()) {
                if (isIncrementRps) {
                    rps += 0.1;
                } else {
                    hoodPosition += 0.1;
                }
            }
            if (kGamePad1.isRightTriggerFirstPressed()) {
                if (isIncrementRps) {
                    rps -= 0.1;
                } else {
                    hoodPosition -= 0.1;
                }
            }

            if (kGamePad1.isLeftBumperPressed()) {
                if (isIncrementRps) {
                    rps += 1;
                } else {
                    hoodPosition += 1;
                }
            }
            if (kGamePad1.isLeftTriggerFirstPressed()) {
                if (isIncrementRps) {
                    rps -= 1;
                } else {
                    hoodPosition -= 1;
                }
            }

            if (isIncrementRps) {
                telemetry.addData("Current Increment: ", "RPS");
            } else {
                telemetry.addData("Current Increment: ", "Hood Position");
            }
            telemetry.addData("RPS: ", rps);
            telemetry.addData("Hood Position", hoodPosition);
            telemetry.update();
        }
    }
}
