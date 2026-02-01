package com.kalipsorobotics.test;

import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodRapidIntervalTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        boolean goingUp = true;

        waitForStart();
        while (opModeIsActive()) {
            if (goingUp) {
                for (double i = ShooterInterpolationConfig.MIN_HOOD; i < ShooterInterpolationConfig.MAX_HOOD; i += 0.05) {
                    hood.setPosition(i);
                }
                goingUp = false;
            } else {
                for (double i = ShooterInterpolationConfig.MAX_HOOD; i > ShooterInterpolationConfig.MIN_HOOD; i -= 0.05) {
                    hood.setPosition(i);
                }
                goingUp = true;
            }
        }



    }
}
