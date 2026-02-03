package com.kalipsorobotics.test;

import static com.kalipsorobotics.decode.configs.ShooterConfig.hoodFlipDirection;

import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class HoodServoToZero extends LinearOpMode {

    private double hoodPos = ShooterInterpolationConfig.MIN_HOOD;
    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        KServo khood = new KServo(hood, KServo.AXON_MAX_SPEED, 255, hoodPos, hoodFlipDirection);
        hood.setPosition(hoodPos);

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.dpad_down) {
                hoodPos+=0.0001;
            } else if (gamepad1.dpad_up) {
                hoodPos-=0.0001;
            }
            khood.setPosition(hoodPos);
            telemetry.addData("HOOD POS ", "Hood Pos: " + hoodPos);
            telemetry.addData("REAL HOOD POS ", "REAL hood: " + khood.getPosition());
            telemetry.update();
            // 0.03 lowest
            // 0.605 highest
        }
    }
}
