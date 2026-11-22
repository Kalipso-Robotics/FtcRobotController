package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

@TeleOp
public class ShooterEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Shooter shooter = new Shooter(opModeUtilities);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                shooter.getShooterMotor().setPower(0.1);
            } else {
                shooter.getShooterMotor().setPower(0);
            }


            telemetry.addData("shooter", shooter.getShooterMotor().getCurrentPosition());

            telemetry.update();

        }
    }
}
