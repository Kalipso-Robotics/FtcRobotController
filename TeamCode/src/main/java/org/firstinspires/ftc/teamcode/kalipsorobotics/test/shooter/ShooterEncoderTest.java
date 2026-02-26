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
                shooter.getShooter1().setPower(0.1);
            } else {
                shooter.getShooter1().setPower(0);
            }

            if (gamepad1.left_bumper) {
                shooter.getShooter2().setPower(0.1);
            } else {
                shooter.getShooter2().setPower(0);
            }


            telemetry.addData("shooter1", shooter.getShooter1().getCurrentPosition());
            telemetry.addData("shooter2", shooter.getShooter2().getCurrentPosition());

            telemetry.update();

        }
    }
}
