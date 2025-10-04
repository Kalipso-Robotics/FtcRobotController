package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Shooter;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter", group = "Linear OpMode")
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {



        DcMotor turret = hardwareMap.dcMotor.get("turret");

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo revolver = hardwareMap.servo.get("revolver");
        double revolverPosition = 0.5;

        double power = 0;

        Servo kickerRight = hardwareMap.servo.get("kicker1");
        Servo kickerLeft = hardwareMap.servo.get("kicker2");
//        kickerRight.setPosition(0.5);
//        kickerLeft.setPosition(0.5);




        //Servo pusher = hardwareMap.servo.get("hood");
        double pusherPosition = 0.9;
        //pusher.setPosition(pusherPosition);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);

        KFileWriter shooterDataWriter = new KFileWriter("shooterData", opModeUtilities);

        Shooter shooter = new Shooter(opModeUtilities);

        waitForStart();

        while (opModeIsActive()) {
            shooterDataWriter.writeLine("Shooter RPS: " + shooter.getRPS() + " HoodPos: " + shooter.getHoodPosition());

            driveAction.move(gamepad2);


            if (gamepad1.left_trigger > 0.1) {
                pusherPosition += 0.001;
                shooter.getHood().setPosition(pusherPosition);
            } else if (gamepad1.right_trigger > 0.1) {
                pusherPosition -= 0.001;
                shooter.getHood().setPosition(pusherPosition);
            }


            if (gamepad1.dpad_up) {
                power += 0.0001;
            } else if (gamepad1.dpad_down) {
                power -= 0.0001;
            }

            if (gamepad1.dpad_left) {
                shooter.getShooter1().setPower(power);
                shooter.getShooter2().setPower(power);
            } else {
                shooter.getShooter1().setPower(0);
                shooter.getShooter2().setPower(0);
            }

            if (gamepad1.left_bumper) {
                turret.setPower(0.7);
            } else if (gamepad1.right_bumper) {
                turret.setPower(-0.7);
            } else {
                turret.setPower(0);
            }

            //increase kickerLeft to go up decrease to go down
            //decrease kickerRight to go up increase to go down

            if (gamepad1.x) {
                kickerRight.setPosition(0.5);
                kickerLeft.setPosition(0.75);
            } else if (gamepad1.b){
                kickerRight.setPosition(0.8);
                kickerLeft.setPosition(0.45);
            }


            if (gamepad1.right_stick_x != 0) {
                intake.setPower(gamepad1.right_stick_x);
            } else {
                intake.setPower(0);
            }


            if (gamepad1.left_stick_x > 0) {
                revolver.setPosition(revolverPosition);
                revolverPosition += 0.01;
            } else if (gamepad1.left_stick_x < 0) {
                revolver.setPosition(revolverPosition);
                revolverPosition -= 0.01;
            }


            //0.9219
            //0.64
            telemetry.addLine("Shooter RPS: " + shooter.getRPS() + "\nHoodPos: " + shooter.getHoodPosition());
            telemetry.addData("revolver position", revolverPosition);
//            telemetry.addData("kickerRight position", kickerPosition1);
//            telemetry.addData("kickerLeft position", kickerPosition2);
            telemetry.addData("power", power);
            telemetry.addData("pusher position", pusherPosition);
            telemetry.update();

        }

        shooterDataWriter.close();

    }

}
