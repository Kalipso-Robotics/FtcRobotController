package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Bring-up step 1: drive motor wiring and encoders. Hold a button to spin ONE motor forward at TEST_POWER and check it is
 * the right wheel, turning forward, and its encoder counts up.
 * GP1: A = fLeft, B = fRight, X = bLeft, Y = bRight.
 */
@TeleOp(name = "HardwareTest", group = "Test")
public class HardwareTest extends LinearOpMode {
    public static final String TAG = "Test_Hardware";
    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        DcMotor fRight = hardwareMap.get(DcMotor.class, "fRight");
        DcMotor bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        DcMotor bRight = hardwareMap.get(DcMotor.class, "bRight");
        String[] names = {"fLeft", "fRight", "bLeft", "bRight"};
        DcMotor[] motors = {fLeft, fRight, bLeft, bRight};

        telemetry.addLine("Hold A=fLeft B=fRight X=bLeft Y=bRight to spin one motor at " + TEST_POWER);
        telemetry.update();
        waitForStart();
        String lastSpinning = "none";
        while (opModeIsActive()) {
            boolean[] held = {gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y};
            String spinning = "none";
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(held[i] ? TEST_POWER : 0);
                if (held[i]) {
                    spinning = names[i];
                }
            }
            if (!spinning.equals(lastSpinning)) {
                String s = spinning;
                KLog.d(TAG, () -> "event=spin motor=" + s);
                lastSpinning = spinning;
            }
            telemetry.addLine("Hold A=fLeft B=fRight X=bLeft Y=bRight (power " + TEST_POWER + ")");
            telemetry.addData("spinning", spinning);
            for (int i = 0; i < motors.length; i++) {
                telemetry.addData(names[i] + " ticks", motors[i].getCurrentPosition());
            }
            telemetry.update();
        }
        for (DcMotor m : motors) {
            m.setPower(0);
        }
    }
}
