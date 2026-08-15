package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;


@TeleOp(name="8 Motor, 12 Servo & Sensor Test", group="Test")
public class TesterOpMode extends KOpMode {

    private DriveTrain driveTrain;

    // other 4 motors
    private DcMotor motorOne = null;
    private DcMotor motorTwo = null;
    private DcMotor motorThree = null;
    private DcMotor motorFour = null;

    // motor toggle states
    private boolean isMotorOneSelected = true; // true = motor 1, false = motor 2
    private boolean isMotorThreeSelected = true; // true = motor 3, false = motor 4

    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // servos
    private final Servo[] servos = new Servo[12];
    private final String[] servoNames = new String[12];
    private int selectedServoIndex = 0;

    private boolean lastY = false;
    private boolean lastA = false;

    // servo selection
    private boolean lastB = false;
    private boolean lastX = false;

    // sensors
    private TouchSensor touchSensor = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;

    @Override
    protected void initializeRobotConfig() {
        // Required by KOpMode
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        // ==============================================================
        // HARDWARE MAP INITIALIZATION



        driveTrain = DriveTrain.getInstance(opModeUtilities);

        // other 4 motors
        motorOne = hardwareMap.get(DcMotor.class, "motorOne");
        motorTwo = hardwareMap.get(DcMotor.class, "motorTwo");
        motorThree = hardwareMap.get(DcMotor.class, "motorThree");
        motorFour = hardwareMap.get(DcMotor.class, "motorFour");

        // servos 1 - 12
        for (int i = 0; i < 12; i++) {
            servos[i] = hardwareMap.get(Servo.class, "servo" + (i + 1));
            servoNames[i] = "Servo " + (i + 1); // Default placeholder name
        }

        // ==============================================================
        // EDIT HERE LATER
        // ==============================================================
        // servoNames[0] = "Slide Servo";
        // servoNames[1] = "Claw Servo";
        // servoNames[2] = "Intake Servo";
        // etc (idk the names yet)
        // ==============================================================

        // sensors
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color_distance");

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // runs initializeRobot() first
        initializeRobot();

        telemetry.addData("Status", "Initialized - Motors, Servos & Sensors Ready");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

            // ==============================================================
            // MECANUM DRIVE CONTROL

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;


            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            driveTrain.setPower(
                    leftFrontPower / 2.0,
                    rightFrontPower / 2.0,
                    leftBackPower / 2.0,
                    rightBackPower / 2.0
            );

            // ==============================================================
            // AUXILIARY MOTORS

            // right bumper selecting motor 1 or 2
            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumper) {
                isMotorOneSelected = !isMotorOneSelected;
            }
            lastRightBumper = currentRightBumper;

            // apply right trigger to selected motor
            double rightTriggerPower = gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger * 0.5 : 0.0;
            if (isMotorOneSelected) {
                motorOne.setPower(rightTriggerPower);
                motorTwo.setPower(0.0);
            } else {
                motorOne.setPower(0.0);
                motorTwo.setPower(rightTriggerPower);
            }

            // right bumper selecting motor 3 or 4
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                isMotorThreeSelected = !isMotorThreeSelected;
            }
            lastLeftBumper = currentLeftBumper;

            // apply left trigger to selected motor
            double leftTriggerPower = gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger * 0.5 : 0.0;
            if (isMotorThreeSelected) {
                motorThree.setPower(leftTriggerPower);
                motorFour.setPower(0.0);
            } else {
                motorThree.setPower(0.0);
                motorFour.setPower(leftTriggerPower);
            }

            // ==============================================================
            // SERVO ARRAY <- X Y B A

            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentA = gamepad1.a;

            // select next or previous servo
            if (currentB && !lastB) {
                selectedServoIndex++;
                if (selectedServoIndex >= servos.length) {
                    selectedServoIndex = 0;
                }
            }
            if (currentX && !lastX) {
                selectedServoIndex--;
                if (selectedServoIndex < 0) {
                    selectedServoIndex = servos.length - 1;
                }
            }

            // selected servos current position
            double currentPos = servos[selectedServoIndex].getPosition();

            // y <- set position up
            // how much to adjust by
            double STEP_SIZE = 0.02;
            if (currentY && !lastY) {
                double newPos = Math.min(1.0, currentPos + STEP_SIZE);
                servos[selectedServoIndex].setPosition(newPos);
            }

            // a <- set position down
            if (currentA && !lastA) {
                double newPos = Math.max(0.0, currentPos - STEP_SIZE);
                servos[selectedServoIndex].setPosition(newPos);
            }

            // save button states
            lastB = currentB;
            lastX = currentX;
            lastY = currentY;
            lastA = currentA;

            // ==============================================================
            // TELEMETRY DISPLAY

            // selected motors
            telemetry.addData("RT Motor", isMotorOneSelected ? "1" : "2");
            telemetry.addData("LT Motor", isMotorThreeSelected ? "3" : "4");

            // sensors
            telemetry.addData("Touch", touchSensor.isPressed() ? "PRESSED" : "Released");
            telemetry.addData("Distance", "%.1f in", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("RGB", "%d, %d, %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());

            //selected servo name and what it does
            telemetry.addData("Selected Servo", servoNames[selectedServoIndex]);
            telemetry.addData("Servo Position", "%.2f", servos[selectedServoIndex].getPosition());

            telemetry.update();
        }
        cleanupRobot();
    }
}

