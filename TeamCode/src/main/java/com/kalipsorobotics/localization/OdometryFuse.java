package com.kalipsorobotics.localization;



import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Point;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.kalipsorobotics.utilities.OpModeUtilities;


public class OdometryFuse {
    OpModeUtilities opModeUtilities;
    private final SparkFunOTOS myOtos;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;
    private final DcMotor backEncoder;

    public OdometryFuse(SparkFunOTOS myOtos, DcMotor rightEncoder, DcMotor leftEncoder, DcMotor backEncoder) {
        this.myOtos = myOtos;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.backEncoder = backEncoder;
    }
    public Point WheelUpdateData() {
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        return(new Point(backEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition()));
    }
    public Point SparkUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(pos.x, pos.y));
    }
    public Point AverageUpdateData() {
        SparkFunOTOS.Pose2D SparkFunOTOS;
        double TICKSTOINCH = 40 / -13510.0 * (40.0 / 40.3612);
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return(new Point(((rightEncoder.getCurrentPosition() * TICKSTOINCH) + pos.x) / 2, ((backEncoder.getCurrentPosition() * TICKSTOINCH) + pos.y) / 2));
    }

    public Point Filter(Point sparkPoint, Point wheelPoint) {
        int diffenceDebug = 2;
        if ((sparkPoint.getX() - wheelPoint.getX() < diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < diffenceDebug) && (sparkPoint.getX() - wheelPoint.getX() > -diffenceDebug) && (sparkPoint.getY() - wheelPoint.getY() < -diffenceDebug)) { return(WheelUpdateData()); }
        else { return(SparkUpdateData()); }
    }

    public Point CollectData() {
        Point point = Filter(SparkUpdateData(), WheelUpdateData());
        return(point);
    }

    //configure SPARK FUN Otos
    @SuppressLint("DefaultLocale")
    public String configureOtos(SparkFunOTOS myOtos) {

        myOtos.setLinearUnit(DistanceUnit.INCH);

        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);



        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

            // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        return("OTOS configured! \n Hardware version: " + hwVersion.major + hwVersion.minor + "\n" +
                "Firmware Version: " + fwVersion.major + fwVersion.minor);
        }
//    public void runOpMode() throws InterruptedException {
//        DcMotor leftFront = hardwareMap.dcMotor.get("fLeft");
//        DcMotor rightFront = hardwareMap.dcMotor.get("fRight");
//        DcMotor leftBack = hardwareMap.dcMotor.get("bLeft");
//        DcMotor rightBack = hardwareMap.dcMotor.get("bRight");
//
//        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
//        configureOtos();
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //Inches per tick for wheel odometry
//        double INCHES_PER_TICK = 40 / -13510.0 * (40.0 / 40.3612);
//
//        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
//            double forward = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            double strafe = gamepad1.left_stick_x;
//            leftFront.setPower(forward + turn + strafe);
//            rightFront.setPower(forward - turn - strafe);
//            leftBack.setPower(forward + turn - strafe);
//            rightBack.setPower(forward - turn + strafe);
//
//            // Reset the tracking when needed
//            if (gamepad1.y) {
//                myOtos.resetTracking();
//            }
//
//            // Re-calibrate the IMU (for fixing issues ig)
//            if (gamepad1.x) {
//                myOtos.calibrateImu();
//            }
//            //Info things
//            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
//            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
//            telemetry.addLine();
//
//            //sprkfun update pos
//            telemetry.addData("X coordinate", pos.x);
//            telemetry.addData("Y coordinate", pos.y);
//            telemetry.addData("Heading angle", pos.h);
//
//            telemetry.addLine();
//
//            telemetry.addData("Wheel Encoder pos x", leftBack.getCurrentPosition() * INCHES_PER_TICK);
//            telemetry.addData("Wheel Encoder pos y", leftFront.getCurrentPosition() * INCHES_PER_TICK);
//            //telemetry.addData("average x", "" + ((leftFront.getCurrentPosition() + pos.x) / 2));
//            //telemetry.addData("average y", "" + ((leftBack.getCurrentPosition() + pos.y) / 2));
//            telemetry.update();
}

    /*
    NOTE!!!!!!!!!!

    wheel encoder more acurate moving forward
    otos prob more accurate for purpursuit
    otos VERY accurate for heading

    NOTE!!!!!!!!!!
     */




