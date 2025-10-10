package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.math.MathFunctions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {
    private static DriveTrain single_instance = null;

    private final OpModeUtilities opModeUtilities;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    //private SparkFunOTOS otos;
    private DcMotor backEncoder;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private GoBildaPinpointDriver odo;


    private double fLeftPower;
    private double fRightPower;
    private double bLeftPower;
    private double bRightPower;

    private DriveTrain(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);

        //sparkResetData(true, Math.toRadians(180));

        resetWheelOdom();

       // testMotorDeleteLater = opModeUtilities.getHardwareMap().dcMotor.get("testMotor");
    }

    public static synchronized DriveTrain getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new DriveTrain(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, DriveTrain driveTrain) {
        driveTrain.fLeft = hardwareMap.dcMotor.get("fLeft");
        driveTrain.fRight = hardwareMap.dcMotor.get("fRight");
        driveTrain.bLeft = hardwareMap.dcMotor.get("bLeft");
        driveTrain.bRight = hardwareMap.dcMotor.get("bRight");

        KLog.d("drive", "fleft port #:" + driveTrain.fLeft.getPortNumber());
        KLog.d("drive", "fRight port #:" + driveTrain.fRight.getPortNumber());
        KLog.d("drive", "bLeft port #:" + driveTrain.bLeft.getPortNumber());
        KLog.d("drive", "bRight port #:" + driveTrain.bRight.getPortNumber());

        driveTrain.fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveTrain.bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveTrain.bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveTrain.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveTrain.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveTrain.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveTrain.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveTrain.bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveTrain.odo = hardwareMap.get(GoBildaPinpointDriver.class,"goBildaOdometry");

        driveTrain.rightEncoder = driveTrain.bRight;
        driveTrain.leftEncoder = driveTrain.bLeft;
        driveTrain.backEncoder = driveTrain.fRight;
    }

    public void setFLeftPower(double power) {
        fLeft.setPower(power);
        fLeftPower = power;
    }

    public void setFRightPower(double power) {
        fRight.setPower(power);
        fRightPower = power;
    }

    public void setBLeftPower(double power) {
        bLeft.setPower(power);
        bLeftPower = power;
    }

    public void setBRightPower(double power) {
        bRight.setPower(power);
        bRightPower = power;
    }

    public void setPower (double fLeftPower, double fRightPower, double bLeftPower, double bRightPower){
        setFLeftPower(fLeftPower);
        setFRightPower(fRightPower);
        setBLeftPower(bLeftPower);
        setBRightPower(bRightPower);
        KLog.d("purepursaction_power", "power " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);
    }

    public void setPower(double power) {
        setFLeftPower(power);
        setFRightPower(power);
        setBLeftPower(power);
        setBRightPower(power);
    }
    public DcMotor getBackEncoder() {
        return backEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getfLeft() {
        return fLeft;
    }

    public double getfLeftTicks() {
        return fLeft.getCurrentPosition();
    }

    public DcMotor getfRight() {
        return fRight;
    }

    public DcMotor getbLeft() {
        return bLeft;
    }

    public DcMotor getbRight() {
        return bRight;
    }


    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }


    public void setPowerWithRangeClippingMinThreshold(double fLeftPower, double fRightPower, double bLeftPower,
                                                      double bRightPower, double minPowerThreshold) {
        double biggestPowerAbs = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        double scaleFactor = 1;

        //clipping power to -1 and 1 and min power is not 0.2 and -0.2
        if (biggestPowerAbs > 1) {
            scaleFactor = 1 / biggestPowerAbs;
        } else if (biggestPowerAbs < minPowerThreshold) {
            scaleFactor = (minPowerThreshold / biggestPowerAbs);
        }

        fLeftPower *= scaleFactor;
        fRightPower *= scaleFactor;
        bLeftPower *= scaleFactor;
        bRightPower *= scaleFactor;
        setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
    }

    public void resetWheelOdom() {
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getfLeftPower() {
        return fLeft.getPower();
    }

    public double getfRightPower() {
        return fRight.getPower();
    }

    public double getbLeftPower() {
        return bLeft.getPower();
    }

    public double getbRightPower() {
        return bRight.getPower();
    }

}
