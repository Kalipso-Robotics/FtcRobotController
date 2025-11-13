package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private static Turret single_instance = null;

    private OpModeUtilities opModeUtilities;

    public DcMotor turretMotor;

    public static final double TICKS_PER_ROTATION = 384.5;
    public static final double GEAR_RATIO = 125.0/32.0;

    public static final double TICKS_PER_RADIAN = (TICKS_PER_ROTATION * GEAR_RATIO) / (2*Math.PI);
    public static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360.0;

    private Turret(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities, opModeUtilities.getHardwareMap(), this);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static synchronized Turret getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Turret(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities, opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(OpModeUtilities opModeUtilities, HardwareMap hardwareMap, Turret turret) {
        turret.opModeUtilities = opModeUtilities;
        turret.turretMotor = hardwareMap.dcMotor.get("turret");
        turret.turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getTurretMotor(){
        return turretMotor;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public double getCurrentAngleRad() {
        return turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;
    }

}
