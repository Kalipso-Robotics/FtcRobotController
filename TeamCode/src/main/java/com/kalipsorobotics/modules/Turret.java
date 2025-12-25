package com.kalipsorobotics.modules;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private static Turret single_instance = null;

    private OpModeUtilities opModeUtilities;

    public KMotor turretMotor;

    public static final double TICKS_PER_ROTATION = 384.5;
    public static final double BIG_TO_SMALL_PULLEY = 125.0/32.0;

    public static final double TICKS_PER_RADIAN = (TICKS_PER_ROTATION * BIG_TO_SMALL_PULLEY) / (2 * Math.PI);
    public static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * BIG_TO_SMALL_PULLEY) / 360.0;

    private Turret(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities, opModeUtilities.getHardwareMap(), this);
        // Reset encoder then set back to RUN_USING_ENCODER
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        DcMotor turretDcMotor = opModeUtilities.getHardwareMap().dcMotor.get("turret");
        turretDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.turretMotor = new KMotor(turretDcMotor, TurretConfig.kP, TurretConfig.kI, TurretConfig.kD, TurretConfig.kF, TurretConfig.kS);
    }

    public KMotor getTurretMotor(){
        return turretMotor;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public double getCurrentAngleRad() {
        return turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;
    }

    public double getCurrentVelocity() {
        return turretMotor.getMotor().getVelocity() / TICKS_PER_DEGREE;
    }
    public void stop() {
        KLog.d("TurretPower", "Current Power before Stopping: " + turretMotor.getPower());
        turretMotor.stop();
        KLog.d("TurretPower", "Power after STOP: " + turretMotor.getPower());

    }

}
