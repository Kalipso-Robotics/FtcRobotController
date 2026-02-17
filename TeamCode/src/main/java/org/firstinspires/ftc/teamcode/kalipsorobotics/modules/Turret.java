package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private static Turret single_instance = null;

    private OpModeUtilities opModeUtilities;

    public KMotor turretMotor;


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
        turret.turretMotor.setTicksOffset(TurretConfig.TICKS_INIT_OFFSET);
    }

    public KMotor getTurretMotor(){
        return turretMotor;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public double getCurrentAngleRad() {
        return turretMotor.getCurrentPosition() / TurretConfig.TICKS_PER_RADIAN;
    }

    public double getCurrentVelocity() {
        return turretMotor.getMotor().getVelocity() / TurretConfig.TICKS_PER_DEGREE;
    }
    public void stop() {
        turretMotor.setPower(0);
    }

}
