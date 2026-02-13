package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.CalculateTickPer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {

    private static Outtake single_instance = null;

    //TODO MAKE PRIVATE
    private final OpModeUtilities opModeUtilities;
    public DcMotor linearSlide1;

    private Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        KLog.d("Outtake_LS", "init Outtake");

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);

        resetEncoders();
    }

    public static synchronized Outtake getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Outtake(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, Outtake outtake) {
        outtake.linearSlide1 = hardwareMap.dcMotor.get("linearSlide1");

        outtake.linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void resetEncoders() {
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {

    }

    public double getCurrentPosMm() {
        return CalculateTickPer.ticksToMmLS(getLinearSlide1().getCurrentPosition());
    }

    public DcMotor getLinearSlide1() {
        return linearSlide1;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}