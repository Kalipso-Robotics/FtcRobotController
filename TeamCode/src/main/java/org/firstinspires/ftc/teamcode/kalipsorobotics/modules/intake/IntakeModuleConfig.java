package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * One intake side's settings (instances: IntakeConfig.SIDE_A / SIDE_B). Wiring is final and read once at init;
 * the numbers are public non-final and read every call, so Dashboard edits apply live.
 */
public class IntakeModuleConfig {
    public final String motorName;
    public final String pivotName;
    public final DcMotorSimple.Direction direction; // + power = intake in; fix spin here, not with a minus sign in callers

    public double pivotDownPos; // measured with IntakeTest
    public double pivotUpPos;
    public double initPivotPos;
    public double kF; // feed closed loop, power per RPS = 1 / free RPS at full power; read "kF est" in IntakeTest
    public double kP; // power per RPS of error; raise until the loaded dip shrinks without hunting

    public IntakeModuleConfig(String motorName, String pivotName, DcMotorSimple.Direction direction,
                              double pivotDownPos, double pivotUpPos, double initPivotPos, double kF, double kP) {
        this.motorName = motorName;
        this.pivotName = pivotName;
        this.direction = direction;
        this.pivotDownPos = pivotDownPos;
        this.pivotUpPos = pivotUpPos;
        this.initPivotPos = initPivotPos;
        this.kF = kF;
        this.kP = kP;
    }
}
